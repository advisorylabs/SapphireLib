#include "sapphirelib/telemetry/logger.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <iterator>
#include <limits>
#include <mutex>
#include <string>
#include <string_view>

#include "pros/error.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/version.h"
#include "sapphirelib/telemetry/csv_format.hpp"
#include "sapphirelib/telemetry/file_naming.hpp"
#include "sapphirelib/util/clock.hpp"
#include "sapphirelib/util/log.hpp"
#include "sapphirelib/version.hpp"

// when the robot program was linked. PROS generates this at every link (libpros has a weak
// default), unlike __DATE__/__TIME__, which only change when this file is recompiled
extern "C" const char* const _PROS_COMPILE_TIMESTAMP;

namespace sapphirelib::telemetry {

namespace {

// ring sizes for the two event channels. Phases change a few times a match; user events come
// in bursts, so that ring gets more room
constexpr std::size_t kSystemEventsCapacity = 32;
constexpr std::size_t kUserEventsCapacity = 128;

// the channel handed back for a failed registration. No columns, never announced or drained
constexpr std::size_t kNullChannelCapacity = 8;

// smallest staging buffer: the header and a few of the longest #chan lines must fit
constexpr std::size_t kMinStagingBytes = 4096;

// list_files() buffer, about 600 log names. Only held while opening a file
constexpr std::size_t kListingBytes = 8192;

// PROS refuses paths of 128 characters or more
constexpr std::size_t kMaxPathBytes = 128;

// rows one writer pass drains at most
constexpr std::size_t kMaxRowsPerPass = 4096;

// how often the writer checks the card is still there, and retries with no card
constexpr std::uint32_t kCardCheckMs = 1000;

// wait after finding a card but failing to create a file (exFAT, full, write-protected). PROS
// leaks a little on every failed open, so don't retry in a tight loop
constexpr std::uint32_t kOpenFailedBackoffMs = 5000;

// names tried past the listing's highest before giving up
constexpr int kMaxExistenceChecks = 50;

// SD faults after which the writer stops reopening files. Each fault on a pulled card abandons
// a FILE, so this bounds the leak
constexpr std::uint32_t kMaxFaults = 8;

// the card's root, the fallback for a missing folder
constexpr const char* kRootDirectory = "/usd";

// the Logger the free telemetry::event() logs to: the last one started
std::atomic<Logger*> activeLogger{nullptr};

static_assert(std::atomic<Logger*>::is_always_lock_free,
              "telemetry::event() reads the active Logger with one atomic load");

// the phase event's message for a competition status, e.g. "autonomous,comp=1,field=1"
void phaseText(std::uint8_t status, char* out, std::size_t size) {
    const char* mode = (status & COMPETITION_DISABLED) != 0     ? "disabled"
                       : (status & COMPETITION_AUTONOMOUS) != 0 ? "autonomous"
                                                                : "opcontrol";
    std::snprintf(out, size, "%s,comp=%d,field=%d", mode,
                  (status & COMPETITION_CONNECTED) != 0 ? 1 : 0,
                  (status & COMPETITION_SYSTEM) != 0 ? 1 : 0);
}

} // namespace

// registration

Logger::Logger(LoggerConfig config)
    : config_(config),
      policy_(config.recordAtStart, config.recordUnderCompetition, config.competitionStopDelayMs) {
    if (config_.directory == nullptr) config_.directory = LoggerConfig{}.directory;
    if (config_.robotName == nullptr) config_.robotName = "";
    // a zero period would make that task spin
    config_.samplePeriodMs = std::max<std::uint32_t>(config_.samplePeriodMs, 1);
    config_.writerPeriodMs = std::max<std::uint32_t>(config_.writerPeriodMs, 1);
    config_.stagingBytes = std::max(config_.stagingBytes, kMinStagingBytes);

    nullChannel_ = std::make_unique<Channel>(
        ChannelSchema{.id = 0xFFFF, .name = "_", .kind = ChannelKind::samples},
        kNullChannelCapacity);

    // ids 0 and 1 are always the event channels
    channels_[0] = std::make_unique<Channel>(
        ChannelSchema{.id = 0, .name = "sys", .kind = ChannelKind::events, .decimals = 0},
        kSystemEventsCapacity, &fileEpoch_, &recording_);
    channels_[1] = std::make_unique<Channel>(
        ChannelSchema{.id = 1, .name = "events", .kind = ChannelKind::events, .decimals = 0},
        kUserEventsCapacity, &fileEpoch_, &recording_);
    systemEvents_ = channels_[0].get();
    userEvents_ = channels_[1].get();
    channelCount_.store(2, std::memory_order_release);
}

Logger::~Logger() {
    Logger* self = this;
    activeLogger.compare_exchange_strong(self, nullptr, std::memory_order_acq_rel);
    for (const auto& [observed, probe] : probes_) {
        if (observed->observer() == probe.get()) observed->setObserver(nullptr);
    }
    if (sampler_) sampler_->remove();
    if (writer_) writer_->remove();
}

Channel& Logger::pid(const char* name, PID& pid, ChannelOptions options) {
    std::lock_guard<pros::Mutex> lock(registrationMutex_);
    Channel& channel =
        addChannelLocked(name, ChannelKind::pid, kPidColumns, std::size(kPidColumns), options);
    if (isNullChannel(channel)) return channel;

    if (pid.observer() != nullptr) {
        SAPPHIRELIB_LOG_WARN("telemetry", "pid channel '%s' replaces that PID's existing observer",
                             channel.schema().name.c_str());
    }
    auto probe = std::make_unique<PidProbe>(channel, pid);
    pid.setObserver(probe.get());
    // kept so the destructor can detach it
    probes_.emplace_back(&pid, std::move(probe));
    return channel;
}

Channel& Logger::channel(const char* name, std::initializer_list<const char*> columns,
                         ChannelOptions options) {
    std::lock_guard<pros::Mutex> lock(registrationMutex_);
    return addChannelLocked(name, ChannelKind::samples, columns.begin(), columns.size(), options);
}

Channel& Logger::poll(const char* name, std::initializer_list<const char*> columns,
                      std::uint32_t periodMs, std::function<void(double* values)> read,
                      ChannelOptions options) {
    return pollColumns(name, columns.begin(), columns.size(), periodMs, std::move(read), options);
}

Channel& Logger::pollColumns(const char* name, const char* const* columns, std::size_t columnCount,
                             std::uint32_t periodMs, std::function<void(double* values)> read,
                             ChannelOptions options) {
    std::lock_guard<pros::Mutex> lock(registrationMutex_);
    const std::size_t count = polledCount_.load(std::memory_order_relaxed);
    if (count >= kMaxPolled || !read) {
        SAPPHIRELIB_LOG_WARN("telemetry", "can't poll '%s': %s; it won't be logged",
                             name != nullptr ? name : "",
                             !read ? "no read function" : "every poll slot is taken");
        return *nullChannel_;
    }

    Channel& channel = addChannelLocked(name, ChannelKind::samples, columns, columnCount, options);
    if (isNullChannel(channel)) return channel;

    polled_[count] = std::make_unique<Polled>(
        Polled{.channel = &channel, .periodMs = periodMs, .read = std::move(read)});
    polledCount_.store(count + 1, std::memory_order_release);
    return channel;
}

Channel& Logger::pose(const odom::Odometry& odometry, const char* name, std::uint32_t periodMs) {
    return poll(name, {"x", "y", "heading"}, periodMs, [&odometry](double* values) {
        // getPose() takes the odometry's mutex briefly. Fine here, since neither task is ever
        // deleted while holding it
        const odom::Pose pose = odometry.getPose();
        values[0] = pose.xIn;
        values[1] = pose.yIn;
        values[2] = pose.headingDeg;
    });
}

Channel& Logger::motor(const char* name, std::int8_t port, std::uint32_t periodMs,
                       ChannelOptions options) {
    return pollColumns(
        name, kMotorColumns, kMotorColumnCount, periodMs,
        [port](double* values) {
            // quick reads of what the brain already has cached from the motor
            motorRow(MotorReadings{.voltageMv = pros::c::motor_get_voltage(port),
                                   .currentMa = pros::c::motor_get_current_draw(port),
                                   .temperatureC = pros::c::motor_get_temperature(port),
                                   .velocityRpm = pros::c::motor_get_actual_velocity(port),
                                   .efficiencyPct = pros::c::motor_get_efficiency(port),
                                   .faults = pros::c::motor_get_faults(port)},
                     values);
        },
        options);
}

Channel& Logger::addChannelLocked(const char* name, ChannelKind kind, const char* const* columns,
                                  std::size_t columnCount, ChannelOptions options) {
    std::string clean = sanitizeName(name);
    const std::size_t count = channelCount_.load(std::memory_order_relaxed);

    // channels are matched by name across files, so a duplicate would merge two streams
    for (std::size_t i = 0; i < count; ++i) {
        if (channels_[i]->schema().name == clean) {
            SAPPHIRELIB_LOG_WARN("telemetry",
                                 "channel name '%s' is taken (\"sys\" and \"events\" are "
                                 "reserved); that channel won't be logged",
                                 clean.c_str());
            return *nullChannel_;
        }
    }
    if (count >= kMaxChannels) {
        SAPPHIRELIB_LOG_WARN("telemetry", "no room for channel '%s' (a Logger holds %u)",
                             clean.c_str(), static_cast<unsigned>(kMaxChannels));
        return *nullChannel_;
    }
    if (kind == ChannelKind::samples && (columnCount == 0 || columnCount > kMaxColumns)) {
        SAPPHIRELIB_LOG_WARN("telemetry", "channel '%s' has %u columns; %s", clean.c_str(),
                             static_cast<unsigned>(columnCount),
                             columnCount == 0 ? "it can't record anything"
                                              : "only the first 13 are logged");
        columnCount = std::min(columnCount, kMaxColumns);
    }

    ChannelSchema schema{.id = static_cast<std::uint16_t>(count),
                         .name = std::move(clean),
                         .kind = kind,
                         .decimals = std::clamp(options.decimals, 0, 6)};
    schema.columns.reserve(columnCount);
    for (std::size_t i = 0; i < columnCount; ++i) {
        schema.columns.push_back(sanitizeName(columns[i]));
    }

    channels_[count] =
        std::make_unique<Channel>(std::move(schema), options.capacity, &fileEpoch_, &recording_);
    // publish last: the tasks only ever look at entries below the count they loaded
    channelCount_.store(count + 1, std::memory_order_release);
    return *channels_[count];
}

bool Logger::isNullChannel(const Channel& channel) const { return &channel == nullChannel_.get(); }

// producers

bool Logger::event(const char* tag, const char* format, ...) {
    std::va_list args;
    va_start(args, format);
    const bool logged = vevent(tag, format, args);
    va_end(args);
    return logged;
}

bool Logger::vevent(const char* tag, const char* format, std::va_list args) {
    // on the caller's stack, so formatting never touches the heap
    char message[kMaxEventMessageChars + 1];
    message[0] = '\0';
    if (format != nullptr && std::vsnprintf(message, sizeof(message), format, args) < 0) {
        message[0] = '\0';
    }
    return userEvents_->recordEvent(tag, message);
}

bool event(const char* tag, const char* format, ...) {
    Logger* logger = activeLogger.load(std::memory_order_acquire);
    if (logger == nullptr) return false;
    std::va_list args;
    va_start(args, format);
    const bool logged = logger->vevent(tag, format, args);
    va_end(args);
    return logged;
}

// control

void Logger::addMeta(const char* key, const char* value) {
    std::lock_guard<pros::Mutex> lock(registrationMutex_);
    if (started_) {
        SAPPHIRELIB_LOG_WARN("telemetry", "addMeta(\"%s\") after start() is ignored",
                             key != nullptr ? key : "");
        return;
    }
    meta_.emplace_back(key != nullptr ? key : "", value != nullptr ? value : "");
}

void Logger::startRecording() {
    request_.store(RecordingRequest::start, std::memory_order_release);
}

void Logger::stopRecording() { request_.store(RecordingRequest::stop, std::memory_order_release); }

bool Logger::recording() const { return recording_.load(std::memory_order_acquire); }

bool Logger::start() {
    std::lock_guard<pros::Mutex> lock(registrationMutex_);
    if (started_) return false;
    started_ = true;

    // allocate everything up front, so neither task allocates later
    staging_ = std::make_unique<char[]>(config_.stagingBytes);
    stdioBuffer_ = std::make_unique<char[]>(config_.stagingBytes);
    // the sampler's first tick decides whether to record (LoggerConfig::recordAtStart)
    state_.store(LoggerState::idle, std::memory_order_release);
    activeLogger.store(this, std::memory_order_release);

    // below every control loop, so polling never preempts one, and above the writer, whose SD
    // writes can take a couple hundred milliseconds
    sampler_ = std::make_unique<pros::Task>([this] { samplerLoop(); }, TASK_PRIORITY_DEFAULT - 1,
                                            TASK_STACK_DEPTH_DEFAULT, "SL Sampler");
    // as low as a task can usefully go. The ProducerGate reclaim needs every producer above this
    writer_ = std::make_unique<pros::Task>([this] { writerLoop(); }, TASK_PRIORITY_MIN + 1,
                                           TASK_STACK_DEPTH_DEFAULT, "SL Writer");
    return true;
}

void Logger::requestFlush() { flushRequested_.store(true, std::memory_order_release); }

LoggerStatus Logger::status() const {
    return LoggerStatus{.state = state_.load(std::memory_order_acquire),
                        .fileIndex = fileIndex_.load(std::memory_order_relaxed),
                        .inRootFolder = inRootFolder_.load(std::memory_order_relaxed),
                        .bytesWritten = bytesWritten_.load(std::memory_order_relaxed),
                        .droppedRows = droppedRows(),
                        .slowestWriteUs = slowestWriteUs_.load(std::memory_order_relaxed),
                        .recording = recording_.load(std::memory_order_acquire),
                        .reason = reason_.load(std::memory_order_relaxed),
                        .cardInserted = cardInserted_.load(std::memory_order_relaxed)};
}

std::uint32_t Logger::droppedRows() const {
    std::uint32_t dropped = 0;
    const std::size_t count = channelCount_.load(std::memory_order_acquire);
    for (std::size_t i = 0; i < count; ++i) {
        dropped += channels_[i]->droppedFull() + channels_[i]->droppedContended();
    }
    return dropped;
}

// sampler task

void Logger::samplerLoop() {
    std::uint32_t wake = pros::millis();
    while (true) {
        samplerTick();
        pros::Task::delay_until(&wake, config_.samplePeriodMs);
    }
}

void Logger::samplerTick() {
    const std::uint64_t tickStartUs = sapphirelib::micros();
    const std::uint8_t status = pros::competition::get_status();
    const std::uint32_t nowMs = sapphirelib::millis();

    // start or stop first, so a recording that starts on this tick logs this tick's phase and
    // polls too
    const RecordingChange change =
        policy_.update((status & COMPETITION_CONNECTED) != 0,
                       request_.exchange(RecordingRequest::none, std::memory_order_acq_rel), nowMs);
    if (change.stop) {
        stopReason_.store(change.reason, std::memory_order_relaxed);
        reason_.store(RecordingReason::none, std::memory_order_relaxed);
        // released after the reason, so a writer that sees the gate shut also sees why
        recording_.store(false, std::memory_order_release);
        flushRequested_.store(true, std::memory_order_release);
    }
    if (change.start) {
        reason_.store(change.reason, std::memory_order_relaxed);
        // a new session before the gate opens, so the writer opens a new file for it even if the
        // last recording's file hasn't been closed yet
        session_.fetch_add(1, std::memory_order_release);
        recording_.store(true, std::memory_order_release);
    }
    const bool recording = recording_.load(std::memory_order_relaxed);

    if (status != lastStatus_) {
        char text[48];
        phaseText(status, text, sizeof(text));
        // only marked seen once the row went in, so a dropped one is retried next tick. Between
        // recordings there's no file for it; the next file logs the phase when it opens
        if (!recording || systemEvents_->recordEvent("phase", text)) lastStatus_ = status;
        // raised after the row is committed, so the pass that acts on it also writes the row. The
        // end of a match is on the card moments after it's disabled
        flushRequested_.store(true, std::memory_order_release);
    }

    const bool disabledUnderControl =
        (status & COMPETITION_DISABLED) != 0 && (status & COMPETITION_CONNECTED) != 0;
    if (recording && !(disabledUnderControl && !config_.pollWhileDisabled)) pollSources(nowMs);

    const std::uint64_t tickUs = sapphirelib::micros() - tickStartUs;
    samplerBusyUs_.fetch_add(static_cast<std::uint32_t>(std::min<std::uint64_t>(tickUs, 1000000)),
                             std::memory_order_relaxed);
}

void Logger::pollSources(std::uint32_t nowMs) {
    const std::size_t count = polledCount_.load(std::memory_order_acquire);
    for (std::size_t i = 0; i < count; ++i) {
        Polled& source = *polled_[i];
        // half a period of slack, so a tick that runs a millisecond early doesn't skip a source
        if (source.polledOnce &&
            nowMs - source.lastMs + config_.samplePeriodMs / 2 < source.periodMs) {
            continue;
        }
        source.polledOnce = true;
        source.lastMs = nowMs;

        double values[kMaxColumns];
        std::fill(std::begin(values), std::end(values), std::numeric_limits<double>::quiet_NaN());
        source.read(values);
        source.channel->record(values, source.channel->schema().columns.size());
    }
}

// writer task
//
// rows are formatted into staging_ and handed to the card in one fwrite() + fflush() per chunk,
// through a FILE whose stdio buffer is exactly as big as staging_. So each chunk is one write
// (and one sync), instead of newlib splitting it into 1KB writes

template <typename Format> bool Logger::stage(Format format, bool countsAsRow) {
    // two tries: a line that doesn't fit is formatted again after writing the buffer out, so it's
    // never split across two writes
    for (int attempt = 0; attempt < 2 && file_ != nullptr; ++attempt) {
        const std::size_t length = format(staging_.get() + used_, config_.stagingBytes - used_);
        if (length > 0) {
            used_ += length;
            if (countsAsRow) ++fileRows_;
            return true;
        }
        // didn't fit an empty buffer (can't happen at kMinStagingBytes), so skip it. Or the write
        // failed, and we've faulted
        if (used_ == 0 || !writeStaged()) return false;
    }
    return false;
}

void Logger::writerLoop() {
    std::uint32_t wake = pros::millis();
    while (true) {
        writerPass();
        pros::Task::delay_until(&wake, config_.writerPeriodMs);
    }
}

void Logger::writerPass() {
    const std::uint64_t passStartUs = sapphirelib::micros();
    passWriteUs_ = 0;
    const std::size_t count = channelCount_.load(std::memory_order_acquire);
    // taken before draining: the sampler commits a phase row and then raises this, so the drain
    // below includes that row
    const bool flushNow = flushRequested_.exchange(false, std::memory_order_acq_rel);
    // acquire pairs with the sampler's release: seeing the gate shut means seeing the stop's
    // reason, and seeing it open means seeing the session it opened for
    const bool wanted = recording_.load(std::memory_order_acquire);
    const std::uint32_t session = session_.load(std::memory_order_acquire);

    // the recording stopped, or stopped and a new one started since this file opened
    if (file_ != nullptr && (!wanted || session != fileSession_)) closeFile(count);

    manageFile(sapphirelib::millis(), count, wanted);

    for (std::size_t i = 0; i < count; ++i) {
        if (channels_[i]->gate().breakIfStuck()) ++breaks_;
    }

    if (file_ != nullptr) announceChannels(count);
    drain(count);

    if (file_ != nullptr && sapphirelib::millis() - lastHealthMs_ >= config_.healthPeriodMs) {
        writeHealth(count);
    }
    if (file_ != nullptr && used_ > 0 &&
        (flushNow || sapphirelib::millis() - lastWriteMs_ >= config_.flushPeriodMs)) {
        writeStaged();
    }

    // fmt_us: this pass's time minus its SD writes (wmax_us/wavg_us already cover those)
    const std::uint64_t passUs = sapphirelib::micros() - passStartUs;
    const std::uint64_t busyUs = passUs > passWriteUs_ ? passUs - passWriteUs_ : 0;
    writerBusyUs_ += static_cast<std::uint32_t>(std::min<std::uint64_t>(busyUs, 1000000));
}

void Logger::closeFile(std::size_t channelCount) {
    // everything recorded before the stop belongs in this file. The gate is shut, so the rings
    // only empty from here
    for (int pass = 0; pass < 8 && file_ != nullptr; ++pass) {
        announceChannels(channelCount);
        if (!drain(channelCount)) break;
    }
    if (file_ != nullptr) {
        char message[32];
        std::snprintf(message, sizeof(message), "stop,%s",
                      recordingReasonName(stopReason_.load(std::memory_order_relaxed)));
        const std::uint64_t nowUs = sapphirelib::micros();
        stage([&](char* out,
                  std::size_t size) { return formatEvent(out, size, nowUs, "rec", message); },
              true);
        writeStaged();
    }
    // writeStaged() closes the file itself if the card failed
    if (file_ != nullptr) {
        std::fclose(file_);
        file_ = nullptr;
        SAPPHIRELIB_LOG_INFO("telemetry", "recording stopped; %s closed", fileName_);
    }
    used_ = 0;
    fileSession_ = 0;
}

void Logger::manageFile(std::uint32_t nowMs, std::size_t channelCount, bool wanted) {
    if (file_ == nullptr) {
        if (!wanted) {
            // between recordings: nothing to open, and the next start tries right away. The
            // card's presence is still checked, for the Home page
            state_.store(LoggerState::idle, std::memory_order_release);
            openBackoffMs_ = 0;
            // a fault in the last recording is behind us: the next file starts a recording,
            // rather than continuing one
            reopenPending_ = false;
            if (nowMs - lastCardCheckMs_ >= kCardCheckMs) {
                lastCardCheckMs_ = nowMs;
                cardInserted_.store(pros::usd::is_installed() != 0, std::memory_order_relaxed);
            }
            return;
        }
        if (faults_ >= kMaxFaults) return; // gave up, see fault()
        if (openBackoffMs_ != 0 && nowMs - lastOpenAttemptMs_ < openBackoffMs_) return;
        lastOpenAttemptMs_ = nowMs;
        tryOpen(channelCount);
        return;
    }
    // a pulled card doesn't always fail the next write cleanly, so check for it directly
    if (nowMs - lastCardCheckMs_ >= kCardCheckMs) {
        lastCardCheckMs_ = nowMs;
        const bool inserted = pros::usd::is_installed() != 0;
        cardInserted_.store(inserted, std::memory_order_relaxed);
        if (!inserted) fault("card removed");
    }
}

void Logger::tryOpen(std::size_t channelCount) {
    // a fault stays visible until a file opens again
    auto notLogging = [this] {
        if (state_.load(std::memory_order_relaxed) != LoggerState::faulted) {
            state_.store(LoggerState::waitingForCard, std::memory_order_release);
        }
    };

    const bool inserted = pros::usd::is_installed() != 0;
    cardInserted_.store(inserted, std::memory_order_relaxed);
    if (!inserted) {
        notLogging();
        openBackoffMs_ = kCardCheckMs;
        return;
    }

    const OpenResult inFolder = openIn(config_.directory, channelCount, nullptr);
    if (inFolder == OpenResult::opened) return;

    // the folder is missing, or fopen() failed in it (which on PROS usually means the same thing).
    // Fall back to the card's root, unless that's where we were
    if (listingPath(config_.directory) != "/") {
        char note[kMaxPathBytes + 16];
        std::snprintf(note, sizeof(note), "%s,%s",
                      inFolder == OpenResult::folderMissing ? "dir_missing" : "open_failed",
                      config_.directory);
        if (inFolder == OpenResult::folderMissing) {
            SAPPHIRELIB_LOG_WARN("telemetry",
                                 "%s isn't on the SD card (the V5 can't create folders: make it on "
                                 "a computer); logging to the card's root instead",
                                 config_.directory);
        } else {
            SAPPHIRELIB_LOG_WARN("telemetry",
                                 "can't create a file in %s; logging to the card's root instead",
                                 config_.directory);
        }
        if (openIn(kRootDirectory, channelCount, note) == OpenResult::opened) return;
    }

    notLogging();
    openBackoffMs_ = kOpenFailedBackoffMs;
    SAPPHIRELIB_LOG_ERROR("telemetry",
                          "can't create a log file on the SD card (FAT32 only; is it full or "
                          "locked?); retrying in %lu s",
                          static_cast<unsigned long>(kOpenFailedBackoffMs / 1000));
}

Logger::OpenResult Logger::openIn(const char* directory, std::size_t channelCount,
                                  const char* fallbackNote) {
    // pick a name: the next number after the highest already there
    std::int32_t index = 1;
    {
        // list_files() wants the path without "/usd", NUL-terminated
        char listPath[kMaxPathBytes];
        const std::string_view view = listingPath(directory);
        if (view.size() >= sizeof(listPath)) return OpenResult::failed;
        listPath[view.copy(listPath, view.size())] = '\0';

        // zeroed and filled to one byte short, so it's always terminated
        auto listing = std::make_unique<char[]>(kListingBytes);
        errno = 0;
        if (pros::usd::list_files(listPath, listing.get(),
                                  static_cast<std::int32_t>(kListingBytes - 1)) == PROS_ERR) {
            if (errno == ENOENT && fallbackNote == nullptr) return OpenResult::folderMissing;
            // any other failure: carry on as if the folder were empty. The existence check below
            // still keeps an old log from being overwritten
            listing[0] = '\0';
        }
        const std::int32_t highest =
            highestLogFileIndex(std::string_view(listing.get(), std::strlen(listing.get())));
        if (highest >= 0) index = highest + 1;
    }

    // don't trust the listing alone before opening with "w", which truncates. Skip each name that
    // turns out to exist
    char path[kMaxPathBytes];
    for (int checks = 0;; ++checks) {
        if (checks >= kMaxExistenceChecks || index > static_cast<std::int32_t>(kMaxLogIndex) ||
            !formatLogFilePath(path, sizeof(path), directory, static_cast<std::uint32_t>(index))) {
            SAPPHIRELIB_LOG_ERROR("telemetry",
                                  "no free log file name in %s (empty the folder now and then)",
                                  directory);
            return OpenResult::failed;
        }
        std::FILE* existing = std::fopen(path, "r");
        if (existing == nullptr) break;
        std::fclose(existing);
        ++index;
    }

    // "w" is the only mode that creates a file here; PROS rejects the read-write modes
    std::FILE* file = std::fopen(path, "w");
    if (file == nullptr) return OpenResult::failed;

    // after a fault the old buffer went with the abandoned FILE, so this file gets a fresh one
    if (!stdioBuffer_) stdioBuffer_ = std::make_unique<char[]>(config_.stagingBytes);
    if (std::setvbuf(file, stdioBuffer_.get(), _IOFBF, config_.stagingBytes) != 0) {
        SAPPHIRELIB_LOG_WARN("telemetry", "setvbuf failed; SD writes will be split up");
    }

    // the file is open: reset everything that's per file
    char previousName[sizeof(fileName_)];
    std::memcpy(previousName, fileName_, sizeof(previousName));
    formatLogFileName(fileName_, sizeof(fileName_), static_cast<std::uint32_t>(index));
    file_ = file;
    fileSession_ = session_.load(std::memory_order_acquire);
    used_ = 0;
    announced_ = 0;
    lastDrops_.fill({0, 0}); // so the first D rows restate any drops so far
    fileRows_ = 0;
    fileBytes_ = 0;
    fileWrites_ = 0;
    healthWriteMaxUs_ = 0;
    healthWriteSumUs_ = 0;
    healthWrites_ = 0;
    fileIndex_.store(index, std::memory_order_relaxed);
    inRootFolder_.store(fallbackNote != nullptr, std::memory_order_relaxed);
    bytesWritten_.store(0, std::memory_order_relaxed);
    // the H row's busy times count from here, not from before the recording started
    samplerBusyUs_.store(0, std::memory_order_relaxed);
    writerBusyUs_ = 0;
    // every PidProbe sees this and repeats its C and G rows, so each file stands on its own
    fileEpoch_.fetch_add(1, std::memory_order_release);

    // header, schemas, and the events that open every file
    const std::uint64_t openUs = sapphirelib::micros();
    const FileHeader header{.library = SAPPHIRELIB_VERSION,
                            .kernel = PROS_VERSION_STRING,
                            .build = _PROS_COMPILE_TIMESTAMP,
                            .robotName = config_.robotName,
                            .fileName = fileName_,
                            .directory = directory,
                            .openUs = openUs};
    stage([&](char* out, std::size_t size) { return formatHeader(out, size, header); }, false);
    // the program's own #meta lines (addMeta()), fixed since start()
    for (const auto& [key, value] : meta_) {
        stage([&](char* out,
                  std::size_t size) { return formatMeta(out, size, key.c_str(), value.c_str()); },
              false);
    }
    announceChannels(channelCount);

    auto stageEvent = [&](const char* tag, const char* message) {
        stage([&](char* out,
                  std::size_t size) { return formatEvent(out, size, openUs, tag, message); },
              true);
    };
    char message[kMaxPathBytes + 48];
    std::snprintf(message, sizeof(message), "open,%s", fileName_);
    stageEvent("file", message);
    phaseText(pros::competition::get_status(), message, sizeof(message));
    stageEvent("phase", message);
    // a file reopened after a fault continues its recording; any other starts one
    if (!reopenPending_) {
        std::snprintf(message, sizeof(message), "start,%s",
                      recordingReasonName(reason_.load(std::memory_order_relaxed)));
        stageEvent("rec", message);
    }
    if (fallbackNote != nullptr) stageEvent("sd", fallbackNote);
    if (reopenPending_) {
        std::snprintf(message, sizeof(message), "reopened,prev=%s,faults=%lu", previousName,
                      static_cast<unsigned long>(faults_));
        stageEvent("sd", message);
        reopenPending_ = false;
    }

    state_.store(LoggerState::logging, std::memory_order_release);
    const std::uint32_t nowMs = sapphirelib::millis();
    lastCardCheckMs_ = nowMs;
    lastHealthMs_ = nowMs;
    // write it right away, so even a run cut short leaves a readable file
    if (writeStaged()) SAPPHIRELIB_LOG_INFO("telemetry", "logging to %s", path);
    return OpenResult::opened;
}

void Logger::announceChannels(std::size_t channelCount) {
    for (; announced_ < channelCount; ++announced_) {
        const ChannelSchema& schema = channels_[announced_]->schema();
        const bool staged = stage(
            [&](char* out, std::size_t size) { return formatSchema(out, size, schema); }, false);
        if (!staged && file_ == nullptr) return; // faulted: the next file announces them all
    }
}

bool Logger::drain(std::size_t channelCount) {
    // merge the channels by t_us, one snapshot per pass; rows committed mid-pass wait for the next
    const Record* heads[kMaxChannels] = {};
    for (std::size_t i = 0; i < channelCount; ++i) heads[i] = channels_[i]->front();

    for (std::size_t rows = 0; rows < kMaxRowsPerPass; ++rows) {
        std::size_t next = channelCount;
        for (std::size_t i = 0; i < channelCount; ++i) {
            if (heads[i] != nullptr && (next == channelCount || heads[i]->tUs < heads[next]->tUs)) {
                next = i;
            }
        }
        if (next == channelCount) return false; // everything drained

        Channel& channel = *channels_[next];
        const Record& record = *heads[next];

        // a long event is its event Record plus continuations, committed together, so take them
        // together
        const Record* parts[kMaxEventRecords] = {&record};
        std::size_t taken = 1;
        if (record.kind == RecordKind::event) {
            while (taken <= record.count && taken < kMaxEventRecords) {
                const Record* part = channel.peek(taken);
                if (part == nullptr || part->kind != RecordKind::eventContinued) break;
                parts[taken++] = part;
            }
        }

        // a continuation with no event before it (only after a ring resync) is discarded
        if (record.kind != RecordKind::eventContinued) {
            bool staged = false;
            if (file_ != nullptr) {
                staged = record.kind == RecordKind::event
                             ? stage(
                                   [&](char* out, std::size_t size) {
                                       return formatEventRecords(out, size, parts, taken);
                                   },
                                   true)
                             : stage(
                                   [&](char* out, std::size_t size) {
                                       return formatRecord(out, size, channel.schema(), record);
                                   },
                                   true);
            }
            // recording with no file open (no card yet, or after a fault). Leftovers from
            // before a stop don't count: they weren't meant for a file
            if (!staged && file_ == nullptr && recording_.load(std::memory_order_relaxed)) {
                ++unlogged_;
            }
        }

        channel.pop(taken);
        heads[next] = channel.front();
    }
    return true; // hit the per-pass limit; more may be waiting
}

void Logger::writeHealth(std::size_t channelCount) {
    lastHealthMs_ = sapphirelib::millis();

    std::uint32_t drops = 0;
    std::uint32_t resyncs = 0;
    for (std::size_t i = 0; i < channelCount; ++i) {
        drops += channels_[i]->droppedFull() + channels_[i]->droppedContended();
        resyncs += channels_[i]->resyncs();
    }
    const WriterStats stats{
        .rows = fileRows_,
        .bytes = fileBytes_,
        .writes = fileWrites_,
        .writeMaxUs = healthWriteMaxUs_,
        .writeAvgUs =
            healthWrites_ != 0 ? static_cast<std::uint32_t>(healthWriteSumUs_ / healthWrites_) : 0,
        .drops = drops,
        .unlogged = unlogged_,
        .resyncs = resyncs,
        .breaks = breaks_,
        .faults = faults_,
        .samplerUs = samplerBusyUs_.exchange(0, std::memory_order_relaxed),
        .formatUs = writerBusyUs_,
    };
    const std::uint64_t nowUs = sapphirelib::micros();
    if (!stage([&](char* out, std::size_t size) { return formatHealth(out, size, nowUs, stats); },
               true)) {
        return;
    }
    healthWriteMaxUs_ = 0;
    healthWriteSumUs_ = 0;
    healthWrites_ = 0;
    writerBusyUs_ = 0;

    // D rows only for channels whose counts moved since the last ones in this file
    for (std::size_t i = 0; i < channelCount; ++i) {
        const std::pair<std::uint32_t, std::uint32_t> dropped{channels_[i]->droppedFull(),
                                                              channels_[i]->droppedContended()};
        if (dropped == lastDrops_[i]) continue;
        const auto id = static_cast<std::uint16_t>(i);
        if (!stage(
                [&](char* out, std::size_t size) {
                    return formatDrops(out, size, nowUs, id, dropped.first, dropped.second);
                },
                true)) {
            return;
        }
        lastDrops_[i] = dropped;
    }
}

bool Logger::writeStaged() {
    if (file_ == nullptr) return false;
    if (used_ == 0) return true;

    const std::uint64_t startUs = sapphirelib::micros();
    // fwrite() only copies into the same-size stdio buffer; fflush() is the one write and sync
    const bool ok =
        std::fwrite(staging_.get(), 1, used_, file_) == used_ && std::fflush(file_) == 0;
    const auto us = static_cast<std::uint32_t>(
        std::min<std::uint64_t>(sapphirelib::micros() - startUs, UINT32_MAX));
    lastWriteMs_ = sapphirelib::millis();

    ++fileWrites_;
    ++healthWrites_;
    healthWriteSumUs_ += us;
    passWriteUs_ += us;
    healthWriteMaxUs_ = std::max(healthWriteMaxUs_, us);
    if (us > slowestWriteUs_.load(std::memory_order_relaxed)) {
        slowestWriteUs_.store(us, std::memory_order_relaxed);
    }

    if (!ok) {
        fault("write failed");
        return false;
    }
    fileBytes_ += static_cast<std::uint32_t>(used_);
    bytesWritten_.store(fileBytes_, std::memory_order_relaxed);
    used_ = 0;
    return true;
}

void Logger::fault(const char* reason) {
    ++faults_;
    SAPPHIRELIB_LOG_ERROR("telemetry", "SD fault on %s: %s (fault %lu)%s", fileName_, reason,
                          static_cast<unsigned long>(faults_),
                          faults_ >= kMaxFaults ? "; no more log files this run" : "");

    if (file_ != nullptr) {
        if (pros::usd::is_installed() != 0) {
            std::fclose(file_);
        } else {
            // closing a file on a pulled card is undocumented, so abandon the FILE instead
            // (kMaxFaults bounds the leak). Its buffer goes with it; the next file gets a new one
            static_cast<void>(stdioBuffer_.release());
        }
        file_ = nullptr;
    }
    used_ = 0; // staged rows were for that file
    reopenPending_ = true;
    state_.store(LoggerState::faulted, std::memory_order_release);
    lastOpenAttemptMs_ = sapphirelib::millis();
    openBackoffMs_ = kCardCheckMs;
}

} // namespace sapphirelib::telemetry
