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

// When the robot program was linked: PROS generates this at every link
// (common.mk's _pros_ld_timestamp, compiled as C), and libpros carries a weak
// default. Better than this file's own __DATE__/__TIME__, which only change
// when logger.cpp itself is recompiled — a log should say which *program
// build* wrote it.
extern "C" const char* const _PROS_COMPILE_TIMESTAMP;

namespace sapphirelib::telemetry {

namespace {

/// Ring sizes for the two built-in event channels. Phases change a handful of
/// times a match; user events come in bursts (a motion's start and end are
/// 2-3 Records each), so that ring gets more room.
constexpr std::size_t kSystemEventsCapacity = 32;
constexpr std::size_t kUserEventsCapacity = 128;

/// The channel handed back for a registration that failed. It has no columns,
/// so record() on it does nothing, and it's never announced or drained.
constexpr std::size_t kNullChannelCapacity = 8;

/// Smallest staging buffer: the header plus a few of the longest `#chan`
/// lines must fit in an empty one.
constexpr std::size_t kMinStagingBytes = 4096;

/// list_files() buffer — about 600 log names. Held only while opening a file.
constexpr std::size_t kListingBytes = 8192;

/// PROS refuses paths of 128 characters or more (vfs.c's MAX_FILELEN).
constexpr std::size_t kMaxPathBytes = 128;

/// Rows one writer pass drains before yielding to its own schedule again.
constexpr std::size_t kMaxRowsPerPass = 4096;

/// How often the writer checks the card is still there while logging, and
/// waits between attempts while there's no card.
constexpr std::uint32_t kCardCheckMs = 1000;

/// Wait after an attempt that found a card but couldn't create a file —
/// exFAT, full, write-protected. Retrying fopen() in a tight loop only piles
/// up the small allocation PROS leaks on each failed open.
constexpr std::uint32_t kOpenFailedBackoffMs = 5000;

/// Names tried past the listing's highest before giving up on an open.
constexpr int kMaxExistenceChecks = 50;

/// SD faults after which the writer stops reopening files for the rest of the
/// run. Each fault on a pulled card abandons a FILE (see fault()), so this is
/// what keeps those leaks bounded.
constexpr std::uint32_t kMaxFaults = 8;

/// fopen() path of the card's root folder, the fallback for a missing one.
constexpr const char* kRootDirectory = "/usd";

/// The Logger the free telemetry::event() logs to: the last one started.
std::atomic<Logger*> activeLogger{nullptr};

static_assert(std::atomic<Logger*>::is_always_lock_free,
              "telemetry::event() reads the active Logger with one atomic load");

/// The `phase` event's message for a competition::get_status() value, e.g.
/// "autonomous,comp=1,field=1" (docs/TELEMETRY_FORMAT.md).
void phaseText(std::uint8_t status, char* out, std::size_t size) {
    const char* mode = (status & COMPETITION_DISABLED) != 0     ? "disabled"
                       : (status & COMPETITION_AUTONOMOUS) != 0 ? "autonomous"
                                                                : "opcontrol";
    std::snprintf(out, size, "%s,comp=%d,field=%d", mode,
                  (status & COMPETITION_CONNECTED) != 0 ? 1 : 0,
                  (status & COMPETITION_SYSTEM) != 0 ? 1 : 0);
}

} // namespace

// --- Registration ------------------------------------------------------------

Logger::Logger(LoggerConfig config) : config_(config) {
    if (config_.directory == nullptr) config_.directory = LoggerConfig{}.directory;
    if (config_.robotName == nullptr) config_.robotName = "";
    // A zero period would make that task spin; everything else tolerates any
    // value.
    config_.samplePeriodMs = std::max<std::uint32_t>(config_.samplePeriodMs, 1);
    config_.writerPeriodMs = std::max<std::uint32_t>(config_.writerPeriodMs, 1);
    config_.stagingBytes = std::max(config_.stagingBytes, kMinStagingBytes);

    nullChannel_ = std::make_unique<Channel>(
        ChannelSchema{.id = 0xFFFF, .name = "_", .kind = ChannelKind::samples},
        kNullChannelCapacity);

    // Ids 0 and 1 are always the event channels (the format spec promises it).
    channels_[0] = std::make_unique<Channel>(
        ChannelSchema{.id = 0, .name = "sys", .kind = ChannelKind::events, .decimals = 0},
        kSystemEventsCapacity, &fileEpoch_);
    channels_[1] = std::make_unique<Channel>(
        ChannelSchema{.id = 1, .name = "events", .kind = ChannelKind::events, .decimals = 0},
        kUserEventsCapacity, &fileEpoch_);
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
    // Kept (not just the probe) so the destructor can detach it again.
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
        // getPose() takes Odometry's pose mutex for a moment. That's safe
        // here: PROS mutexes inherit priority, and neither this task nor the
        // Odometry task is ever deleted while holding it.
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
            // Six quick reads of state the brain already has cached from the
            // motor's last status packet; no radio or device round trip.
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

    // Channels are matched by name across files, so a second channel with the
    // same name would silently merge two streams in the app.
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

    channels_[count] = std::make_unique<Channel>(std::move(schema), options.capacity, &fileEpoch_);
    // Published last: the sampler, writer, and status() only ever look at
    // entries below the count they loaded.
    channelCount_.store(count + 1, std::memory_order_release);
    return *channels_[count];
}

bool Logger::isNullChannel(const Channel& channel) const { return &channel == nullChannel_.get(); }

// --- Producers ---------------------------------------------------------------

bool Logger::event(const char* tag, const char* format, ...) {
    std::va_list args;
    va_start(args, format);
    const bool logged = vevent(tag, format, args);
    va_end(args);
    return logged;
}

bool Logger::vevent(const char* tag, const char* format, std::va_list args) {
    // On the caller's stack and sized to what one event can carry, so
    // formatting never touches the heap (newlib's own float formatting
    // aside — see the header).
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

// --- Control -----------------------------------------------------------------

bool Logger::start() {
    std::lock_guard<pros::Mutex> lock(registrationMutex_);
    if (started_) return false;
    started_ = true;

    // Everything the writer needs up front, so neither task ever allocates
    // on a schedule.
    staging_ = std::make_unique<char[]>(config_.stagingBytes);
    stdioBuffer_ = std::make_unique<char[]>(config_.stagingBytes);
    state_.store(LoggerState::waitingForCard, std::memory_order_release);
    activeLogger.store(this, std::memory_order_release);

    // Below every control loop (8), so recording and polling never preempt
    // one — and above the writer, whose SD writes can take a couple of
    // hundred milliseconds that pose sampling shouldn't share.
    sampler_ = std::make_unique<pros::Task>([this] { samplerLoop(); }, TASK_PRIORITY_DEFAULT - 1,
                                            TASK_STACK_DEPTH_DEFAULT, "SL Sampler");
    // As low as a task can usefully go: it only runs when nothing else wants
    // the CPU. The ProducerGate reclaim rule depends on every producer being
    // above this.
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
                        .slowestWriteUs = slowestWriteUs_.load(std::memory_order_relaxed)};
}

std::uint32_t Logger::droppedRows() const {
    std::uint32_t dropped = 0;
    const std::size_t count = channelCount_.load(std::memory_order_acquire);
    for (std::size_t i = 0; i < count; ++i) {
        dropped += channels_[i]->droppedFull() + channels_[i]->droppedContended();
    }
    return dropped;
}

// --- Sampler task ------------------------------------------------------------

void Logger::samplerLoop() {
    std::uint32_t wake = pros::millis();
    while (true) {
        samplerTick();
        pros::Task::delay_until(&wake, config_.samplePeriodMs);
    }
}

void Logger::samplerTick() {
    const std::uint8_t status = pros::competition::get_status();
    if (status != lastStatus_) {
        char text[48];
        phaseText(status, text, sizeof(text));
        // Only marked seen once the row went in, so a dropped one is retried
        // next tick rather than lost.
        if (systemEvents_->recordEvent("phase", text)) lastStatus_ = status;
        // Raised after the row is committed: the writer takes the request
        // before it drains, so the pass that acts on it also writes the row.
        // Entering disabled at the end of a match is on the card moments
        // later, before anyone can reach the power switch.
        flushRequested_.store(true, std::memory_order_release);
    }

    const bool disabledUnderControl =
        (status & COMPETITION_DISABLED) != 0 && (status & COMPETITION_CONNECTED) != 0;
    if (disabledUnderControl && !config_.pollWhileDisabled) return;

    const std::uint32_t nowMs = sapphirelib::millis();
    const std::size_t count = polledCount_.load(std::memory_order_acquire);
    for (std::size_t i = 0; i < count; ++i) {
        Polled& source = *polled_[i];
        // Half a sample period of slack, so a source whose period equals
        // samplePeriodMs isn't skipped by a tick that ran a millisecond early.
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

// --- Writer task -------------------------------------------------------------
//
// The file is written with stdio, but on the writer's terms: rows are
// formatted into staging_ and handed to the card in one fwrite() + fflush()
// per chunk, through a FILE whose stdio buffer (setvbuf()) is exactly as big
// as staging_. So a chunk is always one write — and on PROS 4.0.7+ every
// write is also a vexFileSync — and stdio never allocates a buffer of its
// own. Without that, newlib would split each chunk into BUFSIZ (1KB)
// writes, a sync apiece.

template <typename Format> bool Logger::stage(Format format, bool countsAsRow) {
    // Two tries: a line that doesn't fit in what's left of the buffer is
    // formatted again after writing the buffer out — never split across two
    // writes. The formatters write nothing we count when they return 0.
    for (int attempt = 0; attempt < 2 && file_ != nullptr; ++attempt) {
        const std::size_t length = format(staging_.get() + used_, config_.stagingBytes - used_);
        if (length > 0) {
            used_ += length;
            if (countsAsRow) ++fileRows_;
            return true;
        }
        // Didn't fit even an empty buffer (can't happen at kMinStagingBytes):
        // skip it rather than stall. Or the write failed, and we've faulted.
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
    const std::size_t count = channelCount_.load(std::memory_order_acquire);
    // Taken before draining: the sampler commits a phase row and *then*
    // raises this, so a request seen here guarantees the drain below includes
    // that row.
    const bool flushNow = flushRequested_.exchange(false, std::memory_order_acq_rel);

    manageFile(sapphirelib::millis(), count);

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
}

void Logger::manageFile(std::uint32_t nowMs, std::size_t channelCount) {
    if (file_ == nullptr) {
        if (faults_ >= kMaxFaults) return; // gave up; see fault()
        if (openBackoffMs_ != 0 && nowMs - lastOpenAttemptMs_ < openBackoffMs_) return;
        lastOpenAttemptMs_ = nowMs;
        tryOpen(channelCount);
        return;
    }
    // A pulled card doesn't always fail the next write cleanly (that's
    // undocumented), so look for it directly too.
    if (nowMs - lastCardCheckMs_ >= kCardCheckMs) {
        lastCardCheckMs_ = nowMs;
        if (pros::usd::is_installed() == 0) fault("card removed");
    }
}

void Logger::tryOpen(std::size_t channelCount) {
    // A fault stays visible (status(), HomePage) until a file opens again.
    auto notLogging = [this] {
        if (state_.load(std::memory_order_relaxed) != LoggerState::faulted) {
            state_.store(LoggerState::waitingForCard, std::memory_order_release);
        }
    };

    if (pros::usd::is_installed() == 0) {
        notLogging();
        openBackoffMs_ = kCardCheckMs;
        return;
    }

    const OpenResult inFolder = openIn(config_.directory, channelCount, nullptr);
    if (inFolder == OpenResult::opened) return;

    // The folder is missing — or fopen() failed in it, which on PROS usually
    // means the same thing (a failed open always reports ENFILE, whatever the
    // cause). The card's root is the fallback, unless that's where we were.
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
    // --- Pick a name: the next index after the highest already there -------
    std::int32_t index = 1;
    {
        // list_files() wants the path without "/usd" — and NUL-terminated,
        // which a string_view slice of `directory` may not be.
        char listPath[kMaxPathBytes];
        const std::string_view view = listingPath(directory);
        if (view.size() >= sizeof(listPath)) return OpenResult::failed;
        listPath[view.copy(listPath, view.size())] = '\0';

        // Zeroed, and filled to one byte short, so it's always terminated.
        auto listing = std::make_unique<char[]>(kListingBytes);
        errno = 0;
        if (pros::usd::list_files(listPath, listing.get(),
                                  static_cast<std::int32_t>(kListingBytes - 1)) == PROS_ERR) {
            if (errno == ENOENT && fallbackNote == nullptr) return OpenResult::folderMissing;
            // Any other failure: carry on as if the folder were empty. The
            // existence check below still keeps an old log from being
            // overwritten.
            listing[0] = '\0';
        }
        const std::int32_t highest =
            highestLogFileIndex(std::string_view(listing.get(), std::strlen(listing.get())));
        if (highest >= 0) index = highest + 1;
    }

    // Never trust the listing alone before opening with "w", which truncates:
    // it may have been cut short by its buffer, or name files in a form we
    // didn't expect. Each name that turns out to exist is skipped.
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

    // "w" is the only mode that creates a file here: PROS rejects every
    // read-write mode, and whether "a" creates one is unverified.
    std::FILE* file = std::fopen(path, "w");
    if (file == nullptr) return OpenResult::failed;

    // After a fault on a pulled card the old buffer went with the abandoned
    // FILE (see fault()); this file gets a fresh one.
    if (!stdioBuffer_) stdioBuffer_ = std::make_unique<char[]>(config_.stagingBytes);
    if (std::setvbuf(file, stdioBuffer_.get(), _IOFBF, config_.stagingBytes) != 0) {
        SAPPHIRELIB_LOG_WARN("telemetry", "setvbuf failed; SD writes will be split up");
    }

    // --- The file is open: reset everything that's per file ----------------
    char previousName[sizeof(fileName_)];
    std::memcpy(previousName, fileName_, sizeof(previousName));
    formatLogFileName(fileName_, sizeof(fileName_), static_cast<std::uint32_t>(index));
    file_ = file;
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
    // Every PidProbe sees this at its next step and repeats its C and G rows,
    // so each file can be read without the one before it.
    fileEpoch_.fetch_add(1, std::memory_order_release);

    // --- Header, schemas, and the events that open every file --------------
    const std::uint64_t openUs = sapphirelib::micros();
    const FileHeader header{.library = SAPPHIRELIB_VERSION,
                            .kernel = PROS_VERSION_STRING,
                            .build = _PROS_COMPILE_TIMESTAMP,
                            .robotName = config_.robotName,
                            .fileName = fileName_,
                            .directory = directory,
                            .openUs = openUs};
    stage([&](char* out, std::size_t size) { return formatHeader(out, size, header); }, false);
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
    // On the card at once, so even a run cut short a moment from now leaves a
    // readable file. (A failure here faults like any other write.)
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

void Logger::drain(std::size_t channelCount) {
    // A k-way merge on t_us over each channel's oldest row. A head is
    // refreshed only for the channel just taken from, so one pass merges one
    // snapshot; rows committed mid-pass wait for the next.
    const Record* heads[kMaxChannels] = {};
    for (std::size_t i = 0; i < channelCount; ++i) heads[i] = channels_[i]->front();

    for (std::size_t rows = 0; rows < kMaxRowsPerPass; ++rows) {
        std::size_t next = channelCount;
        for (std::size_t i = 0; i < channelCount; ++i) {
            if (heads[i] != nullptr && (next == channelCount || heads[i]->tUs < heads[next]->tUs)) {
                next = i;
            }
        }
        if (next == channelCount) return; // everything drained

        Channel& channel = *channels_[next];
        const Record& record = *heads[next];

        // An event longer than one Record is its event Record plus
        // continuations, committed together — so they're all here, and are
        // taken together.
        const Record* parts[kMaxEventRecords] = {&record};
        std::size_t taken = 1;
        if (record.kind == RecordKind::event) {
            while (taken <= record.count && taken < kMaxEventRecords) {
                const Record* part = channel.peek(taken);
                if (part == nullptr || part->kind != RecordKind::eventContinued) break;
                parts[taken++] = part;
            }
        }

        // A continuation at the front with no event before it (only possible
        // after a ring resync) has no row of its own; it's just discarded.
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
            if (!staged && file_ == nullptr) ++unlogged_;
        }

        channel.pop(taken);
        heads[next] = channel.front();
    }
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
    };
    const std::uint64_t nowUs = sapphirelib::micros();
    if (!stage([&](char* out, std::size_t size) { return formatHealth(out, size, nowUs, stats); },
               true)) {
        return;
    }
    healthWriteMaxUs_ = 0;
    healthWriteSumUs_ = 0;
    healthWrites_ = 0;

    // D rows only for channels whose counts moved since the last ones in this
    // file (lastDrops_ is cleared at each open, so a new file restates them).
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
    // fwrite() only copies into stdioBuffer_ (the same size, so it never
    // spills part of it); fflush() is the one write + sync.
    const bool ok =
        std::fwrite(staging_.get(), 1, used_, file_) == used_ && std::fflush(file_) == 0;
    const auto us = static_cast<std::uint32_t>(
        std::min<std::uint64_t>(sapphirelib::micros() - startUs, UINT32_MAX));
    lastWriteMs_ = sapphirelib::millis();

    ++fileWrites_;
    ++healthWrites_;
    healthWriteSumUs_ += us;
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
            // What closing a file on a pulled card does is undocumented, so
            // the FILE is abandoned instead — one of the VFS's 27 file slots,
            // leaked (kMaxFaults bounds how many). It still points at
            // stdioBuffer_, so the buffer is abandoned with it: the next file
            // gets a fresh one, and nothing that ever walks newlib's open
            // streams (a stray fflush(NULL)) can push the next file's bytes
            // through this dead one.
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
