/**
 * \file sapphirelib/telemetry/logger.hpp
 *
 * SD-card telemetry: records PID steps, pose, and any custom values while the
 * robot runs, into one CSV-style file per program run for off-robot analysis
 * and tuning (format: docs/TELEMETRY_FORMAT.md; reference reader:
 * tools/telemetry/slt_read.py). Entirely opt-in — nothing else in SapphireLib
 * constructs one — and it never slows the code it records: producers copy 64
 * bytes and move on, and all SD work happens on the library's own
 * lowest-priority task.
 *
 * Team 96671H — Hitmen
 */

#pragma once

#include <array>
#include <atomic>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <initializer_list>
#include <memory>
#include <utility>
#include <vector>

#include "pros/rtos.hpp"
#include "sapphirelib/control/pid.hpp"
#include "sapphirelib/odom/odometry.hpp"
#include "sapphirelib/telemetry/channel.hpp"
#include "sapphirelib/telemetry/event.hpp"
#include "sapphirelib/telemetry/motor_row.hpp"

namespace sapphirelib::telemetry {

/// Channels one Logger can hold, its two event channels included. Room for a
/// robot that logs every motor (Logger::motor()) on top of its PIDs.
constexpr std::size_t kMaxChannels = 48;

/// poll()/pose()/motor() sources one Logger can hold.
constexpr std::size_t kMaxPolled = 32;

struct LoggerConfig {
    /// Folder for log files, "/usd/..." form. It must already exist on the
    /// card — the V5 can't create folders (PROS's mkdir is unimplemented) — so
    /// make it on a computer once. If it's missing, files go to the card's
    /// root instead and status().inRootFolder says so. Static storage (a
    /// string literal): it's read for the life of the program.
    const char* directory = "/usd/sl";

    /// Written into each file's header, to tell robots' logs apart. Static
    /// storage, like `directory`.
    const char* robotName = "";

    /// How often the sampler task polls poll()/pose() sources and the
    /// competition state. Each source's own period is honored on top.
    std::uint32_t samplePeriodMs = 10;

    /// How often the writer task drains every channel.
    std::uint32_t writerPeriodMs = 20;

    /// Longest drained rows wait in the staging buffer before being written to
    /// the card — which also bounds what a sudden power-off loses. On PROS
    /// 4.2.2 every write is a filesystem sync, so lowering this spends card
    /// time, not correctness. Phase changes and requestFlush() write at once.
    std::uint32_t flushPeriodMs = 250;

    /// How often `H` (writer health) and `D` (dropped rows) lines are written.
    std::uint32_t healthPeriodMs = 1000;

    /// Staging buffer, in bytes: the most one write to the card carries. At
    /// least 4KB. Allocated twice, in start(): once to format rows into, and
    /// once as the file's stdio buffer, sized to match so every chunk reaches
    /// the card as one write (and one filesystem sync) rather than 1KB pieces.
    std::size_t stagingBytes = 16 * 1024;

    /// False (the default) pauses poll()/pose() sources while the robot is
    /// disabled under competition control — a robot sitting in the queue
    /// would otherwise log an unchanging pose at 100Hz for minutes. With no
    /// competition control connected, PROS reports enabled and polling runs.
    bool pollWhileDisabled = false;
};

enum class LoggerState : std::uint8_t {
    stopped,        ///< start() hasn't been called.
    waitingForCard, ///< No card, or no file could be opened yet; retrying.
    logging,        ///< Writing to a file.
    faulted,        ///< A write failed (card pulled or full); rows are being
                    ///< dropped until a new file opens.
};

/// A snapshot for a status readout (HomePage line, controller screen).
struct LoggerStatus {
    LoggerState state = LoggerState::stopped;
    /// Current or last file's index — 42 for SL000042.CSV; -1 if none yet.
    std::int32_t fileIndex = -1;
    /// True if `directory` was missing and the file is in the card's root.
    bool inRootFolder = false;
    /// Bytes written to the current file.
    std::uint32_t bytesWritten = 0;
    /// Rows dropped across every channel since start().
    std::uint32_t droppedRows = 0;
    /// Slowest single write to the card since start(), in microseconds.
    std::uint32_t slowestWriteUs = 0;
};

/// The SD-card telemetry recorder. Make exactly one, with static storage
/// duration (a function-local `static`), and call start() once.
///
/// Register channels in initialize(). Registration takes a pros::Mutex that a
/// task deleted mid-call would orphan, and PID::setObserver() isn't
/// synchronized — both are fine at init and wrong from autonomous()/
/// opcontrol(). Channels registered after start() still work: their `#chan`
/// line is written before their first row.
///
/// Everything a producer calls — Channel::record(), Logger::event(), the free
/// telemetry::event(), a PID's observer — is safe from any task, never
/// blocks, never allocates on the record path, and drops (counted) rather
/// than wait.
///
/// Two tasks do the rest:
///   - the sampler ("SL Sampler", priority TASK_PRIORITY_DEFAULT - 1) polls
///     poll()/pose() sources and watches the competition phase, logging an
///     `E,...,phase,...` row and forcing a write on every change — so the
///     end of a match is on the card moments after the robot is disabled;
///   - the writer ("SL Writer", TASK_PRIORITY_MIN + 1) owns the file. It
///     drains every channel, merges rows by timestamp, formats them into a
///     staging buffer, and writes that buffer every flushPeriodMs. It opens
///     one file per program run (a new one after a recovered SD fault),
///     checks every second that the card is still there, and records its own
///     health in the file (`H`/`D` rows).
class Logger {
public:
    explicit Logger(LoggerConfig config = {});

    /// Detaches every PID observer and stops both tasks. Only here so misuse
    /// fails soft: a Logger is meant to outlive the program.
    ~Logger();

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    /// Records every update() of `pid` in a pid channel named `name` (see
    /// PidProbe), and its config and gains at the top of each file and whenever
    /// setGains() changes them — so gains that PidTunerPage or Auto-Tune set
    /// mid-session land in the log without either knowing telemetry exists.
    /// Works for any sapphirelib::PID: drivetrain controllers via
    /// drivePID()/turnPID()/headingHoldPID(), a mechanism's own. Replaces any
    /// observer `pid` already had (with a warning). Call before any task
    /// starts running `pid`.
    Channel& pid(const char* name, PID& pid, ChannelOptions options = {});

    /// A channel of your own columns (at most kMaxColumns), recorded with
    /// Channel::record() from wherever the values are computed — e.g. what a
    /// mechanism actually commanded after feedforward and clamping.
    ///
    /// Names are sanitized (see sanitizeName()) and must be unique; "sys" and
    /// "events" are reserved. A duplicate name, or a full table, logs a warning
    /// and returns a channel that is never written — never a null reference.
    Channel& channel(const char* name, std::initializer_list<const char*> columns,
                     ChannelOptions options = {});

    /// A channel whose values `read` fills in, called on the sampler task every
    /// `periodMs` (rounded to samplePeriodMs). `read` gets an array of
    /// kMaxColumns NaNs and fills the first columns.size(); it must be quick
    /// and must not block — it shares the sampler with every other source.
    Channel& poll(const char* name, std::initializer_list<const char*> columns,
                  std::uint32_t periodMs, std::function<void(double* values)> read,
                  ChannelOptions options = {});

    /// poll() for an Odometry's pose: columns x, y, heading (inches, inches,
    /// degrees 0-360, as odom::Pose). `odometry` must outlive the Logger.
    Channel& pose(const odom::Odometry& odometry, const char* name = "odom",
                  std::uint32_t periodMs = 10);

    /// poll() for one V5 smart motor's health, read straight off its port
    /// (no pros::Motor needed, and nothing is ever commanded): the volts it's
    /// applying, current (A), temperature (°C), velocity (RPM), efficiency
    /// (%) and fault bits — kMotorColumns, see motor_row.hpp. A negative port
    /// reports volts and RPM in the reversed direction, as a pros::Motor on
    /// that port would. An unplugged motor logs whole rows of NaN.
    ///
    /// The evidence for a robot that faded mid-match: V5 motors cut their own
    /// power as they heat and say nothing, so only this shows it. Name it
    /// "motor.<something>" by convention (the telemetry analyzer finds motor
    /// channels by their columns either way). 100ms is plenty: temperature
    /// moves over tens of seconds, and a stall shows in current for far longer
    /// than that.
    Channel& motor(const char* name, std::int8_t port, std::uint32_t periodMs = 100,
                   ChannelOptions options = {.capacity = 32, .decimals = 2});

    /// Logs an `E` row: a marker the app can cut runs on (auton start/end, a
    /// driver's "that looked wrong" button, a macro firing). Formats on the
    /// calling task, so keep it out of tight loops — it's for markers, not
    /// data (newlib may allocate once per task the first time it formats a
    /// float). The tag is cut to 15 characters and the message to 191. False
    /// if dropped. Library code without a Logger in hand uses the free
    /// telemetry::event() (event.hpp), which lands here once start() has run.
    bool event(const char* tag, const char* format, ...) __attribute__((format(printf, 3, 4)));

    /// event() with a va_list — what the free telemetry::event() forwards to.
    bool vevent(const char* tag, const char* format, std::va_list args);

    /// Starts the sampler (priority TASK_PRIORITY_DEFAULT - 1) and writer
    /// (TASK_PRIORITY_MIN + 1) tasks and returns at once; the card is checked
    /// and the file opened on the writer task, so a missing card never slows
    /// initialize(). Also makes this the Logger the free telemetry::event()
    /// logs to. False only if already started.
    bool start();

    /// Asks the writer to write everything staged at its next pass (within
    /// writerPeriodMs) — e.g. before a risky test, or before powering off.
    void requestFlush();

    /// Safe from any task, including the GUI's.
    LoggerStatus status() const;

private:
    struct Polled {
        Channel* channel;
        std::uint32_t periodMs;
        std::function<void(double*)> read;
        // Sampler task only.
        std::uint32_t lastMs = 0;
        bool polledOnce = false;
    };

    /// poll()'s body, for a column list that isn't an initializer_list.
    Channel& pollColumns(const char* name, const char* const* columns, std::size_t columnCount,
                         std::uint32_t periodMs, std::function<void(double* values)> read,
                         ChannelOptions options);

    /// Registration helper; the caller holds registrationMutex_.
    Channel& addChannelLocked(const char* name, ChannelKind kind, const char* const* columns,
                              std::size_t columnCount, ChannelOptions options);
    bool isNullChannel(const Channel& channel) const;

    std::uint32_t droppedRows() const;

    void samplerLoop();
    void samplerTick();

    /// How one attempt to open a file in one folder went.
    enum class OpenResult : std::uint8_t { opened, folderMissing, failed };

    // Writer task. Each helper runs only there (tryOpen() and friends touch
    // the writer-state members below without a lock for that reason).
    void writerLoop();
    void writerPass();
    void manageFile(std::uint32_t nowMs, std::size_t channelCount);
    void tryOpen(std::size_t channelCount);
    OpenResult openIn(const char* directory, std::size_t channelCount, const char* fallbackNote);
    void announceChannels(std::size_t channelCount);
    void drain(std::size_t channelCount);
    void writeHealth(std::size_t channelCount);
    bool writeStaged();
    void fault(const char* reason);
    template <typename Format> bool stage(Format format, bool countsAsRow);

    LoggerConfig config_;
    pros::Mutex registrationMutex_;
    bool started_ = false; // under registrationMutex_

    // Append-only tables: an entry is fully built before the count that
    // publishes it is incremented (release), and readers load the count
    // (acquire) — so the tasks iterate them without a lock. A channel's
    // index in channels_ is its id in the file.
    std::array<std::unique_ptr<Channel>, kMaxChannels> channels_;
    std::atomic<std::size_t> channelCount_{0};
    std::array<std::unique_ptr<Polled>, kMaxPolled> polled_;
    std::atomic<std::size_t> polledCount_{0};
    std::vector<std::pair<PID*, std::unique_ptr<PidProbe>>> probes_; // under mutex
    std::unique_ptr<Channel> nullChannel_;

    Channel* systemEvents_ = nullptr; // id 0 — phases; the sampler is its producer
    Channel* userEvents_ = nullptr;   // id 1 — event()

    std::atomic<std::uint32_t> fileEpoch_{0};
    std::atomic<bool> flushRequested_{false};

    /// The competition status the sampler last logged (sampler task only). No
    /// real status has every bit set, so its first tick always logs one.
    std::uint8_t lastStatus_ = 0xFF;

    // status() reads these from any task; only the writer (and start())
    // stores them.
    std::atomic<LoggerState> state_{LoggerState::stopped};
    std::atomic<std::int32_t> fileIndex_{-1};
    std::atomic<bool> inRootFolder_{false};
    std::atomic<std::uint32_t> bytesWritten_{0};
    std::atomic<std::uint32_t> slowestWriteUs_{0};

    // --- Writer-task state -------------------------------------------------
    // Allocated in start(), before the writer exists; after that only the
    // writer task touches any of it.
    std::unique_ptr<char[]> staging_;     // rows formatted, not yet written
    std::unique_ptr<char[]> stdioBuffer_; // handed to setvbuf() for file_
    std::size_t used_ = 0;                // bytes of staging_ in use
    std::FILE* file_ = nullptr;           // null unless logging
    char fileName_[16] = {};              // current (or last) file's name
    std::size_t announced_ = 0;           // channels whose #chan is in file_
    std::array<std::pair<std::uint32_t, std::uint32_t>, kMaxChannels> lastDrops_{};

    std::uint32_t lastOpenAttemptMs_ = 0;
    std::uint32_t openBackoffMs_ = 0; // 0: try at the next pass
    std::uint32_t lastCardCheckMs_ = 0;
    std::uint32_t lastWriteMs_ = 0;
    std::uint32_t lastHealthMs_ = 0;

    // Per-file counters (reset when a file opens).
    std::uint32_t fileRows_ = 0;
    std::uint32_t fileBytes_ = 0;
    std::uint32_t fileWrites_ = 0;
    // Write timing since the last H row.
    std::uint32_t healthWriteMaxUs_ = 0;
    std::uint64_t healthWriteSumUs_ = 0;
    std::uint32_t healthWrites_ = 0;
    // Since start().
    std::uint32_t unlogged_ = 0;
    std::uint32_t breaks_ = 0;
    std::uint32_t faults_ = 0;
    bool reopenPending_ = false; // the next file opened follows a fault

    std::unique_ptr<pros::Task> sampler_;
    std::unique_ptr<pros::Task> writer_;
};

} // namespace sapphirelib::telemetry
