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
#include <string>
#include <utility>
#include <vector>

#include "pros/rtos.hpp"
#include "sapphirelib/control/pid.hpp"
#include "sapphirelib/odom/odometry.hpp"
#include "sapphirelib/telemetry/channel.hpp"
#include "sapphirelib/telemetry/event.hpp"
#include "sapphirelib/telemetry/motor_row.hpp"
#include "sapphirelib/telemetry/recording_policy.hpp"

namespace sapphirelib::telemetry {

// channels one Logger can hold, its two event channels included
constexpr std::size_t kMaxChannels = 48;

// poll(), pose(), and motor() sources one Logger can hold
constexpr std::size_t kMaxPolled = 32;

/**
 * @brief Settings for the telemetry logger
 */
struct LoggerConfig {
    /**
     * folder for log files, "/usd/..." form. "/usd/sl" by default. It must already exist, since the
     * V5 can't create folders, so make it on a computer once. If it's missing, files go to the
     * card's root. Must be a string literal
     */
    const char* directory = "/usd/sl";

    /** written into each file's header, to tell robots' logs apart. Must be a string literal */
    const char* robotName = "";

    /** how often poll() sources and the competition state are checked, in milliseconds */
    std::uint32_t samplePeriodMs = 10;

    /** how often the writer drains every channel, in milliseconds */
    std::uint32_t writerPeriodMs = 20;

    /**
     * longest time rows wait before being written to the card, in milliseconds. Also bounds what a
     * sudden power off loses. Phase changes and requestFlush() write right away
     */
    std::uint32_t flushPeriodMs = 250;

    /** how often writer health and dropped row counts are logged, in milliseconds */
    std::uint32_t healthPeriodMs = 1000;

    /** staging buffer size in bytes, the most one write to the card carries. At least 4KB */
    std::size_t stagingBytes = 16 * 1024;

    /**
     * whether poll() sources keep running while disabled under competition control. false by
     * default, so a robot sitting in the queue doesn't log for minutes
     */
    bool pollWhileDisabled = false;

    /**
     * record from start() on, in one file, until stopRecording(). true by default. false waits
     * for startRecording(), e.g. a button on the Home page, or for competition control
     */
    bool recordAtStart = true;

    /**
     * record whenever competition control (a field, or a competition switch) is connected: a new
     * file when it connects, closed once it has been disconnected for competitionStopDelayMs.
     * true by default. A recording already running when it connects is left alone
     */
    bool recordUnderCompetition = true;

    /**
     * how long competition control must stay disconnected before a recording it started ends, in
     * milliseconds, so a tether that drops for a moment doesn't split a match across two files
     */
    std::uint32_t competitionStopDelayMs = 5000;
};

/**
 * @brief What the logger is doing
 */
enum class LoggerState : std::uint8_t {
    stopped,        // start() hasn't been called
    idle,           // started, not recording. Any file is closed, so the card is safe to pull
    waitingForCard, // recording, but no card, or no file could be opened yet. Retrying
    logging,        // writing to a file
    faulted,        // a write failed (card pulled or full). Rows are dropped until a new file opens
};

/**
 * @brief A snapshot of the logger, for a status readout
 */
struct LoggerStatus {
    /** what the logger is doing */
    LoggerState state = LoggerState::stopped;
    /** the current or last file's number, 42 for SL000042.CSV. -1 if none yet */
    std::int32_t fileIndex = -1;
    /** whether the folder was missing and the file is in the card's root */
    bool inRootFolder = false;
    /** bytes written to the current file */
    std::uint32_t bytesWritten = 0;
    /** rows dropped across every channel since start() */
    std::uint32_t droppedRows = 0;
    /** slowest write to the card since start(), in microseconds */
    std::uint32_t slowestWriteUs = 0;

    /** whether a recording is running (it may still be waiting for a card) */
    bool recording = false;

    /** why the running recording started. none when there isn't one */
    RecordingReason reason = RecordingReason::none;

    /** whether an SD card was in the brain, as of the last check (about once a second) */
    bool cardInserted = false;
};

/**
 * @brief Records PIDs, the pose, motor health, your own values, and events to the SD card
 *
 * One file per recording (docs/TELEMETRY_FORMAT.md), for the telemetry analyzer. By default a
 * recording runs from start() to the end of the program; LoggerConfig::recordAtStart = false waits
 * for startRecording() (a button) or for competition control to connect. Between recordings the
 * file is closed and nothing is recorded. It never slows the code it records: recording copies 64
 * bytes and moves on, and all SD work happens on the logger's own low priority task. Recording is
 * safe from any task and never blocks
 *
 * @note make exactly one, with static storage, and register channels in initialize(). Registering
 * from autonomous() or opcontrol() isn't safe
 *
 * @b Example
 * @code {.cpp}
 * sapphirelib::telemetry::Logger& logger() {
 *     static sapphirelib::telemetry::Logger instance({.robotName = "1234A"});
 *     return instance;
 * }
 *
 * void initialize() {
 *     logger().pid("drive", drivetrain().drivePID());
 *     logger().pid("turn", drivetrain().turnPID());
 *     logger().pose(odometry());
 *     logger().motor("motor.lift", 20);
 *     logger().start();
 * }
 * @endcode
 */
class Logger {
public:
    /**
     * @brief Construct a new Logger. Starts nothing until start()
     *
     * @param config logger settings
     */
    explicit Logger(LoggerConfig config = {});

    // detaches every PID observer and stops both tasks. A Logger is meant to outlive the program
    ~Logger();

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    /**
     * @brief Log every update() of a PID, and its gains whenever they change
     *
     * Gains the tuner page or Auto-Tune set show up in the log on their own. Replaces any observer
     * the PID already had. Call before any task starts running the PID
     *
     * @param name channel name
     * @param pid the PID
     * @param options channel settings
     * @return Channel& the channel
     *
     * @b Example
     * @code {.cpp}
     * logger().pid("lift", lift.pid(), {.capacity = 128, .decimals = 3});
     * @endcode
     */
    Channel& pid(const char* name, PID& pid, ChannelOptions options = {});

    /**
     * @brief Make a channel for your own values, recorded with Channel::record()
     *
     * Names must be unique; "sys" and "events" are reserved. A duplicate name or a full table logs
     * a warning and returns a channel that's never written
     *
     * @param name channel name
     * @param columns column names, at most 13
     * @param options channel settings
     * @return Channel& the channel
     *
     * @b Example
     * @code {.cpp}
     * sapphirelib::telemetry::Channel& intakeLog = logger().channel("intake", {"volts", "rpm"});
     * // in opcontrol
     * intakeLog.record({intakeVolts, intake.get_actual_velocity()});
     * @endcode
     */
    Channel& channel(const char* name, std::initializer_list<const char*> columns,
                     ChannelOptions options = {});

    /**
     * @brief Make a channel the logger fills in itself every periodMs
     *
     * @note read runs on the logger's sampler task alongside every other source, so it must be
     * quick and must not block
     *
     * @param name channel name
     * @param columns column names, at most 13
     * @param periodMs how often to read, in milliseconds. Rounded to samplePeriodMs
     * @param read fills in the values, given an array of NaNs
     * @param options channel settings
     * @return Channel& the channel
     *
     * @b Example
     * @code {.cpp}
     * logger().poll("batt", {"volts", "pct"}, 1000, [](double* values) {
     *     values[0] = pros::battery::get_voltage() / 1000.0;
     *     values[1] = pros::battery::get_capacity();
     * });
     * @endcode
     */
    Channel& poll(const char* name, std::initializer_list<const char*> columns,
                  std::uint32_t periodMs, std::function<void(double* values)> read,
                  ChannelOptions options = {});

    /**
     * @brief Log an odometry's pose: x, y (inches), and heading (0-360 degrees)
     *
     * @param odometry the odometry. Must outlive the Logger
     * @param name channel name. "odom" by default
     * @param periodMs how often to read, in milliseconds. 10 by default
     * @return Channel& the channel
     */
    Channel& pose(const odom::Odometry& odometry, const char* name = "odom",
                  std::uint32_t periodMs = 10);

    /**
     * @brief Log a motor's health: volts, amps, temperature, RPM, efficiency, and fault bits
     *
     * Read straight off the port, and nothing is commanded. An unplugged motor logs rows of NaN.
     * V5 motors cut their own power as they heat and report it nowhere else, so this is how a
     * mechanism that faded mid-match shows up. See motor_row.hpp for the columns
     *
     * @param name channel name, "motor.<something>" by convention
     * @param port motor port. Negative reports volts and RPM reversed
     * @param periodMs how often to read, in milliseconds. 100 by default
     * @param options channel settings
     * @return Channel& the channel
     *
     * @b Example
     * @code {.cpp}
     * logger().motor("motor.liftA", -20);
     * logger().motor("motor.liftB", 19);
     * @endcode
     */
    Channel& motor(const char* name, std::int8_t port, std::uint32_t periodMs = 100,
                   ChannelOptions options = {.capacity = 32, .decimals = 2});

    /**
     * @brief Log an event, printf style
     *
     * For markers the analyzer can find later: an auton starting, a driver's "that looked wrong"
     * button. Formats on the calling task, so keep it out of tight loops. Code without a Logger
     * can use the free telemetry::event()
     *
     * @param tag event tag, cut to 15 characters
     * @param format printf format string. The message is cut to 191 characters
     * @return true the event was logged
     *
     * @b Example
     * @code {.cpp}
     * logger().event("auton", "start,%s", "left side");
     * @endcode
     */
    bool event(const char* tag, const char* format, ...) __attribute__((format(printf, 3, 4)));

    /**
     * @brief event() with a va_list
     */
    bool vevent(const char* tag, const char* format, std::va_list args);

    /**
     * @brief Add a "#meta,<key>,<value>" line to the top of every file, e.g. the settings a
     * localizer or a TUNE.CFG ran with, so each log says what produced it
     *
     * Call before start(); later calls are ignored. Keys keep letters, digits, '_', '.', and '-',
     * at most 63 of them; values are cut to 63 characters. Both are copied
     *
     * @param key the key, e.g. "tune.rev"
     * @param value the value, e.g. "7"
     *
     * @b Example
     * @code {.cpp}
     * logger().addMeta("mcl.particles", "300");
     * @endcode
     */
    void addMeta(const char* key, const char* value);

    /**
     * @brief Start a recording, in a new file. Safe from any task, e.g. a GUI button
     *
     * Takes effect within samplePeriodMs. Does nothing if one is already running
     */
    void startRecording();

    /**
     * @brief End the recording, and close its file within writerPeriodMs. Safe from any task
     *
     * Everything recorded before the call is written first, so pulling the card once
     * status().state is idle loses nothing. A recording competition control started can be
     * stopped too; the next connection starts another
     */
    void stopRecording();

    /**
     * @brief Whether a recording is running. Safe from any task
     */
    bool recording() const;

    /**
     * @brief Start the logger's tasks. Returns right away
     *
     * The card is checked and the file opened on the logger's own task, so a missing card never
     * slows initialize(). Also makes this the Logger that the free telemetry::event() logs to
     *
     * @return true the logger started
     * @return false it was already started
     */
    bool start();

    /**
     * @brief Write everything waiting to the card within writerPeriodMs, e.g. before a risky test
     */
    void requestFlush();

    /**
     * @brief Get a snapshot of the logger. Safe from any task
     */
    LoggerStatus status() const;

private:
    struct Polled {
        Channel* channel;
        std::uint32_t periodMs;
        std::function<void(double*)> read;
        // sampler task only
        std::uint32_t lastMs = 0;
        bool polledOnce = false;
    };

    // poll()'s body, for a column list that isn't an initializer_list
    Channel& pollColumns(const char* name, const char* const* columns, std::size_t columnCount,
                         std::uint32_t periodMs, std::function<void(double* values)> read,
                         ChannelOptions options);

    // registration helper. The caller holds registrationMutex_
    Channel& addChannelLocked(const char* name, ChannelKind kind, const char* const* columns,
                              std::size_t columnCount, ChannelOptions options);
    bool isNullChannel(const Channel& channel) const;

    std::uint32_t droppedRows() const;

    void samplerLoop();
    void samplerTick();
    void pollSources(std::uint32_t nowMs);

    // how one attempt to open a file in one folder went
    enum class OpenResult : std::uint8_t { opened, folderMissing, failed };

    // writer task only, so these touch the writer state below without a lock
    void writerLoop();
    void writerPass();
    void manageFile(std::uint32_t nowMs, std::size_t channelCount, bool wanted);
    void closeFile(std::size_t channelCount);
    void tryOpen(std::size_t channelCount);
    OpenResult openIn(const char* directory, std::size_t channelCount, const char* fallbackNote);
    void announceChannels(std::size_t channelCount);
    // true if it stopped at the per-pass row limit with rows still waiting
    bool drain(std::size_t channelCount);
    void writeHealth(std::size_t channelCount);
    bool writeStaged();
    void fault(const char* reason);
    template <typename Format> bool stage(Format format, bool countsAsRow);

    LoggerConfig config_;
    pros::Mutex registrationMutex_;
    bool started_ = false; // under registrationMutex_

    // append-only tables: an entry is fully built before its count is published (release), and
    // readers load the count (acquire), so the tasks read them without a lock. A channel's index
    // is its id in the file
    std::array<std::unique_ptr<Channel>, kMaxChannels> channels_;
    std::atomic<std::size_t> channelCount_{0};
    std::array<std::unique_ptr<Polled>, kMaxPolled> polled_;
    std::atomic<std::size_t> polledCount_{0};
    std::vector<std::pair<PID*, std::unique_ptr<PidProbe>>> probes_; // under mutex
    std::vector<std::pair<std::string, std::string>> meta_;          // fixed once started_
    std::unique_ptr<Channel> nullChannel_;

    Channel* systemEvents_ = nullptr; // id 0, phases. The sampler records into it
    Channel* userEvents_ = nullptr;   // id 1, event()

    std::atomic<std::uint32_t> fileEpoch_{0};
    std::atomic<bool> flushRequested_{false};

    // recording: every channel records only while recording_ is set. The sampler owns the policy
    // and sets it, after counting a new session for every start, so the writer can tell a stop
    // and a start apart from one recording. request_ carries startRecording()/stopRecording() to
    // the sampler
    RecordingPolicy policy_; // sampler only
    std::atomic<bool> recording_{false};
    std::atomic<std::uint32_t> session_{0};
    std::atomic<RecordingRequest> request_{RecordingRequest::none};
    std::atomic<RecordingReason> reason_{RecordingReason::none};
    std::atomic<RecordingReason> stopReason_{RecordingReason::none};

    // sampler time for the H row's samp_us: wall time, so an upper bound on its CPU
    std::atomic<std::uint32_t> samplerBusyUs_{0};

    // the competition status the sampler last logged (sampler only). No real status has every bit
    // set, so the first tick always logs one
    std::uint8_t lastStatus_ = 0xFF;

    // status() reads these from any task; only the writer (and start()) stores them
    std::atomic<LoggerState> state_{LoggerState::stopped};
    std::atomic<std::int32_t> fileIndex_{-1};
    std::atomic<bool> inRootFolder_{false};
    std::atomic<std::uint32_t> bytesWritten_{0};
    std::atomic<std::uint32_t> slowestWriteUs_{0};
    std::atomic<bool> cardInserted_{false};

    // writer task state. Allocated in start(), before the writer exists; only the writer touches
    // it after that
    std::unique_ptr<char[]> staging_;     // rows formatted, not yet written
    std::unique_ptr<char[]> stdioBuffer_; // handed to setvbuf() for file_
    std::size_t used_ = 0;                // bytes of staging_ in use
    std::FILE* file_ = nullptr;           // null unless logging
    char fileName_[16] = {};              // current (or last) file's name
    std::size_t announced_ = 0;           // channels whose #chan is in file_
    std::uint32_t fileSession_ = 0;       // the recording file_ belongs to
    std::array<std::pair<std::uint32_t, std::uint32_t>, kMaxChannels> lastDrops_{};

    std::uint32_t lastOpenAttemptMs_ = 0;
    std::uint32_t openBackoffMs_ = 0; // 0: try at the next pass
    std::uint32_t lastCardCheckMs_ = 0;
    std::uint32_t lastWriteMs_ = 0;
    std::uint32_t lastHealthMs_ = 0;

    // per-file counters, reset when a file opens
    std::uint32_t fileRows_ = 0;
    std::uint32_t fileBytes_ = 0;
    std::uint32_t fileWrites_ = 0;
    // write timing since the last H row
    std::uint32_t healthWriteMaxUs_ = 0;
    std::uint64_t healthWriteSumUs_ = 0;
    std::uint32_t healthWrites_ = 0;
    // since start()
    std::uint32_t unlogged_ = 0;
    std::uint32_t breaks_ = 0;
    std::uint32_t faults_ = 0;
    // writer time outside SD writes since the last H row (fmt_us), and SD write time this pass
    std::uint32_t writerBusyUs_ = 0;
    std::uint32_t passWriteUs_ = 0;
    bool reopenPending_ = false; // the next file opened follows a fault

    std::unique_ptr<pros::Task> sampler_;
    std::unique_ptr<pros::Task> writer_;
};

} // namespace sapphirelib::telemetry
