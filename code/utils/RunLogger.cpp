#include "RunLogger.h"

#include "Version.h"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <system_error>

#if defined(_WIN32)
#include <process.h>
#else
#include <unistd.h>
#endif

namespace hgasp {

namespace {

constexpr std::uintmax_t kDefaultSegmentBytes =
    8U * 1024U * 1024U;
constexpr std::uintmax_t kMinimumSegmentBytes =
    64U * 1024U;
constexpr std::uintmax_t kMaximumSegmentBytes =
    64U * 1024U * 1024U;
constexpr unsigned int kDefaultBackupCount = 2U;
constexpr unsigned int kMaximumBackupCount = 5U;
constexpr std::size_t kDefaultMessageLimit = 4096U;
constexpr std::size_t kMaximumMessageLimit = 16384U;
constexpr std::size_t kMaximumFieldCount = 64U;
constexpr std::size_t kMaximumNameBytes = 128U;
constexpr std::size_t kMaximumStatusBytes = 64U * 1024U;

std::atomic<RunLogger*> activeLogger{nullptr};

long processId() noexcept {
#if defined(_WIN32)
    return static_cast<long>(_getpid());
#else
    return static_cast<long>(getpid());
#endif
}

std::string sanitizeName(std::string value) {
    for (char& character : value) {
        const unsigned char byte = static_cast<unsigned char>(character);
        if (!std::isalnum(byte) && character != '-' && character != '_') {
            character = '_';
        }
    }
    return value.empty() ? "instance" : value;
}

std::string runTimestamp() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t time =
        std::chrono::system_clock::to_time_t(now);
    std::tm utcTime{};
#if defined(_WIN32)
    gmtime_s(&utcTime, &time);
#else
    gmtime_r(&time, &utcTime);
#endif
    std::ostringstream output;
    output << std::put_time(&utcTime, "%Y%m%dT%H%M%SZ");
    return output.str();
}

std::string utcNow() {
    const auto now = std::chrono::system_clock::now();
    const auto milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) %
        1000;
    const std::time_t time =
        std::chrono::system_clock::to_time_t(now);
    std::tm utcTime{};
#if defined(_WIN32)
    gmtime_s(&utcTime, &time);
#else
    gmtime_r(&time, &utcTime);
#endif
    std::ostringstream output;
    output << std::put_time(&utcTime, "%Y-%m-%dT%H:%M:%S")
           << '.' << std::setw(3) << std::setfill('0')
           << milliseconds.count() << 'Z';
    return output.str();
}

std::size_t utf8PrefixLength(
        const std::string& value,
        const std::size_t maximumBytes) noexcept {
    if (value.size() <= maximumBytes) {
        return value.size();
    }
    std::size_t length = maximumBytes;
    while (length > 0U &&
           (static_cast<unsigned char>(value[length]) & 0xc0U) == 0x80U) {
        --length;
    }
    if (length == 0U) {
        return 0U;
    }
    const unsigned char lead =
        static_cast<unsigned char>(value[length]);
    std::size_t expected = 1U;
    if ((lead & 0xe0U) == 0xc0U) {
        expected = 2U;
    } else if ((lead & 0xf0U) == 0xe0U) {
        expected = 3U;
    } else if ((lead & 0xf8U) == 0xf0U) {
        expected = 4U;
    }
    if (length + expected > maximumBytes) {
        return length;
    }
    return maximumBytes;
}

std::string boundedString(
        const std::string& value,
        const std::size_t maximumBytes) {
    if (value.size() <= maximumBytes) {
        return value;
    }
    static const std::string suffix = "...[truncated]";
    if (maximumBytes <= suffix.size()) {
        return suffix.substr(0U, maximumBytes);
    }
    const std::size_t prefixLimit = maximumBytes - suffix.size();
    return value.substr(
               0U, utf8PrefixLength(value, prefixLimit)) +
           suffix;
}

std::string jsonEscape(
        const std::string& raw,
        const std::size_t maximumBytes) {
    const std::string value = boundedString(raw, maximumBytes);
    std::ostringstream output;
    for (const unsigned char character : value) {
        switch (character) {
            case '"':
                output << "\\\"";
                break;
            case '\\':
                output << "\\\\";
                break;
            case '\b':
                output << "\\b";
                break;
            case '\f':
                output << "\\f";
                break;
            case '\n':
                output << "\\n";
                break;
            case '\r':
                output << "\\r";
                break;
            case '\t':
                output << "\\t";
                break;
            default:
                if (character < 0x20U) {
                    output << "\\u00"
                           << std::hex << std::setw(2)
                           << std::setfill('0')
                           << static_cast<unsigned int>(character)
                           << std::dec;
                } else {
                    output << static_cast<char>(character);
                }
        }
    }
    return output.str();
}

bool sensitiveFieldName(const std::string& name) {
    std::string lowered = name;
    std::transform(
        lowered.begin(),
        lowered.end(),
        lowered.begin(),
        [](const unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });
    return lowered.find("token") != std::string::npos ||
           lowered.find("secret") != std::string::npos ||
           lowered.find("password") != std::string::npos ||
           lowered.find("passwd") != std::string::npos ||
           lowered.find("access_id") != std::string::npos ||
           lowered.find("accessid") != std::string::npos;
}

bool auditEvent(
        const std::string& component,
        const std::string& eventName) noexcept {
    if (eventName == "incumbent_changed" ||
        eventName == "checkpoint" ||
        eventName == "progress") {
        return true;
    }
    if (component == "sp") {
        return eventName == "adaptive_decision" ||
            eventName == "solve_started" ||
            eventName == "solve_skipped_preflight" ||
            eventName == "solve_skipped" ||
            eventName == "lp_probe_finished" ||
            eventName == "mip_skipped" ||
            eventName == "solve_finished" ||
            eventName == "budget_accounted" ||
            eventName == "injection_finished";
    }
    if (component == "route_pool") {
        return eventName == "final_master_diagnostic";
    }
    if (component == "population") {
        return eventName.find("configuration") != std::string::npos;
    }
    if (component == "lifecycle") {
        return eventName == "run.started" ||
            eventName == "component_activation" ||
            eventName == "orchestrator.shutdown_started" ||
            eventName == "orchestrator.shutdown_phase" ||
            eventName == "orchestrator.shutdown_completed" ||
            eventName == "data.shutdown_started" ||
            eventName == "data.shutdown_completed" ||
            eventName == "solver_context.shutdown_started" ||
            eventName == "solver_context.shutdown_completed" ||
            eventName == "run.completed" ||
            eventName == "run.failed";
    }
    if (component == "timing") {
        return eventName == "component_timing_final" ||
            eventName == "component_timing_complete" ||
            eventName == "hot_path_counters_final" ||
            eventName == "hot_path_counters_complete";
    }
    if (component == "logging") {
        return eventName == "logger.stopped";
    }
    return false;
}

const char* levelName(const LogLevel level) noexcept {
    switch (level) {
        case LogLevel::trace:
            return "trace";
        case LogLevel::debug:
            return "debug";
        case LogLevel::info:
            return "info";
        case LogLevel::warning:
            return "warning";
        case LogLevel::error:
            return "error";
    }
    return "info";
}

LogLevel configuredLevel() noexcept {
    const char* raw = std::getenv("HGASP_LOG_LEVEL");
    if (raw == nullptr || *raw == '\0') {
        return LogLevel::info;
    }
    const std::string value(raw);
    if (value == "trace") {
        return LogLevel::trace;
    }
    if (value == "debug") {
        return LogLevel::debug;
    }
    if (value == "warning" || value == "warn") {
        return LogLevel::warning;
    }
    if (value == "error") {
        return LogLevel::error;
    }
    return LogLevel::info;
}

bool configuredBoolean(
        const char* const name,
        const bool fallback) noexcept {
    const char* raw = std::getenv(name);
    if (raw == nullptr || *raw == '\0') {
        return fallback;
    }
    const std::string value(raw);
    if (value == "1" || value == "true" || value == "on") {
        return true;
    }
    if (value == "0" || value == "false" || value == "off") {
        return false;
    }
    return fallback;
}

std::uintmax_t configuredUnsigned(
        const char* name,
        const std::uintmax_t fallback,
        const std::uintmax_t minimum,
        const std::uintmax_t maximum) noexcept {
    const char* raw = std::getenv(name);
    if (raw == nullptr || *raw == '\0') {
        return fallback;
    }
    errno = 0;
    char* end = nullptr;
    const unsigned long long parsed =
        std::strtoull(raw, &end, 10);
    if (errno != 0 || end == raw || *end != '\0') {
        return fallback;
    }
    return std::clamp<std::uintmax_t>(
        static_cast<std::uintmax_t>(parsed),
        minimum,
        maximum);
}

std::filesystem::path backupPath(
        const std::filesystem::path& base,
        const unsigned int index) {
    return std::filesystem::path(
        base.string() + "." + std::to_string(index));
}

bool secureDirectory(
        const std::filesystem::path& path,
        std::error_code& error) noexcept {
    std::filesystem::permissions(
        path,
        std::filesystem::perms::owner_all,
        std::filesystem::perm_options::replace,
        error);
    return !error;
}

bool secureFile(
        const std::filesystem::path& path,
        std::error_code& error) noexcept {
    std::filesystem::permissions(
        path,
        std::filesystem::perms::owner_read |
            std::filesystem::perms::owner_write,
        std::filesystem::perm_options::replace,
        error);
    return !error;
}

bool rotateFile(
        const std::filesystem::path& base,
        const unsigned int backups) noexcept {
    std::error_code error;
    if (backups == 0U) {
        std::filesystem::remove(base, error);
        return !error;
    }
    std::filesystem::remove(backupPath(base, backups), error);
    if (error) {
        return false;
    }
    for (unsigned int index = backups; index > 1U; --index) {
        const std::filesystem::path source =
            backupPath(base, index - 1U);
        if (!std::filesystem::exists(source, error)) {
            if (error) {
                return false;
            }
            continue;
        }
        std::filesystem::rename(
            source, backupPath(base, index), error);
        if (error) {
            return false;
        }
        if (!secureFile(backupPath(base, index), error)) {
            return false;
        }
    }
    if (std::filesystem::exists(base, error)) {
        if (error) {
            return false;
        }
        std::filesystem::rename(base, backupPath(base, 1U), error);
        if (error) {
            return false;
        }
        if (!secureFile(backupPath(base, 1U), error)) {
            return false;
        }
    }
    return true;
}

void appendJsonValue(
        std::ostringstream& output,
        const EventValue& value,
        const std::size_t stringLimit) {
    switch (value.kind()) {
        case EventValue::Kind::null_value:
            output << "null";
            break;
        case EventValue::Kind::boolean:
            output << (value.boolean() ? "true" : "false");
            break;
        case EventValue::Kind::signed_integer:
            output << value.signedInteger();
            break;
        case EventValue::Kind::unsigned_integer:
            output << value.unsignedInteger();
            break;
        case EventValue::Kind::number:
            output << std::setprecision(
                std::numeric_limits<double>::max_digits10)
                   << value.number();
            break;
        case EventValue::Kind::text:
            output << '"'
                   << jsonEscape(value.text(), stringLimit)
                   << '"';
            break;
    }
}

void appendFields(
        std::ostringstream& output,
        const EventFields& fields,
        const std::size_t stringLimit) {
    output << '{';
    const std::size_t count =
        std::min(fields.size(), kMaximumFieldCount);
    for (std::size_t index = 0; index < count; ++index) {
        if (index != 0U) {
            output << ',';
        }
        output << '"'
               << jsonEscape(fields[index].name, kMaximumNameBytes)
               << "\":";
        if (sensitiveFieldName(fields[index].name)) {
            output << "\"[REDACTED]\"";
        } else {
            appendJsonValue(output, fields[index].value, stringLimit);
        }
    }
    if (fields.size() > count) {
        if (count != 0U) {
            output << ',';
        }
        output << "\"_fields_truncated\":true";
    }
    output << '}';
}

}  // namespace

class TeeStreamBuffer final : public std::streambuf {
public:
    TeeStreamBuffer(
        RunLogger& logger,
        std::streambuf* console)
        : logger_(logger), console_(console) {}

protected:
    int overflow(const int character) override {
        if (traits_type::eq_int_type(character, traits_type::eof())) {
            return sync() == 0 ? traits_type::not_eof(character)
                               : traits_type::eof();
        }
        const char value = traits_type::to_char_type(character);
        logger_.writeHuman(console_, &value, 1);
        return character;
    }

    std::streamsize xsputn(
        const char* data,
        const std::streamsize size) override {
        return logger_.writeHuman(console_, data, size);
    }

    int sync() override {
        return logger_.syncHuman(console_);
    }

private:
    RunLogger& logger_;
    std::streambuf* console_;
};

std::filesystem::path RunLogger::chooseDirectory(
    const std::string& instancePath,
    const int seed) {
    const char* configured = std::getenv("HGASP_LOG_DIR");
    if (configured != nullptr && *configured != '\0') {
        return std::filesystem::path(configured);
    }

    const std::string instanceName = sanitizeName(
        std::filesystem::path(instancePath).stem().string());
    return std::filesystem::path("logs") /
           instanceName /
           ("seed_" + std::to_string(seed)) /
           (runTimestamp() + "_pid_" + std::to_string(processId()));
}

bool RunLogger::consoleEnabled() {
    const char* configured = std::getenv("HGASP_LOG_CONSOLE");
    if (configured == nullptr || *configured == '\0' ||
        std::string(configured) == "1") {
        return true;
    }
    if (std::string(configured) == "0") {
        return false;
    }
    // Logging configuration must never prevent the solver from starting.
    return true;
}

RunLogger::RunLogger(
    const std::string& instancePath,
    const int seed)
    : directory_(chooseDirectory(instancePath, seed)),
      logFile_(directory_ / "hgasp.log"),
      eventFile_(directory_ / "events.jsonl"),
      auditFile_(directory_ / "audit_events.jsonl"),
      statusFile_(directory_ / "algorithm.status.json"),
      monotonicStarted_(std::chrono::steady_clock::now()),
      cpuStarted_(std::clock()),
      segmentBytes_(configuredUnsigned(
          "HGASP_LOG_SEGMENT_BYTES",
          kDefaultSegmentBytes,
          kMinimumSegmentBytes,
          kMaximumSegmentBytes)),
      backupCount_(static_cast<unsigned int>(configuredUnsigned(
          "HGASP_LOG_BACKUPS",
          kDefaultBackupCount,
          0U,
          kMaximumBackupCount))),
      messageLimit_(static_cast<std::size_t>(configuredUnsigned(
          "HGASP_LOG_MESSAGE_MAX_BYTES",
          kDefaultMessageLimit,
          256U,
          kMaximumMessageLimit))),
      flushEveryEvents_(configuredUnsigned(
          "HGASP_LOG_FLUSH_EVERY_EVENTS",
          64U,
          1U,
          4096U)),
      minimumLevel_(configuredLevel()),
      duplicateAuditEvents_(configuredBoolean(
          "HGASP_LOG_DUPLICATE_AUDIT_EVENTS",
          false)) {
    originalStdout_ = std::cout.rdbuf();
    originalStderr_ = std::cerr.rdbuf();

    std::error_code error;
    std::filesystem::create_directories(directory_, error);
    if (error || !secureDirectory(directory_, error)) {
        humanEnabled_ = false;
        eventsEnabled_ = false;
        auditEnabled_ = false;
        statusEnabled_ = false;
        warnOnce(
            "cannot initialize secure log directory " +
            directory_.string() + "; continuing without file logs");
    } else {
        humanEnabled_ = openHumanUnlocked();
        eventsEnabled_ = openEventsUnlocked();
        auditEnabled_ = openAuditUnlocked();
        if (!humanEnabled_ || !eventsEnabled_ ||
            !auditEnabled_) {
            warnOnce(
                "one or more log files could not be opened; "
                "continuing with reduced logging");
        }
    }

    const bool useConsole = consoleEnabled();
    stdoutBuffer_ = std::make_unique<TeeStreamBuffer>(
        *this, useConsole ? originalStdout_ : nullptr);
    stderrBuffer_ = std::make_unique<TeeStreamBuffer>(
        *this, useConsole ? originalStderr_ : nullptr);
    std::cout.rdbuf(stdoutBuffer_.get());
    std::cerr.rdbuf(stderrBuffer_.get());

    activeLogger.store(this, std::memory_order_release);
    logging_detail::eventCallback.store(
        &RunLogger::eventTrampoline, std::memory_order_release);
    logging_detail::statusCallback.store(
        &RunLogger::statusTrampoline, std::memory_order_release);

    std::cout << "[Log] run_directory=" << directory_.string() << '\n';
    EventFields loggerFields;
    loggerFields.reserve(8U);
    loggerFields.emplace_back(
        "directory", EventValue(directory_.string()));
    loggerFields.emplace_back(
        "segment_bytes",
        EventValue(
            static_cast<unsigned long long>(segmentBytes_)));
    loggerFields.emplace_back(
        "backups", EventValue(backupCount_));
    loggerFields.emplace_back(
        "human_file_enabled", EventValue(humanEnabled_));
    loggerFields.emplace_back(
        "event_file_enabled", EventValue(eventsEnabled_));
    loggerFields.emplace_back(
        "audit_file_enabled", EventValue(auditEnabled_));
    loggerFields.emplace_back(
        "flush_every_events",
        EventValue(
            static_cast<unsigned long long>(
                flushEveryEvents_)));
    loggerFields.emplace_back(
        "duplicate_audit_events",
        EventValue(duplicateAuditEvents_));
    event(
        LogLevel::info,
        "logging",
        "logger.started",
        {},
        std::move(loggerFields));
    EventFields statusFields;
    statusFields.reserve(2U);
    statusFields.emplace_back("seed", EventValue(seed));
    statusFields.emplace_back(
        "instance", EventValue(instancePath));
    status(
        "lifecycle",
        "logger_ready",
        std::move(statusFields));
}

RunLogger::~RunLogger() {
    writeEvent(
        LogLevel::info,
        "logging",
        "logger.stopped",
        {},
        {});
    logging_detail::eventCallback.store(
        nullptr, std::memory_order_release);
    logging_detail::statusCallback.store(
        nullptr, std::memory_order_release);
    activeLogger.store(nullptr, std::memory_order_release);

    std::cout.flush();
    std::cerr.flush();
    if (originalStdout_ != nullptr) {
        std::cout.rdbuf(originalStdout_);
    }
    if (originalStderr_ != nullptr) {
        std::cerr.rdbuf(originalStderr_);
    }
    const std::lock_guard<std::mutex> lock(writeMutex_);
    humanStream_.flush();
    eventStream_.flush();
    auditStream_.flush();
}

const std::filesystem::path& RunLogger::directory() const noexcept {
    return directory_;
}

const std::filesystem::path& RunLogger::logFile() const noexcept {
    return logFile_;
}

const std::filesystem::path& RunLogger::eventFile() const noexcept {
    return eventFile_;
}

const std::filesystem::path& RunLogger::auditFile() const noexcept {
    return auditFile_;
}

const std::filesystem::path& RunLogger::statusFile() const noexcept {
    return statusFile_;
}

bool RunLogger::fileLoggingAvailable() const noexcept {
    return humanEnabled_ || eventsEnabled_ ||
        auditEnabled_;
}

void RunLogger::eventTrampoline(
        const LogLevel level,
        const std::string& component,
        const std::string& eventName,
        const std::string& message,
        const EventFields& fields) noexcept {
    RunLogger* logger =
        activeLogger.load(std::memory_order_acquire);
    if (logger != nullptr) {
        logger->writeEvent(
            level, component, eventName, message, fields);
    }
}

void RunLogger::statusTrampoline(
        const std::string& component,
        const std::string& state,
        const EventFields& fields) noexcept {
    RunLogger* logger =
        activeLogger.load(std::memory_order_acquire);
    if (logger != nullptr) {
        logger->writeStatus(component, state, fields);
    }
}

bool RunLogger::openHumanUnlocked() noexcept {
    try {
        std::error_code error;
        if (std::filesystem::exists(logFile_, error) &&
            std::filesystem::file_size(logFile_, error) >=
                segmentBytes_) {
            if (error || !rotateFile(logFile_, backupCount_)) {
                return false;
            }
        }
        humanStream_.open(
            logFile_, std::ios::out | std::ios::app | std::ios::binary);
        if (!humanStream_) {
            return false;
        }
        if (!secureFile(logFile_, error)) {
            humanStream_.close();
            return false;
        }
        humanBytes_ =
            std::filesystem::file_size(logFile_, error);
        return !error;
    } catch (...) {
        return false;
    }
}

bool RunLogger::openEventsUnlocked() noexcept {
    try {
        std::error_code error;
        if (std::filesystem::exists(eventFile_, error) &&
            std::filesystem::file_size(eventFile_, error) >=
                segmentBytes_) {
            if (error || !rotateFile(eventFile_, backupCount_)) {
                return false;
            }
        }
        eventStream_.open(
            eventFile_, std::ios::out | std::ios::app | std::ios::binary);
        if (!eventStream_) {
            return false;
        }
        if (!secureFile(eventFile_, error)) {
            eventStream_.close();
            return false;
        }
        eventBytes_ =
            std::filesystem::file_size(eventFile_, error);
        return !error;
    } catch (...) {
        return false;
    }
}

bool RunLogger::openAuditUnlocked() noexcept {
    try {
        std::error_code error;
        if (std::filesystem::exists(auditFile_, error) &&
            std::filesystem::file_size(auditFile_, error) >=
                segmentBytes_) {
            if (error || !rotateFile(auditFile_, backupCount_)) {
                return false;
            }
        }
        auditStream_.open(
            auditFile_,
            std::ios::out | std::ios::app | std::ios::binary);
        if (!auditStream_) {
            return false;
        }
        if (!secureFile(auditFile_, error)) {
            auditStream_.close();
            return false;
        }
        auditBytes_ =
            std::filesystem::file_size(auditFile_, error);
        return !error;
    } catch (...) {
        return false;
    }
}

bool RunLogger::rotateHumanUnlocked() noexcept {
    humanStream_.flush();
    humanStream_.close();
    if (!rotateFile(logFile_, backupCount_)) {
        humanEnabled_ = false;
        warnOnce("cannot rotate hgasp.log; disabling the human log sink");
        return false;
    }
    humanBytes_ = 0U;
    if (!openHumanUnlocked()) {
        humanEnabled_ = false;
        warnOnce("cannot reopen hgasp.log after rotation");
        return false;
    }
    return true;
}

bool RunLogger::rotateEventsUnlocked() noexcept {
    eventStream_.flush();
    eventStream_.close();
    if (!rotateFile(eventFile_, backupCount_)) {
        eventsEnabled_ = false;
        warnOnce("cannot rotate events.jsonl; disabling structured events");
        return false;
    }
    eventBytes_ = 0U;
    eventsSinceFlush_ = 0U;
    if (!openEventsUnlocked()) {
        eventsEnabled_ = false;
        warnOnce("cannot reopen events.jsonl after rotation");
        return false;
    }
    return true;
}

bool RunLogger::rotateAuditUnlocked() noexcept {
    std::error_code existenceError;
    const bool discardsRetainedHistory =
        backupCount_ == 0U ||
        std::filesystem::exists(
            backupPath(auditFile_, backupCount_),
            existenceError);
    if (existenceError) {
        auditEnabled_ = false;
        warnOnce(
            "cannot inspect compact audit rotation state; "
            "falling back to rotating structured events");
        return false;
    }
    auditHistoryTruncated_ =
        auditHistoryTruncated_ || discardsRetainedHistory;
    auditStream_.flush();
    auditStream_.close();
    if (!rotateFile(auditFile_, backupCount_)) {
        auditEnabled_ = false;
        warnOnce(
            "cannot rotate audit_events.jsonl; "
            "falling back to rotating structured events");
        return false;
    }
    auditBytes_ = 0U;
    if (!openAuditUnlocked()) {
        auditEnabled_ = false;
        warnOnce(
            "cannot reopen audit_events.jsonl after rotation; "
            "falling back to rotating structured events");
        return false;
    }
    if (auditHistoryTruncated_) {
        std::ostringstream marker;
        marker
            << "{\"schema\":1"
            << ",\"ts_utc\":\"" << utcNow() << '"'
            << ",\"component\":\"logging\""
            << ",\"event\":\"audit_history_truncated\""
            << ",\"pid\":" << processId()
            << ",\"fields\":{\"history_truncated\":true}}\n";
        const std::string line = marker.str();
        auditStream_.write(
            line.data(),
            static_cast<std::streamsize>(line.size()));
        auditStream_.flush();
        if (!auditStream_) {
            auditEnabled_ = false;
            warnOnce(
                "cannot persist compact audit truncation marker; "
                "falling back to rotating structured events");
            return false;
        }
        auditBytes_ += line.size();
    }
    return true;
}

std::streamsize RunLogger::writeHuman(
        std::streambuf* console,
        const char* data,
        const std::streamsize size) noexcept {
    if (size <= 0) {
        return 0;
    }
    const std::lock_guard<std::mutex> lock(writeMutex_);
    if (console != nullptr) {
        console->sputn(data, size);
    }
    if (!humanEnabled_) {
        return size;
    }

    try {
        std::streamsize offset = 0;
        while (offset < size && humanEnabled_) {
            if (humanBytes_ >= segmentBytes_ &&
                !rotateHumanUnlocked()) {
                break;
            }
            const std::uintmax_t capacity =
                segmentBytes_ - humanBytes_;
            const std::streamsize chunk =
                std::min<std::streamsize>(
                    size - offset,
                    static_cast<std::streamsize>(capacity));
            humanStream_.write(data + offset, chunk);
            if (!humanStream_) {
                humanEnabled_ = false;
                warnOnce(
                    "write failure in hgasp.log; "
                    "continuing without the human file sink");
                break;
            }
            humanBytes_ += static_cast<std::uintmax_t>(chunk);
            offset += chunk;
        }
    } catch (...) {
        humanEnabled_ = false;
        warnOnce(
            "unexpected file error in hgasp.log; "
            "continuing without the human file sink");
    }
    // Never poison std::cout/std::cerr when an optional log sink fails.
    return size;
}

int RunLogger::syncHuman(std::streambuf* console) noexcept {
    const std::lock_guard<std::mutex> lock(writeMutex_);
    if (console != nullptr) {
        console->pubsync();
    }
    if (humanEnabled_) {
        humanStream_.flush();
        if (!humanStream_) {
            humanEnabled_ = false;
            warnOnce(
                "flush failure in hgasp.log; "
                "continuing without the human file sink");
        }
    }
    return 0;
}

void RunLogger::writeEvent(
        const LogLevel level,
        const std::string& component,
        const std::string& eventName,
        const std::string& message,
        const EventFields& fields) noexcept {
    const bool regularLevel =
        static_cast<int>(level) >=
        static_cast<int>(minimumLevel_);
    const bool compactAudit =
        auditEvent(component, eventName);
    if (!regularLevel && !compactAudit) {
        return;
    }
    const std::lock_guard<std::mutex> lock(writeMutex_);
    if (!eventsEnabled_ && !auditEnabled_) {
        return;
    }
    try {
        const std::uint64_t sequence = ++sequence_;
        const auto monotonicNow =
            std::chrono::steady_clock::now();
        const auto monotonicMilliseconds =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                monotonicNow - monotonicStarted_).count();
        const double cpuSeconds =
            static_cast<double>(std::clock() - cpuStarted_) /
            static_cast<double>(CLOCKS_PER_SEC);

        std::ostringstream output;
        output << "{\"schema\":1"
               << ",\"seq\":" << sequence
               << ",\"ts_utc\":\"" << utcNow() << '"'
               << ",\"mono_ms\":" << monotonicMilliseconds
               << ",\"cpu_s\":" << std::setprecision(12)
               << std::max(0.0, cpuSeconds)
               << ",\"level\":\"" << levelName(level) << '"'
               << ",\"component\":\""
               << jsonEscape(component, kMaximumNameBytes) << '"'
               << ",\"event\":\""
               << jsonEscape(eventName, kMaximumNameBytes) << '"'
               << ",\"pid\":" << processId();
        if (!message.empty()) {
            output << ",\"message\":\""
                   << jsonEscape(message, messageLimit_) << '"';
        }
        output << ",\"fields\":";
        appendFields(output, fields, messageLimit_);
        output << "}\n";
        std::string line = output.str();

        if (line.size() > segmentBytes_) {
            std::ostringstream fallback;
            fallback
                << "{\"schema\":1,\"seq\":" << sequence
                << ",\"ts_utc\":\"" << utcNow() << '"'
                << ",\"level\":\"" << levelName(level) << '"'
                << ",\"component\":\"logging\""
                << ",\"event\":\"event.truncated\""
                << ",\"pid\":" << processId()
                << ",\"fields\":{\"original_bytes\":"
                << line.size() << "}}\n";
            line = fallback.str();
        }
        bool auditPersisted = false;
        if (auditEnabled_ && compactAudit) {
            if (auditBytes_ > 0U &&
                auditBytes_ + line.size() > segmentBytes_ &&
                !rotateAuditUnlocked()) {
                auditEnabled_ = false;
            }
            if (auditEnabled_) {
                auditStream_.write(
                    line.data(),
                    static_cast<std::streamsize>(line.size()));
                auditStream_.flush();
                if (!auditStream_) {
                    auditEnabled_ = false;
                    warnOnce(
                        "write failure in audit_events.jsonl; "
                        "falling back to rotating structured events");
                } else {
                    auditBytes_ += line.size();
                    auditPersisted = true;
                }
            }
        }

        const bool writeRegularEvent =
            (!compactAudit && regularLevel) ||
            (compactAudit &&
             (duplicateAuditEvents_ || !auditPersisted));
        if (eventsEnabled_ && writeRegularEvent) {
            if (eventBytes_ > 0U &&
                eventBytes_ + line.size() > segmentBytes_ &&
                !rotateEventsUnlocked()) {
                eventsEnabled_ = false;
            }
            if (eventsEnabled_) {
                eventStream_.write(
                    line.data(),
                    static_cast<std::streamsize>(line.size()));
                ++eventsSinceFlush_;
                if (level >= LogLevel::warning ||
                    eventsSinceFlush_ >= flushEveryEvents_) {
                    eventStream_.flush();
                    eventsSinceFlush_ = 0U;
                }
                if (!eventStream_) {
                    eventsEnabled_ = false;
                    warnOnce(
                        "write failure in events.jsonl; "
                        "continuing without rotating structured events");
                } else {
                    eventBytes_ += line.size();
                }
            }
        }
    } catch (...) {
        eventsEnabled_ = false;
        auditEnabled_ = false;
        warnOnce(
            "unexpected structured-log file error; "
            "continuing without structured events");
    }
}

void RunLogger::writeStatus(
        const std::string& component,
        const std::string& state,
        const EventFields& fields) noexcept {
    const std::lock_guard<std::mutex> lock(writeMutex_);
    if (!statusEnabled_) {
        return;
    }
    try {
        const auto monotonicNow =
            std::chrono::steady_clock::now();
        const auto monotonicMilliseconds =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                monotonicNow - monotonicStarted_).count();
        const double cpuSeconds =
            static_cast<double>(std::clock() - cpuStarted_) /
            static_cast<double>(CLOCKS_PER_SEC);
        std::ostringstream output;
        output << "{\"schema\":1"
               << ",\"updated_at\":\"" << utcNow() << '"'
               << ",\"mono_ms\":" << monotonicMilliseconds
               << ",\"cpu_s\":" << std::setprecision(12)
               << std::max(0.0, cpuSeconds)
               << ",\"pid\":" << processId()
               << ",\"last_event_seq\":" << sequence_
               << ",\"component\":\""
               << jsonEscape(component, kMaximumNameBytes) << '"'
               << ",\"state\":\""
               << jsonEscape(state, kMaximumNameBytes) << '"'
               << ",\"fields\":";
        appendFields(
            output,
            fields,
            std::min<std::size_t>(messageLimit_, 1024U));
        output << "}\n";
        std::string payload = output.str();
        if (payload.size() > kMaximumStatusBytes) {
            std::ostringstream fallback;
            fallback
                << "{\"schema\":1,\"updated_at\":\"" << utcNow()
                << "\",\"pid\":" << processId()
                << ",\"last_event_seq\":" << sequence_
                << ",\"component\":\"logging\""
                << ",\"state\":\"status_truncated\""
                << ",\"fields\":{\"original_bytes\":"
                << payload.size() << "}}\n";
            payload = fallback.str();
        }

        const std::filesystem::path temporary =
            statusFile_.string() + ".tmp";
        {
            std::ofstream stream(
                temporary,
                std::ios::out | std::ios::trunc | std::ios::binary);
            if (!stream) {
                throw std::system_error(
                    std::make_error_code(std::errc::io_error));
            }
            std::error_code permissionError;
            if (!secureFile(temporary, permissionError)) {
                throw std::system_error(permissionError);
            }
            stream.write(
                payload.data(),
                static_cast<std::streamsize>(payload.size()));
            stream.flush();
            if (!stream) {
                throw std::system_error(
                    std::make_error_code(std::errc::no_space_on_device));
            }
        }
        std::error_code error;
        std::filesystem::rename(temporary, statusFile_, error);
        if (error) {
            std::filesystem::remove(temporary, error);
            throw std::system_error(error);
        }
        if (!secureFile(statusFile_, error)) {
            throw std::system_error(error);
        }
    } catch (...) {
        statusEnabled_ = false;
        warnOnce(
            "cannot atomically update algorithm.status.json; "
            "continuing without status snapshots");
    }
}

void RunLogger::warnOnce(const std::string& message) noexcept {
    if (warningEmitted_) {
        return;
    }
    warningEmitted_ = true;
    if (originalStderr_ == nullptr) {
        return;
    }
    const std::string line =
        "[Log Warning] " +
        boundedString(message, kDefaultMessageLimit) + "\n";
    originalStderr_->sputn(
        line.data(), static_cast<std::streamsize>(line.size()));
    originalStderr_->pubsync();
}

}  // namespace hgasp
