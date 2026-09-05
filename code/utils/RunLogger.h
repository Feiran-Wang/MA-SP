#ifndef HGASP_RUN_LOGGER_H
#define HGASP_RUN_LOGGER_H

#include "SearchComponents.h"
#include "Version.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <memory>
#include <mutex>
#include <optional>
#include <streambuf>
#include <string>
#include <utility>
#include <vector>

namespace hgasp {

class TeeStreamBuffer;

enum class LogLevel {
    trace = 0,
    debug = 1,
    info = 2,
    warning = 3,
    error = 4
};

class EventValue {
public:
    enum class Kind {
        null_value,
        boolean,
        signed_integer,
        unsigned_integer,
        number,
        text
    };

    EventValue() noexcept = default;
    EventValue(std::nullptr_t) noexcept {}
    EventValue(bool value) noexcept
        : kind_(Kind::boolean), boolean_(value) {}
    EventValue(int value) noexcept
        : kind_(Kind::signed_integer),
          signedInteger_(static_cast<std::int64_t>(value)) {}
    EventValue(long value) noexcept
        : kind_(Kind::signed_integer),
          signedInteger_(static_cast<std::int64_t>(value)) {}
    EventValue(long long value) noexcept
        : kind_(Kind::signed_integer),
          signedInteger_(static_cast<std::int64_t>(value)) {}
    EventValue(unsigned int value) noexcept
        : kind_(Kind::unsigned_integer),
          unsignedInteger_(static_cast<std::uint64_t>(value)) {}
    EventValue(unsigned long value) noexcept
        : kind_(Kind::unsigned_integer),
          unsignedInteger_(static_cast<std::uint64_t>(value)) {}
    EventValue(unsigned long long value) noexcept
        : kind_(Kind::unsigned_integer),
          unsignedInteger_(static_cast<std::uint64_t>(value)) {}
    EventValue(double value) noexcept
        : kind_(
              std::isfinite(value)
                  ? Kind::number
                  : Kind::null_value),
          number_(std::isfinite(value) ? value : 0.0) {}
    EventValue(float value) noexcept
        : EventValue(static_cast<double>(value)) {}
    EventValue(const char* value)
        : kind_(
              value == nullptr ? Kind::null_value : Kind::text),
          text_(value == nullptr ? std::string() : std::string(value)) {}
    EventValue(std::string value)
        : kind_(Kind::text), text_(std::move(value)) {}

    Kind kind() const noexcept { return kind_; }
    bool boolean() const noexcept { return boolean_; }
    std::int64_t signedInteger() const noexcept {
        return signedInteger_;
    }
    std::uint64_t unsignedInteger() const noexcept {
        return unsignedInteger_;
    }
    double number() const noexcept { return number_; }
    const std::string& text() const noexcept { return text_; }

private:
    Kind kind_ = Kind::null_value;
    bool boolean_ = false;
    std::int64_t signedInteger_ = 0;
    std::uint64_t unsignedInteger_ = 0;
    double number_ = 0.0;
    std::string text_;
};

struct EventField {
    std::string name;
    EventValue value;

    EventField(std::string fieldName, EventValue fieldValue)
        : name(std::move(fieldName)), value(std::move(fieldValue)) {}
};

using EventFields = std::vector<EventField>;

namespace logging_detail {

using EventCallback = void (*)(
    LogLevel,
    const std::string&,
    const std::string&,
    const std::string&,
    const EventFields&) noexcept;
using StatusCallback = void (*)(
    const std::string&,
    const std::string&,
    const EventFields&) noexcept;

// The engine is also linked into unit-test binaries which do not contain
// RunLogger.cpp. Inline callback slots keep engine instrumentation a safe
// no-op in those binaries while the hgasp executable installs the real sink.
inline std::atomic<EventCallback> eventCallback{nullptr};
inline std::atomic<StatusCallback> statusCallback{nullptr};

}  // namespace logging_detail

class RunLogger {
public:
    RunLogger(const std::string& instancePath, int seed);
    ~RunLogger();

    RunLogger(const RunLogger&) = delete;
    RunLogger& operator=(const RunLogger&) = delete;

    const std::filesystem::path& directory() const noexcept;
    const std::filesystem::path& logFile() const noexcept;
    const std::filesystem::path& eventFile() const noexcept;
    const std::filesystem::path& auditFile() const noexcept;
    const std::filesystem::path& statusFile() const noexcept;
    bool fileLoggingAvailable() const noexcept;

    static void event(
        LogLevel level,
        const std::string& component,
        const std::string& eventName,
        const std::string& message = {},
        EventFields fields = {}) noexcept {
        const auto callback =
            logging_detail::eventCallback.load(std::memory_order_acquire);
        if (callback == nullptr) {
            return;
        }
        callback(level, component, eventName, message, fields);
    }

    static void status(
        const std::string& component,
        const std::string& state,
        EventFields fields = {}) noexcept {
        const auto callback =
            logging_detail::statusCallback.load(std::memory_order_acquire);
        if (callback == nullptr) {
            return;
        }
        callback(component, state, fields);
    }

    static void componentActivation(
            const ComponentActivationSummary& summary) noexcept {
        EventFields fields{
            {"solver_version", kSolverVersion},
            {"algorithm_revision_id", kAlgorithmRevisionId},
            {"search_policy_id", kSearchPolicyId},
        };
        fields.reserve(3U + summary.components.size() * 4U);
        for (const ComponentActivation& component : summary.components) {
            fields.emplace_back(
                component.name + ".configured", component.configured);
            fields.emplace_back(
                component.name + ".effective", component.effective);
            fields.emplace_back(
                component.name + ".activation_count",
                static_cast<unsigned long long>(
                    component.activationCount));
            if (!component.effective) {
                fields.emplace_back(
                    component.name + ".disabled_reason",
                    component.disabledReason);
            }
        }
        event(
            LogLevel::info,
            "lifecycle",
            "component_activation",
            {},
            std::move(fields));
    }

private:
    friend class TeeStreamBuffer;

    static std::filesystem::path chooseDirectory(
        const std::string& instancePath,
        int seed);
    static bool consoleEnabled();
    static void eventTrampoline(
        LogLevel level,
        const std::string& component,
        const std::string& eventName,
        const std::string& message,
        const EventFields& fields) noexcept;
    static void statusTrampoline(
        const std::string& component,
        const std::string& state,
        const EventFields& fields) noexcept;

    void writeEvent(
        LogLevel level,
        const std::string& component,
        const std::string& eventName,
        const std::string& message,
        const EventFields& fields) noexcept;
    void writeStatus(
        const std::string& component,
        const std::string& state,
        const EventFields& fields) noexcept;
    std::streamsize writeHuman(
        std::streambuf* console,
        const char* data,
        std::streamsize size) noexcept;
    int syncHuman(std::streambuf* console) noexcept;
    void warnOnce(const std::string& message) noexcept;
    bool rotateHumanUnlocked() noexcept;
    bool rotateEventsUnlocked() noexcept;
    bool rotateAuditUnlocked() noexcept;
    bool openHumanUnlocked() noexcept;
    bool openEventsUnlocked() noexcept;
    bool openAuditUnlocked() noexcept;

    std::filesystem::path directory_;
    std::filesystem::path logFile_;
    std::filesystem::path eventFile_;
    std::filesystem::path auditFile_;
    std::filesystem::path statusFile_;
    std::ofstream humanStream_;
    std::ofstream eventStream_;
    std::ofstream auditStream_;
    std::mutex writeMutex_;
    std::streambuf* originalStdout_ = nullptr;
    std::streambuf* originalStderr_ = nullptr;
    std::unique_ptr<TeeStreamBuffer> stdoutBuffer_;
    std::unique_ptr<TeeStreamBuffer> stderrBuffer_;
    std::chrono::steady_clock::time_point monotonicStarted_;
    std::clock_t cpuStarted_ = 0;
    std::uint64_t sequence_ = 0;
    std::uintmax_t segmentBytes_ = 8U * 1024U * 1024U;
    unsigned int backupCount_ = 2;
    std::size_t messageLimit_ = 4096;
    std::uint64_t flushEveryEvents_ = 64U;
    LogLevel minimumLevel_ = LogLevel::info;
    std::uintmax_t humanBytes_ = 0;
    std::uintmax_t eventBytes_ = 0;
    std::uintmax_t auditBytes_ = 0;
    std::uint64_t eventsSinceFlush_ = 0U;
    bool duplicateAuditEvents_ = false;
    bool humanEnabled_ = true;
    bool eventsEnabled_ = true;
    bool auditEnabled_ = true;
    bool statusEnabled_ = true;
    bool warningEmitted_ = false;
    bool auditHistoryTruncated_ = false;
};

}  // namespace hgasp

#endif
