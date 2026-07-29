#include "RunLogger.h"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#if !defined(_WIN32)
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace {

int fail(const std::string& message) {
    std::cerr << "run_logger_test: " << message << '\n';
    return 1;
}

std::string readFile(const std::filesystem::path& path) {
    std::ifstream stream(path, std::ios::binary);
    std::ostringstream content;
    content << stream.rdbuf();
    return content.str();
}

bool boundedSegments(
        const std::filesystem::path& base,
        const std::uintmax_t limit,
        const unsigned int backups) {
    for (unsigned int index = 0; index <= backups; ++index) {
        const std::filesystem::path path =
            index == 0U
                ? base
                : std::filesystem::path(
                      base.string() + "." + std::to_string(index));
        if (std::filesystem::exists(path) &&
            std::filesystem::file_size(path) > limit) {
            return false;
        }
    }
    return true;
}

}  // namespace

int main() {
    constexpr std::uintmax_t segmentBytes = 64U * 1024U;
    const std::filesystem::path root =
        std::filesystem::temp_directory_path() /
        ("hgasp-run-logger-test-" +
#if defined(_WIN32)
         std::to_string(_getpid())
#else
         std::to_string(getpid())
#endif
        );
    std::error_code error;
    std::filesystem::remove_all(root, error);

#if defined(_WIN32)
    _putenv_s("HGASP_LOG_DIR", root.string().c_str());
    _putenv_s("HGASP_LOG_CONSOLE", "0");
    _putenv_s("HGASP_LOG_SEGMENT_BYTES", "65536");
    _putenv_s("HGASP_LOG_BACKUPS", "2");
    _putenv_s("HGASP_LOG_LEVEL", "trace");
#else
    setenv("HGASP_LOG_DIR", root.string().c_str(), 1);
    setenv("HGASP_LOG_CONSOLE", "0", 1);
    setenv("HGASP_LOG_SEGMENT_BYTES", "65536", 1);
    setenv("HGASP_LOG_BACKUPS", "2", 1);
    setenv("HGASP_LOG_LEVEL", "trace", 1);
#endif

    {
        hgasp::RunLogger logger("synthetic.instance", 7);
        const std::string payload(2048U, 'x');
        for (int index = 0; index < 600; ++index) {
            hgasp::RunLogger::event(
                hgasp::LogLevel::debug,
                "test",
                "flood",
                payload,
                {{"index", index}});
        }
        hgasp::RunLogger::event(
            hgasp::LogLevel::warning,
            "test",
            "escaping",
            "quote=\" newline=\n",
            {
                {"finite", 1.25},
                {"nonfinite",
                 std::numeric_limits<double>::infinity()},
                {"text", "a\\b\nc"},
                {"access_token", "must-not-be-persisted"}
            });
        std::cout << std::string(200000U, 'h');
        std::cout.flush();

        hgasp::EventFields statusFields;
        for (int index = 0; index < 64; ++index) {
            statusFields.emplace_back(
                "field_" + std::to_string(index),
                std::string(4096U, 's'));
        }
        hgasp::RunLogger::status(
            "test", "large_status", std::move(statusFields));
    }

    const std::filesystem::path human = root / "hgasp.log";
    const std::filesystem::path events = root / "events.jsonl";
    const std::filesystem::path status =
        root / "algorithm.status.json";
    if (!boundedSegments(human, segmentBytes, 2U)) {
        return fail("human log exceeded the configured segment limit");
    }
    if (!boundedSegments(events, segmentBytes, 2U)) {
        return fail("event log exceeded the configured segment limit");
    }
    if (!std::filesystem::is_regular_file(status) ||
        std::filesystem::file_size(status) > 64U * 1024U) {
        return fail("atomic status file is absent or unbounded");
    }

    const std::string currentEvents = readFile(events);
    if (currentEvents.find("\"event\":\"escaping\"") ==
            std::string::npos ||
        currentEvents.find("quote=\\\" newline=\\n") ==
            std::string::npos ||
        currentEvents.find("\"nonfinite\":null") ==
            std::string::npos ||
        currentEvents.find("must-not-be-persisted") !=
            std::string::npos ||
        currentEvents.find("\"access_token\":\"[REDACTED]\"") ==
            std::string::npos) {
        return fail("structured JSON escaping or finite-number handling failed");
    }
    const std::string statusText = readFile(status);
    if (statusText.empty() || statusText.front() != '{' ||
        statusText.back() != '\n') {
        return fail("status snapshot is not a complete JSON line");
    }

#if !defined(_WIN32)
    struct stat directoryStat {};
    struct stat eventStat {};
    if (stat(root.c_str(), &directoryStat) != 0 ||
        (directoryStat.st_mode & 0777) != 0700) {
        return fail("log directory permissions are not 0700");
    }
    if (stat(events.c_str(), &eventStat) != 0 ||
        (eventStat.st_mode & 0777) != 0600) {
        return fail("log file permissions are not 0600");
    }
#endif

    // A log directory failure must degrade to the original console stream
    // without throwing or poisoning std::cout.
    const std::filesystem::path blocker = root / "not-a-directory";
    {
        std::ofstream stream(blocker);
        stream << "block";
    }
    const std::filesystem::path invalid = blocker / "child";
#if defined(_WIN32)
    _putenv_s("HGASP_LOG_DIR", invalid.string().c_str());
#else
    setenv("HGASP_LOG_DIR", invalid.string().c_str(), 1);
#endif
    {
        hgasp::RunLogger degraded("synthetic.instance", 8);
        hgasp::RunLogger::event(
            hgasp::LogLevel::error,
            "test",
            "degraded");
        std::cout << "degraded logger remains usable\n";
    }

    std::filesystem::remove_all(root, error);
    return 0;
}
