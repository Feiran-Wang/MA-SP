#include "RunLogger.h"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

#if defined(_WIN32)
#include <process.h>
#else
#include <unistd.h>
#endif

namespace hgasp {

class TeeStreamBuffer final : public std::streambuf {
public:
    TeeStreamBuffer(
        std::streambuf* console,
        std::streambuf* file,
        std::mutex& writeMutex)
        : console_(console), file_(file), writeMutex_(writeMutex) {}

protected:
    int overflow(int character) override {
        if (traits_type::eq_int_type(character, traits_type::eof())) {
            return sync() == 0 ? traits_type::not_eof(character)
                               : traits_type::eof();
        }

        const std::lock_guard<std::mutex> lock(writeMutex_);
        const char value = traits_type::to_char_type(character);
        const bool consoleOk =
            console_ == nullptr ||
            !traits_type::eq_int_type(
                console_->sputc(value),
                traits_type::eof());
        const bool fileOk =
            !traits_type::eq_int_type(file_->sputc(value), traits_type::eof());
        return consoleOk && fileOk ? character : traits_type::eof();
    }

    std::streamsize xsputn(
        const char* data,
        std::streamsize size) override {
        const std::lock_guard<std::mutex> lock(writeMutex_);
        const std::streamsize consoleWritten =
            console_ == nullptr ? size : console_->sputn(data, size);
        const std::streamsize fileWritten = file_->sputn(data, size);
        return std::min(consoleWritten, fileWritten);
    }

    int sync() override {
        const std::lock_guard<std::mutex> lock(writeMutex_);
        const bool consoleOk = console_ == nullptr || console_->pubsync() == 0;
        const bool fileOk = file_->pubsync() == 0;
        return consoleOk && fileOk ? 0 : -1;
    }

private:
    std::streambuf* console_;
    std::streambuf* file_;
    std::mutex& writeMutex_;
};

namespace {

std::string sanitizeName(std::string value) {
    for (char& character : value) {
        const unsigned char byte = static_cast<unsigned char>(character);
        if (!std::isalnum(byte) && character != '-' && character != '_') {
            character = '_';
        }
    }
    if (value.empty()) {
        return "instance";
    }
    return value;
}

std::string runTimestamp() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t time = std::chrono::system_clock::to_time_t(now);
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

long processId() {
#if defined(_WIN32)
    return static_cast<long>(_getpid());
#else
    return static_cast<long>(getpid());
#endif
}

}  // namespace

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
    throw std::invalid_argument("HGASP_LOG_CONSOLE must be 0 or 1");
}

RunLogger::RunLogger(
    const std::string& instancePath,
    const int seed)
    : directory_(chooseDirectory(instancePath, seed)),
      logFile_(directory_ / "hgasp.log") {
    std::error_code error;
    std::filesystem::create_directories(directory_, error);
    if (error) {
        throw std::runtime_error(
            "cannot create log directory " + directory_.string() +
            ": " + error.message());
    }

    stream_.open(logFile_, std::ios::out | std::ios::app);
    if (!stream_) {
        throw std::runtime_error(
            "cannot open HGA-SP log file " + logFile_.string());
    }

    originalStdout_ = std::cout.rdbuf();
    originalStderr_ = std::cerr.rdbuf();
    const bool useConsole = consoleEnabled();
    stdoutBuffer_ = std::make_unique<TeeStreamBuffer>(
        useConsole ? originalStdout_ : nullptr,
        stream_.rdbuf(),
        writeMutex_);
    stderrBuffer_ = std::make_unique<TeeStreamBuffer>(
        useConsole ? originalStderr_ : nullptr,
        stream_.rdbuf(),
        writeMutex_);
    std::cout.rdbuf(stdoutBuffer_.get());
    std::cerr.rdbuf(stderrBuffer_.get());

    std::cout << "[Log] run_directory=" << directory_.string() << '\n';
}

RunLogger::~RunLogger() {
    std::cout.flush();
    std::cerr.flush();
    if (originalStdout_ != nullptr) {
        std::cout.rdbuf(originalStdout_);
    }
    if (originalStderr_ != nullptr) {
        std::cerr.rdbuf(originalStderr_);
    }
    stream_.flush();
}

const std::filesystem::path& RunLogger::directory() const noexcept {
    return directory_;
}

const std::filesystem::path& RunLogger::logFile() const noexcept {
    return logFile_;
}

}  // namespace hgasp
