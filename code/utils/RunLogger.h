#ifndef HGASP_RUN_LOGGER_H
#define HGASP_RUN_LOGGER_H

#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <streambuf>
#include <string>

namespace hgasp {

class TeeStreamBuffer;

class RunLogger {
public:
    RunLogger(const std::string& instancePath, int seed);
    ~RunLogger();

    RunLogger(const RunLogger&) = delete;
    RunLogger& operator=(const RunLogger&) = delete;

    const std::filesystem::path& directory() const noexcept;
    const std::filesystem::path& logFile() const noexcept;

private:
    static std::filesystem::path chooseDirectory(
        const std::string& instancePath,
        int seed);
    static bool consoleEnabled();

    std::filesystem::path directory_;
    std::filesystem::path logFile_;
    std::ofstream stream_;
    std::mutex writeMutex_;
    std::streambuf* originalStdout_ = nullptr;
    std::streambuf* originalStderr_ = nullptr;
    std::unique_ptr<TeeStreamBuffer> stdoutBuffer_;
    std::unique_ptr<TeeStreamBuffer> stderrBuffer_;
};

}  // namespace hgasp

#endif
