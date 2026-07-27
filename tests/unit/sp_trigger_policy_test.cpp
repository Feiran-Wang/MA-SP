#include "AlgorithmOrchestrator.h"
#include "read_data.h"

#include <cstdlib>
#include <iostream>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>

std::mt19937 global_rng;

namespace {

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

class EnvironmentGuard {
public:
    explicit EnvironmentGuard(const char* name) : name_(name) {
        const char* value = std::getenv(name);
        if (value != nullptr) {
            originalValue_ = value;
        }
    }

    ~EnvironmentGuard() {
        if (originalValue_.has_value()) {
            setenv(name_.c_str(), originalValue_->c_str(), 1);
        } else {
            unsetenv(name_.c_str());
        }
    }

private:
    std::string name_;
    std::optional<std::string> originalValue_;
};

template <typename Callable>
void requireInvalidArgument(
    Callable&& callable,
    const std::string& message) {
    bool rejected = false;
    try {
        callable();
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    require(rejected, message);
}

void testDefaultTriggerSchedule() {
    require(
        !AlgorithmOrchestrator::shouldTriggerSp(50000, 50001, 0),
        "default schedule triggered before its frequency");
    require(
        AlgorithmOrchestrator::shouldTriggerSp(50001, 50001, 0),
        "default schedule did not trigger at its frequency");
    require(
        AlgorithmOrchestrator::shouldTriggerSp(100002, 50001, 0),
        "default schedule did not repeat after one frequency");
}

void testOffsetTriggerSchedule() {
    require(
        !AlgorithmOrchestrator::shouldTriggerSp(50001, 50001, 137),
        "offset schedule triggered at the unshifted iteration");
    require(
        !AlgorithmOrchestrator::shouldTriggerSp(50137, 50001, 137),
        "offset schedule triggered before frequency plus offset");
    require(
        AlgorithmOrchestrator::shouldTriggerSp(50138, 50001, 137),
        "offset schedule missed its first shifted trigger");
    require(
        AlgorithmOrchestrator::shouldTriggerSp(100139, 50001, 137),
        "offset schedule did not retain its frequency");
    require(
        !AlgorithmOrchestrator::shouldTriggerSp(100140, 50001, 137),
        "offset schedule triggered outside its cadence");
}

void testInvalidTriggerSchedule() {
    requireInvalidArgument(
        [] { AlgorithmOrchestrator::shouldTriggerSp(1, 0, 0); },
        "zero trigger frequency was accepted");
    requireInvalidArgument(
        [] { AlgorithmOrchestrator::shouldTriggerSp(1, 10, -1); },
        "negative trigger offset was accepted");
    requireInvalidArgument(
        [] { AlgorithmOrchestrator::shouldTriggerSp(1, 10, 10); },
        "trigger offset equal to the frequency was accepted");
}

void testEnvironmentOverride() {
    EnvironmentGuard frequencyGuard("HGASP_SP_TRIGGER_FREQUENCY");
    EnvironmentGuard offsetGuard("HGASP_SP_TRIGGER_OFFSET");

    unsetenv("HGASP_SP_TRIGGER_FREQUENCY");
    unsetenv("HGASP_SP_TRIGGER_OFFSET");
    read_data defaults;
    defaults.applyEnvironmentOverrides();
    require(defaults.spTriggerOffset == 0, "default trigger offset changed");

    setenv("HGASP_SP_TRIGGER_FREQUENCY", "100", 1);
    setenv("HGASP_SP_TRIGGER_OFFSET", "17", 1);
    read_data configured;
    configured.applyEnvironmentOverrides();
    require(
        configured.spTriggerFrequency == 100,
        "trigger frequency override was not applied");
    require(
        configured.spTriggerOffset == 17,
        "trigger offset override was not applied");

    setenv("HGASP_SP_TRIGGER_OFFSET", "100", 1);
    requireInvalidArgument(
        [] {
            read_data invalid;
            invalid.applyEnvironmentOverrides();
        },
        "trigger offset equal to the configured frequency was accepted");
}

}  // namespace

int main() {
    try {
        testDefaultTriggerSchedule();
        testOffsetTriggerSchedule();
        testInvalidTriggerSchedule();
        testEnvironmentOverride();
        std::cout << "SP trigger policy tests passed" << std::endl;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "SP trigger policy test failed: " << error.what()
                  << std::endl;
        return 1;
    }
}
