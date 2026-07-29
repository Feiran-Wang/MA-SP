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

void testAttemptTailLatchesEveryOutcome() {
    SearchAttemptScheduler scheduler;
    int iteration = 7;
    int nonImprovingIterations = 3;

    // A feasible child, a rejected child and a zero-child EAX batch all call
    // the same completion operation.  The scheduler deliberately has no
    // outcome parameter, so none of those paths can suppress the event tail.
    scheduler.completeAttempt(
        iteration, 10, 0);
    scheduler.settleNonImprovement(
        nonImprovingIterations, false);
    scheduler.completeAttempt(
        iteration, 10, 0);
    scheduler.settleNonImprovement(
        nonImprovingIterations, false);
    require(
        !scheduler.hasPendingSpTrigger(),
        "SP was latched before the scheduled attempt");

    scheduler.completeAttempt(
        iteration, 10, 0);
    scheduler.settleNonImprovement(
        nonImprovingIterations, false);
    require(iteration == 10, "attempt tail did not advance iteration");
    require(
        nonImprovingIterations == 6,
        "attempt tail did not advance the non-improvement counter");
    require(
        scheduler.hasPendingSpTrigger(),
        "scheduled SP was lost at the common attempt tail");

    // A pending request survives later attempts until the orchestrator reaches
    // its safe execution point and explicitly consumes it.
    scheduler.completeAttempt(
        iteration, 10, 0);
    scheduler.settleNonImprovement(
        nonImprovingIterations, false);
    require(
        scheduler.hasPendingSpTrigger(),
        "an unconsumed SP request was not latched");
    require(
        scheduler.consumeSpTrigger(),
        "the latched SP request could not be consumed");
    require(
        !scheduler.consumeSpTrigger(),
        "one scheduled SP request was consumed more than once");
}

void testAttemptTailHonorsOffset() {
    SearchAttemptScheduler scheduler;
    int iteration = 11;
    int nonImprovingIterations = 0;

    scheduler.completeAttempt(
        iteration, 10, 3);
    scheduler.settleNonImprovement(
        nonImprovingIterations, false);
    require(
        !scheduler.hasPendingSpTrigger(),
        "offset trigger fired one attempt early");
    scheduler.completeAttempt(
        iteration, 10, 3);
    scheduler.settleNonImprovement(
        nonImprovingIterations, false);
    require(
        scheduler.consumeSpTrigger(),
        "offset trigger was not latched at frequency plus offset");
}

void testAttemptTailSettlesOneUnifiedImprovementCounter() {
    SearchAttemptScheduler scheduler;
    int nonImprovingIterations = 8;

    // The caller performs this comparison only after the final incumbent for
    // the attempt is known. It therefore makes no difference whether HGA or
    // a scheduled SP call produced the improvement.
    scheduler.settleNonImprovement(
        nonImprovingIterations, true);
    require(
        nonImprovingIterations == 0,
        "a restart-best improvement did not reset stagnation");

    scheduler.settleNonImprovement(
        nonImprovingIterations, false);
    require(
        nonImprovingIterations == 1,
        "a non-improving attempt was not counted exactly once");
}

void testSpOutcomePolicyIsFailClosed() {
    require(
        isSupportedSpTerminalStatus(SpStatus::optimal),
        "OPTIMAL should be a supported SP outcome");
    require(
        isSupportedSpTerminalStatus(SpStatus::time_limit) &&
            isSupportedSpTerminalStatus(SpStatus::work_limit) &&
            isSupportedSpTerminalStatus(SpStatus::infeasible),
        "the explicit nonfatal SP outcome set changed");
    require(
        mayContinueWithoutSpIncumbent(SpStatus::time_limit) &&
            mayContinueWithoutSpIncumbent(SpStatus::work_limit) &&
            mayContinueWithoutSpIncumbent(SpStatus::infeasible),
        "the no-incumbent whitelist changed");
    require(
        !mayContinueWithoutSpIncumbent(SpStatus::optimal),
        "OPTIMAL without an incumbent must be fatal");

    const SpStatus fatalStatuses[] = {
        SpStatus::not_run,
        SpStatus::invalid_input,
        SpStatus::error,
        SpStatus::numeric,
        SpStatus::memory_limit,
        SpStatus::interrupted,
        SpStatus::unbounded,
        SpStatus::inf_or_unbounded,
        SpStatus::cutoff,
        SpStatus::iteration_limit,
        SpStatus::suboptimal,
        SpStatus::node_limit,
        SpStatus::solution_limit,
        SpStatus::user_objective_limit,
    };
    for (const SpStatus status : fatalStatuses) {
        require(
            !isSupportedSpTerminalStatus(status),
            "an unexpected SP status was not classified as fatal");
        require(
            !mayContinueWithoutSpIncumbent(status),
            "an unexpected no-incumbent status was allowed to continue");
    }
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
        testAttemptTailLatchesEveryOutcome();
        testAttemptTailHonorsOffset();
        testAttemptTailSettlesOneUnifiedImprovementCounter();
        testSpOutcomePolicyIsFailClosed();
        testEnvironmentOverride();
        std::cout << "SP trigger policy tests passed" << std::endl;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "SP trigger policy test failed: " << error.what()
                  << std::endl;
        return 1;
    }
}
