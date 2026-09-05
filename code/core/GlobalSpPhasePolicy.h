#ifndef HGASP_CORE_GLOBAL_SP_PHASE_POLICY_H_
#define HGASP_CORE_GLOBAL_SP_PHASE_POLICY_H_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace hgasp {

enum class GlobalSpPhaseAction {
    None,
    EarlyGlobal,
    PreReductionGlobal1,
    Reduction1,
    PreReductionGlobal2,
    Reduction2,
    FinalGlobal,
};

const char* toString(GlobalSpPhaseAction action) noexcept;

enum class GlobalSpPhaseCompletion {
    Completed,
    ValidNoImprovement,
    IncompleteMaster,
    DeferredBudget,
    SkippedDisabled,
    SkippedNoCheckpointFeedback,
    FatalError,
};

const char* toString(GlobalSpPhaseCompletion completion) noexcept;

struct GlobalSpCheckpointFeedback {
    bool checkpointObserved = false;
    bool dualFeedbackAvailable = false;
    bool incumbentImproved = false;
    std::size_t novelFounderCandidates = 0U;
};

// A missing/failed first checkpoint is not evidence that the second call is
// useless. Only a completed 50% observation with no usable feedback at all
// suppresses the 75% exploratory Global-SP call.
inline bool shouldExecuteSecondExploratoryGlobal(
        const GlobalSpCheckpointFeedback& feedback) noexcept {
    return !feedback.checkpointObserved ||
        feedback.dualFeedbackAvailable ||
        feedback.incumbentImproved ||
        feedback.novelFounderCandidates != 0U;
}

struct GlobalSpPhaseConfig {
    // Zero inherits iterLimit. A positive value is an explicit policy
    // horizon and may exceed a deliberately truncated development run; this
    // keeps 30k challenger-only observations ahead of the frozen 45k phase.
    std::uint64_t configuredHorizonAttempts = 0U;
    bool earlyGlobalEnabled = false;
    double earlyGlobalFraction = 0.25;
    double preReductionGlobal1Fraction = 0.45;
    double populationReduction1Fraction = 0.50;
    double preReductionGlobal2Fraction = 0.70;
    double populationReduction2Fraction = 0.75;
    double minimumAbsorptionFraction = 0.05;
    bool preReductionGlobalEnabled = true;
    double lpTimeLimitSeconds = 10.0;
    double mipTimeLimitSeconds = 15.0;
    std::size_t maximumInjections = 3U;
    double earlyGlobalBaseMipSeconds = 15.0;
    double earlyGlobalMaximumMipSeconds = 15.0;
    double firstGlobalBaseMipSeconds = 15.0;
    double firstGlobalMaximumMipSeconds = 15.0;
    double secondGlobalBaseMipSeconds = 15.0;
    double secondGlobalMaximumMipSeconds = 15.0;
    bool scheduledFinalGlobalEnabled = false;
    // 0.95 maps the canonical 100k horizon to the requested last 5000
    // attempts and scales proportionally in shorter development runs.
    double finalGlobalFraction = 0.95;
    double finalGlobalBaseMipSeconds = 30.0;
    double finalGlobalMaximumMipSeconds = 360.0;
    std::size_t finalGlobalMaximumInjections = 3U;
};

struct GlobalSpPhaseDecision {
    GlobalSpPhaseAction action = GlobalSpPhaseAction::None;
    bool claimed = false;
    bool executionRequired = false;
    bool preReductionGlobal = false;
    std::uint64_t thresholdAttempt = 0U;
    std::uint64_t phaseHorizonAttempts = 0U;
    double lpTimeLimitSeconds = 0.0;
    double mipTimeLimitSeconds = 0.0;
    double improvementExtensionGateSeconds = 0.0;
    std::size_t maximumInjections = 0U;
    std::string reason = "not_due";
};

// Stateful, run-level phase latch. claimNext() returns at most one action and
// latches it until complete() is called, which makes Global-before-reduction
// ordering explicit even when the attempt counter jumps over several phases.
// Search restarts preserve the latch. Final Global has a distinct action and
// never consumes any earlier scheduled Global phase.
class GlobalSpPhasePolicy {
public:
    GlobalSpPhasePolicy(
        std::uint64_t iterationLimit,
        GlobalSpPhaseConfig config = {});

    // When scheduledPhasesAvailable is false, only a requested Final Global
    // can be claimed; static-dependent phases remain explicitly incomplete.
    GlobalSpPhaseDecision claimNext(
        std::uint64_t currentAttempt,
        bool finalRequested = false,
        bool scheduledPhasesAvailable = true);

    void complete(
        GlobalSpPhaseAction action,
        GlobalSpPhaseCompletion completion);

    // An HGA population restart is not a new run. This call intentionally
    // preserves all completed and in-flight phase state.
    void observeSearchRestart() noexcept {
        ++restartCount_;
    }

    bool completed(GlobalSpPhaseAction action) const noexcept;
    std::uint64_t threshold(
        GlobalSpPhaseAction action) const;

    std::uint64_t phaseHorizonAttempts() const noexcept {
        return phaseHorizonAttempts_;
    }

    std::optional<GlobalSpPhaseAction> inFlight() const noexcept {
        return inFlight_;
    }

    bool fatal() const noexcept {
        return fatal_;
    }

    std::size_t restartCount() const noexcept {
        return restartCount_;
    }

    const std::vector<GlobalSpPhaseAction>& completedActions()
            const noexcept {
        return completedActions_;
    }

private:
    static void validateConfig(const GlobalSpPhaseConfig& config);
    static std::uint64_t phaseThreshold(
        std::uint64_t horizon,
        double fraction);
    std::size_t phaseIndex(GlobalSpPhaseAction action) const;
    GlobalSpPhaseDecision claim(
        GlobalSpPhaseAction action,
        std::uint64_t thresholdAttempt,
        bool executionRequired,
        const char* reason);

    GlobalSpPhaseConfig config_;
    std::uint64_t phaseHorizonAttempts_ = 0U;
    std::uint64_t thresholds_[6] = {};
    bool completed_[6] = {};
    std::optional<GlobalSpPhaseAction> inFlight_;
    bool fatal_ = false;
    std::uint64_t maximumObservedAttempt_ = 0U;
    std::size_t restartCount_ = 0U;
    std::vector<GlobalSpPhaseAction> completedActions_;
};

// V3.3 inherits budgets stated at 500 customers. For smaller instances, scale all
// four Global-SP windows linearly with problem size, while retaining a 20%
// floor so startup/model overhead does not collapse a call into a token
// gesture. Larger instances keep the audited 500-customer hard caps.
inline double v33GlobalSpBudgetScale(
        const std::size_t customerCount,
        const std::size_t referenceCustomerCount = 500U) {
    if (customerCount == 0U || referenceCustomerCount == 0U) {
        throw std::invalid_argument(
            "Global-SP budget scaling requires positive customer counts");
    }
    const double raw = static_cast<double>(customerCount) /
        static_cast<double>(referenceCustomerCount);
    return std::max(0.20, std::min(1.0, raw));
}

}  // namespace hgasp

#endif  // HGASP_CORE_GLOBAL_SP_PHASE_POLICY_H_
