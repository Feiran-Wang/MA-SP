#ifndef HGASP_CORE_SP_EXECUTION_PLAN_H_
#define HGASP_CORE_SP_EXECUTION_PLAN_H_

#include <cmath>
#include <stdexcept>
#include <string_view>

namespace hgasp {

enum class SearchTerminationReason {
    Unset,
    IterationLimit,
    ProcessCpuLimit,
    ReferenceTarget,
    RigorousLpNoImprovement,
    HeuristicMipNoImprovement,
    PostFinalGlobalStagnation,
};

inline std::string_view toString(
        const SearchTerminationReason reason) noexcept {
    switch (reason) {
    case SearchTerminationReason::IterationLimit:
        return "iteration_limit";
    case SearchTerminationReason::ProcessCpuLimit:
        return "process_cpu_limit";
    case SearchTerminationReason::ReferenceTarget:
        return "reference_target";
    case SearchTerminationReason::RigorousLpNoImprovement:
        return "rigorous_lp_no_improvement";
    case SearchTerminationReason::HeuristicMipNoImprovement:
        return "heuristic_mip_no_improvement";
    case SearchTerminationReason::PostFinalGlobalStagnation:
        return "post_final_global_stagnation";
    case SearchTerminationReason::Unset:
        return "unset";
    }
    return "unset";
}

enum class EarlyStopEvidenceClass {
    NotApplicable,
    RigorousLpNoImprovement,
    HeuristicMipNoImprovement,
    PostFinalGlobalStagnation,
};

inline std::string_view toString(
        const EarlyStopEvidenceClass value) noexcept {
    switch (value) {
    case EarlyStopEvidenceClass::NotApplicable:
        return "not_applicable";
    case EarlyStopEvidenceClass::RigorousLpNoImprovement:
        return "rigorous_lp_no_improvement";
    case EarlyStopEvidenceClass::HeuristicMipNoImprovement:
        return "heuristic_mip_no_improvement";
    case EarlyStopEvidenceClass::PostFinalGlobalStagnation:
        return "post_final_global_stagnation";
    }
    return "not_applicable";
}

struct SearchTerminationRecord {
    int requestedIterations = 0;
    int completedIterations = 0;
    SearchTerminationReason reason = SearchTerminationReason::Unset;
    EarlyStopEvidenceClass evidenceClass =
        EarlyStopEvidenceClass::NotApplicable;
    bool lpRigorousNoImprovement = false;
    bool mipAttempted = false;
    bool mipImproved = false;

    bool isSet() const noexcept {
        return reason != SearchTerminationReason::Unset;
    }

    bool internallyConsistent() const noexcept {
        if (!isSet() || requestedIterations < 0 ||
            completedIterations < 0 ||
            completedIterations > requestedIterations) {
            return false;
        }
        switch (reason) {
        case SearchTerminationReason::RigorousLpNoImprovement:
            return evidenceClass ==
                       EarlyStopEvidenceClass::RigorousLpNoImprovement &&
                lpRigorousNoImprovement && !mipAttempted && !mipImproved;
        case SearchTerminationReason::HeuristicMipNoImprovement:
            return evidenceClass ==
                       EarlyStopEvidenceClass::HeuristicMipNoImprovement &&
                mipAttempted && !lpRigorousNoImprovement && !mipImproved;
        case SearchTerminationReason::PostFinalGlobalStagnation:
            return evidenceClass ==
                       EarlyStopEvidenceClass::PostFinalGlobalStagnation &&
                !lpRigorousNoImprovement && mipAttempted && mipImproved;
        case SearchTerminationReason::IterationLimit:
        case SearchTerminationReason::ProcessCpuLimit:
        case SearchTerminationReason::ReferenceTarget:
            return evidenceClass ==
                       EarlyStopEvidenceClass::NotApplicable &&
                !lpRigorousNoImprovement && !mipAttempted && !mipImproved;
        case SearchTerminationReason::Unset:
            return false;
        }
        return false;
    }
};

inline SearchTerminationRecord makeSearchTerminationRecord(
        const int requestedIterations,
        const int completedIterations,
        const SearchTerminationReason reason,
        const EarlyStopEvidenceClass evidenceClass =
            EarlyStopEvidenceClass::NotApplicable,
        const bool lpRigorousNoImprovement = false,
        const bool mipAttempted = false,
        const bool mipImproved = false) {
    SearchTerminationRecord record{
        requestedIterations,
        completedIterations,
        reason,
        evidenceClass,
        lpRigorousNoImprovement,
        mipAttempted,
        mipImproved,
    };
    if (!record.internallyConsistent()) {
        throw std::invalid_argument(
            "search termination record is contradictory");
    }
    return record;
}

inline SearchTerminationRecord classifyCompletedFinalGlobalStop(
        const int requestedIterations,
        const int completedIterations,
        const bool rigorousLpNoImprovement,
        const bool mipAttempted,
        const bool mipImproved) {
    if (mipImproved) {
        return makeSearchTerminationRecord(
            requestedIterations,
            completedIterations,
            SearchTerminationReason::PostFinalGlobalStagnation,
            EarlyStopEvidenceClass::PostFinalGlobalStagnation,
            false,
            mipAttempted,
            true);
    }
    if (rigorousLpNoImprovement) {
        return makeSearchTerminationRecord(
            requestedIterations,
            completedIterations,
            SearchTerminationReason::RigorousLpNoImprovement,
            EarlyStopEvidenceClass::RigorousLpNoImprovement,
            true,
            false,
            false);
    }
    return makeSearchTerminationRecord(
        requestedIterations,
        completedIterations,
        SearchTerminationReason::HeuristicMipNoImprovement,
        EarlyStopEvidenceClass::HeuristicMipNoImprovement,
        false,
        mipAttempted,
        false);
}

struct SpExecutionPlan {
    bool runLp = false;
    bool runMip = false;
    bool staged = false;
    bool combined = false;
};

inline SpExecutionPlan makeSpExecutionPlan(
        const bool lpConsumerEnabled,
        const bool mipRequired) noexcept {
    SpExecutionPlan plan;
    plan.runLp = lpConsumerEnabled;
    plan.runMip = mipRequired;
    // A Global P2 LP consumer must observe the exact same immutable snapshot
    // that is later offered to the integer RMP. Keeping the stages explicit
    // allows a rigorous LP-bound gate to suppress only provably unnecessary
    // MIP work. The legacy combined path is intentionally no longer selected.
    plan.staged = plan.runLp && plan.runMip;
    plan.combined = false;
    return plan;
}

struct P2MipGateDecision {
    bool runMip = true;
    bool rigorousNoImprovement = false;
    const char* reason = "lp_gate_unavailable";
};

inline bool isNonfatalBudgetedP2LpOnlyCompletion(
        const bool hardMandatory,
        const bool lpFeedbackAvailable,
        const bool mipAttempted,
        const bool mipSkippedByBudget) noexcept {
    return !hardMandatory && lpFeedbackAvailable &&
        !mipAttempted && mipSkippedByBudget;
}

inline bool p2LpBoundExcludesImprovement(
        const double lpLowerBound,
        const double incumbentObjective,
        const double improvementTolerance) noexcept {
    return std::isfinite(lpLowerBound) &&
        std::isfinite(incumbentObjective) &&
        std::isfinite(improvementTolerance) &&
        improvementTolerance >= 0.0 &&
        lpLowerBound >=
            incumbentObjective - improvementTolerance;
}

inline double selectP2MipTimeLimit(
        const bool local,
        const bool hardMandatoryFinal,
        const bool hardTimeWindows,
        const bool twFinalGateEnabled,
        const double localOnlineSeconds,
        const double globalOnlineSeconds,
        const double globalFinalSeconds,
        const double twFinalSeconds) noexcept {
    if (!hardMandatoryFinal) {
        return local
            ? localOnlineSeconds
            : globalOnlineSeconds;
    }
    return hardTimeWindows && twFinalGateEnabled
        ? twFinalSeconds
        : globalFinalSeconds;
}

inline P2MipGateDecision evaluateP2MipGate(
        const bool gateEnabled,
        const bool lpCompletedEligible,
        const bool exactSnapshot,
        const bool noImprovementPossible) noexcept {
    P2MipGateDecision decision;
    if (!gateEnabled) {
        decision.reason = "lp_gate_disabled";
        return decision;
    }
    if (!lpCompletedEligible) {
        decision.reason = "lp_incomplete_or_ambiguous";
        return decision;
    }
    if (!exactSnapshot) {
        decision.reason = "lp_snapshot_mismatch";
        return decision;
    }
    if (!noImprovementPossible) {
        decision.reason = "lp_has_improvement_potential";
        return decision;
    }
    decision.runMip = false;
    decision.rigorousNoImprovement = true;
    decision.reason = "lp_bound_no_potential";
    return decision;
}

inline bool satisfiesFinalIntensification(
        const bool hardMandatoryFinal,
        const bool mipAttempted) noexcept {
    return hardMandatoryFinal && mipAttempted;
}

inline bool satisfiesFinalIntensification(
        const bool hardMandatoryFinal,
        const bool mipAttempted,
        const bool rigorousLpGateMaySatisfy,
        const bool rigorousLpNoImprovement) noexcept {
    return hardMandatoryFinal &&
        (mipAttempted ||
         (rigorousLpGateMaySatisfy &&
          rigorousLpNoImprovement));
}

inline bool shouldRequestHardTwEarlyStopGate(
        const bool enabled,
        const bool hardTimeWindows,
        const int iteration,
        const int nonImprovingAttempts,
        const int minimumIteration,
        const int minimumNonImprovingAttempts) noexcept {
    return enabled && hardTimeWindows &&
        iteration >= minimumIteration &&
        nonImprovingAttempts >= minimumNonImprovingAttempts;
}

enum class HardTwEarlyStopAction {
    None,
    ExecuteFinalGlobal,
    StopAfterCompletedFinal,
};

inline HardTwEarlyStopAction decideHardTwEarlyStopAction(
        const bool enabled,
        const bool hardTimeWindows,
        const bool finalPhaseReady,
        const bool finalGlobalComplete,
        const int iteration,
        const int nonImprovingAttempts,
        const int minimumIteration,
        const int minimumNonImprovingAttempts) noexcept {
    if (!finalPhaseReady ||
        !shouldRequestHardTwEarlyStopGate(
            enabled,
            hardTimeWindows,
            iteration,
            nonImprovingAttempts,
            minimumIteration,
            minimumNonImprovingAttempts)) {
        return HardTwEarlyStopAction::None;
    }
    return finalGlobalComplete
        ? HardTwEarlyStopAction::StopAfterCompletedFinal
        : HardTwEarlyStopAction::ExecuteFinalGlobal;
}

inline bool shouldStopAfterHardTwFinalGate(
        const bool finalGateSatisfied,
        const bool mipImprovedIncumbent) noexcept {
    return finalGateSatisfied && !mipImprovedIncumbent;
}

}  // namespace hgasp

#endif  // HGASP_CORE_SP_EXECUTION_PLAN_H_
