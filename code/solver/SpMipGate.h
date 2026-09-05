#ifndef HGASP_SOLVER_SP_MIP_GATE_H_
#define HGASP_SOLVER_SP_MIP_GATE_H_

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>

namespace hgasp {

struct SpMipGateDecision {
    bool skipMip = false;
    bool noImprovementPossible = false;
    bool belowHeuristicThreshold = false;
    double relativePotential = 0.0;
    double boundSafetyMargin = 0.0;
    std::string reason;
};

enum class FinalGlobalMipTier {
    LegacyFull,
    NoPotential,
    Short,
    Medium,
    Full,
};

inline const char* toString(
        const FinalGlobalMipTier tier) noexcept {
    switch (tier) {
        case FinalGlobalMipTier::LegacyFull:
            return "legacy_full";
        case FinalGlobalMipTier::NoPotential:
            return "no_potential";
        case FinalGlobalMipTier::Short:
            return "short";
        case FinalGlobalMipTier::Medium:
            return "medium";
        case FinalGlobalMipTier::Full:
            return "full";
    }
    return "unknown";
}

struct FinalGlobalMipPlan {
    FinalGlobalMipTier tier =
        FinalGlobalMipTier::LegacyFull;
    bool skipMip = false;
    bool noImprovementPossible = false;
    double relativePotential = 0.0;
    double boundSafetyMargin = 0.0;
    double timeLimitSeconds = 0.0;
    std::string reason = "tiering_disabled";
};

struct FinalGlobalMipExtensionPlan {
    bool extend = false;
    FinalGlobalMipTier nextTier =
        FinalGlobalMipTier::NoPotential;
    double observedImprovement = 0.0;
    // A later stage rebuilds the finite RMP and receives only the difference
    // between the cumulative tier limits. The validated incumbent from the
    // completed stage is supplied as its MIP start.
    double additionalTimeLimitSeconds = 0.0;
    double cumulativeTimeLimitSeconds = 0.0;
    std::optional<double> nextTargetObjective;
    std::string reason = "no_extension";
};

struct FinalGlobalWorkExtensionPlan {
    bool permitsExtension = true;
    std::optional<double> nextWorkLimit;
    double cumulativeWork = 0.0;
    std::string reason = "work_limit_disabled";
};

// WorkLimit is cumulative across one logical staged Final Global call.
// Rebuilding the finite RMP must not grant the configured amount again.
inline FinalGlobalWorkExtensionPlan
evaluateFinalGlobalWorkExtension(
        const double configuredWorkLimit,
        const double workConsumedBeforeStage,
        const std::optional<double> completedStageWork) {
    if (!std::isfinite(configuredWorkLimit) ||
        configuredWorkLimit < 0.0 ||
        !std::isfinite(workConsumedBeforeStage) ||
        workConsumedBeforeStage < 0.0 ||
        (completedStageWork.has_value() &&
         (!std::isfinite(*completedStageWork) ||
          *completedStageWork < 0.0))) {
        throw std::invalid_argument(
            "invalid staged final-Global work budget");
    }

    FinalGlobalWorkExtensionPlan plan;
    plan.cumulativeWork = workConsumedBeforeStage;
    if (completedStageWork.has_value()) {
        plan.cumulativeWork += *completedStageWork;
        if (!std::isfinite(plan.cumulativeWork)) {
            throw std::invalid_argument(
                "staged final-Global work budget overflow");
        }
    }
    if (configuredWorkLimit == 0.0) {
        return plan;
    }
    if (!completedStageWork.has_value()) {
        plan.permitsExtension = false;
        plan.reason = "stage_work_unavailable";
        return plan;
    }
    const double remaining =
        configuredWorkLimit - plan.cumulativeWork;
    const double tolerance =
        1.0e-12 * std::max(1.0, configuredWorkLimit);
    if (remaining <= tolerance) {
        plan.permitsExtension = false;
        plan.reason = "work_limit_exhausted";
        return plan;
    }
    plan.nextWorkLimit = remaining;
    plan.reason = "remaining_work_available";
    return plan;
}

inline void validateFinalGlobalMipBudgets(
        const double shortTimeLimitSeconds,
        const double mediumTimeLimitSeconds,
        const double fullTimeLimitSeconds) {
    if (!std::isfinite(shortTimeLimitSeconds) ||
        !std::isfinite(mediumTimeLimitSeconds) ||
        !std::isfinite(fullTimeLimitSeconds) ||
        shortTimeLimitSeconds <= 0.0 ||
        mediumTimeLimitSeconds < shortTimeLimitSeconds ||
        fullTimeLimitSeconds < mediumTimeLimitSeconds) {
        throw std::invalid_argument(
            "invalid staged final-Global time budgets");
    }
}

// Chooses only the initial effort stage for the mandatory final Global
// integer RMP. Tiered execution always starts at the short stage. The LP
// objective remains diagnostic and may prove that the finite-RMP relaxation
// is already near the incumbent, but it cannot grant medium effort directly.
// Later stages are reserved for a stage that has demonstrated an independently
// validated strict incumbent improvement.
inline FinalGlobalMipPlan evaluateFinalGlobalMipPlan(
        const std::optional<double> lpObjective,
        const double targetObjective,
        const double minimumImprovement,
        const bool tieringEnabled,
        const double lowPotentialThreshold,
        const double highPotentialThreshold,
        const double shortTimeLimitSeconds,
        const double mediumTimeLimitSeconds,
        const double fullTimeLimitSeconds) {
    if (!std::isfinite(targetObjective) ||
        !std::isfinite(minimumImprovement) ||
        minimumImprovement < 0.0 ||
        !std::isfinite(fullTimeLimitSeconds) ||
        fullTimeLimitSeconds <= 0.0 ||
        (lpObjective.has_value() &&
         !std::isfinite(*lpObjective))) {
        throw std::invalid_argument(
            "invalid tiered final-Global input");
    }
    if (tieringEnabled) {
        if (!std::isfinite(lowPotentialThreshold) ||
            !std::isfinite(highPotentialThreshold) ||
            lowPotentialThreshold < 0.0 ||
            highPotentialThreshold < lowPotentialThreshold) {
            throw std::invalid_argument(
                "invalid tiered final-Global input");
        }
        validateFinalGlobalMipBudgets(
            shortTimeLimitSeconds,
            mediumTimeLimitSeconds,
            fullTimeLimitSeconds);
    }

    FinalGlobalMipPlan plan;
    plan.boundSafetyMargin =
        1.0e-6 *
        std::max(1.0, std::fabs(targetObjective));
    plan.timeLimitSeconds = fullTimeLimitSeconds;
    if (lpObjective.has_value()) {
        plan.relativePotential = std::max(
            0.0,
            targetObjective - *lpObjective) /
            std::max(1.0, std::fabs(targetObjective));
        plan.noImprovementPossible =
            *lpObjective >=
            targetObjective - minimumImprovement +
                plan.boundSafetyMargin;
    }
    if (!tieringEnabled) {
        return plan;
    }
    plan.tier = FinalGlobalMipTier::Short;
    plan.timeLimitSeconds = shortTimeLimitSeconds;
    if (!lpObjective.has_value()) {
        plan.reason = "lp_bound_unavailable_short_first";
        return plan;
    }
    if (plan.noImprovementPossible) {
        plan.skipMip = false;
        plan.reason =
            "final_lp_bound_near_incumbent_short_first";
        return plan;
    }
    if (plan.relativePotential < lowPotentialThreshold) {
        plan.reason = "relative_potential_low_short_first";
        return plan;
    }
    if (plan.relativePotential < highPotentialThreshold) {
        plan.reason = "relative_potential_medium_short_first";
        return plan;
    }
    plan.reason = "relative_potential_high_short_first";
    return plan;
}

// Decides whether a completed mandatory final-Global stage has earned more
// effort. A caller must pass an objective only for an independently validated
// incumbent from that stage. The comparison objective is the incumbent that
// the completed stage had to beat: the HGA target for the first stage and the
// prior stage's validated objective thereafter.
inline FinalGlobalMipExtensionPlan
evaluateFinalGlobalMipExtension(
        const FinalGlobalMipTier completedTier,
        const std::optional<double> completedObjective,
        const double comparisonObjective,
        const double minimumImprovement,
        const bool completedStageOptimal,
        const double shortTimeLimitSeconds,
        const double mediumTimeLimitSeconds,
        const double fullTimeLimitSeconds) {
    if (completedTier != FinalGlobalMipTier::Short &&
        completedTier != FinalGlobalMipTier::Medium &&
        completedTier != FinalGlobalMipTier::Full) {
        throw std::invalid_argument(
            "invalid completed final-Global stage");
    }
    if (!std::isfinite(comparisonObjective) ||
        !std::isfinite(minimumImprovement) ||
        minimumImprovement < 0.0 ||
        (completedObjective.has_value() &&
         !std::isfinite(*completedObjective))) {
        throw std::invalid_argument(
            "invalid staged final-Global observation");
    }
    validateFinalGlobalMipBudgets(
        shortTimeLimitSeconds,
        mediumTimeLimitSeconds,
        fullTimeLimitSeconds);

    FinalGlobalMipExtensionPlan plan;
    plan.cumulativeTimeLimitSeconds =
        completedTier == FinalGlobalMipTier::Short
            ? shortTimeLimitSeconds
            : (completedTier == FinalGlobalMipTier::Medium
                   ? mediumTimeLimitSeconds
                   : fullTimeLimitSeconds);
    if (completedObjective.has_value()) {
        plan.observedImprovement =
            comparisonObjective - *completedObjective;
    }
    if (completedTier == FinalGlobalMipTier::Full) {
        plan.reason = "full_stage_complete";
        return plan;
    }
    if (completedStageOptimal) {
        plan.reason = "stage_proved_finite_rmp_optimal";
        return plan;
    }
    if (!completedObjective.has_value()) {
        plan.reason = "no_validated_stage_incumbent";
        return plan;
    }

    const bool strictImprovement =
        minimumImprovement == 0.0
            ? *completedObjective < comparisonObjective
            : *completedObjective <
                  comparisonObjective - minimumImprovement;
    if (!strictImprovement) {
        plan.reason = "no_strict_stage_improvement";
        return plan;
    }
    plan.nextTargetObjective =
        *completedObjective - minimumImprovement;
    if (!std::isfinite(*plan.nextTargetObjective)) {
        throw std::invalid_argument(
            "invalid staged final-Global improvement target");
    }

    if (completedTier == FinalGlobalMipTier::Short &&
        mediumTimeLimitSeconds > shortTimeLimitSeconds) {
        plan.extend = true;
        plan.nextTier = FinalGlobalMipTier::Medium;
        plan.additionalTimeLimitSeconds =
            mediumTimeLimitSeconds -
            shortTimeLimitSeconds;
        plan.cumulativeTimeLimitSeconds =
            mediumTimeLimitSeconds;
        plan.reason = "strict_improvement_extend_medium";
        return plan;
    }
    if (fullTimeLimitSeconds >
        plan.cumulativeTimeLimitSeconds) {
        plan.extend = true;
        plan.nextTier = FinalGlobalMipTier::Full;
        plan.additionalTimeLimitSeconds =
            fullTimeLimitSeconds -
            plan.cumulativeTimeLimitSeconds;
        plan.cumulativeTimeLimitSeconds =
            fullTimeLimitSeconds;
        plan.reason = "strict_improvement_extend_full";
        return plan;
    }
    plan.reason = "no_additional_stage_budget";
    return plan;
}

// Evaluates only an optimal LP-RMP objective. A missing objective means that
// the probe did not establish a usable bound and therefore cannot suppress
// the integer solve.
inline SpMipGateDecision evaluateSpMipGate(
        const std::optional<double> lpObjective,
        const double targetObjective,
        const double minimumImprovement,
        const double relativePotentialThreshold,
        const bool gateEnabled,
        const bool heuristicThresholdEnabled = true) {
    if (!std::isfinite(targetObjective) ||
        !std::isfinite(minimumImprovement) ||
        minimumImprovement < 0.0 ||
        !std::isfinite(relativePotentialThreshold) ||
        relativePotentialThreshold < 0.0 ||
        (lpObjective.has_value() &&
         !std::isfinite(*lpObjective))) {
        throw std::invalid_argument(
            "invalid LP-to-MIP gate input");
    }
    SpMipGateDecision decision;
    decision.boundSafetyMargin =
        1.0e-6 *
        std::max(1.0, std::fabs(targetObjective));
    if (!lpObjective.has_value()) {
        return decision;
    }
    decision.relativePotential = std::max(
        0.0,
        targetObjective - *lpObjective) /
        std::max(1.0, std::fabs(targetObjective));
    decision.noImprovementPossible =
        *lpObjective >=
        targetObjective - minimumImprovement +
            decision.boundSafetyMargin;
    decision.belowHeuristicThreshold =
        decision.relativePotential <
            relativePotentialThreshold;
    if (!gateEnabled) {
        return decision;
    }
    if (decision.noImprovementPossible) {
        decision.skipMip = true;
        decision.reason = "lp_bound_no_potential";
    } else if (heuristicThresholdEnabled &&
               decision.belowHeuristicThreshold) {
        decision.skipMip = true;
        decision.reason =
            "lp_potential_below_threshold";
    }
    return decision;
}

// A weak Local elite should normally retain only its LP feedback, but a
// configuration with material LP potential is still allowed an integer
// education call. This avoids turning the global-frontier guard into a hard
// barrier against newly discovered depot configurations.
inline bool shouldSkipLocalMipOutsideGlobalFrontier(
        const double localObjective,
        const double globalObjective,
        const double relativeLpPotential,
        const double frontierTolerance = 0.005,
        const double promisingPotential = 0.002) {
    if (!std::isfinite(localObjective) ||
        !std::isfinite(globalObjective) ||
        !std::isfinite(relativeLpPotential) ||
        !std::isfinite(frontierTolerance) ||
        !std::isfinite(promisingPotential) ||
        relativeLpPotential < 0.0 ||
        frontierTolerance < 0.0 ||
        promisingPotential < 0.0) {
        throw std::invalid_argument(
            "invalid Local-SP frontier gate input");
    }
    const bool nearFrontier =
        localObjective <=
            globalObjective +
                frontierTolerance *
                    std::max(1.0, std::fabs(globalObjective));
    return !nearFrontier &&
        relativeLpPotential < promisingPotential;
}

}  // namespace hgasp

#endif  // HGASP_SOLVER_SP_MIP_GATE_H_
