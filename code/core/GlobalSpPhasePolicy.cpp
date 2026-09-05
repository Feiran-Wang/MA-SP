#include "GlobalSpPhasePolicy.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace hgasp {
namespace {

constexpr GlobalSpPhaseAction kScheduledActions[] = {
    GlobalSpPhaseAction::EarlyGlobal,
    GlobalSpPhaseAction::PreReductionGlobal1,
    GlobalSpPhaseAction::Reduction1,
    GlobalSpPhaseAction::PreReductionGlobal2,
    GlobalSpPhaseAction::Reduction2,
};

bool isPreReductionGlobal(
        const GlobalSpPhaseAction action) noexcept {
    return action == GlobalSpPhaseAction::EarlyGlobal ||
        action == GlobalSpPhaseAction::PreReductionGlobal1 ||
        action == GlobalSpPhaseAction::PreReductionGlobal2;
}

bool isReduction(const GlobalSpPhaseAction action) noexcept {
    return action == GlobalSpPhaseAction::Reduction1 ||
        action == GlobalSpPhaseAction::Reduction2;
}

}  // namespace

const char* toString(const GlobalSpPhaseAction action) noexcept {
    switch (action) {
    case GlobalSpPhaseAction::None:
        return "none";
    case GlobalSpPhaseAction::EarlyGlobal:
        return "early_global";
    case GlobalSpPhaseAction::PreReductionGlobal1:
        return "pre_reduction_global_1";
    case GlobalSpPhaseAction::Reduction1:
        return "population_reduction_1";
    case GlobalSpPhaseAction::PreReductionGlobal2:
        return "pre_reduction_global_2";
    case GlobalSpPhaseAction::Reduction2:
        return "population_reduction_2";
    case GlobalSpPhaseAction::FinalGlobal:
        return "final_global";
    }
    return "unknown";
}

const char* toString(
        const GlobalSpPhaseCompletion completion) noexcept {
    switch (completion) {
    case GlobalSpPhaseCompletion::Completed:
        return "completed";
    case GlobalSpPhaseCompletion::ValidNoImprovement:
        return "valid_no_improvement";
    case GlobalSpPhaseCompletion::IncompleteMaster:
        return "incomplete_master";
    case GlobalSpPhaseCompletion::DeferredBudget:
        return "deferred_budget";
    case GlobalSpPhaseCompletion::SkippedDisabled:
        return "skipped_disabled";
    case GlobalSpPhaseCompletion::SkippedNoCheckpointFeedback:
        return "skipped_no_checkpoint_feedback";
    case GlobalSpPhaseCompletion::FatalError:
        return "fatal_error";
    }
    return "unknown";
}

void GlobalSpPhasePolicy::validateConfig(
        const GlobalSpPhaseConfig& config) {
    const double fractions[] = {
        config.earlyGlobalFraction,
        config.preReductionGlobal1Fraction,
        config.populationReduction1Fraction,
        config.preReductionGlobal2Fraction,
        config.populationReduction2Fraction,
        config.minimumAbsorptionFraction,
    };
    for (const double fraction : fractions) {
        if (!std::isfinite(fraction) ||
            fraction < 0.0 || fraction > 1.0) {
            throw std::invalid_argument(
                "Global-SP phase fraction must be in [0,1]");
        }
    }
    if (!(config.earlyGlobalFraction <=
              config.preReductionGlobal1Fraction &&
          config.preReductionGlobal1Fraction <=
              config.populationReduction1Fraction &&
          config.populationReduction1Fraction <
              config.preReductionGlobal2Fraction &&
          config.preReductionGlobal2Fraction <=
              config.populationReduction2Fraction) ||
        config.populationReduction1Fraction -
                config.preReductionGlobal1Fraction + 1.0e-12 <
            config.minimumAbsorptionFraction ||
        config.populationReduction2Fraction -
                config.preReductionGlobal2Fraction + 1.0e-12 <
            config.minimumAbsorptionFraction ||
        !std::isfinite(config.lpTimeLimitSeconds) ||
        config.lpTimeLimitSeconds < 0.0 ||
        !std::isfinite(config.mipTimeLimitSeconds) ||
        config.mipTimeLimitSeconds < 0.0 ||
        config.maximumInjections == 0U ||
        !std::isfinite(config.earlyGlobalBaseMipSeconds) ||
        !std::isfinite(config.earlyGlobalMaximumMipSeconds) ||
        !std::isfinite(config.firstGlobalBaseMipSeconds) ||
        !std::isfinite(config.firstGlobalMaximumMipSeconds) ||
        !std::isfinite(config.secondGlobalBaseMipSeconds) ||
        !std::isfinite(config.secondGlobalMaximumMipSeconds) ||
        !std::isfinite(config.finalGlobalFraction) ||
        config.earlyGlobalBaseMipSeconds <= 0.0 ||
        config.earlyGlobalBaseMipSeconds >
            config.earlyGlobalMaximumMipSeconds ||
        config.firstGlobalBaseMipSeconds <= 0.0 ||
        config.firstGlobalBaseMipSeconds >
            config.firstGlobalMaximumMipSeconds ||
        config.secondGlobalBaseMipSeconds <= 0.0 ||
        config.secondGlobalBaseMipSeconds >
            config.secondGlobalMaximumMipSeconds ||
        config.finalGlobalFraction < config.populationReduction2Fraction ||
        config.finalGlobalFraction > 1.0 ||
        (config.scheduledFinalGlobalEnabled &&
         (!std::isfinite(config.finalGlobalBaseMipSeconds) ||
          !std::isfinite(config.finalGlobalMaximumMipSeconds) ||
          config.finalGlobalBaseMipSeconds <= 0.0 ||
          config.finalGlobalBaseMipSeconds >
              config.finalGlobalMaximumMipSeconds ||
          config.finalGlobalMaximumInjections == 0U)) ||
        (config.preReductionGlobalEnabled &&
         (config.lpTimeLimitSeconds <= 0.0 ||
          config.mipTimeLimitSeconds <= 0.0))) {
        throw std::invalid_argument(
            "invalid Global-SP phase configuration");
    }
}

std::uint64_t GlobalSpPhasePolicy::phaseThreshold(
        const std::uint64_t horizon,
        const double fraction) {
    const long double scaled =
        static_cast<long double>(horizon) *
        static_cast<long double>(fraction);
    // Decimal manifest fractions such as 0.45 are not exact binary doubles.
    // Remove only representation-scale noise before taking the conservative
    // ceiling; a genuinely fractional threshold still rounds upward.
    const long double representationTolerance =
        1.0e-12L * std::max(1.0L, std::fabs(scaled));
    return static_cast<std::uint64_t>(
        std::ceil(scaled - representationTolerance));
}

GlobalSpPhasePolicy::GlobalSpPhasePolicy(
        const std::uint64_t iterationLimit,
        const GlobalSpPhaseConfig config)
    : config_(config) {
    validateConfig(config_);
    if (iterationLimit == 0U) {
        throw std::invalid_argument(
            "Global-SP attempt phases require a positive iteration limit");
    }
    phaseHorizonAttempts_ =
        config_.configuredHorizonAttempts == 0U
        ? iterationLimit
        : config_.configuredHorizonAttempts;
    const double fractions[] = {
        config_.earlyGlobalFraction,
        config_.preReductionGlobal1Fraction,
        config_.populationReduction1Fraction,
        config_.preReductionGlobal2Fraction,
        config_.populationReduction2Fraction,
    };
    for (std::size_t index = 0U; index < 5U; ++index) {
        thresholds_[index] = phaseThreshold(
            phaseHorizonAttempts_, fractions[index]);
    }
    thresholds_[5] = phaseThreshold(
        phaseHorizonAttempts_, config_.finalGlobalFraction);
}

std::size_t GlobalSpPhasePolicy::phaseIndex(
        const GlobalSpPhaseAction action) const {
    switch (action) {
    case GlobalSpPhaseAction::EarlyGlobal:
        return 0U;
    case GlobalSpPhaseAction::PreReductionGlobal1:
        return 1U;
    case GlobalSpPhaseAction::Reduction1:
        return 2U;
    case GlobalSpPhaseAction::PreReductionGlobal2:
        return 3U;
    case GlobalSpPhaseAction::Reduction2:
        return 4U;
    case GlobalSpPhaseAction::FinalGlobal:
        return 5U;
    case GlobalSpPhaseAction::None:
        break;
    }
    throw std::invalid_argument("none is not an executable phase");
}

bool GlobalSpPhasePolicy::completed(
        const GlobalSpPhaseAction action) const noexcept {
    switch (action) {
    case GlobalSpPhaseAction::EarlyGlobal:
        return completed_[0];
    case GlobalSpPhaseAction::PreReductionGlobal1:
        return completed_[1];
    case GlobalSpPhaseAction::Reduction1:
        return completed_[2];
    case GlobalSpPhaseAction::PreReductionGlobal2:
        return completed_[3];
    case GlobalSpPhaseAction::Reduction2:
        return completed_[4];
    case GlobalSpPhaseAction::FinalGlobal:
        return completed_[5];
    case GlobalSpPhaseAction::None:
        return false;
    }
    return false;
}

std::uint64_t GlobalSpPhasePolicy::threshold(
        const GlobalSpPhaseAction action) const {
    if (action == GlobalSpPhaseAction::FinalGlobal) {
        return thresholds_[5];
    }
    return thresholds_[phaseIndex(action)];
}

GlobalSpPhaseDecision GlobalSpPhasePolicy::claim(
        const GlobalSpPhaseAction action,
        const std::uint64_t thresholdAttempt,
        const bool executionRequired,
        const char* const reason) {
    inFlight_ = action;
    GlobalSpPhaseDecision decision;
    decision.action = action;
    decision.claimed = true;
    decision.executionRequired = executionRequired;
    decision.preReductionGlobal = isPreReductionGlobal(action);
    decision.thresholdAttempt = thresholdAttempt;
    decision.phaseHorizonAttempts = phaseHorizonAttempts_;
    decision.reason = reason;
    if (decision.preReductionGlobal && executionRequired) {
        decision.lpTimeLimitSeconds = config_.lpTimeLimitSeconds;
        const double base = action == GlobalSpPhaseAction::EarlyGlobal
            ? config_.earlyGlobalBaseMipSeconds
            : (action == GlobalSpPhaseAction::PreReductionGlobal1
                   ? config_.firstGlobalBaseMipSeconds
                   : config_.secondGlobalBaseMipSeconds);
        const double maximum = action == GlobalSpPhaseAction::EarlyGlobal
            ? config_.earlyGlobalMaximumMipSeconds
            : (action == GlobalSpPhaseAction::PreReductionGlobal1
                   ? config_.firstGlobalMaximumMipSeconds
                   : config_.secondGlobalMaximumMipSeconds);
        decision.mipTimeLimitSeconds = maximum;
        decision.improvementExtensionGateSeconds =
            base < maximum ? base : 0.0;
        decision.maximumInjections = config_.maximumInjections;
    } else if (action == GlobalSpPhaseAction::FinalGlobal &&
               executionRequired &&
               config_.scheduledFinalGlobalEnabled) {
        decision.lpTimeLimitSeconds = 0.0;
        decision.mipTimeLimitSeconds =
            config_.finalGlobalMaximumMipSeconds;
        decision.improvementExtensionGateSeconds =
            config_.finalGlobalBaseMipSeconds <
                    config_.finalGlobalMaximumMipSeconds
                ? config_.finalGlobalBaseMipSeconds
                : 0.0;
        decision.maximumInjections =
            config_.finalGlobalMaximumInjections;
    }
    return decision;
}

GlobalSpPhaseDecision GlobalSpPhasePolicy::claimNext(
        const std::uint64_t currentAttempt,
        const bool finalRequested,
        const bool scheduledPhasesAvailable) {
    GlobalSpPhaseDecision decision;
    decision.phaseHorizonAttempts = phaseHorizonAttempts_;
    if (currentAttempt < maximumObservedAttempt_) {
        throw std::invalid_argument(
            "Global-SP phase attempt counter moved backwards; use the "
            "run-level cumulative attempt counter across restarts");
    }
    maximumObservedAttempt_ = currentAttempt;
    if (fatal_) {
        decision.reason = "fatal_phase_error";
        return decision;
    }
    if (inFlight_.has_value()) {
        decision.reason = "phase_in_flight";
        return decision;
    }

    // Static materialization can fail safely into a free-only population.
    // The scheduled Global/reduction phases depend on the missing static
    // registry, but the run-level Final Global does not. Preserve the same
    // one-shot Final latch without pretending that unavailable reductions
    // completed.
    if (!scheduledPhasesAvailable) {
        if (finalRequested && !completed_[5]) {
            return claim(
                GlobalSpPhaseAction::FinalGlobal,
                currentAttempt,
                true,
                "final_requested_without_static_populations");
        }
        decision.reason = completed_[5]
            ? "final_already_completed"
            : "scheduled_phases_unavailable";
        return decision;
    }

    for (std::size_t index = 0U; index < 5U; ++index) {
        if (index == 0U && !config_.earlyGlobalEnabled) {
            continue;
        }
        if (completed_[index]) {
            continue;
        }
        if (currentAttempt < thresholds_[index]) {
            decision.thresholdAttempt = thresholds_[index];
            decision.reason = finalRequested
                ? "final_waiting_for_scheduled_phases"
                : "not_due";
            return decision;
        }
        const GlobalSpPhaseAction action = kScheduledActions[index];
        const bool global = isPreReductionGlobal(action);
        const bool executionRequired =
            !global || config_.preReductionGlobalEnabled;
        return claim(
            action,
            thresholds_[index],
            executionRequired,
            executionRequired ? "due" : "configured_disabled");
    }

    if (!completed_[5] &&
        (finalRequested ||
         (config_.scheduledFinalGlobalEnabled &&
          currentAttempt >= thresholds_[5]))) {
        return claim(
            GlobalSpPhaseAction::FinalGlobal,
            thresholds_[5],
            true,
            finalRequested
                ? "final_requested"
                : "scheduled_last_5000_attempts");
    }
    decision.reason = completed_[5]
        ? "final_already_completed"
        : "no_phase_due";
    return decision;
}

void GlobalSpPhasePolicy::complete(
        const GlobalSpPhaseAction action,
        const GlobalSpPhaseCompletion completion) {
    if (!inFlight_.has_value() || *inFlight_ != action) {
        throw std::logic_error(
            "Global-SP phase completion does not match the in-flight "
            "action");
    }
    const bool global = isPreReductionGlobal(action);
    if (isReduction(action) &&
        completion != GlobalSpPhaseCompletion::Completed) {
        throw std::invalid_argument(
            "population reduction requires a completed outcome");
    }
    if (completion == GlobalSpPhaseCompletion::SkippedDisabled &&
        (!global || config_.preReductionGlobalEnabled)) {
        throw std::invalid_argument(
            "only a disabled pre-reduction Global phase may be skipped");
    }
    if (completion ==
            GlobalSpPhaseCompletion::SkippedNoCheckpointFeedback &&
        action != GlobalSpPhaseAction::PreReductionGlobal2) {
        throw std::invalid_argument(
            "only the second exploratory Global phase may be skipped "
            "after an unproductive first checkpoint");
    }
    if (global && !config_.preReductionGlobalEnabled &&
        completion != GlobalSpPhaseCompletion::SkippedDisabled) {
        throw std::invalid_argument(
            "a disabled pre-reduction Global phase must be recorded as "
            "skipped");
    }
    if (action == GlobalSpPhaseAction::FinalGlobal &&
        completion == GlobalSpPhaseCompletion::DeferredBudget) {
        throw std::invalid_argument(
            "mandatory Final Global cannot be consumed by a budget gate");
    }
    if (!global && action != GlobalSpPhaseAction::FinalGlobal &&
        completion == GlobalSpPhaseCompletion::FatalError) {
        throw std::invalid_argument(
            "a pure reduction phase cannot report a Gurobi fatal error");
    }

    const std::size_t index = phaseIndex(action);
    inFlight_.reset();
    if (completion == GlobalSpPhaseCompletion::FatalError) {
        fatal_ = true;
        return;
    }
    completed_[index] = true;
    completedActions_.push_back(action);
}

}  // namespace hgasp
