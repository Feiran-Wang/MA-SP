#include "PopulationLocalSpController.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace hgasp {
namespace {

std::uint64_t nonRegressingDelta(
        const std::uint64_t current,
        const std::uint64_t reference) noexcept {
    return current >= reference ? current - reference : 0U;
}

double nonRegressingDelta(
        const double current,
        const double reference) noexcept {
    return current >= reference ? current - reference : 0.0;
}

double replacementGainRatio(
        const PopulationLocalSpSnapshot& current,
        const PopulationLocalSpSnapshot& reference) noexcept {
    const double gain = nonRegressingDelta(
        current.configurationRepresentativeReplacementGain,
        reference.configurationRepresentativeReplacementGain);
    const double denominator = std::max(
        std::fabs(current.populationBestObjective), 1.0e-12);
    return gain / denominator;
}

bool samePopulationLifetime(
        const PopulationLocalSpSnapshot& left,
        const PopulationLocalSpSnapshot& right) noexcept {
    return left.population == right.population &&
        left.configuration == right.configuration;
}

}  // namespace

const char* toString(const PopulationLocalSpPhase phase) noexcept {
    switch (phase) {
    case PopulationLocalSpPhase::Active:
        return "active";
    case PopulationLocalSpPhase::Eligible:
        return "eligible";
    case PopulationLocalSpPhase::Running:
        return "running";
    case PopulationLocalSpPhase::Improved:
        return "improved";
    case PopulationLocalSpPhase::Exhausted:
        return "exhausted";
    case PopulationLocalSpPhase::Retired:
        return "retired";
    }
    return "unknown";
}

const char* toString(
        const PopulationLocalSpOutcomeKind outcome) noexcept {
    switch (outcome) {
    case PopulationLocalSpOutcomeKind::Improved:
        return "improved";
    case PopulationLocalSpOutcomeKind::ExhaustedLpBound:
        return "exhausted_lp_bound";
    case PopulationLocalSpOutcomeKind::ExhaustedMipOptimalNoImprovement:
        return "exhausted_mip_optimal_no_improvement";
    case PopulationLocalSpOutcomeKind::ExhaustedTimeLimitNoImprovement:
        return "exhausted_time_limit_no_improvement";
    case PopulationLocalSpOutcomeKind::ExhaustedTerminalNoImprovement:
        return "exhausted_terminal_no_improvement";
    case PopulationLocalSpOutcomeKind::DeferredIncompleteCoverage:
        return "deferred_incomplete_coverage";
    case PopulationLocalSpOutcomeKind::DeferredCpuBudget:
        return "deferred_cpu_budget";
    case PopulationLocalSpOutcomeKind::DeferredEnvironmentFailure:
        return "deferred_environment_failure";
    case PopulationLocalSpOutcomeKind::DeferredStalePopulation:
        return "deferred_stale_population";
    case PopulationLocalSpOutcomeKind::DeferredSnapshotChanged:
        return "deferred_snapshot_changed";
    case PopulationLocalSpOutcomeKind::DeferredUnsupportedStatus:
        return "deferred_unsupported_status";
    }
    return "unknown";
}

PopulationLocalSpController::PopulationLocalSpController(
        const PopulationLocalSpPolicy policy) {
    configure(policy);
}

void PopulationLocalSpController::validatePolicy(
        const PopulationLocalSpPolicy& policy) {
    if (!std::isfinite(policy.minimumReplacementGainRatio) ||
        policy.minimumReplacementGainRatio < 0.0 ||
        !std::isfinite(policy.lpTimeLimitSeconds) ||
        policy.lpTimeLimitSeconds < 0.0 ||
        !std::isfinite(policy.mipTimeLimitSeconds) ||
        policy.mipTimeLimitSeconds < 0.0 ||
        policy.maximumCallsPerLifetime == 0U) {
        throw std::invalid_argument(
            "invalid population Local-SP policy");
    }
}

void PopulationLocalSpController::validateObservation(
        const PopulationLocalSpObservation& observation) {
    validatePopulationLocalSpSnapshot(observation.snapshot);
    if (observation.createdAttempt > observation.currentAttempt ||
        observation.lastPopulationBestImprovementAttempt >
            observation.currentAttempt ||
        observation.lastPopulationBestImprovementAttempt <
            observation.createdAttempt) {
        throw std::invalid_argument(
            "invalid population Local-SP attempt observation");
    }
}

void PopulationLocalSpController::validateOutcome(
        const PopulationLocalSpOutcome& outcome) {
    if (!std::isfinite(outcome.outerCpuSeconds) ||
        outcome.outerCpuSeconds < 0.0 ||
        !std::isfinite(outcome.directObjectiveImprovement) ||
        outcome.directObjectiveImprovement < 0.0 ||
        !std::isfinite(outcome.postLocalSearchImprovement) ||
        outcome.postLocalSearchImprovement < 0.0) {
        throw std::invalid_argument(
            "invalid population Local-SP outcome");
    }
    if (outcome.kind == PopulationLocalSpOutcomeKind::Improved &&
        outcome.directObjectiveImprovement <= 0.0 &&
        outcome.postLocalSearchImprovement <= 0.0) {
        throw std::invalid_argument(
            "an improved Local-SP outcome requires a strict improvement");
    }
    if (outcome.kind != PopulationLocalSpOutcomeKind::Improved &&
        (outcome.directObjectiveImprovement > 0.0 ||
         outcome.postLocalSearchImprovement > 0.0)) {
        throw std::invalid_argument(
            "a non-improving Local-SP outcome cannot carry improvement");
    }
}

bool PopulationLocalSpController::isExhaustedOutcome(
        const PopulationLocalSpOutcomeKind outcome) noexcept {
    return outcome == PopulationLocalSpOutcomeKind::ExhaustedLpBound ||
        outcome ==
            PopulationLocalSpOutcomeKind::ExhaustedMipOptimalNoImprovement ||
        outcome ==
            PopulationLocalSpOutcomeKind::ExhaustedTimeLimitNoImprovement ||
        outcome ==
            PopulationLocalSpOutcomeKind::ExhaustedTerminalNoImprovement;
}

bool PopulationLocalSpController::isDeferredOutcome(
        const PopulationLocalSpOutcomeKind outcome) noexcept {
    return outcome ==
            PopulationLocalSpOutcomeKind::DeferredIncompleteCoverage ||
        outcome == PopulationLocalSpOutcomeKind::DeferredCpuBudget ||
        outcome ==
            PopulationLocalSpOutcomeKind::DeferredEnvironmentFailure ||
        outcome ==
            PopulationLocalSpOutcomeKind::DeferredStalePopulation ||
        outcome ==
            PopulationLocalSpOutcomeKind::DeferredSnapshotChanged ||
        outcome ==
            PopulationLocalSpOutcomeKind::DeferredUnsupportedStatus;
}

void PopulationLocalSpController::configure(
        const PopulationLocalSpPolicy policy) {
    validatePolicy(policy);
    policy_ = policy;
}

void PopulationLocalSpController::retireOlderSlotGenerations(
        const PopulationLocalSpToken population) noexcept {
    for (auto& entry : states_) {
        if (entry.first.slot == population.slot &&
            entry.first != population) {
            entry.second.phase = PopulationLocalSpPhase::Retired;
            entry.second.runningSnapshot.reset();
        }
    }
}

PopulationLocalSpController::State&
PopulationLocalSpController::stateFor(
        const PopulationLocalSpObservation& observation) {
    auto found = states_.find(observation.snapshot.population);
    if (found == states_.end()) {
        bool reusedSlot = false;
        for (const auto& entry : states_) {
            if (entry.first.slot == observation.snapshot.population.slot) {
                reusedSlot = true;
                break;
            }
        }
        retireOlderSlotGenerations(observation.snapshot.population);
        State state;
        state.population = observation.snapshot.population;
        state.configuration = observation.snapshot.configuration;
        state.createdAttempt = observation.createdAttempt;
        state.bestObservedObjective =
            observation.snapshot.populationBestObjective;
        state.lastPopulationBestImprovementAttempt =
            observation.lastPopulationBestImprovementAttempt;
        state.referenceSnapshot = observation.snapshot;
        state.referenceGlobalInjectionCount =
            observation.verifiedGlobalInjectionCount;
        state.latestSnapshot = observation.snapshot;
        state.latestGlobalInjectionCount =
            observation.verifiedGlobalInjectionCount;
        state.meaningfulChangePending = reusedSlot;
        state.reactivationCount = reusedSlot ? 1U : 0U;
        state.lastReactivationReason = reusedSlot
            ? "generation_changed"
            : "population_registered";
        const auto inserted = states_.emplace(
            observation.snapshot.population, std::move(state));
        return inserted.first->second;
    }

    State& state = found->second;
    if (state.createdAttempt != observation.createdAttempt) {
        throw std::invalid_argument(
            "population Local-SP token reused with a different birth "
            "attempt");
    }
    if (state.configuration != observation.snapshot.configuration &&
        state.phase != PopulationLocalSpPhase::Running &&
        state.phase != PopulationLocalSpPhase::Retired) {
        state.configuration = observation.snapshot.configuration;
        state.referenceSnapshot = observation.snapshot;
        state.referenceGlobalInjectionCount =
            observation.verifiedGlobalInjectionCount;
        state.latestSnapshot = observation.snapshot;
        state.latestGlobalInjectionCount =
            observation.verifiedGlobalInjectionCount;
        state.consumedSnapshot.reset();
        state.bestObservedObjective =
            observation.snapshot.populationBestObjective;
        state.lastPopulationBestImprovementAttempt =
            observation.lastPopulationBestImprovementAttempt;
        state.phase = PopulationLocalSpPhase::Active;
        state.meaningfulChangePending = true;
        ++state.reactivationCount;
        state.lastReactivationReason = "configuration_changed";
    }
    return state;
}

PopulationLocalSpDecision PopulationLocalSpController::makeDecision(
        const PopulationLocalSpObservation& observation,
        const State& state) const {
    PopulationLocalSpDecision decision;
    decision.snapshot = observation.snapshot;
    decision.phase = state.phase;
    decision.currentAttempt = observation.currentAttempt;
    decision.populationAgeAttempts =
        observation.currentAttempt - state.createdAttempt;
    decision.stagnationAttempts =
        observation.currentAttempt -
        state.lastPopulationBestImprovementAttempt;
    decision.newCustomerSets = nonRegressingDelta(
        observation.snapshot.configurationAppendCount,
        state.referenceSnapshot.configurationAppendCount);
    decision.representativeReplacements = nonRegressingDelta(
        observation.snapshot
            .configurationRepresentativeReplacementCount,
        state.referenceSnapshot
            .configurationRepresentativeReplacementCount);
    decision.replacementGainRatio = replacementGainRatio(
        observation.snapshot, state.referenceSnapshot);
    decision.verifiedGlobalInjectionCount =
        observation.verifiedGlobalInjectionCount;
    decision.callCount = state.callCount;
    decision.productiveCallCount = state.productiveCallCount;
    decision.lastCallAttempt = state.lastCallAttempt;
    decision.historicalDirectImprovement = state.directImprovement;
    decision.lpTimeLimitSeconds = policy_.lpTimeLimitSeconds;
    decision.mipTimeLimitSeconds = policy_.mipTimeLimitSeconds;
    decision.snapshotId =
        populationLocalSpSnapshotId(observation.snapshot);
    return decision;
}

PopulationLocalSpDecision PopulationLocalSpController::decide(
        const PopulationLocalSpObservation& observation,
        const LocalSpFamilyDecision& familyDecision) {
    validateObservation(observation);
    State& state = stateFor(observation);

    if (state.phase != PopulationLocalSpPhase::Retired &&
        state.phase != PopulationLocalSpPhase::Running) {
        if (observation.snapshot.populationBestObjective <
            state.bestObservedObjective) {
            state.bestObservedObjective =
                observation.snapshot.populationBestObjective;
            state.lastPopulationBestImprovementAttempt =
                observation.currentAttempt;
        }
        state.lastPopulationBestImprovementAttempt = std::max(
            state.lastPopulationBestImprovementAttempt,
            observation.lastPopulationBestImprovementAttempt);
        state.latestSnapshot = observation.snapshot;
        state.latestGlobalInjectionCount =
            observation.verifiedGlobalInjectionCount;
    }

    PopulationLocalSpDecision decision =
        makeDecision(observation, state);
    const auto reject = [&](const char* const reason) {
        decision.eligible = false;
        decision.reason = reason;
        decision.phase = state.phase;
        if (state.phase == PopulationLocalSpPhase::Eligible) {
            state.phase = PopulationLocalSpPhase::Active;
            decision.phase = state.phase;
        }
        return decision;
    };

    if (state.phase == PopulationLocalSpPhase::Retired) {
        return reject("population_retired");
    }
    if (state.phase == PopulationLocalSpPhase::Running) {
        return reject("population_local_sp_running");
    }
    if (!observation.populationActive) {
        return reject("population_inactive");
    }

    const bool counterRegression =
        observation.snapshot.configurationAppendCount <
            state.referenceSnapshot.configurationAppendCount ||
        observation.snapshot
                .configurationRepresentativeReplacementCount <
            state.referenceSnapshot
                .configurationRepresentativeReplacementCount ||
        observation.snapshot
                .configurationRepresentativeReplacementGain <
            state.referenceSnapshot
                .configurationRepresentativeReplacementGain;
    if (counterRegression) {
        return reject("configuration_counter_regression");
    }

    const bool bestImproved =
        observation.snapshot.populationBestObjective <
        state.referenceSnapshot.populationBestObjective;
    const bool globalInjected =
        observation.verifiedGlobalInjectionCount >
        state.referenceGlobalInjectionCount;
    const bool newCustomerSetsMeaningful =
        decision.newCustomerSets > 0U &&
        decision.newCustomerSets >=
            policy_.minimumNewCustomerSets;
    const bool representativeGainMeaningful =
        decision.representativeReplacements > 0U &&
        decision.replacementGainRatio > 0.0 &&
        decision.replacementGainRatio + 1.0e-15 >=
            policy_.minimumReplacementGainRatio;

    if ((state.phase == PopulationLocalSpPhase::Improved ||
         state.phase == PopulationLocalSpPhase::Exhausted) &&
        !state.meaningfulChangePending) {
        const char* reactivationReason = nullptr;
        if (bestImproved) {
            reactivationReason = "population_best_improved";
        } else if (globalInjected) {
            reactivationReason = "verified_global_injection";
        } else if (newCustomerSetsMeaningful) {
            reactivationReason = "new_customer_sets";
        } else if (representativeGainMeaningful) {
            reactivationReason = "representative_replacement_gain";
        }
        if (reactivationReason == nullptr) {
            return reject("snapshot_already_consumed");
        }
        state.phase = PopulationLocalSpPhase::Active;
        state.consumedSnapshot.reset();
        state.meaningfulChangePending = true;
        ++state.reactivationCount;
        state.lastReactivationReason = reactivationReason;
        decision.phase = state.phase;
    }

    if (state.callCount >= policy_.maximumCallsPerLifetime) {
        return reject("population_call_limit");
    }
    if (decision.populationAgeAttempts < policy_.minimumAgeAttempts) {
        return reject("population_too_young");
    }
    if (decision.stagnationAttempts < policy_.stagnationAttempts) {
        return reject("population_not_stagnant");
    }
    if (!observation.completeCustomerCoverage) {
        return reject("incomplete_customer_coverage");
    }

    const bool initialChangeThresholdDisabled =
        policy_.minimumNewCustomerSets == 0U &&
        policy_.minimumReplacementGainRatio == 0.0;
    const bool meaningfulChangeAvailable =
        state.meaningfulChangePending ||
        newCustomerSetsMeaningful ||
        representativeGainMeaningful ||
        bestImproved || globalInjected ||
        initialChangeThresholdDisabled;
    if (!meaningfulChangeAvailable) {
        return reject("insufficient_meaningful_master_change");
    }
    if (!familyDecision.eligible) {
        decision.reason = toString(familyDecision.reason);
        decision.phase = state.phase;
        if (state.phase == PopulationLocalSpPhase::Eligible) {
            state.phase = PopulationLocalSpPhase::Active;
            decision.phase = state.phase;
        }
        return decision;
    }

    state.phase = PopulationLocalSpPhase::Eligible;
    decision.phase = state.phase;
    decision.eligible = true;
    decision.reason = "eligible";
    return decision;
}

std::optional<PopulationLocalSpDecision>
PopulationLocalSpController::selectCandidate(
        const std::vector<PopulationLocalSpDecision>& decisions) const {
    std::optional<PopulationLocalSpDecision> selected;
    const auto preferred = [](
            const PopulationLocalSpDecision& left,
            const PopulationLocalSpDecision& right) {
        if (left.callCount != right.callCount) {
            return left.callCount < right.callCount;
        }
        if (left.lastCallAttempt != right.lastCallAttempt) {
            if (!left.lastCallAttempt.has_value()) {
                return true;
            }
            if (!right.lastCallAttempt.has_value()) {
                return false;
            }
            return *left.lastCallAttempt < *right.lastCallAttempt;
        }
        if (left.stagnationAttempts != right.stagnationAttempts) {
            return left.stagnationAttempts > right.stagnationAttempts;
        }
        if (left.historicalDirectImprovement !=
            right.historicalDirectImprovement) {
            return left.historicalDirectImprovement >
                right.historicalDirectImprovement;
        }
        if (left.snapshot.populationBestObjective !=
            right.snapshot.populationBestObjective) {
            return left.snapshot.populationBestObjective <
                right.snapshot.populationBestObjective;
        }
        if (left.newCustomerSets != right.newCustomerSets) {
            return left.newCustomerSets > right.newCustomerSets;
        }
        return left.snapshot.population < right.snapshot.population;
    };
    for (const PopulationLocalSpDecision& decision : decisions) {
        if (!decision.eligible) {
            continue;
        }
        if (!selected.has_value() || preferred(decision, *selected)) {
            selected = decision;
        }
    }
    return selected;
}

void PopulationLocalSpController::start(
        const PopulationLocalSpDecision& decision) {
    if (!decision.eligible) {
        throw std::invalid_argument(
            "cannot start an ineligible population Local-SP decision");
    }
    validatePopulationLocalSpSnapshot(decision.snapshot);
    const auto found = states_.find(decision.snapshot.population);
    if (found == states_.end() ||
        found->second.phase == PopulationLocalSpPhase::Retired) {
        throw std::logic_error(
            "cannot start Local SP for a stale population token");
    }
    State& state = found->second;
    if (state.phase != PopulationLocalSpPhase::Eligible ||
        state.latestSnapshot != decision.snapshot ||
        state.latestGlobalInjectionCount !=
            decision.verifiedGlobalInjectionCount) {
        throw std::logic_error(
            "population Local-SP decision became stale before start");
    }
    state.phase = PopulationLocalSpPhase::Running;
    state.runningSnapshot = decision.snapshot;
    state.runningGlobalInjectionCount =
        decision.verifiedGlobalInjectionCount;
    state.populationBestAtLastCall =
        decision.snapshot.populationBestObjective;
    state.lastCallAttempt = decision.currentAttempt;
    ++state.callCount;
}

PopulationLocalSpCompletion PopulationLocalSpController::complete(
        const PopulationLocalSpToken population,
        const PopulationLocalSpSnapshot& currentSnapshot,
        const PopulationLocalSpOutcome& outcome) {
    validatePopulationLocalSpSnapshot(currentSnapshot);
    validateOutcome(outcome);
    PopulationLocalSpCompletion completion;
    const auto found = states_.find(population);
    if (found == states_.end()) {
        completion.effectiveOutcome =
            PopulationLocalSpOutcomeKind::DeferredStalePopulation;
        completion.phase = PopulationLocalSpPhase::Retired;
        completion.stalePopulation = true;
        return completion;
    }
    State& state = found->second;
    if (state.phase == PopulationLocalSpPhase::Retired) {
        state.cpuSeconds += outcome.outerCpuSeconds;
        state.lastStatus =
            PopulationLocalSpOutcomeKind::DeferredStalePopulation;
        completion.effectiveOutcome =
            PopulationLocalSpOutcomeKind::DeferredStalePopulation;
        completion.phase = PopulationLocalSpPhase::Retired;
        completion.stalePopulation = true;
        return completion;
    }
    if (state.phase != PopulationLocalSpPhase::Running ||
        !state.runningSnapshot.has_value()) {
        throw std::logic_error(
            "population Local-SP completion has no matching running call");
    }

    state.cpuSeconds += outcome.outerCpuSeconds;
    PopulationLocalSpOutcomeKind effective = outcome.kind;
    if (currentSnapshot.population != population ||
        !samePopulationLifetime(
            currentSnapshot, *state.runningSnapshot)) {
        effective =
            PopulationLocalSpOutcomeKind::DeferredStalePopulation;
        completion.stalePopulation = true;
    } else if (currentSnapshot != *state.runningSnapshot) {
        effective =
            PopulationLocalSpOutcomeKind::DeferredSnapshotChanged;
        completion.staleSnapshot = true;
    }
    completion.effectiveOutcome = effective;
    state.lastStatus = effective;

    if (effective == PopulationLocalSpOutcomeKind::Improved) {
        state.phase = PopulationLocalSpPhase::Improved;
        state.referenceSnapshot = *state.runningSnapshot;
        state.referenceGlobalInjectionCount =
            state.runningGlobalInjectionCount;
        state.consumedSnapshot = *state.runningSnapshot;
        state.meaningfulChangePending = false;
        state.directImprovement +=
            outcome.directObjectiveImprovement;
        ++state.productiveCallCount;
        completion.snapshotConsumed = true;
    } else if (isExhaustedOutcome(effective)) {
        state.phase = PopulationLocalSpPhase::Exhausted;
        state.referenceSnapshot = *state.runningSnapshot;
        state.referenceGlobalInjectionCount =
            state.runningGlobalInjectionCount;
        state.consumedSnapshot = *state.runningSnapshot;
        state.meaningfulChangePending = false;
        completion.snapshotConsumed = true;
    } else if (isDeferredOutcome(effective)) {
        state.phase = PopulationLocalSpPhase::Active;
    } else {
        throw std::logic_error(
            "unclassified population Local-SP completion");
    }

    if (completion.snapshotConsumed) {
        completion.consumedSnapshotId =
            populationLocalSpSnapshotId(*state.runningSnapshot);
    }
    state.runningSnapshot.reset();
    state.runningGlobalInjectionCount = 0U;
    completion.phase = state.phase;
    return completion;
}

void PopulationLocalSpController::retire(
        const PopulationLocalSpToken population) noexcept {
    const auto found = states_.find(population);
    if (found == states_.end()) {
        return;
    }
    found->second.phase = PopulationLocalSpPhase::Retired;
    found->second.runningSnapshot.reset();
}

void PopulationLocalSpController::retireAll() noexcept {
    for (auto& entry : states_) {
        entry.second.phase = PopulationLocalSpPhase::Retired;
        entry.second.runningSnapshot.reset();
    }
}

std::optional<PopulationLocalSpStateView>
PopulationLocalSpController::state(
        const PopulationLocalSpToken population) const {
    const auto found = states_.find(population);
    if (found == states_.end()) {
        return std::nullopt;
    }
    const State& state = found->second;
    PopulationLocalSpStateView view;
    view.population = state.population;
    view.configuration = state.configuration;
    view.phase = state.phase;
    view.populationBestAtLastCall = state.populationBestAtLastCall;
    view.lastPopulationBestImprovementAttempt =
        state.lastPopulationBestImprovementAttempt;
    view.observedConfigurationAppendCount =
        state.referenceSnapshot.configurationAppendCount;
    view.observedRepresentativeReplacementCount =
        state.referenceSnapshot
            .configurationRepresentativeReplacementCount;
    view.observedRepresentativeReplacementGain =
        state.referenceSnapshot
            .configurationRepresentativeReplacementGain;
    view.lastCallAttempt = state.lastCallAttempt;
    view.lastStatus = state.lastStatus;
    if (state.phase == PopulationLocalSpPhase::Exhausted) {
        view.exhaustedSnapshot = state.consumedSnapshot;
    }
    view.callCount = state.callCount;
    view.productiveCallCount = state.productiveCallCount;
    view.reactivationCount = state.reactivationCount;
    view.cpuSeconds = state.cpuSeconds;
    view.directImprovement = state.directImprovement;
    view.lastReactivationReason = state.lastReactivationReason;
    return view;
}

}  // namespace hgasp
