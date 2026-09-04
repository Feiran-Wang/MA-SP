#ifndef HGASP_CORE_ADAPTIVE_SP_CONTROLLER_H_
#define HGASP_CORE_ADAPTIVE_SP_CONTROLLER_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "NormalizedSpUtility.h"

enum class SpDispatch {
    None,
    Local,
    Global,
};

struct AdaptiveSpPolicy {
    int minimumInterval = 5000;
    // Prevent adjacent configuration scopes from launching Local probes on
    // consecutive offspring when they share the same global stagnation
    // signal. Per-scope backoff remains independent.
    int crossScopeLocalMinimumInterval = 500;
    int globalMinimumInterval = 10000;
    std::uint64_t localNewColumnThreshold = 250U;
    int localStagnationThreshold = 5000;
    int globalStagnationThreshold = 20000;
    int localZeroUtilityPatience = 2;
    int globalZeroUtilityPatience = 1;
    double utilityEmaWeight = 0.25;
    double highUtilityThreshold = 1.0e-3;
};

struct AdaptiveSpContext {
    int iteration = 0;
    int nonImprovingIterations = 0;
    std::uint64_t masterVersion = 0U;
    // Optional scheduling counters. When supplied, only appended columns
    // contribute to column novelty; cheaper representatives remain visible
    // as master improvements without being mislabeled as new columns.
    std::optional<std::uint64_t> masterAppendCount;
    std::optional<std::uint64_t>
        masterRepresentativeReplacementCount;
    std::optional<double>
        masterRepresentativeReplacementCostGain;
    // When set, Local novelty is measured against the exact depot-filtered
    // master view for localScopeKey. Global novelty always uses masterVersion.
    std::optional<std::uint64_t> localMasterVersion;
    std::optional<std::uint64_t> localMasterAppendCount;
    std::optional<std::uint64_t>
        localMasterRepresentativeReplacementCount;
    std::optional<double>
        localMasterRepresentativeReplacementCostGain;
    // Opaque depot-configuration identity. Scope zero preserves callers that
    // do not need configuration-local cooldown state.
    std::uint64_t localScopeKey = 0U;
    bool scheduledGlobal = false;
    // A pending hard mandatory final request bypasses novelty, retry interval,
    // minimum interval, remaining-CPU and relative-share guards exactly once.
    bool hardMandatoryGlobal = false;
    bool localEnabled = true;
    bool finalIntensificationPhase = false;
    double remainingCpuSeconds =
        std::numeric_limits<double>::infinity();
};

struct AdaptiveSpDecision {
    SpDispatch dispatch = SpDispatch::None;
    std::uint64_t newColumnEvents = 0U;
    std::uint64_t representativeReplacementEvents = 0U;
    double representativeReplacementCostGain = 0.0;
    std::uint64_t effectiveLocalThreshold = 0U;
    int effectiveLocalInterval = 0;
    std::uint64_t localScopeKey = 0U;
    std::string reason = "none";
};

struct AdaptiveSpOutcome {
    double objectiveImprovement = 0.0;
    // When present, objective utility is normalized by this pre-SP objective.
    // Legacy direct controller tests/callers may omit it and retain their
    // already-normalized improvement contract.
    std::optional<double> objectiveReference;
    double cpuSeconds = 0.0;
    // Marginal credit only: repeatedly observing an unchanged useful column
    // must report zero here.
    double incrementalUsefulColumnCredit = 0.0;
    // Structural novelty is a search-volume signal, not an optimization
    // reward.  It is tracked independently and never added to either utility.
    std::uint64_t structuralNovelty = 0U;
    bool satisfiesFinalIntensification = false;
    // A complete LP observation, or an explicit full-snapshot MIP-only call,
    // consumes a Local master version. Feedback-free failed attempts remain
    // retryable after the normal interval.
    bool masterSnapshotObserved = true;
    // Cumulative scheduling counters observed by the completed LP snapshot.
    // Legacy callers may leave these unset and retain masterVersion cadence.
    std::optional<std::uint64_t> observedAppendCount;
    std::optional<std::uint64_t>
        observedRepresentativeReplacementCount;
    std::optional<double>
        observedRepresentativeReplacementCostGain;
};

class AdaptiveSpController {
private:
    struct LocalControlState {
        int lastIteration = -1;
        std::uint64_t lastVersion = 0U;
        std::uint64_t lastAppendCount = 0U;
        std::uint64_t lastRepresentativeReplacementCount = 0U;
        double lastRepresentativeReplacementCostGain = 0.0;
        double lastCpuSeconds = 0.0;
        double objectiveUtilityEma = 0.0;
        double usefulColumnUtilityEma = 0.0;
        std::uint64_t lastStructuralNovelty = 0U;
        bool hasUtility = false;
        int zeroUtilityStreak = 0;
        int productiveWindow = 0;
        int budgetDeferredIteration = -1;
        // A failed Local prerequisite consumes only this configuration view.
        // It is neither a solve nor a utility observation. The exact
        // depot-filtered version must change before this scope is eligible
        // again.
        int preconditionDeferredIteration = -1;
        std::uint64_t preconditionDeferredVersion = 0U;
        bool hasPreconditionDeferral = false;
    };

public:
    AdaptiveSpController() = default;

    explicit AdaptiveSpController(AdaptiveSpPolicy policy)
        : policy_(policy) {
        validatePolicy();
    }

    void configure(AdaptiveSpPolicy policy) {
        policy_ = policy;
        validatePolicy();
    }

    AdaptiveSpDecision decide(
            const AdaptiveSpContext& context) {
        return decideP2(context);
    }

    void reportOutcome(
        const SpDispatch dispatch,
        const int iteration,
        const std::uint64_t masterVersion,
        const AdaptiveSpOutcome& outcome,
        const std::uint64_t localScopeKey = 0U) {
        reportImpl(
            dispatch,
            iteration,
            masterVersion,
            outcome.objectiveImprovement,
            outcome.objectiveReference,
            outcome.cpuSeconds,
            outcome.incrementalUsefulColumnCredit,
            outcome.structuralNovelty,
            outcome.satisfiesFinalIntensification,
            outcome.masterSnapshotObserved,
            outcome.observedAppendCount,
            outcome.observedRepresentativeReplacementCount,
            outcome.observedRepresentativeReplacementCostGain,
            localScopeKey);
    }

    void report(
        const SpDispatch dispatch,
        const int iteration,
        const std::uint64_t masterVersion,
        const double objectiveImprovement,
        const double cpuSeconds,
        const double usefulColumnUtility = 0.0,
        const bool satisfiesFinalIntensification = false,
        const std::uint64_t localScopeKey = 0U) {
        reportImpl(
            dispatch,
            iteration,
            masterVersion,
            objectiveImprovement,
            std::nullopt,
            cpuSeconds,
            usefulColumnUtility,
            0U,
            satisfiesFinalIntensification,
            true,
            std::nullopt,
            std::nullopt,
            std::nullopt,
            localScopeKey);
    }

private:
    void reportImpl(
        const SpDispatch dispatch,
        const int iteration,
        const std::uint64_t masterVersion,
        const double objectiveImprovement,
        const std::optional<double> objectiveReference,
        const double cpuSeconds,
        const double incrementalUsefulColumnCredit,
        const std::uint64_t structuralNovelty,
        const bool satisfiesFinalIntensification,
        const bool masterSnapshotObserved,
        const std::optional<std::uint64_t> observedAppendCount,
        const std::optional<std::uint64_t>
            observedRepresentativeReplacementCount,
        const std::optional<double>
            observedRepresentativeReplacementCostGain,
        const std::uint64_t localScopeKey) {
        if (dispatch == SpDispatch::None) {
            return;
        }
        const double observedCpuSeconds =
            std::max(0.0, cpuSeconds);
        const double objectiveUtility =
            objectiveReference.has_value()
                ? hgasp::normalizedObjectiveUtility(
                      *objectiveReference,
                      *objectiveReference -
                          std::max(0.0, objectiveImprovement),
                      observedCpuSeconds)
                : std::max(0.0, objectiveImprovement) /
                      std::max(1.0e-9, observedCpuSeconds);
        const double usefulColumnUtility =
            std::max(0.0, incrementalUsefulColumnCredit) /
            std::max(1.0e-9, observedCpuSeconds);
        // Only an incremental useful-column observation is useful; repeatedly
        // observing the same column reports zero credit and keeps the
        // exponential interval backoff in force.
        const bool productive =
            objectiveImprovement > 0.0 ||
            incrementalUsefulColumnCredit > 0.0;

        if (dispatch == SpDispatch::Global) {
            lastGlobalIteration_ = iteration;
            lastGlobalVersion_ = masterVersion;
            lastGlobalAppendCount_ =
                observedAppendCount.value_or(masterVersion);
            lastGlobalRepresentativeReplacementCount_ =
                observedRepresentativeReplacementCount.value_or(0U);
            lastGlobalRepresentativeReplacementCostGain_ =
                observedRepresentativeReplacementCostGain.value_or(0.0);
            lastGlobalCpuSeconds_ = observedCpuSeconds;
            globalBudgetDeferredIteration_ = -1;
            lastGlobalStructuralNovelty_ = structuralNovelty;
            updateUtilityEma(
                hasGlobalUtility_,
                globalObjectiveUtilityEma_,
                globalUsefulColumnUtilityEma_,
                objectiveUtility,
                usefulColumnUtility);
            if (satisfiesFinalIntensification) {
                finalIntensificationIssued_ = true;
            }
            if (!productive) {
                globalZeroUtilityStreak_ =
                    std::min(globalZeroUtilityStreak_ + 1, 8);
            } else {
                globalZeroUtilityStreak_ = 0;
            }
            return;
        }

        LocalControlState& local =
            localStateFor(localScopeKey);
        local.lastIteration = iteration;
        lastAnyLocalIteration_ = iteration;
        if (masterSnapshotObserved) {
            local.lastVersion = masterVersion;
            local.lastAppendCount =
                observedAppendCount.value_or(masterVersion);
            local.lastRepresentativeReplacementCount =
                observedRepresentativeReplacementCount.value_or(0U);
            local.lastRepresentativeReplacementCostGain =
                observedRepresentativeReplacementCostGain.value_or(0.0);
        }
        local.lastCpuSeconds = observedCpuSeconds;
        local.budgetDeferredIteration = -1;
        local.hasPreconditionDeferral = false;
        local.lastStructuralNovelty = structuralNovelty;
        // Consume a previously earned short window only after Local work
        // actually started and reported an outcome for this same scope.
        local.productiveWindow =
            std::max(0, local.productiveWindow - 1);
        updateUtilityEma(
            local.hasUtility,
            local.objectiveUtilityEma,
            local.usefulColumnUtilityEma,
            objectiveUtility,
            usefulColumnUtility);
        if (!productive) {
            local.zeroUtilityStreak =
                std::min(local.zeroUtilityStreak + 1, 8);
            local.productiveWindow = 0;
        } else {
            local.zeroUtilityStreak = 0;
            // A productive Local call earns one shorter-novelty window. A
            // later zero-result immediately starts backoff in this scope.
            local.productiveWindow = 1;
        }
    }

public:
    double utilityEma() const noexcept {
        return hasGlobalUtility_
            ? globalUtilityEma()
            : localUtilityEma();
    }

    double localUtilityEma() const noexcept {
        double maximum = 0.0;
        for (const auto& entry : localStates_) {
            maximum = std::max(
                maximum,
                localUtilityEma(entry.second));
        }
        return maximum;
    }

    double localUtilityEma(
            const std::uint64_t localScopeKey) const noexcept {
        const auto found = localStates_.find(localScopeKey);
        return found == localStates_.end()
            ? 0.0
            : localUtilityEma(found->second);
    }

    double globalUtilityEma() const noexcept {
        return std::max(
            globalObjectiveUtilityEma_,
            globalUsefulColumnUtilityEma_);
    }

    void reportBudgetDeferral(
            const SpDispatch dispatch,
            const int iteration,
            const std::uint64_t localScopeKey = 0U) {
        if (dispatch == SpDispatch::Global) {
            globalBudgetDeferredIteration_ = iteration;
        } else if (dispatch == SpDispatch::Local) {
            localStateFor(localScopeKey)
                .budgetDeferredIteration = iteration;
        }
    }

    void reportLocalPreconditionDeferral(
            const int iteration,
            const std::uint64_t localMasterVersion,
            const std::uint64_t localScopeKey = 0U,
            const std::optional<std::uint64_t> localAppendCount =
                std::nullopt,
            const std::optional<std::uint64_t>
                localRepresentativeReplacementCount = std::nullopt,
            const std::optional<double>
                localRepresentativeReplacementCostGain = std::nullopt) {
        LocalControlState& local =
            localStateFor(localScopeKey);
        local.preconditionDeferredIteration = iteration;
        local.preconditionDeferredVersion =
            localMasterVersion;
        local.hasPreconditionDeferral = true;
        // Consume only the observed configuration view. Do not change the
        // last execution iteration, CPU estimate, utility EMA, productive
        // window, or zero-utility streak.
        local.lastVersion = localMasterVersion;
        local.lastAppendCount =
            localAppendCount.value_or(localMasterVersion);
        local.lastRepresentativeReplacementCount =
            localRepresentativeReplacementCount.value_or(0U);
        local.lastRepresentativeReplacementCostGain =
            localRepresentativeReplacementCostGain.value_or(0.0);
    }

    std::uint64_t lastDispatchVersion() const noexcept {
        std::uint64_t version = lastGlobalVersion_;
        for (const auto& entry : localStates_) {
            version = std::max(
                version, entry.second.lastVersion);
        }
        return version;
    }

    void resetLocalScope(
            const std::uint64_t localScopeKey) noexcept {
        localStates_.erase(localScopeKey);
    }

    // Retiring a depot-configuration scope prevents its cooldown, utility and
    // deferrals from leaking into a later local controller state while leaving
    // the shared Global controller state untouched.
    void retireLocalScope(
            const std::uint64_t localScopeKey) noexcept {
        resetLocalScope(localScopeKey);
    }

    std::size_t localScopeCount() const noexcept {
        return localStates_.size();
    }

    bool finalIntensificationPending() const noexcept {
        return !finalIntensificationIssued_;
    }

    // Run-level Final Global completion can be owned by the static phase
    // policy or the hard-TW gate rather than by this controller's own
    // dispatch. Synchronize that authoritative completion explicitly so the
    // adaptive path cannot issue a duplicate final solve.
    void markFinalIntensificationSatisfied() noexcept {
        finalIntensificationIssued_ = true;
    }

private:
    AdaptiveSpDecision decideP2(
            const AdaptiveSpContext& context) {
        AdaptiveSpDecision decision;
        LocalControlState& localState =
            localStateFor(context.localScopeKey);
        decision.localScopeKey = context.localScopeKey;
        const auto noveltySince =
            [](const std::uint64_t currentVersion,
               const std::uint64_t previousVersion) {
                return currentVersion >= previousVersion
                    ? currentVersion - previousVersion
                    : currentVersion;
            };
        const std::uint64_t localMasterVersion =
            context.localMasterVersion.value_or(
                context.masterVersion);
        if (localState.hasPreconditionDeferral &&
            localMasterVersion !=
                localState.preconditionDeferredVersion) {
            localState.hasPreconditionDeferral = false;
        }
        const bool unchangedDeferredPrecondition =
            localState.hasPreconditionDeferral &&
            localMasterVersion ==
                localState.preconditionDeferredVersion;
        const bool globalCandidate =
            context.scheduledGlobal ||
            context.finalIntensificationPhase ||
            context.nonImprovingIterations >=
                policy_.globalStagnationThreshold;
        decision.newColumnEvents =
            globalCandidate
                ? noveltySince(
                      context.masterVersion,
                      lastGlobalVersion_)
                : noveltySince(
                      localMasterVersion,
                      localState.lastVersion);
        decision.effectiveLocalThreshold =
            policy_.localNewColumnThreshold;
        decision.effectiveLocalInterval =
            effectiveLocalInterval(localState);
        if (context.hardMandatoryGlobal) {
            if (finalIntensificationIssued_) {
                decision.reason =
                    "final_intensification_already_issued";
                return decision;
            }
            decision.dispatch = SpDispatch::Global;
            decision.reason = context.scheduledGlobal
                ? "scheduled_final_global"
                : "final_intensification";
            return decision;
        }
        if (!globalCandidate &&
            unchangedDeferredPrecondition) {
            decision.reason =
                "local_precondition_unchanged";
            return decision;
        }
        if (globalCandidate &&
            globalBudgetDeferredIteration_ >= 0 &&
            context.iteration - globalBudgetDeferredIteration_ <
                policy_.globalMinimumInterval) {
            decision.reason = "global_budget_retry_interval";
            return decision;
        }
        if (!globalCandidate &&
            localState.budgetDeferredIteration >= 0 &&
            context.iteration -
                    localState.budgetDeferredIteration <
                policy_.minimumInterval) {
            decision.reason = "local_budget_retry_interval";
            return decision;
        }
        if (decision.newColumnEvents == 0U) {
            decision.reason = "unchanged_master";
            return decision;
        }
        const double relevantCpuSeconds =
            globalCandidate
                ? lastGlobalCpuSeconds_
                : localState.lastCpuSeconds;
        if (relevantCpuSeconds > 0.0 &&
            std::isfinite(context.remainingCpuSeconds) &&
            context.remainingCpuSeconds <=
                1.25 * relevantCpuSeconds) {
            decision.reason = "insufficient_cpu_budget";
            return decision;
        }
        const int sinceLastGlobal =
            lastGlobalIteration_ < 0
                ? context.iteration
                : context.iteration - lastGlobalIteration_;
        if (globalCandidate &&
            lastGlobalIteration_ >= 0 &&
            sinceLastGlobal < effectiveGlobalInterval()) {
            decision.reason = "global_backoff";
            return decision;
        }
        if (context.scheduledGlobal) {
            decision.dispatch = SpDispatch::Global;
            decision.reason = "scheduled_global_with_novelty";
            return decision;
        }
        if (context.finalIntensificationPhase) {
            if (finalIntensificationIssued_) {
                decision.reason =
                    "final_intensification_already_issued";
                return decision;
            }
            decision.dispatch = SpDispatch::Global;
            decision.reason = "final_intensification";
            return decision;
        }
        if (context.nonImprovingIterations >=
                policy_.globalStagnationThreshold) {
            decision.dispatch = SpDispatch::Global;
            decision.reason = "global_stagnation";
            return decision;
        }
        const int sinceLastLocal =
            localState.lastIteration < 0
                ? context.iteration
                : context.iteration -
                      localState.lastIteration;
        if (sinceLastLocal < decision.effectiveLocalInterval) {
            decision.reason =
                decision.effectiveLocalInterval >
                        policy_.minimumInterval
                    ? "local_zero_utility_cooldown"
                    : "local_minimum_interval";
            return decision;
        }
        if (context.localEnabled &&
            (decision.newColumnEvents >=
                 decision.effectiveLocalThreshold ||
             context.nonImprovingIterations >=
                 policy_.localStagnationThreshold)) {
            decision.dispatch = SpDispatch::Local;
            decision.reason =
                context.nonImprovingIterations >=
                        policy_.localStagnationThreshold
                    ? "local_stagnation"
                    : "column_novelty";
            return decision;
        }
        decision.reason = "insufficient_novelty";
        return decision;
    }

    void validatePolicy() const {
        if (policy_.minimumInterval < 1 ||
            policy_.crossScopeLocalMinimumInterval < 1 ||
            policy_.globalMinimumInterval < 1 ||
            policy_.localNewColumnThreshold < 1U ||
            policy_.localStagnationThreshold < 1 ||
            policy_.globalStagnationThreshold <
                policy_.localStagnationThreshold ||
            policy_.localZeroUtilityPatience < 1 ||
            policy_.globalZeroUtilityPatience < 1 ||
            !std::isfinite(policy_.utilityEmaWeight) ||
            policy_.utilityEmaWeight <= 0.0 ||
            policy_.utilityEmaWeight > 1.0 ||
            !std::isfinite(policy_.highUtilityThreshold) ||
            policy_.highUtilityThreshold <= 0.0) {
            throw std::invalid_argument(
                "invalid adaptive SP trigger policy");
        }
    }

    LocalControlState& localStateFor(
            const std::uint64_t localScopeKey) {
        return localStates_[localScopeKey];
    }

    static double localUtilityEma(
            const LocalControlState& local) noexcept {
        return std::max(
            local.objectiveUtilityEma,
            local.usefulColumnUtilityEma);
    }

    void updateUtilityEma(
            bool& hasUtility,
            double& objectiveUtilityEma,
            double& usefulColumnUtilityEma,
            const double objectiveUtility,
            const double usefulColumnUtility) const noexcept {
        if (!hasUtility) {
            objectiveUtilityEma = objectiveUtility;
            usefulColumnUtilityEma = usefulColumnUtility;
            hasUtility = true;
            return;
        }
        objectiveUtilityEma =
            policy_.utilityEmaWeight * objectiveUtility +
            (1.0 - policy_.utilityEmaWeight) *
                objectiveUtilityEma;
        usefulColumnUtilityEma =
            policy_.utilityEmaWeight * usefulColumnUtility +
            (1.0 - policy_.utilityEmaWeight) *
                usefulColumnUtilityEma;
    }

    int localBackoffExponent(
            const LocalControlState& local) const noexcept {
        return std::min(
            std::max(
                0,
                local.zeroUtilityStreak -
                    policy_.localZeroUtilityPatience + 1),
            3);
    }

    std::uint64_t effectiveLocalThreshold(
            const LocalControlState& local) const noexcept {
        if (local.productiveWindow > 0 &&
            local.hasUtility &&
            localUtilityEma(local) >=
                policy_.highUtilityThreshold) {
            return std::max<std::uint64_t>(
                1U,
                policy_.localNewColumnThreshold / 2U);
        }
        const int exponent = localBackoffExponent(local);
        if (exponent == 0) {
            return policy_.localNewColumnThreshold;
        }
        const std::uint64_t maximum =
            std::numeric_limits<std::uint64_t>::max();
        return policy_.localNewColumnThreshold >
                       (maximum >> exponent)
            ? maximum
            : policy_.localNewColumnThreshold << exponent;
    }

    int effectiveLocalInterval(
            const LocalControlState& local) const noexcept {
        const int exponent = localBackoffExponent(local);
        const long long scaled =
            static_cast<long long>(
                policy_.minimumInterval) << exponent;
        return scaled >=
                    static_cast<long long>(
                        std::numeric_limits<int>::max())
            ? std::numeric_limits<int>::max()
            : static_cast<int>(scaled);
    }

    int effectiveGlobalInterval() const noexcept {
        const int base = policy_.globalMinimumInterval;
        const int exponent = std::min(
            std::max(
                0,
                globalZeroUtilityStreak_ -
                    policy_.globalZeroUtilityPatience + 1),
            3);
        const long long scaled =
            static_cast<long long>(base) << exponent;
        return scaled >=
                    static_cast<long long>(
                        std::numeric_limits<int>::max())
            ? std::numeric_limits<int>::max()
            : static_cast<int>(scaled);
    }

    AdaptiveSpPolicy policy_;
    std::unordered_map<std::uint64_t, LocalControlState>
        localStates_;
    int lastGlobalIteration_ = -1;
    int lastAnyLocalIteration_ = -1;
    std::uint64_t lastGlobalVersion_ = 0U;
    std::uint64_t lastGlobalAppendCount_ = 0U;
    std::uint64_t lastGlobalRepresentativeReplacementCount_ = 0U;
    double lastGlobalRepresentativeReplacementCostGain_ = 0.0;
    double lastGlobalCpuSeconds_ = 0.0;
    double globalObjectiveUtilityEma_ = 0.0;
    double globalUsefulColumnUtilityEma_ = 0.0;
    std::uint64_t lastGlobalStructuralNovelty_ = 0U;
    bool hasGlobalUtility_ = false;
    int globalZeroUtilityStreak_ = 0;
    int globalBudgetDeferredIteration_ = -1;
    bool finalIntensificationIssued_ = false;
};

#endif  // HGASP_CORE_ADAPTIVE_SP_CONTROLLER_H_
