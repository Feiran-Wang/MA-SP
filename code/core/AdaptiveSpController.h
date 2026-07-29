#ifndef HGASP_CORE_ADAPTIVE_SP_CONTROLLER_H_
#define HGASP_CORE_ADAPTIVE_SP_CONTROLLER_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

enum class SpDispatch {
    None,
    Local,
    Global,
};

struct AdaptiveSpPolicy {
    int minimumInterval = 5000;
    std::uint64_t localNewColumnThreshold = 250U;
    int localStagnationThreshold = 5000;
    int globalStagnationThreshold = 20000;
    double utilityEmaWeight = 0.25;
    double highUtilityThreshold = 1.0e-3;
};

struct AdaptiveSpContext {
    int iteration = 0;
    int nonImprovingIterations = 0;
    std::uint64_t masterVersion = 0U;
    bool scheduledGlobal = false;
    bool localEnabled = true;
    bool finalIntensificationPhase = false;
    double remainingCpuSeconds =
        std::numeric_limits<double>::infinity();
};

struct AdaptiveSpDecision {
    SpDispatch dispatch = SpDispatch::None;
    std::uint64_t newColumnEvents = 0U;
    std::uint64_t effectiveLocalThreshold = 0U;
    std::string reason = "none";
};

class AdaptiveSpController {
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
        AdaptiveSpDecision decision;
        const auto noveltySince =
            [&](const std::uint64_t version) {
                return context.masterVersion >= version
                    ? context.masterVersion - version
                    : context.masterVersion;
            };
        const std::uint64_t localNovelty =
            noveltySince(lastLocalVersion_);
        const std::uint64_t globalNovelty =
            noveltySince(lastGlobalVersion_);
        const bool globalCandidate =
            context.scheduledGlobal ||
            context.finalIntensificationPhase ||
            context.nonImprovingIterations >=
                policy_.globalStagnationThreshold;
        decision.newColumnEvents =
            globalCandidate ? globalNovelty : localNovelty;
        decision.effectiveLocalThreshold =
            effectiveLocalThreshold();

        if (decision.newColumnEvents == 0U) {
            decision.reason = "unchanged_master";
            return decision;
        }
        const double relevantCpuSeconds =
            context.scheduledGlobal ||
                    context.finalIntensificationPhase ||
                    context.nonImprovingIterations >=
                        policy_.globalStagnationThreshold
                ? lastGlobalCpuSeconds_
                : lastLocalCpuSeconds_;
        if (relevantCpuSeconds > 0.0 &&
            std::isfinite(context.remainingCpuSeconds) &&
            context.remainingCpuSeconds <=
                1.25 * relevantCpuSeconds) {
            decision.reason = "insufficient_cpu_budget";
            return decision;
        }

        // A scheduled global event is a maximum-interval guarantee. It may
        // bypass the local minimum interval when genuinely new columns exist.
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
            finalIntensificationIssued_ = true;
            decision.dispatch = SpDispatch::Global;
            decision.reason = "final_intensification";
            return decision;
        }

        const int sinceLastGlobal =
            lastGlobalIteration_ < 0
                ? context.iteration
                : context.iteration - lastGlobalIteration_;
        if (context.nonImprovingIterations >=
                policy_.globalStagnationThreshold) {
            if (sinceLastGlobal >= effectiveGlobalInterval()) {
                decision.dispatch = SpDispatch::Global;
                decision.reason = "global_stagnation";
                return decision;
            }
            decision.reason = "global_backoff";
            return decision;
        }

        const int sinceLastLocal =
            lastLocalIteration_ < 0
                ? context.iteration
                : context.iteration - lastLocalIteration_;
        if (sinceLastLocal < policy_.minimumInterval) {
            decision.reason = "local_minimum_interval";
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

    void report(
        const SpDispatch dispatch,
        const int iteration,
        const std::uint64_t masterVersion,
        const double objectiveImprovement,
        const double cpuSeconds) {
        if (dispatch == SpDispatch::None) {
            return;
        }
        int* lastIteration = dispatch == SpDispatch::Global
            ? &lastGlobalIteration_
            : &lastLocalIteration_;
        std::uint64_t* lastVersion =
            dispatch == SpDispatch::Global
                ? &lastGlobalVersion_
                : &lastLocalVersion_;
        double* lastCpuSeconds =
            dispatch == SpDispatch::Global
                ? &lastGlobalCpuSeconds_
                : &lastLocalCpuSeconds_;
        double* utilityEma =
            dispatch == SpDispatch::Global
                ? &globalUtilityEma_
                : &localUtilityEma_;
        bool* hasUtility = dispatch == SpDispatch::Global
            ? &hasGlobalUtility_
            : &hasLocalUtility_;

        *lastIteration = iteration;
        *lastVersion = masterVersion;
        *lastCpuSeconds = std::max(0.0, cpuSeconds);
        const double utility =
            objectiveImprovement /
            std::max(1.0e-9, *lastCpuSeconds);
        if (!*hasUtility) {
            *utilityEma = utility;
            *hasUtility = true;
        } else {
            *utilityEma =
                policy_.utilityEmaWeight * utility +
                (1.0 - policy_.utilityEmaWeight) *
                    *utilityEma;
        }
        if (dispatch == SpDispatch::Global) {
            if (objectiveImprovement <= 0.0) {
                globalZeroUtilityStreak_ =
                    std::min(globalZeroUtilityStreak_ + 1, 8);
            } else {
                globalZeroUtilityStreak_ = 0;
            }
        }
    }

    double utilityEma() const noexcept {
        return hasGlobalUtility_
            ? globalUtilityEma_
            : localUtilityEma_;
    }

    double localUtilityEma() const noexcept {
        return localUtilityEma_;
    }

    double globalUtilityEma() const noexcept {
        return globalUtilityEma_;
    }

    std::uint64_t lastDispatchVersion() const noexcept {
        return std::max(lastLocalVersion_, lastGlobalVersion_);
    }

private:
    void validatePolicy() const {
        if (policy_.minimumInterval < 1 ||
            policy_.localNewColumnThreshold < 1U ||
            policy_.localStagnationThreshold < 1 ||
            policy_.globalStagnationThreshold <
                policy_.localStagnationThreshold ||
            !std::isfinite(policy_.utilityEmaWeight) ||
            policy_.utilityEmaWeight <= 0.0 ||
            policy_.utilityEmaWeight > 1.0 ||
            !std::isfinite(policy_.highUtilityThreshold) ||
            policy_.highUtilityThreshold <= 0.0) {
            throw std::invalid_argument(
                "invalid adaptive SP trigger policy");
        }
    }

    std::uint64_t effectiveLocalThreshold() const noexcept {
        if (!hasLocalUtility_) {
            return policy_.localNewColumnThreshold;
        }
        if (localUtilityEma_ <= 0.0) {
            return policy_.localNewColumnThreshold >
                           std::numeric_limits<std::uint64_t>::max() / 2U
                ? std::numeric_limits<std::uint64_t>::max()
                : 2U * policy_.localNewColumnThreshold;
        }
        if (localUtilityEma_ >= policy_.highUtilityThreshold) {
            return std::max<std::uint64_t>(
                1U,
                policy_.localNewColumnThreshold / 2U);
        }
        return policy_.localNewColumnThreshold;
    }

    int effectiveGlobalInterval() const noexcept {
        const int base = std::max(
            policy_.minimumInterval,
            policy_.globalStagnationThreshold);
        const int exponent =
            std::min(globalZeroUtilityStreak_, 3);
        const long long scaled =
            static_cast<long long>(base) << exponent;
        return scaled >=
                    static_cast<long long>(
                        std::numeric_limits<int>::max())
            ? std::numeric_limits<int>::max()
            : static_cast<int>(scaled);
    }

    AdaptiveSpPolicy policy_;
    int lastLocalIteration_ = -1;
    int lastGlobalIteration_ = -1;
    std::uint64_t lastLocalVersion_ = 0U;
    std::uint64_t lastGlobalVersion_ = 0U;
    double lastLocalCpuSeconds_ = 0.0;
    double lastGlobalCpuSeconds_ = 0.0;
    double localUtilityEma_ = 0.0;
    double globalUtilityEma_ = 0.0;
    bool hasLocalUtility_ = false;
    bool hasGlobalUtility_ = false;
    int globalZeroUtilityStreak_ = 0;
    bool finalIntensificationIssued_ = false;
};

#endif  // HGASP_CORE_ADAPTIVE_SP_CONTROLLER_H_
