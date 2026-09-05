#ifndef HGASP_CORE_SEARCH_ATTEMPT_SCHEDULER_H_
#define HGASP_CORE_SEARCH_ATTEMPT_SCHEDULER_H_

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

enum class RestartControlAction {
    None,
    FullRestart,
    SuppressedByRemainingBudget,
};

struct RestartControlContext {
    int nonImprovingIterations = 0;
    int stagnationThreshold = 0;
    int remainingIterations = 0;
    double remainingCpuSeconds =
        std::numeric_limits<double>::infinity();
    int minimumRemainingIterations = 0;
    double minimumRemainingCpuSeconds = 0.0;
};

struct RestartControlDecision {
    RestartControlAction action = RestartControlAction::None;
    bool iterationBudgetInsufficient = false;
    bool cpuBudgetInsufficient = false;
};

// Separates physical search attempts from HGAMP-compatible productive
// iterations. Every generated/rejected/degenerate attempt is retained for
// audit, while the public search horizon advances only after a feasible
// offspring reaches the refinement/admission boundary.
class SearchAttemptScheduler {
public:
	static std::uint64_t productionNonproductiveAttemptLimit(
			const int productiveIterationLimit) noexcept {
		// One complete requested productive horizon without a feasible refined
		// child is a deterministic fail-closed boundary. This preserves the
		// HGAMP productive counter while preventing an iteration-only run from
		// looping forever when no process-CPU limit was supplied.
		return static_cast<std::uint64_t>(
			productiveIterationLimit > 0 ? productiveIterationLimit : 1);
	}

	explicit SearchAttemptScheduler(
			const std::uint64_t maximumConsecutiveNonproductiveAttempts =
				std::numeric_limits<std::uint64_t>::max())
		: maximumConsecutiveNonproductiveAttempts_(
			  maximumConsecutiveNonproductiveAttempts) {
		if (maximumConsecutiveNonproductiveAttempts_ == 0U) {
			throw std::invalid_argument(
				"nonproductive-attempt liveness guard must be positive");
		}
	}
	static bool isRestartIterationReachable(
			const int stagnationThreshold,
			const int minimumRemainingIterations,
			const int iterationLimit) {
		if (stagnationThreshold < 0 ||
			minimumRemainingIterations < 0 ||
			iterationLimit < 0) {
			throw std::invalid_argument(
				"restart reachability requires nonnegative limits");
		}
		return static_cast<long long>(stagnationThreshold) + 1LL +
			static_cast<long long>(minimumRemainingIterations) <=
			static_cast<long long>(iterationLimit);
	}

    static bool isSpScheduled(
        const int iteration,
        const int triggerFrequency,
        const int triggerOffset) {
        if (triggerFrequency <= 0) {
            throw std::invalid_argument(
                "SP trigger frequency must be positive");
        }
        if (triggerOffset < 0 || triggerOffset >= triggerFrequency) {
            throw std::invalid_argument(
                "SP trigger offset must be non-negative and smaller than "
                "the trigger frequency");
        }

        const long long firstTrigger =
            static_cast<long long>(triggerFrequency) + triggerOffset;
        if (static_cast<long long>(iteration) < firstTrigger) {
            return false;
        }
        return (static_cast<long long>(iteration) - triggerOffset) %
                   triggerFrequency ==
               0;
    }

    bool completeAttempt(
        int& productiveIteration,
        const int triggerFrequency,
        const int triggerOffset,
        const bool productiveFeasibleOffspring) {
        if (completedAttemptCount_ ==
            std::numeric_limits<std::uint64_t>::max()) {
            throw std::overflow_error(
                "search-attempt counter exceeded its integer range");
        }
        ++completedAttemptCount_;
        if (!productiveFeasibleOffspring) {
            if (consecutiveNonproductiveAttemptCount_ ==
                    std::numeric_limits<std::uint64_t>::max() ||
                ++consecutiveNonproductiveAttemptCount_ >
                    maximumConsecutiveNonproductiveAttempts_) {
                throw std::runtime_error(
                    "search produced no feasible offspring before the "
                    "nonproductive-attempt liveness guard");
            }
            return false;
        }
        if (productiveIteration == std::numeric_limits<int>::max() ||
            productiveFeasibleOffspringCount_ ==
                std::numeric_limits<std::uint64_t>::max()) {
            throw std::overflow_error(
                "productive-offspring counter exceeded its integer range");
        }

        const int completedIteration = productiveIteration + 1;
        const bool shouldLatch =
            isSpScheduled(
                completedIteration,
                triggerFrequency,
                triggerOffset);
        ++productiveIteration;
        ++productiveFeasibleOffspringCount_;
        consecutiveNonproductiveAttemptCount_ = 0U;
        spTriggerPending_ = spTriggerPending_ || shouldLatch;
        return true;
    }

    std::uint64_t completedAttemptCount() const noexcept {
        return completedAttemptCount_;
    }

    std::uint64_t productiveFeasibleOffspringCount() const noexcept {
        return productiveFeasibleOffspringCount_;
    }

    std::uint64_t consecutiveNonproductiveAttemptCount() const noexcept {
        return consecutiveNonproductiveAttemptCount_;
    }

    // Finalizes the stagnation counter exactly once after every completed
    // attempt. The caller compares the restart incumbent from immediately
    // before the attempt with the final restart incumbent after all scheduled
    // work (HGA and, when dispatched, SP). This gives both improvement sources
    // identical semantics.
    void settleNonImprovement(
        int& nonImprovingIterations,
        const bool restartBestImproved) const {
        if (restartBestImproved) {
            nonImprovingIterations = 0;
            return;
        }
        if (nonImprovingIterations ==
            std::numeric_limits<int>::max()) {
            throw std::overflow_error(
                "non-improvement counter exceeded its integer range");
        }
        ++nonImprovingIterations;
    }

    bool hasPendingSpTrigger() const noexcept {
        return spTriggerPending_;
    }

    bool consumeSpTrigger() noexcept {
        if (!spTriggerPending_) {
            return false;
        }
        spTriggerPending_ = false;
        return true;
    }

    // Execution acknowledgement is intentionally separate from reading the
    // pending bit. A controller or SP budget guard may decline a proposal,
    // in which case the event must remain latched for a later safe point.
    bool acknowledgeGlobalExecution(
            const bool executionStarted) noexcept {
        return executionStarted
            ? consumeSpTrigger()
            : false;
    }

    // A full population restart replaces every parent generation. Children
    // still buffered from the pre-restart EAX batch must therefore be
    // discarded before the caller selects another parent pair.
    void observePopulationRestart() noexcept {
        offspringBatchInvalidated_ = true;
    }

    bool consumeOffspringBatchInvalidation() noexcept {
        const bool invalidated = offspringBatchInvalidated_;
        offspringBatchInvalidated_ = false;
        return invalidated;
    }

    // Classifies a restart request without mutating the true stagnation
    // counter. A remaining-budget rejection is an observation, not an
    // improvement, so callers must keep the counter intact. Keeping this
    // decision separate also leaves a narrow seam for a future partial-restart
    // policy without changing attempt accounting.
    static RestartControlDecision decideRestart(
            const RestartControlContext& context) {
        if (context.nonImprovingIterations < 0 ||
            context.stagnationThreshold < 0 ||
            context.minimumRemainingIterations < 0 ||
            !std::isfinite(
                context.minimumRemainingCpuSeconds) ||
            context.minimumRemainingCpuSeconds < 0.0 ||
            std::isnan(context.remainingCpuSeconds)) {
            throw std::invalid_argument(
                "restart control contains an invalid budget value");
        }

        RestartControlDecision decision;
        if (context.nonImprovingIterations <=
            context.stagnationThreshold) {
            return decision;
        }

        decision.iterationBudgetInsufficient =
            context.remainingIterations <
            context.minimumRemainingIterations;
        decision.cpuBudgetInsufficient =
            context.remainingCpuSeconds <
            context.minimumRemainingCpuSeconds;
        decision.action =
            decision.iterationBudgetInsufficient ||
                    decision.cpuBudgetInsufficient
                ? RestartControlAction::
                      SuppressedByRemainingBudget
                : RestartControlAction::FullRestart;
        return decision;
    }

private:
    std::uint64_t completedAttemptCount_ = 0U;
    std::uint64_t productiveFeasibleOffspringCount_ = 0U;
    std::uint64_t consecutiveNonproductiveAttemptCount_ = 0U;
    std::uint64_t maximumConsecutiveNonproductiveAttempts_ = 0U;
    bool spTriggerPending_ = false;
    bool offspringBatchInvalidated_ = false;
};

#endif  // HGASP_CORE_SEARCH_ATTEMPT_SCHEDULER_H_
