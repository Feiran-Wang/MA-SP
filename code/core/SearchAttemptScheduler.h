#ifndef HGASP_CORE_SEARCH_ATTEMPT_SCHEDULER_H_
#define HGASP_CORE_SEARCH_ATTEMPT_SCHEDULER_H_

#include <limits>
#include <stdexcept>

// Owns the state that must advance exactly once for every completed search
// attempt.  Keeping the SP request latched decouples the trigger schedule from
// the outcome of the attempt (feasible child, rejected child, or zero-child
// EAX batch).
class SearchAttemptScheduler {
public:
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

    void completeAttempt(
        int& iteration,
        const int triggerFrequency,
        const int triggerOffset) {
        if (iteration == std::numeric_limits<int>::max()) {
            throw std::overflow_error(
                "search-attempt counter exceeded its integer range");
        }

        const int completedIteration = iteration + 1;
        const bool shouldLatch =
            isSpScheduled(
                completedIteration,
                triggerFrequency,
                triggerOffset);
        ++iteration;
        spTriggerPending_ = spTriggerPending_ || shouldLatch;
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

private:
    bool spTriggerPending_ = false;
};

#endif  // HGASP_CORE_SEARCH_ATTEMPT_SCHEDULER_H_
