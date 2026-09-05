#ifndef HGASP_CORE_CHALLENGER_SCHEDULING_STATE_H_
#define HGASP_CORE_CHALLENGER_SCHEDULING_STATE_H_

#include <cstdint>
#include <stdexcept>

namespace hgasp {

// Generation-local accounting for the existing protected-primary quota.
// This class deliberately does not decide which turns are protected; that
// frozen policy remains owned by DynamicChallengerPolicy.
struct ChallengerSchedulingState {
    std::uint64_t generation = 0U;
    std::uint64_t opportunitiesSinceTrialStart = 0U;
    std::uint64_t protectedPrimarySelections = 0U;

    void beginGeneration(const std::uint64_t newGeneration) {
        if (newGeneration == 0U) {
            throw std::invalid_argument(
                "challenger scheduling generation must be non-zero");
        }
        generation = newGeneration;
        opportunitiesSinceTrialStart = 0U;
        protectedPrimarySelections = 0U;
    }

    std::uint64_t recordOpportunity() noexcept {
        return ++opportunitiesSinceTrialStart;
    }

    void recordProtectedPrimarySelection() noexcept {
        ++protectedPrimarySelections;
    }
};

}  // namespace hgasp

#endif  // HGASP_CORE_CHALLENGER_SCHEDULING_STATE_H_
