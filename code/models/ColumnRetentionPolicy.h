#ifndef HGASP_MODELS_COLUMN_RETENTION_POLICY_H_
#define HGASP_MODELS_COLUMN_RETENTION_POLICY_H_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace hgasp {

struct MasterColumnEvaluationState {
    std::uint64_t insertedMasterVersion = 0U;
    std::uint64_t lastLpVersion = 0U;
    bool evaluatedByLp = false;
};

inline std::size_t newColumnGraceCapacity(
        const std::size_t maximumPoolSize) noexcept {
    return std::min<std::size_t>(1000U, maximumPoolSize / 10U);
}

struct GraceCandidate {
    std::size_t index = 0U;
    std::uint64_t insertedMasterVersion = 0U;
};

inline std::vector<std::size_t> selectOldestGraceCandidates(
        std::vector<GraceCandidate> candidates,
        const std::size_t capacity) {
    std::sort(
        candidates.begin(),
        candidates.end(),
        [](const GraceCandidate& left,
           const GraceCandidate& right) {
            if (left.insertedMasterVersion !=
                right.insertedMasterVersion) {
                return left.insertedMasterVersion <
                    right.insertedMasterVersion;
            }
            return left.index < right.index;
        });
    if (candidates.size() > capacity) {
        candidates.resize(capacity);
    }
    std::vector<std::size_t> selected;
    selected.reserve(candidates.size());
    for (const GraceCandidate& candidate : candidates) {
        selected.push_back(candidate.index);
    }
    return selected;
}

inline bool hasCurrentReducedCost(
        const MasterColumnEvaluationState& state,
        const std::uint64_t currentMasterVersion) noexcept {
    return state.evaluatedByLp &&
        state.lastLpVersion == currentMasterVersion;
}

}  // namespace hgasp

#endif  // HGASP_MODELS_COLUMN_RETENTION_POLICY_H_
