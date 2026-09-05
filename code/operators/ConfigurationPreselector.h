#ifndef HGASP_OPERATORS_CONFIGURATIONPRESELECTOR_H_
#define HGASP_OPERATORS_CONFIGURATIONPRESELECTOR_H_

#include <algorithm>
#include <cstddef>
#include <limits>
#include <vector>

namespace hgasp {

inline constexpr std::size_t kConfigurationCheapLimit = 30U;
inline constexpr std::size_t kConfigurationFinalLimit = 15U;
inline constexpr std::size_t kConfigurationConstructionSamples = 3U;

struct ConfigurationPreselectionCandidate {
    std::size_t sourceIndex = 0U;
    double totalCapacity = 0.0;
    double cheapScore = std::numeric_limits<double>::infinity();
    std::vector<double> successfulObjectives;
};

inline double successfulObjectiveMedian(
        std::vector<double> objectives) {
    if (objectives.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    std::sort(objectives.begin(), objectives.end());
    const std::size_t middle = objectives.size() / 2U;
    if ((objectives.size() & 1U) != 0U) {
        return objectives[middle];
    }
    return 0.5 * (objectives[middle - 1U] + objectives[middle]);
}

inline std::vector<ConfigurationPreselectionCandidate>
capacityAndCheapPrefilter(
        std::vector<ConfigurationPreselectionCandidate> candidates,
        const double totalDemand) {
    candidates.erase(
        std::remove_if(
            candidates.begin(),
            candidates.end(),
            [totalDemand](const ConfigurationPreselectionCandidate& candidate) {
                return candidate.totalCapacity < totalDemand;
            }),
        candidates.end());
    std::stable_sort(
        candidates.begin(),
        candidates.end(),
        [](const ConfigurationPreselectionCandidate& lhs,
           const ConfigurationPreselectionCandidate& rhs) {
            if (lhs.cheapScore != rhs.cheapScore) {
                return lhs.cheapScore < rhs.cheapScore;
            }
            return lhs.sourceIndex < rhs.sourceIndex;
        });
    if (candidates.size() > kConfigurationCheapLimit) {
        candidates.resize(kConfigurationCheapLimit);
    }
    return candidates;
}

inline std::vector<ConfigurationPreselectionCandidate>
rankConstructedConfigurations(
        std::vector<ConfigurationPreselectionCandidate> candidates,
        const std::size_t finalLimit = kConfigurationFinalLimit) {
    std::stable_sort(
        candidates.begin(),
        candidates.end(),
        [](const ConfigurationPreselectionCandidate& lhs,
           const ConfigurationPreselectionCandidate& rhs) {
            const bool lhsSucceeded = !lhs.successfulObjectives.empty();
            const bool rhsSucceeded = !rhs.successfulObjectives.empty();
            if (lhsSucceeded != rhsSucceeded) {
                return lhsSucceeded;
            }
            const double lhsMedian =
                successfulObjectiveMedian(lhs.successfulObjectives);
            const double rhsMedian =
                successfulObjectiveMedian(rhs.successfulObjectives);
            if (lhsMedian != rhsMedian) {
                return lhsMedian < rhsMedian;
            }
            if (lhs.cheapScore != rhs.cheapScore) {
                return lhs.cheapScore < rhs.cheapScore;
            }
            return lhs.sourceIndex < rhs.sourceIndex;
        });
    if (candidates.size() > finalLimit) {
        candidates.resize(finalLimit);
    }
    return candidates;
}

}  // namespace hgasp

#endif  // HGASP_OPERATORS_CONFIGURATIONPRESELECTOR_H_
