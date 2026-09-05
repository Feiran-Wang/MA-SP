#ifndef HGASP_CORE_NORMALIZED_SP_UTILITY_H_
#define HGASP_CORE_NORMALIZED_SP_UTILITY_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace hgasp {

constexpr double kSpUtilityFloor = 1.0e-9;

inline double normalizedObjectiveUtility(
        const double before,
        const double after,
        const double cpuSeconds) noexcept {
    if (!std::isfinite(before) || !std::isfinite(after) ||
        !std::isfinite(cpuSeconds)) {
        return 0.0;
    }
    return std::max(0.0, before - after) /
        std::max(std::fabs(before), kSpUtilityFloor) /
        std::max(cpuSeconds, kSpUtilityFloor);
}

inline double reducedCostRobustScale(
        const std::vector<double>& reducedCosts) {
    std::vector<double> magnitudes;
    magnitudes.reserve(reducedCosts.size());
    for (const double value : reducedCosts) {
        if (std::isfinite(value)) {
            magnitudes.push_back(std::fabs(value));
        }
    }
    if (magnitudes.empty()) {
        return kSpUtilityFloor;
    }
    const std::size_t middle = magnitudes.size() / 2U;
    std::nth_element(
        magnitudes.begin(),
        magnitudes.begin() + static_cast<std::ptrdiff_t>(middle),
        magnitudes.end());
    double median = magnitudes[middle];
    if (magnitudes.size() % 2U == 0U) {
        const double lower = *std::max_element(
            magnitudes.begin(),
            magnitudes.begin() + static_cast<std::ptrdiff_t>(middle));
        median = 0.5 * (lower + median);
    }
    return std::max(median, kSpUtilityFloor);
}

inline double normalizedReducedCostQuality(
        const double reducedCost,
        const double scale) noexcept {
    if (!std::isfinite(reducedCost) || !std::isfinite(scale) ||
        scale <= 0.0) {
        return 0.0;
    }
    return std::exp(
        -std::max(0.0, reducedCost) /
        std::max(scale, kSpUtilityFloor));
}

}  // namespace hgasp

#endif  // HGASP_CORE_NORMALIZED_SP_UTILITY_H_
