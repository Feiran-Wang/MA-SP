#ifndef HGASP_CORE_DIVERSE_SP_SOLUTION_SELECTOR_H_
#define HGASP_CORE_DIVERSE_SP_SOLUTION_SELECTOR_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <vector>

#include "SpTypes.h"

namespace hgasp {

inline double routeSetJaccardDistance(
        const SpSolution& first,
        const SpSolution& second) {
    std::unordered_set<std::size_t> firstRoutes(
        first.selectedRouteIndices.begin(),
        first.selectedRouteIndices.end());
    std::size_t intersection = 0U;
    std::unordered_set<std::size_t> unionRoutes = firstRoutes;
    for (const std::size_t route :
         second.selectedRouteIndices) {
        if (firstRoutes.count(route) != 0U) {
            ++intersection;
        }
        unionRoutes.insert(route);
    }
    if (unionRoutes.empty()) {
        return 0.0;
    }
    return 1.0 -
        static_cast<double>(intersection) /
            static_cast<double>(unionRoutes.size());
}

// Greedy max-min selection. The best objective is always injected first;
// later members maximize their minimum route-set distance from every already
// selected solution. Objective and original order provide deterministic ties.
inline std::vector<std::size_t> selectDiverseSpSolutions(
        const std::vector<SpSolution>& candidates,
        const std::size_t maximumCount) {
    if (maximumCount == 0U || candidates.empty()) {
        return {};
    }
    for (const SpSolution& candidate : candidates) {
        if (candidate.selectedRouteIndices.empty() ||
            !std::isfinite(candidate.objective)) {
            throw std::invalid_argument(
                "diverse SP selection requires finite, nonempty solutions");
        }
    }

    std::size_t best = 0U;
    for (std::size_t index = 1U;
         index < candidates.size();
         ++index) {
        if (candidates[index].objective <
            candidates[best].objective) {
            best = index;
        }
    }

    std::vector<std::size_t> selected{best};
    std::vector<bool> used(candidates.size(), false);
    used[best] = true;
    while (selected.size() <
           std::min(maximumCount, candidates.size())) {
        std::size_t chosen = candidates.size();
        double chosenDistance = -1.0;
        for (std::size_t candidate = 0U;
             candidate < candidates.size();
             ++candidate) {
            if (used[candidate]) {
                continue;
            }
            double minimumDistance =
                std::numeric_limits<double>::infinity();
            for (const std::size_t existing : selected) {
                minimumDistance = std::min(
                    minimumDistance,
                    routeSetJaccardDistance(
                        candidates[candidate],
                        candidates[existing]));
            }
            if (chosen == candidates.size() ||
                minimumDistance > chosenDistance + 1.0e-12 ||
                (std::fabs(
                     minimumDistance - chosenDistance) <=
                     1.0e-12 &&
                 (candidates[candidate].objective <
                      candidates[chosen].objective ||
                  (candidates[candidate].objective ==
                       candidates[chosen].objective &&
                   candidate < chosen)))) {
                chosen = candidate;
                chosenDistance = minimumDistance;
            }
        }
        if (chosen == candidates.size()) {
            break;
        }
        used[chosen] = true;
        selected.push_back(chosen);
    }
    return selected;
}

}  // namespace hgasp

#endif  // HGASP_CORE_DIVERSE_SP_SOLUTION_SELECTOR_H_
