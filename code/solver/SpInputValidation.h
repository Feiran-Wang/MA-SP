#ifndef HGASP_SOLVER_SP_INPUT_VALIDATION_H_
#define HGASP_SOLVER_SP_INPUT_VALIDATION_H_

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "Route.h"
#include "SpTypes.h"

namespace hgasp::sp_internal {

inline std::optional<int> boundedServerStartupTimeout(
        const int configuredSeconds,
        const double remainingOuterWallSeconds) noexcept {
    if (configuredSeconds <= 0 ||
        !std::isfinite(remainingOuterWallSeconds) ||
        remainingOuterWallSeconds < 1.0) {
        return std::nullopt;
    }
    const double boundedRemaining = std::min(
        remainingOuterWallSeconds,
        static_cast<double>(std::numeric_limits<int>::max()));
    return std::min(
        configuredSeconds,
        static_cast<int>(std::floor(boundedRemaining)));
}

// Side-effect-free SP boundary validation. An empty diagnostic means the
// value problem is admissible. This helper never constructs a solver model or
// acquires a Gurobi environment.
std::string validateInput(
    const SpConfig& config,
    const SpProblem& problem,
    const std::vector<StandaloneRoute>& routes);

}  // namespace hgasp::sp_internal

#endif  // HGASP_SOLVER_SP_INPUT_VALIDATION_H_
