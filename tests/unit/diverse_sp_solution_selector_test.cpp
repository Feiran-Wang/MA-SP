#include "DiverseSpSolutionSelector.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

SpSolution solution(
        const double objective,
        std::vector<std::size_t> routes) {
    SpSolution value;
    value.objective = objective;
    value.selectedRouteIndices = std::move(routes);
    return value;
}

void testGreedyMaxMinSelection() {
    const std::vector<SpSolution> candidates = {
        solution(100.0, {0, 1, 2}),
        solution(100.5, {0, 1, 3}),
        solution(101.0, {4, 5, 6}),
        solution(101.5, {4, 5, 7}),
    };
    const std::vector<std::size_t> selected =
        hgasp::selectDiverseSpSolutions(candidates, 3U);
    require(
        selected == std::vector<std::size_t>({0, 2, 1}),
        "selector did not choose best-first then max-min diversity");
    require(
        std::fabs(
            hgasp::routeSetJaccardDistance(
                candidates[0], candidates[2]) -
            1.0) < 1.0e-12,
        "disjoint route sets should have distance one");
}

void testInvalidAndZeroCountHandling() {
    require(
        hgasp::selectDiverseSpSolutions(
            {solution(1.0, {0})}, 0U).empty(),
        "zero requested injections should return no solution");
    bool rejected = false;
    try {
        (void)hgasp::selectDiverseSpSolutions(
            {solution(1.0, {})}, 1U);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    require(rejected, "empty SP candidate was not rejected");
}

}  // namespace

int main() {
    try {
        testGreedyMaxMinSelection();
        testInvalidAndZeroCountHandling();
        std::cout
            << "diverse_sp_solution_selector_test: PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr
            << "diverse_sp_solution_selector_test: "
            << error.what() << '\n';
        return 1;
    }
}
