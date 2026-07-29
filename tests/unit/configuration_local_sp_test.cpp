#include "AlgorithmOrchestrator.h"
#include "Route.h"

#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

std::mt19937 global_rng(1802U);

namespace {

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

StandaloneRoute route(
    const int depot,
    std::vector<int> customers) {
    StandaloneRoute result;
    result.locId = depot;
    result.sequence = std::move(customers);
    result.totalLoad = 1;
    result.routeCost = 1.0;
    return result;
}

void testConfigurationFiltering() {
    const std::vector<StandaloneRoute> pool = {
        route(0, {0}),
        route(1, {1}),
        route(2, {2}),
        route(1, {0, 2}),
    };
    const std::vector<StandaloneRoute> filtered =
        AlgorithmOrchestrator::filterRoutesForConfiguration(
            pool,
            {false, true, false});
    require(
        filtered.size() == 2U,
        "local pool retained the wrong number of columns");
    require(
        filtered[0].locId == 1 &&
            filtered[1].locId == 1,
        "local pool contains a route from a closed depot");
    require(
        filtered[0].sequence == std::vector<int>({1}) &&
            filtered[1].sequence ==
                std::vector<int>({0, 2}),
        "local pool filtering changed route order");
}

void testOnlyGlobalDualFeedbackIsImported() {
    SpProblem global;
    require(
        AlgorithmOrchestrator::shouldImportDualFeedback(global),
        "global LP feedback was incorrectly rejected");
    SpProblem local;
    local.fixedDepotOpen = {true, false};
    require(
        !AlgorithmOrchestrator::shouldImportDualFeedback(local),
        "configuration-local duals may overwrite global guidance");
}

void testActivePopulationTargetsLocalSp() {
    const std::vector<bool> nonEmpty{
        true, false, true};
    require(
        AlgorithmOrchestrator::chooseLocalPopulation(
            2, nonEmpty, 0) == 2,
        "local SP ignored the population that produced the event");
    require(
        AlgorithmOrchestrator::chooseLocalPopulation(
            1, nonEmpty, 0) == 0,
        "local SP did not fall back from an empty active population");
}

}  // namespace

int main() {
    try {
        testConfigurationFiltering();
        testOnlyGlobalDualFeedbackIsImported();
        testActivePopulationTargetsLocalSp();
        std::cout << "configuration_local_sp_test: PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "configuration_local_sp_test: "
                  << error.what() << '\n';
        return 1;
    }
}
