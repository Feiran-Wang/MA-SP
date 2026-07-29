#include "AdaptiveSearchAllocator.h"
#include "PopulationIncubationGuard.h"

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

void testPopulationUcbAllocation() {
    hgasp::AdaptiveSearchPolicy policy;
    policy.exploration = 0.2;
    policy.diversityWeight = 0.1;
    policy.columnWeight = 0.05;
    policy.emaWeight = 0.5;
    hgasp::AdaptiveSearchAllocator allocator(policy);

    const std::vector<int> eligible{0, 1};
    const std::vector<double> diversity{0.1, 0.8};
    require(
        allocator.selectPopulation(eligible, diversity) == 0,
        "first untried population was not selected");
    allocator.reportPopulation(
        0, hgasp::SearchArmObservation{0.0, 1.0, 0U});
    require(
        allocator.selectPopulation(eligible, diversity) == 1,
        "second untried population was not explored");
    allocator.reportPopulation(
        1, hgasp::SearchArmObservation{10.0, 1.0, 2U});
    require(
        allocator.selectPopulation(eligible, diversity) == 1,
        "productive population did not receive exploitation budget");
    require(
        allocator.populationImprovementEma(1) > 9.9,
        "population improvement-per-CPU EMA is incorrect");
}

void testMutationAllocation() {
    hgasp::AdaptiveSearchPolicy policy;
    policy.mutationMinimumPercent = 5;
    policy.mutationMaximumPercent = 40;
    hgasp::AdaptiveSearchAllocator allocator(policy);

    const int stagnant =
        allocator.mutationProbability(
            15, 0.10, 1.0, false);
    const int afterSp =
        allocator.mutationProbability(
            15, 0.10, 1.0, true);
    require(
        stagnant > 15,
        "stagnation and low diversity did not increase mutation");
    require(
        afterSp < stagnant,
        "recent SP improvement did not shift budget to exploitation");

    allocator.reportMutation(true, 1.0, 1.0);
    const int productive =
        allocator.mutationProbability(
            15, 0.10, 1.0, false);
    require(
        productive >= stagnant &&
            productive <= policy.mutationMaximumPercent,
        "productive mutation feedback was not reflected safely");
}

void testSharedCreditAndConfigurationReset() {
    hgasp::AdaptiveSearchAllocator allocator;
    const hgasp::SearchArmObservation joint{
        10.0, 2.0, 6.0};
    allocator.reportSharedPopulations(0, 1, joint);
    require(
        allocator.populationReports(0) == 1U &&
            allocator.populationReports(1) == 1U,
        "dual-parent EAX outcome was not credited to both parents");
    require(
        std::fabs(
            allocator.populationImprovementEma(0) - 2.5) <
                1.0e-12 &&
            std::fabs(
                allocator.populationImprovementEma(1) - 2.5) <
                1.0e-12,
        "dual-parent credit was not split evenly");
    allocator.resetPopulation(0);
    require(
        allocator.populationReports(0) == 0U &&
            allocator.populationReports(1) == 1U,
        "replaced configuration retained stale arm history");
}

void testPopulationIncubationGuard() {
    hgasp::PopulationIncubationGuard guard(5U, 1U, 20U, 3U);
    for (int attempt = 0;
         guard.beginAttempt(1U, false);
         ++attempt) {
        guard.observe(1U);
    }
    require(
        guard.attempts() == 3U &&
            guard.noProgressAttempts() == 3U,
        "incubation guard did not stop a no-progress loop");

    hgasp::PopulationIncubationGuard stopped(
        5U, 1U, 20U, 3U);
    require(
        !stopped.beginAttempt(1U, true) &&
            stopped.stoppedByExternalRequest(),
        "incubation guard ignored an external stop request");
}

}  // namespace

int main() {
    try {
        testPopulationUcbAllocation();
        testMutationAllocation();
        testSharedCreditAndConfigurationReset();
        testPopulationIncubationGuard();
        std::cout
            << "adaptive_search_allocator_test: PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr
            << "adaptive_search_allocator_test: "
            << error.what() << '\n';
        return 1;
    }
}
