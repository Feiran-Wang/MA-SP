#include "Individual.h"
#include "popManager.h"
#include "read_data.h"

#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

std::mt19937 global_rng(1702U);

namespace {

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void initializeData(read_data& data) {
    data.problemMode = hgasp::ProblemMode::Clrp;
    data.numV = 4;
    data.numL = 1;
    data.numC = 5;
    data.numLocConfiguration = 0;
    data.define();
    data.vehCap = 2;
    data.totalDemand = 4;
    data.usingCostVeh = 0.0;
    for (int customer = 0; customer < data.numV; ++customer) {
        data.demCus[customer] = 1;
    }
    data.invLoc[4] = 4;
    data.openCostLoc[4] = 0.0;
    for (int from = 0; from < data.numC; ++from) {
        for (int to = 0; to < data.numC; ++to) {
            data.D[from][to] = from == to ? 0.0 : 1.0;
            data.T[from][to] = data.D[from][to];
        }
    }
}

StandaloneRoute route(std::vector<int> customers) {
    StandaloneRoute result;
    result.locId = 0;
    result.routeCost = 3.0;
    result.totalLoad = 2;
    result.sequence = std::move(customers);
    return result;
}

void build(
    Individual& individual,
    read_data& data,
    std::vector<StandaloneRoute> routes) {
    individual.define(&data);
    individual.rebuildFromSP(routes);
    require(individual.isFeasible, "constructed individual is infeasible");
}

void testStructuralDuplicateAdmission() {
    read_data data;
    initializeData(data);

    Individual first;
    build(first, data, {route({0, 1}), route({2, 3})});
    Individual sameCostDifferentStructure;
    build(
        sameCostDifferentStructure,
        data,
        {route({0, 2}), route({1, 3})});
    Individual sameStructureDifferentSlots;
    build(
        sameStructureDifferentSlots,
        data,
        {route({1, 3}), route({0, 2})});

    require(
        std::fabs(first.fit - sameCostDifferentStructure.fit) < 1e-9,
        "test solutions do not have equal objective values");
    require(
        !first.hasSameRouteStructure(sameCostDifferentStructure),
        "different route structures were classified as duplicates");
    require(
        sameCostDifferentStructure.hasSameRouteStructure(
            sameStructureDifferentSlots),
        "route-slot order changed the structural signature");

    Individual restartSentinel;
    restartSentinel.fit = -1.0;
    Individual globalSentinel;
    globalSentinel.fit = -1.0;

    popManager manager(&data);
    manager.bestIndivRestart = &restartSentinel;
    manager.global = &globalSentinel;

    manager.addIndividual(&first, 0, false, nullptr);
    require(
        manager.popula[0].size() == 1U,
        "first individual was not admitted");

    manager.addIndividual(
        &sameCostDifferentStructure, 0, false, nullptr);
    require(
        manager.popula[0].size() == 2U,
        "equal-cost structurally distinct individual was rejected");

    manager.addIndividual(
        &sameStructureDifferentSlots, 0, false, nullptr);
    require(
        manager.popula[0].size() == 2U,
        "structural clone unexpectedly entered the population");
}

}  // namespace

int main() {
    try {
        testStructuralDuplicateAdmission();
        std::cout << "population_structural_duplicate_test: PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "population_structural_duplicate_test: "
                  << error.what() << '\n';
        return 1;
    }
}
