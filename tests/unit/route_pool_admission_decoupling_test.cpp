#include "Individual.h"
#include "RoutePool.h"
#include "popManager.h"
#include "read_data.h"

#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

std::mt19937 global_rng(1701U);

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
    data.spMaxPoolSize = 100;
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

void testPopulationDuplicateStillCrossesRouteBoundary() {
    read_data data;
    initializeData(data);

    Individual first;
    first.define(&data);
    first.rebuildFromSP({route({0, 1}), route({2, 3})});

    Individual exactDuplicate;
    exactDuplicate.define(&data);
    exactDuplicate.rebuildFromSP({route({0, 1}), route({2, 3})});

    Individual restartSentinel;
    restartSentinel.fit = -1.0;
    Individual globalSentinel;
    globalSentinel.fit = -1.0;

    RoutePool pool(&data);
    popManager manager(&data);
    manager.bestIndivRestart = &restartSentinel;
    manager.global = &globalSentinel;

    manager.addIndividual(&first, 0, false, &pool);
    require(
        manager.popula[0].size() == 1U,
        "first feasible individual was not admitted");
    require(
        pool.getPool().size() == 2U,
        "first feasible individual did not contribute both routes");
    require(
        pool.getFeasibleSubmissionCount() == 1U,
        "first feasible route-source submission was not counted");

    manager.addIndividual(&exactDuplicate, 0, false, &pool);
    require(
        manager.popula[0].size() == 1U,
        "exact duplicate unexpectedly entered the population");
    require(
        pool.getFeasibleSubmissionCount() == 2U,
        "population rejection skipped the route-submission boundary");
    require(
        pool.getPool().size() == 2U,
        "exact route deduplication changed after admission decoupling");
}

}  // namespace

int main() {
    try {
        testPopulationDuplicateStillCrossesRouteBoundary();
        std::cout << "route_pool_admission_decoupling_test: PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "route_pool_admission_decoupling_test: "
                  << error.what() << '\n';
        return 1;
    }
}
