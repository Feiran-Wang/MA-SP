#include "Individual.h"
#include "RoutePool.h"
#include "read_data.h"

#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

std::mt19937 global_rng(1703U);

namespace {

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void initializeData(read_data& data) {
    data.problemMode = hgasp::ProblemMode::Clrp;
    data.numV = 2;
    data.numL = 1;
    data.numC = 3;
    data.spMaxPoolSize = 100;
    data.define();
    data.vehCap = 5;
    data.totalDemand = 5;
    data.usingCostVeh = 0.0;
    data.demCus[0] = 2;
    data.demCus[1] = 3;
    data.invLoc[2] = 5;
    data.openCostLoc[2] = 0.0;

    for (int from = 0; from < data.numC; ++from) {
        for (int to = 0; to < data.numC; ++to) {
            data.D[from][to] = 0.0;
            data.T[from][to] = 0.0;
        }
    }

    // depot(2) -> 0 -> 1 -> depot = 48
    data.D[2][0] = 10.0;
    data.D[0][1] = 20.0;
    data.D[1][2] = 18.0;
    // depot(2) -> 1 -> 0 -> depot = 102
    data.D[2][1] = 30.0;
    data.D[1][0] = 40.0;
    data.D[0][2] = 32.0;
}

StandaloneRoute route(
    std::vector<int> sequence,
    const double cost) {
    StandaloneRoute result;
    result.locId = 0;
    result.routeCost = cost;
    result.totalLoad = 5;
    result.sequence = std::move(sequence);
    return result;
}

void testExactMasterColumnDominance() {
    read_data data;
    initializeData(data);

    Individual expensive;
    expensive.define(&data);
    expensive.rebuildFromSP({route({1, 0}, 102.0)});

    Individual cheap;
    cheap.define(&data);
    cheap.rebuildFromSP({route({0, 1}, 48.0)});

    RoutePool pool(&data);
    pool.extractRoutes(&expensive);
    pool.extractRoutes(&cheap);

    require(
        pool.getSearchArchive().size() == 2U,
        "ordered search archive did not retain both route variants");
    require(
        pool.getPool().size() == 1U,
        "same-depot same-customer-set columns were not compressed");
    require(
        pool.getPool().front().sequence ==
            std::vector<int>({0, 1}),
        "master pool did not retain the cheapest route order");
    require(
        std::fabs(pool.getPool().front().routeCost - 48.0) < 1e-9,
        "master pool retained the wrong route cost");
    require(
        pool.getDominanceReplacementCount() == 1U,
        "cheaper master-column replacement was not counted");
}

void testRouteHistoryHasHardBounds() {
    read_data data;
    initializeData(data);
    data.spMaxPoolSize = 1;

    Individual expensive;
    expensive.define(&data);
    expensive.rebuildFromSP({route({1, 0}, 102.0)});
    Individual cheap;
    cheap.define(&data);
    cheap.rebuildFromSP({route({0, 1}, 48.0)});

    RoutePool pool(&data);
    pool.extractRoutes(&expensive);
    pool.extractRoutes(&cheap);
    require(
        pool.getSearchArchive().size() <= 1U &&
            pool.getBestArchivedColumnCount() <= 1U,
        "route provenance history exceeded its hard bound");
    require(
        pool.getSearchArchiveDiscardCount() == 1U,
        "bounded search archive did not record discarded history");
    require(
        pool.getPool().front().sequence ==
            std::vector<int>({0, 1}),
        "history bounding blocked a cheaper master-column update");
}

}  // namespace

int main() {
    try {
        testExactMasterColumnDominance();
        testRouteHistoryHasHardBounds();
        std::cout << "route_pool_column_dominance_test: PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "route_pool_column_dominance_test: "
                  << error.what() << '\n';
        return 1;
    }
}
