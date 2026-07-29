#include "Individual.h"
#include "RoutePool.h"
#include "read_data.h"

#include <algorithm>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

std::mt19937 global_rng(1801U);

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
    data.spMaxPoolSize = 4;
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
    const std::vector<std::vector<int>>& routes) {
    individual.define(&data);
    std::vector<StandaloneRoute> values;
    for (const std::vector<int>& customers : routes) {
        values.push_back(route(customers));
    }
    individual.rebuildFromSP(values);
    require(individual.isFeasible, "constructed individual is infeasible");
}

std::vector<int> canonical(const StandaloneRoute& routeValue) {
    std::vector<int> customers = routeValue.sequence;
    std::sort(customers.begin(), customers.end());
    return customers;
}

bool contains(
    const std::vector<StandaloneRoute>& pool,
    const std::vector<int>& customers) {
    return std::any_of(
        pool.begin(),
        pool.end(),
        [&](const StandaloneRoute& routeValue) {
            return canonical(routeValue) == customers;
        });
}

void testProtectedBackboneAndTieredPruning() {
    read_data data;
    initializeData(data);

    Individual backbone;
    build(backbone, data, {{0, 1}, {2, 3}});
    Individual alternativeOne;
    build(alternativeOne, data, {{0, 2}, {1, 3}});
    Individual alternativeTwo;
    build(alternativeTwo, data, {{0, 3}, {1, 2}});

    RoutePool pool(&data);
    pool.extractRoutes(&backbone);
    pool.setProtectedRoutes(
        RouteProtection::Backbone,
        pool.exportRoutes(&backbone));
    pool.extractRoutes(&alternativeOne);
    pool.extractRoutes(&alternativeTwo);

    require(
        pool.getPool().size() == 3U,
        "tiered pruning did not leave the configured 80-percent target");
    require(
        contains(pool.getPool(), {0, 1}) &&
            contains(pool.getPool(), {2, 3}),
        "protected feasible backbone was removed");
    require(
        pool.getLastPruneProtectedCount() == 2U,
        "protected tier retained the wrong number of columns");
    require(
        pool.getLastPruneRarityCount() == 1U,
        "rarity tier did not fill the remaining target slot");

    std::vector<int> coverage(data.numV, 0);
    for (const StandaloneRoute& routeValue : pool.getPool()) {
        for (const int customer : routeValue.sequence) {
            ++coverage[customer];
        }
    }
    require(
        std::all_of(
            coverage.begin(),
            coverage.end(),
            [](const int count) { return count > 0; }),
        "pruning destroyed complete customer coverage");
}

}  // namespace

int main() {
    try {
        testProtectedBackboneAndTieredPruning();
        std::cout << "route_pool_tiered_pruning_test: PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "route_pool_tiered_pruning_test: "
                  << error.what() << '\n';
        return 1;
    }
}
