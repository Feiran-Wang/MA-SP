#include "Individual.h"
#include "RoutePool.h"
#include "read_data.h"

#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

std::mt19937 global_rng;

namespace {

int fail(const std::string& message) {
    std::cerr << "route_cost_contract_test: " << message << std::endl;
    return 1;
}

bool close(const double lhs, const double rhs) {
    return std::fabs(lhs - rhs) <= 1.0e-9;
}

StandaloneRoute makeRoute(
    std::vector<int> sequence,
    const double cost) {
    StandaloneRoute route;
    route.locId = 0;
    route.routeCost = cost;
    route.totalLoad = 5;
    route.sequence = std::move(sequence);
    return route;
}

}  // namespace

int main() {
    read_data data;
    data.numV = 2;
    data.numL = 1;
    data.numC = 3;
    data.define();
    data.demCus[0] = 2;
    data.demCus[1] = 3;
    data.totalDemand = 5;
    data.vehCap = 5;
    data.invLoc[2] = 10;
    data.openCostLoc[2] = 7.0;
    data.usingCostVeh = 4.0;
    data.spMaxPoolSize = 100;

    for (int from = 0; from < data.numC; ++from) {
        for (int to = 0; to < data.numC; ++to) {
            data.D[from][to] = 0.0;
        }
    }

    // Directed forward route: depot(2) -> 0 -> 1 -> depot(2) = 48.
    data.D[2][0] = 10.0;
    data.D[0][1] = 20.0;
    data.D[1][2] = 18.0;

    // Its reversal has a deliberately different cost: 102.
    data.D[2][1] = 30.0;
    data.D[1][0] = 40.0;
    data.D[0][2] = 32.0;

    Individual individual;
    individual.define(&data);
    individual.rebuildFromSP({makeRoute({0, 1}, 48.0)});
    if (!individual.isFeasible ||
        !close(individual.dis, 48.0) ||
        !close(individual.fit, 59.0) ||
        individual.route[0].nbClients != 2) {
        return fail("forward route aggregates do not use D[from][to]");
    }

    RoutePool pool(&data);
    pool.extractRoutes(&individual);
    if (pool.getPool().size() != 1) {
        return fail("route exporter did not produce exactly one route");
    }
    const StandaloneRoute& exported = pool.getPool().front();
    if (exported.sequence != std::vector<int>({0, 1}) ||
        exported.totalLoad != 5 ||
        !close(exported.routeCost, 48.0)) {
        return fail("route exporter changed the directed route contract");
    }

    Individual reverse;
    reverse.define(&data);
    reverse.rebuildFromSP({makeRoute({1, 0}, 102.0)});
    if (!close(reverse.dis, 102.0)) {
        return fail("reverse route did not retain its distinct directed cost");
    }

    Individual invalid;
    invalid.define(&data);
    try {
        invalid.rebuildFromSP({makeRoute({0, 1}, 47.0)});
        return fail("importer accepted a route with an inconsistent cost");
    } catch (const std::invalid_argument&) {
        // Expected: the value-level route is rejected before links are changed.
    }

    std::cout << "route_cost_contract_test: PASS" << std::endl;
    return 0;
}
