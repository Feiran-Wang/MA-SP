#include "Individual.h"
#include "read_data.h"

#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

std::mt19937 global_rng(1704U);

namespace {

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void initializeData(read_data& data) {
    data.problemMode = hgasp::ProblemMode::Clrp;
    data.numV = 6;
    data.numL = 1;
    data.numC = 7;
    data.define();
    data.vehCap = 100;
    data.totalDemand = 6;
    data.usingCostVeh = 0.0;
    for (int customer = 0; customer < data.numV; ++customer) {
        data.demCus[customer] = 1;
    }
    data.invLoc[6] = 6;
    data.openCostLoc[6] = 0.0;
    for (int from = 0; from < data.numC; ++from) {
        for (int to = 0; to < data.numC; ++to) {
            data.D[from][to] = from == to ? 0.0 : 1.0;
            data.T[from][to] = data.D[from][to];
        }
    }
}

StandaloneRoute singleton(const int customer) {
    StandaloneRoute route;
    route.locId = 0;
    route.routeCost = 2.0;
    route.totalLoad = 1;
    route.sequence = {customer};
    return route;
}

void testSafeUnlimitedFleetRepresentationBound() {
    read_data data;
    require(
        !data.spEnforceRepresentationRouteLimit,
        "redundant SP representation constraint is enabled by default");
    initializeData(data);

    require(
        Individual::representationRouteLimit(data, 0) == data.numV,
        "fixed-slot representation is not using the proved customer bound");

    std::vector<StandaloneRoute> routes;
    for (int customer = 0; customer < data.numV; ++customer) {
        routes.push_back(singleton(customer));
    }

    Individual individual;
    individual.define(&data);
    individual.rebuildFromSP(routes);
    require(
        individual.isFeasible,
        "safe-bound representation could not import n singleton routes");
    require(
        individual.location[0].maxVisit == data.numV,
        "allocated route slots do not match the safe bound");
    require(
        individual.location[0].splitTime == data.numV,
        "SP reconstruction did not retain all singleton routes");
}

void testCapacityAndDemandTighteningIsSafe() {
    read_data data;
    initializeData(data);
    data.numV = 6;
    data.invLoc[6] = 5;
    for (int customer = 0; customer < 6; ++customer) {
        data.demCus[customer] = 2;
    }
    require(
        Individual::representationRouteLimit(data, 0) == 2,
        "depot-capacity/minimum-demand bound was not applied");

    data.demCus[0] = 0;
    data.demCus[1] = 0;
    require(
        Individual::representationRouteLimit(data, 0) == 4,
        "zero-demand customers were not defended in the bound");

    for (int customer = 0; customer < 6; ++customer) {
        data.demCus[customer] = 0;
    }
    require(
        Individual::representationRouteLimit(data, 0) == 6,
        "all-zero demand fallback is not safe");
}

}  // namespace

int main() {
    try {
        testSafeUnlimitedFleetRepresentationBound();
        testCapacityAndDemandTighteningIsSafe();
        std::cout << "representation_route_bound_test: PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "representation_route_bound_test: "
                  << error.what() << '\n';
        return 1;
    }
}
