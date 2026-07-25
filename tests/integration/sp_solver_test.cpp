#include "Route.h"
#include "SPSolver.h"

#include <cmath>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace {

StandaloneRoute route(
    const int depot,
    const double travelCost,
    const int load,
    std::vector<int> customers) {
    StandaloneRoute value;
    value.locId = depot;
    value.routeCost = travelCost;
    value.totalLoad = load;
    value.sequence = std::move(customers);
    return value;
}

int fail(const std::string& message) {
    std::cerr << "sp_solver_test: " << message << std::endl;
    return 1;
}

}  // namespace

int main() {
    SpConfig config;
    config.timeLimitSeconds = 10.0;
    config.mipGap = 0.0;
    config.threads = 1;
    config.seed = 42;
    config.outputFlag = 0;
    config.mipFocus = 0;
    config.heuristics = 0.05;

    SPSolver solver(config);

    SpProblem problem;
    problem.name = "known_optimum";
    problem.customerDemands = {2, 2, 3};
    problem.vehicleCapacity = 5;
    problem.vehicleFixedCost = 1.0;
    problem.depots = {
        SpDepot{10.0, 7, std::nullopt},
        SpDepot{3.0, 7, std::nullopt},
    };

    const std::vector<StandaloneRoute> incompletePool = {
        route(0, 2.0, 4, {0, 1}),
    };
    const SpResult invalid = solver.solve(problem, incompletePool);
    if (invalid.status != SpStatus::invalid_input ||
        invalid.diagnostic.find("does not cover") == std::string::npos) {
        return fail("incomplete route pool was not rejected before optimization");
    }

    const std::vector<StandaloneRoute> pool = {
        route(0, 2.0, 4, {0, 1}),
        route(0, 2.0, 3, {2}),
        route(1, 5.0, 2, {0}),
        route(1, 1.0, 5, {1, 2}),
        route(1, 10.0, 4, {0, 1}),
        route(1, 10.0, 3, {2}),
    };

    const SpResult result = solver.solve(problem, pool);
    if (result.status != SpStatus::optimal) {
        return fail(
            "expected OPTIMAL, received " + result.statusName +
            " (" + result.diagnostic + ")");
    }
    if (!result.hasFeasibleSolution()) {
        return fail("optimal solve did not return a feasible route selection");
    }
    if (!result.incumbentValidated || result.solutionCount < 1) {
        return fail("Gurobi incumbent was not independently validated");
    }
    if (!result.objective.has_value() ||
        std::fabs(*result.objective - 11.0) > 1e-9) {
        return fail("known optimum should be 11");
    }
    if (result.selectedRouteIndices != std::vector<std::size_t>({2, 3})) {
        return fail("unexpected route combination selected");
    }
    if (result.solverVersion.empty()) {
        return fail("Gurobi version was not recorded");
    }
    if (!result.bestBound.has_value() ||
        std::fabs(*result.bestBound - 11.0) > 1e-9) {
        return fail("known optimum bound should be 11");
    }
    if (!result.mipGap.has_value() || *result.mipGap > 1e-9) {
        return fail("known optimum should report zero MIP gap");
    }
    if (result.solveTimeSeconds < 0.0 ||
        result.totalTimeSeconds < result.solveTimeSeconds) {
        return fail("solver timings are inconsistent");
    }

    SpProblem noExactPartition;
    noExactPartition.name = "no_exact_partition";
    noExactPartition.customerDemands = {1, 1, 1};
    noExactPartition.vehicleCapacity = 3;
    noExactPartition.vehicleFixedCost = 0.0;
    noExactPartition.depots = {
        SpDepot{0.0, 3, std::nullopt},
    };
    const std::vector<StandaloneRoute> overlappingPool = {
        route(0, 1.0, 2, {0, 1}),
        route(0, 1.0, 2, {1, 2}),
    };
    const SpResult infeasible =
        solver.solve(noExactPartition, overlappingPool);
    if (infeasible.status != SpStatus::infeasible ||
        infeasible.solutionCount != 0 ||
        infeasible.hasFeasibleSolution() ||
        infeasible.objective.has_value() ||
        !infeasible.selectedRouteIndices.empty()) {
        return fail("exact-cover infeasibility was not reported safely");
    }

    SpProblem unlimitedFleet;
    unlimitedFleet.name = "unlimited_fleet";
    unlimitedFleet.customerDemands = {1, 1, 1};
    unlimitedFleet.vehicleCapacity = 1;
    unlimitedFleet.vehicleFixedCost = 2.0;
    unlimitedFleet.depots = {
        SpDepot{5.0, 3, std::nullopt},
    };
    const std::vector<StandaloneRoute> singletonPool = {
        route(0, 1.0, 1, {0}),
        route(0, 2.0, 1, {1}),
        route(0, 3.0, 1, {2}),
    };
    const SpResult unlimited =
        solver.solve(unlimitedFleet, singletonPool);
    if (unlimited.status != SpStatus::optimal ||
        !unlimited.hasFeasibleSolution() ||
        unlimited.selectedRouteIndices.size() != 3 ||
        !unlimited.objective.has_value() ||
        std::fabs(*unlimited.objective - 17.0) > 1e-9) {
        return fail(
            "unlimited-fleet model should select all three singleton routes");
    }

    unlimitedFleet.depots[0].representationRouteLimit = 2;
    const SpResult representationLimited =
        solver.solve(unlimitedFleet, singletonPool);
    if (representationLimited.status != SpStatus::infeasible ||
        representationLimited.hasFeasibleSolution() ||
        representationLimited.solutionCount != 0) {
        return fail(
            "representation limit should be separate from fleet size");
    }

    SpConfig exhaustedConfig = config;
    exhaustedConfig.timeLimitSeconds = 1.0e-15;
    SPSolver exhaustedSolver(exhaustedConfig);
    unlimitedFleet.depots[0].representationRouteLimit.reset();
    const SpResult exhausted =
        exhaustedSolver.solve(unlimitedFleet, singletonPool);
    if (exhausted.status != SpStatus::time_limit ||
        exhausted.hasFeasibleSolution() ||
        exhausted.solutionCount != 0) {
        return fail("pre-model SP time-budget exhaustion was not structured");
    }

    std::cout << "sp_solver_test: OPTIMAL objective=" << *result.objective
              << " gurobi=" << result.solverVersion << std::endl;
    return 0;
}
