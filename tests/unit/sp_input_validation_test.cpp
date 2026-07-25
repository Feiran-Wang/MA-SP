#include "Route.h"
#include "SPSolver.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace {

int fail(const std::string& message) {
    std::cerr << "sp_input_validation_test: " << message << std::endl;
    return 1;
}

SpProblem problem() {
    SpProblem value;
    value.name = "input_validation";
    value.customerDemands = {2, 3};
    value.vehicleCapacity = 5;
    value.vehicleFixedCost = 1.0;
    value.depots = {SpDepot{4.0, 5, std::nullopt}};
    return value;
}

StandaloneRoute route(
    const int load,
    std::vector<int> customers) {
    StandaloneRoute value;
    value.locId = 0;
    value.routeCost = 2.0;
    value.totalLoad = load;
    value.sequence = std::move(customers);
    return value;
}

int expectInvalid(
    const std::string& label,
    const SpConfig& config,
    const SpProblem& value,
    const std::vector<StandaloneRoute>& routes,
    const std::string& expectedDiagnostic) {
    SPSolver solver(config);
    const SpResult result = solver.solve(value, routes);
    if (result.status != SpStatus::invalid_input) {
        return fail(label + " was not rejected");
    }
    if (!expectedDiagnostic.empty() &&
        result.diagnostic.find(expectedDiagnostic) == std::string::npos) {
        return fail(
            label + " produced unexpected diagnostic: " +
            result.diagnostic);
    }
    if (result.hasFeasibleSolution() ||
        result.incumbentValidated ||
        result.solutionCount != 0) {
        return fail(label + " exposed a feasible incumbent");
    }
    return 0;
}

}  // namespace

int main() {
    const SpConfig defaultConfig;
    const SpProblem defaultProblem = problem();

    if (expectInvalid(
            "missing customer",
            defaultConfig,
            defaultProblem,
            {route(2, {0})},
            "does not cover") != 0) {
        return 1;
    }

    SpProblem repeatedProblem = defaultProblem;
    repeatedProblem.vehicleCapacity = 10;
    repeatedProblem.depots[0].capacity = 10;
    if (expectInvalid(
            "repeated customer",
            defaultConfig,
            repeatedProblem,
            {route(7, {0, 0, 1})},
            "same customer") != 0) {
        return 1;
    }

    if (expectInvalid(
            "inconsistent stored load",
            defaultConfig,
            defaultProblem,
            {route(4, {0, 1})},
            "does not match") != 0) {
        return 1;
    }

    StandaloneRoute unknownCustomer = route(5, {0, 2});
    if (expectInvalid(
            "unknown customer",
            defaultConfig,
            defaultProblem,
            {unknownCustomer},
            "unknown customer") != 0) {
        return 1;
    }

    StandaloneRoute nonFiniteCost = route(5, {0, 1});
    nonFiniteCost.routeCost = std::numeric_limits<double>::quiet_NaN();
    if (expectInvalid(
            "non-finite route cost",
            defaultConfig,
            defaultProblem,
            {nonFiniteCost},
            "finite and non-negative") != 0) {
        return 1;
    }

    SpProblem overflowingLoadProblem = defaultProblem;
    overflowingLoadProblem.customerDemands = {
        std::numeric_limits<int>::max(),
        std::numeric_limits<int>::max(),
    };
    overflowingLoadProblem.vehicleCapacity = std::numeric_limits<int>::max();
    overflowingLoadProblem.depots[0].capacity =
        std::numeric_limits<int>::max();
    if (expectInvalid(
            "overflowing route load",
            defaultConfig,
            overflowingLoadProblem,
            {route(std::numeric_limits<int>::max(), {0, 1})},
            "does not match") != 0) {
        return 1;
    }

    SpProblem overflowingObjectiveProblem = defaultProblem;
    overflowingObjectiveProblem.vehicleFixedCost =
        std::numeric_limits<double>::max();
    StandaloneRoute overflowingObjective = route(5, {0, 1});
    overflowingObjective.routeCost = std::numeric_limits<double>::max();
    if (expectInvalid(
            "overflowing route objective",
            defaultConfig,
            overflowingObjectiveProblem,
            {overflowingObjective},
            "objective coefficients") != 0) {
        return 1;
    }

    SpConfig invalidConfig = defaultConfig;
    invalidConfig.seed = 2000000001;
    if (expectInvalid(
            "out-of-range seed",
            invalidConfig,
            defaultProblem,
            {route(5, {0, 1})},
            "seed") != 0) {
        return 1;
    }

    invalidConfig = defaultConfig;
    invalidConfig.timeLimitSeconds =
        std::numeric_limits<double>::infinity();
    if (expectInvalid(
            "non-finite time limit",
            invalidConfig,
            defaultProblem,
            {route(5, {0, 1})},
            "time limit") != 0) {
        return 1;
    }

    std::cout << "sp_input_validation_test: PASS" << std::endl;
    return 0;
}
