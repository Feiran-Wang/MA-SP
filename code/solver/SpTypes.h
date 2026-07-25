#ifndef HGASP_SOLVER_SP_TYPES_H_
#define HGASP_SOLVER_SP_TYPES_H_

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

enum class SpStatus {
    not_run,
    optimal,
    time_limit,
    infeasible,
    inf_or_unbounded,
    unbounded,
    cutoff,
    iteration_limit,
    node_limit,
    solution_limit,
    interrupted,
    suboptimal,
    numeric,
    user_objective_limit,
    work_limit,
    memory_limit,
    invalid_input,
    error
};

struct SpConfig {
    double timeLimitSeconds = 360.0;
    double mipGap = 0.001;
    int threads = 1;
    int seed = 0;
    int outputFlag = 0;
    int mipFocus = 1;
    double heuristics = 0.15;
    std::string logFile;
};

struct SpDepot {
    double openingCost = 0.0;
    int capacity = 0;

    // The CLRP studied by HGAMP has an unlimited fleet. This optional bound is
    // not part of the mathematical problem; it only protects the fixed-slot
    // Individual representation during SP solution import.
    std::optional<int> representationRouteLimit;
};

struct SpProblem {
    std::string name;
    std::vector<int> customerDemands;
    int vehicleCapacity = 0;
    double vehicleFixedCost = 0.0;
    std::vector<SpDepot> depots;
};

struct SpResult {
    SpStatus status = SpStatus::not_run;
    std::string statusName = "NOT_RUN";
    std::vector<std::size_t> selectedRouteIndices;
    std::optional<double> objective;
    std::optional<double> bestBound;
    std::optional<double> mipGap;
    double solveTimeSeconds = 0.0;
    double totalTimeSeconds = 0.0;
    int solutionCount = 0;
    bool incumbentValidated = false;
    int nativeStatus = 0;
    int errorCode = 0;
    std::string solverVersion;
    std::string diagnostic;

    bool hasFeasibleSolution() const noexcept {
        return solutionCount > 0 &&
               incumbentValidated &&
               !selectedRouteIndices.empty();
    }
};

#endif  // HGASP_SOLVER_SP_TYPES_H_
