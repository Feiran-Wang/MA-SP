#ifndef HGASP_SOLVER_SP_TYPES_H_
#define HGASP_SOLVER_SP_TYPES_H_

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "Route.h"

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

// The hybrid controller intentionally fails closed. Only these terminal
// outcomes have well-defined continuation semantics; every solver failure,
// ambiguous/unbounded state, numerical issue, interruption, memory limit, or
// unanticipated status aborts the run instead of silently degrading to HGA.
inline bool isSupportedSpTerminalStatus(
    const SpStatus status) noexcept {
    return status == SpStatus::optimal ||
           status == SpStatus::time_limit ||
           status == SpStatus::work_limit ||
           status == SpStatus::infeasible;
}

inline bool mayContinueWithoutSpIncumbent(
    const SpStatus status) noexcept {
    return status == SpStatus::time_limit ||
           status == SpStatus::work_limit ||
           status == SpStatus::infeasible;
}

struct SpConfig {
    double timeLimitSeconds = 360.0;
    // Disabled when zero. A positive value provides a deterministic secondary
    // optimization budget in Gurobi work units.
    double workLimit = 0.0;
    double mipGap = 0.001;
    int threads = 1;
    int seed = 0;
    int outputFlag = 0;
    int mipFocus = 1;
    double heuristics = 0.15;
    std::string logFile;

    // Configuration parameters applied before the lazily-created process
    // environment is started. A zero token duration leaves Gurobi's WLS
    // duration and refresh policy unchanged.
    int wlsTokenDurationMinutes = 0;
    double wlsTokenRefresh = 0.9;
    // Level 1 records only Compute Server/WLS client errors. It does not
    // enable the regular optimization log.
    int clientLogLevel = 1;
    int serverTimeoutSeconds = 30;

    // Maximum total attempts, including the first one, for retryable
    // environment/license/network failures.
    int environmentMaxAttempts = 3;
    int environmentRetryBaseMilliseconds = 250;

    // P2 collaboration controls. The LP-RMP is a continuous restricted
    // master used only for dual guidance; the integer SP remains the source
    // of injected exact-cover incumbents.
    bool lpDualFeedbackEnabled = true;
    double lpTimeLimitSeconds = 10.0;
    bool persistentLpEnabled = true;
    int solutionPoolSize = 5;
    double solutionPoolGap = 0.02;
    int solutionPoolSearchMode = 2;
};

struct SpDepot {
    double openingCost = 0.0;
    int capacity = 0;

    // The CLRP studied by HGAMP has an unlimited fleet. When present, this is
    // the proved redundant numV representation bound, not a fleet constraint.
    std::optional<int> representationRouteLimit;
};

struct SpProblem {
    std::string name;
    std::vector<int> customerDemands;
    int vehicleCapacity = 0;
    double vehicleFixedCost = 0.0;
    std::vector<SpDepot> depots;
    // Empty for the global RMP. A non-empty vector fixes every depot-open
    // variable to the corresponding configuration value for a local RMP.
    std::vector<bool> fixedDepotOpen;
};

// A complete incumbent supplied by the heuristic layer. The solver remaps
// these value routes to the current master-column snapshot and independently
// validates the resulting exact cover before setting any Start attributes.
struct SpMipStart {
    std::vector<StandaloneRoute> routes;
    std::string source = "external";
};

struct SpSolution {
    std::vector<std::size_t> selectedRouteIndices;
    double objective = 0.0;
};

struct SpResult {
    SpStatus status = SpStatus::not_run;
    std::string statusName = "NOT_RUN";
    std::vector<std::size_t> selectedRouteIndices;
    std::optional<double> objective;
    std::optional<double> bestBound;
    std::optional<double> mipGap;
    // Algorithm-performance timings use process CPU time. This is the common
    // timing basis used by the HGA stopping rule and experiment comparisons.
    double solveTimeSeconds = 0.0;
    double validationTimeSeconds = 0.0;
    double environmentTimeSeconds = 0.0;
    double retrySleepTimeSeconds = 0.0;
    double buildTimeSeconds = 0.0;
    double optimizeTimeSeconds = 0.0;
    double postprocessTimeSeconds = 0.0;
    double totalTimeSeconds = 0.0;

    // Wall time remains diagnostic only. Gurobi's native TimeLimit and remote
    // environment/network safety timeouts are wall-clock mechanisms and
    // cannot be expressed as process CPU time.
    double solverWallTimeSeconds = 0.0;
    double retrySleepWallTimeSeconds = 0.0;
    double totalWallTimeSeconds = 0.0;
    std::optional<double> work;
    std::optional<double> nodeCount;
    int modelVariables = 0;
    int modelConstraints = 0;
    int modelNonzeros = 0;
    int environmentStartAttempts = 0;
    int environmentRetryCount = 0;
    bool environmentReused = false;
    // True means that a complete validated Start vector was submitted to the
    // model. Gurobi does not expose a separate authoritative acceptance flag.
    bool mipStartApplied = false;
    int mipStartRouteCount = 0;
    std::string mipStartSource;
    int solutionCount = 0;
    std::vector<SpSolution> candidateSolutions;
    bool incumbentValidated = false;
    int nativeStatus = 0;
    int errorCode = 0;
    std::string solverVersion;
    std::string diagnostic;

    // LP-RMP feedback is indexed against the exact route snapshot supplied
    // to solve(). Customer duals are coverage-constraint shadow prices;
    // route reduced costs already include every active master constraint.
    bool lpFeedbackAvailable = false;
    std::optional<double> lpObjective;
    std::vector<double> customerDuals;
    std::vector<double> depotCapacityDuals;
    std::vector<double> routeReducedCosts;
    double lpBuildTimeSeconds = 0.0;
    double lpOptimizeTimeSeconds = 0.0;

    // Persistent LP synchronization provenance.
    bool persistentLpReused = false;
    int persistentLpAddedColumns = 0;
    int persistentLpReactivatedColumns = 0;
    int persistentLpDisabledColumns = 0;
    int persistentLpStoredColumns = 0;
    int persistentLpActiveColumns = 0;
    bool persistentLpCompacted = false;
    int persistentLpColumnsBeforeCompaction = 0;

    bool hasFeasibleSolution() const noexcept {
        return solutionCount > 0 &&
               incumbentValidated &&
               !selectedRouteIndices.empty();
    }
};

#endif  // HGASP_SOLVER_SP_TYPES_H_
