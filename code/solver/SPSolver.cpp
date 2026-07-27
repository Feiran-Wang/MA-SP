#include "SPSolver.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>

#include "gurobi_c++.h"

namespace {

using Clock = std::chrono::steady_clock;

double elapsedSeconds(const Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}

SpResult makeResult(
    const SpStatus status,
    std::string statusName,
    std::string diagnostic) {
    SpResult result;
    result.status = status;
    result.statusName = std::move(statusName);
    result.diagnostic = std::move(diagnostic);
    return result;
}

std::string validateInput(
    const SpConfig& config,
    const SpProblem& problem,
    const std::vector<StandaloneRoute>& routes) {
    if (!std::isfinite(config.timeLimitSeconds) || config.timeLimitSeconds <= 0.0) {
        return "SP time limit must be finite and positive";
    }
    if (!std::isfinite(config.workLimit) || config.workLimit < 0.0) {
        return "SP work limit must be finite and non-negative";
    }
    if (!std::isfinite(config.mipGap) || config.mipGap < 0.0) {
        return "SP MIP gap must be finite and non-negative";
    }
    if (config.threads < 1 || config.threads > 1024) {
        return "SP thread count must be between 1 and 1024";
    }
    if (config.seed < 0 || config.seed > 2000000000) {
        return "SP seed must be between 0 and 2000000000";
    }
    if (config.outputFlag != 0 && config.outputFlag != 1) {
        return "SP output flag must be either 0 or 1";
    }
    if (config.mipFocus < 0 || config.mipFocus > 3) {
        return "SP MIP focus must be between 0 and 3";
    }
    if (!std::isfinite(config.heuristics) ||
        config.heuristics < 0.0 ||
        config.heuristics > 1.0) {
        return "SP heuristics fraction must be between 0 and 1";
    }
    if (config.wlsTokenDurationMinutes < 0 ||
        config.wlsTokenDurationMinutes > 2000000000) {
        return "WLS token duration must be between 0 and 2000000000 minutes";
    }
    if (!std::isfinite(config.wlsTokenRefresh) ||
        config.wlsTokenRefresh < 0.0 ||
        config.wlsTokenRefresh > 1.0) {
        return "WLS token refresh fraction must be between 0 and 1";
    }
    if (config.clientLogLevel < 0 || config.clientLogLevel > 3) {
        return "Gurobi client log level must be between 0 and 3";
    }
    if (config.serverTimeoutSeconds < 1 ||
        config.serverTimeoutSeconds > 2000000000) {
        return "Gurobi server timeout must be between 1 and 2000000000 seconds";
    }
    if (config.environmentMaxAttempts < 1 ||
        config.environmentMaxAttempts > 10) {
        return "Gurobi environment attempts must be between 1 and 10";
    }
    if (config.environmentRetryBaseMilliseconds < 0 ||
        config.environmentRetryBaseMilliseconds > 60000) {
        return "Gurobi environment retry delay must be between 0 and 60000 ms";
    }
    if (problem.customerDemands.empty()) {
        return "SP requires at least one customer";
    }
    if (problem.depots.empty()) {
        return "SP requires at least one depot";
    }
    if (problem.vehicleCapacity <= 0) {
        return "vehicle capacity must be positive";
    }
    if (!std::isfinite(problem.vehicleFixedCost) ||
        problem.vehicleFixedCost < 0.0) {
        return "vehicle fixed cost must be finite and non-negative";
    }
    for (std::size_t customer = 0; customer < problem.customerDemands.size(); ++customer) {
        if (problem.customerDemands[customer] <= 0) {
            return "customer demands must be positive";
        }
    }
    for (std::size_t depot = 0; depot < problem.depots.size(); ++depot) {
        const SpDepot& data = problem.depots[depot];
        if (data.capacity <= 0) {
            return "depot capacities must be positive";
        }
        if (!std::isfinite(data.openingCost) || data.openingCost < 0.0) {
            return "depot opening costs must be finite and non-negative";
        }
        if (data.representationRouteLimit.has_value() &&
            *data.representationRouteLimit < 1) {
            return "depot representation route limits must be positive";
        }
    }
    if (routes.empty()) {
        return "route pool is empty";
    }

    std::vector<bool> covered(problem.customerDemands.size(), false);
    const std::size_t unmarked = std::numeric_limits<std::size_t>::max();
    std::vector<std::size_t> routeMarks(
        problem.customerDemands.size(),
        unmarked);
    for (std::size_t routeIndex = 0; routeIndex < routes.size(); ++routeIndex) {
        const StandaloneRoute& route = routes[routeIndex];
        if (route.locId < 0 ||
            static_cast<std::size_t>(route.locId) >= problem.depots.size()) {
            return "route references an unknown depot";
        }
        if (route.sequence.empty()) {
            return "route pool contains an empty route";
        }
        if (!std::isfinite(route.routeCost) || route.routeCost < 0.0) {
            return "route travel costs must be finite and non-negative";
        }
        if (!std::isfinite(route.routeCost + problem.vehicleFixedCost)) {
            return "route objective coefficients must be finite";
        }
        if (route.totalLoad <= 0 ||
            route.totalLoad > problem.vehicleCapacity ||
            route.totalLoad > problem.depots[route.locId].capacity) {
            return "route load violates a vehicle or depot capacity";
        }

        std::int64_t computedLoad = 0;
        for (const int customer : route.sequence) {
            if (customer < 0 ||
                static_cast<std::size_t>(customer) >= problem.customerDemands.size()) {
                return "route references an unknown customer";
            }
            if (routeMarks[customer] == routeIndex) {
                return "a route visits the same customer more than once";
            }
            routeMarks[customer] = routeIndex;
            computedLoad +=
                static_cast<std::int64_t>(problem.customerDemands[customer]);
            covered[customer] = true;
        }
        if (computedLoad != static_cast<std::int64_t>(route.totalLoad)) {
            return "stored route load does not match customer demands";
        }
    }

    std::vector<int> missing;
    for (std::size_t customer = 0; customer < covered.size(); ++customer) {
        if (!covered[customer]) {
            missing.push_back(static_cast<int>(customer));
        }
    }
    if (!missing.empty()) {
        std::ostringstream message;
        message << "route pool does not cover customers";
        const std::size_t displayCount = std::min<std::size_t>(missing.size(), 10);
        for (std::size_t index = 0; index < displayCount; ++index) {
            message << (index == 0 ? " " : ", ") << missing[index];
        }
        if (missing.size() > displayCount) {
            message << " (and " << missing.size() - displayCount << " more)";
        }
        return message.str();
    }
    return {};
}

struct EnvironmentSettings {
    int threadLimit = 1;
    int tokenDurationMinutes = 0;
    double tokenRefresh = 0.9;
    int clientLogLevel = 1;
    int serverTimeoutSeconds = 30;
    std::string logFile;

    bool operator==(const EnvironmentSettings& other) const noexcept {
        return threadLimit == other.threadLimit &&
               tokenDurationMinutes == other.tokenDurationMinutes &&
               tokenRefresh == other.tokenRefresh &&
               clientLogLevel == other.clientLogLevel &&
               serverTimeoutSeconds == other.serverTimeoutSeconds &&
               logFile == other.logFile;
    }
};

EnvironmentSettings environmentSettings(const SpConfig& config) {
    EnvironmentSettings settings;
    settings.threadLimit = config.threads;
    settings.tokenDurationMinutes = config.wlsTokenDurationMinutes;
    settings.tokenRefresh = config.wlsTokenRefresh;
    settings.clientLogLevel = config.clientLogLevel;
    settings.serverTimeoutSeconds = std::max(
        1,
        std::min(
            config.serverTimeoutSeconds,
            static_cast<int>(std::min(
                std::ceil(config.timeLimitSeconds),
                2000000000.0))));
    if (config.clientLogLevel > 0) {
        settings.logFile = config.logFile;
    }
    return settings;
}

// A Gurobi environment is a license/session boundary and is not thread-safe.
// Experiment workers are separate processes, so each process owns one explicit
// environment manager. Models in that process serialize access and reuse the
// lazily started environment across all periodic SP calls.
class ProcessEnvironment {
public:
    std::unique_lock<std::mutex> lock() {
        return std::unique_lock<std::mutex>(mutex_);
    }

    GRBEnv& acquire(
        const SpConfig& config,
        SpResult& result) {
        const EnvironmentSettings requested = environmentSettings(config);
        if (environment_ && !(settings_ == requested)) {
            environment_.reset();
        }
        if (environment_) {
            result.environmentReused = true;
            return *environment_;
        }

        ++result.environmentStartAttempts;
        std::unique_ptr<GRBEnv> candidate(new GRBEnv(true));
        candidate->set(GRB_IntParam_ThreadLimit, requested.threadLimit);
        if (requested.tokenDurationMinutes > 0) {
            candidate->set(
                GRB_IntParam_WLSTokenDuration,
                requested.tokenDurationMinutes);
            candidate->set(
                GRB_DoubleParam_WLSTokenRefresh,
                requested.tokenRefresh);
        }
        candidate->set(
            GRB_IntParam_CSClientLog,
            requested.clientLogLevel);
        candidate->set(
            GRB_IntParam_ServerTimeout,
            requested.serverTimeoutSeconds);
        if (requested.clientLogLevel > 0 &&
            !requested.logFile.empty()) {
            candidate->set(GRB_IntParam_OutputFlag, 1);
            candidate->set(
                GRB_StringParam_LogFile,
                requested.logFile);
        } else {
            candidate->set(GRB_IntParam_OutputFlag, 0);
        }
        candidate->set(GRB_IntParam_LogToConsole, 0);
        candidate->start();

        settings_ = requested;
        environment_ = std::move(candidate);
        return *environment_;
    }

    void reset() noexcept {
        environment_.reset();
    }

private:
    std::mutex mutex_;
    std::unique_ptr<GRBEnv> environment_;
    EnvironmentSettings settings_;
};

ProcessEnvironment& processEnvironment() {
    static ProcessEnvironment environment;
    return environment;
}

bool containsAny(
    const std::string& text,
    const std::vector<std::string>& needles) {
    std::string lowered = text;
    std::transform(
        lowered.begin(),
        lowered.end(),
        lowered.begin(),
        [](const unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });
    for (const std::string& needle : needles) {
        if (lowered.find(needle) != std::string::npos) {
            return true;
        }
    }
    return false;
}

bool retryableEnvironmentError(const GRBException& error) {
    if (error.getErrorCode() == GRB_ERROR_NETWORK ||
        error.getErrorCode() == GRB_ERROR_JOB_REJECTED) {
        return true;
    }
    if (error.getErrorCode() != GRB_ERROR_NO_LICENSE) {
        return false;
    }
    return containsAny(
        error.getMessage(),
        {
            "wls",
            "token",
            "tls",
            "ssl",
            "http",
            "network",
            "connect",
            "session",
            "expired",
            "refresh",
        });
}

int retryDelayMilliseconds(
    const SpConfig& config,
    const int failedAttempt) {
    const int exponent = std::min(failedAttempt - 1, 6);
    const std::int64_t exponential =
        static_cast<std::int64_t>(
            config.environmentRetryBaseMilliseconds) *
        (static_cast<std::int64_t>(1) << exponent);
    std::uint32_t mixed = 2166136261U;
    for (const unsigned char character : config.logFile) {
        mixed ^= character;
        mixed *= 16777619U;
    }
    mixed ^=
        static_cast<std::uint32_t>(config.seed) * 2654435761U;
    mixed ^=
        static_cast<std::uint32_t>(failedAttempt) * 2246822519U;
    const int jitter = static_cast<int>(mixed % 101U);
    return static_cast<int>(
        std::min<std::int64_t>(exponential + jitter, 60000));
}

bool validateSelection(
    const SpProblem& problem,
    const std::vector<StandaloneRoute>& routes,
    const std::vector<std::size_t>& selected,
    std::string& diagnostic,
    double& recomputedObjective) {
    std::vector<int> coverage(problem.customerDemands.size(), 0);
    std::vector<std::int64_t> depotLoad(problem.depots.size(), 0);
    std::vector<std::size_t> depotRouteCount(problem.depots.size(), 0);
    std::vector<bool> depotUsed(problem.depots.size(), false);
    recomputedObjective = 0.0;

    for (const std::size_t routeIndex : selected) {
        if (routeIndex >= routes.size()) {
            diagnostic = "Gurobi returned an out-of-range route index";
            return false;
        }
        const StandaloneRoute& route = routes[routeIndex];
        depotLoad[route.locId] += route.totalLoad;
        ++depotRouteCount[route.locId];
        depotUsed[route.locId] = true;
        recomputedObjective +=
            route.routeCost + problem.vehicleFixedCost;
        if (!std::isfinite(recomputedObjective)) {
            diagnostic = "selected route objective is not finite";
            return false;
        }
        for (const int customer : route.sequence) {
            ++coverage[customer];
        }
    }
    for (std::size_t customer = 0; customer < coverage.size(); ++customer) {
        if (coverage[customer] != 1) {
            diagnostic = "selected routes do not form an exact customer partition";
            return false;
        }
    }
    for (std::size_t depot = 0; depot < problem.depots.size(); ++depot) {
        if (depotLoad[depot] >
            static_cast<std::int64_t>(problem.depots[depot].capacity)) {
            diagnostic = "selected routes exceed a depot capacity";
            return false;
        }
        if (problem.depots[depot].representationRouteLimit.has_value() &&
            depotRouteCount[depot] >
                static_cast<std::size_t>(
                    *problem.depots[depot].representationRouteLimit)) {
            diagnostic = "selected routes exceed the fixed-slot representation limit";
            return false;
        }
        if (depotUsed[depot]) {
            recomputedObjective += problem.depots[depot].openingCost;
            if (!std::isfinite(recomputedObjective)) {
                diagnostic = "selected solution objective is not finite";
                return false;
            }
        }
    }
    return true;
}

std::string gurobiVersion() {
    int major = 0;
    int minor = 0;
    int technical = 0;
    GRBversion(&major, &minor, &technical);
    return std::to_string(major) + "." +
           std::to_string(minor) + "." +
           std::to_string(technical);
}

SpStatus mapStatus(const int nativeStatus) {
    switch (nativeStatus) {
        case GRB_OPTIMAL:
            return SpStatus::optimal;
        case GRB_TIME_LIMIT:
            return SpStatus::time_limit;
        case GRB_INFEASIBLE:
            return SpStatus::infeasible;
        case GRB_INF_OR_UNBD:
            return SpStatus::inf_or_unbounded;
        case GRB_UNBOUNDED:
            return SpStatus::unbounded;
        case GRB_CUTOFF:
            return SpStatus::cutoff;
        case GRB_ITERATION_LIMIT:
            return SpStatus::iteration_limit;
        case GRB_NODE_LIMIT:
            return SpStatus::node_limit;
        case GRB_SOLUTION_LIMIT:
            return SpStatus::solution_limit;
        case GRB_INTERRUPTED:
            return SpStatus::interrupted;
        case GRB_SUBOPTIMAL:
            return SpStatus::suboptimal;
        case GRB_NUMERIC:
            return SpStatus::numeric;
        case GRB_USER_OBJ_LIMIT:
            return SpStatus::user_objective_limit;
        case GRB_WORK_LIMIT:
            return SpStatus::work_limit;
        case GRB_MEM_LIMIT:
            return SpStatus::memory_limit;
        default:
            return SpStatus::error;
    }
}

std::string statusName(const int nativeStatus) {
    switch (nativeStatus) {
        case GRB_LOADED:
            return "LOADED";
        case GRB_OPTIMAL:
            return "OPTIMAL";
        case GRB_INFEASIBLE:
            return "INFEASIBLE";
        case GRB_INF_OR_UNBD:
            return "INF_OR_UNBD";
        case GRB_UNBOUNDED:
            return "UNBOUNDED";
        case GRB_CUTOFF:
            return "CUTOFF";
        case GRB_ITERATION_LIMIT:
            return "ITERATION_LIMIT";
        case GRB_NODE_LIMIT:
            return "NODE_LIMIT";
        case GRB_TIME_LIMIT:
            return "TIME_LIMIT";
        case GRB_SOLUTION_LIMIT:
            return "SOLUTION_LIMIT";
        case GRB_INTERRUPTED:
            return "INTERRUPTED";
        case GRB_NUMERIC:
            return "NUMERIC";
        case GRB_SUBOPTIMAL:
            return "SUBOPTIMAL";
        case GRB_INPROGRESS:
            return "INPROGRESS";
        case GRB_USER_OBJ_LIMIT:
            return "USER_OBJ_LIMIT";
        case GRB_WORK_LIMIT:
            return "WORK_LIMIT";
        case GRB_MEM_LIMIT:
            return "MEM_LIMIT";
        default:
            return "STATUS_" + std::to_string(nativeStatus);
    }
}

}  // namespace

class SPSolver::Impl {
public:
    explicit Impl(SpConfig value) : config(std::move(value)) {}

    SpConfig config;

    struct Attempt {
        SpResult result;
        bool retryableEnvironmentFailure = false;
    };

    static bool sameProblem(
        const SpProblem& first,
        const SpProblem& second) {
        if (first.name != second.name ||
            first.customerDemands != second.customerDemands ||
            first.vehicleCapacity != second.vehicleCapacity ||
            first.vehicleFixedCost != second.vehicleFixedCost ||
            first.depots.size() != second.depots.size()) {
            return false;
        }
        for (std::size_t depot = 0; depot < first.depots.size(); ++depot) {
            if (first.depots[depot].openingCost !=
                    second.depots[depot].openingCost ||
                first.depots[depot].capacity !=
                    second.depots[depot].capacity ||
                first.depots[depot].representationRouteLimit !=
                    second.depots[depot].representationRouteLimit) {
                return false;
            }
        }
        return true;
    }

    bool mapPreviousSelection(
        const SpProblem& problem,
        const std::vector<StandaloneRoute>& routes,
        std::vector<std::size_t>& mapped) const {
        mapped.clear();
        if (!previousProblem.has_value() ||
            previousSelection.empty() ||
            !sameProblem(*previousProblem, problem)) {
            return false;
        }

        std::unordered_map<
            StandaloneRoute,
            std::size_t,
            StandaloneRouteHash>
            currentIndices;
        currentIndices.reserve(routes.size());
        for (std::size_t route = 0; route < routes.size(); ++route) {
            currentIndices.emplace(routes[route], route);
        }

        mapped.reserve(previousSelection.size());
        for (const StandaloneRoute& previous : previousSelection) {
            const auto found = currentIndices.find(previous);
            if (found == currentIndices.end()) {
                mapped.clear();
                return false;
            }
            mapped.push_back(found->second);
        }

        std::string validationError;
        double objective = 0.0;
        if (!validateSelection(
                problem,
                routes,
                mapped,
                validationError,
                objective)) {
            mapped.clear();
            return false;
        }
        return true;
    }

    void rememberSelection(
        const SpProblem& problem,
        const std::vector<StandaloneRoute>& routes,
        const std::vector<std::size_t>& selected) {
        previousProblem = problem;
        previousSelection.clear();
        previousSelection.reserve(selected.size());
        for (const std::size_t route : selected) {
            previousSelection.push_back(routes[route]);
        }
    }

    Attempt solveAttempt(
        const SpProblem& problem,
        const std::vector<StandaloneRoute>& routes,
        const Clock::time_point overallStarted) {
        Attempt attempt;
        SpResult& result = attempt.result;
        result.solverVersion = gurobiVersion();

        enum class Phase {
            environment,
            build,
            optimize,
            postprocess,
        };

        Phase phase = Phase::environment;
        Clock::time_point phaseStarted = Clock::now();
        auto recordInterruptedPhase = [&] {
            const double elapsed = elapsedSeconds(phaseStarted);
            switch (phase) {
                case Phase::environment:
                    result.environmentTimeSeconds += elapsed;
                    break;
                case Phase::build:
                    result.buildTimeSeconds += elapsed;
                    break;
                case Phase::optimize:
                    result.optimizeTimeSeconds += elapsed;
                    break;
                case Phase::postprocess:
                    result.postprocessTimeSeconds += elapsed;
                    break;
            }
        };

        try {
            ProcessEnvironment& environmentManager =
                processEnvironment();
            std::unique_lock<std::mutex> environmentLock =
                environmentManager.lock();

            if (elapsedSeconds(overallStarted) >=
                config.timeLimitSeconds) {
                result.status = SpStatus::time_limit;
                result.statusName = "TIME_LIMIT";
                result.diagnostic =
                    "SP time budget was exhausted before acquiring Gurobi";
                result.environmentTimeSeconds +=
                    elapsedSeconds(phaseStarted);
                return attempt;
            }

            GRBEnv& environment =
                environmentManager.acquire(config, result);
            result.environmentTimeSeconds += elapsedSeconds(phaseStarted);
            if (elapsedSeconds(overallStarted) >=
                config.timeLimitSeconds) {
                result.status = SpStatus::time_limit;
                result.statusName = "TIME_LIMIT";
                result.diagnostic =
                    "SP time budget was exhausted while acquiring Gurobi";
                return attempt;
            }

            phase = Phase::build;
            phaseStarted = Clock::now();

            GRBModel model(environment);
            model.set(GRB_IntParam_OutputFlag, config.outputFlag);
            model.set(GRB_IntParam_LogToConsole, config.outputFlag);
            if (!config.logFile.empty()) {
                model.set(GRB_StringParam_LogFile, config.logFile);
            }
            model.set(GRB_DoubleParam_MIPGap, config.mipGap);
            model.set(GRB_IntParam_Threads, config.threads);
            model.set(GRB_IntParam_Seed, config.seed);
            model.set(GRB_IntParam_MIPFocus, config.mipFocus);
            model.set(GRB_DoubleParam_Heuristics, config.heuristics);
            if (config.workLimit > 0.0) {
                model.set(GRB_DoubleParam_WorkLimit, config.workLimit);
            }

            const std::size_t numRoutes = routes.size();
            const std::size_t numCustomers =
                problem.customerDemands.size();
            const std::size_t numDepots = problem.depots.size();

            std::vector<GRBVar> routeVariables;
            routeVariables.reserve(numRoutes);
            for (std::size_t route = 0; route < numRoutes; ++route) {
                routeVariables.push_back(model.addVar(
                    0.0,
                    1.0,
                    routes[route].routeCost +
                        problem.vehicleFixedCost,
                    GRB_BINARY,
                    "route_" + std::to_string(route)));
            }

            std::vector<GRBVar> depotVariables;
            depotVariables.reserve(numDepots);
            for (std::size_t depot = 0; depot < numDepots; ++depot) {
                depotVariables.push_back(model.addVar(
                    0.0,
                    1.0,
                    problem.depots[depot].openingCost,
                    GRB_BINARY,
                    "depot_" + std::to_string(depot)));
            }
            model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

            std::vector<GRBLinExpr> customerCoverage(numCustomers);
            std::vector<GRBLinExpr> depotLoad(numDepots);
            std::vector<GRBLinExpr> depotRouteCount(numDepots);

            for (std::size_t route = 0; route < numRoutes; ++route) {
                const StandaloneRoute& column = routes[route];
                depotLoad[column.locId] +=
                    static_cast<double>(column.totalLoad) *
                    routeVariables[route];
                depotRouteCount[column.locId] +=
                    routeVariables[route];
                for (const int customer : column.sequence) {
                    customerCoverage[customer] +=
                        routeVariables[route];
                }

                // Strong facility-route linking. The previous aggregate
                // candidate-count Big-M was valid only for integer solutions
                // and provided a very weak LP relaxation.
                model.addConstr(
                    routeVariables[route] <=
                        depotVariables[column.locId],
                    "link_" + std::to_string(route));
            }

            for (std::size_t customer = 0;
                 customer < numCustomers;
                 ++customer) {
                model.addConstr(
                    customerCoverage[customer] == 1.0,
                    "cover_" + std::to_string(customer));
            }
            for (std::size_t depot = 0; depot < numDepots; ++depot) {
                model.addConstr(
                    depotLoad[depot] <=
                        static_cast<double>(
                            problem.depots[depot].capacity) *
                            depotVariables[depot],
                    "capacity_" + std::to_string(depot));

                if (problem.depots[depot]
                        .representationRouteLimit.has_value()) {
                    model.addConstr(
                        depotRouteCount[depot] <=
                            *problem.depots[depot]
                                 .representationRouteLimit,
                        "representation_" +
                            std::to_string(depot));
                }
            }

            std::vector<std::size_t> startSelection;
            if (mapPreviousSelection(
                    problem,
                    routes,
                    startSelection)) {
                for (GRBVar& variable : routeVariables) {
                    variable.set(GRB_DoubleAttr_Start, 0.0);
                }
                for (GRBVar& variable : depotVariables) {
                    variable.set(GRB_DoubleAttr_Start, 0.0);
                }
                for (const std::size_t route : startSelection) {
                    routeVariables[route].set(
                        GRB_DoubleAttr_Start,
                        1.0);
                    depotVariables[routes[route].locId].set(
                        GRB_DoubleAttr_Start,
                        1.0);
                }
                result.mipStartApplied = true;
                result.mipStartRouteCount =
                    static_cast<int>(startSelection.size());
            }

            model.update();
            result.modelVariables =
                model.get(GRB_IntAttr_NumVars);
            result.modelConstraints =
                model.get(GRB_IntAttr_NumConstrs);
            result.modelNonzeros =
                model.get(GRB_IntAttr_NumNZs);
            result.buildTimeSeconds += elapsedSeconds(phaseStarted);

            const double remaining =
                config.timeLimitSeconds -
                elapsedSeconds(overallStarted);
            if (remaining <= 0.0) {
                result.status = SpStatus::time_limit;
                result.statusName = "TIME_LIMIT";
                result.diagnostic =
                    "SP time budget was exhausted while constructing the model";
                return attempt;
            }

            model.set(GRB_DoubleParam_TimeLimit, remaining);
            phase = Phase::optimize;
            phaseStarted = Clock::now();
            model.optimize();
            result.optimizeTimeSeconds += elapsedSeconds(phaseStarted);

            phase = Phase::postprocess;
            phaseStarted = Clock::now();

            result.nativeStatus =
                model.get(GRB_IntAttr_Status);
            result.status = mapStatus(result.nativeStatus);
            result.statusName = statusName(result.nativeStatus);
            result.solveTimeSeconds =
                model.get(GRB_DoubleAttr_Runtime);
            result.solutionCount =
                model.get(GRB_IntAttr_SolCount);

            try {
                result.bestBound =
                    model.get(GRB_DoubleAttr_ObjBound);
            } catch (const GRBException&) {
                result.bestBound.reset();
            }
            try {
                result.work = model.get(GRB_DoubleAttr_Work);
            } catch (const GRBException&) {
                result.work.reset();
            }
            try {
                result.nodeCount =
                    model.get(GRB_DoubleAttr_NodeCount);
            } catch (const GRBException&) {
                result.nodeCount.reset();
            }

            if (result.solutionCount > 0) {
                const double objective =
                    model.get(GRB_DoubleAttr_ObjVal);
                std::optional<double> mipGap;
                try {
                    mipGap =
                        model.get(GRB_DoubleAttr_MIPGap);
                } catch (const GRBException&) {
                    mipGap.reset();
                }

                std::vector<std::size_t>
                    selectedRouteIndices;
                for (std::size_t route = 0;
                     route < numRoutes;
                     ++route) {
                    if (routeVariables[route].get(
                            GRB_DoubleAttr_X) > 0.5) {
                        selectedRouteIndices.push_back(route);
                    }
                }

                std::string validationError;
                double recomputedObjective = 0.0;
                if (!validateSelection(
                        problem,
                        routes,
                        selectedRouteIndices,
                        validationError,
                        recomputedObjective) ||
                    std::fabs(
                        recomputedObjective - objective) >
                        1.0e-6 *
                            std::max(
                                1.0,
                                std::fabs(objective))) {
                    if (validationError.empty()) {
                        validationError =
                            "Gurobi objective does not match "
                            "the selected routes";
                    }
                    result.status = SpStatus::error;
                    result.statusName = "INVALID_SOLUTION";
                    result.errorCode = -1;
                    result.diagnostic =
                        std::move(validationError);
                    result.selectedRouteIndices.clear();
                    result.objective.reset();
                    result.mipGap.reset();
                    result.incumbentValidated = false;
                } else {
                    result.selectedRouteIndices =
                        std::move(selectedRouteIndices);
                    result.objective = objective;
                    result.mipGap = mipGap;
                    result.incumbentValidated = true;
                }
            } else {
                result.diagnostic =
                    "Gurobi finished without a feasible incumbent";
            }

            result.postprocessTimeSeconds +=
                elapsedSeconds(phaseStarted);
        } catch (const GRBException& error) {
            recordInterruptedPhase();
            result.status = SpStatus::error;
            result.statusName = "GUROBI_ERROR";
            result.errorCode = error.getErrorCode();
            result.diagnostic = error.getMessage();
            attempt.retryableEnvironmentFailure =
                retryableEnvironmentError(error);
            if (attempt.retryableEnvironmentFailure) {
                ProcessEnvironment& environmentManager =
                    processEnvironment();
                std::unique_lock<std::mutex> resetLock =
                    environmentManager.lock();
                environmentManager.reset();
            }
        } catch (const std::exception& error) {
            recordInterruptedPhase();
            result.status = SpStatus::error;
            result.statusName = "ERROR";
            result.diagnostic = error.what();
        } catch (...) {
            recordInterruptedPhase();
            result.status = SpStatus::error;
            result.statusName = "ERROR";
            result.diagnostic =
                "unknown exception while solving set partitioning";
        }
        return attempt;
    }

private:
    std::optional<SpProblem> previousProblem;
    std::vector<StandaloneRoute> previousSelection;
};

SPSolver::SPSolver(SpConfig config)
    : impl_(new Impl(std::move(config))) {}

SPSolver::~SPSolver() = default;
SPSolver::SPSolver(SPSolver&&) noexcept = default;
SPSolver& SPSolver::operator=(SPSolver&&) noexcept = default;

const SpConfig& SPSolver::config() const noexcept {
    return impl_->config;
}

SpResult SPSolver::solve(
    const SpProblem& problem,
    const std::vector<StandaloneRoute>& routes) {
    const Clock::time_point started = Clock::now();
    const Clock::time_point validationStarted = Clock::now();
    const std::string inputError = validateInput(impl_->config, problem, routes);
    const double validationTime =
        elapsedSeconds(validationStarted);
    if (!inputError.empty()) {
        SpResult result = makeResult(
            SpStatus::invalid_input,
            "INVALID_INPUT",
            inputError);
        result.validationTimeSeconds = validationTime;
        result.totalTimeSeconds = elapsedSeconds(started);
        return result;
    }

    double accumulatedEnvironmentTime = 0.0;
    double accumulatedBuildTime = 0.0;
    double accumulatedOptimizeTime = 0.0;
    double accumulatedPostprocessTime = 0.0;
    double retrySleepTime = 0.0;
    int accumulatedEnvironmentStarts = 0;
    bool environmentWasReused = false;

    for (int attemptNumber = 1;
         attemptNumber <= impl_->config.environmentMaxAttempts;
         ++attemptNumber) {
        Impl::Attempt attempt =
            impl_->solveAttempt(problem, routes, started);
        SpResult& result = attempt.result;

        result.validationTimeSeconds = validationTime;
        result.environmentTimeSeconds +=
            accumulatedEnvironmentTime;
        result.buildTimeSeconds += accumulatedBuildTime;
        result.optimizeTimeSeconds +=
            accumulatedOptimizeTime;
        result.postprocessTimeSeconds +=
            accumulatedPostprocessTime;
        result.retrySleepTimeSeconds = retrySleepTime;
        result.environmentStartAttempts +=
            accumulatedEnvironmentStarts;
        result.environmentReused =
            result.environmentReused || environmentWasReused;
        result.environmentRetryCount = attemptNumber - 1;

        if (!attempt.retryableEnvironmentFailure ||
            attemptNumber >=
                impl_->config.environmentMaxAttempts) {
            if (attempt.retryableEnvironmentFailure &&
                attemptNumber >=
                    impl_->config.environmentMaxAttempts) {
                result.diagnostic +=
                    " (retryable environment failure; attempts exhausted)";
            }
            if (result.hasFeasibleSolution()) {
                impl_->rememberSelection(
                    problem,
                    routes,
                    result.selectedRouteIndices);
            }
            result.totalTimeSeconds = elapsedSeconds(started);
            return result;
        }

        accumulatedEnvironmentTime =
            result.environmentTimeSeconds;
        accumulatedBuildTime = result.buildTimeSeconds;
        accumulatedOptimizeTime = result.optimizeTimeSeconds;
        accumulatedPostprocessTime =
            result.postprocessTimeSeconds;
        accumulatedEnvironmentStarts =
            result.environmentStartAttempts;
        environmentWasReused = result.environmentReused;

        const int delayMilliseconds =
            retryDelayMilliseconds(
                impl_->config,
                attemptNumber);
        if (elapsedSeconds(started) +
                static_cast<double>(delayMilliseconds) / 1000.0 >=
            impl_->config.timeLimitSeconds) {
            result.diagnostic +=
                " (retry skipped because the SP time budget is exhausted)";
            result.totalTimeSeconds = elapsedSeconds(started);
            return result;
        }
        const Clock::time_point retrySleepStarted = Clock::now();
        std::this_thread::sleep_for(
            std::chrono::milliseconds(delayMilliseconds));
        retrySleepTime += elapsedSeconds(retrySleepStarted);
    }

    SpResult result = makeResult(
        SpStatus::error,
        "ERROR",
        "internal error: SP retry loop exited unexpectedly");
    result.validationTimeSeconds = validationTime;
    result.totalTimeSeconds = elapsedSeconds(started);
    return result;
}
