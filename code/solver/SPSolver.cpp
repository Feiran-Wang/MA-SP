#include "SPSolver.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
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
    if (!std::isfinite(config.mipGap) || config.mipGap < 0.0) {
        return "SP MIP gap must be finite and non-negative";
    }
    if (config.threads < 1) {
        return "SP thread count must be positive";
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

// Gurobi environments are not shared across experiment worker processes. Within
// one process, SP calls reuse this lazily started environment and are serialized
// so no two models access it concurrently.
GRBEnv& processEnvironment() {
    static const std::unique_ptr<GRBEnv> environment = [] {
        std::unique_ptr<GRBEnv> candidate(new GRBEnv(true));
        candidate->set(GRB_IntParam_OutputFlag, 0);
        candidate->start();
        return candidate;
    }();
    return *environment;
}

std::mutex& processEnvironmentMutex() {
    static std::mutex mutex;
    return mutex;
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
    const std::string inputError = validateInput(impl_->config, problem, routes);
    if (!inputError.empty()) {
        SpResult result = makeResult(
            SpStatus::invalid_input,
            "INVALID_INPUT",
            inputError);
        result.totalTimeSeconds = elapsedSeconds(started);
        return result;
    }

    SpResult result;
    result.solverVersion = gurobiVersion();

    try {
        std::unique_lock<std::mutex> environmentLock(
            processEnvironmentMutex());

        if (elapsedSeconds(started) >= impl_->config.timeLimitSeconds) {
            result.status = SpStatus::time_limit;
            result.statusName = "TIME_LIMIT";
            result.diagnostic =
                "SP time budget was exhausted before acquiring Gurobi";
            result.totalTimeSeconds = elapsedSeconds(started);
            return result;
        }

        GRBEnv& environment = processEnvironment();
        if (elapsedSeconds(started) >= impl_->config.timeLimitSeconds) {
            result.status = SpStatus::time_limit;
            result.statusName = "TIME_LIMIT";
            result.diagnostic =
                "SP time budget was exhausted while acquiring Gurobi";
            result.totalTimeSeconds = elapsedSeconds(started);
            return result;
        }

        GRBModel model(environment);
        model.set(GRB_IntParam_OutputFlag, impl_->config.outputFlag);
        if (!impl_->config.logFile.empty()) {
            model.set(GRB_StringParam_LogFile, impl_->config.logFile);
        }
        model.set(GRB_DoubleParam_MIPGap, impl_->config.mipGap);
        model.set(GRB_IntParam_Threads, impl_->config.threads);
        model.set(GRB_IntParam_Seed, impl_->config.seed);
        model.set(GRB_IntParam_MIPFocus, impl_->config.mipFocus);
        model.set(GRB_DoubleParam_Heuristics, impl_->config.heuristics);

        const std::size_t numRoutes = routes.size();
        const std::size_t numCustomers = problem.customerDemands.size();
        const std::size_t numDepots = problem.depots.size();

        std::vector<GRBVar> routeVariables;
        routeVariables.reserve(numRoutes);
        for (std::size_t route = 0; route < numRoutes; ++route) {
            routeVariables.push_back(model.addVar(
                0.0,
                1.0,
                routes[route].routeCost + problem.vehicleFixedCost,
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
        std::vector<int> routesByDepot(numDepots, 0);

        for (std::size_t route = 0; route < numRoutes; ++route) {
            const StandaloneRoute& column = routes[route];
            depotLoad[column.locId] +=
                static_cast<double>(column.totalLoad) * routeVariables[route];
            depotRouteCount[column.locId] += routeVariables[route];
            ++routesByDepot[column.locId];
            for (const int customer : column.sequence) {
                customerCoverage[customer] += routeVariables[route];
            }
        }

        for (std::size_t customer = 0; customer < numCustomers; ++customer) {
            model.addConstr(
                customerCoverage[customer] == 1.0,
                "cover_" + std::to_string(customer));
        }
        for (std::size_t depot = 0; depot < numDepots; ++depot) {
            model.addConstr(
                depotLoad[depot] <=
                    static_cast<double>(problem.depots[depot].capacity) *
                        depotVariables[depot],
                "capacity_" + std::to_string(depot));

            // This is only a linking constraint. Unlike the optional
            // representation bound, it does not impose a fleet limit.
            model.addConstr(
                depotRouteCount[depot] <=
                    static_cast<double>(routesByDepot[depot]) *
                        depotVariables[depot],
                "open_" + std::to_string(depot));

            if (problem.depots[depot].representationRouteLimit.has_value()) {
                model.addConstr(
                    depotRouteCount[depot] <=
                        *problem.depots[depot].representationRouteLimit,
                    "representation_" + std::to_string(depot));
            }
        }

        const double remaining =
            impl_->config.timeLimitSeconds - elapsedSeconds(started);
        if (remaining <= 0.0) {
            result.status = SpStatus::time_limit;
            result.statusName = "TIME_LIMIT";
            result.solverVersion = gurobiVersion();
            result.diagnostic =
                "SP time budget was exhausted while constructing the model";
            result.totalTimeSeconds = elapsedSeconds(started);
            return result;
        }
        model.set(GRB_DoubleParam_TimeLimit, remaining);
        model.optimize();

        result.nativeStatus = model.get(GRB_IntAttr_Status);
        result.status = mapStatus(result.nativeStatus);
        result.statusName = statusName(result.nativeStatus);
        result.solveTimeSeconds = model.get(GRB_DoubleAttr_Runtime);
        result.solutionCount = model.get(GRB_IntAttr_SolCount);

        try {
            result.bestBound = model.get(GRB_DoubleAttr_ObjBound);
        } catch (const GRBException&) {
            result.bestBound.reset();
        }

        if (result.solutionCount > 0) {
            const double objective = model.get(GRB_DoubleAttr_ObjVal);
            std::optional<double> mipGap;
            try {
                mipGap = model.get(GRB_DoubleAttr_MIPGap);
            } catch (const GRBException&) {
                mipGap.reset();
            }

            std::vector<std::size_t> selectedRouteIndices;
            for (std::size_t route = 0; route < numRoutes; ++route) {
                if (routeVariables[route].get(GRB_DoubleAttr_X) > 0.5) {
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
                        std::max(1.0, std::fabs(objective))) {
                if (validationError.empty()) {
                    validationError =
                        "Gurobi objective does not match the selected routes";
                }
                result.status = SpStatus::error;
                result.statusName = "INVALID_SOLUTION";
                result.errorCode = -1;
                result.diagnostic = std::move(validationError);
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
            result.diagnostic = "Gurobi finished without a feasible incumbent";
        }
    } catch (const GRBException& error) {
        result.status = SpStatus::error;
        result.statusName = "GUROBI_ERROR";
        result.errorCode = error.getErrorCode();
        result.diagnostic = error.getMessage();
    } catch (const std::exception& error) {
        result.status = SpStatus::error;
        result.statusName = "ERROR";
        result.diagnostic = error.what();
    } catch (...) {
        result.status = SpStatus::error;
        result.statusName = "ERROR";
        result.diagnostic = "unknown exception while solving set partitioning";
    }

    result.totalTimeSeconds = elapsedSeconds(started);
    return result;
}
