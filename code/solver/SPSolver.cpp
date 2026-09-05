#include "SPSolver.h"
#include "SpInputValidation.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <ctime>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "gurobi_c++.h"

namespace {

using WallClock = std::chrono::steady_clock;

double elapsedWallSeconds(const WallClock::time_point start) {
    return std::chrono::duration<double>(
        WallClock::now() - start).count();
}

double elapsedCpuSeconds(const std::clock_t start) {
    return static_cast<double>(std::clock() - start) /
           static_cast<double>(CLOCKS_PER_SEC);
}

constexpr std::size_t kMaximumMipIncumbentObservations = 4096U;

bool strictlyImprovesObjective(
        const double candidate,
        const double incumbent) noexcept {
    const double scale = std::max(
        1.0, std::max(std::fabs(candidate), std::fabs(incumbent)));
    return candidate < incumbent - 1.0e-9 * scale;
}

class MipIncumbentTraceCallback final : public GRBCallback {
public:
    MipIncumbentTraceCallback(
            const std::clock_t processCpuStarted,
            const std::optional<double> submittedStartObjective,
            const double primalIdleTimeLimitSeconds,
            const double improvementExtensionGateSeconds)
        : processCpuStarted_(processCpuStarted),
          submittedStartObjective_(submittedStartObjective),
          primalIdleTimeLimitSeconds_(primalIdleTimeLimitSeconds),
          improvementExtensionGateSeconds_(
              improvementExtensionGateSeconds) {}

    const std::vector<SpMipIncumbentObservation>& observations() const {
        return observations_;
    }

    bool truncated() const noexcept {
        return truncated_;
    }

    const std::string& diagnostic() const noexcept {
        return diagnostic_;
    }

    bool terminatedByPrimalIdle() const noexcept {
        return terminatedByPrimalIdle_;
    }

    bool terminatedByNoImprovementGate() const noexcept {
        return terminatedByNoImprovementGate_;
    }

protected:
    void callback() override {
        if ((where != GRB_CB_MIPSOL && where != GRB_CB_MIP) ||
            !diagnostic_.empty()) {
            return;
        }
        try {
            const double runtime = getDoubleInfo(GRB_CB_RUNTIME);
            if (where == GRB_CB_MIP) {
                if (improvementExtensionGateSeconds_ > 0.0 &&
                    runtime >= improvementExtensionGateSeconds_ &&
                    !submittedStartImproved_) {
                    terminatedByNoImprovementGate_ = true;
                    abort();
                    return;
                }
                if (primalIdleTimeLimitSeconds_ > 0.0 &&
                    lastIncumbentWallSeconds_.has_value() &&
                    runtime - *lastIncumbentWallSeconds_ >=
                        primalIdleTimeLimitSeconds_) {
                    terminatedByPrimalIdle_ = true;
                    abort();
                }
                return;
            }
            const double objective = getDoubleInfo(GRB_CB_MIPSOL_OBJ);
            if (!std::isfinite(objective) ||
                (lastObjective_.has_value() &&
                 !strictlyImprovesObjective(
                     objective, *lastObjective_))) {
                return;
            }
            lastObjective_ = objective;
            lastIncumbentWallSeconds_ = runtime;
            if (observations_.size() >=
                kMaximumMipIncumbentObservations) {
                truncated_ = true;
                return;
            }
            SpMipIncumbentObservation observation;
            observation.ordinal =
                static_cast<int>(observations_.size() + 1U);
            observation.objective = objective;
            observation.bestBound =
                getDoubleInfo(GRB_CB_MIPSOL_OBJBND);
            observation.solverWallSeconds = runtime;
            observation.processCpuSeconds =
                elapsedCpuSeconds(processCpuStarted_);
            observation.work = getDoubleInfo(GRB_CB_WORK);
            observation.nodeCount =
                getDoubleInfo(GRB_CB_MIPSOL_NODCNT);
            observation.improvesSubmittedStart =
                submittedStartObjective_.has_value() &&
                strictlyImprovesObjective(
                    objective, *submittedStartObjective_);
            submittedStartImproved_ = submittedStartImproved_ ||
                observation.improvesSubmittedStart;
            observations_.push_back(std::move(observation));
        } catch (const GRBException& error) {
            diagnostic_ = error.getMessage();
        } catch (const std::exception& error) {
            diagnostic_ = error.what();
        } catch (...) {
            diagnostic_ = "unknown incumbent-trace callback failure";
        }
    }

private:
    std::clock_t processCpuStarted_;
    std::optional<double> submittedStartObjective_;
    std::optional<double> lastObjective_;
    std::optional<double> lastIncumbentWallSeconds_;
    std::vector<SpMipIncumbentObservation> observations_;
    double primalIdleTimeLimitSeconds_ = 0.0;
    double improvementExtensionGateSeconds_ = 0.0;
    bool submittedStartImproved_ = false;
    bool terminatedByNoImprovementGate_ = false;
    bool truncated_ = false;
    bool terminatedByPrimalIdle_ = false;
    std::string diagnostic_;
};

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

std::string validateInputImpl(
    const SpConfig& config,
    const SpProblem& problem,
    const std::vector<StandaloneRoute>& routes) {
    if (!std::isfinite(config.timeLimitSeconds) || config.timeLimitSeconds <= 0.0) {
        return "SP time limit must be finite and positive";
    }
    if (!std::isfinite(config.primalIdleTimeLimitSeconds) ||
        config.primalIdleTimeLimitSeconds < 0.0) {
        return "SP primal idle time limit must be finite and non-negative";
    }
    if (!std::isfinite(config.improvementExtensionGateSeconds) ||
        config.improvementExtensionGateSeconds < 0.0 ||
        (config.improvementExtensionGateSeconds > 0.0 &&
         config.improvementExtensionGateSeconds >=
             config.timeLimitSeconds)) {
        return "SP improvement-extension gate must be zero or smaller than "
               "the hard time limit";
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
    if (!std::isfinite(config.lpTimeLimitSeconds) ||
        config.lpTimeLimitSeconds <= 0.0) {
        return "LP-RMP time limit must be finite and positive";
    }
    if (config.solutionPoolSize < 1 ||
        config.solutionPoolSize > 1000) {
        return "SP solution-pool size must be between 1 and 1000";
    }
    if (!std::isfinite(config.solutionPoolGap) ||
        config.solutionPoolGap < 0.0) {
        return "SP solution-pool gap must be finite and non-negative";
    }
    if (config.solutionPoolSearchMode < 0 ||
        config.solutionPoolSearchMode > 2) {
        return "SP solution-pool search mode must be between 0 and 2";
    }
    if (problem.customerDemands.empty()) {
        return "SP requires at least one customer";
    }
    if (problem.depots.empty()) {
        return "SP requires at least one depot";
    }
    if (!problem.fixedDepotOpen.empty() &&
        problem.fixedDepotOpen.size() != problem.depots.size()) {
        return "fixed depot configuration size does not match the depots";
    }
    if (!problem.fixedDepotOpen.empty() &&
        std::none_of(
            problem.fixedDepotOpen.begin(),
            problem.fixedDepotOpen.end(),
            [](const bool open) { return open; })) {
        return "fixed depot configuration must open at least one depot";
    }
    if (!problem.fixedDepotOpen.empty() &&
        !problem.excludedDepotOpenConfigurations.empty()) {
        return "fixed and excluded depot configurations cannot be combined";
    }
    for (const auto& excluded :
         problem.excludedDepotOpenConfigurations) {
        if (excluded.size() != problem.depots.size()) {
            return "excluded depot configuration size does not match the depots";
        }
        if (std::none_of(
                excluded.begin(),
                excluded.end(),
                [](const bool open) { return open; })) {
            return "excluded depot configuration must open at least one depot";
        }
    }
    if (problem.vehicleCapacity <= 0) {
        return "vehicle capacity must be positive";
    }
    if (!std::isfinite(problem.vehicleFixedCost) ||
        problem.vehicleFixedCost < 0.0) {
        return "vehicle fixed cost must be finite and non-negative";
    }
    for (std::size_t customer = 0; customer < problem.customerDemands.size(); ++customer) {
        if (problem.customerDemands[customer] < 0) {
            return "customer demands must be non-negative";
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
        if (!problem.fixedDepotOpen.empty() &&
            !problem.fixedDepotOpen[route.locId]) {
            return "route pool contains a route outside the fixed depot "
                   "configuration";
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
        if (route.totalLoad < 0 ||
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
    // Environment identity must not depend on a local/global model budget;
    // otherwise alternating local and global RMP calls would tear down the
    // shared environment and invalidate both persistent LP masters.
    settings.serverTimeoutSeconds =
        config.serverTimeoutSeconds;
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
        SpResult& result,
        const int startupTimeoutSeconds) {
        const EnvironmentSettings requested = environmentSettings(config);
        if (environment_ && !(settings_ == requested)) {
            environment_.reset();
            ++generation_;
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
            startupTimeoutSeconds);
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
        ++generation_;
        return *environment_;
    }

    void reset() noexcept {
        environment_.reset();
        ++generation_;
    }

    std::uint64_t generation() const noexcept {
        return generation_;
    }

private:
    std::mutex mutex_;
    std::unique_ptr<GRBEnv> environment_;
    EnvironmentSettings settings_;
    std::uint64_t generation_ = 0U;
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
        if (!problem.fixedDepotOpen.empty() &&
            depotUsed[depot] &&
            !problem.fixedDepotOpen[depot]) {
            diagnostic =
                "selected routes use a depot outside the fixed configuration";
            return false;
        }
        if (!problem.fixedDepotOpen.empty() &&
            problem.fixedDepotOpen[depot] &&
            !depotUsed[depot]) {
            diagnostic =
                "selected routes do not use every depot in the fixed "
                "configuration";
            return false;
        }
        const bool chargeOpening =
            problem.fixedDepotOpen.empty()
                ? depotUsed[depot]
                : problem.fixedDepotOpen[depot];
        if (chargeOpening) {
            recomputedObjective += problem.depots[depot].openingCost;
            if (!std::isfinite(recomputedObjective)) {
                diagnostic = "selected solution objective is not finite";
                return false;
            }
        }
    }
    for (const auto& excluded :
         problem.excludedDepotOpenConfigurations) {
        if (depotUsed == excluded) {
            diagnostic =
                "selected routes use an excluded depot configuration";
            return false;
        }
    }
    return true;
}

struct ColumnIdentity {
    int depot = -1;
    std::vector<int> customers;

    bool operator==(const ColumnIdentity& other) const noexcept {
        return depot == other.depot && customers == other.customers;
    }
};

struct ColumnIdentityHash {
    std::size_t operator()(const ColumnIdentity& key) const noexcept {
        std::size_t seed = key.customers.size();
        seed ^= std::hash<int>()(key.depot) +
                0x9e3779b9U +
                (seed << 6U) +
                (seed >> 2U);
        for (const int customer : key.customers) {
            seed ^= std::hash<int>()(customer) +
                    0x9e3779b9U +
                    (seed << 6U) +
                    (seed >> 2U);
        }
        return seed;
    }
};

ColumnIdentity columnIdentity(const StandaloneRoute& route) {
    ColumnIdentity identity;
    identity.depot = route.locId;
    identity.customers = route.sequence;
    std::sort(identity.customers.begin(), identity.customers.end());
    return identity;
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

std::string hgasp::sp_internal::validateInput(
    const SpConfig& config,
    const SpProblem& problem,
    const std::vector<StandaloneRoute>& routes) {
    return validateInputImpl(config, problem, routes);
}

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
            first.fixedDepotOpen != second.fixedDepotOpen ||
            first.excludedDepotOpenConfigurations !=
                second.excludedDepotOpenConfigurations ||
            first.requireOpenDepotUse != second.requireOpenDepotUse ||
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

    // The continuous RMP is kept across solve() calls. New master columns are
    // appended, pruned columns are disabled with UB=0, and dominated columns
    // that reappear are reactivated. The integer model below intentionally
    // remains a current-snapshot model so stale columns can never enter an
    // injected solution.
    struct PersistentLpMaster {
        std::optional<SpProblem> problem;
        GRBEnv* environment = nullptr;
        std::uint64_t environmentGeneration = 0U;
        std::unique_ptr<GRBModel> model;
        std::vector<GRBVar> routeVariables;
        std::vector<GRBVar> depotVariables;
        std::vector<GRBConstr> customerCoverage;
        std::vector<GRBConstr> depotCapacity;
        std::vector<std::optional<GRBConstr>> representationLimits;
        std::vector<std::optional<GRBConstr>> depotUse;
        std::vector<StandaloneRoute> storedRoutes;
        std::vector<bool> active;
        std::unordered_map<
            ColumnIdentity,
            std::size_t,
            ColumnIdentityHash>
            slotByIdentity;

        void clear() noexcept {
            model.reset();
            problem.reset();
            environment = nullptr;
            environmentGeneration = 0U;
            routeVariables.clear();
            depotVariables.clear();
            customerCoverage.clear();
            depotCapacity.clear();
            representationLimits.clear();
            depotUse.clear();
            storedRoutes.clear();
            active.clear();
            slotByIdentity.clear();
        }

        void buildBase(
            GRBEnv& env,
            const std::uint64_t generation,
            const SpProblem& value,
            const SpConfig& config) {
            clear();
            environment = &env;
            environmentGeneration = generation;
            problem = value;
            model.reset(new GRBModel(env));
            model->set(GRB_IntParam_OutputFlag, 0);
            model->set(GRB_IntParam_LogToConsole, 0);
            model->set(GRB_IntParam_Threads, config.threads);
            model->set(GRB_IntParam_Seed, config.seed);
            model->set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

            depotVariables.reserve(value.depots.size());
            for (std::size_t depot = 0;
                 depot < value.depots.size();
                 ++depot) {
                depotVariables.push_back(model->addVar(
                    0.0,
                    1.0,
                    value.depots[depot].openingCost,
                    GRB_CONTINUOUS,
                    "lp_depot_" + std::to_string(depot)));
            }
            model->update();

            customerCoverage.reserve(
                value.customerDemands.size());
            for (std::size_t customer = 0;
                 customer < value.customerDemands.size();
                 ++customer) {
                GRBLinExpr empty = 0.0;
                customerCoverage.push_back(model->addConstr(
                    empty == 1.0,
                    "lp_cover_" + std::to_string(customer)));
            }

            depotCapacity.reserve(value.depots.size());
            representationLimits.resize(value.depots.size());
            depotUse.resize(value.depots.size());
            for (std::size_t depot = 0;
                 depot < value.depots.size();
                 ++depot) {
                GRBLinExpr empty = 0.0;
                depotCapacity.push_back(model->addConstr(
                    empty <=
                        static_cast<double>(
                            value.depots[depot].capacity) *
                            depotVariables[depot],
                    "lp_capacity_" + std::to_string(depot)));
                if (value.depots[depot]
                        .representationRouteLimit.has_value()) {
                    GRBLinExpr routeCount = 0.0;
                    representationLimits[depot] =
                        model->addConstr(
                            routeCount <=
                                *value.depots[depot]
                                     .representationRouteLimit,
                            "lp_representation_" +
                                std::to_string(depot));
                }
                if (!value.fixedDepotOpen.empty()) {
                    model->addConstr(
                        depotVariables[depot] ==
                            (value.fixedDepotOpen[depot]
                                 ? 1.0
                                 : 0.0),
                        "lp_fixed_depot_" +
                            std::to_string(depot));
                }
                const bool requireUse = value.requireOpenDepotUse ||
                    (!value.fixedDepotOpen.empty() &&
                     value.fixedDepotOpen[depot]);
                if (requireUse) {
                    GRBLinExpr routeCount = 0.0;
                    depotUse[depot] = model->addConstr(
                        routeCount >= depotVariables[depot],
                        "lp_use_depot_" + std::to_string(depot));
                }
            }
            for (std::size_t excludedIndex = 0U;
                 excludedIndex <
                     value.excludedDepotOpenConfigurations.size();
                 ++excludedIndex) {
                GRBLinExpr differs = 0.0;
                const auto& excluded =
                    value.excludedDepotOpenConfigurations[excludedIndex];
                for (std::size_t depot = 0U;
                     depot < excluded.size();
                     ++depot) {
                    differs += excluded[depot]
                        ? 1.0 - depotVariables[depot]
                        : depotVariables[depot];
                }
                model->addConstr(
                    differs >= 1.0,
                    "lp_excluded_configuration_" +
                        std::to_string(excludedIndex));
            }
            model->update();
        }

        std::size_t addColumn(
            const StandaloneRoute& route,
            const SpProblem& value) {
            GRBColumn column;
            for (const int customer : route.sequence) {
                column.addTerm(
                    1.0,
                    customerCoverage[
                        static_cast<std::size_t>(customer)]);
            }
            column.addTerm(
                static_cast<double>(route.totalLoad),
                depotCapacity[
                    static_cast<std::size_t>(route.locId)]);
            if (representationLimits[
                    static_cast<std::size_t>(route.locId)]
                    .has_value()) {
                column.addTerm(
                    1.0,
                    *representationLimits[
                        static_cast<std::size_t>(route.locId)]);
            }
            if (depotUse[
                    static_cast<std::size_t>(route.locId)]
                    .has_value()) {
                column.addTerm(
                    1.0,
                    *depotUse[
                        static_cast<std::size_t>(route.locId)]);
            }
            const std::size_t slot = routeVariables.size();
            routeVariables.push_back(model->addVar(
                0.0,
                1.0,
                route.routeCost + value.vehicleFixedCost,
                GRB_CONTINUOUS,
                column,
                "lp_route_" + std::to_string(slot)));
            model->addConstr(
                routeVariables.back() <=
                    depotVariables[
                        static_cast<std::size_t>(route.locId)],
                "lp_link_" + std::to_string(slot));
            storedRoutes.push_back(route);
            active.push_back(true);
            slotByIdentity.emplace(columnIdentity(route), slot);
            return slot;
        }

        void solve(
            GRBEnv& env,
            const std::uint64_t generation,
            const SpProblem& value,
            const std::vector<StandaloneRoute>& routes,
            const SpConfig& config,
            SpResult& result) {
            result.lpAttempted = true;
            const std::clock_t buildStarted = std::clock();
            bool reusable =
                config.persistentLpEnabled &&
                model != nullptr &&
                environment == &env &&
                environmentGeneration == generation &&
                problem.has_value() &&
                Impl::sameProblem(*problem, value);
            if (!reusable) {
                buildBase(
                    env, generation, value, config);
            } else if (
                !routes.empty() &&
                routeVariables.size() >
                    2U * routes.size()) {
                result.persistentLpCompacted = true;
                result.persistentLpColumnsBeforeCompaction =
                    static_cast<int>(routeVariables.size());
                // Rebuilding is bounded by the active snapshot and releases
                // inactive variables/constraints accumulated by incremental
                // synchronization. The Gurobi environment remains reusable.
                buildBase(
                    env, generation, value, config);
                reusable = false;
            }
            result.persistentLpReused = reusable;

            std::vector<std::size_t> currentSlots;
            currentSlots.reserve(routes.size());
            std::vector<bool> present(active.size(), false);
            for (const StandaloneRoute& route : routes) {
                const ColumnIdentity identity =
                    columnIdentity(route);
                const auto found = slotByIdentity.find(identity);
                std::size_t slot = 0U;
                if (found == slotByIdentity.end()) {
                    slot = addColumn(route, value);
                    present.resize(active.size(), false);
                    ++result.persistentLpAddedColumns;
                } else {
                    slot = found->second;
                    if (!active[slot]) {
                        ++result.persistentLpReactivatedColumns;
                    }
                    active[slot] = true;
                    routeVariables[slot].set(
                        GRB_DoubleAttr_UB, 1.0);
                    routeVariables[slot].set(
                        GRB_DoubleAttr_Obj,
                        route.routeCost +
                            value.vehicleFixedCost);
                    storedRoutes[slot] = route;
                }
                present[slot] = true;
                currentSlots.push_back(slot);
            }
            for (std::size_t slot = 0;
                 slot < active.size();
                 ++slot) {
                if (active[slot] && !present[slot]) {
                    active[slot] = false;
                    routeVariables[slot].set(
                        GRB_DoubleAttr_UB, 0.0);
                    ++result.persistentLpDisabledColumns;
                }
            }
            model->update();
            result.persistentLpStoredColumns =
                static_cast<int>(routeVariables.size());
            result.persistentLpActiveColumns =
                static_cast<int>(routes.size());
            result.lpBuildTimeSeconds +=
                elapsedCpuSeconds(buildStarted);

            model->set(
                GRB_DoubleParam_TimeLimit,
                config.lpTimeLimitSeconds);
            const std::clock_t optimizeStarted = std::clock();
            model->optimize();
            result.lpOptimizeTimeSeconds +=
                elapsedCpuSeconds(optimizeStarted);
            result.lpNativeStatus =
                model->get(GRB_IntAttr_Status);
            result.lpStatusName =
                statusName(result.lpNativeStatus);
            if (result.lpNativeStatus != GRB_OPTIMAL) {
                if (!config.persistentLpEnabled) {
                    clear();
                }
                return;
            }

            result.lpObjective =
                model->get(GRB_DoubleAttr_ObjVal);
            result.customerDuals.resize(
                customerCoverage.size());
            for (std::size_t customer = 0;
                 customer < customerCoverage.size();
                 ++customer) {
                result.customerDuals[customer] =
                    customerCoverage[customer].get(
                        GRB_DoubleAttr_Pi);
            }
            result.depotCapacityDuals.resize(
                depotCapacity.size());
            for (std::size_t depot = 0;
                 depot < depotCapacity.size();
                 ++depot) {
                result.depotCapacityDuals[depot] =
                    depotCapacity[depot].get(
                        GRB_DoubleAttr_Pi);
            }
            result.routeReducedCosts.resize(routes.size());
            result.routeLpValues.resize(routes.size());
            for (std::size_t route = 0;
                 route < routes.size();
                 ++route) {
                result.routeReducedCosts[route] =
                    routeVariables[currentSlots[route]].get(
                        GRB_DoubleAttr_RC);
                result.routeLpValues[route] =
                    routeVariables[currentSlots[route]].get(
                        GRB_DoubleAttr_X);
            }
            result.depotLpValues.resize(depotVariables.size());
            for (std::size_t depot = 0;
                 depot < depotVariables.size();
                 ++depot) {
                result.depotLpValues[depot] =
                    depotVariables[depot].get(
                        GRB_DoubleAttr_X);
            }
            result.lpFeedbackAvailable = true;

            if (!config.persistentLpEnabled) {
                clear();
            }
        }
    };

    bool mapSelection(
        const SpProblem& problem,
        const std::vector<StandaloneRoute>& routes,
        const std::vector<StandaloneRoute>& candidate,
        std::vector<std::size_t>& mapped,
        double* const mappedObjective = nullptr) const {
        mapped.clear();
        if (candidate.empty()) {
            return false;
        }

        std::unordered_map<
            ColumnIdentity,
            std::size_t,
            ColumnIdentityHash>
            currentIndices;
        currentIndices.reserve(routes.size());
        for (std::size_t route = 0; route < routes.size(); ++route) {
            currentIndices.emplace(
                columnIdentity(routes[route]), route);
        }

        mapped.reserve(candidate.size());
        for (const StandaloneRoute& selectedRoute : candidate) {
            const auto found = currentIndices.find(
                columnIdentity(selectedRoute));
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
        if (mappedObjective != nullptr) {
            *mappedObjective = objective;
        }
        return true;
    }

    bool mapPreviousSelection(
        const SpProblem& problem,
        const std::vector<StandaloneRoute>& routes,
        std::vector<std::size_t>& mapped,
        double* const mappedObjective = nullptr) const {
        if (!previousProblem.has_value() ||
            !sameProblem(*previousProblem, problem)) {
            mapped.clear();
            return false;
        }
        return mapSelection(
            problem,
            routes,
            previousSelection,
            mapped,
            mappedObjective);
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
        const SpMipStart& preferredStart,
        const WallClock::time_point overallWallStarted,
        const SpConfig& callConfig,
        const SpSolveStage stage,
        const SpMipMode mipMode,
        const std::optional<double> mipTargetObjective) {
        Attempt attempt;
        SpResult& result = attempt.result;
        result.solveStage = stage;
        result.mipMode = mipMode;
        result.mipTargetObjective = mipTargetObjective;
        result.solverVersion = gurobiVersion();
        result.mipStartRequestedSource = preferredStart.source.empty()
            ? "external"
            : preferredStart.source;
        result.mipStartRequestedRouteCount =
            static_cast<int>(preferredStart.routes.size());
        result.mipStartRequired = preferredStart.requireMapping;

        enum class Phase {
            environment,
            build,
            optimize,
            postprocess,
        };

        Phase phase = Phase::environment;
        std::clock_t phaseCpuStarted = std::clock();
        auto recordInterruptedPhase = [&] {
            const double elapsed = elapsedCpuSeconds(phaseCpuStarted);
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

            if (elapsedWallSeconds(overallWallStarted) >=
                callConfig.timeLimitSeconds) {
                result.status = SpStatus::time_limit;
                result.statusName = "TIME_LIMIT";
                result.diagnostic =
                    "SP time budget was exhausted before acquiring Gurobi";
                result.environmentTimeSeconds +=
                    elapsedCpuSeconds(phaseCpuStarted);
                return attempt;
            }

            const double startupRemainingSeconds =
                callConfig.timeLimitSeconds -
                elapsedWallSeconds(overallWallStarted);
            const std::optional<int> startupTimeoutSeconds =
                hgasp::sp_internal::boundedServerStartupTimeout(
                    callConfig.serverTimeoutSeconds,
                    startupRemainingSeconds);
            if (!startupTimeoutSeconds.has_value()) {
                result.status = SpStatus::time_limit;
                result.statusName = "TIME_LIMIT";
                result.diagnostic =
                    "SP time budget cannot cover a positive integer "
                    "Gurobi startup timeout";
                result.environmentTimeSeconds +=
                    elapsedCpuSeconds(phaseCpuStarted);
                return attempt;
            }
            // ServerTimeout is an integer and must remain positive.  Its
            // per-call value is capped by the remaining steady-wall budget,
            // while ProcessEnvironment identity continues to use only the
            // stable configured value.
            GRBEnv& environment = environmentManager.acquire(
                callConfig, result, *startupTimeoutSeconds);
            result.environmentTimeSeconds +=
                elapsedCpuSeconds(phaseCpuStarted);
            if (elapsedWallSeconds(overallWallStarted) >=
                callConfig.timeLimitSeconds) {
                result.status = SpStatus::time_limit;
                result.statusName = "TIME_LIMIT";
                result.diagnostic =
                    "SP time budget was exhausted while acquiring Gurobi";
                return attempt;
            }

            const bool runLp =
                stage == SpSolveStage::lp_only ||
                (stage == SpSolveStage::combined &&
                 callConfig.lpDualFeedbackEnabled);
            if (runLp) {
                phase = Phase::build;
                phaseCpuStarted = std::clock();
                SpConfig lpConfig = callConfig;
                lpConfig.lpTimeLimitSeconds = std::min(
                    callConfig.lpTimeLimitSeconds,
                    std::max(
                        1.0e-6,
                        callConfig.timeLimitSeconds -
                            elapsedWallSeconds(
                                overallWallStarted)));
                persistentLp.solve(
                    environment,
                    environmentManager.generation(),
                    problem,
                    routes,
                    lpConfig,
                    result);
                // PersistentLpMaster records its own split build/optimize
                // CPU metrics. Reset the generic phase checkpoint so a later
                // exception does not double-count that work as MIP build.
                phaseCpuStarted = std::clock();
            }
            if (stage == SpSolveStage::lp_only) {
                result.nativeStatus = result.lpNativeStatus;
                result.status = mapStatus(result.lpNativeStatus);
                result.statusName = result.lpStatusName;
                if (result.lpNativeStatus == 0) {
                    result.status = SpStatus::error;
                    result.statusName = "LP_NOT_RUN";
                    result.diagnostic =
                        "LP guidance solve did not publish a native status";
                } else if (!result.lpFeedbackAvailable &&
                           result.diagnostic.empty()) {
                    result.diagnostic =
                        "LP guidance solve finished without optimal feedback";
                }
                result.solveTimeSeconds =
                    result.lpOptimizeTimeSeconds;
                return attempt;
            }

            phase = Phase::build;
            phaseCpuStarted = std::clock();
            result.mipAttempted = true;

            GRBModel model(environment);
            model.set(GRB_IntParam_OutputFlag, callConfig.outputFlag);
            model.set(
                GRB_IntParam_LogToConsole,
                callConfig.outputFlag);
            if (!callConfig.logFile.empty()) {
                model.set(
                    GRB_StringParam_LogFile,
                    callConfig.logFile);
            }
            model.set(GRB_DoubleParam_MIPGap, callConfig.mipGap);
            model.set(GRB_IntParam_Threads, callConfig.threads);
            model.set(GRB_IntParam_Seed, callConfig.seed);
            model.set(GRB_IntParam_MIPFocus, callConfig.mipFocus);
            model.set(
                GRB_DoubleParam_Heuristics,
                callConfig.heuristics);
            model.set(
                GRB_IntParam_PoolSolutions,
                callConfig.solutionPoolSize);
            model.set(
                GRB_IntParam_PoolSearchMode,
                callConfig.solutionPoolSearchMode);
            model.set(
                GRB_DoubleParam_PoolGap,
                callConfig.solutionPoolGap);
            if (callConfig.workLimit > 0.0) {
                model.set(
                    GRB_DoubleParam_WorkLimit,
                    callConfig.workLimit);
            }
            if (mipTargetObjective.has_value()) {
                model.set(
                    GRB_DoubleParam_BestObjStop,
                    *mipTargetObjective);
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
                if (!problem.fixedDepotOpen.empty()) {
                    model.addConstr(
                        depotVariables[depot] ==
                            (problem.fixedDepotOpen[depot]
                                 ? 1.0
                                 : 0.0),
                        "fixed_depot_" +
                            std::to_string(depot));
                }
                const bool requireUse = problem.requireOpenDepotUse ||
                    (!problem.fixedDepotOpen.empty() &&
                     problem.fixedDepotOpen[depot]);
                if (requireUse) {
                    model.addConstr(
                        depotRouteCount[depot] >=
                            depotVariables[depot],
                        "use_depot_" + std::to_string(depot));
                }
            }
            for (std::size_t excludedIndex = 0U;
                 excludedIndex <
                     problem.excludedDepotOpenConfigurations.size();
                 ++excludedIndex) {
                GRBLinExpr differs = 0.0;
                const auto& excluded =
                    problem.excludedDepotOpenConfigurations[excludedIndex];
                for (std::size_t depot = 0U;
                     depot < excluded.size();
                     ++depot) {
                    differs += excluded[depot]
                        ? 1.0 - depotVariables[depot]
                        : depotVariables[depot];
                }
                model.addConstr(
                    differs >= 1.0,
                    "excluded_configuration_" +
                        std::to_string(excludedIndex));
            }

            std::vector<std::size_t> startSelection;
            std::string startSource;
            double startObjective = 0.0;
            result.mipStartPreferredMapped = mapSelection(
                    problem,
                    routes,
                    preferredStart.routes,
                    startSelection,
                    &startObjective);
            if (result.mipStartPreferredMapped) {
                startSource = preferredStart.source.empty()
                    ? "external"
                    : preferredStart.source;
            } else if (preferredStart.requireMapping) {
                throw std::logic_error(
                    "required MIP start from " +
                    result.mipStartRequestedSource +
                    " cannot be mapped to the active master snapshot");
            } else if (mapPreviousSelection(
                           problem,
                           routes,
                           startSelection,
                           &startObjective)) {
                startSource = "previous_sp";
                result.mipStartFallbackUsed = true;
            }
            if (!startSelection.empty()) {
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
                result.mipStartSource = startSource;
                result.mipStartObjective = startObjective;
            }

            model.update();
            result.modelVariables =
                model.get(GRB_IntAttr_NumVars);
            result.modelConstraints =
                model.get(GRB_IntAttr_NumConstrs);
            result.modelNonzeros =
                model.get(GRB_IntAttr_NumNZs);
            result.buildTimeSeconds +=
                elapsedCpuSeconds(phaseCpuStarted);
            // Keep the active-phase checkpoint current. If setting the
            // remaining model parameters throws, the catch block records only
            // the CPU time since this checkpoint instead of adding the whole
            // already-accounted model-build phase a second time.
            phaseCpuStarted = std::clock();

            const double remaining =
                callConfig.timeLimitSeconds -
                elapsedWallSeconds(overallWallStarted);
            if (remaining <= 0.0) {
                result.status = SpStatus::time_limit;
                result.statusName = "TIME_LIMIT";
                result.diagnostic =
                    "SP time budget was exhausted while constructing the model";
                return attempt;
            }

            model.set(GRB_DoubleParam_TimeLimit, remaining);
            phase = Phase::optimize;
            phaseCpuStarted = std::clock();
            MipIncumbentTraceCallback incumbentTrace(
                phaseCpuStarted,
                result.mipStartObjective,
                callConfig.primalIdleTimeLimitSeconds,
                callConfig.improvementExtensionGateSeconds);
            model.setCallback(&incumbentTrace);
            model.optimize();
            result.optimizeTimeSeconds +=
                elapsedCpuSeconds(phaseCpuStarted);
            result.mipIncumbentTraceAvailable =
                incumbentTrace.diagnostic().empty();
            result.mipIncumbentTraceTruncated =
                incumbentTrace.truncated();
            result.mipIncumbentTraceDiagnostic =
                incumbentTrace.diagnostic();
            result.mipIncumbentTrace =
                incumbentTrace.observations();
            result.mipPrimalIdleTerminated =
                incumbentTrace.terminatedByPrimalIdle();
            result.mipPrimalIdleTimeLimitSeconds =
                callConfig.primalIdleTimeLimitSeconds;
            result.mipNoImprovementGateTerminated =
                incumbentTrace.terminatedByNoImprovementGate();
            result.mipImprovementExtensionGateSeconds =
                callConfig.improvementExtensionGateSeconds;

            phase = Phase::postprocess;
            phaseCpuStarted = std::clock();

            result.nativeStatus =
                model.get(GRB_IntAttr_Status);
            result.status = mapStatus(result.nativeStatus);
            result.statusName = statusName(result.nativeStatus);
            if (result.mipPrimalIdleTerminated) {
                result.status = SpStatus::primal_idle_limit;
                result.statusName = "PRIMAL_IDLE_LIMIT";
            } else if (result.mipNoImprovementGateTerminated) {
                result.status = SpStatus::time_limit;
                result.statusName = "NO_IMPROVEMENT_EXTENSION_GATE";
            }
            result.solverWallTimeSeconds =
                model.get(GRB_DoubleAttr_Runtime);
            result.mipSolverWallTimeSeconds =
                result.solverWallTimeSeconds;
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

                const int candidatesToRead = std::min(
                    result.solutionCount,
                    callConfig.solutionPoolSize);
                std::unordered_set<std::string> seenSelections;
                for (int candidate = 0;
                     candidate < candidatesToRead;
                     ++candidate) {
                    model.set(
                        GRB_IntParam_SolutionNumber,
                        candidate);
                    std::vector<std::size_t> selected;
                    std::ostringstream signature;
                    for (std::size_t route = 0;
                         route < numRoutes;
                         ++route) {
                        if (routeVariables[route].get(
                                GRB_DoubleAttr_Xn) > 0.5) {
                            selected.push_back(route);
                            signature << route << ',';
                        }
                    }
                    if (!seenSelections.insert(
                            signature.str()).second) {
                        continue;
                    }
                    const double poolObjective =
                        model.get(
                            GRB_DoubleAttr_PoolObjVal);
                    std::string candidateError;
                    double recomputed = 0.0;
                    if (!validateSelection(
                            problem,
                            routes,
                            selected,
                            candidateError,
                            recomputed) ||
                        std::fabs(
                            recomputed - poolObjective) >
                            1.0e-6 *
                                std::max(
                                    1.0,
                                    std::fabs(poolObjective))) {
                        if (candidate == 0) {
                            if (candidateError.empty()) {
                                candidateError =
                                    "Gurobi objective does not match "
                                    "the selected routes";
                            }
                            result.status = SpStatus::error;
                            result.statusName =
                                "INVALID_SOLUTION";
                            result.errorCode = -1;
                            result.diagnostic =
                                std::move(candidateError);
                            result.selectedRouteIndices.clear();
                            result.objective.reset();
                            result.mipGap.reset();
                            result.incumbentValidated = false;
                            break;
                        }
                        continue;
                    }
                    SpSolution solution;
                    solution.selectedRouteIndices =
                        std::move(selected);
                    solution.objective = poolObjective;
                    result.candidateSolutions.push_back(
                        std::move(solution));
                }

                if (result.status == SpStatus::error) {
                    // Primary solution validation failed above.
                } else if (result.candidateSolutions.empty()) {
                    result.status = SpStatus::error;
                    result.statusName = "INVALID_SOLUTION_POOL";
                    result.errorCode = -1;
                    result.diagnostic =
                        "Gurobi reported solutions but no validated "
                        "solution-pool member was available";
                    result.incumbentValidated = false;
                } else {
                    const SpSolution& primary =
                        result.candidateSolutions.front();
                    if (std::fabs(
                            primary.objective - objective) >
                        1.0e-6 *
                            std::max(
                                1.0,
                                std::fabs(objective))) {
                        result.status = SpStatus::error;
                        result.statusName =
                            "INVALID_SOLUTION_POOL_ORDER";
                        result.errorCode = -1;
                        result.diagnostic =
                            "solution-pool member zero is not the "
                            "reported incumbent";
                        result.incumbentValidated = false;
                    } else {
                        result.selectedRouteIndices =
                            primary.selectedRouteIndices;
                        result.objective = objective;
                        result.mipGap = mipGap;
                        result.incumbentValidated = true;
                    }
                }
            } else {
                result.diagnostic =
                    "Gurobi finished without a feasible incumbent";
            }

            result.postprocessTimeSeconds +=
                elapsedCpuSeconds(phaseCpuStarted);
            result.solveTimeSeconds =
                result.optimizeTimeSeconds;
        } catch (const GRBException& error) {
            recordInterruptedPhase();
            result.status = SpStatus::error;
            result.statusName = "GUROBI_ERROR";
            result.errorCode = error.getErrorCode();
            result.diagnostic = error.getMessage();
            attempt.retryableEnvironmentFailure =
                retryableEnvironmentError(error);
            if (attempt.retryableEnvironmentFailure) {
                persistentLp.clear();
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
    PersistentLpMaster persistentLp;
};

SPSolver::SPSolver(SpConfig config)
    : impl_(new Impl(std::move(config))) {}

SPSolver::~SPSolver() = default;
SPSolver::SPSolver(SPSolver&&) noexcept = default;
SPSolver& SPSolver::operator=(SPSolver&&) noexcept = default;

const SpConfig& SPSolver::config() const noexcept {
    return impl_->config;
}

void SPSolver::shutdownProcessEnvironment() noexcept {
    try {
        ProcessEnvironment& environmentManager =
            processEnvironment();
        std::unique_lock<std::mutex> environmentLock =
            environmentManager.lock();
        environmentManager.reset();
    } catch (...) {
        // Process teardown must remain noexcept.  A Gurobi API exception is
        // not actionable here because all models have already been released.
    }
}

SpResult SPSolver::solve(
    const SpProblem& problem,
    const std::vector<StandaloneRoute>& routes) {
    return solve(problem, routes, SpMipStart{});
}

SpResult SPSolver::solve(
    const SpProblem& problem,
    const std::vector<StandaloneRoute>& routes,
    const SpMipStart& preferredStart) {
    return solveInternal(
        problem,
        routes,
        preferredStart,
        impl_->config,
        SpSolveStage::combined,
        SpMipMode::intensification,
        std::nullopt);
}

SpResult SPSolver::solveLp(
    const SpProblem& problem,
    const std::vector<StandaloneRoute>& routes,
    const SpLpRequest& request) {
    SpConfig effectiveConfig = impl_->config;
    effectiveConfig.lpDualFeedbackEnabled = true;
    if (request.timeLimitSeconds.has_value()) {
        effectiveConfig.timeLimitSeconds =
            *request.timeLimitSeconds;
        effectiveConfig.lpTimeLimitSeconds =
            *request.timeLimitSeconds;
    } else {
        effectiveConfig.timeLimitSeconds =
            effectiveConfig.lpTimeLimitSeconds;
    }
    return solveInternal(
        problem,
        routes,
        SpMipStart{},
        effectiveConfig,
        SpSolveStage::lp_only,
        SpMipMode::intensification,
        std::nullopt);
}

SpResult SPSolver::solveMip(
    const SpProblem& problem,
    const std::vector<StandaloneRoute>& routes,
    const SpMipRequest& request) {
    return solveMip(
        problem,
        routes,
        request,
        SpMipStart{});
}

SpResult SPSolver::solveMip(
    const SpProblem& problem,
    const std::vector<StandaloneRoute>& routes,
    const SpMipRequest& request,
    const SpMipStart& preferredStart) {
    SpConfig effectiveConfig = impl_->config;
    effectiveConfig.lpDualFeedbackEnabled = false;
    if (request.mode == SpMipMode::first_improvement) {
        effectiveConfig.solutionPoolSize =
            request.solutionPoolSize.value_or(1);
        effectiveConfig.solutionPoolSearchMode =
            request.solutionPoolSearchMode.value_or(0);
    }
    if (request.timeLimitSeconds.has_value()) {
        effectiveConfig.timeLimitSeconds =
            *request.timeLimitSeconds;
    }
    if (request.workLimit.has_value()) {
        effectiveConfig.workLimit = *request.workLimit;
    }
    if (request.primalIdleTimeLimitSeconds.has_value()) {
        effectiveConfig.primalIdleTimeLimitSeconds =
            *request.primalIdleTimeLimitSeconds;
    }
    if (request.improvementExtensionGateSeconds.has_value()) {
        effectiveConfig.improvementExtensionGateSeconds =
            *request.improvementExtensionGateSeconds;
    }
    if (request.solutionPoolSize.has_value()) {
        effectiveConfig.solutionPoolSize =
            *request.solutionPoolSize;
    }
    if (request.solutionPoolGap.has_value()) {
        effectiveConfig.solutionPoolGap =
            *request.solutionPoolGap;
    }
    if (request.solutionPoolSearchMode.has_value()) {
        effectiveConfig.solutionPoolSearchMode =
            *request.solutionPoolSearchMode;
    }
    return solveInternal(
        problem,
        routes,
        preferredStart,
        effectiveConfig,
        SpSolveStage::mip_only,
        request.mode,
        request.targetObjective);
}

SpResult SPSolver::solveInternal(
    const SpProblem& problem,
    const std::vector<StandaloneRoute>& routes,
    const SpMipStart& preferredStart,
    const SpConfig& effectiveConfig,
    const SpSolveStage stage,
    const SpMipMode mipMode,
    const std::optional<double> mipTargetObjective) {
    const WallClock::time_point wallStarted = WallClock::now();
    const std::clock_t cpuStarted = std::clock();
    const std::clock_t validationCpuStarted = std::clock();
    std::string inputError =
        hgasp::sp_internal::validateInput(
            effectiveConfig, problem, routes);
    if (inputError.empty() &&
        mipTargetObjective.has_value() &&
        !std::isfinite(*mipTargetObjective)) {
        inputError =
            "MIP target objective must be finite when provided";
    }
    const double validationTime =
        elapsedCpuSeconds(validationCpuStarted);
    if (!inputError.empty()) {
        SpResult result = makeResult(
            SpStatus::invalid_input,
            "INVALID_INPUT",
            inputError);
        result.solveStage = stage;
        result.mipMode = mipMode;
        result.mipTargetObjective = mipTargetObjective;
        result.validationTimeSeconds = validationTime;
        result.totalTimeSeconds = elapsedCpuSeconds(cpuStarted);
        result.totalWallTimeSeconds =
            elapsedWallSeconds(wallStarted);
        return result;
    }

    double accumulatedEnvironmentTime = 0.0;
    double accumulatedBuildTime = 0.0;
    double accumulatedOptimizeTime = 0.0;
    double accumulatedPostprocessTime = 0.0;
    double accumulatedLpBuildTime = 0.0;
    double accumulatedLpOptimizeTime = 0.0;
    double retrySleepCpuTime = 0.0;
    double retrySleepWallTime = 0.0;
    double accumulatedSolverWallTime = 0.0;
    double accumulatedMipSolverWallTime = 0.0;
    int accumulatedEnvironmentStarts = 0;
    bool environmentWasReused = false;

    for (int attemptNumber = 1;
         attemptNumber <= effectiveConfig.environmentMaxAttempts;
         ++attemptNumber) {
        Impl::Attempt attempt =
            impl_->solveAttempt(
                problem,
                routes,
                preferredStart,
                wallStarted,
                effectiveConfig,
                stage,
                mipMode,
                mipTargetObjective);
        SpResult& result = attempt.result;

        result.validationTimeSeconds = validationTime;
        result.environmentTimeSeconds +=
            accumulatedEnvironmentTime;
        result.buildTimeSeconds += accumulatedBuildTime;
        result.optimizeTimeSeconds +=
            accumulatedOptimizeTime;
        result.postprocessTimeSeconds +=
            accumulatedPostprocessTime;
        result.lpBuildTimeSeconds +=
            accumulatedLpBuildTime;
        result.lpOptimizeTimeSeconds +=
            accumulatedLpOptimizeTime;
        result.retrySleepTimeSeconds = retrySleepCpuTime;
        result.retrySleepWallTimeSeconds = retrySleepWallTime;
        result.solverWallTimeSeconds +=
            accumulatedSolverWallTime;
        result.mipSolverWallTimeSeconds +=
            accumulatedMipSolverWallTime;
        result.environmentStartAttempts +=
            accumulatedEnvironmentStarts;
        result.environmentReused =
            result.environmentReused || environmentWasReused;
        result.environmentRetryCount = attemptNumber - 1;

        if (!attempt.retryableEnvironmentFailure ||
            attemptNumber >=
                effectiveConfig.environmentMaxAttempts) {
            if (attempt.retryableEnvironmentFailure &&
                attemptNumber >=
                    effectiveConfig.environmentMaxAttempts) {
                result.diagnostic +=
                    " (retryable environment failure; attempts exhausted)";
            }
            if (result.hasFeasibleSolution()) {
                impl_->rememberSelection(
                    problem,
                    routes,
                    result.selectedRouteIndices);
            }
            result.solveTimeSeconds =
                stage == SpSolveStage::lp_only
                    ? result.lpOptimizeTimeSeconds
                    : result.optimizeTimeSeconds;
            result.totalTimeSeconds =
                elapsedCpuSeconds(cpuStarted);
            result.totalWallTimeSeconds =
                elapsedWallSeconds(wallStarted);
            return result;
        }

        accumulatedEnvironmentTime =
            result.environmentTimeSeconds;
        accumulatedBuildTime = result.buildTimeSeconds;
        accumulatedOptimizeTime = result.optimizeTimeSeconds;
        accumulatedPostprocessTime =
            result.postprocessTimeSeconds;
        accumulatedLpBuildTime =
            result.lpBuildTimeSeconds;
        accumulatedLpOptimizeTime =
            result.lpOptimizeTimeSeconds;
        accumulatedSolverWallTime =
            result.solverWallTimeSeconds;
        accumulatedMipSolverWallTime =
            result.mipSolverWallTimeSeconds;
        accumulatedEnvironmentStarts =
            result.environmentStartAttempts;
        environmentWasReused = result.environmentReused;

        const int delayMilliseconds =
            retryDelayMilliseconds(
                effectiveConfig,
                attemptNumber);
        if (elapsedWallSeconds(wallStarted) +
                static_cast<double>(delayMilliseconds) / 1000.0 >=
            effectiveConfig.timeLimitSeconds) {
            result.diagnostic +=
                " (retry skipped because the SP time budget is exhausted)";
            result.solveTimeSeconds =
                stage == SpSolveStage::lp_only
                    ? result.lpOptimizeTimeSeconds
                    : result.optimizeTimeSeconds;
            result.totalTimeSeconds =
                elapsedCpuSeconds(cpuStarted);
            result.totalWallTimeSeconds =
                elapsedWallSeconds(wallStarted);
            return result;
        }
        const WallClock::time_point retrySleepWallStarted =
            WallClock::now();
        const std::clock_t retrySleepCpuStarted = std::clock();
        std::this_thread::sleep_for(
            std::chrono::milliseconds(delayMilliseconds));
        retrySleepCpuTime +=
            elapsedCpuSeconds(retrySleepCpuStarted);
        retrySleepWallTime +=
            elapsedWallSeconds(retrySleepWallStarted);
    }

    SpResult result = makeResult(
        SpStatus::error,
        "ERROR",
        "internal error: SP retry loop exited unexpectedly");
    result.solveStage = stage;
    result.mipMode = mipMode;
    result.mipTargetObjective = mipTargetObjective;
    result.validationTimeSeconds = validationTime;
    result.totalTimeSeconds = elapsedCpuSeconds(cpuStarted);
    result.totalWallTimeSeconds =
        elapsedWallSeconds(wallStarted);
    return result;
}
