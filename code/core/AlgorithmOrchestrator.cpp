/******************************************************************************
 * File: AlgorithmOrchestrator.cpp
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description: 
 *   High-level controller managing the algorithm lifecycle and orchestrating modules.
 ******************************************************************************/

#include "AlgorithmOrchestrator.h"
#include "FinalMasterScopePolicy.h"
#include "SpContributionAttribution.h"
#include "SpExecutionPlan.h"
#include "ComponentProfiler.h"
#include "DiverseSpSolutionSelector.h"
#include "NumericPolicy.h"
#include "RunLogger.h"
#include "SpMipGate.h"
#include "SearchComponents.h"
#include "basic.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_set>

namespace {

using SpWallClock = std::chrono::steady_clock;

double elapsedSpWallSeconds(
        const SpWallClock::time_point started) noexcept {
    return std::chrono::duration<double>(
        SpWallClock::now() - started).count();
}

class ScopeExit final {
public:
    explicit ScopeExit(std::function<void()> action)
        : action_(std::move(action)) {}
    ~ScopeExit() noexcept {
        if (!active_) {
            return;
        }
        try {
            action_();
        } catch (...) {
            // Accounting cleanup must not replace the original exception.
        }
    }
    ScopeExit(const ScopeExit&) = delete;
    ScopeExit& operator=(const ScopeExit&) = delete;
    void run() {
        if (!active_) {
            return;
        }
        action_();
        active_ = false;
    }

private:
    std::function<void()> action_;
    bool active_ = true;
};

struct StagedMipAccounting {
    void add(const SpResult& stage) {
        ++stages;
        solveTimeSeconds += stage.solveTimeSeconds;
        validationTimeSeconds += stage.validationTimeSeconds;
        environmentTimeSeconds += stage.environmentTimeSeconds;
        retrySleepTimeSeconds += stage.retrySleepTimeSeconds;
        buildTimeSeconds += stage.buildTimeSeconds;
        optimizeTimeSeconds += stage.optimizeTimeSeconds;
        postprocessTimeSeconds += stage.postprocessTimeSeconds;
        totalTimeSeconds += stage.totalTimeSeconds;
        solverWallTimeSeconds += stage.solverWallTimeSeconds;
        mipSolverWallTimeSeconds +=
            stage.mipSolverWallTimeSeconds;
        retrySleepWallTimeSeconds +=
            stage.retrySleepWallTimeSeconds;
        totalWallTimeSeconds += stage.totalWallTimeSeconds;
        environmentStartAttempts +=
            stage.environmentStartAttempts;
        environmentRetryCount +=
            stage.environmentRetryCount;
        environmentReused =
            environmentReused || stage.environmentReused;
        if (stage.work.has_value()) {
            cumulativeWork += *stage.work;
        } else {
            completeWork = false;
        }
        if (stage.nodeCount.has_value()) {
            cumulativeNodes += *stage.nodeCount;
        } else {
            completeNodes = false;
        }
    }

    void apply(SpResult& result) const {
        if (stages == 0U) {
            return;
        }
        result.solveTimeSeconds = solveTimeSeconds;
        result.validationTimeSeconds = validationTimeSeconds;
        result.environmentTimeSeconds = environmentTimeSeconds;
        result.retrySleepTimeSeconds = retrySleepTimeSeconds;
        result.buildTimeSeconds = buildTimeSeconds;
        result.optimizeTimeSeconds = optimizeTimeSeconds;
        result.postprocessTimeSeconds = postprocessTimeSeconds;
        result.totalTimeSeconds = totalTimeSeconds;
        result.solverWallTimeSeconds = solverWallTimeSeconds;
        result.mipSolverWallTimeSeconds =
            mipSolverWallTimeSeconds;
        result.retrySleepWallTimeSeconds =
            retrySleepWallTimeSeconds;
        result.totalWallTimeSeconds = totalWallTimeSeconds;
        result.environmentStartAttempts =
            environmentStartAttempts;
        result.environmentRetryCount =
            environmentRetryCount;
        result.environmentReused = environmentReused;
        result.work = completeWork
            ? std::optional<double>(cumulativeWork)
            : std::nullopt;
        result.nodeCount = completeNodes
            ? std::optional<double>(cumulativeNodes)
            : std::nullopt;
    }

    std::size_t stages = 0U;
    bool completeWork = true;
    bool completeNodes = true;
    double cumulativeWork = 0.0;
    double cumulativeNodes = 0.0;
    double solveTimeSeconds = 0.0;
    double validationTimeSeconds = 0.0;
    double environmentTimeSeconds = 0.0;
    double retrySleepTimeSeconds = 0.0;
    double buildTimeSeconds = 0.0;
    double optimizeTimeSeconds = 0.0;
    double postprocessTimeSeconds = 0.0;
    double totalTimeSeconds = 0.0;
    double solverWallTimeSeconds = 0.0;
    double mipSolverWallTimeSeconds = 0.0;
    double retrySleepWallTimeSeconds = 0.0;
    double totalWallTimeSeconds = 0.0;
    int environmentStartAttempts = 0;
    int environmentRetryCount = 0;
    bool environmentReused = false;
};

void attachP2LpObservation(
        SpResult& mipResult,
        const SpResult& lpResult,
        const bool includeRouteIndexedFeedback = true) {
    mipResult.solveStage = SpSolveStage::combined;
    mipResult.lpAttempted = lpResult.lpAttempted;
    mipResult.lpNativeStatus = lpResult.lpNativeStatus;
    mipResult.lpStatusName = lpResult.lpStatusName;
    mipResult.lpFeedbackAvailable =
        includeRouteIndexedFeedback && lpResult.lpFeedbackAvailable;
    mipResult.lpObjective = lpResult.lpObjective;
    mipResult.customerDuals = lpResult.customerDuals;
    mipResult.depotCapacityDuals =
        lpResult.depotCapacityDuals;
    if (includeRouteIndexedFeedback) {
        mipResult.routeReducedCosts =
            lpResult.routeReducedCosts;
        mipResult.routeLpValues = lpResult.routeLpValues;
    }
    mipResult.depotLpValues = lpResult.depotLpValues;
    mipResult.lpBuildTimeSeconds =
        lpResult.lpBuildTimeSeconds;
    mipResult.lpOptimizeTimeSeconds =
        lpResult.lpOptimizeTimeSeconds;
    mipResult.solveTimeSeconds +=
        lpResult.solveTimeSeconds;
    mipResult.validationTimeSeconds +=
        lpResult.validationTimeSeconds;
    mipResult.environmentTimeSeconds +=
        lpResult.environmentTimeSeconds;
    mipResult.retrySleepTimeSeconds +=
        lpResult.retrySleepTimeSeconds;
    mipResult.buildTimeSeconds +=
        lpResult.buildTimeSeconds;
    mipResult.optimizeTimeSeconds +=
        lpResult.optimizeTimeSeconds;
    mipResult.postprocessTimeSeconds +=
        lpResult.postprocessTimeSeconds;
    mipResult.totalTimeSeconds +=
        lpResult.totalTimeSeconds;
    mipResult.solverWallTimeSeconds +=
        lpResult.solverWallTimeSeconds;
    mipResult.retrySleepWallTimeSeconds +=
        lpResult.retrySleepWallTimeSeconds;
    mipResult.totalWallTimeSeconds +=
        lpResult.totalWallTimeSeconds;
    mipResult.environmentStartAttempts +=
        lpResult.environmentStartAttempts;
    mipResult.environmentRetryCount +=
        lpResult.environmentRetryCount;
    mipResult.environmentReused =
        mipResult.environmentReused ||
        lpResult.environmentReused;
    mipResult.persistentLpReused =
        lpResult.persistentLpReused;
    mipResult.persistentLpAddedColumns =
        lpResult.persistentLpAddedColumns;
    mipResult.persistentLpReactivatedColumns =
        lpResult.persistentLpReactivatedColumns;
    mipResult.persistentLpDisabledColumns =
        lpResult.persistentLpDisabledColumns;
    mipResult.persistentLpStoredColumns =
        lpResult.persistentLpStoredColumns;
    mipResult.persistentLpActiveColumns =
        lpResult.persistentLpActiveColumns;
    mipResult.persistentLpCompacted =
        lpResult.persistentLpCompacted;
    mipResult.persistentLpColumnsBeforeCompaction =
        lpResult.persistentLpColumnsBeforeCompaction;
}

SpConfig makeSpConfig(const SolverContext& data) {
    SpConfig config;
    config.timeLimitSeconds = data.spGlobalOuterTimeLimit;
    config.workLimit = data.spWorkLimit;
    config.mipGap = data.spMIPGap;
    config.threads = data.spThreads;
    config.seed = data.spSeed;
    config.outputFlag = data.spOutputFlag;
    config.mipFocus = data.spMIPFocus;
    config.heuristics = data.spHeuristics;
    config.wlsTokenDurationMinutes =
        data.spWLSTokenDurationMinutes;
    config.wlsTokenRefresh = data.spWLSTokenRefresh;
    config.clientLogLevel = data.spClientLogLevel;
    config.serverTimeoutSeconds = data.spServerTimeoutSeconds;
    config.environmentMaxAttempts =
        data.spEnvironmentMaxAttempts;
    config.environmentRetryBaseMilliseconds =
        data.spEnvironmentRetryBaseMilliseconds;
    config.lpDualFeedbackEnabled =
        data.spLpDualFeedbackEnabled;
    config.lpTimeLimitSeconds = data.spGlobalLpTimeLimit;
    config.persistentLpEnabled =
        data.spPersistentLpEnabled;
    config.solutionPoolSize = data.spGlobalSolutionPoolSize;
    config.solutionPoolGap = data.spGlobalSolutionPoolGap;
    config.solutionPoolSearchMode =
        data.spGlobalSolutionPoolSearchMode;

    // main() normally supplies a unique per-run path. Tests and embedding
    // callers may leave it empty to disable the native Gurobi log.
    if (!data.spLogFile.empty()) {
        config.logFile = data.spLogFile;
    }
    return config;
}

std::string configurationLabel(
        const hgasp::DepotConfigurationKey& key) {
    std::string label;
    for (std::size_t index = 0;
         index < key.openDepots.size();
         ++index) {
        if (index > 0U) {
            label += "-";
        }
        label += std::to_string(key.openDepots[index]);
    }
    return label.empty() ? "none" : label;
}

std::uint64_t configurationScopeKey(
        const hgasp::DepotConfigurationKey& key) noexcept {
    // FNV-1a over the canonical, sorted depot vector. Zero is reserved for
    // legacy/unscoped controller state.
    std::uint64_t hash = 1469598103934665603ULL;
    for (const int depot : key.openDepots) {
        const std::uint64_t value =
            static_cast<std::uint64_t>(
                static_cast<std::uint32_t>(depot));
        for (int byte = 0; byte < 4; ++byte) {
            hash ^=
                (value >> (8 * byte)) & 0xffULL;
            hash *= 1099511628211ULL;
        }
    }
    constexpr std::uint64_t islandNamespace = 1ULL << 63U;
    hash &= ~islandNamespace;
    return hash == 0U ? 1U : hash;
}


}  // namespace

AlgorithmOrchestrator::AlgorithmOrchestrator(SolverContext * data)
    : data(data),
      attemptScheduler(
          SearchAttemptScheduler::productionNonproductiveAttemptLimit(
              data->iterLimit)) {
    this->data = data;
    this->routeGenerator = std::unique_ptr<RouteGenerator>(new RouteGenerator(data));
    this->routePool = std::unique_ptr<RoutePool>(new RoutePool(data));
    this->spSolver =
        std::unique_ptr<SPSolver>(new SPSolver(makeSpConfig(*data)));
    SpConfig localConfig = makeSpConfig(*data);
    localConfig.timeLimitSeconds = data->spLocalOuterTimeLimit;
    localConfig.lpTimeLimitSeconds = data->spLocalLpTimeLimit;
    localConfig.solutionPoolSize = data->spLocalSolutionPoolSize;
    localConfig.solutionPoolGap = data->spLocalSolutionPoolGap;
    // Local RMP calls favor opportunistic alternatives. Systematic top-K
    // enumeration is reserved for the less frequent global intensification.
    localConfig.solutionPoolSearchMode =
        data->spLocalSolutionPoolSearchMode;
    this->localSpSolver =
        std::unique_ptr<SPSolver>(
            new SPSolver(std::move(localConfig)));
    AdaptiveSpPolicy adaptivePolicy;
    adaptivePolicy.minimumInterval =
        data->spAdaptiveMinimumInterval;
    adaptivePolicy.crossScopeLocalMinimumInterval =
        data->spCrossScopeLocalMinimumInterval;
    adaptivePolicy.globalMinimumInterval =
        data->spGlobalMinimumInterval;
    adaptivePolicy.localNewColumnThreshold =
        static_cast<std::uint64_t>(
            data->spAdaptiveLocalNewColumns);
    adaptivePolicy.localStagnationThreshold =
        data->spAdaptiveLocalStagnation;
    adaptivePolicy.globalStagnationThreshold =
        data->spAdaptiveGlobalStagnation;
    adaptivePolicy.localZeroUtilityPatience =
        data->spLocalZeroUtilityPatience;
    adaptivePolicy.globalZeroUtilityPatience =
        data->spGlobalZeroUtilityPatience;
    adaptivePolicy.utilityEmaWeight =
        data->spAdaptiveUtilityEmaWeight;
    adaptivePolicy.highUtilityThreshold =
        data->spAdaptiveHighUtilityThreshold;
    adaptiveSpController.configure(adaptivePolicy);

    spBudgetLedger.configure(data->spCpuShareTarget);
    spBudgetLedger.configurePhaseTargets(
        data->spLpProbeCpuShareTarget,
        data->spLocalMipCpuShareTarget,
        data->spGlobalMipCpuShareTarget);
    routeGenerator->setSpBudgetLedger(&spBudgetLedger);

    hgasp::PopulationLocalSpPolicy localPopulationPolicy;
    localPopulationPolicy.stagnationAttempts =
        static_cast<std::uint64_t>(
            data->populationLocalSpStagnationAttempts);
    localPopulationPolicy.minimumAgeAttempts =
        static_cast<std::uint64_t>(
            data->populationLocalSpMinimumAgeAttempts);
    localPopulationPolicy.minimumNewCustomerSets =
        static_cast<std::uint64_t>(
            data->populationLocalSpMinimumNewCustomerSets);
    localPopulationPolicy.minimumReplacementGainRatio =
        data->populationLocalSpMinimumReplacementGainRatio;
    localPopulationPolicy.lpTimeLimitSeconds =
        data->populationLocalSpLpTimeLimitSeconds;
    localPopulationPolicy.mipTimeLimitSeconds =
        data->populationLocalSpMipTimeLimitSeconds;
    localPopulationPolicy.maximumCallsPerLifetime =
        static_cast<std::size_t>(
            data->populationLocalSpMaximumCalls);
    populationLocalSpController.configure(localPopulationPolicy);
    localSpFamilyBudget.configure({
        static_cast<std::uint64_t>(
            data->populationLocalSpFamilyMinimumInterval),
        data->populationLocalSpFamilyCpuShareLimit,
    });

    if (data->staticHgampPopulationsEnabled()) {
        hgasp::GlobalSpPhaseConfig phaseConfig;
        phaseConfig.configuredHorizonAttempts =
            static_cast<std::uint64_t>(
                data->populationPhaseHorizonAttempts);
        phaseConfig.preReductionGlobal1Fraction =
            data->preReductionGlobal1Fraction;
        phaseConfig.populationReduction1Fraction =
            data->populationReduction1Fraction;
        phaseConfig.preReductionGlobal2Fraction =
            data->preReductionGlobal2Fraction;
        phaseConfig.populationReduction2Fraction =
            data->populationReduction2Fraction;
        phaseConfig.minimumAbsorptionFraction =
            data->populationAbsorptionFraction;
        phaseConfig.preReductionGlobalEnabled =
            data->preReductionGlobalSpEnabled;
        phaseConfig.lpTimeLimitSeconds =
            data->preReductionGlobalSpLpTimeLimitSeconds;
        phaseConfig.mipTimeLimitSeconds =
            data->preReductionGlobalSpMipTimeLimitSeconds;
        phaseConfig.maximumInjections =
            static_cast<std::size_t>(
                data->preReductionGlobalSpInjectionCount);
        const bool roleAwareProfile =
            hgasp::usesRoleAwareCheckpointSearch(
                data->parameters.profile);
        if (roleAwareProfile) {
            const double budgetScale =
                hgasp::v33GlobalSpBudgetScale(
                    static_cast<std::size_t>(data->numV));
            phaseConfig.earlyGlobalEnabled = true;
            phaseConfig.earlyGlobalFraction = data->spEarlyGlobalFraction;
            phaseConfig.earlyGlobalBaseMipSeconds =
                data->preReductionGlobalSpMipTimeLimitSeconds * budgetScale;
            phaseConfig.earlyGlobalMaximumMipSeconds =
                6.0 * data->preReductionGlobalSpMipTimeLimitSeconds *
                budgetScale;
            phaseConfig.firstGlobalBaseMipSeconds =
                data->preReductionGlobalSpMipTimeLimitSeconds * budgetScale;
            phaseConfig.firstGlobalMaximumMipSeconds =
                data->preReductionGlobalSpMipTimeLimitSeconds * budgetScale;
            phaseConfig.secondGlobalBaseMipSeconds =
                data->preReductionGlobalSpMipTimeLimitSeconds * budgetScale;
            phaseConfig.secondGlobalMaximumMipSeconds =
                data->preReductionGlobalSpMipTimeLimitSeconds * budgetScale;
            phaseConfig.scheduledFinalGlobalEnabled = true;
            phaseConfig.finalGlobalFraction =
                data->spFinalPhaseStartFraction;
            phaseConfig.finalGlobalMaximumMipSeconds =
                data->hasHardTimeWindows()
                    ? data->spTwFinalMipTimeLimit
                    : data->spFinalGlobalMipTimeLimit * budgetScale;
            phaseConfig.finalGlobalBaseMipSeconds = std::min(
                phaseConfig.finalGlobalMaximumMipSeconds,
                60.0 * budgetScale);
            phaseConfig.finalGlobalMaximumInjections =
                static_cast<std::size_t>(data->spFinalGlobalInjectionCount);
        }
        globalSpPhasePolicy =
            std::make_unique<hgasp::GlobalSpPhasePolicy>(
                static_cast<std::uint64_t>(
                    std::max(1, data->iterLimit)),
                phaseConfig);
    }
}

AlgorithmOrchestrator::~AlgorithmOrchestrator() {
    releaseOwnedComponents();
}

void AlgorithmOrchestrator::releaseOwnedComponents() noexcept {
    if (shutdownCompleted) {
        return;
    }
    // RouteGenerator consumes the route pool and budget ledger;
    // destroy it before those providers.  The explicit resets also avoid the
    // reverse declaration-order teardown that previously destroyed solvers
    // before their consumer.
    routeGenerator.reset();
    routePool.reset();
    localSpSolver.reset();
    spSolver.reset();
    SPSolver::shutdownProcessEnvironment();
    globalSpPhasePolicy.reset();
    shutdownCompleted = true;
}

void AlgorithmOrchestrator::shutdown() {
    if (shutdownCompleted) {
        return;
    }
    hgasp::RunLogger::event(
        hgasp::LogLevel::info,
        "lifecycle",
        "orchestrator.shutdown_started");
    const auto phase = [](const char* component) {
        hgasp::RunLogger::event(
            hgasp::LogLevel::info,
            "lifecycle",
            "orchestrator.shutdown_phase",
            {},
            {{"component", component}});
    };
    phase("route_generator");
    routeGenerator.reset();
    phase("route_pool");
    routePool.reset();
    phase("local_sp_solver");
    localSpSolver.reset();
    phase("global_sp_solver");
    spSolver.reset();
    phase("gurobi_process_environment");
    SPSolver::shutdownProcessEnvironment();
    phase("global_sp_phase_policy");
    globalSpPhasePolicy.reset();
    shutdownCompleted = true;
    hgasp::RunLogger::event(
        hgasp::LogLevel::info,
        "lifecycle",
        "orchestrator.shutdown_completed");
}

void AlgorithmOrchestrator::emitTerminalDiagnostics() {
    if (!searchAccountingEmitted) {
        hgasp::RunLogger::event(
            hgasp::LogLevel::info,
            "search",
            "search_effort_accounting",
            {},
            {
                {"completed_attempts", static_cast<unsigned long long>(
                    attemptScheduler.completedAttemptCount())},
                {"productive_feasible_offspring",
                    static_cast<unsigned long long>(
                        attemptScheduler
                            .productiveFeasibleOffspringCount())},
                {"consecutive_nonproductive_attempts",
                    static_cast<unsigned long long>(
                        attemptScheduler
                            .consecutiveNonproductiveAttemptCount())},
                {"completed_iterations", data->iteration},
                {"iteration_semantics",
                    "hgamp_productive_feasible_offspring"},
            });
        searchAccountingEmitted = true;
    }
    if (routePool == nullptr) return;
#if defined(HGASP_ENABLE_HOT_PATH_COUNTERS)
    const Individual* const incumbent =
        routeGenerator == nullptr
            ? nullptr
            : routeGenerator->globalIncumbent();
    if (incumbent == nullptr || !incumbent->isFeasible) return;
    const RoutePool::FinalMasterDiagnostic diagnostic =
        routePool->summarizeFinalMasterAgainstIncumbent(*incumbent);
    hgasp::RunLogger::event(
        hgasp::LogLevel::info,
        "route_pool",
        "final_master_diagnostic",
        {},
        {
            {"iteration", data->iteration},
            {"active_column_count",
                static_cast<unsigned long long>(
                    diagnostic.activeColumnCount)},
            {"archived_column_count",
                static_cast<unsigned long long>(
                    diagnostic.archivedColumnCount)},
            {"same_key_active_representative_violations",
                static_cast<unsigned long long>(
                    diagnostic
                        .sameKeyActiveRepresentativeViolations)},
            {"same_key_archive_representative_violations",
                static_cast<unsigned long long>(
                    diagnostic
                        .sameKeyArchiveRepresentativeViolations)},
            {"dominance_replacement_count",
                static_cast<unsigned long long>(
                    diagnostic.dominanceReplacementCount)},
            {"dominance_replacement_total_gain",
                diagnostic.dominanceReplacementTotalGain},
            {"final_incumbent_routes_without_matching_master_key",
                static_cast<unsigned long long>(
                    diagnostic
                        .finalIncumbentRoutesWithoutMatchingMasterKey)},
            {"final_incumbent_routes_with_cheaper_matching_active_key",
                static_cast<unsigned long long>(
                    diagnostic
                        .finalIncumbentRoutesWithCheaperMatchingActiveKey)},
            {"final_incumbent_routes_with_cheaper_matching_archive_key",
                static_cast<unsigned long long>(
                    diagnostic
                        .finalIncumbentRoutesWithCheaperMatchingArchiveKey)},
        });
#endif
}

bool AlgorithmOrchestrator::shouldTriggerSp(
    const int iteration,
    const int triggerFrequency,
    const int triggerOffset) {
    return SearchAttemptScheduler::isSpScheduled(
        iteration,
        triggerFrequency,
        triggerOffset);
}

std::vector<StandaloneRoute>
AlgorithmOrchestrator::filterRoutesForConfiguration(
        const std::vector<StandaloneRoute>& routes,
        const std::vector<bool>& openDepots,
        const std::vector<StandaloneRoute>& requiredRoutes) {
    std::vector<StandaloneRoute> filtered;
    std::unordered_set<hgasp::RouteKey, hgasp::RouteKeyHash> keys;
    for (const StandaloneRoute& route : routes) {
        if (route.locId >= 0 &&
            static_cast<std::size_t>(route.locId) <
                openDepots.size() &&
            openDepots[route.locId]) {
            filtered.push_back(route);
            keys.insert(hgasp::makeRouteKey(route));
        }
    }
    for (const StandaloneRoute& route : requiredRoutes) {
        if (route.locId < 0 ||
            static_cast<std::size_t>(route.locId) >=
                openDepots.size() ||
            !openDepots[route.locId]) {
            throw std::invalid_argument(
                "required local-SP route uses a closed depot");
        }
        if (keys.insert(hgasp::makeRouteKey(route)).second) {
            filtered.push_back(route);
        }
    }
    return filtered;
}

bool AlgorithmOrchestrator::shouldImportDualFeedback(
        const SpProblem& problem) noexcept {
    // Local fixed-configuration duals describe a different restricted
    // master and must not erase the latest global route/depot guidance.
    return problem.fixedDepotOpen.empty();
}

int AlgorithmOrchestrator::chooseLocalPopulation(
        const int activePopulation,
        const std::vector<bool>& nonEmptyPopulations,
        const int fallbackPopulation) noexcept {
    if (activePopulation >= 0 &&
        static_cast<std::size_t>(activePopulation) <
            nonEmptyPopulations.size() &&
        nonEmptyPopulations[
            static_cast<std::size_t>(activePopulation)]) {
        return activePopulation;
    }
    return fallbackPopulation;
}

bool AlgorithmOrchestrator::
shouldConsumeLocalMasterVersionOnPreparationSkip(
        const std::string& skipReason) noexcept {
    // Only a valid, configuration-scoped view that lacks a complete route
    // cover is safe to consume. Stale/invalid targets describe controller
    // state that should be rebuilt, and budget guards have their own retry
    // semantics.
    return skipReason ==
        "configuration_pool_incomplete";
}

int AlgorithmOrchestrator::bestPopulationIndex() const noexcept {
    int best = -1;
    double bestFitness = Max;
    for (std::size_t index = 0;
         index < routeGenerator->popman->popula.size();
         ++index) {
        const auto& population =
            routeGenerator->popman->popula[index];
        if (!population.empty() &&
            population.front()->fit < bestFitness) {
            bestFitness = population.front()->fit;
            best = static_cast<int>(index);
        }
    }
    return best;
}

SpProblem AlgorithmOrchestrator::buildSpProblem() const {
    SpProblem problem;
    problem.name = data->pathInstance;
    problem.customerDemands.assign(data->demCus, data->demCus + data->numV);
    problem.vehicleCapacity = data->vehCap;
    problem.vehicleFixedCost = data->usingCostVeh;
    problem.depots.reserve(data->numL);

    for (int depot = 0; depot < data->numL; ++depot) {
        SpDepot spDepot;
        spDepot.openingCost = data->openCostLoc[data->numV + depot];
        spDepot.capacity = data->invLoc[data->numV + depot];
        if (data->spEnforceRepresentationRouteLimit) {
            spDepot.representationRouteLimit =
                Individual::representationRouteLimit(*data, depot);
        }
        problem.depots.push_back(spDepot);
    }
    return problem;
}

void AlgorithmOrchestrator::refreshRouteProtections(
        const bool includePopulationElites) {
    const Individual* incumbent =
        routeGenerator->globalIncumbent();
    if (incumbent != nullptr) {
        const std::vector<StandaloneRoute> globalRoutes =
            routePool->exportRoutes(incumbent);
        // This admission happens before the single prune below, after the
        // keys have been selected for incumbent protection.
        routePool->setProtectedRoutes(
            RouteProtection::GlobalIncumbent,
            globalRoutes);
        lastProtectedGlobalFitness = incumbent->fit;
        if (!backboneInitialized) {
            routePool->setProtectedRoutes(
                RouteProtection::Backbone,
                globalRoutes);
            backboneInitialized = true;
        }
    }

    if (includePopulationElites) {
        std::vector<StandaloneRoute> eliteRoutes;
        for (const auto& population :
             routeGenerator->popman->popula) {
            if (population.empty()) {
                continue;
            }
            const std::vector<StandaloneRoute> routes =
                routePool->exportRoutes(population.front());
            eliteRoutes.insert(
                eliteRoutes.end(),
                routes.begin(),
                routes.end());
        }
        routePool->setProtectedRoutes(
            RouteProtection::PopulationElite,
            eliteRoutes);
    }
    routePool->pruneIfNeeded();
}

AlgorithmOrchestrator::SpExecutionSummary
AlgorithmOrchestrator::accountSpPreparationSkip(
        SpExecutionSummary summary,
        hgasp::SpBudgetCallToken& budgetToken,
        const hgasp::SpScope budgetScope,
        const std::string& scope,
        const std::string& reason,
        const double outerCpuStarted,
        const bool schedulingBypass) {
    summary.skipReason = reason;
    summary.cpuSeconds = std::max(
        0.0,
        data->elapsedCpuSeconds() - outerCpuStarted);
    const std::string callId =
        scope + "@" + std::to_string(data->iteration);
    hgasp::RunLogger::event(
        hgasp::LogLevel::info,
        "sp",
        "solve_skipped_preflight",
        {},
        {
            {"iteration", data->iteration},
            {"scope", scope},
            {"call_id", callId},
            {"reason", reason},
            {"outer_cpu_seconds", summary.cpuSeconds},
        });
    if (spBudgetLedger.settle(
            budgetToken,
            budgetScope,
            summary.cpuSeconds,
            false)) {
        hgasp::RunLogger::event(
            hgasp::LogLevel::info,
            "sp",
            "budget_accounted",
            {},
            {
                {"iteration", data->iteration},
                {"scope", scope},
                {"call_id", callId},
                {"outer_cpu_seconds",
                    summary.cpuSeconds},
                {"execution_mode", "p2"},
                {"scheduling_bypass", schedulingBypass},
                {"mandatory", false},
                {"mandatory_exemption", false},
                {"exemption_reason", "none"},
                {"observed_sp_cpu_seconds",
                    spBudgetLedger.totalCpuSeconds()},
                {"observed_sp_cpu_share",
                    spBudgetLedger.observedShare(
                        data->elapsedCpuSeconds())},
                {"guardable_sp_cpu_seconds",
                    spBudgetLedger.guardableCpuSeconds()},
                {"guardable_sp_cpu_share",
                    spBudgetLedger.guardableShare(
                        data->elapsedCpuSeconds())},
                {"mandatory_exempt_cpu_seconds",
                    spBudgetLedger.mandatoryExemptCpuSeconds()},
                {"family_cpu_seconds",
                    spBudgetLedger.totalCpuSeconds()},
                {"family_cpu_share",
                    spBudgetLedger.observedShare(
                        data->elapsedCpuSeconds())},
                {"target_share",
                    data->spCpuShareTarget},
                {"preparation_only", true},
            });
    }
    return summary;
}

void AlgorithmOrchestrator::settleSpOuterFallback(
        hgasp::SpBudgetCallToken& budgetToken,
        const hgasp::SpScope budgetScope,
        const std::string& scope,
        const double outerCpuStarted,
        const bool schedulingBypass,
        const bool hardMandatory) noexcept {
    try {
        const double cpuSeconds = std::max(
            0.0,
            data->elapsedCpuSeconds() - outerCpuStarted);
        if (!spBudgetLedger.settle(
                budgetToken,
                budgetScope,
                cpuSeconds,
                hardMandatory)) {
            return;
        }
        hgasp::RunLogger::event(
            hgasp::LogLevel::info,
            "sp",
            "budget_accounted",
            {},
            {
                {"iteration", data->iteration},
                {"scope", scope},
                {"call_id",
                    scope + "@" +
                        std::to_string(data->iteration)},
                {"outer_cpu_seconds", cpuSeconds},
                {"execution_mode", "p2"},
                {"scheduling_bypass", schedulingBypass},
                {"mandatory", hardMandatory},
                {"mandatory_exemption", hardMandatory},
                {"exemption_reason",
                    hardMandatory
                        ? "mandatory_final_global"
                        : "none"},
                {"observed_sp_cpu_seconds",
                    spBudgetLedger.totalCpuSeconds()},
                {"observed_sp_cpu_share",
                    spBudgetLedger.observedShare(
                        data->elapsedCpuSeconds())},
                {"guardable_sp_cpu_seconds",
                    spBudgetLedger.guardableCpuSeconds()},
                {"guardable_sp_cpu_share",
                    spBudgetLedger.guardableShare(
                        data->elapsedCpuSeconds())},
                {"mandatory_exempt_cpu_seconds",
                    spBudgetLedger.mandatoryExemptCpuSeconds()},
                {"target_share", data->spCpuShareTarget},
                {"fallback_exit", true},
            });
    } catch (...) {
        // Accounting cleanup must not replace an active search exception.
    }
}

AlgorithmOrchestrator::SpExecutionSummary
AlgorithmOrchestrator::executePendingSp(
        const bool hardMandatory) {
    return executePendingSp(
        hardMandatory, SpExecutionOptions{});
}

AlgorithmOrchestrator::SpExecutionSummary
AlgorithmOrchestrator::executePendingSp(
        const bool hardMandatory,
        const SpExecutionOptions& options) {
    SpExecutionOptions effectiveOptions = options;
    const bool expectsLp = !options.skipLp;
    const hgasp::SpBudgetPhase initialPhase =
        expectsLp
            ? hgasp::SpBudgetPhase::LpProbe
            : (hardMandatory
                   ? hgasp::SpBudgetPhase::FinalGlobal
                   : hgasp::SpBudgetPhase::GlobalMip);
    const double initialLimit =
        options.forceLp && options.lpTimeLimitSeconds.has_value()
            ? *options.lpTimeLimitSeconds
        : expectsLp
            ? (hardMandatory
                   ? data->spFinalGlobalLpTimeLimit
                   : data->spGlobalLpTimeLimit)
            : (options.mipTimeLimitSeconds.has_value()
                   ? *options.mipTimeLimitSeconds
                   : hardMandatory
                   ? data->spFinalGlobalMipTimeLimit
                   : std::min(
                         data->spGlobalFirstImprovementEnabled
                             ? data->spGlobalFirstImprovementTimeLimit
                             : data->spGlobalOuterTimeLimit,
                         data->spOnlineGlobalMipTimeLimit));
    const double expectedInitialCpu =
        hgasp::conservativeProcessCpuProjection(
            initialLimit,
            data->spThreads,
            spBudgetLedger.lastPhaseCpuSeconds(initialPhase));
    const double reserveShare =
        initialPhase == hgasp::SpBudgetPhase::GlobalMip
            ? std::max(
                  data->spGlobalCpuReserveTarget,
                  data->spMipRescueReserveTarget)
            : 0.0;
    const bool budgetExempt =
        hardMandatory || options.reservedPhaseBudget;
    const bool familyAllowed = spBudgetLedger.canSpend(
        hgasp::SpScope::Global,
        expectedInitialCpu,
        data->elapsedCpuSeconds(),
        budgetExempt,
        reserveShare);
    const bool phaseGuardEnabled = true;
    const bool phaseAllowed =
        !phaseGuardEnabled ||
        spBudgetLedger.canSpendPhase(
            initialPhase,
            expectedInitialCpu,
            data->elapsedCpuSeconds(),
            budgetExempt,
            reserveShare);
    const bool budgetAllowed = phaseGuardEnabled
        ? phaseAllowed
        : familyAllowed;
    hgasp::RunLogger::event(
        hgasp::LogLevel::info,
        "sp",
        "budget_guard_decision",
        {},
        {
            {"iteration", data->iteration},
            {"scope", "global"},
            {"execution_mode", "p2"},
            {"scheduling_bypass", options.forceP2Execution},
            {"mandatory", hardMandatory},
            {"mandatory_exemption", hardMandatory},
            {"reserved_phase_budget", options.reservedPhaseBudget},
            {"exemption_reason",
                hardMandatory ? "mandatory_final_global" : "none"},
            {"observed_sp_cpu_seconds",
                spBudgetLedger.totalCpuSeconds()},
            {"observed_sp_cpu_share",
                spBudgetLedger.observedShare(
                    data->elapsedCpuSeconds())},
            {"guardable_sp_cpu_seconds",
                spBudgetLedger.guardableCpuSeconds()},
            {"guardable_sp_cpu_share",
                spBudgetLedger.guardableShare(
                    data->elapsedCpuSeconds())},
            {"mandatory_exempt_cpu_seconds",
                spBudgetLedger.mandatoryExemptCpuSeconds()},
            {"target_share", data->spCpuShareTarget},
            {"forecast_cpu_seconds", expectedInitialCpu},
            {"phase_guard_enabled", phaseGuardEnabled},
            {"observed_phase_cpu_seconds",
                spBudgetLedger.phaseCpuSeconds(initialPhase)},
            {"observed_phase_cpu_share",
                spBudgetLedger.observedPhaseShare(
                    initialPhase, data->elapsedCpuSeconds())},
            {"guardable_phase_cpu_seconds",
                spBudgetLedger.guardablePhaseCpuSeconds(
                    initialPhase)},
            {"guardable_phase_cpu_share",
                spBudgetLedger.guardablePhaseShare(
                    initialPhase, data->elapsedCpuSeconds())},
            {"phase_target_share",
                spBudgetLedger.configuredPhaseTargetShare(
                    initialPhase)},
            {"allowed", budgetAllowed},
        });
    if (!budgetAllowed) {
        SpExecutionSummary skipped;
        skipped.skipReason = "sp_cpu_share_guard";
        hgasp::RunLogger::event(
            hgasp::LogLevel::info,
            "sp",
            "solve_skipped_preflight",
            {},
            {
                {"iteration", data->iteration},
                {"scope", "global"},
                {"reason", skipped.skipReason},
                {"observed_sp_cpu_share",
                    spBudgetLedger.observedShare(
                        data->elapsedCpuSeconds())},
                {"guardable_sp_cpu_share",
                    spBudgetLedger.guardableShare(
                        data->elapsedCpuSeconds())},
                {"sp_cpu_target",
                    data->spCpuShareTarget},
        });
        return skipped;
    }
    const double outerCpuStarted =
        data->elapsedCpuSeconds();
    const SpWallClock::time_point outerWallStarted =
        SpWallClock::now();
    const double outerWallLimitSeconds =
        options.forceP2Execution &&
                options.lpTimeLimitSeconds.has_value() &&
                options.mipTimeLimitSeconds.has_value()
            ? *options.lpTimeLimitSeconds +
                  *options.mipTimeLimitSeconds
            : (hardMandatory
                   ? data->spFinalGlobalOuterTimeLimit
                   : data->spGlobalOuterTimeLimit);
    hgasp::SpBudgetCallToken budgetToken;
    ScopeExit outerSettlementGuard([&]() {
        settleSpOuterFallback(
            budgetToken,
            hgasp::SpScope::Global,
            "global",
            outerCpuStarted,
            options.forceP2Execution,
            hardMandatory);
    });
    hgasp::ScopedComponentTimer spTimer(
        hgasp::TimedComponent::SpLocalGlobal);
    refreshRouteProtections(true);

    SpMipStart hgaStart;
    hgaStart.source = "hga_global_incumbent";
    double targetObjective = restartBestFitness();
    if (const Individual* incumbent =
            routeGenerator->globalIncumbent()) {
        hgaStart.routes = routePool->exportRoutes(incumbent);
        hgaStart.requireMapping = true;
        // LP potential and strict MIP improvement are both measured against
        // the actual Global incumbent, never a weaker restart-local elite.
        targetObjective = incumbent->fit;
    }

    SpProblem problem = buildSpProblem();
    if (effectiveOptions.exploratoryConfiguration) {
        problem.requireOpenDepotUse = true;
        auto appendExcluded = [&](const hgasp::DepotConfigurationKey& key) {
            std::vector<bool> mask(
                static_cast<std::size_t>(data->numL), false);
            for (const int depot : key.openDepots) {
                mask.at(static_cast<std::size_t>(depot)) = true;
            }
            if (std::find(
                    problem.excludedDepotOpenConfigurations.begin(),
                    problem.excludedDepotOpenConfigurations.end(),
                    mask) ==
                problem.excludedDepotOpenConfigurations.end()) {
                problem.excludedDepotOpenConfigurations.push_back(
                    std::move(mask));
            }
        };
        for (const auto& view :
             routeGenerator->staticPopulationRuntimeViews()) {
            if (view.active && !view.unrestricted) {
                appendExcluded(view.configuration);
            }
        }
        if (!hgaStart.routes.empty()) {
            std::vector<int> incumbentDepots;
            incumbentDepots.reserve(hgaStart.routes.size());
            for (const StandaloneRoute& route : hgaStart.routes) {
                incumbentDepots.push_back(route.locId);
            }
            appendExcluded(hgasp::makeDepotConfigurationKey(
                std::move(incumbentDepots)));
        }
        // The incumbent configuration is deliberately forbidden. Its routes
        // remain useful as mandatory active-view columns, but it cannot be a
        // valid MIP start for this exploratory model.
        hgaStart.requireMapping = false;
        hgaStart.source = "hga_incumbent_columns_exploratory";
    }
    std::vector<StandaloneRoute> snapshot = routePool->getPool();
    bool assignmentPrepassImprovedWarmStart = false;
    // Every scheduled CLRP Global-SP call solves the same route master.  Give
    // its incumbent route sequences the same call-local depot-assignment
    // closure as Final Global-SP.  forceP2Execution identifies the two bounded
    // pre-reduction calls; no trigger, persistent RoutePool state, or hard-TW
    // path is added here, and their existing outer budget remains authoritative.
    if ((hardMandatory || options.forceP2Execution) &&
        !data->hasHardTimeWindows() && !hgaStart.routes.empty()) {
        const std::size_t assignmentSeedRouteCount =
            hgaStart.routes.size();
        RoutePool::DepotReassignmentClosure assignmentClosure =
            routePool->buildDepotReassignmentClosure(
                {}, hgaStart.routes);
        SpMipRequest assignmentRequest;
        assignmentRequest.mode = SpMipMode::first_improvement;
        assignmentRequest.timeLimitSeconds = std::min(
            10.0,
            hgasp::remainingOuterSpBudget(
                outerWallLimitSeconds,
                elapsedSpWallSeconds(outerWallStarted)));
        assignmentRequest.targetObjective = targetObjective - Min;
        assignmentRequest.solutionPoolSize = 1;
        assignmentRequest.solutionPoolGap = 0.0;
        assignmentRequest.solutionPoolSearchMode = 0;
        bool assignmentImproved = false;
        double assignmentObjective = targetObjective;
        if (*assignmentRequest.timeLimitSeconds > 0.0) {
            const SpResult assignmentResult = spSolver->solveMip(
                problem,
                assignmentClosure.snapshot,
                assignmentRequest,
                hgaStart);
            if (assignmentResult.hasFeasibleSolution() &&
                !assignmentResult.candidateSolutions.empty()) {
                const auto best = std::min_element(
                    assignmentResult.candidateSolutions.begin(),
                    assignmentResult.candidateSolutions.end(),
                    [](const SpSolution& left, const SpSolution& right) {
                        return left.objective < right.objective;
                    });
                assignmentObjective = best->objective;
                if (assignmentObjective + Min < targetObjective) {
                    std::vector<StandaloneRoute> reassignedRoutes;
                    reassignedRoutes.reserve(
                        best->selectedRouteIndices.size());
                    for (const std::size_t routeIndex :
                         best->selectedRouteIndices) {
                        if (routeIndex >= assignmentClosure.snapshot.size()) {
                            throw std::out_of_range(
                                "depot-reassignment prepass returned an "
                                "invalid route index");
                        }
                        reassignedRoutes.push_back(
                            assignmentClosure.snapshot[routeIndex]);
                    }
                    hgaStart.routes = std::move(reassignedRoutes);
                    hgaStart.source =
                        "incumbent_depot_reassignment_closure";
                    hgaStart.requireMapping = true;
                    assignmentImproved = true;
                    assignmentPrepassImprovedWarmStart = true;
                }
            }
            hgasp::RunLogger::event(
                hgasp::LogLevel::info,
                "sp",
                "depot_reassignment_prepass_finished",
                {},
                {
                    {"iteration", data->iteration},
                    {"status", assignmentResult.statusName},
                    {"seed_routes", static_cast<unsigned long long>(
                        assignmentSeedRouteCount)},
                    {"closure_columns", static_cast<unsigned long long>(
                        assignmentClosure.snapshot.size())},
                    {"objective_before", targetObjective},
                    {"objective_after", assignmentObjective},
                    {"improved", assignmentImproved},
                    {"cpu_seconds", assignmentResult.totalTimeSeconds},
                    {"wall_seconds", assignmentResult.totalWallTimeSeconds},
                });
        }
        RoutePool::DepotReassignmentClosure closure =
            routePool->buildDepotReassignmentClosure(
                std::move(snapshot), hgaStart.routes);
        effectiveOptions.allowEphemeralFeedbackColumns =
            closure.columnsAppended > 0U;
        if (assignmentPrepassImprovedWarmStart) {
            // The call already produced a strict improvement inside its base
            // window, so the same global master may use the full hard cap.
            effectiveOptions.improvementExtensionGateSeconds.reset();
        }
        hgasp::RunLogger::event(
            hgasp::LogLevel::info,
            "sp",
            "depot_reassignment_closure_built",
            {},
            {
                {"iteration", data->iteration},
                {"seed_routes", static_cast<unsigned long long>(
                    hgaStart.routes.size())},
                {"candidates_evaluated", static_cast<unsigned long long>(
                    closure.candidatesEvaluated)},
                {"columns_appended", static_cast<unsigned long long>(
                    closure.columnsAppended)},
                {"representatives_replaced",
                    static_cast<unsigned long long>(
                        closure.representativesReplaced)},
                {"final_snapshot_columns",
                    static_cast<unsigned long long>(
                        closure.snapshot.size())},
            });
        snapshot = std::move(closure.snapshot);
    }

    return executeSpSnapshot(
        *spSolver,
        std::move(problem),
        std::move(snapshot),
        std::move(hgaStart),
        "global",
        targetObjective,
        std::nullopt,
        hardMandatory,
        outerCpuStarted,
        outerWallStarted,
        outerWallLimitSeconds,
        budgetToken,
        effectiveOptions);
}

AlgorithmOrchestrator::SpExecutionSummary
AlgorithmOrchestrator::executeConfigurationLocalSp(
        const int populationIndex) {
    SpExecutionOptions options;
    // A Local RMP is a bounded rescue for one fixed configuration, not an
    // attempt to prove its restricted master optimal.
    options.firstImprovement = true;
    return executeConfigurationLocalSp(populationIndex, options);
}

AlgorithmOrchestrator::SpExecutionSummary
AlgorithmOrchestrator::executeConfigurationLocalSp(
        const int populationIndex,
        const SpExecutionOptions& options) {
    SpExecutionSummary skipped;
    if (populationIndex < 0 ||
        static_cast<std::size_t>(populationIndex) >=
            routeGenerator->popman->popula.size() ||
        routeGenerator->popman->popula[populationIndex].empty()) {
        return skipped;
    }
    const bool expectsLp =
        options.forceLp;
    const hgasp::SpBudgetPhase initialPhase =
        expectsLp
            ? hgasp::SpBudgetPhase::LpProbe
            : hgasp::SpBudgetPhase::LocalMip;
    const double expectedInitialCpu =
        hgasp::conservativeProcessCpuProjection(
            expectsLp
                ? options.lpTimeLimitSeconds.value_or(
                      data->spLocalLpTimeLimit)
                : options.mipTimeLimitSeconds.value_or(
                      std::min(
                          data->spLocalOuterTimeLimit,
                          data->spOnlineLocalMipTimeLimit)),
            data->spThreads,
            spBudgetLedger.lastPhaseCpuSeconds(initialPhase));
    const double reserveShare =
        initialPhase == hgasp::SpBudgetPhase::LocalMip
            ? data->spMipRescueReserveTarget
            : 0.0;
    const bool familyAllowed = spBudgetLedger.canSpend(
        hgasp::SpScope::Local,
        expectedInitialCpu,
        data->elapsedCpuSeconds(),
        false);
    const bool phaseGuardEnabled = true;
    const bool phaseAllowed =
        !phaseGuardEnabled ||
        spBudgetLedger.canSpendPhase(
            initialPhase,
            expectedInitialCpu,
            data->elapsedCpuSeconds(),
            false,
            reserveShare);
    const bool budgetAllowed = phaseGuardEnabled
        ? phaseAllowed
        : familyAllowed;
    hgasp::RunLogger::event(
        hgasp::LogLevel::info,
        "sp",
        "budget_guard_decision",
        {},
        {
            {"iteration", data->iteration},
            {"scope", "local"},
            {"execution_mode", "p2"},
            {"scheduling_bypass", options.forceP2Execution},
            {"mandatory", false},
            {"mandatory_exemption", false},
            {"exemption_reason", "none"},
            {"observed_sp_cpu_seconds",
                spBudgetLedger.totalCpuSeconds()},
            {"observed_sp_cpu_share",
                spBudgetLedger.observedShare(
                    data->elapsedCpuSeconds())},
            {"guardable_sp_cpu_seconds",
                spBudgetLedger.guardableCpuSeconds()},
            {"guardable_sp_cpu_share",
                spBudgetLedger.guardableShare(
                    data->elapsedCpuSeconds())},
            {"mandatory_exempt_cpu_seconds",
                spBudgetLedger.mandatoryExemptCpuSeconds()},
            {"target_share", data->spCpuShareTarget},
            {"forecast_cpu_seconds", expectedInitialCpu},
            {"phase_guard_enabled", phaseGuardEnabled},
            {"observed_phase_cpu_seconds",
                spBudgetLedger.phaseCpuSeconds(initialPhase)},
            {"observed_phase_cpu_share",
                spBudgetLedger.observedPhaseShare(
                    initialPhase, data->elapsedCpuSeconds())},
            {"guardable_phase_cpu_seconds",
                spBudgetLedger.guardablePhaseCpuSeconds(
                    initialPhase)},
            {"guardable_phase_cpu_share",
                spBudgetLedger.guardablePhaseShare(
                    initialPhase, data->elapsedCpuSeconds())},
            {"phase_target_share",
                spBudgetLedger.configuredPhaseTargetShare(
                    initialPhase)},
            {"allowed", budgetAllowed},
        });
    if (!budgetAllowed) {
        skipped.skipReason = "sp_cpu_share_guard";
        hgasp::RunLogger::event(
            hgasp::LogLevel::info,
            "sp",
            "solve_skipped_preflight",
            {},
            {
                {"iteration", data->iteration},
                {"scope", "local"},
                {"reason", skipped.skipReason},
                {"observed_sp_cpu_share",
                    spBudgetLedger.observedShare(
                        data->elapsedCpuSeconds())},
                {"guardable_sp_cpu_share",
                    spBudgetLedger.guardableShare(
                        data->elapsedCpuSeconds())},
                {"sp_cpu_target",
                    data->spCpuShareTarget},
            });
        return skipped;
    }

    const double outerCpuStarted =
        data->elapsedCpuSeconds();
    const SpWallClock::time_point outerWallStarted =
        SpWallClock::now();
    const double outerWallLimitSeconds =
        options.forceP2Execution &&
                options.lpTimeLimitSeconds.has_value() &&
                options.mipTimeLimitSeconds.has_value()
            ? *options.lpTimeLimitSeconds +
                  *options.mipTimeLimitSeconds
            : data->spLocalOuterTimeLimit;
    const std::string spScope =
        "local_population_" +
        std::to_string(populationIndex);
    hgasp::SpBudgetCallToken budgetToken;
    ScopeExit outerSettlementGuard([&]() {
        settleSpOuterFallback(
            budgetToken,
            hgasp::SpScope::Local,
            spScope,
            outerCpuStarted,
            options.forceP2Execution,
            false);
    });
    hgasp::ScopedComponentTimer spTimer(
        hgasp::TimedComponent::SpLocalGlobal);
    refreshRouteProtections(true);
    const Individual* elite =
        routeGenerator->popman->popula[populationIndex].front();
    std::vector<bool> openDepots(data->numL, false);
    for (int depot = 0; depot < data->numL; ++depot) {
        openDepots[depot] = elite->location[depot].used;
    }

    const std::vector<StandaloneRoute> eliteRoutes =
        routePool->exportRoutes(elite);
    std::vector<StandaloneRoute> localPool =
        filterRoutesForConfiguration(
            routePool->getPool(), openDepots, eliteRoutes);
    std::vector<bool> covered(data->numV, false);
    for (const StandaloneRoute& route : localPool) {
        for (const int customer : route.sequence) {
            covered[customer] = true;
        }
    }
    if (localPool.empty() ||
        std::any_of(
            covered.begin(),
            covered.end(),
            [](const bool value) { return !value; })) {
        std::cout
            << "[SP Local Skip] population=" << populationIndex
            << " has no complete route-pool coverage for its fixed "
               "depot configuration"
            << std::endl;
        skipped.columns = localPool.size();
        return accountSpPreparationSkip(
            std::move(skipped),
            budgetToken,
            hgasp::SpScope::Local,
            spScope,
            "configuration_pool_incomplete",
            outerCpuStarted,
            options.forceP2Execution);
    }

    SpProblem localProblem = buildSpProblem();
    localProblem.fixedDepotOpen = openDepots;
    SpMipStart localStart;
    localStart.source = "configuration_elite";
    localStart.routes = eliteRoutes;
    localStart.requireMapping = true;
    return executeSpSnapshot(
        *localSpSolver,
        std::move(localProblem),
        std::move(localPool),
        std::move(localStart),
        spScope,
        elite->fit,
        hgasp::makeDepotConfigurationKey(openDepots),
        false,
        outerCpuStarted,
        outerWallStarted,
        outerWallLimitSeconds,
        budgetToken,
        options);
}


void AlgorithmOrchestrator::recordSpFeedback(
        const SpProblem& problem,
        const std::vector<StandaloneRoute>& snapshot,
        const SpResult& result,
        const std::string& spCallId,
        const bool allowEphemeralFeedbackColumns,
        double* const usefulColumnCredit) {
    if (!result.lpFeedbackAvailable &&
        result.candidateSolutions.empty()) {
        return;
    }

    const bool local = !problem.fixedDepotOpen.empty();
    const std::optional<hgasp::DepotConfigurationKey> configuration =
        local
            ? std::optional<hgasp::DepotConfigurationKey>(
                  hgasp::makeDepotConfigurationKey(
                      problem.fixedDepotOpen))
            : std::nullopt;
    const hgasp::SpFeedbackScope feedbackScope =
        local
            ? hgasp::SpFeedbackScope::ConfigurationLocal
            : hgasp::SpFeedbackScope::Global;

    std::size_t ephemeralFeedbackColumns = 0U;
    if (!local && result.lpFeedbackAvailable) {
        ephemeralFeedbackColumns = routePool->updateDualFeedback(
            snapshot,
            result.customerDuals,
            result.routeReducedCosts,
            allowEphemeralFeedbackColumns);
    }

    const std::vector<hgasp::OriginCredit> credits =
        routePool->applySpFeedbackAndCollectCredits(
            snapshot,
            result,
            data->iteration,
            feedbackScope,
            configuration);
    double totalCredit = 0.0;
    for (const hgasp::OriginCredit& credit : credits) {
        totalCredit += std::max(0.0, credit.credit);
    }
    if (usefulColumnCredit != nullptr) {
        *usefulColumnCredit += totalCredit;
    }
    routeGenerator->reportDelayedRouteCredit(
        credits, spCallId, feedbackScope);

    hgasp::RunLogger::event(
        hgasp::LogLevel::info,
        "sp",
        "feedback_recorded",
        {},
        {
            {"iteration", data->iteration},
            {"scope", local ? "local" : "global"},
            {"call_id", spCallId},
            {"configuration",
                configuration.has_value()
                    ? configurationLabel(*configuration)
                    : std::string("global")},
            {"useful_column_credit", totalCredit},
            {"ephemeral_feedback_columns_skipped",
                static_cast<unsigned long long>(
                    ephemeralFeedbackColumns)},
            {"local_credit_baseline_evictions",
                static_cast<unsigned long long>(
                    routePool->
                        getLocalCreditBaselineEvictionCount())},
        });
}


AlgorithmOrchestrator::SpExecutionSummary
AlgorithmOrchestrator::executeSpSnapshot(
        SPSolver& solver,
        SpProblem problem,
        std::vector<StandaloneRoute> poolSnapshot,
        SpMipStart start,
        const std::string& scope,
        const double targetObjective,
        std::optional<hgasp::DepotConfigurationKey>
            configuration,
        const bool hardMandatory,
        const double outerCpuStarted,
        const SpWallClock::time_point outerWallStarted,
        const double outerWallLimitSeconds,
        hgasp::SpBudgetCallToken& budgetToken,
        const SpExecutionOptions& options) {
    SpExecutionSummary summary;
    summary.attempted = true;
    if (scope.rfind("local_", 0U) == 0U) {
        routeGenerator->reportLocalSpCall(scope);
    }
    summary.columns = poolSnapshot.size();
    summary.configuration = configuration;
    const double beforeObjective = restartBestFitness();
    summary.objectiveReference = configuration.has_value()
        ? targetObjective
        : beforeObjective;
    const double cpuStarted =
        outerCpuStarted >= 0.0
            ? outerCpuStarted
            : data->elapsedCpuSeconds();
    const std::size_t currentPoolSize =
        poolSnapshot.size();
    std::cout << "\n[SP MONITOR] scope=" << scope
              << " iteration=" << data->iteration
              << " | RoutePool Size: " << currentPoolSize
              << std::endl;
    const std::string spCallId =
        scope + "@" + std::to_string(data->iteration);
    const auto assignedStageLimit =
        [&](const double requestedSeconds,
            const char* const stage) {
            const double consumedWallSeconds =
                elapsedSpWallSeconds(outerWallStarted);
            const double remainingSeconds =
                hgasp::remainingOuterSpBudget(
                    outerWallLimitSeconds,
                    consumedWallSeconds);
            const double assignedSeconds = std::min(
                requestedSeconds, remainingSeconds);
            hgasp::RunLogger::event(
                hgasp::LogLevel::info,
                "sp",
                "outer_budget_assigned",
                {},
                {
                    {"iteration", data->iteration},
                    {"scope", scope},
                    {"call_id", spCallId},
                    {"stage", stage == nullptr ? "unknown" : stage},
                    {"declared_outer_wall_seconds",
                        outerWallLimitSeconds},
                    {"consumed_before_stage_wall_seconds",
                        consumedWallSeconds},
                    {"requested_stage_limit_seconds",
                        requestedSeconds},
                    {"assigned_stage_limit_seconds",
                        assignedSeconds},
                });
            return assignedSeconds;
        };
    hgasp::RunLogger::event(
        hgasp::LogLevel::info,
        "sp",
        "solve_started",
        {},
        {
            {"iteration", data->iteration},
            {"scope", scope},
            {"call_id", spCallId},
            {"columns", static_cast<unsigned long long>(
                currentPoolSize)},
            {"mip_start_requested_source",
                start.source.empty() ? "external" : start.source},
            {"mip_start_requested_routes",
                static_cast<unsigned long long>(start.routes.size())},
            {"mip_start_required", start.requireMapping},
            {"search_cpu_seconds",
                data->elapsedCpuSeconds()},
        });
    hgasp::RunLogger::status(
        "sp",
        "solving",
        {
            {"iteration", data->iteration},
            {"scope", scope},
            {"call_id", spCallId},
            {"columns", static_cast<unsigned long long>(
                currentPoolSize)},
        });

    std::vector<StandaloneRoute> solvedSnapshot =
        poolSnapshot;
    SpResult spResult;
    const bool local = !problem.fixedDepotOpen.empty();
    std::optional<hgasp::DepotConfigurationKey>
        startConfiguration = configuration;
    if (!startConfiguration.has_value() && !start.routes.empty()) {
        std::vector<int> depots;
        depots.reserve(start.routes.size());
        for (const StandaloneRoute& route : start.routes) {
            depots.push_back(route.locId);
        }
        startConfiguration =
            hgasp::makeDepotConfigurationKey(std::move(depots));
    }
    hgasp::FinalMasterScope masterScope =
        hgasp::FinalMasterScope::FullActiveMaster;
    const bool globalFirstImprovement =
        !local &&
        (data->spGlobalFirstImprovementEnabled ||
         options.firstImprovement) &&
        !hardMandatory;
    const hgasp::SpBudgetPhase mipBudgetPhase =
        hardMandatory
            ? hgasp::SpBudgetPhase::FinalGlobal
            : (local
                   ? hgasp::SpBudgetPhase::LocalMip
                   : hgasp::SpBudgetPhase::GlobalMip);
    const hgasp::SpScope budgetScope =
        local ? hgasp::SpScope::Local : hgasp::SpScope::Global;
    double lpProbeCpuSeconds = 0.0;
    double mipCpuSeconds = 0.0;
    bool lpFeedbackRecorded = false;
    const auto finalizeSpCpu = [&]() {
        summary.cpuSeconds = std::max(
            0.0,
            data->elapsedCpuSeconds() - cpuStarted);
        if (spBudgetLedger.settle(
                budgetToken,
                budgetScope,
                summary.cpuSeconds,
                hardMandatory)) {
            hgasp::RunLogger::event(
                hgasp::LogLevel::info,
                "sp",
                "budget_accounted",
                {},
                {
                    {"iteration", data->iteration},
                    {"scope", scope},
                    {"call_id", spCallId},
                    {"outer_cpu_seconds",
                        summary.cpuSeconds},
                    {"execution_mode", "p2"},
                    {"scheduling_bypass",
                        options.forceP2Execution},
                    {"mandatory", hardMandatory},
                    {"mandatory_exemption", hardMandatory},
                    {"exemption_reason",
                        hardMandatory
                            ? "mandatory_final_global"
                            : "none"},
                    {"observed_sp_cpu_seconds",
                        spBudgetLedger.totalCpuSeconds()},
                    {"observed_sp_cpu_share",
                        spBudgetLedger.observedShare(
                            data->elapsedCpuSeconds())},
                    {"guardable_sp_cpu_seconds",
                        spBudgetLedger.guardableCpuSeconds()},
                    {"guardable_sp_cpu_share",
                        spBudgetLedger.guardableShare(
                            data->elapsedCpuSeconds())},
                    {"mandatory_exempt_cpu_seconds",
                        spBudgetLedger
                            .mandatoryExemptCpuSeconds()},
                    // Historical field meanings remain all-observed.
                    {"family_cpu_seconds",
                        spBudgetLedger.totalCpuSeconds()},
                    {"family_cpu_share",
                        spBudgetLedger.observedShare(
                            data->elapsedCpuSeconds())},
                    {"target_share",
                        data->spCpuShareTarget},
                    {"lp_probe_cpu_seconds",
                        lpProbeCpuSeconds},
                    {"mip_cpu_seconds", mipCpuSeconds},
                    {"declared_outer_wall_seconds",
                        outerWallLimitSeconds},
                    {"consumed_outer_wall_seconds",
                        elapsedSpWallSeconds(outerWallStarted)},
                    {"remaining_outer_wall_seconds",
                        hgasp::remainingOuterSpBudget(
                            outerWallLimitSeconds,
                            elapsedSpWallSeconds(
                                outerWallStarted))},
                });
        }
    };
		// Global P2 stages an LP over the exact immutable snapshot before
		// integer work. V1.14 Local calls may also force an LP as a private
		// gate; restricted Local duals never overwrite Global guidance.
        const bool lpConsumerEnabled =
            !options.skipLp && shouldImportDualFeedback(problem);
        hgasp::SpExecutionPlan executionPlan =
            hgasp::makeSpExecutionPlan(
                lpConsumerEnabled,
                true);
        if (options.forceLp) {
            executionPlan.runLp = true;
            executionPlan.staged = true;
        }
        const double requestedP2MipTimeLimit =
            options.mipTimeLimitSeconds.has_value()
                ? *options.mipTimeLimitSeconds
                : hgasp::selectP2MipTimeLimit(
                      local,
                      hardMandatory,
                      data->hasHardTimeWindows(),
                      data->twProfileEarlyStopEnabled,
                      std::min(
                          data->spLocalOuterTimeLimit,
                          data->spOnlineLocalMipTimeLimit),
                      std::min(
                          data->spGlobalOuterTimeLimit,
                          data->spOnlineGlobalMipTimeLimit),
                      data->spFinalGlobalMipTimeLimit,
                      data->spTwFinalMipTimeLimit);
        const double requestedP2InitialTimeLimit =
            executionPlan.runLp
                ? options.lpTimeLimitSeconds.value_or(
                      hardMandatory
                          ? data->spFinalGlobalLpTimeLimit
                          : (local
                                 ? data->spLocalLpTimeLimit
                                 : data->spGlobalLpTimeLimit))
                : requestedP2MipTimeLimit;
        const hgasp::SpBudgetPhase p2InitialPhase =
            executionPlan.runLp
                ? hgasp::SpBudgetPhase::LpProbe
                : mipBudgetPhase;
        const double p2InitialForecast =
            hgasp::conservativeProcessCpuProjection(
                requestedP2InitialTimeLimit,
                data->spThreads,
                spBudgetLedger.lastPhaseCpuSeconds(
                    p2InitialPhase));
        const double p2InFlightCpu = std::max(
            0.0,
            data->elapsedCpuSeconds() - cpuStarted);
        const double p2InitialReserve =
            p2InitialPhase == hgasp::SpBudgetPhase::GlobalMip
                ? std::max(
                      data->spGlobalCpuReserveTarget,
                      data->spMipRescueReserveTarget)
                : (p2InitialPhase ==
                           hgasp::SpBudgetPhase::LocalMip
                       ? data->spMipRescueReserveTarget
                       : 0.0);
        const bool p2InitialAllowed =
            spBudgetLedger.canSpendPhase(
                p2InitialPhase,
                p2InitialForecast,
                data->elapsedCpuSeconds(),
                hardMandatory || options.reservedPhaseBudget,
                p2InitialReserve,
                p2InFlightCpu);
        hgasp::RunLogger::event(
            hgasp::LogLevel::info,
            "sp",
            "budget_guard_decision",
            {},
            {
                {"iteration", data->iteration},
                {"scope", scope},
                {"call_id", spCallId},
                {"stage", executionPlan.runLp
                    ? "lp_probe"
                    : "mip"},
                {"execution_mode", "p2"},
                {"scheduling_bypass",
                    options.forceP2Execution},
                {"mandatory", hardMandatory},
                {"mandatory_exemption", hardMandatory},
                {"reserved_phase_budget", options.reservedPhaseBudget},
                {"exemption_reason",
                    hardMandatory
                        ? "mandatory_final_global"
                        : "none"},
                {"observed_sp_cpu_seconds",
                    spBudgetLedger.totalCpuSeconds()},
                {"observed_sp_cpu_share",
                    spBudgetLedger.observedShare(
                        data->elapsedCpuSeconds())},
                {"guardable_sp_cpu_seconds",
                    spBudgetLedger.guardableCpuSeconds()},
                {"guardable_sp_cpu_share",
                    spBudgetLedger.guardableShare(
                        data->elapsedCpuSeconds())},
                {"target_share", data->spCpuShareTarget},
                {"forecast_cpu_seconds", p2InitialForecast},
                {"in_flight_cpu_seconds", p2InFlightCpu},
                {"phase_guard_enabled", true},
                {"observed_phase_cpu_seconds",
                    spBudgetLedger.phaseCpuSeconds(p2InitialPhase)},
                {"observed_phase_cpu_share",
                    spBudgetLedger.observedPhaseShare(
                        p2InitialPhase,
                        data->elapsedCpuSeconds())},
                {"guardable_phase_cpu_seconds",
                    spBudgetLedger.guardablePhaseCpuSeconds(
                        p2InitialPhase)},
                {"guardable_phase_cpu_share",
                    spBudgetLedger.guardablePhaseShare(
                        p2InitialPhase,
                        data->elapsedCpuSeconds())},
                {"phase_target_share",
                    spBudgetLedger.configuredPhaseTargetShare(
                        p2InitialPhase)},
                {"allowed", p2InitialAllowed},
            });
        if (!p2InitialAllowed) {
            summary.attempted = false;
            summary.skipReason = "sp_cpu_share_guard";
            finalizeSpCpu();
            hgasp::RunLogger::event(
                hgasp::LogLevel::info,
                "sp",
                "solve_skipped",
                {},
                {
                    {"iteration", data->iteration},
                    {"scope", scope},
                    {"call_id", spCallId},
                    {"reason", summary.skipReason},
                });
            hgasp::RunLogger::status(
                "sp",
                "idle",
                {
                    {"iteration", data->iteration},
                    {"scope", scope},
                    {"call_id", spCallId},
                    {"status", "SKIPPED"},
                });
            return summary;
        }
        const char* executionMode = executionPlan.combined
            ? "combined"
            : (executionPlan.runLp ? "staged" : "mip_only");
        const hgasp::EventFields p2Fields{
            {"iteration", data->iteration},
            {"scope", scope},
            {"call_id", spCallId},
            {"mode", executionMode},
            {"target_objective", Max},
            {"global_reserve_share", 0.0},
        };
        SpResult p2LpResult;
        hgasp::P2MipGateDecision p2Gate;
        if (executionPlan.runLp) {
            hgasp::RunLogger::event(
                hgasp::LogLevel::info,
                "sp",
                "lp_probe_started",
                {},
                p2Fields);
            SpLpRequest lpRequest;
            const double requestedLpTimeLimit =
                options.lpTimeLimitSeconds.value_or(
                    hardMandatory
                        ? data->spFinalGlobalLpTimeLimit
                        : (local
                               ? data->spLocalLpTimeLimit
                               : data->spGlobalLpTimeLimit));
            lpRequest.timeLimitSeconds = assignedStageLimit(
                requestedLpTimeLimit,
                "lp_probe");
            if (lpRequest.timeLimitSeconds <= 0.0) {
                summary.attempted = false;
                summary.skipReason =
                    "outer_wall_budget_exhausted";
                finalizeSpCpu();
                hgasp::RunLogger::event(
                    hgasp::LogLevel::info,
                    "sp",
                    "solve_skipped",
                    {},
                    {
                        {"iteration", data->iteration},
                        {"scope", scope},
                        {"call_id", spCallId},
                        {"reason", summary.skipReason},
                    });
                hgasp::RunLogger::status(
                    "sp",
                    "idle",
                    {
                        {"iteration", data->iteration},
                        {"scope", scope},
                        {"call_id", spCallId},
                        {"status", "SKIPPED"},
                    });
                return summary;
            }
            const double p2LpCpuStarted =
                data->elapsedCpuSeconds();
            p2LpResult = solver.solveLp(
                problem,
                poolSnapshot,
                lpRequest);
            lpProbeCpuSeconds = std::max(
                0.0,
                data->elapsedCpuSeconds() - p2LpCpuStarted);
            spBudgetLedger.recordPhase(
                hgasp::SpBudgetPhase::LpProbe,
                lpProbeCpuSeconds,
                hardMandatory);
            summary.lpAttempted =
                p2LpResult.lpAttempted;
            summary.lpFeedbackAvailable =
                p2LpResult.lpFeedbackAvailable;
            if (options.exploratoryConfiguration) {
                if (p2LpResult.lpFeedbackAvailable) {
                    recordSpFeedback(
                        problem,
                        poolSnapshot,
                        p2LpResult,
                        spCallId,
                        options.allowEphemeralFeedbackColumns,
                        &summary.usefulColumnCredit);
                    lpFeedbackRecorded = true;
                }
                // A bounded LP may legitimately finish without an optimal
                // dual solution. The active-view limit is still part of the
                // V3.3 master contract: duals refine its ordering, but their
                // absence must not silently send the full pool to the MIP.
                RoutePool::ExploratoryActiveView activeView =
                    routePool->buildExploratoryActiveView(
                        poolSnapshot,
                        p2LpResult,
                        start.routes,
                        options.exploratoryActiveColumnTarget);
                if (activeView.reduced) {
                    poolSnapshot = std::move(activeView.active);
                    solvedSnapshot = poolSnapshot;
                    masterScope =
                        hgasp::FinalMasterScope::
                            ExploratoryActiveView;
                }
                hgasp::RunLogger::event(
                    hgasp::LogLevel::info,
                    "sp",
                    "exploratory_active_view_built",
                    {},
                    {
                        {"iteration", data->iteration},
                        {"call_id", spCallId},
                        {"full_columns",
                            static_cast<unsigned long long>(
                                currentPoolSize)},
                        {"active_columns",
                            static_cast<unsigned long long>(
                                poolSnapshot.size())},
                        {"reserve_columns",
                            static_cast<unsigned long long>(
                                activeView.reserveCount)},
                        {"mandatory_columns",
                            static_cast<unsigned long long>(
                                activeView.mandatoryCount)},
                        {"protected_columns",
                            static_cast<unsigned long long>(
                                activeView.protectedCount)},
                        {"lp_support_columns",
                            static_cast<unsigned long long>(
                                activeView.lpSupportCount)},
                        {"dual_feedback_usable",
                            activeView.dualFeedbackUsable},
                        {"reduced", activeView.reduced},
                    });
            }
            if (!isSupportedSpTerminalStatus(
                    p2LpResult.status)) {
                if (!local ||
                    !options.deferUnsupportedSolverOutcome) {
                    throw std::runtime_error(
                        "fatal or unsupported P2 LP outcome (" +
                        p2LpResult.statusName + "): " +
                        (p2LpResult.diagnostic.empty()
                             ? std::string(
                                   "no diagnostic was provided")
                             : p2LpResult.diagnostic));
                }
                // A population rescue may be retried after its family
                // cooldown when the solver precondition recovers. Preserve
                // the unsupported result for the common audited finish path,
                // but do not launch a MIP after a failed LP stage.
                p2Gate.runMip = false;
                spResult = p2LpResult;
                summary.skipReason =
                    "unsupported_local_lp_status";
            } else {
                const bool eligibleCompletedLp =
                    p2LpResult.status == SpStatus::optimal &&
                    p2LpResult.lpFeedbackAvailable &&
                    p2LpResult.lpObjective.has_value();
                const hgasp::SpMipGateDecision boundGate =
                    hgasp::evaluateSpMipGate(
                        eligibleCompletedLp
                            ? p2LpResult.lpObjective
                            : std::nullopt,
                        targetObjective,
                        Min,
                        0.0,
                        true,
                        false);
                const bool rigorousNoImprovement =
                    eligibleCompletedLp &&
                    hgasp::p2LpBoundExcludesImprovement(
                        *p2LpResult.lpObjective,
                        targetObjective,
                        Min);
                const bool twFinalLpGate =
                    hardMandatory &&
                    data->hasHardTimeWindows() &&
                    data->twProfileEarlyStopEnabled;
                p2Gate = hgasp::evaluateP2MipGate(
                    data->spLpSkipMipIfNoPotential &&
                        !options.exploratoryConfiguration &&
                        (!hardMandatory || twFinalLpGate),
                    eligibleCompletedLp,
                    true,
                    rigorousNoImprovement);
                summary.rigorousLpNoImprovement =
                    p2Gate.rigorousNoImprovement;
                summary.mipSkippedByBound = !p2Gate.runMip;
                summary.lpObjective =
                    p2LpResult.lpObjective.value_or(Max);
                summary.relativeLpPotential =
                    boundGate.relativePotential;
                if (!p2Gate.runMip) {
                    summary.skipReason = p2Gate.reason;
                    spResult = p2LpResult;
                    hgasp::RunLogger::event(
                        hgasp::LogLevel::info,
                        "sp",
                        "mip_skipped",
                        {},
                        {
                            {"iteration", data->iteration},
                            {"scope", scope},
                            {"call_id", spCallId},
                            {"reason", summary.skipReason},
                            {"lp_objective",
                                summary.lpObjective},
                            {"target_objective",
                                targetObjective},
                        });
                }
            }
        }
        if (executionPlan.runMip && p2Gate.runMip) {
            const double p2MipForecast =
                hgasp::conservativeProcessCpuProjection(
                    requestedP2MipTimeLimit,
                    data->spThreads,
                    spBudgetLedger.lastPhaseCpuSeconds(
                        mipBudgetPhase));
            const double p2MipInFlightCpu = std::max(
                0.0,
                data->elapsedCpuSeconds() - cpuStarted);
            const double p2MipReserve = local
                ? data->spMipRescueReserveTarget
                : std::max(
                      data->spGlobalCpuReserveTarget,
                      data->spMipRescueReserveTarget);
            const bool p2MipAllowed =
                spBudgetLedger.canSpendPhase(
                    mipBudgetPhase,
                    p2MipForecast,
                    data->elapsedCpuSeconds(),
                    hardMandatory || options.reservedPhaseBudget,
                    p2MipReserve,
                    p2MipInFlightCpu);
            hgasp::RunLogger::event(
                hgasp::LogLevel::info,
                "sp",
                "budget_guard_decision",
                {},
                {
                    {"iteration", data->iteration},
                    {"scope", scope},
                    {"call_id", spCallId},
                    {"stage", "mip"},
                    {"execution_mode", "p2"},
                    {"scheduling_bypass",
                        options.forceP2Execution},
                    {"mandatory", hardMandatory},
                    {"mandatory_exemption", hardMandatory},
                    {"reserved_phase_budget", options.reservedPhaseBudget},
                    {"exemption_reason",
                        hardMandatory
                            ? "mandatory_final_global"
                            : "none"},
                    {"observed_sp_cpu_seconds",
                        spBudgetLedger.totalCpuSeconds()},
                    {"observed_sp_cpu_share",
                        spBudgetLedger.observedShare(
                            data->elapsedCpuSeconds())},
                    {"guardable_sp_cpu_seconds",
                        spBudgetLedger.guardableCpuSeconds()},
                    {"guardable_sp_cpu_share",
                        spBudgetLedger.guardableShare(
                            data->elapsedCpuSeconds())},
                    {"target_share", data->spCpuShareTarget},
                    {"forecast_cpu_seconds", p2MipForecast},
                    {"in_flight_cpu_seconds", p2MipInFlightCpu},
                    {"phase_guard_enabled", true},
                    {"observed_phase_cpu_seconds",
                        spBudgetLedger.phaseCpuSeconds(
                            mipBudgetPhase)},
                    {"observed_phase_cpu_share",
                        spBudgetLedger.observedPhaseShare(
                            mipBudgetPhase,
                            data->elapsedCpuSeconds())},
                    {"guardable_phase_cpu_seconds",
                        spBudgetLedger.guardablePhaseCpuSeconds(
                            mipBudgetPhase)},
                    {"guardable_phase_cpu_share",
                        spBudgetLedger.guardablePhaseShare(
                            mipBudgetPhase,
                            data->elapsedCpuSeconds())},
                    {"phase_target_share",
                        spBudgetLedger.configuredPhaseTargetShare(
                            mipBudgetPhase)},
                    {"allowed", p2MipAllowed},
                });
            if (!p2MipAllowed) {
                p2Gate.runMip = false;
                summary.mipSkippedByBound = true;
                summary.skipReason = "sp_cpu_share_guard";
                spResult = p2LpResult;
                hgasp::RunLogger::event(
                    hgasp::LogLevel::info,
                    "sp",
                    "mip_skipped",
                    {},
                    {
                        {"iteration", data->iteration},
                        {"scope", scope},
                        {"call_id", spCallId},
                        {"reason", summary.skipReason},
                    });
            }
        }
        if (executionPlan.runMip && p2Gate.runMip) {
            SpMipRequest mipRequest;
            mipRequest.mode = globalFirstImprovement
                ? SpMipMode::first_improvement
                : SpMipMode::intensification;
            const double assignedP2MipTimeLimit =
                assignedStageLimit(
                    requestedP2MipTimeLimit,
                    "mip");
            if (assignedP2MipTimeLimit <= 0.0) {
                p2Gate.runMip = false;
                summary.mipSkippedByBound = true;
                summary.skipReason =
                    "outer_wall_budget_exhausted";
                spResult = p2LpResult;
                hgasp::RunLogger::event(
                    hgasp::LogLevel::info,
                    "sp",
                    "mip_skipped",
                    {},
                    {
                        {"iteration", data->iteration},
                        {"scope", scope},
                        {"call_id", spCallId},
                        {"reason", summary.skipReason},
                    });
                if (!executionPlan.runLp) {
                    summary.attempted = false;
                    finalizeSpCpu();
                    hgasp::RunLogger::status(
                        "sp",
                        "idle",
                        {
                            {"iteration", data->iteration},
                            {"scope", scope},
                            {"call_id", spCallId},
                            {"status", "SKIPPED"},
                        });
                    return summary;
                }
            }
            mipRequest.timeLimitSeconds =
                assignedP2MipTimeLimit;
            if (hardMandatory && data->hasHardTimeWindows() &&
                !options.improvementExtensionGateSeconds.has_value() &&
                data->spFinalGlobalPrimalIdleTimeLimit > 0.0) {
                mipRequest.primalIdleTimeLimitSeconds =
                    data->spFinalGlobalPrimalIdleTimeLimit;
            }
            if (options.improvementExtensionGateSeconds.has_value()) {
                const double remainingBaseWindow =
                    hgasp::remainingOuterSpBudget(
                        *options.improvementExtensionGateSeconds,
                        elapsedSpWallSeconds(outerWallStarted));
                const double assignedGate = std::min(
                    remainingBaseWindow,
                    std::max(0.0, assignedP2MipTimeLimit - Min));
                if (assignedGate > 0.0 &&
                    assignedGate < assignedP2MipTimeLimit) {
                    mipRequest.improvementExtensionGateSeconds =
                        assignedGate;
                }
            }
            if (!p2Gate.runMip) {
                // The common audited finish path retains the LP observation.
            } else {
            if (globalFirstImprovement &&
                std::isfinite(targetObjective)) {
                mipRequest.targetObjective =
                    targetObjective - Min;
            }
            hgasp::RunLogger::event(
                hgasp::LogLevel::info,
                "sp",
                "mip_requested",
                {},
                {
                    {"iteration", data->iteration},
                    {"scope", scope},
                    {"call_id", spCallId},
                    {"mode", executionMode},
                    {"target_objective",
                        mipRequest.targetObjective.value_or(Max)},
                    {"chosen_mip_time_limit_seconds",
                        mipRequest.timeLimitSeconds.value_or(
                            local
                                ? data->spLocalOuterTimeLimit
                                : data->spGlobalOuterTimeLimit)},
                    {"primal_idle_time_limit_seconds",
                        mipRequest.primalIdleTimeLimitSeconds.value_or(0.0)},
                    {"improvement_extension_gate_seconds",
                        mipRequest.improvementExtensionGateSeconds.value_or(
                            0.0)},
                });
            const double p2MipCpuStarted =
                data->elapsedCpuSeconds();
            spResult = solver.solveMip(
                problem,
                poolSnapshot,
                mipRequest,
                start);
            mipCpuSeconds = std::max(
                0.0,
                data->elapsedCpuSeconds() - p2MipCpuStarted);
            spBudgetLedger.recordPhase(
                mipBudgetPhase,
                mipCpuSeconds,
                hardMandatory);
            if (executionPlan.runLp) {
                attachP2LpObservation(
                    spResult,
                    p2LpResult,
                    !options.exploratoryConfiguration);
            }
            }
        }
        summary.lpAttempted = spResult.lpAttempted;
        summary.lpFeedbackAvailable =
            summary.lpFeedbackAvailable ||
            spResult.lpFeedbackAvailable;
        summary.mipAttempted = spResult.mipAttempted;
        summary.finalGateSatisfied =
            summary.rigorousLpNoImprovement ||
            summary.mipAttempted;
        // A mandatory final request is acknowledged after integer work
        // starts. For hard TW, a completed exact-
        // snapshot LP proof also satisfies the final gate without wasting a
        // 60-second integer call.
        summary.finalIntensificationSatisfied =
            hgasp::satisfiesFinalIntensification(
                hardMandatory,
                summary.mipAttempted,
                data->hasHardTimeWindows() &&
                    data->twProfileEarlyStopEnabled,
                summary.rigorousLpNoImprovement);
        if (summary.mipAttempted) {
            hgasp::RunLogger::event(
                hgasp::LogLevel::info,
                "sp",
                "mip_started",
                {},
                p2Fields);
        }
        if (!lpFeedbackRecorded ||
            !spResult.candidateSolutions.empty()) {
            recordSpFeedback(
                problem,
                poolSnapshot,
                spResult,
                spCallId,
                options.allowEphemeralFeedbackColumns,
                &summary.usefulColumnCredit);
        }

    summary.lpStatus = spResult.lpStatusName;
    summary.terminalStatus = spResult.statusName;
    std::size_t incumbentImprovements = 0U;
    double firstImprovementWallSeconds = 0.0;
    double lastImprovementWallSeconds = 0.0;
    for (const SpMipIncumbentObservation& observation :
         spResult.mipIncumbentTrace) {
        if (observation.improvesSubmittedStart) {
            if (incumbentImprovements == 0U) {
                firstImprovementWallSeconds =
                    observation.solverWallSeconds;
            }
            lastImprovementWallSeconds =
                observation.solverWallSeconds;
            ++incumbentImprovements;
        }
        hgasp::RunLogger::event(
            hgasp::LogLevel::info,
            "sp",
            "mip_incumbent_observed",
            {},
            {
                {"iteration", data->iteration},
                {"scope", scope},
                {"call_id", spCallId},
                {"ordinal", observation.ordinal},
                {"objective", observation.objective},
                {"best_bound", observation.bestBound},
                {"mip_solver_wall_seconds",
                    observation.solverWallSeconds},
                {"mip_process_cpu_seconds",
                    observation.processCpuSeconds},
                {"work", observation.work},
                {"nodes", observation.nodeCount},
                {"improves_submitted_start",
                    observation.improvesSubmittedStart},
            });
    }
    if (!spResult.mipIncumbentTraceDiagnostic.empty()) {
        hgasp::RunLogger::event(
            hgasp::LogLevel::warning,
            "sp",
            "mip_incumbent_trace_unavailable",
            spResult.mipIncumbentTraceDiagnostic,
            {
                {"iteration", data->iteration},
                {"scope", scope},
                {"call_id", spCallId},
            });
    }
    hgasp::EventFields solveFields{
        {"iteration", data->iteration},
        {"scope", scope},
        {"call_id", spCallId},
        {"status", spResult.statusName},
        {"columns", static_cast<unsigned long long>(
            solvedSnapshot.size())},
        {"solved_columns", static_cast<unsigned long long>(
            solvedSnapshot.size())},
        {"full_pool_columns",
            static_cast<unsigned long long>(
                currentPoolSize)},
        {"master_scope", hgasp::toString(masterScope)},
        {"lp_attempted", summary.lpAttempted},
        {"mip_attempted", spResult.mipAttempted},
        {"mip_start_requested_source",
            spResult.mipStartRequestedSource},
        {"mip_start_requested_routes",
            spResult.mipStartRequestedRouteCount},
        {"mip_start_required", spResult.mipStartRequired},
        {"mip_start_preferred_mapped",
            spResult.mipStartPreferredMapped},
        {"mip_start_submitted", spResult.mipStartApplied},
        {"mip_start_routes", spResult.mipStartRouteCount},
        {"mip_start_source",
            spResult.mipStartSource.empty()
                ? "none"
                : spResult.mipStartSource},
        {"mip_start_fallback_used",
            spResult.mipStartFallbackUsed},
        {"mip_incumbent_trace_available",
            spResult.mipIncumbentTraceAvailable},
        {"mip_incumbent_trace_truncated",
            spResult.mipIncumbentTraceTruncated},
        {"mip_incumbent_observations",
            static_cast<unsigned long long>(
                spResult.mipIncumbentTrace.size())},
        {"mip_incumbent_improvements_over_start",
            static_cast<unsigned long long>(incumbentImprovements)},
        {"mip_primal_idle_time_limit_seconds",
            spResult.mipPrimalIdleTimeLimitSeconds},
        {"mip_primal_idle_terminated",
            spResult.mipPrimalIdleTerminated},
        {"mip_improvement_extension_gate_seconds",
            spResult.mipImprovementExtensionGateSeconds},
        {"mip_no_improvement_gate_terminated",
            spResult.mipNoImprovementGateTerminated},
        {"mip_no_improvement_tail_wall_seconds",
            std::max(
                0.0,
                spResult.mipSolverWallTimeSeconds -
                    (incumbentImprovements > 0U
                         ? lastImprovementWallSeconds
                         : 0.0))},
        {"relative_lp_potential",
            summary.relativeLpPotential},
        {"observed_sp_cpu_seconds",
            spBudgetLedger.totalCpuSeconds()},
        {"observed_sp_cpu_share",
            spBudgetLedger.observedShare(
                data->elapsedCpuSeconds())},
        {"guardable_sp_cpu_seconds",
            spBudgetLedger.guardableCpuSeconds()},
        {"guardable_sp_cpu_share",
            spBudgetLedger.guardableShare(
                data->elapsedCpuSeconds())},
        {"mandatory_exempt_cpu_seconds",
            spBudgetLedger.mandatoryExemptCpuSeconds()},
        {"sp_cpu_share",
            spBudgetLedger.observedShare(
                data->elapsedCpuSeconds())},
        {"cpu_seconds", spResult.totalTimeSeconds},
        {"search_cpu_seconds",
            data->elapsedCpuSeconds()},
        {"wall_seconds", spResult.totalWallTimeSeconds},
        {"mip_solver_wall_seconds",
            spResult.mipSolverWallTimeSeconds},
        {"lp_columns_stored",
            spResult.persistentLpStoredColumns},
        {"lp_columns_active",
            spResult.persistentLpActiveColumns},
        {"lp_compacted",
            spResult.persistentLpCompacted},
    };
    if (spResult.mipStartObjective.has_value()) {
        solveFields.emplace_back(
            "mip_start_objective", *spResult.mipStartObjective);
    }
    if (incumbentImprovements > 0U) {
        solveFields.emplace_back(
            "mip_first_improvement_wall_seconds",
            firstImprovementWallSeconds);
        solveFields.emplace_back(
            "mip_last_improvement_wall_seconds",
            lastImprovementWallSeconds);
    }
    if (spResult.objective.has_value()) {
        solveFields.emplace_back(
            "objective", *spResult.objective);
    }
    if (spResult.mipGap.has_value()) {
        solveFields.emplace_back("gap", *spResult.mipGap);
    }
    hgasp::RunLogger::event(
        spResult.status == SpStatus::optimal
            ? hgasp::LogLevel::info
            : (isSupportedSpTerminalStatus(spResult.status)
                   ? hgasp::LogLevel::warning
                   : hgasp::LogLevel::error),
        "sp",
        "solve_finished",
        {},
        std::move(solveFields));
    hgasp::RunLogger::status(
        "sp",
        "idle",
        {
            {"iteration", data->iteration},
            {"scope", scope},
            {"call_id", spCallId},
            {"status", spResult.statusName},
        });

    std::cout << "[SP Result] status=" << spResult.statusName
              << " scope=" << scope
              << " solutions=" << spResult.solutionCount
              << " time_basis=process_cpu"
              << " solve=" << std::fixed << std::setprecision(3)
              << spResult.solveTimeSeconds << "s"
              << " validation="
              << spResult.validationTimeSeconds << "s"
              << " environment="
              << spResult.environmentTimeSeconds << "s"
              << " retry_sleep="
              << spResult.retrySleepTimeSeconds << "s"
              << " build=" << spResult.buildTimeSeconds << "s"
              << " optimize="
              << spResult.optimizeTimeSeconds << "s"
              << " postprocess="
              << spResult.postprocessTimeSeconds << "s"
              << " lp_build="
              << spResult.lpBuildTimeSeconds << "s"
              << " lp_optimize="
              << spResult.lpOptimizeTimeSeconds << "s"
              << " total=" << spResult.totalTimeSeconds << "s"
              << " solver_wall="
              << spResult.solverWallTimeSeconds << "s"
              << " retry_sleep_wall="
              << spResult.retrySleepWallTimeSeconds << "s"
              << " total_wall="
              << spResult.totalWallTimeSeconds << "s"
              << " env_attempts="
              << spResult.environmentStartAttempts
              << " env_retries="
              << spResult.environmentRetryCount
              << " env_reused="
              << (spResult.environmentReused ? "true" : "false")
              << " variables=" << spResult.modelVariables
              << " constraints=" << spResult.modelConstraints
              << " nonzeros=" << spResult.modelNonzeros
              // Gurobi exposes whether Start attributes were submitted, not
              // an authoritative "accepted by presolve" acknowledgement.
              << " mip_start_submitted="
              << (spResult.mipStartApplied ? "true" : "false")
              << " mip_start_routes="
              << spResult.mipStartRouteCount
              << " mip_start_source="
              << (spResult.mipStartSource.empty()
                      ? "none"
                      : spResult.mipStartSource)
              << " mip_start_preferred_mapped="
              << (spResult.mipStartPreferredMapped
                      ? "true"
                      : "false")
              << " mip_start_fallback="
              << (spResult.mipStartFallbackUsed
                      ? "true"
                      : "false")
              << " mip_incumbent_observations="
              << spResult.mipIncumbentTrace.size()
              << " mip_incumbent_improvements_over_start="
              << incumbentImprovements
              << " lp_feedback="
              << (spResult.lpFeedbackAvailable ? "true" : "false")
              << " lp_persistent_reused="
              << (spResult.persistentLpReused ? "true" : "false")
              << " lp_columns_added="
              << spResult.persistentLpAddedColumns
              << " lp_columns_reactivated="
              << spResult.persistentLpReactivatedColumns
              << " lp_columns_disabled="
              << spResult.persistentLpDisabledColumns
              << " lp_columns_stored="
              << spResult.persistentLpStoredColumns
              << " lp_columns_active="
              << spResult.persistentLpActiveColumns
              << " lp_compacted="
              << (spResult.persistentLpCompacted
                      ? "true"
                      : "false")
              << " lp_columns_before_compaction="
              << spResult.persistentLpColumnsBeforeCompaction
              << " pool_candidates="
              << spResult.candidateSolutions.size();
    if (spResult.lpObjective.has_value()) {
        std::cout << " lp_objective=" << *spResult.lpObjective;
    }
    if (spResult.objective.has_value()) {
        std::cout << " objective=" << *spResult.objective;
    }
    if (spResult.bestBound.has_value()) {
        std::cout << " best_bound=" << *spResult.bestBound;
    }
    if (spResult.mipGap.has_value()) {
        std::cout << " gap=" << *spResult.mipGap;
    }
    if (spResult.work.has_value()) {
        std::cout << " work=" << *spResult.work;
    }
    if (spResult.nodeCount.has_value()) {
        std::cout << " nodes=" << *spResult.nodeCount;
    }
    if (!spResult.diagnostic.empty()) {
        std::cout << " diagnostic=\"" << spResult.diagnostic
                  << "\"";
    }
    std::cout << std::endl;

    if (!isSupportedSpTerminalStatus(spResult.status)) {
        if (local && options.deferUnsupportedSolverOutcome) {
            summary.deferredEnvironmentFailure =
                spResult.status == SpStatus::error &&
                spResult.environmentStartAttempts > 0 &&
                !spResult.lpAttempted &&
                !spResult.mipAttempted;
            summary.deferredUnsupportedStatus =
                !summary.deferredEnvironmentFailure;
            if (summary.skipReason.empty()) {
                summary.skipReason =
                    summary.deferredEnvironmentFailure
                        ? "local_sp_environment_failure"
                        : "unsupported_local_solver_status";
            }
            finalizeSpCpu();
            return summary;
        }
        throw std::runtime_error(
            "fatal or unsupported SP outcome (" +
            spResult.statusName + "): " +
            (spResult.diagnostic.empty()
                 ? std::string("no diagnostic was provided")
                 : spResult.diagnostic));
    }

    if (currentPoolSize < 10) {
        std::cout
            << "[SP Warning] RoutePool is too small ("
            << currentPoolSize
            << "). Gurobi might not find any improvement."
            << std::endl;
    }

    if (summary.mipSkippedByBound &&
        summary.rigorousLpNoImprovement) {
        finalizeSpCpu();
        return summary;
    }

    const bool mipSkippedByCpuBudget =
        summary.skipReason == "sp_cpu_share_guard";
    if (hgasp::isNonfatalBudgetedP2LpOnlyCompletion(
            hardMandatory,
            summary.lpFeedbackAvailable,
            summary.mipAttempted,
            summary.mipSkippedByBound && mipSkippedByCpuBudget)) {
        finalizeSpCpu();
        return summary;
    }

    const bool hasIncumbent =
        spResult.hasFeasibleSolution();
    if (spResult.status == SpStatus::infeasible &&
        hasIncumbent) {
        throw std::runtime_error(
            "fatal inconsistent SP outcome: INFEASIBLE was "
            "reported together with a validated incumbent");
    }

    if (!hasIncumbent) {
        if (!mayContinueWithoutSpIncumbent(
                spResult.status)) {
            throw std::runtime_error(
                "fatal SP outcome " + spResult.statusName +
                " did not provide a validated incumbent");
        }
        std::cout
            << "[SP Info] Whitelisted status "
            << spResult.statusName
            << " returned no feasible SP incumbent; "
               "the hybrid search may continue.\n"
            << std::endl;
        finalizeSpCpu();
        return summary;
    }

    std::cout
        << "[SP Success] Gurobi found a valid configuration!"
        << std::endl;

    try {
        const std::vector<std::size_t> diverseCandidates =
            hgasp::selectDiverseSpSolutions(
                spResult.candidateSolutions,
                options.maximumInjections.value_or(
                    static_cast<std::size_t>(
                        hardMandatory
                            ? data->spFinalGlobalInjectionCount
                            : (local
                                   ? data->spOnlineLocalInjectionCount
                                   : data->spOnlineGlobalInjectionCount))));
        if (diverseCandidates.empty()) {
            throw std::logic_error(
                "validated SP incumbent has no injectable pool member");
        }

        std::vector<StandaloneRoute> optimalCombination;
        const SpSolution& primary =
            spResult.candidateSolutions[
                diverseCandidates.front()];
        optimalCombination.reserve(
            primary.selectedRouteIndices.size());
        for (const std::size_t routeIndex :
             primary.selectedRouteIndices) {
            if (routeIndex >= solvedSnapshot.size()) {
                throw std::out_of_range(
                    "SP returned a route index outside the solved "
                    "pool snapshot");
            }
            optimalCombination.push_back(
                solvedSnapshot[routeIndex]);
        }
        routePool->setProtectedRoutes(
            RouteProtection::LastSpIncumbent,
            optimalCombination);
        routePool->pruneIfNeeded();

        double bestAttributedPostObjective = Max;
        double bestAttributedDirectImprovement = 0.0;
        double bestAttributedPostLsImprovement = 0.0;
        for (const std::size_t candidateIndex :
             diverseCandidates) {
            ++summary.injectionAttempts;
            const SpSolution& candidate =
                spResult.candidateSolutions[candidateIndex];
            std::vector<StandaloneRoute> combination;
            combination.reserve(
                candidate.selectedRouteIndices.size());
            for (const std::size_t routeIndex :
                 candidate.selectedRouteIndices) {
                if (routeIndex >= solvedSnapshot.size()) {
                    throw std::out_of_range(
                        "SP solution-pool member references a route "
                        "outside the solved snapshot");
                }
                combination.push_back(
                    solvedSnapshot[routeIndex]);
            }

            std::unique_ptr<Individual> rawSpIndiv(new Individual());
            rawSpIndiv->define(data);
            rawSpIndiv->rebuildFromSP(combination);
            rawSpIndiv->decideFeasible();
            if (!rawSpIndiv->isFeasible) {
                throw std::logic_error(
                    "validated SP combination rebuilt infeasibly");
            }
            const double spRawObjective = rawSpIndiv->fit;
            const auto individualConfiguration =
                [this](const Individual& individual) {
                    std::vector<int> depots;
                    for (int depot = 0; depot < data->numL; ++depot) {
                        if (individual.location[depot].used) {
                            depots.push_back(depot);
                        }
                    }
                    return hgasp::makeDepotConfigurationKey(
                        std::move(depots));
                };
            const hgasp::DepotConfigurationKey rawSpConfiguration =
                individualConfiguration(*rawSpIndiv);
            if (options.exploratoryConfiguration) {
                auto existing = std::find_if(
                    summary.exploratoryFounders.begin(),
                    summary.exploratoryFounders.end(),
                    [&](const RouteGenerator::ScheduledConfigurationFounder&
                            founder) {
                        return founder.configuration == rawSpConfiguration;
                    });
                if (existing == summary.exploratoryFounders.end()) {
                    summary.exploratoryFounders.push_back({
                        rawSpConfiguration,
                        combination,
                        spRawObjective,
                    });
                } else if (spRawObjective + Min < existing->objective) {
                    existing->routes = combination;
                    existing->objective = spRawObjective;
                }
            }
            const std::string evidenceUnitPrefix = "sp:" + spCallId +
                ":candidate:" + std::to_string(candidateIndex);

            if (configuration.has_value() &&
                rawSpConfiguration != *configuration) {
                throw std::logic_error(
                    "raw Local-SP result changed the frozen master "
                    "configuration");
            }

            std::unique_ptr<Individual> postLsIndiv(new Individual());
            postLsIndiv->define(data);
            routeGenerator->popman->copyAllData(
                postLsIndiv.get(), rawSpIndiv.get());
            routeGenerator->ls->local_search_run(
                postLsIndiv.get(),
                data->penaltyDepot,
                data->penaltyVeh);
            postLsIndiv->decideFeasible();
            if (!postLsIndiv->isFeasible) {
                throw std::logic_error(
                    "post-SP local search produced an infeasible "
                    "candidate");
            }
            const hgasp::DepotConfigurationKey postLsConfiguration =
                individualConfiguration(*postLsIndiv);
            const bool postLsConfigurationChanged =
                postLsConfiguration != rawSpConfiguration;
            const bool postLsWorsened =
                postLsIndiv->fit > spRawObjective + Min;
            const double postSpLsObjective = postLsIndiv->fit;

            const bool rawImprovesTarget =
                spRawObjective + Min < targetObjective;
            const bool rawExploresConfiguration =
                startConfiguration.has_value() &&
                rawSpConfiguration != *startConfiguration;
            const bool rawIsUseful =
                rawImprovesTarget || rawExploresConfiguration;
            const bool postLsImprovesTarget =
                postSpLsObjective + Min < targetObjective;
            const bool postLsExploresConfiguration =
                startConfiguration.has_value() &&
                postLsConfiguration != *startConfiguration;
            const bool postLsIsUseful =
                postLsImprovesTarget || postLsExploresConfiguration;

            SpInjectionResult rawInjection;
            if (rawIsUseful) {
                rawInjection = routeGenerator->injectIncumbent(
                    std::move(rawSpIndiv),
                    routePool.get(),
                    spCallId,
                    options.populationAdmissionTarget,
                    0U,
                    evidenceUnitPrefix + ":raw");
            }
            if (!local && rawInjection.admitted()) {
                routeGenerator->recordVerifiedGlobalInjection(
                    rawInjection.populationIndex);
            }

            SpInjectionResult postLsInjection;
            const bool submitPostLs =
                postLsIsUseful &&
                (postLsConfigurationChanged || !postLsWorsened);
            if (submitPostLs) {
                postLsInjection = routeGenerator
                    ->injectPostSpLocalSearchCandidate(
                        *postLsIndiv,
                        rawSpConfiguration,
                        routePool.get(),
                        spCallId,
                        postLsConfigurationChanged
                            ? std::nullopt
                            : options.populationAdmissionTarget,
                        evidenceUnitPrefix + ":ls");
            }
            SpInjectionResult injection = rawInjection;
            if (!rawInjection.admitted() && postLsInjection.admitted()) {
                injection.disposition = postLsInjection.disposition;
                injection.populationIndex =
                    postLsInjection.populationIndex;
            }
            injection.incumbentAccepted =
                rawInjection.incumbentAccepted ||
                postLsInjection.incumbentAccepted;
            if (injection.admitted()) {
                ++summary.admittedSolutions;
            } else {
                ++summary.rejectedSolutions;
            }
            if (injection.admitted() ||
                injection.incumbentAccepted) {
                double attributedObjective = Max;
                if (rawInjection.admitted() ||
                    rawInjection.incumbentAccepted) {
                    attributedObjective = spRawObjective;
                }
                if (postLsInjection.admitted() ||
                    postLsInjection.incumbentAccepted) {
                    attributedObjective = std::min(
                        attributedObjective, postSpLsObjective);
                }
                if (attributedObjective + Min <
                    bestAttributedPostObjective) {
                    bestAttributedPostObjective =
                        attributedObjective;
                    bestAttributedDirectImprovement =
                        std::max(
                            0.0,
                            targetObjective - spRawObjective);
                    const double totalTargetImprovement =
                        std::max(
                            0.0, targetObjective - attributedObjective);
                    bestAttributedPostLsImprovement =
                        std::max(
                            0.0,
                            totalTargetImprovement -
                                bestAttributedDirectImprovement);
                }
            }
            const hgasp::SpContribution contribution =
                hgasp::classifySpContribution(
                    beforeObjective,
                    spRawObjective,
                    postSpLsObjective,
                    injection.admitted());
            hgasp::RunLogger::event(
                hgasp::LogLevel::info,
                "sp",
                "contribution_attributed",
                {},
                {
                    {"iteration", data->iteration},
                    {"scope", scope},
                    {"call_id", spCallId},
                    {"global_before", beforeObjective},
                    {"sp_raw_objective", spRawObjective},
                    {"post_sp_ls_objective", postSpLsObjective},
                    {"incumbent_accepted",
                        injection.incumbentAccepted},
                    {"population_admitted", injection.admitted()},
                    {"raw_useful", rawIsUseful},
                    {"post_sp_ls_useful", postLsIsUseful},
                    {"contribution_class",
                        hgasp::toString(contribution)},
                });
        }
        std::stable_sort(
            summary.exploratoryFounders.begin(),
            summary.exploratoryFounders.end(),
            [](const RouteGenerator::ScheduledConfigurationFounder& left,
               const RouteGenerator::ScheduledConfigurationFounder& right) {
                return left.objective < right.objective;
            });
        summary.directSpObjectiveImprovement =
            bestAttributedDirectImprovement;
        summary.postSpLocalSearchImprovement =
            bestAttributedPostLsImprovement;
        std::cout
            << "[SP Injection] scope=" << scope
            << " validated_candidates="
            << spResult.candidateSolutions.size()
            << " attempted=" << summary.injectionAttempts
            << " admitted=" << summary.admittedSolutions
            << " rejected=" << summary.rejectedSolutions
            << std::endl;
        hgasp::RunLogger::event(
            summary.rejectedSolutions == 0U
                ? hgasp::LogLevel::info
                : hgasp::LogLevel::warning,
            "sp",
            "injection_finished",
            {},
            {
                {"iteration", data->iteration},
                {"scope", scope},
                {"call_id", spCallId},
                {"attempted",
                    static_cast<unsigned long long>(
                        summary.injectionAttempts)},
                {"admitted",
                    static_cast<unsigned long long>(
                        summary.admittedSolutions)},
                {"rejected",
                    static_cast<unsigned long long>(
                        summary.rejectedSolutions)},
            });
    } catch (const std::exception& error) {
        throw std::runtime_error(
            std::string(
                "SP incumbent reconstruction or injection failed: ") +
            error.what());
    }
    summary.incumbent = true;
    summary.terminalStatus = spResult.statusName;
    const double afterObjective = restartBestFitness();
    summary.objectiveImprovement =
        std::max(0.0, beforeObjective - afterObjective);
    double targetAfter = afterObjective;
    if (configuration.has_value()) {
        targetAfter = targetObjective;
        std::size_t firstPopulation = 0U;
        std::size_t lastPopulation =
            routeGenerator->popman->popula.size();
        if (options.populationAdmissionTarget.has_value()) {
            const int target = *options.populationAdmissionTarget;
            if (target < 0 ||
                static_cast<std::size_t>(target) >= lastPopulation) {
                throw std::logic_error(
                    "Local-SP population target disappeared before "
                    "outcome attribution");
            }
            firstPopulation = static_cast<std::size_t>(target);
            lastPopulation = firstPopulation + 1U;
        }
        for (std::size_t populationIndex = firstPopulation;
             populationIndex < lastPopulation;
             ++populationIndex) {
            const auto& population =
                routeGenerator->popman->popula[populationIndex];
            for (const Individual* individual : population) {
                std::vector<int> openDepots;
                for (int depot = 0;
                     depot < data->numL;
                     ++depot) {
                    if (individual->location[depot].used) {
                        openDepots.push_back(depot);
                    }
                }
                if (hgasp::makeDepotConfigurationKey(
                        std::move(openDepots)) ==
                    *configuration) {
                    targetAfter =
                        std::min(targetAfter, individual->fit);
                }
            }
        }
    }
    summary.targetObjectiveImprovement =
        std::max(0.0, targetObjective - targetAfter);
    summary.directSpObjectiveImprovement = std::min(
        summary.directSpObjectiveImprovement,
        summary.targetObjectiveImprovement);
    summary.postSpLocalSearchImprovement = std::max(
        0.0,
        summary.targetObjectiveImprovement -
            summary.directSpObjectiveImprovement);
    summary.improvedIncumbent =
        summary.objectiveImprovement > Min;
    finalizeSpCpu();
    return summary;
}

double AlgorithmOrchestrator::restartBestFitness() const noexcept {
    return routeGenerator->popman->bestIndivRestart->fit;
}

hgasp::PopulationLocalSpObservation
AlgorithmOrchestrator::buildV114LocalSpObservation(
        const RouteGenerator::StaticPopulationRuntimeView& view) {
    hgasp::PopulationLocalSpObservation observation;
    observation.snapshot.population = view.localSpToken;
    observation.snapshot.configuration = view.configuration;
    observation.snapshot.populationBestObjective = view.bestObjective;
    const auto coverage =
        routePool->getConfigurationCoverageSummary(
            view.configuration);
    observation.snapshot.configurationAppendCount =
        routePool->getConfigurationMasterAppendCount(
            view.configuration);
    observation.snapshot.configurationRepresentativeReplacementCount =
        routePool->getConfigurationMasterRepresentativeReplacementCount(
            view.configuration);
    observation.snapshot.configurationRepresentativeReplacementGain =
        routePool->getConfigurationMasterRepresentativeReplacementCostGain(
            view.configuration);
    observation.snapshot.rawMasterVersion =
        routePool->getConfigurationMasterVersion(view.configuration);
    observation.snapshot.eligibleRouteCoverageSignature =
        coverage.signature;
    observation.snapshot.eligibleRouteCoverageVersion =
        coverage.version;
    observation.currentAttempt = static_cast<std::uint64_t>(
        std::max(0, data->iteration));
    observation.createdAttempt = view.createdAttempt;
    observation.lastPopulationBestImprovementAttempt =
        std::max(
            view.createdAttempt,
            view.lastBestImprovementAttempt);
    observation.verifiedGlobalInjectionCount =
        view.verifiedGlobalInjectionCount;
    observation.populationActive = view.active;
    observation.completeCustomerCoverage =
        coverage.completeCustomerCoverage;
    return observation;
}

void AlgorithmOrchestrator::initializeV114LocalSpBaselines() {
    if (!data->staticHgampPopulationsEnabled() ||
        !data->populationLocalSpEnabled ||
        !routeGenerator->staticPopulationsMaterialized()) {
        return;
    }
    hgasp::LocalSpFamilyDecision blocked;
    blocked.eligible = false;
    blocked.reason =
        hgasp::LocalSpFamilyGateReason::MinimumInterval;
    for (const auto& view :
         routeGenerator->staticPopulationRuntimeViews()) {
        if (!view.active || view.unrestricted) {
            continue;
        }
        populationLocalSpController.decide(
            buildV114LocalSpObservation(view), blocked);
    }
    v114LocalSpBaselinesInitialized = true;
}

hgasp::PopulationLocalSpOutcome
AlgorithmOrchestrator::classifyV114LocalSpOutcome(
        const SpExecutionSummary& summary) const {
    hgasp::PopulationLocalSpOutcome outcome;
    outcome.outerCpuSeconds = std::max(0.0, summary.cpuSeconds);
    if (summary.targetObjectiveImprovement > Min) {
        outcome.kind =
            hgasp::PopulationLocalSpOutcomeKind::Improved;
        outcome.directObjectiveImprovement =
            summary.directSpObjectiveImprovement;
        outcome.postLocalSearchImprovement =
            summary.postSpLocalSearchImprovement;
    } else if (summary.rigorousLpNoImprovement) {
        outcome.kind =
            hgasp::PopulationLocalSpOutcomeKind::ExhaustedLpBound;
    } else if (summary.deferredEnvironmentFailure) {
        outcome.kind =
            hgasp::PopulationLocalSpOutcomeKind::
                DeferredEnvironmentFailure;
    } else if (summary.deferredUnsupportedStatus) {
        outcome.kind =
            hgasp::PopulationLocalSpOutcomeKind::
                DeferredUnsupportedStatus;
    } else if (!summary.attempted &&
               summary.skipReason == "sp_cpu_share_guard") {
        outcome.kind =
            hgasp::PopulationLocalSpOutcomeKind::DeferredCpuBudget;
    } else if (!summary.attempted) {
        outcome.kind =
            hgasp::PopulationLocalSpOutcomeKind::
                DeferredUnsupportedStatus;
    } else if (summary.terminalStatus == "TIME_LIMIT") {
        outcome.kind =
            hgasp::PopulationLocalSpOutcomeKind::
                ExhaustedTimeLimitNoImprovement;
    } else if (summary.terminalStatus == "OPTIMAL") {
        outcome.kind =
            hgasp::PopulationLocalSpOutcomeKind::
                ExhaustedMipOptimalNoImprovement;
    } else {
        outcome.kind =
            hgasp::PopulationLocalSpOutcomeKind::
                ExhaustedTerminalNoImprovement;
    }
    return outcome;
}

void AlgorithmOrchestrator::dispatchV114PopulationLocalSp() {
    if (!data->populationLocalSpEnabled ||
        !routeGenerator->staticPopulationsMaterialized()) {
        return;
    }
    if (!v114LocalSpBaselinesInitialized) {
        initializeV114LocalSpBaselines();
    }
    const bool roleAwareProfile =
        hgasp::isSetSProductionSearchProfile(data->parameters.profile) ||
        hgasp::isSetFProductionSearchProfile(data->parameters.profile);
    const std::size_t roleAwareFamilyCallLimit =
        hgasp::isSetFProductionSearchProfile(data->parameters.profile)
            ? 2U
            : 4U;
    if (roleAwareProfile &&
        roleAwarePopulationLocalSpCalls >= roleAwareFamilyCallLimit) {
        populationLocalSpDecisionLogSummary.recordSuppressedReason(
            "role_aware_family_call_limit");
        return;
    }
    const int roleAwareReductionEpoch = globalSpPhasePolicy != nullptr &&
            globalSpPhasePolicy->completed(
                hgasp::GlobalSpPhaseAction::Reduction2)
        ? 2
        : (globalSpPhasePolicy != nullptr &&
           globalSpPhasePolicy->completed(
               hgasp::GlobalSpPhaseAction::Reduction1)
              ? 1
              : 0);
    const std::uint64_t attempt = static_cast<std::uint64_t>(
        std::max(0, data->iteration));
    const double expectedCpu =
        static_cast<double>(std::max(1, data->spThreads)) *
        (data->populationLocalSpLpTimeLimitSeconds +
         data->populationLocalSpMipTimeLimitSeconds);
    const hgasp::LocalSpFamilyDecision familyDecision =
        localSpFamilyBudget.decide({
            attempt,
            data->elapsedCpuSeconds(),
            expectedCpu,
        });
    if (!familyDecision.eligible) {
        populationLocalSpDecisionLogSummary.recordSuppressedReason(
            hgasp::toString(familyDecision.reason));
        return;
    }

    std::vector<hgasp::PopulationLocalSpDecision> decisions;
    for (const auto& view :
         routeGenerator->staticPopulationRuntimeViews()) {
        if (!view.active || view.unrestricted) {
            populationLocalSpDecisionLogSummary
                .recordSuppressedReason("population_inactive");
            continue;
        }
        if (roleAwareProfile) {
            const auto lastEpoch = roleAwareLocalSpLastEpoch.find(
                view.localSpToken);
            if (lastEpoch != roleAwareLocalSpLastEpoch.end() &&
                lastEpoch->second == roleAwareReductionEpoch) {
                populationLocalSpDecisionLogSummary
                    .recordSuppressedReason("population_epoch_consumed");
                continue;
            }
        }
        if (attempt < view.createdAttempt ||
            attempt - view.createdAttempt <
                static_cast<std::uint64_t>(
                    data->populationLocalSpMinimumAgeAttempts)) {
            populationLocalSpDecisionLogSummary
                .recordSuppressedReason("population_too_young");
            continue;
        }
        if (attempt < view.lastBestImprovementAttempt ||
            attempt - view.lastBestImprovementAttempt <
                static_cast<std::uint64_t>(
                    data->populationLocalSpStagnationAttempts)) {
            populationLocalSpDecisionLogSummary
                .recordSuppressedReason("population_not_stagnant");
            continue;
        }
        const hgasp::PopulationLocalSpDecision decision =
            populationLocalSpController.decide(
                buildV114LocalSpObservation(view),
                familyDecision);
        decisions.push_back(decision);
        const bool logEligibleDecision =
            populationLocalSpDecisionLogSummary.recordDecision(
                decision.eligible, decision.reason);
        if (logEligibleDecision) {
            hgasp::RunLogger::event(
                hgasp::LogLevel::info,
                "sp",
                "population_local_sp_eligible",
                {},
                {
                    {"iteration", data->iteration},
                    {"population", static_cast<unsigned long long>(
                        decision.snapshot.population.slot)},
                    {"generation", static_cast<unsigned long long>(
                        decision.snapshot.population.generation)},
                    {"configuration",
                        configurationLabel(
                            decision.snapshot.configuration)},
                    {"stagnation", static_cast<unsigned long long>(
                        decision.stagnationAttempts)},
                    {"new_customer_sets", static_cast<unsigned long long>(
                        decision.newCustomerSets)},
                    {"replacement_gain_ratio",
                        decision.replacementGainRatio},
                    {"snapshot_id", static_cast<unsigned long long>(
                        decision.snapshotId)},
                });
        }
        const auto state = populationLocalSpController.state(
            decision.snapshot.population);
        if (state.has_value() && state->reactivationCount >
                loggedLocalSpReactivations[
                    decision.snapshot.population]) {
            loggedLocalSpReactivations[
                decision.snapshot.population] =
                state->reactivationCount;
            hgasp::RunLogger::event(
                hgasp::LogLevel::info,
                "sp",
                "population_local_sp_reactivation",
                {},
                {
                    {"iteration", data->iteration},
                    {"population", static_cast<unsigned long long>(
                        decision.snapshot.population.slot)},
                    {"generation", static_cast<unsigned long long>(
                        decision.snapshot.population.generation)},
                    {"reason", state->lastReactivationReason},
                });
        }
    }
    const auto selected =
        populationLocalSpController.selectCandidate(decisions);
    if (!selected.has_value()) {
        return;
    }
    populationLocalSpDecisionLogSummary.recordSelected();
    hgasp::RunLogger::event(
        hgasp::LogLevel::info,
        "sp",
        "population_local_sp_selected",
        {},
        {
            {"iteration", data->iteration},
            {"population", static_cast<unsigned long long>(
                selected->snapshot.population.slot)},
            {"generation", static_cast<unsigned long long>(
                selected->snapshot.population.generation)},
            {"configuration",
                configurationLabel(selected->snapshot.configuration)},
            {"snapshot_id", static_cast<unsigned long long>(
                selected->snapshotId)},
        });
    populationLocalSpController.start(*selected);
    SpExecutionOptions options;
    // Role-aware production profiles have no dual-guidance consumer on
    // population-local rescue calls.
    // Go directly to primal MIP work; older profiles retain their audited LP
    // bound gate and trajectory.
    options.forceLp = !roleAwareProfile;
    options.firstImprovement = true;
    options.forceP2Execution = true;
    options.deferUnsupportedSolverOutcome = true;
    options.lpTimeLimitSeconds = selected->lpTimeLimitSeconds;
    options.mipTimeLimitSeconds = selected->mipTimeLimitSeconds;
    options.maximumInjections =
        static_cast<std::size_t>(
            data->populationLocalSpInjectionCount);
    options.populationAdmissionTarget =
        static_cast<int>(selected->snapshot.population.slot);
    const SpExecutionSummary summary =
        executeConfigurationLocalSp(
            static_cast<int>(
                selected->snapshot.population.slot),
            options);
    if (roleAwareProfile && summary.attempted) {
        roleAwareLocalSpLastEpoch[selected->snapshot.population] =
            roleAwareReductionEpoch;
        ++roleAwarePopulationLocalSpCalls;
    }
    const hgasp::PopulationLocalSpOutcome outcome =
        classifyV114LocalSpOutcome(summary);
    // Execution is synchronous and no other search action can change the
    // population between dispatch and solver return.  Completing against the
    // exact frozen snapshot therefore records the view that was solved; the
    // admission performed by executeConfigurationLocalSp belongs to the next
    // controller observation.
    const hgasp::PopulationLocalSpCompletion completion =
        populationLocalSpController.complete(
            selected->snapshot.population,
            selected->snapshot,
            outcome);
    localSpFamilyBudget.recordCall(attempt, summary.cpuSeconds);
    if (summary.attempted) {
        hgasp::recordV114ComponentActivation(
            hgasp::V114Component::PopulationLocalSpRescue);
    }
    hgasp::EventFields outcomeFields{
            {"iteration", data->iteration},
            {"population", static_cast<unsigned long long>(
                selected->snapshot.population.slot)},
            {"generation", static_cast<unsigned long long>(
                selected->snapshot.population.generation)},
            {"snapshot_id", static_cast<unsigned long long>(
                selected->snapshotId)},
            {"status", hgasp::toString(
                completion.effectiveOutcome)},
            {"lp_status", summary.lpStatus},
            {"mip_status",
                summary.mipAttempted
                    ? summary.terminalStatus
                    : "NOT_RUN"},
            {"mip_attempted", summary.mipAttempted},
            {"exhausted",
                completion.phase ==
                    hgasp::PopulationLocalSpPhase::Exhausted},
            {"deferred",
                completion.phase ==
                    hgasp::PopulationLocalSpPhase::Active &&
                !completion.snapshotConsumed},
            {"direct_improvement",
                outcome.directObjectiveImprovement},
            {"post_ls_improvement",
                outcome.postLocalSearchImprovement},
            {"global_incumbent_improvement",
                summary.improvedIncumbent},
            {"stale_population", completion.stalePopulation},
            {"stale_snapshot", completion.staleSnapshot},
            {"cpu_seconds", outcome.outerCpuSeconds},
        };
    if (summary.lpObjective < Max / 2.0) {
        outcomeFields.emplace_back(
            "lp_objective", summary.lpObjective);
    }
    hgasp::RunLogger::event(
        hgasp::LogLevel::info,
        "sp",
        "population_local_sp_outcome",
        {},
        std::move(outcomeFields));
}

void AlgorithmOrchestrator::emitPopulationLocalSpDecisionSummary(
        const bool final) noexcept {
    if (!data->populationLocalSpEnabled) {
        return;
    }
    try {
        hgasp::RunLogger::event(
            hgasp::LogLevel::info,
            "sp",
            "population_local_sp_decision_summary",
            {},
            {
                {"iteration", data->iteration},
                {"final", final},
                {"controller_decisions", static_cast<unsigned long long>(
                    populationLocalSpDecisionLogSummary
                        .controllerDecisions)},
                {"not_stagnant", static_cast<unsigned long long>(
                    populationLocalSpDecisionLogSummary.notStagnant)},
                {"snapshot_consumed", static_cast<unsigned long long>(
                    populationLocalSpDecisionLogSummary.snapshotConsumed)},
                {"insufficient_change", static_cast<unsigned long long>(
                    populationLocalSpDecisionLogSummary.insufficientChange)},
                {"family_interval", static_cast<unsigned long long>(
                    populationLocalSpDecisionLogSummary.familyInterval)},
                {"cpu_share", static_cast<unsigned long long>(
                    populationLocalSpDecisionLogSummary.cpuShare)},
                {"incomplete_coverage", static_cast<unsigned long long>(
                    populationLocalSpDecisionLogSummary
                        .incompleteCoverage)},
                {"eligible", static_cast<unsigned long long>(
                    populationLocalSpDecisionLogSummary.eligible)},
                {"selected", static_cast<unsigned long long>(
                    populationLocalSpDecisionLogSummary.selected)},
                {"other_reject", static_cast<unsigned long long>(
                    populationLocalSpDecisionLogSummary.otherReject)},
            });
    } catch (...) {
        // Telemetry must never change Local-SP control flow.
    }
}

void AlgorithmOrchestrator::markFinalGlobalCompleted(
        const SpExecutionSummary& summary,
        const char* const source) {
    if (!summary.attempted ||
        !summary.finalGateSatisfied ||
        !summary.finalIntensificationSatisfied) {
        throw std::logic_error(
            "cannot mark Final Global complete before its mandatory "
            "execution contract is satisfied");
    }
    const bool alreadyCompleted = standaloneFinalGlobalCompleted;
    standaloneFinalGlobalCompleted = true;
    lastCompletedFinalGlobalGate = summary;
    adaptiveSpController.markFinalIntensificationSatisfied();
    hgasp::RunLogger::event(
        hgasp::LogLevel::info,
        "sp",
        "final_global_completion_synchronized",
        {},
        {
            {"iteration", data->iteration},
            {"source", source == nullptr ? "unknown" : source},
            {"already_completed", alreadyCompleted},
            {"adaptive_final_pending",
                adaptiveSpController.finalIntensificationPending()},
        });
}

std::optional<AlgorithmOrchestrator::SpExecutionSummary>
AlgorithmOrchestrator::dispatchV114PhaseActions(
        const bool finalRequested) {
    std::optional<SpExecutionSummary> finalSummary;
    if (globalSpPhasePolicy == nullptr) {
        return finalSummary;
    }
    const bool staticPopulationsAvailable =
        routeGenerator->staticPopulationsMaterialized();
    if (!staticPopulationsAvailable && !finalRequested) {
        return finalSummary;
    }
    for (;;) {
        const hgasp::GlobalSpPhaseDecision decision =
            globalSpPhasePolicy->claimNext(
                static_cast<std::uint64_t>(
                    std::max(0, data->iteration)),
                finalRequested,
                staticPopulationsAvailable);
        if (!decision.claimed) {
            break;
        }
        if (decision.action ==
                hgasp::GlobalSpPhaseAction::Reduction1 ||
            decision.action ==
                hgasp::GlobalSpPhaseAction::Reduction2) {
            const int phase = decision.action ==
                    hgasp::GlobalSpPhaseAction::Reduction1
                ? 1
                : 2;
            const auto retired =
                routeGenerator->reduceStaticPopulations(phase);
            for (const auto& token : retired) {
                populationLocalSpController.retire(token);
                roleAwareLocalSpLastEpoch.erase(token);
            }
            if (pendingConfigurationRotationFounders.has_value()) {
                const auto rotation =
                    routeGenerator->rotateConfigurationFromFounders(
                        *pendingConfigurationRotationFounders,
                        routePool.get(),
                        phase == 1
                            ? "global_sp_checkpoint_50"
                            : "global_sp_checkpoint_75");
                pendingConfigurationRotationFounders.reset();
                if (rotation.replacement.has_value()) {
                    populationLocalSpController.retire(
                        rotation.replacement->retiredToken);
                    roleAwareLocalSpLastEpoch.erase(
                        rotation.replacement->retiredToken);
                }
            }
            refreshRouteProtections(true);
            globalSpPhasePolicy->complete(
                decision.action,
                hgasp::GlobalSpPhaseCompletion::Completed);
            continue;
        }
        if (!decision.executionRequired) {
            globalSpPhasePolicy->complete(
                decision.action,
                hgasp::GlobalSpPhaseCompletion::SkippedDisabled);
            continue;
        }

        if (decision.action ==
                hgasp::GlobalSpPhaseAction::PreReductionGlobal2 &&
            firstExploratoryGlobalFeedback.has_value() &&
            !hgasp::shouldExecuteSecondExploratoryGlobal(
                *firstExploratoryGlobalFeedback)) {
            const auto feedback = *firstExploratoryGlobalFeedback;
            hgasp::RunLogger::event(
                hgasp::LogLevel::info,
                "sp",
                "pre_reduction_global_sp_skipped",
                {},
                {
                    {"iteration", data->iteration},
                    {"phase", hgasp::toString(decision.action)},
                    {"reason", "no_feedback_from_checkpoint_50"},
                    {"dual_feedback_available",
                        feedback.dualFeedbackAvailable},
                    {"incumbent_improved", feedback.incumbentImproved},
                    {"novel_founder_candidates",
                        static_cast<unsigned long long>(
                            feedback.novelFounderCandidates)},
                });
            globalSpPhasePolicy->complete(
                decision.action,
                hgasp::GlobalSpPhaseCompletion::
                    SkippedNoCheckpointFeedback);
            continue;
        }

        const bool final = decision.action ==
            hgasp::GlobalSpPhaseAction::FinalGlobal;
        const double objectiveBefore = restartBestFitness();
        hgasp::RunLogger::event(
            hgasp::LogLevel::info,
            "sp",
            final
                ? "final_global_sp_started"
                : "pre_reduction_global_sp_started",
            {},
            {
                {"iteration", data->iteration},
                {"phase", hgasp::toString(decision.action)},
                {"objective_before", objectiveBefore},
            });
        try {
            SpExecutionOptions options;
            const bool roleAwareProfile =
                hgasp::usesRoleAwareCheckpointSearch(
                    data->parameters.profile);
            const bool exploratory = roleAwareProfile &&
                (decision.action ==
                     hgasp::GlobalSpPhaseAction::PreReductionGlobal1 ||
                 decision.action ==
                     hgasp::GlobalSpPhaseAction::PreReductionGlobal2);
            if (roleAwareProfile) {
                options.skipLp = !exploratory;
                options.forceLp = exploratory;
                options.exploratoryConfiguration = exploratory;
                // This bounded Global call also discovers promising depot
                // configurations. Let the opportunistic solution pool collect
                // alternatives for the full short budget instead of stopping
                // after the first improving cover.
                options.firstImprovement = false;
                options.forceP2Execution = true;
                options.reservedPhaseBudget = true;
                options.lpTimeLimitSeconds = exploratory
                    ? std::optional<double>(decision.lpTimeLimitSeconds)
                    : std::optional<double>(0.0);
                options.mipTimeLimitSeconds =
                    decision.mipTimeLimitSeconds;
                options.improvementExtensionGateSeconds =
                    decision.improvementExtensionGateSeconds;
                options.maximumInjections =
                    decision.maximumInjections;
                options.exploratoryActiveColumnTarget =
                    static_cast<std::size_t>(
                        data->exploratoryActiveColumnTarget);
            } else if (!final) {
                options.forceLp = true;
                options.forceP2Execution = true;
                options.lpTimeLimitSeconds = decision.lpTimeLimitSeconds;
                options.mipTimeLimitSeconds = decision.mipTimeLimitSeconds;
                options.maximumInjections = decision.maximumInjections;
            }
            const SpExecutionSummary summary =
                executePendingSp(final, options);
            if (exploratory) {
                pendingConfigurationRotationFounders =
                    summary.exploratoryFounders;
                if (decision.action == hgasp::GlobalSpPhaseAction::
                        PreReductionGlobal1 && summary.attempted) {
                    firstExploratoryGlobalFeedback =
                        hgasp::GlobalSpCheckpointFeedback{
                            true,
                            summary.lpFeedbackAvailable,
                            summary.improvedIncumbent,
                            summary.exploratoryFounders.size(),
                        };
                }
            } else if (decision.action ==
                       hgasp::GlobalSpPhaseAction::EarlyGlobal) {
                const auto rotated =
                    routeGenerator->rotateBestFreeConfiguration(
                        routePool.get(),
                        "free_founder_checkpoint_25");
                if (rotated.has_value()) {
                    populationLocalSpController.retire(
                        rotated->retiredToken);
                    roleAwareLocalSpLastEpoch.erase(rotated->retiredToken);
                    refreshRouteProtections(true);
                }
            }
            const hgasp::SpColumnTelemetry columnTelemetry =
                hgasp::makeSpColumnTelemetry(
                    summary.usefulColumnCredit,
                    summary.columns);
            const double objectiveAfter = restartBestFitness();
            const bool finalExecutionSatisfied =
                summary.attempted &&
                summary.finalGateSatisfied &&
                summary.finalIntensificationSatisfied;
            if (final && !finalExecutionSatisfied) {
                hgasp::RunLogger::event(
                    hgasp::LogLevel::error,
                    "sp",
                    "final_global_sp_finished",
                    {},
                    {
                        {"iteration", data->iteration},
                        {"phase", hgasp::toString(decision.action)},
                        {"status", "fatal_final_gate_unsatisfied"},
                        {"reason", summary.skipReason},
                        {"attempted", summary.attempted},
                        {"lp_attempted", summary.lpAttempted},
                        {"mip_attempted", summary.mipAttempted},
                        {"final_gate_satisfied",
                            summary.finalGateSatisfied},
                        {"final_intensification_satisfied",
                            summary.finalIntensificationSatisfied},
                        {"objective_before", objectiveBefore},
                        {"objective_after", objectiveAfter},
                        {"useful_column_credit",
                            columnTelemetry.usefulColumnCredit},
                        {"snapshot_column_count",
                            columnTelemetry.snapshotColumnCount},
                        {"cpu_seconds", summary.cpuSeconds},
                    });
                throw std::runtime_error(
                    "mandatory Final Global SP did not satisfy its "
                    "execution gate" +
                    (summary.skipReason.empty()
                         ? std::string()
                         : ": " + summary.skipReason));
            }
            hgasp::GlobalSpPhaseCompletion completion =
                summary.attempted
                    ? (summary.improvedIncumbent
                           ? hgasp::GlobalSpPhaseCompletion::Completed
                           : hgasp::GlobalSpPhaseCompletion::
                                 ValidNoImprovement)
                    : hgasp::GlobalSpPhaseCompletion::DeferredBudget;
            globalSpPhasePolicy->complete(
                decision.action, completion);
            if (final) {
                markFinalGlobalCompleted(
                    summary, "global_sp_phase_policy");
            }
            if (summary.attempted) {
                hgasp::recordV114ComponentActivation(
                    final
                        ? hgasp::V114Component::FinalGlobalSp
                        : hgasp::V114Component::PreReductionGlobalSp);
            }
            hgasp::RunLogger::event(
                hgasp::LogLevel::info,
                "sp",
                final
                    ? "final_global_sp_finished"
                    : "pre_reduction_global_sp_finished",
                {},
                {
                    {"iteration", data->iteration},
                    {"phase", hgasp::toString(decision.action)},
                    {"status", hgasp::toString(completion)},
                    {"objective_before", objectiveBefore},
                    {"objective_after", objectiveAfter},
                    {"useful_column_credit",
                        columnTelemetry.usefulColumnCredit},
                    {"snapshot_column_count",
                        columnTelemetry.snapshotColumnCount},
                    {"cpu_seconds", summary.cpuSeconds},
                });
            if (final) {
                finalSummary = summary;
            }
        } catch (...) {
            if (globalSpPhasePolicy->inFlight().has_value() &&
                *globalSpPhasePolicy->inFlight() == decision.action) {
                globalSpPhasePolicy->complete(
                    decision.action,
                    hgasp::GlobalSpPhaseCompletion::FatalError);
            }
            throw;
        }
    }
    return finalSummary;
}

bool AlgorithmOrchestrator::finishSearchAttempt(
    const double restartBestBeforeAttempt,
    const bool productiveFeasibleOffspring,
    const std::optional<int> activePopulationIndex) {
    const auto executeControlWork = [&]() -> bool {
    hgasp::ScopedComponentTimer controlTimer(
        hgasp::TimedComponent::SearchControl);
    const bool productiveIterationAdvanced =
        attemptScheduler.completeAttempt(
        data->iteration,
        data->spTriggerFrequency,
        data->spTriggerOffset,
        productiveFeasibleOffspring);
    routeGenerator->setCurrentPhysicalAttempt(
        attemptScheduler.completedAttemptCount());

    // HGAMP advances its search horizon and all iteration-driven control only
    // after a feasible offspring has completed refinement. Rejected children
    // and zero-child EAX batches remain auditable physical attempts and still
    // observe hard CPU/target termination, but cannot consume search quality
    // budget, stagnation budget, SP triggers, or population phases.
    if (!productiveIterationAdvanced) {
        const bool stopped = routeGenerator->checkStopCondition();
        if (stopped) {
            emitTerminalDiagnostics();
        }
        return stopped;
    }

    routeGenerator->printProgress(routePool.get());
    if (data->checkpointIntervalAttempts > 0 &&
        data->iteration % data->checkpointIntervalAttempts == 0) {
        emitPopulationLocalSpDecisionSummary(false);
    }

    const Individual* incumbent =
        routeGenerator->globalIncumbent();
    const bool globalChanged =
        incumbent != nullptr &&
        incumbent->fit + Min <
            lastProtectedGlobalFitness;
    const bool refreshElites =
        data->iteration > 0 &&
        data->iteration % 1000 == 0;
    if (globalChanged || refreshElites) {
        refreshRouteProtections(refreshElites);
    }

    if (globalSpPhasePolicy != nullptr) {
        const std::uint64_t horizon =
            globalSpPhasePolicy->phaseHorizonAttempts();
        const bool clrpFinalPhase =
            !hgasp::hasPolicyOwnedFinalGlobalSchedule(
                data->parameters.profile) &&
            !data->hasHardTimeWindows() &&
            static_cast<std::uint64_t>(
                std::max(0, data->iteration)) >=
                static_cast<std::uint64_t>(
                    std::ceil(
                        data->spFinalPhaseStartFraction *
                        static_cast<double>(horizon)));
        const bool terminalFinalPhase =
            data->iteration >= data->iterLimit ||
            (data->timeLimit > 0 &&
             data->elapsedCpuSeconds() >=
                 static_cast<double>(data->timeLimit));
        // V3 has one authoritative scheduled-final owner: the phase policy's
        // 95% threshold. The legacy 90% adaptive-final signal remains only
        // for profiles without a policy-owned Final Global. A true terminal
        // condition may still request the mandatory call immediately.
        dispatchV114PhaseActions(
            clrpFinalPhase || terminalFinalPhase);
    }

    // Termination has priority over a newly-latched SP request. In particular,
    // a child that reaches the global process-CPU boundary must not launch one
    // additional complete SP solve. The request remains unconsumed and is
    // explicitly reported as terminally deferred, so this is a deliberate
    // terminal policy rather than a skipped control-flow event.
    if (routeGenerator->checkStopCondition()) {
        emitTerminalDiagnostics();
        attemptScheduler.settleNonImprovement(
            data->nbIterNonImpro,
            restartBestFitness() + Min <
                restartBestBeforeAttempt);
        if (!data->staticHgampPopulationsEnabled() &&
            attemptScheduler.hasPendingSpTrigger()) {
            std::cout
                << "[SP Deferred] A scheduled SP event remains latched "
                   "because the global search termination condition was "
                   "reached after this HGA attempt."
                << std::endl;
        }
        return true;
    }

    if (data->staticHgampPopulationsEnabled()) {
        dispatchV114PopulationLocalSp();
    } else {
    // Keep the event latched until a Global SP has actually crossed the
    // execution guard. A controller interval/backoff decision must not lose
    // the scheduled request.
    const bool scheduledGlobal =
        attemptScheduler.hasPendingSpTrigger();
    std::vector<bool> nonEmptyPopulations(
        routeGenerator->popman->popula.size(), false);
    for (std::size_t index = 0;
         index < routeGenerator->popman->popula.size();
         ++index) {
        nonEmptyPopulations[index] =
            !routeGenerator->popman->popula[index].empty();
    }
    const int localPopulationIndex =
        chooseLocalPopulation(
            activePopulationIndex.value_or(-1),
            nonEmptyPopulations,
            bestPopulationIndex());
    if (data->spAdaptiveTriggerEnabled) {
        const double elapsedCpu =
            data->elapsedCpuSeconds();
        AdaptiveSpContext context;
        context.iteration = data->iteration;
        context.nonImprovingIterations =
            data->nbIterNonImpro;
        context.masterVersion =
            routePool->getMasterVersion();
        context.masterAppendCount =
            routePool->getMasterAppendCount();
        context.masterRepresentativeReplacementCount =
            routePool->getMasterRepresentativeReplacementCount();
        context.masterRepresentativeReplacementCostGain =
            routePool->getMasterRepresentativeReplacementCostGain();
        std::optional<hgasp::DepotConfigurationKey>
            localConfiguration;
        if (
            localPopulationIndex >= 0 &&
            static_cast<std::size_t>(localPopulationIndex) <
                routeGenerator->popman->popula.size() &&
            !routeGenerator->popman
                 ->popula[static_cast<std::size_t>(
                     localPopulationIndex)].empty()) {
            const Individual* localElite =
                routeGenerator->popman
                    ->popula[static_cast<std::size_t>(
                        localPopulationIndex)].front();
            std::vector<int> openDepots;
            for (int depot = 0; depot < data->numL; ++depot) {
                if (localElite->location[depot].used) {
                    openDepots.push_back(depot);
                }
            }
            localConfiguration =
                hgasp::makeDepotConfigurationKey(
                    std::move(openDepots));
        }
        if (localConfiguration.has_value()) {
            context.localMasterVersion =
                routePool->getConfigurationMasterVersion(
                    *localConfiguration);
            context.localMasterAppendCount =
                routePool->getConfigurationMasterAppendCount(
                    *localConfiguration);
            context.localMasterRepresentativeReplacementCount =
                routePool->
                    getConfigurationMasterRepresentativeReplacementCount(
                        *localConfiguration);
            context.localMasterRepresentativeReplacementCostGain =
                routePool->
                    getConfigurationMasterRepresentativeReplacementCostGain(
                        *localConfiguration);
        }
        if (localConfiguration.has_value()) {
            context.localScopeKey =
                configurationScopeKey(
                    *localConfiguration);
        }
        context.scheduledGlobal = scheduledGlobal;
        context.localEnabled = data->spLocalEnabled;
        context.finalIntensificationPhase =
            (data->iterLimit > 0 &&
             static_cast<double>(data->iteration) >=
                 data->spFinalPhaseStartFraction *
                     static_cast<double>(data->iterLimit)) ||
            (data->timeLimit > 0 &&
             elapsedCpu >=
                 data->spFinalPhaseStartFraction *
                     static_cast<double>(data->timeLimit));
        // Exactly one final Global evaluation is mandatory. Compatibility
        // mode uses one full-budget MIP. Optional tiering starts with the
        // short stage and grants later cumulative stages only after validated
        // strict improvement; a floating LP objective never
        // consumes the request without starting the integer solve.
        context.hardMandatoryGlobal =
            context.finalIntensificationPhase &&
            adaptiveSpController.finalIntensificationPending();
        context.remainingCpuSeconds =
            data->timeLimit > 0
                ? std::max(
                      0.0,
                      static_cast<double>(data->timeLimit) -
                          elapsedCpu)
                : std::numeric_limits<double>::infinity();

        const AdaptiveSpDecision decision =
            adaptiveSpController.decide(context);
        if (decision.dispatch != SpDispatch::None) {
            const char* dispatchName =
                decision.dispatch == SpDispatch::Global
                    ? "global"
                    : (decision.dispatch == SpDispatch::Local
                           ? "local"
                           : "none");
            std::cout
                << "[SP Adaptive] dispatch=" << dispatchName
                << " reason=" << decision.reason
                << " new_column_events="
                << decision.newColumnEvents
                << " local_threshold="
                << decision.effectiveLocalThreshold
                << " local_interval="
                << decision.effectiveLocalInterval
                << " local_scope_key="
                << decision.localScopeKey
                << " utility_ema="
                << adaptiveSpController.utilityEma()
                << " local_utility_ema="
                << adaptiveSpController.localUtilityEma(
                       decision.localScopeKey)
                << " global_utility_ema="
                << adaptiveSpController.globalUtilityEma()
                << std::endl;
            hgasp::RunLogger::event(
                hgasp::LogLevel::info,
                "sp",
                "adaptive_decision",
                {},
                {
                    {"iteration", data->iteration},
                    {"dispatch", dispatchName},
                    {"reason", decision.reason},
                    {"new_column_events",
                        static_cast<unsigned long long>(
                            decision.newColumnEvents)},
                    {"representative_replacement_events",
                        static_cast<unsigned long long>(
                            decision.representativeReplacementEvents)},
                    {"representative_replacement_cost_gain",
                        decision.representativeReplacementCostGain},
                    {"master_version",
                        static_cast<unsigned long long>(
                            context.masterVersion)},
                    {"local_master_version",
                        static_cast<unsigned long long>(
                            context.localMasterVersion.value_or(
                                context.masterVersion))},
                    {"local_threshold",
                        static_cast<unsigned long long>(
                            decision.effectiveLocalThreshold)},
                    {"local_interval",
                        decision.effectiveLocalInterval},
                    {"local_scope_key",
                        static_cast<unsigned long long>(
                            decision.localScopeKey)},
                    {"local_utility_ema",
                        adaptiveSpController.localUtilityEma(
                            decision.localScopeKey)},
                    {"global_utility_ema",
                        adaptiveSpController.globalUtilityEma()},
                    {"local_population",
                        localPopulationIndex},
                });
        }

        SpExecutionSummary summary;
        const bool mandatoryFinalDispatch =
            decision.dispatch == SpDispatch::Global &&
            context.hardMandatoryGlobal &&
            (decision.reason == "final_intensification" ||
             decision.reason == "scheduled_final_global");
        if (decision.dispatch == SpDispatch::Global) {
            summary = executePendingSp(
                mandatoryFinalDispatch);
        } else if (decision.dispatch == SpDispatch::Local) {
            summary = executeConfigurationLocalSp(
                localPopulationIndex);
        }
        if (mandatoryFinalDispatch) {
            const bool finalExecutionSatisfied =
                summary.attempted &&
                summary.finalGateSatisfied &&
                summary.finalIntensificationSatisfied;
            if (!finalExecutionSatisfied) {
                throw std::runtime_error(
                    "mandatory standalone Final Global SP did not "
                    "satisfy its execution gate" +
                    (summary.skipReason.empty()
                         ? std::string()
                         : ": " + summary.skipReason));
            }
            markFinalGlobalCompleted(
                summary, "adaptive_standalone");
        }
        if (decision.dispatch != SpDispatch::None &&
            summary.attempted) {
            if (decision.dispatch == SpDispatch::Global &&
                scheduledGlobal) {
                attemptScheduler.acknowledgeGlobalExecution(
                    true);
            }
            const double controllerImprovement =
                decision.dispatch == SpDispatch::Local
                    ? summary.targetObjectiveImprovement
                    : summary.objectiveImprovement;
            AdaptiveSpOutcome controllerOutcome;
            controllerOutcome.objectiveImprovement =
                controllerImprovement;
            controllerOutcome.cpuSeconds =
                summary.cpuSeconds;
            controllerOutcome.objectiveReference =
                summary.objectiveReference;
            controllerOutcome.incrementalUsefulColumnCredit =
                summary.usefulColumnCredit;
            controllerOutcome.structuralNovelty =
                decision.newColumnEvents;
            controllerOutcome.satisfiesFinalIntensification =
                mandatoryFinalDispatch &&
                summary.finalIntensificationSatisfied;
            controllerOutcome.masterSnapshotObserved =
                decision.dispatch != SpDispatch::Local ||
                summary.lpFeedbackAvailable ||
                summary.mipAttempted;
            controllerOutcome.observedAppendCount =
                decision.dispatch == SpDispatch::Local
                    ? context.localMasterAppendCount.value_or(
                          context.masterAppendCount.value_or(
                              context.masterVersion))
                    : context.masterAppendCount.value_or(
                          context.masterVersion);
            controllerOutcome.observedRepresentativeReplacementCount =
                decision.dispatch == SpDispatch::Local
                    ? context.localMasterRepresentativeReplacementCount
                          .value_or(0U)
                    : context.masterRepresentativeReplacementCount
                          .value_or(0U);
            controllerOutcome.observedRepresentativeReplacementCostGain =
                decision.dispatch == SpDispatch::Local
                    ? context.localMasterRepresentativeReplacementCostGain
                          .value_or(0.0)
                    : context.masterRepresentativeReplacementCostGain
                          .value_or(0.0);
            adaptiveSpController.reportOutcome(
                decision.dispatch,
                data->iteration,
                decision.dispatch == SpDispatch::Local
                    ? context.localMasterVersion.value_or(
                          context.masterVersion)
                    : context.masterVersion,
                controllerOutcome,
                decision.localScopeKey);
            routeGenerator->reportSpImprovement(
                summary.improvedIncumbent);
        } else if (
            decision.dispatch != SpDispatch::None &&
            summary.skipReason == "sp_cpu_share_guard") {
            adaptiveSpController.reportBudgetDeferral(
                decision.dispatch,
                data->iteration,
                decision.localScopeKey);
        } else if (
            decision.dispatch == SpDispatch::Local &&
            shouldConsumeLocalMasterVersionOnPreparationSkip(
                summary.skipReason)) {
            const std::uint64_t localMasterVersion =
                context.localMasterVersion.value_or(
                    context.masterVersion);
            adaptiveSpController
                .reportLocalPreconditionDeferral(
                    data->iteration,
                    localMasterVersion,
                    decision.localScopeKey,
                    context.localMasterAppendCount,
                    context.localMasterRepresentativeReplacementCount,
                    context.localMasterRepresentativeReplacementCostGain);
            hgasp::RunLogger::event(
                hgasp::LogLevel::info,
                "sp",
                "local_precondition_deferred",
                {},
                {
                    {"iteration", data->iteration},
                    {"reason", summary.skipReason},
                    {"local_scope_key",
                        static_cast<unsigned long long>(
                            decision.localScopeKey)},
                    {"local_master_version",
                        static_cast<unsigned long long>(
                            localMasterVersion)},
                    {"preparation_cpu_seconds",
                        summary.cpuSeconds},
                    {"counted_as_solve", false},
                    {"counted_as_utility", false},
                });
        }
    } else if (scheduledGlobal) {
        const SpExecutionSummary summary = executePendingSp();
        if (summary.attempted) {
            attemptScheduler.acknowledgeGlobalExecution(
                true);
        }
        routeGenerator->reportSpImprovement(
            summary.improvedIncumbent);
    } else if (data->spLocalEnabled &&
               data->iteration > 0 &&
               data->iteration %
                       data->spLocalTriggerFrequency ==
                   0) {
        const SpExecutionSummary summary =
            executeConfigurationLocalSp(
                localPopulationIndex);
        routeGenerator->reportSpImprovement(
            summary.improvedIncumbent);
    }
    }

    // Settle exactly once against the attempt's original restart incumbent.
    // The final value includes a possible SP injection, which makes HGA and SP
    // improvements reset stagnation under the same definition.
    attemptScheduler.settleNonImprovement(
        data->nbIterNonImpro,
        restartBestFitness() + Min <
            restartBestBeforeAttempt);

    const bool staticPopulationsAvailable =
        routeGenerator->staticPopulationsMaterialized();
    const bool v114TwPhaseReady =
        globalSpPhasePolicy == nullptr ||
        !staticPopulationsAvailable ||
        globalSpPhasePolicy->completed(
            hgasp::GlobalSpPhaseAction::Reduction2);
    const bool finalGlobalComplete =
        globalSpPhasePolicy != nullptr
        ? globalSpPhasePolicy->completed(
              hgasp::GlobalSpPhaseAction::FinalGlobal)
        : standaloneFinalGlobalCompleted;
    const hgasp::HardTwEarlyStopAction earlyStopAction =
        hgasp::decideHardTwEarlyStopAction(
            data->twProfileEarlyStopEnabled,
            data->hasHardTimeWindows(),
            v114TwPhaseReady,
            finalGlobalComplete,
            data->iteration,
            data->nbIterNonImpro,
            data->twEarlyStopMinimumIteration,
            data->twEarlyStopMinimumNonImprovingAttempts);
    if (earlyStopAction ==
        hgasp::HardTwEarlyStopAction::StopAfterCompletedFinal) {
        if (!lastCompletedFinalGlobalGate.has_value()) {
            throw std::logic_error(
                "completed Final Global gate has no retained evidence");
        }
        const SpExecutionSummary& priorGate =
            *lastCompletedFinalGlobalGate;
        data->searchTermination =
            hgasp::classifyCompletedFinalGlobalStop(
            std::max(0, data->iterLimit),
            std::max(0, data->iteration),
            priorGate.rigorousLpNoImprovement,
            priorGate.mipAttempted,
            priorGate.improvedIncumbent);
        hgasp::RunLogger::event(
            hgasp::LogLevel::info,
            "search",
            "tw_profile_early_stop_after_completed_final",
            {},
            {
                {"iteration", data->iteration},
                {"non_improving", data->nbIterNonImpro},
                {"final_global_repeated", false},
            });
        emitTerminalDiagnostics();
        routeGenerator->finalizeSearch();
        return true;
    }
    if (earlyStopAction ==
        hgasp::HardTwEarlyStopAction::ExecuteFinalGlobal) {
        const std::uint64_t gateMasterVersion =
            routePool->getMasterVersion();
        const std::uint64_t gateAppendCount =
            routePool->getMasterAppendCount();
        const std::uint64_t gateReplacementCount =
            routePool->getMasterRepresentativeReplacementCount();
        const double gateReplacementGain =
            routePool->getMasterRepresentativeReplacementCostGain();
        SpExecutionSummary earlyStopSummary;
        if (globalSpPhasePolicy != nullptr) {
            const auto dispatched =
                dispatchV114PhaseActions(true);
            if (!dispatched.has_value()) {
                throw std::logic_error(
                    "TW final gate was requested but no Final Global "
                    "phase executed");
            }
            earlyStopSummary = *dispatched;
        } else {
            earlyStopSummary = executePendingSp(true);
            const bool finalExecutionSatisfied =
                earlyStopSummary.attempted &&
                earlyStopSummary.finalGateSatisfied &&
                earlyStopSummary.finalIntensificationSatisfied;
            if (!finalExecutionSatisfied) {
                throw std::runtime_error(
                    "mandatory standalone Final Global SP did not "
                    "satisfy its execution gate" +
                    (earlyStopSummary.skipReason.empty()
                         ? std::string()
                         : ": " + earlyStopSummary.skipReason));
            }
            markFinalGlobalCompleted(
                earlyStopSummary, "hard_tw_standalone");
        }
        routeGenerator->reportSpImprovement(
            earlyStopSummary.improvedIncumbent);
        if (data->spAdaptiveTriggerEnabled &&
            earlyStopSummary.attempted) {
            AdaptiveSpOutcome outcome;
            outcome.objectiveImprovement =
                earlyStopSummary.objectiveImprovement;
            outcome.objectiveReference =
                earlyStopSummary.objectiveReference;
            outcome.cpuSeconds =
                earlyStopSummary.cpuSeconds;
            outcome.incrementalUsefulColumnCredit =
                earlyStopSummary.usefulColumnCredit;
            outcome.satisfiesFinalIntensification =
                earlyStopSummary
                    .finalIntensificationSatisfied;
            outcome.observedAppendCount =
                gateAppendCount;
            outcome.observedRepresentativeReplacementCount =
                gateReplacementCount;
            outcome.observedRepresentativeReplacementCostGain =
                gateReplacementGain;
            adaptiveSpController.reportOutcome(
                SpDispatch::Global,
                data->iteration,
                gateMasterVersion,
                outcome);
        }
        if (earlyStopSummary.improvedIncumbent) {
            data->nbIterNonImpro = 0;
        }
        const bool stopAfterGate =
            hgasp::shouldStopAfterHardTwFinalGate(
                earlyStopSummary.finalGateSatisfied,
                earlyStopSummary.improvedIncumbent);
        hgasp::RunLogger::event(
            stopAfterGate
                ? hgasp::LogLevel::info
                : hgasp::LogLevel::warning,
            "search",
            "tw_profile_early_stop_gate_finished",
            {},
            {
                {"iteration", data->iteration},
                {"non_improving", data->nbIterNonImpro},
                {"lp_attempted",
                    earlyStopSummary.lpAttempted},
                {"mip_attempted",
                    earlyStopSummary.mipAttempted},
                {"rigorous_lp_no_improvement",
                    earlyStopSummary
                        .rigorousLpNoImprovement},
                {"mip_improved_incumbent",
                    earlyStopSummary.improvedIncumbent},
                {"gate_satisfied",
                    earlyStopSummary.finalGateSatisfied},
                {"stop", stopAfterGate},
            });
        if (stopAfterGate) {
            if (earlyStopSummary.rigorousLpNoImprovement) {
                data->recordSearchTermination(
                    hgasp::SearchTerminationReason::
                        RigorousLpNoImprovement,
                    hgasp::EarlyStopEvidenceClass::
                        RigorousLpNoImprovement,
                    true,
                    earlyStopSummary.mipAttempted,
                    earlyStopSummary.improvedIncumbent);
            } else {
                data->recordSearchTermination(
                    hgasp::SearchTerminationReason::
                        HeuristicMipNoImprovement,
                    hgasp::EarlyStopEvidenceClass::
                        HeuristicMipNoImprovement,
                    false,
                    earlyStopSummary.mipAttempted,
                    earlyStopSummary.improvedIncumbent);
            }
            emitTerminalDiagnostics();
            routeGenerator->finalizeSearch();
            return true;
        }
    }

    // An SP call itself may consume the remaining process-CPU budget or reach
    // a target value. Stop before launching restart/incubation work.
    if (routeGenerator->checkStopCondition()) {
        emitTerminalDiagnostics();
        return true;
    }

    if (routeGenerator->checkStagnationAndRestart(
            routePool.get())) {
        emitTerminalDiagnostics();
        return true;
    }
    if (routeGenerator->consumeStaticRestartNotice()) {
        attemptScheduler.observePopulationRestart();
        if (globalSpPhasePolicy != nullptr) {
            globalSpPhasePolicy->observeSearchRestart();
        }
        v114LocalSpBaselinesInitialized = false;
        initializeV114LocalSpBaselines();
    }
    routeGenerator->checkSubPopReduction();
    const bool stopped = routeGenerator->checkStopCondition();
    if (stopped) {
        emitTerminalDiagnostics();
    }
    return stopped;
    };

    const bool shouldStop = executeControlWork();
    hgasp::ComponentProfiler& profiler =
        hgasp::ComponentProfiler::instance();
    const std::uint64_t completedIteration =
        static_cast<std::uint64_t>(
            std::max(0, data->iteration));
    if (profiler.shouldEmitSnapshot(completedIteration)) {
        hgasp::emitComponentTimingSnapshot(
            completedIteration,
            data->elapsedCpuSeconds());
        hgasp::emitHotPathCountersSnapshot(completedIteration);
    }
    return shouldStop;
}

void AlgorithmOrchestrator::run() {
    std::cout << "---- Start the search----" << std::endl;
	std::cout << "[HGA Config] search_policy="
			  << hgasp::kSearchPolicyId
			  << " eax_parent_mode="
              << hgasp::toString(data->eaxParentMode) << std::endl;
    std::cout << "[SP Config] trigger=" << data->spTriggerFrequency
              << " trigger_offset=" << data->spTriggerOffset
              << " first_trigger="
              << static_cast<long long>(data->spTriggerFrequency) +
                     data->spTriggerOffset
              << " global_outer_limit="
              << data->spGlobalOuterTimeLimit
              << "s final_global_outer_limit="
              << data->spFinalGlobalOuterTimeLimit
              << "s final_global_mip_limit="
              << data->spFinalGlobalMipTimeLimit
              << "s local_enabled="
              << (data->spLocalEnabled ? "true" : "false")
              << " local_frequency="
              << data->spLocalTriggerFrequency
              << " local_outer_limit="
              << data->spLocalOuterTimeLimit
              << "s adaptive_trigger="
              << (data->spAdaptiveTriggerEnabled
                      ? "true"
                      : "false")
              << " adaptive_min_interval="
              << data->spAdaptiveMinimumInterval
              << " adaptive_local_columns="
              << data->spAdaptiveLocalNewColumns
              << " adaptive_local_stagnation="
              << data->spAdaptiveLocalStagnation
              << " adaptive_global_stagnation="
              << data->spAdaptiveGlobalStagnation
              << " cpu_share_target="
              << data->spCpuShareTarget
              << " global_cpu_reserve_target="
              << data->spGlobalCpuReserveTarget
              << " global_first_improvement="
              << (data->spGlobalFirstImprovementEnabled
                      ? "true"
                      : "false")
              << " global_first_improvement_limit="
              << data->spGlobalFirstImprovementTimeLimit << "s"
              << " online_local_mip_limit="
              << data->spOnlineLocalMipTimeLimit << "s"
              << " online_global_mip_limit="
              << data->spOnlineGlobalMipTimeLimit << "s"
              << " mip_rescue_reserve_target="
              << data->spMipRescueReserveTarget
              << " lp_probe_cpu_share_target="
              << data->spLpProbeCpuShareTarget
              << " local_mip_cpu_share_target="
              << data->spLocalMipCpuShareTarget
              << " global_mip_cpu_share_target="
              << data->spGlobalMipCpuShareTarget
              << " tw_profile_early_stop="
              << (data->twProfileEarlyStopEnabled
                      ? "true"
                      : "false")
              << " tw_early_stop_min_iteration="
              << data->twEarlyStopMinimumIteration
              << " tw_early_stop_min_nonimproving="
              << data->twEarlyStopMinimumNonImprovingAttempts
              << " tw_final_mip_limit="
              << data->spTwFinalMipTimeLimit << "s"
              << " final_global_primal_idle_limit="
              << data->spFinalGlobalPrimalIdleTimeLimit << "s"
              << " lp_dual_feedback="
              << (data->spLpDualFeedbackEnabled
                      ? "true"
                      : "false")
              << " persistent_lp="
              << (data->spPersistentLpEnabled
                      ? "true"
                      : "false")
              << " global_lp_limit="
              << data->spGlobalLpTimeLimit << "s"
              << " local_lp_limit="
              << data->spLocalLpTimeLimit << "s"
              << " final_global_lp_limit="
              << data->spFinalGlobalLpTimeLimit << "s"
              << " global_solution_pool_size="
              << data->spGlobalSolutionPoolSize
              << " local_solution_pool_size="
              << data->spLocalSolutionPoolSize
              << " online_global_injections="
              << data->spOnlineGlobalInjectionCount
              << " online_local_injections="
              << data->spOnlineLocalInjectionCount
              << " final_global_injections="
              << data->spFinalGlobalInjectionCount
              << " adaptive_search_allocation="
              << (data->adaptiveSearchAllocationEnabled
                      ? "true"
                      : "false")
              << " work_limit=" << data->spWorkLimit
              << " work_units gap=" << data->spMIPGap
              << " threads=" << data->spThreads
              << " seed=" << data->spSeed
              << " client_log=" << data->spClientLogLevel
              << " server_timeout="
              << data->spServerTimeoutSeconds << "s"
              << " env_attempts="
              << data->spEnvironmentMaxAttempts
              << " env_retry_base="
              << data->spEnvironmentRetryBaseMilliseconds << "ms"
              << " wls_token_duration="
              << (data->spWLSTokenDurationMinutes == 0
                      ? "automatic"
                      : std::to_string(
                            data->spWLSTokenDurationMinutes) + "m")
              << " representation_limit="
              << (data->spEnforceRepresentationRouteLimit ? "true" : "false")
              << " restart_min_remaining_iterations="
              << data->restartMinimumRemainingIterations
              << " restart_min_remaining_cpu_seconds="
              << data->restartMinimumRemainingCpuSeconds
              << std::endl;
    
    // 1. Initialization
    data->iteration = 0;
    hgasp::ConfigurationScreeningResult configurationScreening;
    {
        hgasp::ScopedComponentTimer timer(
            hgasp::TimedComponent::
                DepotConfigurationPreprocess);
        configurationScreening =
            routeGenerator->preprocessingLocationConfiguration();
    }
    if (routeGenerator->hasFeasibleIncumbent()) {
        if (routeGenerator->checkStopCondition()) {
            emitTerminalDiagnostics();
            emitPopulationLocalSpDecisionSummary(true);
            return;
        }
    }
    
    data->loopp = 0;
    data->nbIterNonImpro = 1;
    
    routeGenerator->initialPop(
        routePool.get(), &configurationScreening);
    routeGenerator->materializeStaticHgampPopulations(
        routePool.get());
    if (routeGenerator->hasFeasibleIncumbent() &&
        routeGenerator->checkStopCondition()) {
        emitTerminalDiagnostics();
        emitPopulationLocalSpDecisionSummary(true);
        return;
    }
    std::cout << "[RoutePool] Building the initial route pool..." << std::endl;
    for (int i = 0; i < data->numLocConfiguration + 1; i++) {
        for (Individual* indiv : routeGenerator->popman->popula[i]) {
            if (indiv->isFeasible) {
                // Most candidates have already been submitted before
                // population admission. This idempotent extraction also
                // captures preprocessing seeds created before a pool was
                // available, without double-counting submission statistics.
                routePool->extractRoutes(indiv);
            }
        }
    }
    std::cout << "[RoutePool] Initial pool ready. Feasible-source mean: "
              << std::fixed << routePool->getAverageFitness() << std::endl;
    refreshRouteProtections(true);

    routeGenerator->getAverageValue(-1);
    initializeV114LocalSpBaselines();

    data->iteration = 0;
    
    while (true) {
        routeGenerator->advanceSingleDynamicChallenger(
            routePool.get(),
            globalSpPhasePolicy != nullptr &&
                globalSpPhasePolicy->completed(
                    hgasp::GlobalSpPhaseAction::Reduction2));
        // 2. Generate and refine HGA offspring.
        const double batchRestartBestBefore =
            restartBestFitness();
        int subPop1 = -1;
        int subPop2 = -1;
        {
            hgasp::ScopedComponentTimer timer(
                hgasp::TimedComponent::
                    ParentAndPopulationSelection);
            subPop1 = routeGenerator->selectsubPop(-1);
            subPop2 = routeGenerator->selectsubPop(subPop1);
        }
        const double batchCpuStarted =
            data->elapsedCpuSeconds();
        const std::uint64_t batchMasterAppendCount =
            routePool->getMasterAppendCount();
        int sizeSubPop = routeGenerator->generateEAXChildren(
            subPop1, subPop2, routePool.get());
        const double batchGenerationCpu =
            data->elapsedCpuSeconds() - batchCpuStarted;
        if (sizeSubPop <= 0) {
            routeGenerator->setCurrentPhysicalAttempt(
                attemptScheduler.completedAttemptCount() + 1U);
            const std::uint64_t traceId =
                routeGenerator->beginOffspringTrace(
                    subPop1, subPop2);
            routeGenerator->reportSearchOutcome(
                subPop1,
                subPop2,
                0.0,
                data->elapsedCpuSeconds() -
                    batchCpuStarted,
                static_cast<std::size_t>(
                    routePool->getMasterAppendCount() -
                        batchMasterAppendCount),
                traceId);
            // A degenerate EAX batch is a genuine physical attempt, but does
            // not consume the HGAMP-compatible productive horizon. Managed
            // runs retain their normal process/CPU timeout as the liveness
            // boundary for a pathologically nonproductive search.
            if (finishSearchAttempt(
                    batchRestartBestBefore,
                    false,
                    subPop1)) {
                emitPopulationLocalSpDecisionSummary(true);
                return;
            }
            // A zero-child batch has no buffered offspring, but consume the
            // restart marker before selecting its replacement parents.
            attemptScheduler.consumeOffspringBatchInvalidation();
            continue;
        }
        
        for (int i = 0; i < sizeSubPop; i++) {
            routeGenerator->setCurrentPhysicalAttempt(
                attemptScheduler.completedAttemptCount() + 1U);
            const std::uint64_t traceId =
                routeGenerator->beginOffspringTrace(
                    subPop1, subPop2);
            const double restartBestBeforeAttempt =
                restartBestFitness();
            const double attemptCpuStarted =
                data->elapsedCpuSeconds();
            const std::uint64_t masterAppendCountBefore =
                routePool->getMasterAppendCount();
            const bool isFeasible = routeGenerator->refineChild(i);
            if (isFeasible) {
                // 3. Integrate the child into the population and route pool.
                const hgasp::RouteSubmissionContext submission =
                    routeGenerator->submissionContext(
                        traceId, subPop1, subPop2, i);
                routeGenerator->integrateToPopulation(
                    i,
                    routePool.get(),
                    &submission);
            }
            routeGenerator->reportSearchOutcome(
                subPop1,
                subPop2,
                std::max(
                    0.0,
                    restartBestBeforeAttempt -
                        restartBestFitness()),
                data->elapsedCpuSeconds() -
                    attemptCpuStarted +
                    batchGenerationCpu /
                        static_cast<double>(
                            std::max(1, sizeSubPop)),
                static_cast<std::size_t>(
                    routePool->getMasterAppendCount() -
                        masterAppendCountBefore),
                traceId);

            // Every child reaches the same event tail, independently of TW
            // feasibility. Rejected offspring still count as genuine search
            // attempts, and passing an index only controls progress output.
            if (finishSearchAttempt(
                    restartBestBeforeAttempt,
                    isFeasible,
                    subPop1)) {
                emitPopulationLocalSpDecisionSummary(true);
                return;
            }
            if (attemptScheduler.consumeOffspringBatchInvalidation()) {
                // The restart retired both frozen parent generations. Do not
                // submit the remaining children from their old EAX batch.
                break;
            }
        }
    }
}
