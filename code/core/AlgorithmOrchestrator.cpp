/******************************************************************************
 * File: AlgorithmOrchestrator.cpp
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description: 
 *   High-level controller managing the algorithm lifecycle and orchestrating modules.
 ******************************************************************************/

#include "AlgorithmOrchestrator.h"
#include "DiverseSpSolutionSelector.h"
#include "RunLogger.h"
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

SpConfig makeSpConfig(const read_data& data) {
    SpConfig config;
    config.timeLimitSeconds = data.spTimeLimit;
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
    config.lpTimeLimitSeconds = data.spLpTimeLimit;
    config.persistentLpEnabled =
        data.spPersistentLpEnabled;
    config.solutionPoolSize = data.spSolutionPoolSize;
    config.solutionPoolGap = data.spSolutionPoolGap;
    config.solutionPoolSearchMode =
        data.spSolutionPoolSearchMode;

    // main() normally supplies a unique per-run path. Tests and embedding
    // callers may leave it empty to disable the native Gurobi log.
    if (!data.spLogFile.empty()) {
        config.logFile = data.spLogFile;
    }
    return config;
}

}  // namespace

AlgorithmOrchestrator::AlgorithmOrchestrator(read_data * data) {
    this->data = data;
    this->routeGenerator = std::unique_ptr<RouteGenerator>(new RouteGenerator(data));
    this->routePool = std::unique_ptr<RoutePool>(new RoutePool(data));
    this->spSolver =
        std::unique_ptr<SPSolver>(new SPSolver(makeSpConfig(*data)));
    SpConfig localConfig = makeSpConfig(*data);
    localConfig.timeLimitSeconds = data->spLocalTimeLimit;
    // Local RMP calls favor opportunistic alternatives. Systematic top-K
    // enumeration is reserved for the less frequent global intensification.
    localConfig.solutionPoolSearchMode =
        std::min(1, data->spSolutionPoolSearchMode);
    this->localSpSolver =
        std::unique_ptr<SPSolver>(
            new SPSolver(std::move(localConfig)));
    AdaptiveSpPolicy adaptivePolicy;
    adaptivePolicy.minimumInterval =
        data->spAdaptiveMinimumInterval;
    adaptivePolicy.localNewColumnThreshold =
        static_cast<std::uint64_t>(
            data->spAdaptiveLocalNewColumns);
    adaptivePolicy.localStagnationThreshold =
        data->spAdaptiveLocalStagnation;
    adaptivePolicy.globalStagnationThreshold =
        data->spAdaptiveGlobalStagnation;
    adaptivePolicy.utilityEmaWeight =
        data->spAdaptiveUtilityEmaWeight;
    adaptivePolicy.highUtilityThreshold =
        data->spAdaptiveHighUtilityThreshold;
    adaptiveSpController.configure(adaptivePolicy);
}

AlgorithmOrchestrator::~AlgorithmOrchestrator() {
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
        const std::vector<bool>& openDepots) {
    std::vector<StandaloneRoute> filtered;
    for (const StandaloneRoute& route : routes) {
        if (route.locId >= 0 &&
            static_cast<std::size_t>(route.locId) <
                openDepots.size() &&
            openDepots[route.locId]) {
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
        routePool->extractRoutes(incumbent);
        const std::vector<StandaloneRoute> globalRoutes =
            routePool->exportRoutes(incumbent);
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
AlgorithmOrchestrator::executePendingSp() {
    refreshRouteProtections(true);

    SpMipStart hgaStart;
    hgaStart.source = "hga_global_incumbent";
    if (const Individual* incumbent =
            routeGenerator->globalIncumbent()) {
        hgaStart.routes = routePool->exportRoutes(incumbent);
    }

    return executeSpSnapshot(
        *spSolver,
        buildSpProblem(),
        routePool->getPool(),
        std::move(hgaStart),
        "global");
}

AlgorithmOrchestrator::SpExecutionSummary
AlgorithmOrchestrator::executeConfigurationLocalSp(
        const int populationIndex) {
    SpExecutionSummary skipped;
    if (populationIndex < 0 ||
        static_cast<std::size_t>(populationIndex) >=
            routeGenerator->popman->popula.size() ||
        routeGenerator->popman->popula[populationIndex].empty()) {
        return skipped;
    }

    refreshRouteProtections(true);
    const Individual* elite =
        routeGenerator->popman->popula[populationIndex].front();
    std::vector<bool> openDepots(data->numL, false);
    for (int depot = 0; depot < data->numL; ++depot) {
        openDepots[depot] = elite->location[depot].used;
    }

    std::vector<StandaloneRoute> localPool =
        filterRoutesForConfiguration(
            routePool->getPool(),
            openDepots);
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
        return skipped;
    }

    SpProblem localProblem = buildSpProblem();
    localProblem.fixedDepotOpen = openDepots;
    SpMipStart localStart;
    localStart.source = "configuration_elite";
    localStart.routes = routePool->exportRoutes(elite);
    return executeSpSnapshot(
        *localSpSolver,
        std::move(localProblem),
        std::move(localPool),
        std::move(localStart),
        "local_population_" +
            std::to_string(populationIndex));
}

AlgorithmOrchestrator::SpExecutionSummary
AlgorithmOrchestrator::executeSpSnapshot(
        SPSolver& solver,
        SpProblem problem,
        std::vector<StandaloneRoute> poolSnapshot,
        SpMipStart start,
        const std::string& scope) {
    SpExecutionSummary summary;
    summary.attempted = true;
    summary.columns = poolSnapshot.size();
    const double beforeObjective = restartBestFitness();
    const double cpuStarted = data->elapsedCpuSeconds();
    const std::size_t currentPoolSize =
        poolSnapshot.size();
    std::cout << "\n[SP MONITOR] scope=" << scope
              << " iteration=" << data->iteration
              << " | RoutePool Size: " << currentPoolSize
              << std::endl;
    const std::string spCallId =
        scope + "@" + std::to_string(data->iteration);
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

    const SpResult spResult =
        solver.solve(
            problem,
            poolSnapshot,
            start);

    hgasp::EventFields solveFields{
        {"iteration", data->iteration},
        {"scope", scope},
        {"call_id", spCallId},
        {"status", spResult.statusName},
        {"columns", static_cast<unsigned long long>(
            currentPoolSize)},
        {"cpu_seconds", spResult.totalTimeSeconds},
        {"wall_seconds", spResult.totalWallTimeSeconds},
        {"lp_columns_stored",
            spResult.persistentLpStoredColumns},
        {"lp_columns_active",
            spResult.persistentLpActiveColumns},
        {"lp_compacted",
            spResult.persistentLpCompacted},
    };
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

    if (spResult.lpFeedbackAvailable &&
        shouldImportDualFeedback(problem)) {
        routePool->updateDualFeedback(
            poolSnapshot,
            spResult.customerDuals,
            spResult.routeReducedCosts);
        routeGenerator->updateDualGuidance(
            spResult.customerDuals);
    }

    if (!isSupportedSpTerminalStatus(spResult.status)) {
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
        summary.cpuSeconds =
            data->elapsedCpuSeconds() - cpuStarted;
        return summary;
    }

    std::cout
        << "[SP Success] Gurobi found a valid configuration!"
        << std::endl;

    try {
        const std::vector<std::size_t> diverseCandidates =
            hgasp::selectDiverseSpSolutions(
                spResult.candidateSolutions,
                static_cast<std::size_t>(
                    data->spDiverseInjectionCount));
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
            if (routeIndex >= poolSnapshot.size()) {
                throw std::out_of_range(
                    "SP returned a route index outside the solved "
                    "pool snapshot");
            }
            optimalCombination.push_back(
                poolSnapshot[routeIndex]);
        }
        routePool->setProtectedRoutes(
            RouteProtection::LastSpIncumbent,
            optimalCombination);
        routePool->pruneIfNeeded();

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
                if (routeIndex >= poolSnapshot.size()) {
                    throw std::out_of_range(
                        "SP solution-pool member references a route "
                        "outside the solved snapshot");
                }
                combination.push_back(
                    poolSnapshot[routeIndex]);
            }

            std::unique_ptr<Individual> spIndiv(new Individual());
            spIndiv->define(data);
            spIndiv->rebuildFromSP(combination);

            // A configuration-local RMP must feed back solutions from that
            // exact configuration. General location search remains a
            // global-SP postprocessor only.
            if (problem.fixedDepotOpen.empty()) {
                routeGenerator->ls->local_search_run(
                    spIndiv.get(),
                    data->penaltyDepot,
                    data->penaltyVeh);
            }

            const SpInjectionResult injection =
                routeGenerator->injectIncumbent(
                std::move(spIndiv),
                routePool.get());
            if (injection.admitted()) {
                ++summary.admittedSolutions;
            } else {
                ++summary.rejectedSolutions;
            }
        }
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
    const double afterObjective = restartBestFitness();
    summary.objectiveImprovement =
        std::max(0.0, beforeObjective - afterObjective);
    summary.improvedIncumbent =
        summary.objectiveImprovement > Min;
    summary.cpuSeconds =
        data->elapsedCpuSeconds() - cpuStarted;
    return summary;
}

double AlgorithmOrchestrator::restartBestFitness() const noexcept {
    return routeGenerator->popman->bestIndivRestart->fit;
}

bool AlgorithmOrchestrator::finishSearchAttempt(
    const double restartBestBeforeAttempt,
    const std::optional<int> progressChildIndex,
    const std::optional<int> activePopulationIndex) {
    attemptScheduler.completeAttempt(
        data->iteration,
        data->spTriggerFrequency,
        data->spTriggerOffset);

    if (progressChildIndex.has_value()) {
        routeGenerator->printProgress(*progressChildIndex);
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

    // Termination has priority over a newly-latched SP request. In particular,
    // a child that reaches the global process-CPU boundary must not launch one
    // additional complete SP solve. The request remains unconsumed and is
    // explicitly reported as terminally deferred, so this is a deliberate
    // terminal policy rather than a skipped control-flow event.
    if (routeGenerator->checkStopCondition()) {
        attemptScheduler.settleNonImprovement(
            data->nbIterNonImpro,
            restartBestFitness() + Min <
                restartBestBeforeAttempt);
        if (attemptScheduler.hasPendingSpTrigger()) {
            std::cout
                << "[SP Deferred] A scheduled SP event remains latched "
                   "because the global search termination condition was "
                   "reached after this HGA attempt."
                << std::endl;
        }
        return true;
    }

    const bool scheduledGlobal =
        attemptScheduler.consumeSpTrigger();
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
        context.scheduledGlobal = scheduledGlobal;
        context.localEnabled = data->spLocalEnabled;
        context.finalIntensificationPhase =
            (data->iterLimit > 0 &&
             static_cast<double>(data->iteration) >=
                 0.9 * static_cast<double>(data->iterLimit)) ||
            (data->timeLimit > 0 &&
             elapsedCpu >=
                 0.9 * static_cast<double>(data->timeLimit));
        context.remainingCpuSeconds =
            data->timeLimit > 0
                ? std::max(
                      0.0,
                      static_cast<double>(data->timeLimit) -
                          elapsedCpu)
                : std::numeric_limits<double>::infinity();

        const AdaptiveSpDecision decision =
            adaptiveSpController.decide(context);
        if (decision.dispatch != SpDispatch::None ||
            scheduledGlobal) {
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
                << " utility_ema="
                << adaptiveSpController.utilityEma()
                << " local_utility_ema="
                << adaptiveSpController.localUtilityEma()
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
                    {"local_threshold",
                        static_cast<unsigned long long>(
                            decision.effectiveLocalThreshold)},
                    {"local_utility_ema",
                        adaptiveSpController.localUtilityEma()},
                    {"global_utility_ema",
                        adaptiveSpController.globalUtilityEma()},
                    {"local_population",
                        localPopulationIndex},
                });
        }

        SpExecutionSummary summary;
        if (decision.dispatch == SpDispatch::Global) {
            summary = executePendingSp();
        } else if (decision.dispatch == SpDispatch::Local) {
            summary = executeConfigurationLocalSp(
                localPopulationIndex);
        }
        if (decision.dispatch != SpDispatch::None) {
            adaptiveSpController.report(
                decision.dispatch,
                data->iteration,
                context.masterVersion,
                summary.objectiveImprovement,
                summary.cpuSeconds);
            routeGenerator->reportSpImprovement(
                summary.improvedIncumbent);
        }
    } else if (scheduledGlobal) {
        const SpExecutionSummary summary = executePendingSp();
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

    // Settle exactly once against the attempt's original restart incumbent.
    // The final value includes a possible SP injection, which makes HGA and SP
    // improvements reset stagnation under the same definition.
    attemptScheduler.settleNonImprovement(
        data->nbIterNonImpro,
        restartBestFitness() + Min <
            restartBestBeforeAttempt);

    // An SP call itself may consume the remaining process-CPU budget or reach
    // a target value. Stop before launching restart/incubation work.
    if (routeGenerator->checkStopCondition()) {
        return true;
    }

    routeGenerator->checkStagnationAndRestart(routePool.get());
    routeGenerator->checkSubPopReduction();
    return routeGenerator->checkStopCondition();
}

void AlgorithmOrchestrator::run() {
    std::cout << "---- Start the search----" << std::endl;
    std::cout << "[HGA Config] eax_parent_mode="
              << hgasp::toString(data->eaxParentMode) << std::endl;
    std::cout << "[SP Config] trigger=" << data->spTriggerFrequency
              << " trigger_offset=" << data->spTriggerOffset
              << " first_trigger="
              << static_cast<long long>(data->spTriggerFrequency) +
                     data->spTriggerOffset
              << " time_limit=" << data->spTimeLimit
              << "s local_enabled="
              << (data->spLocalEnabled ? "true" : "false")
              << " local_frequency="
              << data->spLocalTriggerFrequency
              << " local_time_limit="
              << data->spLocalTimeLimit
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
              << " lp_dual_feedback="
              << (data->spLpDualFeedbackEnabled
                      ? "true"
                      : "false")
              << " persistent_lp="
              << (data->spPersistentLpEnabled
                      ? "true"
                      : "false")
              << " lp_time_limit="
              << data->spLpTimeLimit << "s"
              << " solution_pool_size="
              << data->spSolutionPoolSize
              << " solution_pool_gap="
              << data->spSolutionPoolGap
              << " solution_pool_mode="
              << data->spSolutionPoolSearchMode
              << " diverse_injections="
              << data->spDiverseInjectionCount
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
              << std::endl;
    
    // 1. Initialization
    data->iteration = 0;
    routeGenerator->preprocessingLocationConfiguration();
    if (routeGenerator->hasFeasibleIncumbent() &&
        routeGenerator->checkStopCondition()) {
        return;
    }
    
    data->loopp = 0;
    data->nbIterNonImpro = 1;
    
    routeGenerator->initialPop(routePool.get());
    if (routeGenerator->hasFeasibleIncumbent() &&
        routeGenerator->checkStopCondition()) {
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

    data->iteration = 0;
    
    while (true) {
        // 2. Generate and refine HGA offspring.
        const double batchRestartBestBefore =
            restartBestFitness();
        int subPop1 = routeGenerator->selectsubPop(-1);
        int subPop2 = routeGenerator->selectsubPop(subPop1);
        const double batchCpuStarted =
            data->elapsedCpuSeconds();
        const std::uint64_t batchMasterVersion =
            routePool->getMasterVersion();
        int sizeSubPop = routeGenerator->generateEAXChildren(subPop1, subPop2);
        if (sizeSubPop <= 0) {
            routeGenerator->reportSearchOutcome(
                subPop1,
                subPop2,
                0.0,
                data->elapsedCpuSeconds() -
                    batchCpuStarted,
                static_cast<std::size_t>(
                    routePool->getMasterVersion() -
                        batchMasterVersion));
            // A degenerate EAX batch is a genuine search attempt in either
            // mode. Counting it preserves a finite iteration-only run.
            if (finishSearchAttempt(
                    batchRestartBestBefore,
                    std::nullopt,
                    subPop1)) {
                return;
            }
            continue;
        }
        
        for (int i = 0; i < sizeSubPop; i++) {
            const double restartBestBeforeAttempt =
                restartBestFitness();
            const double attemptCpuStarted =
                data->elapsedCpuSeconds();
            const std::uint64_t masterVersionBefore =
                routePool->getMasterVersion();
            const bool isFeasible = routeGenerator->refineChild(i);
            if (isFeasible) {
                // 3. Integrate the child into the population and route pool.
                routeGenerator->integrateToPopulation(i, routePool.get());
            }
            routeGenerator->reportSearchOutcome(
                subPop1,
                subPop2,
                std::max(
                    0.0,
                    restartBestBeforeAttempt -
                        restartBestFitness()),
                data->elapsedCpuSeconds() -
                    attemptCpuStarted,
                static_cast<std::size_t>(
                    routePool->getMasterVersion() -
                        masterVersionBefore));

            // Every child reaches the same event tail, independently of TW
            // feasibility. Rejected offspring still count as genuine search
            // attempts, and passing an index only controls progress output.
            if (finishSearchAttempt(
                    restartBestBeforeAttempt,
                    isFeasible
                        ? std::optional<int>(i)
                        : std::nullopt,
                    subPop1)) {
                return;
            }
        }
    }
}
