/******************************************************************************
 * File: AlgorithmOrchestrator.cpp
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description: 
 *   High-level controller managing the algorithm lifecycle and orchestrating modules.
 ******************************************************************************/

#include "AlgorithmOrchestrator.h"
#include <algorithm>
#include <iomanip>
#include <iostream>
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
}

AlgorithmOrchestrator::~AlgorithmOrchestrator() {
}

bool AlgorithmOrchestrator::shouldTriggerSp(
    const int iteration,
    const int triggerFrequency,
    const int triggerOffset) {
    if (triggerFrequency <= 0) {
        throw std::invalid_argument(
            "SP trigger frequency must be positive");
    }
    if (triggerOffset < 0 || triggerOffset >= triggerFrequency) {
        throw std::invalid_argument(
            "SP trigger offset must be non-negative and smaller than the "
            "trigger frequency");
    }

    const long long firstTrigger =
        static_cast<long long>(triggerFrequency) + triggerOffset;
    if (static_cast<long long>(iteration) < firstTrigger) {
        return false;
    }
    return (static_cast<long long>(iteration) - triggerOffset) %
               triggerFrequency ==
           0;
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

void AlgorithmOrchestrator::run() {
    std::cout << "---- Start the search----" << std::endl;
    std::cout << "[SP Config] trigger=" << data->spTriggerFrequency
              << " trigger_offset=" << data->spTriggerOffset
              << " first_trigger="
              << static_cast<long long>(data->spTriggerFrequency) +
                     data->spTriggerOffset
              << " time_limit=" << data->spTimeLimit
              << "s work_limit=" << data->spWorkLimit
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
    routeGenerator->preprocessingLocationConfiguration();
    
    data->loopp = 0;
    data->nbIterNonImpro = 1;
    
    routeGenerator->initialPop(routePool.get());

    std::cout << "[RoutePool] Building the initial route pool..." << std::endl;
    for (int i = 0; i < data->numLocConfiguration + 1; i++) {
        for (Individual* indiv : routeGenerator->popman->popula[i]) {
            if (indiv->isFeasible) {
                routePool->extractRoutes(indiv);              
                routePool->recordSolutionFitness(indiv->fit); 
            }
        }
    }
    std::cout << "[RoutePool] Initial pool ready. Admission fitness mean: "
              << std::fixed << routePool->getAverageFitness() << std::endl;

    routeGenerator->getAverageValue(-1);

    data->iteration = 0;
    
    while (true) {
        // 2. Generate and refine HGA offspring.
        int subPop1 = routeGenerator->selectsubPop(-1);
        int subPop2 = routeGenerator->selectsubPop(subPop1);
        
        int sizeSubPop = routeGenerator->generateEAXChildren(subPop1, subPop2);
        
        for (int i = 0; i < sizeSubPop; i++) {
            bool isFeasible = routeGenerator->refineChild(i);
            if (isFeasible) {
                // 3. Integrate the child into the population and route pool.
                routeGenerator->integrateToPopulation(i, routePool.get());
                
                routeGenerator->printProgress(i);
                
                data->iteration++;
                
                // 4. Trigger the set-partitioning master problem.
                if (shouldTriggerSp(
                        data->iteration,
                        data->spTriggerFrequency,
                        data->spTriggerOffset)) {
                    
                    const std::vector<StandaloneRoute>& poolSnapshot =
                        routePool->getPool();
                    const std::size_t currentPoolSize =
                        poolSnapshot.size();
                    std::cout << "\n[SP MONITOR] Iteration " << data->iteration 
                              << " | RoutePool Size: " << currentPoolSize
                              << std::endl;
                              
                    const SpResult spResult =
                        spSolver->solve(buildSpProblem(), poolSnapshot);

                    std::cout << "[SP Result] status=" << spResult.statusName
                              << " solutions=" << spResult.solutionCount
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
                              << " total=" << spResult.totalTimeSeconds << "s"
                              << " env_attempts="
                              << spResult.environmentStartAttempts
                              << " env_retries="
                              << spResult.environmentRetryCount
                              << " env_reused="
                              << (spResult.environmentReused ? "true" : "false")
                              << " variables=" << spResult.modelVariables
                              << " constraints=" << spResult.modelConstraints
                              << " nonzeros=" << spResult.modelNonzeros
                              << " mip_start="
                              << (spResult.mipStartApplied ? "true" : "false")
                              << " mip_start_routes="
                              << spResult.mipStartRouteCount;
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

                    if (currentPoolSize < 10) {
                        std::cout << "[SP Warning] RoutePool is too small (" << currentPoolSize << "). Gurobi might not find any improvement." << std::endl;
                    }
                
                    if (spResult.hasFeasibleSolution()) {
                        std::cout << "[SP Success] Gurobi found a valid configuration!" << std::endl;

                        try {
                            std::vector<StandaloneRoute>
                                optimalCombination;
                            optimalCombination.reserve(
                                spResult.selectedRouteIndices.size());
                            for (const std::size_t routeIndex :
                                 spResult.selectedRouteIndices) {
                                if (routeIndex >= poolSnapshot.size()) {
                                    throw std::out_of_range(
                                        "SP returned a route index outside "
                                        "the solved pool snapshot");
                                }
                                optimalCombination.push_back(
                                    poolSnapshot[routeIndex]);
                            }

                            std::unique_ptr<Individual> spIndiv(
                                new Individual());
                            spIndiv->define(data);
                            spIndiv->rebuildFromSP(
                                optimalCombination);

                            routeGenerator->ls->local_search_run(
                                spIndiv.get(),
                                data->penaltyDepot,
                                data->penaltyVeh);

                            routeGenerator->injectIncumbent(
                                std::move(spIndiv),
                                routePool.get());
                        } catch (const std::exception& error) {
                            std::cerr
                                << "[SP Reject] Incumbent reconstruction or "
                                   "injection failed: "
                                << error.what() << std::endl;
                        }
                    } else {
                        std::cout << "[SP Info] No feasible SP incumbent is available.\n" << std::endl;
                    }
                }

                routeGenerator->checkStagnationAndRestart(routePool.get());
                data->nbIterNonImpro++;

                routeGenerator->checkSubPopReduction();

                if (routeGenerator->checkStopCondition()) {
                    return;
                }
            }
        }
    }
}
