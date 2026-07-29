/******************************************************************************
 * File: AlgorithmOrchestrator.h
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description: 
 *   High-level controller managing the algorithm lifecycle and orchestrating modules.
 ******************************************************************************/

#ifndef ALGORITHM_ORCHESTRATOR_H_
#define ALGORITHM_ORCHESTRATOR_H_

#include "AdaptiveSpController.h"
#include "RouteGenerator.h"
#include "RoutePool.h"
#include "SearchAttemptScheduler.h"
#include "SPSolver.h"
#include "read_data.h"
#include <memory>
#include <optional>
#include <string>
#include <vector>

class AlgorithmOrchestrator {
public:
    AlgorithmOrchestrator(read_data * data);
    virtual ~AlgorithmOrchestrator();

    void run();
    static bool shouldTriggerSp(
        int iteration,
        int triggerFrequency,
        int triggerOffset);
    static std::vector<StandaloneRoute>
    filterRoutesForConfiguration(
        const std::vector<StandaloneRoute>& routes,
        const std::vector<bool>& openDepots);
    static bool shouldImportDualFeedback(
        const SpProblem& problem) noexcept;
    static int chooseLocalPopulation(
        int activePopulation,
        const std::vector<bool>& nonEmptyPopulations,
        int fallbackPopulation) noexcept;

private:
    struct SpExecutionSummary {
        bool attempted = false;
        bool incumbent = false;
        bool improvedIncumbent = false;
        double objectiveImprovement = 0.0;
        double cpuSeconds = 0.0;
        std::size_t columns = 0U;
        std::size_t injectionAttempts = 0U;
        std::size_t admittedSolutions = 0U;
        std::size_t rejectedSolutions = 0U;
    };

    read_data * data;
    std::unique_ptr<RouteGenerator> routeGenerator;
    std::unique_ptr<RoutePool> routePool;
    std::unique_ptr<SPSolver> spSolver;
    std::unique_ptr<SPSolver> localSpSolver;
    SearchAttemptScheduler attemptScheduler;
    AdaptiveSpController adaptiveSpController;
    bool backboneInitialized = false;
    double lastProtectedGlobalFitness = Max;

    SpProblem buildSpProblem() const;
    SpExecutionSummary executePendingSp();
    SpExecutionSummary executeConfigurationLocalSp(
        int populationIndex);
    SpExecutionSummary executeSpSnapshot(
        SPSolver& solver,
        SpProblem problem,
        std::vector<StandaloneRoute> poolSnapshot,
        SpMipStart start,
        const std::string& scope);
    int bestPopulationIndex() const noexcept;
    void refreshRouteProtections(bool includePopulationElites);
    bool finishSearchAttempt(
        double restartBestBeforeAttempt,
        std::optional<int> progressChildIndex = std::nullopt,
        std::optional<int> activePopulationIndex = std::nullopt);
    double restartBestFitness() const noexcept;
};

#endif /* ALGORITHM_ORCHESTRATOR_H_ */
