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
#include "SpBudgetLedger.h"
#include "GlobalSpPhasePolicy.h"
#include "LocalSpFamilyBudget.h"
#include "PopulationLocalSpController.h"
#include "PopulationLocalSpDecisionLogging.h"
#include "SolverContext.h"
#include <chrono>
#include <memory>
#include <map>
#include <optional>
#include <cstdint>
#include <string>
#include <vector>

class AlgorithmOrchestrator {
public:
    AlgorithmOrchestrator(SolverContext * data);
    virtual ~AlgorithmOrchestrator();

    void run();
    // Release search-owned components while the process logger and instance
    // data are still alive.  This makes teardown part of the observable run
    // lifecycle and enforces consumer-before-provider destruction.
    void shutdown();
    static bool shouldTriggerSp(
        int iteration,
        int triggerFrequency,
        int triggerOffset);
    static std::vector<StandaloneRoute>
    filterRoutesForConfiguration(
        const std::vector<StandaloneRoute>& routes,
        const std::vector<bool>& openDepots,
        const std::vector<StandaloneRoute>& requiredRoutes = {});
    static bool shouldImportDualFeedback(
        const SpProblem& problem) noexcept;
    static int chooseLocalPopulation(
        int activePopulation,
        const std::vector<bool>& nonEmptyPopulations,
        int fallbackPopulation) noexcept;
    static bool shouldConsumeLocalMasterVersionOnPreparationSkip(
        const std::string& skipReason) noexcept;

private:
    struct SpExecutionSummary {
        bool attempted = false;
        bool incumbent = false;
        bool improvedIncumbent = false;
        bool lpAttempted = false;
        bool lpFeedbackAvailable = false;
        bool mipAttempted = false;
        bool mipSkippedByBound = false;
        bool rigorousLpNoImprovement = false;
        bool deferredEnvironmentFailure = false;
        bool deferredUnsupportedStatus = false;
        bool finalGateSatisfied = false;
        bool finalIntensificationSatisfied = false;
        double lpObjective = Max;
        double relativeLpPotential = 0.0;
        double finalGlobalMipTimeLimit = 0.0;
        double objectiveImprovement = 0.0;
        double targetObjectiveImprovement = 0.0;
        double directSpObjectiveImprovement = 0.0;
        double postSpLocalSearchImprovement = 0.0;
        double objectiveReference = Max;
        double usefulColumnCredit = 0.0;
        double cpuSeconds = 0.0;
        std::size_t columns = 0U;
        std::size_t injectionAttempts = 0U;
        std::size_t admittedSolutions = 0U;
        std::size_t rejectedSolutions = 0U;
        std::optional<hgasp::DepotConfigurationKey>
            configuration;
        std::vector<RouteGenerator::ScheduledConfigurationFounder>
            exploratoryFounders;
        std::string skipReason;
        std::string lpStatus = "NOT_RUN";
        std::string terminalStatus;
    };

    struct SpExecutionOptions {
        bool forceLp = false;
        bool skipLp = false;
        bool firstImprovement = false;
        bool forceP2Execution = false;
        bool reservedPhaseBudget = false;
        bool deferUnsupportedSolverOutcome = false;
        bool allowEphemeralFeedbackColumns = false;
        bool exploratoryConfiguration = false;
        std::size_t exploratoryActiveColumnTarget = 40000U;
        std::optional<double> lpTimeLimitSeconds;
        std::optional<double> mipTimeLimitSeconds;
        std::optional<double> improvementExtensionGateSeconds;
        std::optional<std::size_t> maximumInjections;
        std::optional<int> populationAdmissionTarget;
    };

    SolverContext * data;
    std::unique_ptr<RouteGenerator> routeGenerator;
    std::unique_ptr<RoutePool> routePool;
    std::unique_ptr<SPSolver> spSolver;
    std::unique_ptr<SPSolver> localSpSolver;
    SearchAttemptScheduler attemptScheduler;
    AdaptiveSpController adaptiveSpController;
    hgasp::SpBudgetLedger spBudgetLedger;
    hgasp::PopulationLocalSpController populationLocalSpController;
    hgasp::LocalSpFamilyBudget localSpFamilyBudget;
    hgasp::PopulationLocalSpDecisionLogSummary
        populationLocalSpDecisionLogSummary;
    std::unique_ptr<hgasp::GlobalSpPhasePolicy>
        globalSpPhasePolicy;
    bool v114LocalSpBaselinesInitialized = false;
    bool standaloneFinalGlobalCompleted = false;
    std::optional<SpExecutionSummary> lastCompletedFinalGlobalGate;
    std::map<hgasp::PopulationLocalSpToken, std::size_t>
        loggedLocalSpReactivations;
    std::map<hgasp::PopulationLocalSpToken, int>
        roleAwareLocalSpLastEpoch;
    std::size_t roleAwarePopulationLocalSpCalls = 0U;
    std::optional<std::vector<RouteGenerator::ScheduledConfigurationFounder>>
        pendingConfigurationRotationFounders;
    std::optional<hgasp::GlobalSpCheckpointFeedback>
        firstExploratoryGlobalFeedback;
    bool backboneInitialized = false;
    double lastProtectedGlobalFitness = Max;
    bool shutdownCompleted = false;
    bool searchAccountingEmitted = false;

    void releaseOwnedComponents() noexcept;

    SpProblem buildSpProblem() const;
    SpExecutionSummary executePendingSp(
        bool hardMandatory = false);
	void markFinalGlobalCompleted(
		const SpExecutionSummary& summary,
		const char* source);
    SpExecutionSummary executePendingSp(
        bool hardMandatory,
        const SpExecutionOptions& options);
    SpExecutionSummary executeConfigurationLocalSp(
        int populationIndex);
    SpExecutionSummary executeConfigurationLocalSp(
        int populationIndex,
        const SpExecutionOptions& options);
    SpExecutionSummary accountSpPreparationSkip(
        SpExecutionSummary summary,
        hgasp::SpBudgetCallToken& budgetToken,
        hgasp::SpScope budgetScope,
        const std::string& scope,
        const std::string& reason,
        double outerCpuStarted,
        bool schedulingBypass);
    SpExecutionSummary executeSpSnapshot(
        SPSolver& solver,
        SpProblem problem,
        std::vector<StandaloneRoute> poolSnapshot,
        SpMipStart start,
        const std::string& scope,
        double targetObjective,
        std::optional<hgasp::DepotConfigurationKey>
            configuration,
        bool hardMandatory,
        double outerCpuStarted,
        std::chrono::steady_clock::time_point outerWallStarted,
        double outerWallLimitSeconds,
        hgasp::SpBudgetCallToken& budgetToken,
        const SpExecutionOptions& options);
    void settleSpOuterFallback(
        hgasp::SpBudgetCallToken& budgetToken,
        hgasp::SpScope budgetScope,
        const std::string& scope,
        double outerCpuStarted,
        bool schedulingBypass,
        bool hardMandatory) noexcept;
    void recordSpFeedback(
        const SpProblem& problem,
        const std::vector<StandaloneRoute>& snapshot,
        const SpResult& result,
        const std::string& spCallId,
        bool allowEphemeralFeedbackColumns,
        double* usefulColumnCredit = nullptr);
    int bestPopulationIndex() const noexcept;
    void refreshRouteProtections(bool includePopulationElites);
    bool finishSearchAttempt(
        double restartBestBeforeAttempt,
        bool productiveFeasibleOffspring,
        std::optional<int> activePopulationIndex = std::nullopt);
    double restartBestFitness() const noexcept;
    hgasp::PopulationLocalSpObservation
    buildV114LocalSpObservation(
        const RouteGenerator::StaticPopulationRuntimeView& view);
    void initializeV114LocalSpBaselines();
    void dispatchV114PopulationLocalSp();
    void emitTerminalDiagnostics();
    void emitPopulationLocalSpDecisionSummary(bool final) noexcept;
    std::optional<SpExecutionSummary>
    dispatchV114PhaseActions(bool finalRequested);
    hgasp::PopulationLocalSpOutcome classifyV114LocalSpOutcome(
        const SpExecutionSummary& summary) const;
};

#endif /* ALGORITHM_ORCHESTRATOR_H_ */
