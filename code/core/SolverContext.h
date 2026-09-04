/******************************************************************************
 * File: SolverContext.h
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description:
 *   Owns one solver run's immutable problem data, resolved configuration,
 *   and mutable search state.
 ******************************************************************************/

#ifndef HGASP_CORE_SOLVER_CONTEXT_H_
#define HGASP_CORE_SOLVER_CONTEXT_H_

#include <ctime>
#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "basic.h"
#include "ConfigurationPopulationMode.h"
#include "EaxParentMode.h"
#include "ParsedInstance.h"
#include "ProblemContext.h"
#include "SearchParameters.h"
#include "SpExecutionPlan.h"

class SolverContext {
public:
	SolverContext();
	~SolverContext() = default;
	SolverContext(const SolverContext&) = delete;
	SolverContext& operator=(const SolverContext&) = delete;
	SolverContext(SolverContext&&) = delete;
	SolverContext& operator=(SolverContext&&) = delete;

	hgasp::SearchParameters parameters;

	// Search state and stopping information.
	int loopp;
	double probFixedCost;
	int iteration;
	int nbIterNonImpro;
	int isFleetLimited = 0;
	std::clock_t startTime;
	int timeLimit;
	int iterLimit;
	hgasp::SearchTerminationRecord searchTermination;
	int totalDemand;
	double avgDemand;
	bool isConstraintLoc;

	// Population parameters.
	int& nbClost = parameters.population.diversityNearestCount;
	int& nbElite = parameters.population.eliteCount;

	// Shaw removal parameters.
	double& determine_para = parameters.mutation.shawRankExponent;

	// HGAMP parameters.
	int& alpha = parameters.localSearch.constructionCorrelationAlpha;
	int& localSearchCustomerAlpha =
		parameters.localSearch.customerCandidateCount;
	int& localSearchDepotBeta =
		parameters.localSearch.depotCandidateCount;
	double& localSearchTemporalPenaltyWeight =
		parameters.localSearch.temporalPenaltyWeight;
	double& localSearchWaitingTimeWeight =
		parameters.localSearch.waitingTimeWeight;
	int& popMin = parameters.population.minimumSize;
	// HGAMP Table 2: Shaw mutation probability zeta = 0.15.
	int& mutationProbabilityPercent =
		parameters.mutation.probabilityPercent;
	double& ratioPerturbLength = parameters.mutation.ruinFraction;
	int& maxIterNonProd = parameters.restart.stagnationAttempts;
	// A full restart is allowed only when enough search budget remains to
	// rebuild and exploit a population. Suppression never resets
	// nbIterNonImpro because it is not an optimization improvement.
	int& restartMinimumRemainingIterations =
		parameters.restart.minimumRemainingAttempts;
	double& restartMinimumRemainingCpuSeconds =
		parameters.restart.minimumRemainingCpuSeconds;
	std::uint64_t restartSuppressedCount = 0U;
	int lastRestartSuppressedIteration = -1;
	int lastRestartSuppressedNonImproving = 0;
	int numLocConfiguration = 2000;
	int& maxIterationPerLoc =
		parameters.population.maximumIterationsPerConfiguration;
	int& initialPopulationCandidateMultiplier =
		parameters.population.initialCandidateMultiplier;
	int& populationIncubationMinimumConstructionAttempts =
		parameters.population.incubationMinimumConstructionAttempts;
	int& populationIncubationTargetConstructionMultiplier =
		parameters.population.incubationTargetConstructionMultiplier;
	int& populationBuildNoProgressMinimumAttempts =
		parameters.population.buildNoProgressMinimumAttempts;
	int& populationBuildNoProgressTargetMultiplier =
		parameters.population.buildNoProgressTargetMultiplier;
	int& populationPromotionConstructionMultiplier =
		parameters.population.promotionConstructionMultiplier;

	// V1.14 restores a static HGAMP population portfolio by default.  The
	// dynamic mode remains available for V1.13 regression studies, while the
	// unrestricted mode is the safe fallback when no fixed configuration can
	// be constructed.
	hgasp::ConfigurationPopulationMode populationMode =
		hgasp::ConfigurationPopulationMode::StaticHgamp;
	int& staticFixedPopulationCount =
		parameters.population.staticFixedCount;
	int& staticPopulationMinimumAgeAttempts =
		parameters.population.staticMinimumAgeAttempts;
	int& residentPostPromotionProtectionAttempts =
		parameters.population.residentPostPromotionProtectionAttempts;
	int& retiredConfigurationCooldownAttempts =
		parameters.configurationPortfolio
			.retiredConfigurationCooldownAttempts;
	int& popProtectionPeriod =
		parameters.population.residentProtectionAttempts;
	// V1.16.1 retains one bounded challenger after the permanent unrestricted
	// population. A second internal slot is reserved only as transactional
	// promotion staging; neither slot is an additional resident population.
	bool singleDynamicChallengerEnabled = true;
	// Comparison tolerance is part of the fixed numeric policy, not the
	// performance-tuning object.
	double challengerObjectiveTolerance = 1.0e-6;
	hgasp::ConfigurationRankerMode configurationRanker =
		hgasp::ConfigurationRankerMode::ProfileAwareCrh;
	int& configurationScreeningCheapLimit =
		parameters.configurationPortfolio.screeningCheapLimit;
	int& configurationScreeningFirstStage =
		parameters.configurationPortfolio.screeningFirstStage;
	int& configurationScreeningFinalists =
		parameters.configurationPortfolio.screeningFinalists;
	int& configurationEvaluationFirstSamples =
		parameters.configurationPortfolio.evaluationFirstSamples;
	int& configurationEvaluationTotalSamples =
		parameters.configurationPortfolio.evaluationTotalSamples;
	bool strictCrossConfigurationMating = true;
	int& crossConfigurationSelectionMaxAttempts =
		parameters.configurationPortfolio
			.crossConfigurationSelectionMaxAttempts;
	int& configurationExpansionMultiplier =
		parameters.configurationPortfolio.expansionMultiplier;
	int& populationPhaseHorizonAttempts =
		parameters.configurationPortfolio.phaseHorizonAttempts;
	double& preReductionGlobal1Fraction =
		parameters.configurationPortfolio.preReductionGlobal1Fraction;
	double& populationReduction1Fraction =
		parameters.configurationPortfolio.populationReduction1Fraction;
	int& populationReduction1SurvivorCount =
		parameters.configurationPortfolio.populationReduction1SurvivorCount;
	double& preReductionGlobal2Fraction =
		parameters.configurationPortfolio.preReductionGlobal2Fraction;
	double& populationReduction2Fraction =
		parameters.configurationPortfolio.populationReduction2Fraction;
	int& populationReduction2SurvivorCount =
		parameters.configurationPortfolio.populationReduction2SurvivorCount;
	double& populationAbsorptionFraction =
		parameters.configurationPortfolio.absorptionFraction;
	bool populationLocalSpEnabled = true;
	int& populationLocalSpStagnationAttempts =
		parameters.sp.populationLocal.stagnationAttempts;
	int& populationLocalSpMinimumAgeAttempts =
		parameters.sp.populationLocal.minimumAgeAttempts;
	int& populationLocalSpMinimumNewCustomerSets =
		parameters.sp.populationLocal.minimumNewCustomerSets;
	double& populationLocalSpMinimumReplacementGainRatio =
		parameters.sp.populationLocal.minimumReplacementGainRatio;
	double& populationLocalSpLpTimeLimitSeconds =
		parameters.sp.populationLocal.lpTimeLimitSeconds;
	double& populationLocalSpMipTimeLimitSeconds =
		parameters.sp.populationLocal.mipTimeLimitSeconds;
	int& populationLocalSpMaximumCalls =
		parameters.sp.populationLocal.maximumCalls;
	int& populationLocalSpFamilyMinimumInterval =
		parameters.sp.populationLocal.familyMinimumIntervalAttempts;
	double& populationLocalSpFamilyCpuShareLimit =
		parameters.sp.populationLocal.familyCpuShareLimit;
	int& populationLocalSpInjectionCount =
		parameters.sp.populationLocal.injectionCount;
	bool preReductionGlobalSpEnabled = true;
	double& preReductionGlobalSpLpTimeLimitSeconds =
		parameters.sp.preReductionGlobal.lpTimeLimitSeconds;
	double& preReductionGlobalSpMipTimeLimitSeconds =
		parameters.sp.preReductionGlobal.mipTimeLimitSeconds;
	int& preReductionGlobalSpInjectionCount =
		parameters.sp.preReductionGlobal.injectionCount;
	double& populationReductionObjectiveWeight =
		parameters.configurationPortfolio.reductionObjectiveWeight;
	double& populationReductionHgaImprovementWeight =
		parameters.configurationPortfolio.reductionHgaImprovementWeight;
	double& populationReductionDiversityWeight =
		parameters.configurationPortfolio.reductionDiversityWeight;
	double& populationReductionGlobalSpCreditWeight =
		parameters.configurationPortfolio.reductionGlobalSpCreditWeight;
	// Compact search checkpoints are intended for anytime analysis. They are
	// deliberately coarser than offspring generation so audit logs remain
	// bounded and focused on decisions and outcomes.
	int checkpointIntervalAttempts = 5000;

	bool staticHgampPopulationsEnabled() const noexcept {
		return populationMode ==
			hgasp::ConfigurationPopulationMode::StaticHgamp;
	}
	bool singleDynamicChallengerEffective() const noexcept {
		return singleDynamicChallengerEnabled &&
			staticHgampPopulationsEnabled() &&
			!parameters.configurationPortfolio
				.scheduledConfigurationRotation;
	}
	bool freeOnlyPopulationEnabled() const noexcept {
		return populationMode ==
			hgasp::ConfigurationPopulationMode::FreeOnly;
	}
	bool adaptivePopulationAllocationEffective() const noexcept {
		return adaptiveSearchAllocationEnabled &&
			!staticHgampPopulationsEnabled();
	}

	// Crossover and penalty parameters.
	int eSets_strategy = 3;
	hgasp::EaxParentMode eaxParentMode =
		hgasp::EaxParentMode::PaperCompatible;
	double penaltyDepot = 1.;
	double penaltyVeh = 1.;
	double normalizaPara;

	// Route-pool and set-partitioning parameters.
	int& spTriggerFrequency =
		parameters.sp.trigger.globalScheduledFrequency;
	// Delays the first scheduled global-SP checkpoint to frequency + offset.
	// Later checkpoints remain frequency iterations apart. In adaptive mode a
	// checkpoint can be skipped for an unchanged master or insufficient CPU,
	// while novelty/stagnation may dispatch SP before the next checkpoint.
	int spTriggerOffset = 0;
	double& spGlobalOuterTimeLimit =
		parameters.sp.online.globalOuterTimeLimitSeconds;
	double& spFinalGlobalMipTimeLimit =
		parameters.sp.finalGlobal.mipTimeLimitSeconds;
	double& spFinalGlobalPrimalIdleTimeLimit =
		parameters.sp.finalGlobal.primalIdleTimeLimitSeconds;
	double& spFinalGlobalOuterTimeLimit =
		parameters.sp.finalGlobal.outerTimeLimitSeconds;
	bool spLocalEnabled = true;
	int& spLocalTriggerFrequency =
		parameters.sp.trigger.localScheduledFrequency;
	double& spLocalOuterTimeLimit =
		parameters.sp.online.localOuterTimeLimitSeconds;
	bool spAdaptiveTriggerEnabled = true;
	int& spAdaptiveMinimumInterval =
		parameters.sp.trigger.adaptiveMinimumInterval;
	int& spCrossScopeLocalMinimumInterval =
		parameters.sp.trigger.crossScopeLocalMinimumInterval;
	int& spAdaptiveLocalNewColumns =
		parameters.sp.trigger.adaptiveLocalNewColumns;
	int& spAdaptiveLocalStagnation =
		parameters.sp.trigger.adaptiveLocalStagnation;
	int& spAdaptiveGlobalStagnation =
		parameters.sp.trigger.adaptiveGlobalStagnation;
	double& spAdaptiveUtilityEmaWeight =
		parameters.sp.trigger.adaptiveUtilityEmaWeight;
	double& spAdaptiveHighUtilityThreshold =
		parameters.sp.trigger.adaptiveHighUtilityThreshold;
	double& spFinalPhaseStartFraction =
		parameters.sp.trigger.finalPhaseStartFraction;
	bool spLpDualFeedbackEnabled = true;
	double& spGlobalLpTimeLimit =
		parameters.sp.online.globalLpTimeLimitSeconds;
	double& spLocalLpTimeLimit =
		parameters.sp.online.localLpTimeLimitSeconds;
	double& spFinalGlobalLpTimeLimit =
		parameters.sp.finalGlobal.lpTimeLimitSeconds;
	bool spPersistentLpEnabled = true;
	int& spGlobalSolutionPoolSize = parameters.sp.solutionPool.globalSize;
	int& spLocalSolutionPoolSize = parameters.sp.solutionPool.localSize;
	double& spGlobalSolutionPoolGap = parameters.sp.solutionPool.globalGap;
	double& spLocalSolutionPoolGap = parameters.sp.solutionPool.localGap;
	int& spGlobalSolutionPoolSearchMode =
		parameters.sp.solutionPool.globalSearchMode;
	int& spLocalSolutionPoolSearchMode =
		parameters.sp.solutionPool.localSearchMode;
	int& spOnlineGlobalInjectionCount =
		parameters.sp.online.globalInjectionCount;
	int& spOnlineLocalInjectionCount =
		parameters.sp.online.localInjectionCount;
	int& spFinalGlobalInjectionCount =
		parameters.sp.finalGlobal.injectionCount;
	bool spLpSkipMipIfNoPotential = true;
	int& spGlobalMinimumInterval =
		parameters.sp.trigger.globalMinimumInterval;
	int& spLocalZeroUtilityPatience =
		parameters.sp.trigger.localZeroUtilityPatience;
	int& spGlobalZeroUtilityPatience =
		parameters.sp.trigger.globalZeroUtilityPatience;
	double& spCpuShareTarget = parameters.sp.cpuBudget.familyShareTarget;
	double& spLpProbeCpuShareTarget =
		parameters.sp.cpuBudget.lpProbeShareTarget;
	double& spLocalMipCpuShareTarget =
		parameters.sp.cpuBudget.localMipShareTarget;
	double& spGlobalMipCpuShareTarget =
		parameters.sp.cpuBudget.globalMipShareTarget;
	// Optional Global-only reserve above the shared SP-family target.  A zero
	// value preserves the legacy shared-budget behavior.
	double& spGlobalCpuReserveTarget =
		parameters.sp.cpuBudget.globalReserveShare;
	// Opportunistic Global calls may stop as soon as they beat the current
	// incumbent. Disabled by default so existing manifests retain their
	// intensification semantics.
	bool spGlobalFirstImprovementEnabled = false;
	double spGlobalFirstImprovementTimeLimit = 20.0;
	// R8.1 limits opportunistic integer recombination to short rescue calls.
	// The configured Local/Global limits remain upper bounds for compatibility.
	double& spOnlineLocalMipTimeLimit =
		parameters.sp.online.localMipTimeLimitSeconds;
	double& spOnlineGlobalMipTimeLimit =
		parameters.sp.online.globalMipTimeLimitSeconds;
	// Local and Global micro-MIPs may temporarily use this family reserve;
	// guidance LP probes may not borrow it.
	double& spMipRescueReserveTarget =
		parameters.sp.cpuBudget.mipRescueReserveShare;
	// Hard-TW early termination is conservative: the thresholds only
	// request a final full-master LP/MIP gate. Disabling this switch restores
	// the iteration/time-limit stopping behavior without changing the model.
	bool twProfileEarlyStopEnabled = true;
	int twEarlyStopMinimumIteration = 15000;
	int twEarlyStopMinimumNonImprovingAttempts = 10000;
	double& spTwFinalMipTimeLimit =
		parameters.sp.finalGlobal.twMipTimeLimitSeconds;
	bool adaptiveSearchAllocationEnabled = true;
	double adaptivePopulationExploration = 0.25;
	double adaptivePopulationDiversityWeight = 0.10;
	double adaptivePopulationColumnWeight = 0.05;
	double adaptiveSearchEmaWeight = 0.20;
	int adaptiveMutationMinimumPercent = 5;
	int adaptiveMutationMaximumPercent = 40;
	// Optional deterministic Gurobi work budget. Zero leaves WorkLimit unset.
	double spWorkLimit = 0;
	double spMIPGap = 0.001;
	int& spMaxPoolSize = parameters.sp.routePoolCapacity;
	int spThreads = 1;
	// Empty means main() assigns an isolated gurobi.log path for the run.
	std::string spLogFile;
	int spOutputFlag = 0;
	int spSeed = 0;
	int& spMIPFocus = parameters.sp.mipFocus;
	double& spHeuristics = parameters.sp.heuristics;
	// Gurobi environment and WLS reliability controls. A zero token duration
	// preserves Gurobi's automatic policy instead of extending license
	// sessions in short-lived experiment workers.
	int spWLSTokenDurationMinutes = 0;
	double spWLSTokenRefresh = 0.9;
	int spClientLogLevel = 1;
	int spServerTimeoutSeconds = 30;
	int spEnvironmentMaxAttempts = 3;
	int spEnvironmentRetryBaseMilliseconds = 250;
	// The Individual representation now uses the proved numV bound, so this
	// redundant master constraint is disabled by default.
	bool spEnforceRepresentationRouteLimit = false;

	// Parsed instance data and neighborhoods.
	std::vector<std::vector<int>> edgeNode;
	std::vector<std::vector<int>> customerToLoc;
	std::vector<std::vector<int>> nearCity;
	std::vector<std::vector<int>> nearCityLoca;
	std::vector<std::vector<int>> edgeLocVec;
	int numV;
	int numL;
	int numC;
	int vehCap;
	int maxVnum;
	std::unique_ptr<hgasp::ProblemContext> problem;
	hgasp::InstanceFormat instanceFormat =
		hgasp::InstanceFormat::LegacyDat;
	// Row pointers retained for the existing D[from][to] API. All rows point
	// into one contiguous allocation to improve spatial locality and reduce
	// allocator/TLB overhead in the route-generator hot loops.
	double** D = nullptr;
	// Travel time is deliberately separate from objective travel cost. The
	// active metric profile selects each matrix independently.
	double** T = nullptr;
	double* x = nullptr;
	double* y = nullptr;
	int* demCus = nullptr;
	int* invLoc = nullptr;
	double* openCostLoc = nullptr;
	double usingCostVeh;
	int realOrInteger = 0;
	std::vector<double> readyTime;
	std::vector<double> dueTime;
	std::vector<double> serviceTime;
	std::vector<double> depotOpenTime;
	std::vector<double> depotCloseTime;
	std::vector<int> originalCustomerIds;
	std::vector<int> originalDepotIds;

	void applyEnvironmentOverrides();
	void allocateProblemStorage();
	void initializeRun(
		std::string pathInstance,
		std::string pathDepotCon,
		std::string pathSolution,
		double bestDis,
		int isOptimal,
		int timeLimit,
		int iterLimit);
	void configureProblemProfile(hgasp::ProblemProfile profile);
	const hgasp::ProblemContext& problemContext() const;
	const hgasp::ProblemProfile& problemProfile() const;
	bool hasHardTimeWindows() const noexcept;
	// Freeze parsed instance/profile data for the production search lifetime.
	// Callers must not mutate raw problem arrays after this succeeds.
	void finalizeProblemDataForSearch();
	bool problemDataFinalizedForSearch() const noexcept;
	bool temporalProblemDataFinalizedForSearch() const noexcept;
	std::uint64_t problemDataRevisionForSearch() const noexcept;
	int cachedRepresentationRouteLimit(int depotIndex) const;
	// Raw depot-configuration seeds remain encoded as numV + depot. This
	// read-only view preserves them for unrestricted configuration-island
	// seeding without changing FreeDiscovery parsing/clearing semantics.
	const std::vector<std::vector<int>>&
	depotConfigurationSeeds() const noexcept;
	std::vector<std::string> splitwithstr(std::string& str, char ch);
	bool equalTwoVar(double A, double B);
	bool stopCondition(double dis);
	void recordSearchTermination(
		hgasp::SearchTerminationReason reason,
		hgasp::EarlyStopEvidenceClass evidenceClass =
			hgasp::EarlyStopEvidenceClass::NotApplicable,
		bool lpRigorousNoImprovement = false,
		bool mipAttempted = false,
		bool mipImproved = false);
	// Reset immediately after instance loading and before search construction.
	// This excludes parser/startup work from the scientific search-time scope.
	void resetSearchCpuClock() noexcept;
	// Process CPU consumed since resetSearchCpuClock(). Algorithm limits and
	// the scientific time reported in solution.txt use this exact scope.
	double elapsedCpuSeconds() const noexcept;
	// Record the time at which the global (not restart-local) incumbent was
	// last strictly improved.
	void recordGlobalIncumbent() noexcept;

	// Run input and best-known-value tracking.
	std::string pathSolution;
	std::string pathDepotCon;
	std::string pathInstance;
	double bestDis;
	int isOptimal;
	bool meetOptimal = true;
	double timeToTarget = Max;
	double meetBestTime;

	// Candidate depot-configuration analysis.
	std::vector<std::pair<double, int>> locRatio;
	// Throws when the non-restrictive depot-search support required by shared
	// V1.1 components is incomplete or malformed.
	void validateDepotSearchSupport() const;
	std::vector<std::set<int>> compactLocConfigure;
	std::vector<std::pair<double, std::set<int>>> LocConfigure;
	int locCostSignificant;
	int dimension = 10000000;
	std::unordered_map<long long int, int> dimTabu1;
	std::unordered_map<long long int, int> dimTabu2;
	bool no_capacity_location = false;

private:
	friend struct SolverContextLoopSafetyTestAccess;
	std::vector<double> distanceStorage_;
	std::vector<double> timeStorage_;
	std::vector<double*> distanceRows_;
	std::vector<double*> timeRows_;
	std::vector<double> xStorage_;
	std::vector<double> yStorage_;
	std::vector<int> customerDemandStorage_;
	std::vector<int> depotCapacityStorage_;
	std::vector<double> depotOpeningCostStorage_;
	std::vector<int> conflictStorage_;
	std::vector<int*> conflictRows_;
	double maxDemand;
	double maxDist;
	int** conflictLoc = nullptr;
	void analyzeLocation();
	void tabuSearchFindCompactConfiguration();
	void initializeDepotSearchSupport();
	void spanningTreeForLoc();
	void analyzeConflict();
	void constructedDepotConfigurations();
	std::vector<std::set<int>> locIncCus;
	void readDepotConfigurations();
	std::vector<std::vector<int>> depotCon;
	void decideUsingDepotConfigurations();
	void invalidateFinalizedProblemData() noexcept;
	bool problemDataFinalizedForSearch_ = false;
	bool temporalProblemDataFinalizedForSearch_ = false;
	std::uint64_t problemDataRevisionForSearch_ = 0U;
	std::vector<int> representationRouteLimits_;
};

#endif  // HGASP_CORE_SOLVER_CONTEXT_H_
