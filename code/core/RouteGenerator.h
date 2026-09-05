/******************************************************************************
 * File: RouteGenerator.h
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description: 
 *   Core engine for generating routes and managing heuristic operators.
 ******************************************************************************/

#ifndef ROUTE_GENERATOR_H_
#define ROUTE_GENERATOR_H_

#include "initialsol.h"
#include "SolverContext.h"
#include "Individual.h"
#include "LS.h"
#include "crossover.h"
#include "popManager.h"
#include "RepairProcedure.h"
#include "mutation.h"
#include "RoutePool.h"
#include "AdaptiveSearchAllocator.h"
#include "GlobalSpPromotionTransaction.h"
#include "ChallengerSchedulingState.h"
#include "PopulationIncubationGuard.h"
#include "RouteProvenance.h"
#include "SpBudgetLedger.h"
#include "StaticConfigurationRegistry.h"
#include "ScreenedConfigurationSeed.h"
#include "ConfigurationCandidateArchive.h"
#include "DynamicChallengerPolicy.h"
#include "PopulationLocalSpSnapshot.h"
#include <vector>
#include <array>
#include <memory>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>

namespace hgasp {

struct ConfigurationScreeningResult {
    std::vector<DepotConfigurationKey> selectedConfigurations;
    std::map<DepotConfigurationKey,
             std::vector<ScreenedConfigurationSeed>> reusableSeeds;
};

struct ConfigurationDiscoveryTrace {
    DepotConfigurationKey beforeConfiguration;
    DepotConfigurationKey afterConfiguration;
    ConfigurationDiscoverySource source =
        ConfigurationDiscoverySource::Count;
    ConfigurationDiscoveryContributorMask contributors = 0U;
    std::string evidenceUnitId;
    std::string routeAncestry;
    bool independentlyValidated = false;
    bool strictGlobalIncumbent = false;
    bool initialized = false;
};

inline ConfigurationDiscoveryTrace initialFreeConfigurationDiscoveryTrace(
        const DepotConfigurationKey& beforeLocalSearch,
        const DepotConfigurationKey& afterLocalSearch,
        std::string evidenceUnitId) {
    ConfigurationDiscoveryTrace trace;
    const bool changed = beforeLocalSearch != afterLocalSearch;
    trace.beforeConfiguration = changed
        ? beforeLocalSearch
        : makeDepotConfigurationKey(std::vector<int>{});
    trace.afterConfiguration = afterLocalSearch;
    trace.source = changed
        ? ConfigurationDiscoverySource::LocalSearch
        : ConfigurationDiscoverySource::FreeInitialization;
    trace.contributors = configurationDiscoverySourceBit(
        ConfigurationDiscoverySource::FreeInitialization);
    if (changed) {
        trace.contributors |= configurationDiscoverySourceBit(
            ConfigurationDiscoverySource::LocalSearch);
    }
    trace.evidenceUnitId = std::move(evidenceUnitId);
    trace.independentlyValidated = true;
    trace.initialized = true;
    return trace;
}

struct ConfigurationPopulationBuildOptions {
    DepotConfigurationKey configuration;
    std::vector<ScreenedConfigurationSeed> seeds;
    std::size_t targetSize = 0U;
    std::size_t maximumConstructionAttempts = 0U;
    bool requireExactConfiguration = true;
    std::optional<PopulationAdmissionTarget> admissionTarget;
    // Transaction staging may consume constructors and LS, but it must not
    // publish routes, update the incumbent, or leak configuration discoveries
    // before the prepared replacement commits.
    bool speculative = false;
    bool retainConfigurationDiscoveries = true;
    std::string source;
};

struct ConfigurationPopulationBuildResult {
    std::size_t seedSnapshotsAttempted = 0U;
    std::size_t seedSnapshotsAdmitted = 0U;
    std::size_t constructionAttempts = 0U;
    std::size_t constructionSuccesses = 0U;
    std::size_t admittedMembers = 0U;
    std::size_t finalPopulationSize = 0U;
    bool reachedTarget = false;
    double cpuSeconds = 0.0;
};

}  // namespace hgasp

enum class PopulationRole {
    FixedResident,
    UnrestrictedFree,
    DynamicChallenger,
    InactiveOrSpare,
};

class RouteGenerator {
public:
    RouteGenerator(SolverContext * data);
    virtual ~RouteGenerator();

    // 1. Initialization
    hgasp::ConfigurationScreeningResult
    preprocessingLocationConfiguration();
    void initialPop(
        RoutePool* pool,
        const hgasp::ConfigurationScreeningResult* screening = nullptr);
    // Builds and registers the V1.14 fixed-configuration portfolio.  For
    // profiles without a configuration file this runs after the unrestricted
    // bootstrap and derives candidates from solutions that were actually
    // feasible under the active problem profile.
    void materializeStaticHgampPopulations(RoutePool* pool);
    struct ExactConfigurationEducationResult {
        bool seedValid = false;
        bool lsAccepted = false;
        // Retained for result/telemetry compatibility. Fixed education now
        // rejects configuration-changing moves before commit, so both remain
        // false.
        bool rolledBack = false;
        bool changedConfigurationRetained = false;
    };
    // Runs the common local-search engine in its explicit exact-configuration
    // mode. Configuration-changing moves are rejected before commit. R1 also
    // uses the same mode for active-resident offspring; free/novel offspring
    // and SP post-LS clones remain unrestricted.
    ExactConfigurationEducationResult educateExactConfigurationSeed(
        Individual& target,
        int populationIndex,
        bool retainConfigurationDiscoveries = true);
    ExactConfigurationEducationResult educateExactConfigurationSeed(
        Individual& target,
        const hgasp::DepotConfigurationKey& configuration,
        bool retainConfigurationDiscoveries = true);
    hgasp::ConfigurationPopulationBuildResult
    buildConfigurationPopulation(
        int populationIndex,
        const hgasp::ConfigurationPopulationBuildOptions& options,
        RoutePool* pool);
    // Advances the single-slot lifecycle at an HGA attempt boundary. The
    // caller supplies the run-level Reduction2 latch so TW late-launch policy
    // does not infer phase state from population sizes.
    void advanceSingleDynamicChallenger(
        RoutePool* pool,
        bool reduction2Completed);
    PopulationRole populationRole(int populationIndex) const noexcept;
    bool challengerActive() const noexcept;
    int challengerPopulationIndex() const noexcept;
    
    // 2. Generation & Heuristics
    int selectsubPop(int subpop1);
    static int deterministicSubpopulationFallback(
        const std::vector<int>& eligible,
        int excludedPopulation,
        const std::vector<std::size_t>& populationSizes);
    int generateEAXChildren(
        int& subPop1,
        int& subPop2,
        RoutePool* pool = nullptr);
    bool refineChild(int childIndex); 
    
    // 3. Integration & Incumbent
    bool integrateToPopulation(
        int childIndex,
        RoutePool* pool,
        const hgasp::RouteSubmissionContext* submission = nullptr);
    SpInjectionResult injectIncumbent(
        std::unique_ptr<Individual> spIndiv,
        RoutePool* pool,
        const std::string& spCallId = std::string(),
        std::optional<int> populationAdmissionTarget = std::nullopt,
        hgasp::ConfigurationDiscoveryContributorMask
            additionalDiscoveryContributors = 0U,
        const std::string& evidenceUnitId = std::string());
    bool retainPostSpLocalSearchConfigurationDiscovery(
        Individual& candidate,
        const hgasp::DepotConfigurationKey& beforeConfiguration,
        RoutePool* pool,
        const std::string& spCallId,
        bool globalSpScope);
    SpInjectionResult injectPostSpLocalSearchCandidate(
        Individual& candidate,
        const hgasp::DepotConfigurationKey& rawSpConfiguration,
        RoutePool* pool,
        const std::string& spCallId,
        std::optional<int> unchangedConfigurationTarget = std::nullopt,
        const std::string& evidenceUnitId = std::string());
    
    // 4. System Status & Restarts
    void getAverageValue(int rank);
    // Returns true when restart initialization reaches a terminal condition.
    bool checkStagnationAndRestart(RoutePool* pool);
    void checkSubPopReduction();
    std::vector<hgasp::PopulationLocalSpToken>
    reduceStaticPopulations(int phase);
    struct StaticPopulationReplacementOutcome {
        hgasp::PopulationLocalSpToken retiredToken;
        hgasp::PopulationLocalSpToken replacementToken;
        int populationIndex = -1;
        hgasp::DepotConfigurationKey configuration;
    };
    struct ScheduledConfigurationFounder {
        hgasp::DepotConfigurationKey configuration;
        std::vector<StandaloneRoute> routes;
        double objective = Max;
    };
    enum class ScheduledConfigurationRotationStatus {
        Rotated,
        Disabled,
        NoCandidates,
        AllCandidatesAlreadyResident,
        RecoverablePromotionFailure,
    };
    struct ScheduledConfigurationRotationResult {
        ScheduledConfigurationRotationStatus status =
            ScheduledConfigurationRotationStatus::NoCandidates;
        std::optional<StaticPopulationReplacementOutcome> replacement;
        std::size_t candidatesConsidered = 0U;
        std::size_t residentDuplicatesSkipped = 0U;
        std::string reason;
    };
    struct StaticPopulationRuntimeView {
        int populationIndex = -1;
        bool active = false;
        bool unrestricted = false;
        hgasp::PopulationLocalSpToken localSpToken;
        hgasp::DepotConfigurationKey configuration;
        std::uint64_t createdAttempt = 0U;
        std::uint64_t lastBestImprovementAttempt = 0U;
        std::uint64_t verifiedGlobalInjectionCount = 0U;
        double bestObjective = Max;
    };
    std::vector<StaticPopulationRuntimeView>
    staticPopulationRuntimeViews() const;
    std::optional<StaticPopulationReplacementOutcome>
    rotateBestFreeConfiguration(RoutePool* pool, const std::string& source);
    ScheduledConfigurationRotationResult
    rotateConfigurationFromFounders(
        const std::vector<ScheduledConfigurationFounder>& founders,
        RoutePool* pool,
        const std::string& source);
    void recordVerifiedGlobalInjection(int populationIndex);
    bool consumeStaticRestartNotice() noexcept;
    bool staticPopulationsMaterialized() const noexcept;
    bool checkStopCondition();
    // Writes the terminal audit summaries and validated incumbent exactly
    // once, including policy-driven stops that do not exhaust a hard limit.
    void finalizeSearch();
    bool hasFeasibleIncumbent() const noexcept;
    const Individual* globalIncumbent() const noexcept;
    void setSpBudgetLedger(
        hgasp::SpBudgetLedger* ledger) noexcept;
    std::uint64_t beginOffspringTrace(
        int firstPopulationIndex,
        int secondPopulationIndex);
    hgasp::RouteSubmissionContext submissionContext(
        std::uint64_t traceId,
        int firstPopulationIndex,
        int secondPopulationIndex,
        int childIndex = -1) const;
    void reportDelayedRouteCredit(
        const std::vector<hgasp::OriginCredit>& credits,
        const std::string& sourceCallId,
        hgasp::SpFeedbackScope scope);
    void reportSearchOutcome(
        int firstPopulationIndex,
        int secondPopulationIndex,
        double objectiveImprovement,
        double cpuSeconds,
        std::size_t newColumns,
        std::uint64_t traceId = 0U);
    void reportSpImprovement(bool improved);
    void reportLocalSpCall(const std::string& scope);
    void printProgress(const RoutePool* pool);
    void setCurrentPhysicalAttempt(std::uint64_t attempt);
    std::uint64_t currentPhysicalAttempt() const noexcept {
        return currentPhysicalAttempt_;
    }

    // Public members strictly needed for state access
    std::unique_ptr<popManager> popman;
    std::unique_ptr<LS> ls;
    SolverContext * data;

private:
    friend struct RouteGeneratorTransactionTestAccess;
    std::unique_ptr<initial_sol> initialSol;
    std::unique_ptr<crossover> cross;
    std::unique_ptr<RepairProcedure> repair;
    std::unique_ptr<mutation> mutationOperator;
	bool refineUnifiedChild(
		int childIndex,
		int mutationProbability);

    std::unique_ptr<Individual> global;
    std::unique_ptr<Individual> loc_best;
    std::unique_ptr<Individual> temIndiv;
    std::unique_ptr<Individual> bestIndivRestart;
    std::unique_ptr<Individual[]> subPop; // used to save offspring solutions
    
    Individual * fa;
    Individual * ma;

    std::vector<double> avgCostPop;
    hgasp::AdaptiveSearchAllocator searchAllocator;
    hgasp::SpBudgetLedger* spBudgetLedger = nullptr;
    int currentMutationProbabilityPercent = 15;
    bool spJustImproved = false;
    bool lastMutationCommitted = false;
    std::uint64_t nextTraceId = 1U;
    std::uint64_t nextOffspringBatchId = 1U;
    std::uint64_t lastOffspringBatchId = 0U;
    std::uint64_t nextConfigurationEvidenceUnitSequence = 1U;
    std::uint64_t currentPhysicalAttempt_ = 0U;
    hgasp::OffspringBatchLineageSnapshot lastBatchParentLineage;
    // Compatibility mirrors while the batch-lineage consumers are migrated;
    // they are assigned only at the same parent-selection boundary.
    std::uint64_t lastFirstParentIndividualId = 0U;
    std::uint64_t lastSecondParentIndividualId = 0U;
    std::optional<hgasp::DepotConfigurationKey>
        lastFirstParentConfiguration;
    std::string lastFirstParentSpAncestry;
    std::string lastSecondParentSpAncestry;
    bool lastChildAdmitted = false;
    bool restartSuppressionLatched = false;
    bool finalComponentSummaryEmitted = false;
    std::uint64_t crossPopulationMatings = 0U;
    std::uint64_t crossConfigurationMatings = 0U;
    std::uint64_t sameConfigurationFallbackMatings = 0U;
    std::uint64_t staticMatingAttempts = 0U;
    std::uint64_t roleAwarePrimaryMatingOpportunity = 0U;
    std::vector<std::uint64_t> roleAwareResidentLastPrimaryOpportunity;
    std::unordered_map<std::string, std::uint64_t>
        sameConfigurationFallbackReasons;
    std::unordered_map<int, std::uint64_t>
        newMasterColumnsByPrimaryPopulation;
    std::unordered_map<int, double>
        selectedMasterColumnsByPopulation;
    std::unordered_map<int, double>
        localSelectedMasterColumnsByPopulation;
    std::map<std::uint64_t, double>
        challengerSelectedColumnsByGeneration;
    std::map<std::uint64_t, double>
        activeHistoricalChallengerSelectionsByGeneration;
    std::map<std::uint64_t, double>
        retiredHistoricalChallengerSelectionsByGeneration;
    std::unordered_map<std::string, std::uint64_t>
        localSpCallsByScope;

    struct StaticPopulationRuntimeState {
        std::uint64_t generation = 1U;
        std::uint64_t createdAttempt = 0U;
        std::uint64_t lastBestImprovementAttempt = 0U;
        std::uint64_t verifiedGlobalInjectionCount = 0U;
        double bestObjective = Max;
        double hgaImprovement = 0.0;
        double hgaCpuSeconds = 0.0;
    };
    std::unique_ptr<hgasp::StaticConfigurationRegistry>
        staticConfigurationRegistry;
    std::vector<StaticPopulationRuntimeState>
        staticPopulationRuntime;
    // A free population may materialize below popMin when structural
    // duplicate filtering leaves fewer distinct individuals (notably on
    // tiny instances).  Preserve the largest viable size observed up to
    // popMin; once popMin is reached it becomes the permanent floor.
    std::size_t permanentFreePopulationFloor = 0U;
    bool staticRestartNotice = false;
    bool staticPopulationStartEmitted = false;
    std::uint64_t staticReplacementCount = 0U;
    std::uint64_t staticReductionCount = 0U;
    hgasp::ConfigurationCandidateArchive candidateArchive;
    hgasp::DynamicChallengerPolicy challengerPolicy;
    hgasp::ChallengerRuntime challengerRuntime;
    std::uint64_t challengerGeneration = 0U;
    hgasp::ChallengerSchedulingState challengerScheduling;
    std::uint64_t uniqueNonresidentConfigurationsSeen = 0U;
    std::uint64_t candidateRecordsCreated = 0U;
    std::uint64_t candidateRecordsRecreated = 0U;
    std::uint64_t candidateRecordsEvicted = 0U;
    std::uint64_t candidateRecordsCapacityRejected = 0U;
    std::uint64_t candidateRecordsEligible = 0U;
    std::uint64_t candidateEligibilityExceptional = 0U;
    std::uint64_t candidateEligibilityRepeatedQuality = 0U;
    std::uint64_t candidateRejectedByQualityFloor = 0U;
    std::uint64_t candidateBlockedByCooldown = 0U;
    std::uint64_t candidateBlockedNoReplaceableResident = 0U;
    std::uint64_t challengerTrialsStarted = 0U;
    std::uint64_t challengerTrialsCompleted = 0U;
    std::uint64_t challengerRetriesAfterCooldown = 0U;
    std::uint64_t challengerPromotions = 0U;
    std::uint64_t challengerRejections = 0U;
    std::uint64_t challengerFillFailures = 0U;
    std::uint64_t globalSpDirectPromotions = 0U;
    std::uint64_t globalSpPromotedActiveChallenger = 0U;
    std::uint64_t scheduledConfigurationRotations = 0U;
    double challengerFillCpuSeconds = 0.0;
    double challengerTrialHgaCpuSeconds = 0.0;
    std::array<std::uint64_t,
        hgasp::configurationDiscoverySourceCount()>
        configurationDiscoveriesBySource{};
    std::array<std::uint64_t,
        hgasp::configurationDiscoverySourceCount()>
        configurationFreeAdmissionsBySource{};
    std::array<std::uint64_t, 5U> configurationChangesByKind{};

    struct PendingConfigurationDiscovery {
        hgasp::ScreenedConfigurationSeed seed;
        hgasp::ConfigurationDiscoveryTrace trace;
    };
    std::map<hgasp::DepotConfigurationKey,
             std::vector<PendingConfigurationDiscovery>>
        pendingConfigurationDiscoveries;
    hgasp::InitialFreePopulationObservationGate
        initialFreePopulationObservationGate;
    std::unordered_map<
        std::uint64_t,
        hgasp::ConfigurationDiscoveryTrace>
        pendingBootstrapFreeDiscoveries;
    std::array<hgasp::ConfigurationDiscoveryTrace, 100U>
        childConfigurationDiscovery;
    std::array<
        std::optional<Individual::PopulationValidationReceipt>,
        100U> childPopulationValidationReceipts;

    struct TraceObservation {
        double cpuSeconds = 0.0;
        int firstPopulation = -1;
        int secondPopulation = -1;
        std::uint64_t firstPopulationGeneration = 0U;
        std::uint64_t secondPopulationGeneration = 0U;
    };
    std::unordered_map<std::uint64_t, TraceObservation>
        traceObservations;
    
    // Helper function moved from solution
    void evolvePop(int rankPop, int &count, RoutePool* pool);
    bool timeLimitReached() const noexcept;
    bool initializationStopRequested() const;
    int currentPhysicalAttemptForProvenance() const;
    void incubatePopulation(
        int populationIndex,
        RoutePool* pool,
        const char* source);
    void registerPopulationConfiguration(int populationIndex);
    std::vector<hgasp::DepotConfigurationKey>
    profileAwareRankConfigurations(
        const std::vector<hgasp::DepotConfigurationKey>& candidates) const;
    hgasp::ConfigurationScreeningResult
    progressivelyEvaluateConfigurations(
        const std::vector<hgasp::DepotConfigurationKey>& ranked);
    std::vector<StandaloneRoute> snapshotValidatedIndividual(
        const Individual& individual) const;
    std::vector<hgasp::DepotConfigurationKey>
    deriveBootstrapConfigurationCandidates() const;
    void publishStaticConfigurations(
        const std::vector<hgasp::DepotConfigurationKey>& configurations);
    void initializeStaticConfigurationRegistry();
    hgasp::ConfigurationCandidateEligibilityContext
    challengerEligibilityContext() const;
    struct ConfigurationAdmissionResult {
        bool admitted = false;
        int populationIndex = -1;
        PopulationRole role = PopulationRole::InactiveOrSpare;
    };
    void observeConfigurationDiscovery(
        const Individual& candidate,
        bool admittedToFree,
        const hgasp::ConfigurationDiscoveryTrace& trace,
        PopulationRole admissionRole);
    ConfigurationAdmissionResult admitConfigurationDiscovery(
        Individual& candidate,
        RoutePool* pool,
        const hgasp::RouteSubmissionContext* submission,
        const hgasp::ConfigurationDiscoveryTrace& trace,
        bool isInitial = false,
        const Individual::PopulationValidationReceipt*
            validation = nullptr);
    hgasp::PopulationAdmissionTarget admissionTargetForConfiguration(
        const hgasp::DepotConfigurationKey& configuration) const;
    int addIndividualToStaticTarget(
        Individual& candidate,
        const hgasp::PopulationAdmissionTarget& target,
        bool isInitial,
        RoutePool* pool,
        const hgasp::RouteSubmissionContext* submission,
        bool* admitted = nullptr,
        std::uint64_t* survivingIndividualId = nullptr,
        bool allowIncumbentUpdate = true,
        const Individual::PopulationValidationReceipt*
            validation = nullptr);
    hgasp::ParentLineageSnapshot captureParentLineage(
        const Individual& parent,
        int populationIndex) const;
    bool isCurrentParentLineage(
        const hgasp::ParentLineageSnapshot& parent) const noexcept;
    std::string makeConfigurationEvidenceUnitId(
        const char* kind);
    bool queuePendingConfigurationDiscovery(
        hgasp::ScreenedConfigurationSeed seed,
        const hgasp::ConfigurationDiscoveryTrace& trace);
    void drainPendingConfigurationDiscoveries(RoutePool* pool);
    void observeInitializedFreePopulation(
        bool registryAlreadyMaterialized);
    void initializeChildConfigurationDiscovery(
        int childIndex,
        hgasp::ConfigurationDiscoverySource source,
        const hgasp::DepotConfigurationKey& before,
        const hgasp::DepotConfigurationKey& after);
    void recordChildConfigurationStage(
        int childIndex,
        hgasp::ConfigurationDiscoverySource source,
        const hgasp::DepotConfigurationKey& before,
        const hgasp::DepotConfigurationKey& after);
    void clearChallengerPopulation();
    void refreshPopulationEliteProtection(RoutePool* pool) const;
    void rejectActiveChallenger(const char* reason, RoutePool* pool);
    std::optional<StaticPopulationReplacementOutcome>
    promoteConfiguration(
        const hgasp::DepotConfigurationKey& configuration,
        hgasp::StaticReplacementAuthority authority,
        Individual* founder,
        RoutePool* pool,
        const std::string& source,
        bool challengerTrialCompleted,
        bool beatsWeakestResident,
        hgasp::PromotionFailureKind* failureKind = nullptr,
        std::string* failureReason = nullptr);
    bool evaluateActiveChallenger(RoutePool* pool);
    std::vector<double> populationObjectives(int populationIndex) const;
    bool exactPopulationConfiguration(
        const Individual& individual,
        int populationIndex) const;
    bool exactConfiguration(
        const Individual& individual,
        const hgasp::DepotConfigurationKey& configuration) const;
    void refreshStaticRuntime(int populationIndex);
    std::size_t refreshPermanentFreePopulationFloor();
    void requirePermanentFreePopulationPreserved(
        std::size_t sizeBefore) const;
    double populationDiversity(int populationIndex) const;
    hgasp::DepotConfigurationKey configurationKey(
        const Individual& individual) const;
    void outputprocess(Individual *indiv);
};

#endif /* ROUTE_GENERATOR_H_ */
