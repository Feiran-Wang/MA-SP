#ifndef HGASP_CORE_SEARCH_PARAMETERS_H_
#define HGASP_CORE_SEARCH_PARAMETERS_H_

#include "BenchmarkFamily.h"

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>

namespace hgasp {

enum class SearchParameterProfile {
    V2P9,
    Hgamp2ParameterCompatibleSetS,
    V3SetS,
    V3SetF,
    V4SetS,
    V4SetF,
};

enum class MutationPolicy {
    Alternating,
    ShawOnly,
};

inline constexpr bool isV3SearchParameterProfile(
        const SearchParameterProfile profile) noexcept {
    return profile == SearchParameterProfile::V3SetS ||
        profile == SearchParameterProfile::V3SetF;
}

inline constexpr bool isSetSProductionSearchProfile(
        const SearchParameterProfile profile) noexcept {
    return profile == SearchParameterProfile::V3SetS ||
        profile == SearchParameterProfile::V4SetS;
}

inline constexpr bool isSetFProductionSearchProfile(
        const SearchParameterProfile profile) noexcept {
    return profile == SearchParameterProfile::V3SetF ||
        profile == SearchParameterProfile::V4SetF;
}

inline constexpr bool usesRoleAwareCheckpointSearch(
        const SearchParameterProfile profile) noexcept {
    return isSetSProductionSearchProfile(profile) ||
        isSetFProductionSearchProfile(profile);
}

inline constexpr bool isV4SearchParameterProfile(
        const SearchParameterProfile profile) noexcept {
    return profile == SearchParameterProfile::V4SetS ||
        profile == SearchParameterProfile::V4SetF;
}

inline constexpr bool preserveActiveResidentConfigurationDuringOffspringLs(
        const SearchParameterProfile profile,
        const bool primaryParentIsFixedResident,
        const bool configurationIsActiveResident) noexcept {
    return usesRoleAwareCheckpointSearch(profile) &&
        primaryParentIsFixedResident &&
        configurationIsActiveResident;
}

inline constexpr bool hasPolicyOwnedFinalGlobalSchedule(
        const SearchParameterProfile profile) noexcept {
    return usesRoleAwareCheckpointSearch(profile);
}

inline constexpr const char* toString(
    const SearchParameterProfile profile) noexcept {
    switch (profile) {
        case SearchParameterProfile::V2P9:
            return "v2-p9";
        case SearchParameterProfile::Hgamp2ParameterCompatibleSetS:
            return "hgamp2-parameter-compatible-set-s";
        case SearchParameterProfile::V3SetS:
            return "v3-set-s";
        case SearchParameterProfile::V3SetF:
            return "v3-set-f";
        case SearchParameterProfile::V4SetS:
            return "v4-set-s";
        case SearchParameterProfile::V4SetF:
            return "v4-set-f";
    }
    return "unknown";
}

inline SearchParameterProfile parseSearchParameterProfile(
    const std::string_view value) {
    if (value == "v2-p9")
        return SearchParameterProfile::V2P9;
    if (value == "hgamp2-parameter-compatible-set-s")
        return SearchParameterProfile::Hgamp2ParameterCompatibleSetS;
    if (value == "v3-set-s")
        return SearchParameterProfile::V3SetS;
    if (value == "v3-set-f")
        return SearchParameterProfile::V3SetF;
    if (value == "v4-set-s")
        return SearchParameterProfile::V4SetS;
    if (value == "v4-set-f")
        return SearchParameterProfile::V4SetF;
    throw std::invalid_argument(
        "unsupported search-parameter profile '" +
        std::string(value) + "'");
}

// Canonical storage for parameters that may change search efficiency or
// solution quality without changing the problem definition. Algorithm-mode
// switches, numeric correctness policy, solver resources, and runtime state
// deliberately remain outside this object.
struct SearchParameters {
    SearchParameterProfile profile = SearchParameterProfile::V2P9;

    struct Population {
        int minimumSize = 30;
        // Lambda is stored independently from mu. The former popMax value
        // overloaded a population-size parameter with a derived upper bound.
        int offspringCount = 30;
        int diversityNearestCount = 5;
        int eliteCount = 4;
        int staticFixedCount = 15;
        int staticMinimumAgeAttempts = 5000;
        // Protection for the newly admitted resident only. Challenger
        // rejection and retired-configuration cooldowns are distinct
        // decisions and deliberately live in their owning groups below.
        int residentPostPromotionProtectionAttempts = 10000;
        int residentProtectionAttempts = 10000;
        int maximumIterationsPerConfiguration = 500;
        int initialCandidateMultiplier = 4;
        int incubationMinimumConstructionAttempts = 100;
        int incubationTargetConstructionMultiplier = 50;
        int buildNoProgressMinimumAttempts = 50;
        int buildNoProgressTargetMultiplier = 10;
        int promotionConstructionMultiplier = 3;

        int maximumSize() const noexcept {
            return minimumSize + offspringCount;
        }
    } population;

    struct Mutation {
        MutationPolicy policy = MutationPolicy::Alternating;
        int probabilityPercent = 15;
        double ruinFraction = 0.25;
        double shawRankExponent = 6.0;
        int insertionCandidateListWidth = 4;
        double insertionRankExponent = 2.0;
        int recreateAttempts = 12;
    } mutation;

    struct LocalSearch {
        int constructionCorrelationAlpha = 20;
        int customerCandidateCount = 20;
        int depotCandidateCount = 20;
        int candidateShuffleDenominator = 20;
        double temporalPenaltyWeight = 0.0;
        double waitingTimeWeight = 0.0;
    } localSearch;

    struct Construction {
        int insertionCandidateListWidth = 8;
        double insertionRankExponent = 6.0;
        int exactInsertionCandidateListWidth = 4;
        double exactInsertionRankExponent = 6.0;
        double depotSelectionRankExponent = 6.0;
    } construction;

    struct Restart {
        int stagnationAttempts = 70000;
        int minimumRemainingAttempts = 50000;
        double minimumRemainingCpuSeconds = 600.0;
    } restart;

    struct ConfigurationPortfolio {
        // V3.3 replaces the continuously managed challenger island with one
        // checkpoint-owned resident lease. The slot is populated only from
        // the initial free population or an exploratory Global-SP result.
        bool scheduledConfigurationRotation = false;
        int screeningCheapLimit = 100;
        int screeningFirstStage = 50;
        int screeningFinalists = 20;
        int evaluationFirstSamples = 3;
        int evaluationTotalSamples = 10;
        int crossConfigurationSelectionMaxAttempts = 64;
        int expansionMultiplier = 4;
        double depotSelectionRankExponent = 6.0;
        int phaseHorizonAttempts = 0;
        double preReductionGlobal1Fraction = 0.45;
        double populationReduction1Fraction = 0.50;
        int populationReduction1SurvivorCount = 8;
        double preReductionGlobal2Fraction = 0.70;
        double populationReduction2Fraction = 0.75;
        int populationReduction2SurvivorCount = 5;
        double absorptionFraction = 0.05;
        double reductionObjectiveWeight = 0.55;
        double reductionHgaImprovementWeight = 0.20;
        double reductionDiversityWeight = 0.15;
        double reductionGlobalSpCreditWeight = 0.10;
        int retiredConfigurationCooldownAttempts = 10000;
    } configurationPortfolio;

    struct Challenger {
        // This is intentionally separate from resident replacement age. The
        // old path reused staticPopulationMinimumAgeAttempts for both roles.
        std::uint64_t selectionMinimumAttempt = 5000U;
        std::uint64_t selectionIntervalAttempts = 500U;
        std::uint64_t rejectionCooldownAttempts = 10000U;
        std::uint64_t clrpTrialAttempts = 2000U;
        std::uint64_t twTrialAttempts = 1000U;
        std::uint64_t protectedPrimaryPeriod = 10U;
        double clrpLatestLaunchFraction = 0.9;
        std::size_t eliteMeanCount = 10U;
        std::size_t targetMinimumSize = 6U;
        std::size_t targetMaximumSize = 10U;
        std::size_t targetPopulationDivisor = 3U;
        std::size_t maximumConstructionMultiplier = 3U;
        std::size_t minimumViableNumerator = 3U;
        std::size_t minimumViableDenominator = 5U;
        std::size_t archiveCapacity = 64U;
        std::size_t archiveTopObjectiveCount = 3U;
        std::size_t archiveSeedCount = 4U;
        std::uint64_t repeatedDiscoveryThreshold = 3U;
        std::uint64_t newObservationsAfterRejection = 3U;
        std::size_t maximumCompletedTrials = 4U;
    } challenger;

    struct SetPartitioning {
        struct Trigger {
            int globalScheduledFrequency = 50001;
            int localScheduledFrequency = 10000;
            int adaptiveMinimumInterval = 5000;
            int crossScopeLocalMinimumInterval = 500;
            int adaptiveLocalNewColumns = 250;
            int adaptiveLocalStagnation = 5000;
            int adaptiveGlobalStagnation = 20000;
            double adaptiveUtilityEmaWeight = 0.25;
            double adaptiveHighUtilityThreshold = 1.0e-3;
            double finalPhaseStartFraction = 0.9;
            int globalMinimumInterval = 10000;
            int localZeroUtilityPatience = 2;
            int globalZeroUtilityPatience = 1;
        } trigger;

        struct PopulationLocal {
            int stagnationAttempts = 10000;
            int minimumAgeAttempts = 5000;
            int minimumNewCustomerSets = 250;
            double minimumReplacementGainRatio = 1.0e-4;
            double lpTimeLimitSeconds = 2.0;
            double mipTimeLimitSeconds = 3.0;
            int maximumCalls = 3;
            int familyMinimumIntervalAttempts = 750;
            double familyCpuShareLimit = 0.08;
            int injectionCount = 1;
        } populationLocal;

        struct PreReductionGlobal {
            double lpTimeLimitSeconds = 10.0;
            double mipTimeLimitSeconds = 15.0;
            int injectionCount = 3;
        } preReductionGlobal;

        struct Online {
            // Outer call deadlines and stage limits are independent. The old
            // spTimeLimit/spLocalTimeLimit values served both meanings.
            double globalOuterTimeLimitSeconds = 360.0;
            double localOuterTimeLimitSeconds = 30.0;
            double globalLpTimeLimitSeconds = 10.0;
            double localLpTimeLimitSeconds = 10.0;
            double globalMipTimeLimitSeconds = 5.0;
            double localMipTimeLimitSeconds = 2.0;
            int globalInjectionCount = 3;
            int localInjectionCount = 3;
        } online;

        struct FinalGlobal {
            double outerTimeLimitSeconds = 360.0;
            double lpTimeLimitSeconds = 10.0;
            double mipTimeLimitSeconds = 360.0;
            double twMipTimeLimitSeconds = 60.0;
            // Zero disables callback termination. Set F uses a conservative
            // trajectory-screened primal idle window; Set S keeps the full
            // final MIP budget because late improvements were observed.
            double primalIdleTimeLimitSeconds = 0.0;
            int injectionCount = 3;
        } finalGlobal;

        struct CpuBudget {
            double familyShareTarget = 0.15;
            double lpProbeShareTarget = 0.04;
            double localMipShareTarget = 0.02;
            double globalMipShareTarget = 0.05;
            double globalReserveShare = 0.0;
            double mipRescueReserveShare = 0.03;
        } cpuBudget;

        struct SolutionPool {
            int globalSize = 5;
            int localSize = 5;
            double globalGap = 0.02;
            double localGap = 0.02;
            // Global SP is a primal improvement/configuration-discovery arm.
            // Mode 1 retains incidental alternatives without the systematic
            // top-K proof work of mode 2.
            int globalSearchMode = 1;
            int localSearchMode = 1;
        } solutionPool;

        int routePoolCapacity = 50000;
        int mipFocus = 1;
        double heuristics = 0.15;
    } sp;
};

// The V2 series promotes the independently confirmed P9 primary candidates to
// profile-specific defaults. Fields not listed below intentionally retain the
// canonical baseline above; fixed algorithm semantics and resource controls
// remain outside SearchParameters.
inline SearchParameters makeV2SetSDefaultSearchParameters() noexcept {
    SearchParameters parameters;
    parameters.population.minimumSize = 20;
    parameters.population.offspringCount = 21;
    parameters.population.diversityNearestCount = 3;
    parameters.population.eliteCount = 2;
    parameters.population.staticFixedCount = 8;
    parameters.configurationPortfolio.populationReduction1SurvivorCount = 8;
    parameters.configurationPortfolio.populationReduction2SurvivorCount = 5;
    parameters.mutation.probabilityPercent = 15;
    parameters.mutation.ruinFraction = 0.218795;
    parameters.mutation.shawRankExponent = 3.002228;
    parameters.localSearch.customerCandidateCount = 11;
    parameters.localSearch.depotCandidateCount = 1;
    parameters.localSearch.candidateShuffleDenominator = 18;
    parameters.challenger.selectionMinimumAttempt = 3016U;
    parameters.challenger.selectionIntervalAttempts = 310U;
    parameters.challenger.clrpTrialAttempts = 3053U;
    parameters.challenger.protectedPrimaryPeriod = 5U;
    parameters.sp.populationLocal.minimumNewCustomerSets = 175;
    parameters.sp.populationLocal.maximumCalls = 4;
    parameters.sp.populationLocal.familyCpuShareLimit = 0.10;
    parameters.sp.cpuBudget.familyShareTarget = 0.15;
    parameters.sp.cpuBudget.lpProbeShareTarget = 0.04;
    parameters.sp.cpuBudget.localMipShareTarget = 0.02;
    parameters.sp.cpuBudget.globalMipShareTarget = 0.05;
    parameters.sp.finalGlobal.primalIdleTimeLimitSeconds = 0.0;
    parameters.sp.routePoolCapacity = 50000;
    return parameters;
}

inline SearchParameters makeV2SetFDefaultSearchParameters() noexcept {
    SearchParameters parameters;
    parameters.population.minimumSize = 28;
    parameters.population.offspringCount = 51;
    parameters.population.diversityNearestCount = 6;
    parameters.population.eliteCount = 8;
    parameters.population.staticFixedCount = 10;
    parameters.configurationPortfolio.populationReduction1SurvivorCount = 5;
    parameters.configurationPortfolio.populationReduction2SurvivorCount = 3;
    parameters.mutation.probabilityPercent = 21;
    parameters.mutation.ruinFraction = 0.288287;
    parameters.mutation.shawRankExponent = 4.983714;
    parameters.localSearch.customerCandidateCount = 10;
    parameters.localSearch.depotCandidateCount = 7;
    parameters.localSearch.candidateShuffleDenominator = 11;
    parameters.challenger.selectionMinimumAttempt = 4633U;
    parameters.challenger.selectionIntervalAttempts = 447U;
    parameters.challenger.twTrialAttempts = 1269U;
    parameters.challenger.protectedPrimaryPeriod = 17U;
    parameters.sp.populationLocal.minimumNewCustomerSets = 100;
    parameters.sp.populationLocal.maximumCalls = 2;
    parameters.sp.populationLocal.familyCpuShareLimit = 0.08;
    parameters.sp.cpuBudget.familyShareTarget = 0.15;
    parameters.sp.cpuBudget.lpProbeShareTarget = 0.04;
    parameters.sp.cpuBudget.localMipShareTarget = 0.02;
    parameters.sp.cpuBudget.globalMipShareTarget = 0.05;
    parameters.sp.finalGlobal.primalIdleTimeLimitSeconds = 10.0;
    parameters.sp.routePoolCapacity = 50000;
    return parameters;
}

// Parameter-compatibility profile for the shared, source-auditable HGAMP2 Set
// S controls. This is not trajectory compatibility: HGA-SP retains its current
// mutation implementation and extension pipeline. HGA-SP-only controls without
// a one-to-one HGAMP2 counterpart intentionally retain their canonical values.
// The 14% value matches HGAMP2's actual `RandomInt(0, 99) > 85` implementation
// (14 outcomes), rather than its nominal 15% description.
inline SearchParameters makeHgamp2CompatibleSetSSearchParameters() noexcept {
    SearchParameters parameters;
    parameters.profile =
        SearchParameterProfile::Hgamp2ParameterCompatibleSetS;
    parameters.population.minimumSize = 30;
    parameters.population.offspringCount = 30;
    parameters.population.diversityNearestCount = 5;
    parameters.population.eliteCount = 4;
    parameters.population.staticFixedCount = 15;
    parameters.population.residentProtectionAttempts = 10000;
    parameters.population.maximumIterationsPerConfiguration = 500;
    parameters.configurationPortfolio.populationReduction1SurvivorCount = 8;
    parameters.configurationPortfolio.populationReduction2SurvivorCount = 5;
    parameters.mutation.probabilityPercent = 14;
    parameters.mutation.policy = MutationPolicy::ShawOnly;
    parameters.mutation.ruinFraction = 0.25;
    parameters.mutation.shawRankExponent = 6.0;
    parameters.localSearch.constructionCorrelationAlpha = 20;
    // The compatible candidate policy consumes the customer-origin,
    // all-vertex-ranked symmetric alpha-union and the instance-sized
    // half-plus-one depot rule. These stored widths remain explicit
    // provenance but do not truncate the derived lists.
    parameters.localSearch.customerCandidateCount = 20;
    parameters.localSearch.depotCandidateCount = 20;
    parameters.localSearch.candidateShuffleDenominator = 20;
    parameters.restart.stagnationAttempts = 70000;
    parameters.restart.minimumRemainingAttempts = 50000;
    parameters.restart.minimumRemainingCpuSeconds = 600.0;
    return parameters;
}

inline SearchParameters makeV3SetSSearchParameters() noexcept {
    SearchParameters parameters;
    parameters.profile = SearchParameterProfile::V3SetS;
    parameters.population.minimumSize = 30;
    parameters.population.offspringCount = 30;
    parameters.population.diversityNearestCount = 5;
    parameters.population.eliteCount = 4;
    parameters.population.staticFixedCount = 15;
    // Exact fixed-configuration education is substantially more expensive
    // than HGAMP2's unrestricted initialization LS. One candidate per target
    // member bounds startup cost; ordinary offspring supplies later diversity.
    parameters.population.initialCandidateMultiplier = 1;
    // Educated screening seeds make a 500-attempt warm-up per configuration
    // redundant. Bound pre-horizon work across the 15 fixed populations.
    parameters.population.maximumIterationsPerConfiguration = 100;
    parameters.configurationPortfolio.populationReduction1SurvivorCount = 8;
    parameters.configurationPortfolio.populationReduction2SurvivorCount = 5;
    parameters.configurationPortfolio.scheduledConfigurationRotation = true;
    // Role-aware R3/V4 search executes Global-SP synchronously at each
    // reduction barrier.
    parameters.configurationPortfolio.preReductionGlobal1Fraction = 0.50;
    parameters.configurationPortfolio.preReductionGlobal2Fraction = 0.75;
    parameters.configurationPortfolio.absorptionFraction = 0.0;
    parameters.configurationPortfolio.screeningCheapLimit = 500;
    parameters.configurationPortfolio.screeningFirstStage = 50;
    parameters.configurationPortfolio.screeningFinalists = 20;
    parameters.configurationPortfolio.evaluationFirstSamples = 3;
    parameters.configurationPortfolio.evaluationTotalSamples = 15;
    parameters.mutation.probabilityPercent = 14;
    parameters.mutation.policy = MutationPolicy::ShawOnly;
    parameters.mutation.ruinFraction = 0.25;
    parameters.mutation.shawRankExponent = 6.0;
    parameters.localSearch.customerCandidateCount = 20;
    // Keep a compact depot promising set: broader than V2 beta=1, without
    // repeating every Split/depot move for half of a 25--30 depot instance.
    parameters.localSearch.depotCandidateCount = 5;
    parameters.localSearch.candidateShuffleDenominator = 20;
    parameters.localSearch.temporalPenaltyWeight = 0.0;
    parameters.localSearch.waitingTimeWeight = 0.0;
    parameters.challenger.selectionMinimumAttempt = 5000U;
    parameters.challenger.selectionIntervalAttempts = 500U;
    parameters.challenger.clrpTrialAttempts = 500U;
    parameters.challenger.targetMinimumSize = 10U;
    parameters.challenger.targetMaximumSize = 10U;
    parameters.challenger.maximumCompletedTrials = 4U;
    parameters.sp.populationLocal.minimumNewCustomerSets = 175;
    parameters.sp.populationLocal.maximumCalls = 2;
    parameters.sp.populationLocal.familyMinimumIntervalAttempts = 5000;
    parameters.sp.populationLocal.familyCpuShareLimit = 0.02;
    parameters.sp.trigger.finalPhaseStartFraction = 0.95;
    parameters.sp.finalGlobal.primalIdleTimeLimitSeconds = 0.0;
    return parameters;
}

inline SearchParameters makeV3SetFSearchParameters() noexcept {
    SearchParameters parameters = makeV2SetFDefaultSearchParameters();
    parameters.profile = SearchParameterProfile::V3SetF;
    parameters.configurationPortfolio.preReductionGlobal1Fraction = 0.50;
    parameters.configurationPortfolio.preReductionGlobal2Fraction = 0.75;
    parameters.configurationPortfolio.absorptionFraction = 0.0;
    parameters.configurationPortfolio.scheduledConfigurationRotation = true;
    parameters.localSearch.temporalPenaltyWeight = 1.0;
    parameters.localSearch.waitingTimeWeight = 0.2;
    parameters.challenger.maximumCompletedTrials = 2U;
    parameters.sp.populationLocal.maximumCalls = 2;
    parameters.sp.populationLocal.familyMinimumIntervalAttempts = 5000;
    parameters.sp.populationLocal.familyCpuShareLimit = 0.02;
    parameters.sp.trigger.finalPhaseStartFraction = 0.95;
    return parameters;
}

// V4.0 intentionally carries the accepted V3.4-R3 numerical presets forward.
// Separate profile IDs make the executable/configuration provenance explicit;
// they do not imply a parameter retuning.
inline SearchParameters makeV4SetSSearchParameters() noexcept {
    SearchParameters parameters = makeV3SetSSearchParameters();
    parameters.profile = SearchParameterProfile::V4SetS;
    return parameters;
}

inline SearchParameters makeV4SetFSearchParameters() noexcept {
    SearchParameters parameters = makeV3SetFSearchParameters();
    parameters.profile = SearchParameterProfile::V4SetF;
    return parameters;
}

inline SearchParameterProfile currentSearchParameterProfile(
        const BenchmarkFamily family) noexcept {
    return family == BenchmarkFamily::SetF
        ? SearchParameterProfile::V4SetF
        : SearchParameterProfile::V4SetS;
}

inline SearchParameters makeV2DefaultSearchParameters(
    const bool hardTimeWindows,
    const SearchParameterProfile profile =
        SearchParameterProfile::V2P9) {
    if (profile == SearchParameterProfile::Hgamp2ParameterCompatibleSetS) {
        if (hardTimeWindows) {
            throw std::invalid_argument(
                "hgamp2-parameter-compatible-set-s cannot be used with hard "
                "time windows");
        }
        return makeHgamp2CompatibleSetSSearchParameters();
    }
    if (profile == SearchParameterProfile::V3SetS) {
        if (hardTimeWindows) {
            throw std::invalid_argument(
                "v3-set-s cannot be used with hard time windows");
        }
        return makeV3SetSSearchParameters();
    }
    if (profile == SearchParameterProfile::V3SetF) {
        if (!hardTimeWindows) {
            throw std::invalid_argument(
                "v3-set-f requires hard time windows");
        }
        return makeV3SetFSearchParameters();
    }
    if (profile == SearchParameterProfile::V4SetS) {
        if (hardTimeWindows) {
            throw std::invalid_argument(
                "v4-set-s cannot be used with hard time windows");
        }
        return makeV4SetSSearchParameters();
    }
    if (profile == SearchParameterProfile::V4SetF) {
        if (!hardTimeWindows) {
            throw std::invalid_argument(
                "v4-set-f requires hard time windows");
        }
        return makeV4SetFSearchParameters();
    }
    return hardTimeWindows ? makeV2SetFDefaultSearchParameters()
                           : makeV2SetSDefaultSearchParameters();
}

inline SearchParameters makeSearchParameters(
        const BenchmarkFamily family,
        const SearchParameterProfile profile) {
    const bool hardTimeWindows = family == BenchmarkFamily::SetF;
    if ((isSetSProductionSearchProfile(profile) ||
         profile == SearchParameterProfile::
             Hgamp2ParameterCompatibleSetS) &&
        family != BenchmarkFamily::SetS) {
        throw std::invalid_argument(
            std::string(toString(profile)) + " requires Set S");
    }
    if (isSetFProductionSearchProfile(profile) &&
        family != BenchmarkFamily::SetF) {
        throw std::invalid_argument(
            std::string(toString(profile)) + " requires Set F");
    }
    return makeV2DefaultSearchParameters(hardTimeWindows, profile);
}

}  // namespace hgasp

#endif  // HGASP_CORE_SEARCH_PARAMETERS_H_
