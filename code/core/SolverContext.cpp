/******************************************************************************
 * File: SolverContext.cpp
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description:
 *   Owns one solver run's problem data, resolved configuration, and search
 *   state. Parsing itself is delegated to code/io.
 ******************************************************************************/

#include "SolverContext.h"
#include "basic.h"
#include "Individual.h"
#include "InstanceLoader.h"
#include "NumericPolicy.h"
#include "RouteEvaluator.h"
#include "StaticHgampPopulationPolicy.h"
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>

namespace {

bool environmentHasValue(const char* name) {
    const char* raw = std::getenv(name);
    return raw != nullptr && *raw != '\0';
}

int environmentInt(
    const char* name,
    const int fallback,
    const int minimum,
    const int maximum) {
    const char* raw = std::getenv(name);
    if (raw == nullptr || *raw == '\0') {
        return fallback;
    }
    errno = 0;
    char* end = nullptr;
    const long value = std::strtol(raw, &end, 10);
    if (errno != 0 || end == raw || *end != '\0' ||
        value < minimum || value > maximum) {
        throw std::invalid_argument(
            std::string(name) + " is outside its accepted integer range");
    }
    return static_cast<int>(value);
}

double environmentDouble(
    const char* name,
    const double fallback,
    const double minimum,
    const double maximum) {
    const char* raw = std::getenv(name);
    if (raw == nullptr || *raw == '\0') {
        return fallback;
    }
    errno = 0;
    char* end = nullptr;
    const double value = std::strtod(raw, &end);
    if (errno != 0 || end == raw || *end != '\0' ||
        !std::isfinite(value) || value < minimum || value > maximum) {
        throw std::invalid_argument(
            std::string(name) + " is outside its accepted numeric range");
    }
    return value;
}

bool environmentBool(const char* name, const bool fallback) {
    return environmentInt(name, fallback ? 1 : 0, 0, 1) == 1;
}

double euclideanDistance(
    const double fromX,
    const double fromY,
    const double toX,
    const double toY) {
    return std::sqrt(
        (fromX - toX) * (fromX - toX) +
        (fromY - toY) * (fromY - toY));
}

double applyRounding(
    const double value,
    const hgasp::NumericRounding rounding) {
    if (rounding == hgasp::NumericRounding::CeilAfterScalingBy100) {
        return std::ceil(value * 100.0);
    }
    if (rounding == hgasp::NumericRounding::Floor) {
        // Coordinates in the published Farham files are integral.  The
        // tiny guard prevents floating-point sqrt error from mapping an
        // exact integer distance k to k-1, without affecting a genuinely
        // non-integral distance on this coordinate scale.
        return std::floor(value + 1.0e-12);
    }
    return value;
}

}  // namespace

SolverContext::SolverContext() {
    // Give embedded/test callers a valid process-CPU origin even when they do
    // not go through main().  main() resets this immediately before the
    // algorithm starts, so published search timings exclude instance parsing.
    startTime = clock();
    meetBestTime = 0.0;
    numV = 0;
    numL = 0;
    numC = 0;
	configureProblemProfile(
		hgasp::makeSetSProblemProfile(
			hgasp::InstanceFormat::LegacyDat));
}

void SolverContext::configureProblemProfile(
	hgasp::ProblemProfile profile) {
	invalidateFinalizedProblemData();
	problem = std::make_unique<hgasp::ProblemContext>(
		std::move(profile));
	instanceFormat = problem->profile().format;
}

const hgasp::ProblemContext& SolverContext::problemContext() const {
	if (problem == nullptr) {
		throw std::logic_error(
			"problem context has not been configured");
	}
	return *problem;
}

const hgasp::ProblemProfile& SolverContext::problemProfile() const {
	return problemContext().profile();
}

bool SolverContext::hasHardTimeWindows() const noexcept {
	return problem != nullptr && problem->hasHardTimeWindows();
}

void SolverContext::invalidateFinalizedProblemData() noexcept {
	problemDataFinalizedForSearch_ = false;
	temporalProblemDataFinalizedForSearch_ = false;
	representationRouteLimits_.clear();
}

bool SolverContext::problemDataFinalizedForSearch() const noexcept {
	return problemDataFinalizedForSearch_;
}

bool SolverContext::temporalProblemDataFinalizedForSearch() const noexcept {
	return temporalProblemDataFinalizedForSearch_;
}

std::uint64_t SolverContext::problemDataRevisionForSearch() const noexcept {
	return problemDataFinalizedForSearch_
		? problemDataRevisionForSearch_
		: 0U;
}

int SolverContext::cachedRepresentationRouteLimit(
		const int depotIndex) const {
	if (!problemDataFinalizedForSearch_) {
		throw std::logic_error(
			"representation route-limit cache is not finalized");
	}
	if (depotIndex < 0 ||
		depotIndex >= static_cast<int>(representationRouteLimits_.size())) {
		throw std::out_of_range(
			"representation route-limit cache depot is out of range");
	}
	return representationRouteLimits_[
		static_cast<std::size_t>(depotIndex)];
}

void SolverContext::finalizeProblemDataForSearch() {
	invalidateFinalizedProblemData();
	const bool requireTemporal = hasHardTimeWindows();
	RouteEvaluator::validateProblemDataForTrustedEvaluation(
		*this, requireTemporal);
	std::vector<int> routeLimits;
	routeLimits.reserve(static_cast<std::size_t>(numL));
	long long totalSlots = 0;
	for (int depot = 0; depot < numL; ++depot) {
		const int limit =
			Individual::representationRouteLimit(*this, depot);
		routeLimits.push_back(limit);
		totalSlots += limit;
		if (totalSlots > std::numeric_limits<int>::max()) {
			throw std::overflow_error(
				"representation route-slot count exceeds integer range");
		}
	}
	representationRouteLimits_ = std::move(routeLimits);
	maxVnum = static_cast<int>(totalSlots);
	if (problemDataRevisionForSearch_ ==
		std::numeric_limits<std::uint64_t>::max()) {
		throw std::overflow_error(
			"problem-data search revision exhausted");
	}
	++problemDataRevisionForSearch_;
	temporalProblemDataFinalizedForSearch_ = requireTemporal;
	problemDataFinalizedForSearch_ = true;
}

const std::vector<std::vector<int>>&
SolverContext::depotConfigurationSeeds() const noexcept {
	return depotCon;
}

void SolverContext::applyEnvironmentOverrides() {
	alpha = environmentInt(
		"HGASP_CONSTRUCTION_CORRELATION_ALPHA",
		alpha,
		1,
		std::numeric_limits<int>::max());
	localSearchCustomerAlpha = environmentInt(
		"HGASP_LOCAL_SEARCH_CUSTOMER_ALPHA",
		localSearchCustomerAlpha,
		1,
		std::numeric_limits<int>::max());
	localSearchDepotBeta = environmentInt(
		"HGASP_LOCAL_SEARCH_DEPOT_BETA",
		localSearchDepotBeta,
		1,
		std::numeric_limits<int>::max());
	if (parameters.profile ==
			hgasp::SearchParameterProfile::Hgamp2ParameterCompatibleSetS &&
		(localSearchCustomerAlpha != 20 || localSearchDepotBeta != 20)) {
		throw std::invalid_argument(
			"hgamp2-parameter-compatible-set-s fixes "
			"local_search_customer_alpha and local_search_depot_beta because "
			"its candidate topology is derived from the shared alpha graph "
			"and instance depot count");
	}
	nbClost = environmentInt(
		"HGASP_POPULATION_DIVERSITY_NEAREST_COUNT",
		nbClost,
		1,
		std::numeric_limits<int>::max());
	nbElite = environmentInt(
		"HGASP_POPULATION_ELITE_COUNT",
		nbElite,
		1,
		std::numeric_limits<int>::max());
	popMin = environmentInt(
		"HGASP_POPULATION_MINIMUM_SIZE",
		popMin,
		1,
		std::numeric_limits<int>::max());
	parameters.population.offspringCount = environmentInt(
		"HGASP_POPULATION_OFFSPRING_COUNT",
		parameters.population.offspringCount,
		1,
		std::numeric_limits<int>::max() - popMin);
	if (nbElite > popMin) {
		throw std::invalid_argument(
			"population elite count must not exceed the minimum size");
	}
	mutationProbabilityPercent = environmentInt(
		"HGASP_MUTATION_PROBABILITY_PERCENT",
		mutationProbabilityPercent,
		0,
		100);
	ratioPerturbLength = environmentDouble(
		"HGASP_MUTATION_RUIN_FRACTION",
		ratioPerturbLength,
		0.0,
		1.0);
	determine_para = environmentDouble(
		"HGASP_MUTATION_SHAW_RANK_EXPONENT",
		determine_para,
		std::numeric_limits<double>::min(),
		std::numeric_limits<double>::max());
	maxIterNonProd = environmentInt(
		"HGASP_RESTART_STAGNATION_ATTEMPTS",
		maxIterNonProd,
		1,
		std::numeric_limits<int>::max());
	maxIterationPerLoc = environmentInt(
		"HGASP_POPULATION_MAXIMUM_ITERATIONS_PER_CONFIGURATION",
		maxIterationPerLoc,
		1,
		std::numeric_limits<int>::max());
	initialPopulationCandidateMultiplier = environmentInt(
		"HGASP_POPULATION_INITIAL_CANDIDATE_MULTIPLIER",
		initialPopulationCandidateMultiplier,
		1,
		1000);
	populationIncubationMinimumConstructionAttempts = environmentInt(
		"HGASP_POPULATION_INCUBATION_MINIMUM_CONSTRUCTION_ATTEMPTS",
		populationIncubationMinimumConstructionAttempts,
		1,
		std::numeric_limits<int>::max());
	populationIncubationTargetConstructionMultiplier = environmentInt(
		"HGASP_POPULATION_INCUBATION_TARGET_CONSTRUCTION_MULTIPLIER",
		populationIncubationTargetConstructionMultiplier,
		1,
		1000);
	populationBuildNoProgressMinimumAttempts = environmentInt(
		"HGASP_POPULATION_BUILD_NO_PROGRESS_MINIMUM_ATTEMPTS",
		populationBuildNoProgressMinimumAttempts,
		1,
		std::numeric_limits<int>::max());
	populationBuildNoProgressTargetMultiplier = environmentInt(
		"HGASP_POPULATION_BUILD_NO_PROGRESS_TARGET_MULTIPLIER",
		populationBuildNoProgressTargetMultiplier,
		1,
		1000);
	populationPromotionConstructionMultiplier = environmentInt(
		"HGASP_POPULATION_PROMOTION_CONSTRUCTION_MULTIPLIER",
		populationPromotionConstructionMultiplier,
		1,
		1000);
	parameters.mutation.insertionCandidateListWidth = environmentInt(
		"HGASP_MUTATION_INSERTION_CANDIDATE_LIST_WIDTH",
		parameters.mutation.insertionCandidateListWidth,
		1,
		1000);
	parameters.mutation.insertionRankExponent = environmentDouble(
		"HGASP_MUTATION_INSERTION_RANK_EXPONENT",
		parameters.mutation.insertionRankExponent,
		std::numeric_limits<double>::min(),
		1000.0);
	parameters.mutation.recreateAttempts = environmentInt(
		"HGASP_MUTATION_RECREATE_ATTEMPTS",
		parameters.mutation.recreateAttempts,
		1,
		std::numeric_limits<int>::max());
	parameters.localSearch.candidateShuffleDenominator = environmentInt(
		"HGASP_LOCAL_SEARCH_CANDIDATE_SHUFFLE_DENOMINATOR",
		parameters.localSearch.candidateShuffleDenominator,
		1,
		std::numeric_limits<int>::max());
	parameters.construction.insertionCandidateListWidth = environmentInt(
		"HGASP_CONSTRUCTION_INSERTION_CANDIDATE_LIST_WIDTH",
		parameters.construction.insertionCandidateListWidth,
		1,
		1000);
	parameters.construction.insertionRankExponent = environmentDouble(
		"HGASP_CONSTRUCTION_INSERTION_RANK_EXPONENT",
		parameters.construction.insertionRankExponent,
		std::numeric_limits<double>::min(),
		1000.0);
	parameters.construction.exactInsertionCandidateListWidth = environmentInt(
		"HGASP_CONSTRUCTION_EXACT_INSERTION_CANDIDATE_LIST_WIDTH",
		parameters.construction.exactInsertionCandidateListWidth,
		1,
		1000);
	parameters.construction.exactInsertionRankExponent = environmentDouble(
		"HGASP_CONSTRUCTION_EXACT_INSERTION_RANK_EXPONENT",
		parameters.construction.exactInsertionRankExponent,
		std::numeric_limits<double>::min(),
		1000.0);
	parameters.construction.depotSelectionRankExponent = environmentDouble(
		"HGASP_CONSTRUCTION_DEPOT_SELECTION_RANK_EXPONENT",
		parameters.construction.depotSelectionRankExponent,
		std::numeric_limits<double>::min(),
		1000.0);

    const char* eaxMode = std::getenv("HGASP_EAX_PARENT_MODE");
    if (eaxMode != nullptr && *eaxMode != '\0') {
        eaxParentMode = hgasp::parseEaxParentMode(eaxMode);
    }

    const char* populationModeValue =
        std::getenv("HGASP_SP_POPULATION_MODE");
    const bool populationModeExplicit =
        populationModeValue != nullptr && *populationModeValue != '\0';
    if (populationModeExplicit) {
        populationMode = hgasp::parseConfigurationPopulationMode(
            populationModeValue);
    }
    const char* configurationRankerValue =
        std::getenv("HGASP_SP_CONFIGURATION_RANKER");
    if (configurationRankerValue != nullptr &&
        *configurationRankerValue != '\0') {
        configurationRanker = hgasp::parseConfigurationRankerMode(
            configurationRankerValue);
    }

    staticFixedPopulationCount = environmentInt(
        "HGASP_SP_STATIC_FIXED_POPULATION_COUNT",
        staticFixedPopulationCount,
        1,
        1000);
    staticFixedPopulationCount = environmentInt(
        "HGASP_POPULATION_STATIC_FIXED_COUNT",
        staticFixedPopulationCount,
        1,
        1000);
    staticPopulationMinimumAgeAttempts = environmentInt(
        "HGASP_SP_STATIC_POPULATION_MINIMUM_AGE_ATTEMPTS",
        staticPopulationMinimumAgeAttempts,
        0,
        std::numeric_limits<int>::max());
    // The legacy environment value intentionally fans out to all three
    // controls. Mixing it with a split value would make the provenance
    // ambiguous, so reject that at the C++ boundary as well as in the runner.
    if (environmentHasValue(
            "HGASP_SP_STATIC_POPULATION_REPLACEMENT_PROTECTION_ATTEMPTS") &&
        (environmentHasValue(
             "HGASP_POPULATION_RESIDENT_POST_PROMOTION_PROTECTION_ATTEMPTS") ||
         environmentHasValue(
             "HGASP_CHALLENGER_REJECTION_COOLDOWN_ATTEMPTS") ||
         environmentHasValue(
             "HGASP_RETIRED_CONFIGURATION_COOLDOWN_ATTEMPTS"))) {
        throw std::invalid_argument(
            "legacy replacement protection cannot be mixed with split "
            "protection/cooldown controls");
    }
    const int legacyReplacementProtectionAttempts = environmentInt(
        "HGASP_SP_STATIC_POPULATION_REPLACEMENT_PROTECTION_ATTEMPTS",
        residentPostPromotionProtectionAttempts,
        0,
        std::numeric_limits<int>::max());
    residentPostPromotionProtectionAttempts = environmentInt(
        "HGASP_POPULATION_RESIDENT_POST_PROMOTION_PROTECTION_ATTEMPTS",
        legacyReplacementProtectionAttempts,
        0,
        std::numeric_limits<int>::max());
    parameters.challenger.rejectionCooldownAttempts =
        static_cast<std::uint64_t>(environmentInt(
            "HGASP_CHALLENGER_REJECTION_COOLDOWN_ATTEMPTS",
            legacyReplacementProtectionAttempts,
            0,
            std::numeric_limits<int>::max()));
    retiredConfigurationCooldownAttempts = environmentInt(
        "HGASP_RETIRED_CONFIGURATION_COOLDOWN_ATTEMPTS",
        legacyReplacementProtectionAttempts,
        0,
        std::numeric_limits<int>::max());
    popProtectionPeriod = environmentInt(
        "HGASP_POPULATION_RESIDENT_PROTECTION_ATTEMPTS",
        popProtectionPeriod,
        0,
        std::numeric_limits<int>::max());
    parameters.challenger.selectionMinimumAttempt =
        static_cast<std::uint64_t>(environmentInt(
            "HGASP_CHALLENGER_SELECTION_MINIMUM_ATTEMPT",
            static_cast<int>(
                parameters.challenger.selectionMinimumAttempt),
            0,
            std::numeric_limits<int>::max()));
    parameters.challenger.selectionIntervalAttempts =
        static_cast<std::uint64_t>(environmentInt(
            "HGASP_CHALLENGER_SELECTION_INTERVAL_ATTEMPTS",
            static_cast<int>(
                parameters.challenger.selectionIntervalAttempts),
            1,
            std::numeric_limits<int>::max()));
    parameters.challenger.clrpTrialAttempts =
        static_cast<std::uint64_t>(environmentInt(
            "HGASP_CHALLENGER_CLRP_TRIAL_ATTEMPTS",
            static_cast<int>(parameters.challenger.clrpTrialAttempts),
            1,
            std::numeric_limits<int>::max()));
    parameters.challenger.twTrialAttempts =
        static_cast<std::uint64_t>(environmentInt(
            "HGASP_CHALLENGER_TW_TRIAL_ATTEMPTS",
            static_cast<int>(parameters.challenger.twTrialAttempts),
            1,
            std::numeric_limits<int>::max()));
    parameters.challenger.protectedPrimaryPeriod =
        static_cast<std::uint64_t>(environmentInt(
            "HGASP_CHALLENGER_PROTECTED_PRIMARY_PERIOD",
            static_cast<int>(
                parameters.challenger.protectedPrimaryPeriod),
            1,
            std::numeric_limits<int>::max()));
    parameters.challenger.clrpLatestLaunchFraction = environmentDouble(
        "HGASP_CHALLENGER_CLRP_LATEST_LAUNCH_FRACTION",
        parameters.challenger.clrpLatestLaunchFraction,
        std::numeric_limits<double>::min(),
        1.0);
    parameters.challenger.eliteMeanCount =
        static_cast<std::size_t>(environmentInt(
            "HGASP_CHALLENGER_ELITE_MEAN_COUNT",
            static_cast<int>(parameters.challenger.eliteMeanCount),
            1,
            std::numeric_limits<int>::max()));
    parameters.challenger.targetMinimumSize =
        static_cast<std::size_t>(environmentInt(
            "HGASP_CHALLENGER_TARGET_MINIMUM_SIZE",
            static_cast<int>(parameters.challenger.targetMinimumSize),
            1,
            std::numeric_limits<int>::max()));
    parameters.challenger.targetMaximumSize =
        static_cast<std::size_t>(environmentInt(
            "HGASP_CHALLENGER_TARGET_MAXIMUM_SIZE",
            static_cast<int>(parameters.challenger.targetMaximumSize),
            static_cast<int>(parameters.challenger.targetMinimumSize),
            std::numeric_limits<int>::max()));
    parameters.challenger.targetPopulationDivisor =
        static_cast<std::size_t>(environmentInt(
            "HGASP_CHALLENGER_TARGET_POPULATION_DIVISOR",
            static_cast<int>(
                parameters.challenger.targetPopulationDivisor),
            1,
            std::numeric_limits<int>::max()));
    parameters.challenger.maximumConstructionMultiplier =
        static_cast<std::size_t>(environmentInt(
            "HGASP_CHALLENGER_MAXIMUM_CONSTRUCTION_MULTIPLIER",
            static_cast<int>(
                parameters.challenger.maximumConstructionMultiplier),
            1,
            std::numeric_limits<int>::max()));
    parameters.challenger.minimumViableNumerator =
        static_cast<std::size_t>(environmentInt(
            "HGASP_CHALLENGER_MINIMUM_VIABLE_NUMERATOR",
            static_cast<int>(
                parameters.challenger.minimumViableNumerator),
            1,
            std::numeric_limits<int>::max()));
    parameters.challenger.minimumViableDenominator =
        static_cast<std::size_t>(environmentInt(
            "HGASP_CHALLENGER_MINIMUM_VIABLE_DENOMINATOR",
            static_cast<int>(
                parameters.challenger.minimumViableDenominator),
            static_cast<int>(
                parameters.challenger.minimumViableNumerator),
            std::numeric_limits<int>::max()));
    parameters.challenger.archiveCapacity =
        static_cast<std::size_t>(environmentInt(
            "HGASP_CHALLENGER_ARCHIVE_CAPACITY",
            static_cast<int>(parameters.challenger.archiveCapacity),
            1,
            std::numeric_limits<int>::max()));
    parameters.challenger.archiveTopObjectiveCount =
        static_cast<std::size_t>(environmentInt(
            "HGASP_CHALLENGER_ARCHIVE_TOP_OBJECTIVE_COUNT",
            static_cast<int>(
                parameters.challenger.archiveTopObjectiveCount),
            1,
            std::numeric_limits<int>::max()));
    parameters.challenger.archiveSeedCount =
        static_cast<std::size_t>(environmentInt(
            "HGASP_CHALLENGER_ARCHIVE_SEED_COUNT",
            static_cast<int>(parameters.challenger.archiveSeedCount),
            1,
            std::numeric_limits<int>::max()));
    parameters.challenger.repeatedDiscoveryThreshold =
        static_cast<std::uint64_t>(environmentInt(
            "HGASP_CHALLENGER_REPEATED_DISCOVERY_THRESHOLD",
            static_cast<int>(
                parameters.challenger.repeatedDiscoveryThreshold),
            1,
            std::numeric_limits<int>::max()));
    parameters.challenger.newObservationsAfterRejection =
        static_cast<std::uint64_t>(environmentInt(
            "HGASP_CHALLENGER_NEW_OBSERVATIONS_AFTER_REJECTION",
            static_cast<int>(
                parameters.challenger.newObservationsAfterRejection),
            1,
            std::numeric_limits<int>::max()));
    if (parameters.challenger.archiveTopObjectiveCount >
            parameters.challenger.archiveCapacity ||
        parameters.challenger.archiveSeedCount >
            parameters.challenger.archiveCapacity) {
        throw std::invalid_argument(
            "challenger archive top-objective and seed counts must not "
            "exceed archive capacity");
    }
    singleDynamicChallengerEnabled = environmentBool(
        "HGASP_SP_SINGLE_DYNAMIC_CHALLENGER_ENABLED",
        singleDynamicChallengerEnabled);
    configurationScreeningCheapLimit = environmentInt(
        "HGASP_SP_CONFIGURATION_SCREENING_CHEAP_LIMIT",
        configurationScreeningCheapLimit,
        1,
        std::numeric_limits<int>::max());
	parameters.configurationPortfolio.expansionMultiplier = environmentInt(
		"HGASP_CONFIGURATION_EXPANSION_MULTIPLIER",
		parameters.configurationPortfolio.expansionMultiplier,
		1,
		1000);
	parameters.configurationPortfolio.depotSelectionRankExponent =
		environmentDouble(
			"HGASP_CONFIGURATION_DEPOT_SELECTION_RANK_EXPONENT",
			parameters.configurationPortfolio.depotSelectionRankExponent,
			std::numeric_limits<double>::min(),
			1000.0);
    configurationScreeningFirstStage = environmentInt(
        "HGASP_SP_CONFIGURATION_SCREENING_FIRST_STAGE",
        configurationScreeningFirstStage,
        1,
        std::numeric_limits<int>::max());
    configurationScreeningFinalists = environmentInt(
        "HGASP_SP_CONFIGURATION_SCREENING_FINALISTS",
        configurationScreeningFinalists,
        1,
        std::numeric_limits<int>::max());
    configurationEvaluationFirstSamples = environmentInt(
        "HGASP_SP_CONFIGURATION_EVALUATION_FIRST_SAMPLES",
        configurationEvaluationFirstSamples,
        1,
        std::numeric_limits<int>::max());
    configurationEvaluationTotalSamples = environmentInt(
        "HGASP_SP_CONFIGURATION_EVALUATION_TOTAL_SAMPLES",
        configurationEvaluationTotalSamples,
        1,
        std::numeric_limits<int>::max());
    strictCrossConfigurationMating = environmentBool(
        "HGASP_SP_STRICT_CROSS_CONFIGURATION_MATING",
        strictCrossConfigurationMating);
    crossConfigurationSelectionMaxAttempts = environmentInt(
        "HGASP_SP_CROSS_CONFIGURATION_SELECTION_MAX_ATTEMPTS",
        crossConfigurationSelectionMaxAttempts,
        1,
        static_cast<int>(
            hgasp::kStaticCrossConfigurationSelectionMaxAttempts));
    populationPhaseHorizonAttempts = environmentInt(
        "HGASP_SP_POPULATION_PHASE_HORIZON_ATTEMPTS",
        populationPhaseHorizonAttempts,
        0,
        std::numeric_limits<int>::max());
    preReductionGlobal1Fraction = environmentDouble(
        "HGASP_SP_PRE_REDUCTION_GLOBAL_1_FRACTION",
        preReductionGlobal1Fraction,
        0.0,
        1.0);
    populationReduction1Fraction = environmentDouble(
        "HGASP_SP_POPULATION_REDUCTION_1_FRACTION",
        populationReduction1Fraction,
        0.0,
        1.0);
    populationReduction1SurvivorCount = environmentInt(
        "HGASP_POPULATION_REDUCTION_1_SURVIVOR_COUNT",
        populationReduction1SurvivorCount,
        1,
        staticFixedPopulationCount);
    preReductionGlobal2Fraction = environmentDouble(
        "HGASP_SP_PRE_REDUCTION_GLOBAL_2_FRACTION",
        preReductionGlobal2Fraction,
        0.0,
        1.0);
    populationReduction2Fraction = environmentDouble(
        "HGASP_SP_POPULATION_REDUCTION_2_FRACTION",
        populationReduction2Fraction,
        0.0,
        1.0);
    populationReduction2SurvivorCount = environmentInt(
        "HGASP_POPULATION_REDUCTION_2_SURVIVOR_COUNT",
        populationReduction2SurvivorCount,
        1,
        populationReduction1SurvivorCount);
    populationAbsorptionFraction = environmentDouble(
        "HGASP_SP_POPULATION_ABSORPTION_FRACTION",
        populationAbsorptionFraction,
        0.0,
        1.0);
    populationLocalSpEnabled = environmentBool(
        "HGASP_SP_POPULATION_LOCAL_SP_ENABLED",
        populationLocalSpEnabled);
    populationLocalSpStagnationAttempts = environmentInt(
        "HGASP_SP_POPULATION_LOCAL_SP_STAGNATION_ATTEMPTS",
        populationLocalSpStagnationAttempts,
        1,
        std::numeric_limits<int>::max());
    populationLocalSpMinimumAgeAttempts = environmentInt(
        "HGASP_SP_POPULATION_LOCAL_SP_MINIMUM_AGE_ATTEMPTS",
        populationLocalSpMinimumAgeAttempts,
        0,
        std::numeric_limits<int>::max());
    populationLocalSpMinimumNewCustomerSets = environmentInt(
        "HGASP_SP_POPULATION_LOCAL_SP_MINIMUM_NEW_CUSTOMER_SETS",
        populationLocalSpMinimumNewCustomerSets,
        1,
        std::numeric_limits<int>::max());
    populationLocalSpMinimumReplacementGainRatio = environmentDouble(
        "HGASP_SP_POPULATION_LOCAL_SP_MINIMUM_REPLACEMENT_GAIN_RATIO",
        populationLocalSpMinimumReplacementGainRatio,
        0.0,
        std::numeric_limits<double>::max());
    populationLocalSpLpTimeLimitSeconds = environmentDouble(
        "HGASP_SP_POPULATION_LOCAL_SP_LP_TIME_LIMIT_SECONDS",
        populationLocalSpLpTimeLimitSeconds,
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::max());
    populationLocalSpMipTimeLimitSeconds = environmentDouble(
        "HGASP_SP_POPULATION_LOCAL_SP_MIP_TIME_LIMIT_SECONDS",
        populationLocalSpMipTimeLimitSeconds,
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::max());
    populationLocalSpMaximumCalls = environmentInt(
        "HGASP_SP_POPULATION_LOCAL_SP_MAXIMUM_CALLS",
        populationLocalSpMaximumCalls,
        1,
        std::numeric_limits<int>::max());
    populationLocalSpInjectionCount = environmentInt(
        "HGASP_SP_POPULATION_LOCAL_SP_INJECTION_COUNT",
        populationLocalSpInjectionCount,
        1,
        1000);
    populationLocalSpFamilyMinimumInterval = environmentInt(
        "HGASP_SP_POPULATION_LOCAL_SP_FAMILY_MINIMUM_INTERVAL",
        populationLocalSpFamilyMinimumInterval,
        1,
        std::numeric_limits<int>::max());
    populationLocalSpFamilyCpuShareLimit = environmentDouble(
        "HGASP_SP_POPULATION_LOCAL_SP_FAMILY_CPU_SHARE_LIMIT",
        populationLocalSpFamilyCpuShareLimit,
        0.0,
        1.0);
    preReductionGlobalSpEnabled = environmentBool(
        "HGASP_SP_PRE_REDUCTION_GLOBAL_SP_ENABLED",
        preReductionGlobalSpEnabled);
    preReductionGlobalSpLpTimeLimitSeconds = environmentDouble(
        "HGASP_SP_PRE_REDUCTION_GLOBAL_SP_LP_TIME_LIMIT_SECONDS",
        preReductionGlobalSpLpTimeLimitSeconds,
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::max());
    preReductionGlobalSpMipTimeLimitSeconds = environmentDouble(
        "HGASP_SP_PRE_REDUCTION_GLOBAL_SP_MIP_TIME_LIMIT_SECONDS",
        preReductionGlobalSpMipTimeLimitSeconds,
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::max());
    preReductionGlobalSpInjectionCount = environmentInt(
        "HGASP_SP_PRE_REDUCTION_GLOBAL_SP_INJECTION_COUNT",
        preReductionGlobalSpInjectionCount,
        1,
        1000);
    populationReductionObjectiveWeight = environmentDouble(
        "HGASP_SP_POPULATION_REDUCTION_OBJECTIVE_WEIGHT",
        populationReductionObjectiveWeight,
        0.0,
        1.0);
    populationReductionHgaImprovementWeight = environmentDouble(
        "HGASP_SP_POPULATION_REDUCTION_HGA_IMPROVEMENT_WEIGHT",
        populationReductionHgaImprovementWeight,
        0.0,
        1.0);
    populationReductionDiversityWeight = environmentDouble(
        "HGASP_SP_POPULATION_REDUCTION_DIVERSITY_WEIGHT",
        populationReductionDiversityWeight,
        0.0,
        1.0);
    populationReductionGlobalSpCreditWeight = environmentDouble(
        "HGASP_SP_POPULATION_REDUCTION_GLOBAL_SP_CREDIT_WEIGHT",
        populationReductionGlobalSpCreditWeight,
        0.0,
        1.0);

    if (configurationScreeningFirstStage >
            configurationScreeningCheapLimit ||
        configurationScreeningFinalists >
            configurationScreeningFirstStage ||
        staticFixedPopulationCount >
            configurationScreeningFinalists) {
        throw std::invalid_argument(
            "configuration screening limits must satisfy cheap >= first "
            ">= finalists >= requested fixed populations");
    }
    if (configurationEvaluationFirstSamples >
        configurationEvaluationTotalSamples) {
        throw std::invalid_argument(
            "configuration first-stage samples must not exceed total "
            "samples");
    }
    if (!(preReductionGlobal1Fraction <=
              populationReduction1Fraction &&
          populationReduction1Fraction <
              preReductionGlobal2Fraction &&
          preReductionGlobal2Fraction <=
              populationReduction2Fraction)) {
        throw std::invalid_argument(
            "population phase fractions must be ordered; a Global-SP phase "
            "may coincide with its following reduction barrier");
    }
    if (populationReduction2SurvivorCount >
            populationReduction1SurvivorCount ||
        populationReduction1SurvivorCount > staticFixedPopulationCount) {
        throw std::invalid_argument(
            "population survivor counts must satisfy phase2 <= phase1 <= "
            "static fixed population count");
    }
    if (preReductionGlobal1Fraction +
                populationAbsorptionFraction >
            populationReduction1Fraction + 1.0e-12 ||
        preReductionGlobal2Fraction +
                populationAbsorptionFraction >
            populationReduction2Fraction + 1.0e-12) {
        throw std::invalid_argument(
            "population absorption windows must finish before reduction");
    }
    const double reductionWeightSum =
        populationReductionObjectiveWeight +
        populationReductionHgaImprovementWeight +
        populationReductionDiversityWeight +
        populationReductionGlobalSpCreditWeight;
    if (std::abs(reductionWeightSum - 1.0) > 1.0e-9) {
        throw std::invalid_argument(
            "population reduction weights must sum to one");
    }

    restartMinimumRemainingIterations = environmentInt(
        "HGASP_RESTART_MINIMUM_REMAINING_ITERATIONS",
        restartMinimumRemainingIterations,
        0,
        std::numeric_limits<int>::max());
    restartMinimumRemainingCpuSeconds = environmentDouble(
        "HGASP_RESTART_MINIMUM_REMAINING_CPU_SECONDS",
        restartMinimumRemainingCpuSeconds,
        0.0,
        std::numeric_limits<double>::max());

    spTriggerFrequency = environmentInt(
        "HGASP_SP_TRIGGER_FREQUENCY",
        spTriggerFrequency,
        1,
        std::numeric_limits<int>::max());
    spTriggerOffset = environmentInt(
        "HGASP_SP_TRIGGER_OFFSET",
        spTriggerOffset,
        0,
        std::numeric_limits<int>::max());
    if (spTriggerOffset < 0 ||
        spTriggerOffset >= spTriggerFrequency) {
        throw std::invalid_argument(
            "HGASP_SP_TRIGGER_OFFSET must be non-negative and smaller than "
            "HGASP_SP_TRIGGER_FREQUENCY");
    }
    const double legacyGlobalTimeLimit = environmentDouble(
        "HGASP_SP_TIME_LIMIT_SECONDS",
        spGlobalOuterTimeLimit,
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::max());
    spGlobalOuterTimeLimit = legacyGlobalTimeLimit;
    spFinalGlobalOuterTimeLimit = legacyGlobalTimeLimit;
    spFinalGlobalMipTimeLimit = legacyGlobalTimeLimit;
    spGlobalOuterTimeLimit = environmentDouble(
        "HGASP_SP_GLOBAL_OUTER_TIME_LIMIT_SECONDS",
        spGlobalOuterTimeLimit,
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::max());
    spFinalGlobalOuterTimeLimit = environmentDouble(
        "HGASP_SP_FINAL_GLOBAL_OUTER_TIME_LIMIT_SECONDS",
        spFinalGlobalOuterTimeLimit,
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::max());
    spFinalGlobalMipTimeLimit = environmentDouble(
        "HGASP_SP_FINAL_GLOBAL_MIP_TIME_LIMIT_SECONDS",
        spFinalGlobalMipTimeLimit,
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::max());
    spFinalGlobalPrimalIdleTimeLimit = environmentDouble(
        "HGASP_SP_FINAL_GLOBAL_PRIMAL_IDLE_TIME_LIMIT_SECONDS",
        spFinalGlobalPrimalIdleTimeLimit,
        0.0,
        std::numeric_limits<double>::max());
    spLocalEnabled = environmentBool(
        "HGASP_SP_LOCAL_ENABLED",
        spLocalEnabled);
    spLocalTriggerFrequency = environmentInt(
        "HGASP_SP_LOCAL_TRIGGER_FREQUENCY",
        spLocalTriggerFrequency,
        1,
        std::numeric_limits<int>::max());
    spLocalOuterTimeLimit = environmentDouble(
        "HGASP_SP_LOCAL_TIME_LIMIT_SECONDS",
        spLocalOuterTimeLimit,
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::max());
    spLocalOuterTimeLimit = environmentDouble(
        "HGASP_SP_LOCAL_OUTER_TIME_LIMIT_SECONDS",
        spLocalOuterTimeLimit,
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::max());
    spAdaptiveTriggerEnabled = environmentBool(
        "HGASP_SP_ADAPTIVE_TRIGGER_ENABLED",
        spAdaptiveTriggerEnabled);
    spAdaptiveMinimumInterval = environmentInt(
        "HGASP_SP_ADAPTIVE_MINIMUM_INTERVAL",
        spAdaptiveMinimumInterval,
        1,
        std::numeric_limits<int>::max());
    spCrossScopeLocalMinimumInterval = environmentInt(
        "HGASP_SP_CROSS_SCOPE_LOCAL_MINIMUM_INTERVAL",
        spCrossScopeLocalMinimumInterval,
        1,
        std::numeric_limits<int>::max());
    spAdaptiveLocalNewColumns = environmentInt(
        "HGASP_SP_ADAPTIVE_LOCAL_NEW_COLUMNS",
        spAdaptiveLocalNewColumns,
        1,
        std::numeric_limits<int>::max());
    spAdaptiveLocalStagnation = environmentInt(
        "HGASP_SP_ADAPTIVE_LOCAL_STAGNATION",
        spAdaptiveLocalStagnation,
        1,
        std::numeric_limits<int>::max());
    spAdaptiveGlobalStagnation = environmentInt(
        "HGASP_SP_ADAPTIVE_GLOBAL_STAGNATION",
        spAdaptiveGlobalStagnation,
        spAdaptiveLocalStagnation,
        std::numeric_limits<int>::max());
    spAdaptiveUtilityEmaWeight = environmentDouble(
        "HGASP_SP_ADAPTIVE_UTILITY_EMA_WEIGHT",
        spAdaptiveUtilityEmaWeight,
        std::numeric_limits<double>::min(),
        1.0);
    spAdaptiveHighUtilityThreshold = environmentDouble(
        "HGASP_SP_ADAPTIVE_HIGH_UTILITY_THRESHOLD",
        spAdaptiveHighUtilityThreshold,
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::max());
    spFinalPhaseStartFraction = environmentDouble(
        "HGASP_SP_FINAL_PHASE_START_FRACTION",
        spFinalPhaseStartFraction,
        std::numeric_limits<double>::min(),
        1.0);
    spLpDualFeedbackEnabled = environmentBool(
        "HGASP_SP_LP_DUAL_FEEDBACK_ENABLED",
        spLpDualFeedbackEnabled);
    const double legacyLpTimeLimit = environmentDouble(
        "HGASP_SP_LP_TIME_LIMIT_SECONDS",
        spGlobalLpTimeLimit,
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::max());
    spGlobalLpTimeLimit = legacyLpTimeLimit;
    spLocalLpTimeLimit = legacyLpTimeLimit;
    spFinalGlobalLpTimeLimit = legacyLpTimeLimit;
    spGlobalLpTimeLimit = environmentDouble(
        "HGASP_SP_GLOBAL_LP_TIME_LIMIT_SECONDS",
        spGlobalLpTimeLimit,
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::max());
    spLocalLpTimeLimit = environmentDouble(
        "HGASP_SP_LOCAL_LP_TIME_LIMIT_SECONDS",
        spLocalLpTimeLimit,
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::max());
    spFinalGlobalLpTimeLimit = environmentDouble(
        "HGASP_SP_FINAL_GLOBAL_LP_TIME_LIMIT_SECONDS",
        spFinalGlobalLpTimeLimit,
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::max());
    spPersistentLpEnabled = environmentBool(
        "HGASP_SP_PERSISTENT_LP_ENABLED",
        spPersistentLpEnabled);
    const int legacySolutionPoolSize = environmentInt(
        "HGASP_SP_SOLUTION_POOL_SIZE",
        spGlobalSolutionPoolSize,
        1,
        1000);
    spGlobalSolutionPoolSize = legacySolutionPoolSize;
    spLocalSolutionPoolSize = legacySolutionPoolSize;
    spGlobalSolutionPoolSize = environmentInt(
        "HGASP_SP_GLOBAL_SOLUTION_POOL_SIZE",
        spGlobalSolutionPoolSize,
        1,
        1000);
    spLocalSolutionPoolSize = environmentInt(
        "HGASP_SP_LOCAL_SOLUTION_POOL_SIZE",
        spLocalSolutionPoolSize,
        1,
        1000);
    const double legacySolutionPoolGap = environmentDouble(
        "HGASP_SP_SOLUTION_POOL_GAP",
        spGlobalSolutionPoolGap,
        0.0,
        std::numeric_limits<double>::max());
    spGlobalSolutionPoolGap = legacySolutionPoolGap;
    spLocalSolutionPoolGap = legacySolutionPoolGap;
    spGlobalSolutionPoolGap = environmentDouble(
        "HGASP_SP_GLOBAL_SOLUTION_POOL_GAP",
        spGlobalSolutionPoolGap,
        0.0,
        std::numeric_limits<double>::max());
    spLocalSolutionPoolGap = environmentDouble(
        "HGASP_SP_LOCAL_SOLUTION_POOL_GAP",
        spLocalSolutionPoolGap,
        0.0,
        std::numeric_limits<double>::max());
    const int legacySolutionPoolSearchMode = environmentInt(
        "HGASP_SP_SOLUTION_POOL_SEARCH_MODE",
        spGlobalSolutionPoolSearchMode,
        0,
        2);
    spGlobalSolutionPoolSearchMode = legacySolutionPoolSearchMode;
    spLocalSolutionPoolSearchMode =
        std::min(1, legacySolutionPoolSearchMode);
    spGlobalSolutionPoolSearchMode = environmentInt(
        "HGASP_SP_GLOBAL_SOLUTION_POOL_SEARCH_MODE",
        spGlobalSolutionPoolSearchMode,
        0,
        2);
    spLocalSolutionPoolSearchMode = environmentInt(
        "HGASP_SP_LOCAL_SOLUTION_POOL_SEARCH_MODE",
        spLocalSolutionPoolSearchMode,
        0,
        2);
    const int legacyInjectionCount = environmentInt(
        "HGASP_SP_DIVERSE_INJECTION_COUNT",
        spOnlineGlobalInjectionCount,
        1,
        1000);
    spOnlineGlobalInjectionCount = legacyInjectionCount;
    spOnlineLocalInjectionCount = legacyInjectionCount;
    spFinalGlobalInjectionCount = legacyInjectionCount;
    spOnlineGlobalInjectionCount = environmentInt(
        "HGASP_SP_ONLINE_GLOBAL_INJECTION_COUNT",
        spOnlineGlobalInjectionCount,
        1,
        spGlobalSolutionPoolSize);
    spOnlineLocalInjectionCount = environmentInt(
        "HGASP_SP_ONLINE_LOCAL_INJECTION_COUNT",
        spOnlineLocalInjectionCount,
        1,
        spLocalSolutionPoolSize);
    spFinalGlobalInjectionCount = environmentInt(
        "HGASP_SP_FINAL_GLOBAL_INJECTION_COUNT",
        spFinalGlobalInjectionCount,
        1,
        spGlobalSolutionPoolSize);
    checkpointIntervalAttempts = environmentInt(
        "HGASP_CHECKPOINT_INTERVAL_ATTEMPTS",
        checkpointIntervalAttempts,
        1,
        std::numeric_limits<int>::max());
    spLpSkipMipIfNoPotential = environmentBool(
        "HGASP_SP_LP_SKIP_MIP_IF_NO_POTENTIAL",
        spLpSkipMipIfNoPotential);
    spGlobalMinimumInterval = environmentInt(
        "HGASP_SP_GLOBAL_MINIMUM_INTERVAL",
        spGlobalMinimumInterval,
        1,
        std::numeric_limits<int>::max());
    spLocalZeroUtilityPatience = environmentInt(
        "HGASP_SP_LOCAL_ZERO_UTILITY_PATIENCE",
        spLocalZeroUtilityPatience,
        1,
        std::numeric_limits<int>::max());
    spGlobalZeroUtilityPatience = environmentInt(
        "HGASP_SP_GLOBAL_ZERO_UTILITY_PATIENCE",
        spGlobalZeroUtilityPatience,
        1,
        std::numeric_limits<int>::max());
    spCpuShareTarget = environmentDouble(
        "HGASP_SP_CPU_SHARE_TARGET",
        spCpuShareTarget,
        0.0,
        1.0);
    spLpProbeCpuShareTarget = environmentDouble(
        "HGASP_SP_LP_PROBE_CPU_SHARE_TARGET",
        spLpProbeCpuShareTarget,
        0.0,
        1.0);
    spLocalMipCpuShareTarget = environmentDouble(
        "HGASP_SP_LOCAL_MIP_CPU_SHARE_TARGET",
        spLocalMipCpuShareTarget,
        0.0,
        1.0);
    spGlobalMipCpuShareTarget = environmentDouble(
        "HGASP_SP_GLOBAL_MIP_CPU_SHARE_TARGET",
        spGlobalMipCpuShareTarget,
        0.0,
        1.0);
    spGlobalCpuReserveTarget = environmentDouble(
        "HGASP_SP_GLOBAL_CPU_RESERVE_TARGET",
        spGlobalCpuReserveTarget,
        0.0,
        1.0);
    if (spCpuShareTarget + spGlobalCpuReserveTarget >
        1.0 + 1.0e-12) {
        throw std::invalid_argument(
            "HGASP_SP_CPU_SHARE_TARGET plus "
            "HGASP_SP_GLOBAL_CPU_RESERVE_TARGET must not exceed one");
    }
    spGlobalFirstImprovementEnabled = environmentBool(
        "HGASP_SP_GLOBAL_FIRST_IMPROVEMENT_ENABLED",
        spGlobalFirstImprovementEnabled);
    spGlobalFirstImprovementTimeLimit = environmentDouble(
        "HGASP_SP_GLOBAL_FIRST_IMPROVEMENT_TIME_LIMIT_SECONDS",
        spGlobalFirstImprovementTimeLimit,
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::max());
    spOnlineLocalMipTimeLimit = environmentDouble(
        "HGASP_SP_LOCAL_MICRO_MIP_TIME_LIMIT_SECONDS",
        spOnlineLocalMipTimeLimit,
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::max());
    spOnlineGlobalMipTimeLimit = environmentDouble(
        "HGASP_SP_GLOBAL_MICRO_MIP_TIME_LIMIT_SECONDS",
        spOnlineGlobalMipTimeLimit,
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::max());
    spOnlineLocalMipTimeLimit = environmentDouble(
        "HGASP_SP_ONLINE_LOCAL_MIP_TIME_LIMIT_SECONDS",
        spOnlineLocalMipTimeLimit,
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::max());
    spOnlineGlobalMipTimeLimit = environmentDouble(
        "HGASP_SP_ONLINE_GLOBAL_MIP_TIME_LIMIT_SECONDS",
        spOnlineGlobalMipTimeLimit,
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::max());
    spMipRescueReserveTarget = environmentDouble(
        "HGASP_SP_MIP_RESCUE_RESERVE_TARGET",
        spMipRescueReserveTarget,
        0.0,
        1.0);
    if (spCpuShareTarget + spMipRescueReserveTarget >
        1.0 + 1.0e-12) {
        throw std::invalid_argument(
            "HGASP_SP_CPU_SHARE_TARGET plus "
            "HGASP_SP_MIP_RESCUE_RESERVE_TARGET must not exceed one");
    }
    if (spLpProbeCpuShareTarget > spCpuShareTarget + 1.0e-12 ||
        spLocalMipCpuShareTarget >
            spCpuShareTarget + spMipRescueReserveTarget + 1.0e-12 ||
        spGlobalMipCpuShareTarget >
            spCpuShareTarget +
                std::max(
                    spGlobalCpuReserveTarget,
                    spMipRescueReserveTarget) +
                1.0e-12) {
        throw std::invalid_argument(
            "SP phase CPU-share targets must fit their family and reserve "
            "budgets");
    }
    twProfileEarlyStopEnabled = environmentBool(
        "HGASP_TW_PROFILE_EARLY_STOP_ENABLED",
        twProfileEarlyStopEnabled);
    twEarlyStopMinimumIteration = environmentInt(
        "HGASP_TW_EARLY_STOP_MINIMUM_ITERATION",
        twEarlyStopMinimumIteration,
        1,
        std::numeric_limits<int>::max());
    twEarlyStopMinimumNonImprovingAttempts = environmentInt(
        "HGASP_TW_EARLY_STOP_MINIMUM_NONIMPROVING_ATTEMPTS",
        twEarlyStopMinimumNonImprovingAttempts,
        1,
        std::numeric_limits<int>::max());
    spTwFinalMipTimeLimit = environmentDouble(
        "HGASP_SP_TW_FINAL_MIP_TIME_LIMIT_SECONDS",
        spTwFinalMipTimeLimit,
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::max());
    adaptiveSearchAllocationEnabled = environmentBool(
        "HGASP_ADAPTIVE_SEARCH_ALLOCATION_ENABLED",
        adaptiveSearchAllocationEnabled);
    adaptivePopulationExploration = environmentDouble(
        "HGASP_ADAPTIVE_POPULATION_EXPLORATION",
        adaptivePopulationExploration,
        0.0,
        std::numeric_limits<double>::max());
    adaptivePopulationDiversityWeight = environmentDouble(
        "HGASP_ADAPTIVE_POPULATION_DIVERSITY_WEIGHT",
        adaptivePopulationDiversityWeight,
        0.0,
        std::numeric_limits<double>::max());
    adaptivePopulationColumnWeight = environmentDouble(
        "HGASP_ADAPTIVE_POPULATION_COLUMN_WEIGHT",
        adaptivePopulationColumnWeight,
        0.0,
        std::numeric_limits<double>::max());
    adaptiveSearchEmaWeight = environmentDouble(
        "HGASP_ADAPTIVE_SEARCH_EMA_WEIGHT",
        adaptiveSearchEmaWeight,
        std::numeric_limits<double>::min(),
        1.0);
    adaptiveMutationMinimumPercent = environmentInt(
        "HGASP_ADAPTIVE_MUTATION_MINIMUM_PERCENT",
        adaptiveMutationMinimumPercent,
        0,
        100);
    adaptiveMutationMaximumPercent = environmentInt(
        "HGASP_ADAPTIVE_MUTATION_MAXIMUM_PERCENT",
        adaptiveMutationMaximumPercent,
        adaptiveMutationMinimumPercent,
        100);
    if (preReductionGlobalSpInjectionCount >
            spGlobalSolutionPoolSize ||
        populationLocalSpInjectionCount > spLocalSolutionPoolSize ||
        spOnlineGlobalInjectionCount > spGlobalSolutionPoolSize ||
        spOnlineLocalInjectionCount > spLocalSolutionPoolSize ||
        spFinalGlobalInjectionCount > spGlobalSolutionPoolSize) {
        throw std::invalid_argument(
            "SP injection counts must not exceed their solution-pool size");
    }
    spWorkLimit = environmentDouble(
        "HGASP_SP_WORK_LIMIT",
        spWorkLimit,
        0.0,
        std::numeric_limits<double>::max());
    spMIPGap = environmentDouble(
        "HGASP_SP_MIP_GAP",
        spMIPGap,
        0.0,
        1.0);
    spMaxPoolSize = environmentInt(
        "HGASP_SP_MAX_POOL_SIZE",
        spMaxPoolSize,
        1,
        std::numeric_limits<int>::max());
    spThreads = environmentInt(
        "HGASP_SP_THREADS",
        spThreads,
        1,
        1024);
    spSeed = environmentInt(
        "HGASP_SP_SEED",
        spSeed,
        0,
        2000000000);
    spOutputFlag = environmentInt(
        "HGASP_SP_OUTPUT_FLAG",
        spOutputFlag,
        0,
        1);
    spMIPFocus = environmentInt(
        "HGASP_SP_MIP_FOCUS",
        spMIPFocus,
        0,
        3);
    spHeuristics = environmentDouble(
        "HGASP_SP_HEURISTICS",
        spHeuristics,
        0.0,
        1.0);
    spWLSTokenDurationMinutes = environmentInt(
        "HGASP_SP_WLS_TOKEN_DURATION_MINUTES",
        spWLSTokenDurationMinutes,
        0,
        2000000000);
    spWLSTokenRefresh = environmentDouble(
        "HGASP_SP_WLS_TOKEN_REFRESH",
        spWLSTokenRefresh,
        0.0,
        1.0);
    spClientLogLevel = environmentInt(
        "HGASP_SP_CLIENT_LOG_LEVEL",
        spClientLogLevel,
        0,
        3);
    spServerTimeoutSeconds = environmentInt(
        "HGASP_SP_SERVER_TIMEOUT_SECONDS",
        spServerTimeoutSeconds,
        1,
        2000000000);
    spEnvironmentMaxAttempts = environmentInt(
        "HGASP_SP_ENVIRONMENT_MAX_ATTEMPTS",
        spEnvironmentMaxAttempts,
        1,
        10);
    spEnvironmentRetryBaseMilliseconds = environmentInt(
        "HGASP_SP_ENVIRONMENT_RETRY_BASE_MILLISECONDS",
        spEnvironmentRetryBaseMilliseconds,
        0,
        60000);
    spEnforceRepresentationRouteLimit = environmentBool(
        "HGASP_SP_REPRESENTATION_ROUTE_LIMIT",
        spEnforceRepresentationRouteLimit);

    const char* logFile = std::getenv("HGASP_SP_LOG_FILE");
    if (logFile != nullptr) {
        spLogFile = logFile;
    }
}

void SolverContext::allocateProblemStorage(){
	invalidateFinalizedProblemData();
	dimTabu1.clear();
	dimTabu2.clear();
	if (numV <= 0 || numL <= 0 || numC <= 0 || numC != numV + numL) {
		throw std::invalid_argument(
			"problem dimensions must satisfy customers > 0, depots > 0, "
			"and nodes = customers + depots");
	}

	const std::size_t nodeCount = static_cast<std::size_t>(numC);
	if (nodeCount >
		std::numeric_limits<std::size_t>::max() / nodeCount) {
		throw std::overflow_error("problem matrix dimensions overflow");
	}
	const std::size_t matrixSize = nodeCount * nodeCount;
	distanceStorage_.assign(matrixSize, 0.0);
	timeStorage_.assign(matrixSize, 0.0);
	distanceRows_.resize(nodeCount);
	timeRows_.resize(nodeCount);
	for (int i = 0; i < numC; ++i) {
		distanceRows_[static_cast<std::size_t>(i)] =
			distanceStorage_.data() +
			static_cast<std::size_t>(i) *
			static_cast<std::size_t>(numC);
		timeRows_[static_cast<std::size_t>(i)] =
			timeStorage_.data() +
			static_cast<std::size_t>(i) *
			static_cast<std::size_t>(numC);
	}
	D = distanceRows_.data();
	T = timeRows_.data();
	xStorage_.assign(nodeCount, 0.0);
	yStorage_.assign(nodeCount, 0.0);
	customerDemandStorage_.assign(
		static_cast<std::size_t>(numV), 0);
	depotCapacityStorage_.assign(nodeCount, 0);
	depotOpeningCostStorage_.assign(nodeCount, 0.0);
	x = xStorage_.data();
	y = yStorage_.data();
	demCus = customerDemandStorage_.data();
	invLoc = depotCapacityStorage_.data();
	openCostLoc = depotOpeningCostStorage_.data();
	edgeNode = std::vector< std::vector < int > >(numV);
	customerToLoc = std::vector< std::vector < int > >(numV);
	edgeLocVec = std::vector < std::vector < int > > (numL);
	nearCity = std::vector< std::vector < int > >(numV);
	nearCityLoca = std::vector< std::vector < int > >(numL);
	const std::size_t depotCount = static_cast<std::size_t>(numL);
	if (depotCount >
		std::numeric_limits<std::size_t>::max() / depotCount) {
		throw std::overflow_error("depot conflict-matrix dimensions overflow");
	}
	conflictStorage_.assign(depotCount * depotCount, 0);
	conflictRows_.resize(depotCount);
	for (int i = 0; i < numL; ++i) {
		conflictRows_[static_cast<std::size_t>(i)] =
			conflictStorage_.data() +
			static_cast<std::size_t>(i) * depotCount;
	}
	conflictLoc = conflictRows_.data();
	locIncCus = std::vector<std::set<int>>(numL);
	readyTime.assign(
		static_cast<std::size_t>(numV), 0.0);
	dueTime.assign(
		static_cast<std::size_t>(numV),
		std::numeric_limits<double>::infinity());
	serviceTime.assign(
		static_cast<std::size_t>(numV), 0.0);
	depotOpenTime.assign(
		static_cast<std::size_t>(numL), 0.0);
	depotCloseTime.assign(
		static_cast<std::size_t>(numL),
		std::numeric_limits<double>::infinity());
	originalCustomerIds.resize(static_cast<std::size_t>(numV));
	originalDepotIds.resize(static_cast<std::size_t>(numL));
	for (int customer = 0; customer < numV; ++customer) {
		originalCustomerIds[static_cast<std::size_t>(customer)] = customer;
	}
	for (int depot = 0; depot < numL; ++depot) {
		originalDepotIds[static_cast<std::size_t>(depot)] = depot;
	}
}
void SolverContext::initializeRun(
	std::string pathInstance,
	std::string pathDepotCon,
	std::string pathSolution,
	double bestDis,
	int isOptimal,
		int timeLimit,
		int iterLimit) {
	this->timeLimit = timeLimit;
	this->iterLimit = iterLimit;
	this->bestDis = bestDis;
	this->isOptimal = isOptimal;
	this->pathSolution = pathSolution;
	this->pathDepotCon = pathDepotCon;
	this->pathInstance = pathInstance;

	const hgasp::ParsedInstance parsed =
		hgasp::loadInstance(pathInstance);
	configureProblemProfile(parsed.profile);
	hgasp::SearchParameterProfile searchParameterProfile =
		hgasp::currentSearchParameterProfile(
			parsed.profile.benchmarkFamily);
	const char* profileValue = std::getenv("HGASP_SEARCH_PARAMETER_PROFILE");
	if (profileValue != nullptr && *profileValue != '\0') {
		searchParameterProfile =
			hgasp::parseSearchParameterProfile(profileValue);
	}
	parameters = hgasp::makeSearchParameters(
		parsed.profile.benchmarkFamily,
		searchParameterProfile);
	// Profile defaults must be selected before explicit process overrides so
	// runner/CLI configurations retain precedence and parameter-dependent
	// preprocessing observes the resolved values.
	applyEnvironmentOverrides();
	numV = static_cast<int>(parsed.customers.size());
	numL = static_cast<int>(parsed.depots.size());
	numC = numV + numL;
	vehCap = parsed.vehicleCapacity;
	usingCostVeh = parsed.vehicleFixedCost;
	realOrInteger = parsed.legacyRealOrInteger;
	allocateProblemStorage();

	for (int customer = 0; customer < numV; ++customer) {
		const hgasp::ParsedCustomer& source =
			parsed.customers[static_cast<std::size_t>(customer)];
		x[customer] = source.x;
		y[customer] = source.y;
		demCus[customer] = source.demand;
		readyTime[static_cast<std::size_t>(customer)] =
			source.readyTime;
		dueTime[static_cast<std::size_t>(customer)] = source.dueTime;
		serviceTime[static_cast<std::size_t>(customer)] =
			source.serviceTime;
		originalCustomerIds[static_cast<std::size_t>(customer)] =
			source.originalId;
	}
	for (int depot = 0; depot < numL; ++depot) {
		const hgasp::ParsedDepot& source =
			parsed.depots[static_cast<std::size_t>(depot)];
		const int node = numV + depot;
		x[node] = source.x;
		y[node] = source.y;
		invLoc[node] = source.capacity;
		openCostLoc[node] = source.openingCost;
		depotOpenTime[static_cast<std::size_t>(depot)] =
			source.openTime;
		depotCloseTime[static_cast<std::size_t>(depot)] =
			source.closeTime;
		originalDepotIds[static_cast<std::size_t>(depot)] =
			source.originalId;
	}

	readDepotConfigurations();


	totalDemand=0;
	maxDemand=0;
	for (int i=0; i < numV; i++){
		if (demCus[i] > vehCap) {
			throw std::invalid_argument(
				"instance is infeasible: customer " +
				std::to_string(originalCustomerIds[
					static_cast<std::size_t>(i)]) +
				" demand exceeds vehicle capacity");
		}
		if (demCus[i] > maxDemand)
			maxDemand=demCus[i];
		totalDemand+=demCus[i];
	}
	if (totalDemand <= 0) {
		throw std::invalid_argument(
			"instance must contain a positive total customer demand");
	}
	long long aggregateDepotCapacity = 0;
	for (int depot = 0; depot < numL; ++depot) {
		aggregateDepotCapacity +=
			static_cast<long long>(invLoc[numV + depot]);
	}
	if (aggregateDepotCapacity <
		static_cast<long long>(totalDemand)) {
		throw std::invalid_argument(
			"instance is infeasible: aggregate depot capacity is smaller "
			"than total customer demand");
	}
	normalizaPara = double(numV) / double(totalDemand);
	avgDemand = double(totalDemand) / double(numV);
	//
	// process the initial data

	isConstraintLoc = false;
	for (int i=numV;i<numC;i++){
		if (invLoc[i] <= totalDemand){
			isConstraintLoc = true;
			break;
		}
	}
	/***********************************************************************************************/
	/* Build distinct objective-cost and travel-time matrices. */
	maxDist=0;
	for (int i=0;i<numV;i++){
		for (int j=0;j<numV;j++){
			const double distance =
				euclideanDistance(x[i], y[i], x[j], y[j]);
				D[i][j] = applyRounding(
					distance,
					parsed.profile.matrix.travelCostRounding);
				T[i][j] = applyRounding(
					distance,
					parsed.profile.matrix.travelTimeRounding);
			if (distance > maxDist)
				maxDist=distance;
		}
	}
	for (int i=0; i < numV;i++){
		for (int j=numV; j<numC; j++){
			const double distance =
				euclideanDistance(x[i], y[i], x[j], y[j]);
				D[i][j] = applyRounding(
					distance,
					parsed.profile.matrix.travelCostRounding);
			D[j][i] = D[i][j];
				T[i][j] = applyRounding(
					distance,
					parsed.profile.matrix.travelTimeRounding);
			T[j][i] = T[i][j];
		}
	}
	for (int i=numV; i<numC; i++)
		for (int j=numV; j<numC; j++) {
			D[i][j] = 0;
			T[i][j] = 0;
		}
	// the most key element to decide promising depot configurations.
	/****************************************************************************/

	/****************************************************************************/
	/* construct sparse graph****************/
	std::vector < std::set < int > > setCorrelatedVertices = std::vector < std::set <int> >(numC);
	std::vector < std::pair <double, int> > orderProximity;
	for (int i = 0; i <numV; i++){
		orderProximity.clear();
		for (int j = 0; j <numC; j++)
			if (i != j) orderProximity.push_back(std::pair <double, int>(D[i][j], j));

		sort(orderProximity.begin(), orderProximity.end());
		for (int j = 0; j < std::min<int>(alpha, numV - 2);  j++){
			setCorrelatedVertices[i].insert(orderProximity[j].second);
			setCorrelatedVertices[orderProximity[j].second].insert(i);
		}
		for (int j=0;j < (int)orderProximity.size();j++)
			nearCity[i].push_back(orderProximity[j].second);
	}
	// Filling the vector of correlated vertices
	for (int i = 0; i <numV; i++)
		for (int x : setCorrelatedVertices[i])
			edgeNode[i].push_back(x);
	// to control the number of locations closed to customers.
	for (int i = 0; i< numV; i++){
		for (int j=0; j<(int)nearCity[i].size(); j++){
			if (nearCity[i][j] >= numV){
				customerToLoc[i].push_back(nearCity[i][j]);
				if ((int)customerToLoc[i].size() > numL/2.)
					break;
			}
		}
	}
	if (problemProfile().customerDepotCandidates ==
		hgasp::CustomerDepotCandidatePolicy::
			CapacityCompatibleAllDepots) {
		// The legacy nearest-depot truncation is a search acceleration, not a
		// valid CLRP domain reduction. Keep every depot whose capacity can hold
		// the customer. Do not use singleton temporal feasibility here: per-arc
		// rounding or a non-metric input can make an indirect visit feasible even
		// when the direct singleton route is not.
		for (int customer = 0; customer < numV; ++customer) {
			customerToLoc[customer].clear();
			for (int depot = 0; depot < numL; ++depot) {
				const int depotNode = numV + depot;
				if (demCus[customer] <= invLoc[depotNode]) {
					customerToLoc[customer].push_back(depotNode);
				}
			}
			if (customerToLoc[customer].empty()) {
				throw std::runtime_error(
						"CLRP instance is infeasible: customer " +
					std::to_string(originalCustomerIds[customer]) +
					" cannot be served from any candidate depot");
			}
		}
	}
	//
	std::vector < std::set < int > > setCorrelatedLocation = std::vector < std::set <int> >(numL);
	for (int i=0; i < numL; i++){
		orderProximity.clear();
		for (int j = 0; j <numV; j++)
			orderProximity.push_back(std::pair <double, int>(D[j][i+numV], j));
		sort(orderProximity.begin(), orderProximity.end());

		for (int j = 0; j < std::min<int>(alpha, numV - 2);  j++)
				setCorrelatedLocation[i].insert(orderProximity[j].second);

		for (int j=0;j < (int)orderProximity.size();j++)
			nearCityLoca[i].push_back(orderProximity[j].second);
	}
	for (int i = 0; i <numL; i++)
		for (int x : setCorrelatedLocation[i])
			edgeLocVec[i].push_back(x);
	/****************************************************///

	// Shared search components need one complete depot ranking regardless of
	// whether restrictive static depot configurations are enabled. This support
	// does not remove any depot or depot combination from the feasible domain.
	initializeDepotSearchSupport();

	// Static depot-configuration preprocessing is a search restriction based on
	// spatial and capacity proximity, not a valid domain reduction. V1.1 uses
	// free discovery for every constraint profile and lets the common objective
	// and optional configuration islands allocate depot combinations.
	if (problemProfile().depotConfigurations ==
		hgasp::DepotConfigurationPolicy::FreeDiscovery) {
		no_capacity_location = true;
		numLocConfiguration = 0;
		LocConfigure.clear();
		compactLocConfigure.clear();
	} else {
		decideUsingDepotConfigurations();
	}
	// Legacy static-configuration construction reads locRatio in depot-index
	// order. Sort only after that policy-specific work has completed; free
	// discovery reaches the same ranked representation without changing the
	// preserved static branch's inputs.
	std::sort(locRatio.begin(), locRatio.end());
	finalizeProblemDataForSearch();

}
void SolverContext::decideUsingDepotConfigurations(){
	if (usingDepotConfiguration == 0)//not using
		analyzeLocation();
	else if (usingDepotConfiguration == 1){// only using
		std::set<int> array;
		LocConfigure.clear();
		for (int i=0;i<(int)depotCon.size();i++){
			array.clear();
			for (int j=0;j<(int)depotCon[i].size();j++)
				array.emplace (depotCon[i][j]);
			LocConfigure.push_back({0,array});
		}

	}
	else if (usingDepotConfiguration == 2){// mixed used
		analyzeLocation();
		std::set<long>binary;
		long var;
		int size;
		for (int i=0;i<(int)LocConfigure.size();i++){
			auto it = LocConfigure[i].second.begin();
			var = 0;
			while(it != LocConfigure[i].second.end()){
				var += std::pow(2,*it-numV);
				it++;
			}
			binary.emplace(var);
		}
		//
		std::set<int> array;
		for (int i=0;i<(int)depotCon.size();i++){
			array.clear();
			var = 0;
			for (int j=0;j<(int)depotCon[i].size();j++){
				array.emplace (depotCon[i][j]);
				var += std::pow(2, depotCon[i][j] - numV);
			}
			size = binary.size();
			binary.insert(var);
			if (size == (int)binary.size())continue;
			else LocConfigure.insert(LocConfigure.begin(),{0,array});
		}
	}
}

std::vector<std::string>SolverContext:: splitwithstr(std::string &str,char ch){
	  std::string tmpstr = str+ch;
	  std::vector<std::string> res;
	  if (str.size()<=0)
		return res;
	  size_t pos = tmpstr.find(ch);
	  while (pos!=std::string::npos){
			std::string child_str = tmpstr.substr(0, pos);
			if (child_str.size()>0)
			  res.push_back(child_str);
			tmpstr = tmpstr.substr(pos + 1);
			pos = tmpstr.find(ch);
	  }
	  if(res.empty()){
		  res.push_back(str);
		  return res;
	  }
	  return res;
}
//
void SolverContext::resetSearchCpuClock() noexcept {
	startTime = clock();
	meetBestTime = 0.0;
}

double SolverContext::elapsedCpuSeconds() const noexcept {
	return static_cast<double>(clock() - startTime) /
		static_cast<double>(CLOCKS_PER_SEC);
}

void SolverContext::recordGlobalIncumbent() noexcept {
	meetBestTime = elapsedCpuSeconds();
}

void SolverContext::recordSearchTermination(
		hgasp::SearchTerminationReason reason,
		hgasp::EarlyStopEvidenceClass evidenceClass,
		const bool lpRigorousNoImprovement,
		const bool mipAttempted,
		const bool mipImproved) {
	searchTermination = hgasp::makeSearchTerminationRecord(
		std::max(0, iterLimit),
		std::max(0, iteration),
		reason,
		evidenceClass,
		lpRigorousNoImprovement,
		mipAttempted,
		mipImproved);
}

bool SolverContext::stopCondition(double dis){
	const double elapsedTime = elapsedCpuSeconds();
	if (timeLimit > 0 && elapsedTime >= timeLimit) {
		recordSearchTermination(
			hgasp::SearchTerminationReason::ProcessCpuLimit);
		return true;
	}
	if (iteration >= iterLimit) {
		recordSearchTermination(
			hgasp::SearchTerminationReason::IterationLimit);
		return true;
	}
	const double reportedDis =
		hgasp::roundObjectiveForReporting(
			dis, problemProfile().reporting);
	if (meetOptimal && reportedDis <= bestDis + Min){
		timeToTarget = elapsedCpuSeconds();
		meetOptimal = false;
	}
	if (std::abs(bestDis - reportedDis) < Min){
		if (isOptimal) {
			recordSearchTermination(
				hgasp::SearchTerminationReason::ReferenceTarget);
			return true;
		}
	}
	return false;
}
bool SolverContext::equalTwoVar(double A, double B){
	if (abs(A-B) < Min)return true;
	else return false;
}
//////////////////////////////////////////////////to configure the promising locations.
void SolverContext::analyzeLocation(){
//	for (int i=numV; i<numC;i++)
//		locRatio.push_back({- double(invLoc[i]) / double(openCostLoc[i]), i});

	if (numL>=10)analyzeConflict();
	//
//	solutionTabuSearchFindCompactConfiguration();
//	if (locCostSignificant == 1 || locCostSignificant == 0)
//		tabuSearchFindCompactConfiguration();
//	else
	int min_depot = Max;
	int max_depot = 0;
	for (int i=numV;i<numC;i++){
		if (min_depot > invLoc[i])
			min_depot = invLoc[i];
		if (max_depot < invLoc[i])
			max_depot = invLoc[i];
	}
	if (min_depot > totalDemand){
		std::vector<int> array(numL);
		for (int i=0;i<numL;i++){
			array[i] = invLoc[i+numV];
			invLoc [i+numV] = (double)totalDemand / ((double)numL / 2.);
		//	invLoc[i+numV] = 0;
		}
	//	for (int i=0;i<numV;i++){
	//		invLoc[customerToLoc[i][0]] += demCus[i] * 2;
	//	}


		constructedDepotConfigurations();
		std::cout<<"The number of Depot Configurations is: "<<(int)LocConfigure.size()<<std::endl;
	//	if ((int)LocConfigure.size() < 500){
			std::cout<<"triggering the tabu search to find more depot configurations"<<std::endl;
		//	tabuSearchFindCompactConfiguration();

		for (int i=0;i<numL;i++){
			invLoc[i+numV] = array[i];
		}
//		tabuSearchFindCompactConfiguration();
//		no_capacity_location = true;
		std::sort(locRatio.begin(),locRatio.end());
		return;
	}

	if (numL < 10)
		tabuSearchFindCompactConfiguration();
	else{
		constructedDepotConfigurations();
		std::cout<<"The number of Depot Configurations is: "<<(int)LocConfigure.size()<<std::endl;
	//	if ((int)LocConfigure.size() < 500){
			std::cout<<"triggering the tabu search to find more depot configurations"<<std::endl;
			tabuSearchFindCompactConfiguration();
	//	}
	}

	std::sort(LocConfigure.begin(), LocConfigure.end());
	std::cout<<"finishing the depot configurations"<<std::endl;
	std::sort(locRatio.begin(),locRatio.end());
}
void SolverContext::initializeDepotSearchSupport(){
	locRatio.clear();
	if (locIncCus.size() != static_cast<std::size_t>(numL)) {
		locIncCus.assign(static_cast<std::size_t>(numL), {});
	} else {
		for (std::set<int>& customers : locIncCus) {
			customers.clear();
		}
	}
	spanningTreeForLoc();
	validateDepotSearchSupport();
}

void SolverContext::validateDepotSearchSupport() const {
	if (locIncCus.size() != static_cast<std::size_t>(numL)) {
		throw std::logic_error(
			"depot-search support has the wrong number of customer-support "
			"sets");
	}
	if (locRatio.size() != static_cast<std::size_t>(numL)) {
		throw std::logic_error(
			"depot-search support is incomplete: expected " +
			std::to_string(numL) + " ranked depots, found " +
			std::to_string(locRatio.size()));
	}
	std::vector<bool> seen(static_cast<std::size_t>(numL), false);
	for (const std::pair<double, int>& rankedDepot : locRatio) {
		if (!std::isfinite(rankedDepot.first)) {
			throw std::logic_error(
				"depot-search support contains a non-finite ranking score");
		}
		const int depotIndex = rankedDepot.second - numV;
		if (depotIndex < 0 || depotIndex >= numL) {
			throw std::logic_error(
				"depot-search support contains an out-of-range depot node");
		}
		if (seen[static_cast<std::size_t>(depotIndex)]) {
			throw std::logic_error(
				"depot-search support contains a duplicate depot node");
		}
		seen[static_cast<std::size_t>(depotIndex)] = true;
	}
}

void SolverContext::analyzeConflict(){
	double min_x = Max, min_y = Max, max_x = 0,max_y = 0;
	double range_x, range_y;
	for (int i=0;i<numC;i++){
		if (x[i] < min_x) min_x = x[i];
		if (y[i] < min_y) min_y = y[i];
		if (x[i] > max_x) max_x = x[i];
		if (y[i] > max_y) max_y = y[i];
	}
	double range_loc, min_loc = Max, max_loc = 0;
	for (int i=numV;i<numC;i++){
		if (invLoc[i] < min_loc)min_loc = invLoc[i];
		if (invLoc[i] > max_loc)max_loc = invLoc[i];
	}
	range_loc = max_loc - min_loc;
	range_x = max_x - min_x;
	range_y = max_y - min_y;
	//decide the instance whether fixed location costs significant.
	double avgCost = 0;
	int count = 0;
	for (int i=0;i<numV;i++){
		for (int j=0;j<(int)edgeNode[i].size();j++){
			avgCost += edgeNode[i][j];
			count ++;
		}
	}
	avgCost = avgCost / count;
	//double travelCost = avgCost * numV;
	double locCost = 0;
	for (int i=0;i<numL;i++)
		locCost += openCostLoc[i + numV];
	// there are two cases. the first case is type C and the other cases are all other instances
//	if (travelCost > locCost * 10)// this type is not complete since types a b d are not
//		locCostSignificant = 0;// type c
//	else if (travelCost * 100 < locCost)
//		locCostSignificant = 1;// type e
//	else locCostSignificant = 2;
	//the following is to decide some locations conflict
	double dis_measure = (range_x + range_y) / 2.;
	double ratio = range_loc / min_loc;// if the value is greater than 1, it is type e, otherwise it is other type.
	double xx,yy,xy;
	for (int i=numV;i<numC;i++){
		if (ratio > 1 && ((double)invLoc[i] / (double)totalDemand > 0.2)){// the type e
			if (max_loc / invLoc[i] > 1.8)continue;
			for (int j = i; j < numC; j++){
				if (i == j || ((double)invLoc[i] / (double)invLoc[j] > 1.8))continue;
				xx = std::abs(x[i] - x[j]);
				yy = std::abs(y[i] - y[j]);
				xy = (xx + yy) / 2.;
				if (xy / dis_measure < (double)invLoc[i] / (double)totalDemand){
					conflictLoc[i - numV][j - numV] = 1;
					conflictLoc[j - numV][i - numV] = 1;
				}
			}
		}
		if (ratio < 1 && (double)invLoc[i] / (double)totalDemand > 0.1){// the type a b d
			for (int j = i; j < numC; j++){
				if (i == j)continue;
				xx = std::abs(x[i] - x[j]);
				yy = std::abs(y[i] - y[j]);
				xy = (xx + yy) / 2.;
				if (2.0*xy / dis_measure < (double)invLoc[i] / (double)totalDemand){// the coefficient 2 is used to not limit too much
					conflictLoc[i - numV][j - numV] = 1;
					conflictLoc[j - numV][i - numV] = 1;
				}
			}
		}
	}
/*
	for (int i=0;i<numL;i++){
		for (int j=0;j<numL;j++)
			std::cout<<conflictLoc[i][j]<<" ";
		std::cout<<std::endl;
	}
*/
}
void SolverContext::spanningTreeForLoc(){
	int * check_cus;
	check_cus = new int [numV];

	double * low_cost;
	low_cost = new double [numV];

	//////////////the following is the spanning tree method
	double min_dis;
	double total_dis_loc;
	int total_dem;
	int v;
	for (int l=0;l<numL;l++){
		for (int i=0;i<numV;i++)check_cus[i] = 0;
		total_dis_loc = 0;
		total_dem = 0;
		for (int j=0;j<numV;j++)
			low_cost[j]=D[l+numV][j];
		int numVehicle = 0;
		int selectedCustomers = 0;
		while(selectedCustomers < numV){
			int totalVeh = 0;
			bool madeProgress = false;
			for (int i=0;i< numV;i++){
				min_dis=Max;
				v=-1;
				for (int j=0;j<numV;j++){
					if (check_cus[j]==0 && min_dis>low_cost[j]){
						v=j;
						min_dis=low_cost[j];
					}
				}
				if (v!=-1){
					if (total_dem + demCus[v] >= invLoc[l+numV] || totalVeh >= vehCap)break;
					check_cus[v] = 1;
					selectedCustomers++;
					madeProgress = true;
					locIncCus[l].insert(v);
					total_dis_loc += min_dis;
					total_dem += demCus[v];
					totalVeh += demCus[v];
					for (int k=0;k<numV;k++){
						if (check_cus[k]==0 && D[k][v]<low_cost[k])
							low_cost[k]=D[k][v];
					}
				}
			}
			numVehicle++;
			// A depot with capacity above the total demand can exhaust every
			// unchecked customer.  The legacy code then dereferenced demCus[-1]
			// and could spin forever.  No progress means the finite Prim-style
			// construction is complete (or the next customer cannot fit).
			if (!madeProgress || v < 0 || selectedCustomers >= numV)
				break;
			if (total_dem + demCus[v] >= invLoc[l+numV])
				break;
			for (int j=0;j<numV;j++)
				if (check_cus[j] == 1)
					low_cost[j]=Max;

		}
		double cost_loc = openCostLoc[l + numV] + total_dis_loc + numVehicle * usingCostVeh;
	//	locRatio.push_back({ cost_loc / double(total_dem), l+numV});
		locRatio.push_back({(double)cost_loc, l+numV});
	}
	delete [] check_cus;
	delete [] low_cost;
	//
/*	double total_dis_loc;
	int total_dem;
	for (int l=0;l<numL;l++){
		for (int i=0;i<numV;i++)check_cus[i] = 0;
		total_dis_loc = 0;
		total_dem = 0;
		for (int i=0;i<numV;i++){
			if (total_dem + demCus[nearCityLoca[l][i]] <= invLoc[l+numV]){
				total_dem += demCus[nearCityLoca[l][i]];
				total_dis_loc += D[l+numV][i];
				locIncCus[l].insert(nearCityLoca[l][i]);
			}
			else break;
		}
		double cost_loc = openCostLoc[l + numV] + total_dis_loc;
	//	locRatio.push_back({ cost_loc / double(total_dem), l+numV});
		locRatio.push_back({(double)cost_loc, l+numV});
	}
*/
}

void SolverContext::tabuSearchFindCompactConfiguration(){
	std::vector<std::set<int>> listLocConfigure;
	// using tabu search to determine that does this instance has perfect locations configurations
	int *sol,*best_sol, *tabu_table, *dem;
	sol = new int [numL];
	best_sol = new int [numL];
	tabu_table = new int [numL];
	for (int i=0;i<numL;i++)tabu_table[i] = 0;
	dem = new int [numL];
	int cusDem = 0;
	int obj=0,best_obj;
	// construct a initial solution
	int count;
	for (int i=0;i<numL;i++){
		dem[i] = locRatio[i].first;
	}
	const int maximumConstructionAttempts =
		std::max(100, numL * 100);
	int constructionAttempt = 0;
	while(constructionAttempt++ < maximumConstructionAttempts){
		for (int i=0;i<numL;i++)sol[i] = 0;
		cusDem = 0;
		obj = 0;
		int seed = RandomInt(0, numL - 1);
		int cumLoc = 0;
		count = 0;
		while(1){
			cumLoc += invLoc[seed+numV];
			sol[seed] = 1;
			cusDem += invLoc[seed+numV];
			obj += dem[seed];
				if (cumLoc >= totalDemand)break;
			count = 0;
			while(1){
				seed = RandomInt(0, numL - 1);
				if (sol[seed] == 0){
					bool isConfilict = false;
					for (int i=0;i<numL;i++){
						if (sol[i] == 1){
							if(conflictLoc[i][seed] == 1){
								isConfilict = true;
								break;
							}
						}
					}
					if (!isConfilict)break;
				}
				count ++;
				if (count > 1000)
					break;
			}
			if (count > 1000)break;
		}
			if (count < 1000)break;
	}
	if (constructionAttempt > maximumConstructionAttempts) {
		// conflictLoc is only a preprocessing heuristic.  Falling back to all
		// depots preserves the original CLRP domain and deterministically
		// supplies a capacity-feasible starting configuration.
		cusDem = 0;
		obj = 0;
		for (int depot = 0; depot < numL; ++depot) {
			sol[depot] = 1;
			cusDem += invLoc[depot + numV];
			obj += dem[depot];
		}
		if (cusDem < totalDemand) {
			throw std::runtime_error(
				"depot-configuration preprocessing found insufficient "
				"aggregate capacity");
		}
	}
	int seed;
	best_obj = obj;
	// The tabu walk records configurations after its first move.  A
	// capacity-tight construction can already be locally immovable, so retain
	// the feasible seed before entering the neighborhood loop.
	std::set<int> initialDepots;
	for (int depot = 0; depot < numL; ++depot) {
		if (sol[depot] == 1)
			initialDepots.insert(depot + numV);
	}
	if (cusDem >= totalDemand && !initialDepots.empty()) {
		LocConfigure.push_back({static_cast<double>(obj), initialDepots});
	}
	int can_node_1[100],can_node_2[100];
	int can_tabu_node_1[100],can_tabu_node_2[100];
	//
	int delta,min_delta, min_tabu_delta;
	int num_best, num_tabu_best;
	int no_improve = 0;
	double tabuLength = 0.3;
	int length_improve = 2000;
	int tabu_length = ceil(numL * tabuLength);
	int step = 1;
	int loc1, loc2;

	while (1){
		min_delta=Max;min_tabu_delta=Max;num_best=0;num_tabu_best=0;
		for (int i=0;i<numL;i++){
			if (sol[i] == 1){
				if (cusDem - invLoc[i+numV] >= totalDemand){
					delta = -dem[i];
					if (tabu_table[i] < step){
						if (delta < min_delta){
							min_delta = delta;
							can_node_1[0] = i;
							can_node_2[0] = -1;
							num_best = 1;
						}
						else if (delta == min_delta && num_best < 100){
							can_node_1[num_best] = i;
							can_node_2[num_best] = -1;
							num_best ++;
						}
					}
					else if (tabu_table[i] > step){
						if (delta < min_tabu_delta){
							min_tabu_delta = delta;
							can_tabu_node_1[0] = i;
							can_tabu_node_2[0] = -1;
							num_tabu_best = 1;
						}
						else if (delta == min_tabu_delta && num_tabu_best < 100){
							can_tabu_node_1[num_tabu_best] = i;
							can_tabu_node_2[num_tabu_best] = -1;
							num_tabu_best ++;
						}
					}
				}
				//
				for (int j=0; j<numL;j++){
					if (i == j || sol[j] == 1)continue;
					if (cusDem - invLoc[i+numV] + invLoc[j+numV] < totalDemand)continue;
					//decide conflict
					seed = j;
					bool isConfilict = false;
					for (int ii=0;ii<numL;ii++){
						if (ii == i)continue;
						if (sol[ii] == 1){
							if(conflictLoc[ii][seed] == 1){
								isConfilict = true;
								break;
							}
						}
					}
					if (isConfilict)continue;
					//

					delta = dem[j] - dem[i];
					if (tabu_table[i] <= step && tabu_table[j] <= step){
						if (delta < min_delta){
							min_delta = delta;
							can_node_1[0] = i;
							can_node_2[0] = j;
							num_best = 1;
						}
						else if (delta == min_delta && num_best < 100){
							can_node_1[num_best] = i;
							can_node_2[num_best] = j;
							num_best ++;
						}
					}
					else if (tabu_table[i] > step || tabu_table[j]> step ){
						if (delta < min_tabu_delta){
							min_tabu_delta = delta;
							can_tabu_node_1[0] = i;
							can_tabu_node_2[0] = j;
							num_tabu_best = 1;
						}
						else if (delta == min_tabu_delta && num_tabu_best < 100){
							can_tabu_node_1[num_tabu_best] = i;
							can_tabu_node_2[num_tabu_best] = j;
							num_tabu_best ++;
						}
					}
				}
			}
			else{
				seed = i;
				bool isConfilict = false;
				for (int ii=0;ii<numL;ii++){
					if (sol[ii] == 1){
						if(conflictLoc[ii][seed] == 1){
							isConfilict = true;
							break;
						}
					}
				}
				if (isConfilict)continue;
				////////////////////////////////////////////////////
				delta = dem[i];
				if (tabu_table[i] <= step){
					if (delta < min_delta){
						min_delta = delta;
						can_node_1[0] = i;
						can_node_2[0] = -2;
						num_best = 1;
					}
					else{
						can_node_1[num_best] = i;
						can_node_2[num_best] = -2;
						num_best ++;
					}
				}
				else if (tabu_table[i] > step){
					if (delta < min_tabu_delta){
						min_tabu_delta = delta;
						can_tabu_node_1[0] = i;
						can_tabu_node_2[0] = -2;
						num_tabu_best = 1;
					}
					else if (delta == min_tabu_delta && num_tabu_best < 100){
						can_tabu_node_1[num_tabu_best] = i;
						can_tabu_node_2[num_tabu_best] = -2;
						num_tabu_best ++;
					}
				}
			}
		}
		if (min_tabu_delta == Max && min_delta == Max)break;
		int rand_order;
		if ((num_tabu_best>0 && min_tabu_delta < min_delta && (obj + min_tabu_delta< best_obj)) || num_best==0){
			obj += min_tabu_delta;
			rand_order = RandomInt(0, num_tabu_best - 1);
			loc1 =can_tabu_node_1[rand_order];
			loc2 =can_tabu_node_2[rand_order];
			tabu_table[loc1]=step+tabu_length+RandomInt(0, tabu_length - 1);
			if (loc2 >-1)
				tabu_table[loc2]=step+tabu_length+RandomInt(0, tabu_length - 1);
		}
		else{
			obj += min_delta;
			rand_order = RandomInt(0, num_best - 1);
			loc1 = can_node_1[rand_order];
			loc2 = can_node_2[rand_order];
			tabu_table[loc1]=step+tabu_length+RandomInt(0, tabu_length - 1);
			if (loc2 > -1)
				tabu_table[loc2]=step+tabu_length+RandomInt(0, tabu_length - 1);
		}
		if (loc2 > -1){
			sol[loc1] = 0;
			sol[loc2] = 1;
			cusDem += invLoc[loc2 + numV];
			cusDem = cusDem - invLoc[loc1 +numV];
		}
		else if (loc2 == -1){
			sol[loc1] = 0;
			cusDem = cusDem - invLoc[loc1 + numV];
		}
		else if (loc2 == -2){
			sol[loc1] = 1;
			cusDem += invLoc[loc1 + numV];
		}
		//
		if (best_obj > obj){
			best_obj = obj;
			for (int i=0;i<numL;i++)
				best_sol[i] = sol[i];
		}
		else	no_improve ++;
		step++;
		if (no_improve > length_improve){
	//		disturbation(sol, cusDem);
	//		no_improve = 0;

			break;
		}
		//
/*		for (int i=0;i<numL;i++)
			std::cout<<i<<" ";
		std::cout<<std::endl;
		for (int i=0;i<numL;i++)
			std::cout<<sol[i]<<" ";
		std::cout<<std::endl;
*/
		//
		std::set<int>depot;
		double cost = 0;
		for (int i=0;i<numL;i++)
			if (sol[i] == 1){
				depot.insert(i+numV);
				cost += dem[i];
			}
		////////////////////////////
		int sumDem = 0;
		for (int i=0;i<numL;i++){
			if (sol[i] == 1)
				sumDem += invLoc[i+numV];
		}
		bool isContinue = false;
		for (int i=0;i<numL;i++){
			if (sol[i] == 1)
				if (sumDem - invLoc[i+numV] >= totalDemand){
					isContinue = true;
					break;
				}
		}
		if (isContinue)continue;
		/////////////////////////////


		bool same = true;
		for (int i=0;i<(int)LocConfigure.size();i++){
			if ((int)depot.size() == (int)LocConfigure[i].second.size()){
				auto it1 = depot.begin();
				auto it2 = LocConfigure[i].second.begin();
				bool theSame = true;
				while(it1 != depot.end()){
					if (*it1 != *it2){
						theSame = false;break;
					}
					it1 ++;
					it2 ++;
				}
				if (theSame)same = false;
			}
		}
		if (same){
			if ((int)LocConfigure.size() >= numLocConfiguration){
				std::sort(LocConfigure.begin(), LocConfigure.end());
				LocConfigure.pop_back();
			}
			LocConfigure.push_back({cost,depot});
		}
		int sum =0;
		for (int i=0;i<numL;i++)
			sum += sol[i];

//		if (sol[4] == 1 && sol[6] == 1 && sol[14] == 1 && sol[16] == 1)
//			std::cout<<" the best results  "<<std::endl;
	}
	delete [] sol;
	delete [] best_sol;
	delete [] tabu_table;
	delete [] dem;
	//
//	std::sort(LocConfigure.begin(), LocConfigure.end());
	//
/*	std::set<int>depot;
	depot.insert(304);
	depot.insert(306);depot.insert(314);depot.insert(316);
	LocConfigure.push_back({100,depot});
	std::sort(LocConfigure.begin(), LocConfigure.end());
*/
/*  this code repeats the evaluation logic described by Schneider et al.
	std::set<int>depot;
	depot.insert(200);
	depot.insert(203);depot.insert(204);depot.insert(205);depot.insert(206);depot.insert(210);
	LocConfigure.push_back({100,depot});
	std::sort(LocConfigure.begin(), LocConfigure.end());
	return;

	LocConfigure.push_back({100,depot});
	std::vector<int>loc;
	for (int i=0;i<(int)LocConfigure.size();i++){
		loc.clear();
		auto it = LocConfigure[i].second.begin();
		for (;it != LocConfigure[i].second.end();it++)
			loc.push_back(*it);
		//computing the rating function
		double costLoc = 0;
		double part1 = 0,part2 = 0;
		for (int j=0;j<(int)loc.size();j++){
			for (int k=0;k<numV;k++)
				part1 += demCus[k] * pow(0.95, (D[loc[j]][k]* 2)/100);

			for (int k=0;k<(int)loc.size();k++)
				part2 += invLoc[loc[k]] * pow(0.95, (D[loc[j]][loc[k]] * 2)/100);
			costLoc += std::abs(part1 - part2) + openCostLoc[loc[j]];
		}
		LocConfigure[i].first = costLoc;
	}
	std::sort(LocConfigure.begin(), LocConfigure.end());
*/



/*	for (int i=0;i < (int)LocConfigure.size(); i++){
		auto it = LocConfigure[i].second.begin();
		while(it != LocConfigure[i].second.end()){
			std::cout<<*it<<" ";
			it ++;
		}
		std::cout<<std::endl;
	}
	*/
}
void SolverContext::constructedDepotConfigurations(){
	//the line function to estimate the coverRatio
	double min_coverRatio = 0.1;
	double max_coverRatio = 0.6;
	double kParameter = max_coverRatio - min_coverRatio;
	//
	// This legacy preprocessing branch only needs a membership bit. Keeping it
	// local makes repeated instance initialization exception-safe and avoids a
	// 40 MB integer allocation for a boolean table.
	std::vector<unsigned char> solutionTabu(
		static_cast<std::size_t>(dimension), 0U);
	int seed;
	double coverRatio = 0.2;// assuming the covering ratio is 50%
	double costDepotConf;
	int countOver;
	int iter;
	int totalDem;
	std::set<int>coverVertex;
	std::vector<std::pair<double, int>>priorityOrder;// the first is the ratio of overlaping vertices and the second is the order of depot
	std::set<int>sol;
	std::vector<int>depot;
	std::vector<std::pair<double, int>>orderDepots;
	std::set<int>array;
	std::set<int>array2;
	int iteration = 0;
	int no_get = 0;
	bool isUsed;
	isUsed = true;
	while(1){
		sol.clear();
		depot.clear();
		coverVertex.clear();
		orderDepots.clear();
		for (int i=0;i<numL;i++)depot.push_back(i);
		seed = RandomInt(0, numL - 1);
		costDepotConf = locRatio[seed].first;
		sol.insert(seed);
		totalDem = invLoc[seed+numV];
		depot.erase(depot.begin()+seed);//
		//
		auto it = locIncCus[seed].begin();
		for (;it != locIncCus[seed].end(); it++) coverVertex.insert(*it);
		while(totalDem < totalDemand){// select other depots
			priorityOrder.clear();
			for (int i=0;i<(int)depot.size();i++){//deicde the overlapping vertices
				countOver = 0;iter = 0;
				auto it1 = locIncCus[depot[i]].begin();
				for (;it1 != locIncCus[depot[i]].end(); it1++){
					array2.insert(*it1);

					auto it2 = coverVertex.find(*it1);
					if (it2 != coverVertex.end()){
						countOver ++;
					}
					iter ++;
				}

				double var = (double)countOver / (double)iter;
				priorityOrder.push_back({var, depot[i]});
			}
			sort(priorityOrder.begin(), priorityOrder.end());
			orderDepots.clear();
			for (int i=0;i<(int)priorityOrder.size();i++){
				int cap = invLoc[priorityOrder[i].second + numV] + totalDem;
				coverRatio = kParameter * ((double)cap / (double)totalDemand) + min_coverRatio;
				if (priorityOrder[i].first > coverRatio)break;// discard these non-promising depots
				else orderDepots.push_back({locRatio[priorityOrder[i].second].first,priorityOrder[i].second});
			}

			if ((int)orderDepots.size() == 0){
				if (isUsed){
					if (iteration > 100 && (int)LocConfigure.size() == 0)
						isUsed = false;
					break;
				}
					else{
						int relaxationSteps = 0;
						while ((int)orderDepots.size() == 0 &&
							   !priorityOrder.empty() &&
							   relaxationSteps++ < 20){
							coverRatio += 0.1;
						for (int i=0;i<(int)priorityOrder.size();i++){
						//	int cap = invLoc[priorityOrder[i].second + numV] + totalDem;
						//	coverRatio = kParameter * ((double)cap / (double)totalDemand) + min_coverRatio;
							if (priorityOrder[i].first > coverRatio)break;// discard these non-promising depots
							else orderDepots.push_back({locRatio[priorityOrder[i].second].first,priorityOrder[i].second});
							}
						}
						if (orderDepots.empty())
							break;
					}
				}

			sort(orderDepots.begin(),orderDepots.end());

			double y = RandomDouble(0.0, 1.0);
			int rank = int(pow(
				y,
				parameters.configurationPortfolio
					.depotSelectionRankExponent) *
				(int)orderDepots.size());

			seed = orderDepots[rank].second;

			auto it1 = locIncCus[seed].begin();
			for (;it1 != locIncCus[seed].end(); it1++)
				coverVertex.insert(*it1);
			int index = -1;
			for (int i=0;i<(int)depot.size();i++)
				if (depot[i] == seed){
					index = i;
					break;
				}
			depot.erase(depot.begin() + index);
			sol.insert(seed);
			costDepotConf += locRatio[seed].first;
			totalDem += invLoc[seed+numV];
		}
		if (totalDem < totalDemand){
			no_get ++;
			iteration ++;
			if (no_get > 1000) {
				std::sort(LocConfigure.begin(), LocConfigure.end());
				std::cout
					<<"finishing construction depots configurations"
					<<std::endl;
				break;
			}
			continue;
		}


		it = sol.begin();
		array.clear();
		long long int binaryVal = 0;
		for (;it != sol.end();it++){
			array.insert(*it + numV);
			binaryVal += pow(2,*it);
		}
		// solution based
		if(solutionTabu[static_cast<std::size_t>(
				binaryVal % dimension)] == 0U){
			solutionTabu[static_cast<std::size_t>(
				binaryVal % dimension)] = 1U;
			//
			LocConfigure.push_back({costDepotConf,array});
			if ((int)LocConfigure.size() >= numLocConfiguration){
				std::sort(LocConfigure.begin(), LocConfigure.end());
				std::cout<<"finishing construction depots configurations"<<std::endl;
				break;
			}
			no_get = 0;
		}
		else no_get ++;
		if (no_get > 1000){
			std::sort(LocConfigure.begin(), LocConfigure.end());
			std::cout<<"finishing construction depots configurations"<<std::endl;
			std::cout
				<< "instance preprocessing process CPU time  "
				<< elapsedCpuSeconds() << std::endl;
			break;
		}
		iteration++;
	//	std::cout<<iteration<<"   "<<(int)LocConfigure.size()<<std::endl;
	}
}

void SolverContext::readDepotConfigurations(){
	std::vector<int>array;
	std::string var;
	if (pathDepotCon != "NULL"){
		std::ifstream inputFile(pathDepotCon);
			if (inputFile.is_open()){
				int numDepotCon;
				if (!(inputFile >> numDepotCon) || numDepotCon < 0) {
					throw std::runtime_error(
						"invalid depot configuration header: " +
						pathDepotCon);
				}
				for (int i=0;i<numDepotCon;i++){
					array.clear();
					if (!(inputFile >> var)) {
						throw std::runtime_error(
							"unexpected end of depot configuration file: " +
							pathDepotCon);
					}
					while(var != "e"){
						array.push_back(std::stoi(var) + numV);
						if (!(inputFile >> var)) {
							throw std::runtime_error(
								"missing 'e' terminator in depot "
								"configuration file: " + pathDepotCon);
						}
					}
				depotCon.push_back(array);
			}
		}
        else {
            throw std::runtime_error(
                "failed to open depot configuration file: " +
                pathDepotCon);
        }
    }
    else {
        return;
    }
}
