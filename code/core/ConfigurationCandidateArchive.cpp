#include "ConfigurationCandidateArchive.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace hgasp {
namespace {

bool containsConfiguration(
        const std::vector<DepotConfigurationKey>& configurations,
        const DepotConfigurationKey& wanted) {
    return std::find(
        configurations.begin(), configurations.end(), wanted) !=
        configurations.end();
}

bool sameSeedStructure(
        const ScreenedConfigurationSeed& left,
        const ScreenedConfigurationSeed& right) {
    if (left.routes.size() != right.routes.size()) {
        return false;
    }
    std::vector<RouteKey> leftRoutes;
    std::vector<RouteKey> rightRoutes;
    leftRoutes.reserve(left.routes.size());
    rightRoutes.reserve(right.routes.size());
    for (const StandaloneRoute& route : left.routes) {
        leftRoutes.push_back(makeRouteKey(route));
    }
    for (const StandaloneRoute& route : right.routes) {
        rightRoutes.push_back(makeRouteKey(route));
    }
    std::sort(leftRoutes.begin(), leftRoutes.end());
    std::sort(rightRoutes.begin(), rightRoutes.end());
    return leftRoutes == rightRoutes;
}

}  // namespace

bool screenedConfigurationSeedHasHigherPriority(
        const ScreenedConfigurationSeed& left,
        const ScreenedConfigurationSeed& right) {
    if (left.objective != right.objective) {
        return left.objective < right.objective;
    }
    if (left.diversity != right.diversity) {
        return left.diversity > right.diversity;
    }
    const DepotConfigurationKey leftConfiguration =
        makeDepotConfigurationKey(left.configuration.openDepots);
    const DepotConfigurationKey rightConfiguration =
        makeDepotConfigurationKey(right.configuration.openDepots);
    if (leftConfiguration != rightConfiguration) {
        return leftConfiguration < rightConfiguration;
    }
    if (left.sourceIndex != right.sourceIndex) {
        return left.sourceIndex < right.sourceIndex;
    }
    return false;
}

const char* toString(
        const ConfigurationDiscoverySource source) noexcept {
    switch (source) {
    case ConfigurationDiscoverySource::CrhInitialization:
        return "crh_initialization";
    case ConfigurationDiscoverySource::StCrhInitialization:
        return "st_crh_initialization";
    case ConfigurationDiscoverySource::FreeInitialization:
        return "free_initialization";
    case ConfigurationDiscoverySource::MdEaxRecombination:
        return "mdeax_recombination";
    case ConfigurationDiscoverySource::ConstraintFeasibleReconstruction:
        return "constraint_feasible_reconstruction";
    case ConfigurationDiscoverySource::CapacityRepair:
        return "capacity_repair";
    case ConfigurationDiscoverySource::Mutation:
        return "mutation";
    case ConfigurationDiscoverySource::LocalSearch:
        return "local_search";
    case ConfigurationDiscoverySource::GlobalSp:
        return "global_sp";
    case ConfigurationDiscoverySource::Count:
        return "none";
    }
    return "unknown";
}

bool isGlobalSpConfigurationDiscoveryCall(
        const std::string_view spCallId) noexcept {
    return spCallId.rfind("local_", 0U) != 0U;
}

const char* toString(const ConfigurationChangeKind kind) noexcept {
    switch (kind) {
    case ConfigurationChangeKind::Unchanged:
        return "unchanged";
    case ConfigurationChangeKind::Open:
        return "open";
    case ConfigurationChangeKind::Close:
        return "close";
    case ConfigurationChangeKind::Swap:
        return "swap";
    case ConfigurationChangeKind::Mixed:
        return "mixed";
    }
    return "unknown";
}

ConfigurationChangeKind classifyConfigurationChange(
        const DepotConfigurationKey& before,
        const DepotConfigurationKey& after) noexcept {
    if (before == after) {
        return ConfigurationChangeKind::Unchanged;
    }
    const bool opened = std::any_of(
        after.openDepots.begin(), after.openDepots.end(),
        [&before](const int depot) {
            return !std::binary_search(
                before.openDepots.begin(), before.openDepots.end(), depot);
        });
    const bool closed = std::any_of(
        before.openDepots.begin(), before.openDepots.end(),
        [&after](const int depot) {
            return !std::binary_search(
                after.openDepots.begin(), after.openDepots.end(), depot);
        });
    if (opened && !closed) {
        return ConfigurationChangeKind::Open;
    }
    if (closed && !opened) {
        return ConfigurationChangeKind::Close;
    }
    if (opened && closed &&
        before.openDepots.size() == after.openDepots.size()) {
        return ConfigurationChangeKind::Swap;
    }
    return ConfigurationChangeKind::Mixed;
}

const char* toString(
        const ConfigurationCandidateEligibilityReason reason) noexcept {
    switch (reason) {
    case ConfigurationCandidateEligibilityReason::ActiveResident:
        return "active_resident";
    case ConfigurationCandidateEligibilityReason::CurrentChallenger:
        return "current_challenger";
    case ConfigurationCandidateEligibilityReason::MissingSeed:
        return "missing_seed";
    case ConfigurationCandidateEligibilityReason::NoReplaceableResident:
        return "no_replaceable_resident";
    case ConfigurationCandidateEligibilityReason::Cooldown:
        return "cooldown";
    case ConfigurationCandidateEligibilityReason::
            NoNewEvidenceAfterRejection:
        return "no_new_evidence_after_rejection";
    case ConfigurationCandidateEligibilityReason::
            InsufficientObservations:
        return "insufficient_observations";
    case ConfigurationCandidateEligibilityReason::QualityFloor:
        return "quality_floor";
    case ConfigurationCandidateEligibilityReason::ExceptionalBest:
        return "exceptional_best";
    case ConfigurationCandidateEligibilityReason::
            RepeatedWithQualityFloor:
        return "repeated_with_quality_floor";
    }
    return "unknown";
}

double ConfigurationCandidateRecord::top3Mean() const noexcept {
    if (topObjectives.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    return std::accumulate(
               topObjectives.begin(), topObjectives.end(), 0.0) /
        static_cast<double>(topObjectives.size());
}

ConfigurationCandidateArchive::ConfigurationCandidateArchive(
        ConfigurationCandidateArchivePolicy policy)
    : policy_(std::move(policy)) {
    if (policy_.maximumRecords == 0U ||
        policy_.topObjectiveLimit == 0U ||
        policy_.seedLimit == 0U ||
        policy_.repeatedDiscoveryThreshold == 0U ||
        policy_.newObservationsAfterRejection == 0U ||
        !std::isfinite(policy_.objectiveTolerance) ||
        policy_.objectiveTolerance < 0.0) {
        throw std::invalid_argument(
            "configuration candidate archive policy is invalid");
    }
}

DepotConfigurationKey ConfigurationCandidateArchive::canonicalize(
        const DepotConfigurationKey& configuration) const {
    DepotConfigurationKey canonical =
        makeDepotConfigurationKey(configuration.openDepots);
    if (canonical.openDepots.empty()) {
        throw std::invalid_argument(
            "candidate configuration opens no depot");
    }
    return canonical;
}

const ConfigurationCandidateRecord*
ConfigurationCandidateArchive::find(
        const DepotConfigurationKey& configuration) const {
    const DepotConfigurationKey canonical = canonicalize(configuration);
    const auto found = records_.find(canonical);
    return found == records_.end() ? nullptr : &found->second;
}

const ConfigurationObservationHistory*
ConfigurationCandidateArchive::findObservationHistory(
        const DepotConfigurationKey& configuration) const {
    const DepotConfigurationKey canonical = canonicalize(configuration);
    const auto found = observationHistory_.find(canonical);
    return found == observationHistory_.end() ? nullptr : &found->second;
}

const ConfigurationTrialHistory*
ConfigurationCandidateArchive::findTrialHistory(
        const DepotConfigurationKey& configuration) const {
    const DepotConfigurationKey canonical = canonicalize(configuration);
    const auto found = trialHistory_.find(canonical);
    return found == trialHistory_.end() ? nullptr : &found->second;
}

ConfigurationCandidateEligibilityDecision
ConfigurationCandidateArchive::evaluateEligibility(
        const ConfigurationCandidateRecord& record,
        const ConfigurationCandidateEligibilityContext& context) const {
    if (containsConfiguration(
            context.activeResidents, record.configuration)) {
        return {false,
            ConfigurationCandidateEligibilityReason::ActiveResident};
    }
    if (context.currentChallenger.has_value() &&
        *context.currentChallenger == record.configuration) {
        return {false,
            ConfigurationCandidateEligibilityReason::CurrentChallenger};
    }
    if (record.diverseSeeds.empty()) {
        return {false,
            ConfigurationCandidateEligibilityReason::MissingSeed};
    }
    const auto& qualityReference =
        context.qualityReferenceResident.has_value()
            ? context.qualityReferenceResident
            : context.weakestReplaceableResident;
    if (!qualityReference.has_value()) {
        return {false,
            ConfigurationCandidateEligibilityReason::
                NoReplaceableResident};
    }

    const ConfigurationObservationHistory* observation =
        findObservationHistory(record.configuration);
    if (observation == nullptr) {
        throw std::logic_error(
            "working candidate has no persistent observation history");
    }
    const ConfigurationTrialHistory* trial =
        findTrialHistory(record.configuration);
    double qualifyingBestObjective = observation->bestObjective;
    if (trial != nullptr) {
        const bool hasEvidenceBoundary =
            trial->totalRejections > 0U ||
            trial->totalResidentRetirements > 0U;
        if (hasEvidenceBoundary) {
            if (context.attempt <
                trial->latestEvidenceCooldownUntilAttempt) {
                return {false,
                    ConfigurationCandidateEligibilityReason::Cooldown};
            }
            const std::uint64_t newObservations =
                observation->totalObservations >=
                    trial->totalObservationsAtLatestEvidenceBoundary
                    ? observation->totalObservations -
                        trial->totalObservationsAtLatestEvidenceBoundary
                    : 0U;
            qualifyingBestObjective =
                trial->bestObjectiveAfterLatestEvidenceBoundary;
            const bool strictNewBest =
                qualifyingBestObjective + policy_.objectiveTolerance <
                    trial->bestObjectiveAtLatestEvidenceBoundary;
            if (newObservations <
                    policy_.newObservationsAfterRejection &&
                !strictNewBest) {
                return {false,
                    ConfigurationCandidateEligibilityReason::
                        NoNewEvidenceAfterRejection};
            }
        }
    }

    const ResidentQualitySnapshot& resident = *qualityReference;
    const bool exceptional =
        std::isfinite(resident.bestObjective) &&
        qualifyingBestObjective + policy_.objectiveTolerance <
            resident.bestObjective;
    if (exceptional) {
        return {true,
            ConfigurationCandidateEligibilityReason::ExceptionalBest};
    }
    if (observation->totalObservations <
        policy_.repeatedDiscoveryThreshold) {
        return {false,
            ConfigurationCandidateEligibilityReason::
                InsufficientObservations};
    }
    if (!std::isfinite(resident.eliteMean10) ||
        !(qualifyingBestObjective + policy_.objectiveTolerance <
          resident.eliteMean10)) {
        return {false,
            ConfigurationCandidateEligibilityReason::QualityFloor};
    }
    return {true,
        ConfigurationCandidateEligibilityReason::
            RepeatedWithQualityFloor};
}

std::vector<const ConfigurationCandidateRecord*>
ConfigurationCandidateArchive::eligibleRecords(
        const ConfigurationCandidateEligibilityContext& context) const {
    std::vector<const ConfigurationCandidateRecord*> eligible;
    for (const auto& entry : records_) {
        if (isEligible(entry.second, context)) {
            eligible.push_back(&entry.second);
        }
    }
    return eligible;
}

bool ConfigurationCandidateArchive::makeRoomForNewRecord(
        const ConfigurationCandidateEligibilityContext& context,
        bool& evicted) {
    evicted = false;
    if (records_.size() < policy_.maximumRecords) {
        return true;
    }

    auto eviction = records_.end();
    for (auto candidate = records_.begin();
         candidate != records_.end(); ++candidate) {
        const ConfigurationCandidateRecord& record = candidate->second;
        if ((context.currentChallenger.has_value() &&
            *context.currentChallenger == record.configuration) ||
            isEligible(record, context)) {
            continue;
        }
        if (eviction == records_.end()) {
            eviction = candidate;
            continue;
        }
        const ConfigurationCandidateRecord& current = eviction->second;
        const bool worse =
            record.observationCount < current.observationCount ||
            (record.observationCount == current.observationCount &&
             (record.top3Mean() > current.top3Mean() ||
              (record.top3Mean() == current.top3Mean() &&
               (record.lastSeenAttempt < current.lastSeenAttempt ||
                (record.lastSeenAttempt == current.lastSeenAttempt &&
                 current.configuration < record.configuration)))));
        if (worse) {
            eviction = candidate;
        }
    }
    if (eviction == records_.end()) {
        return false;
    }
    records_.erase(eviction);
    evicted = true;
    return true;
}

bool ConfigurationCandidateArchive::retainSeed(
        ConfigurationCandidateRecord& record,
        ScreenedConfigurationSeed seed) const {
    seed.configuration = canonicalize(seed.configuration);
    if (seed.configuration != record.configuration ||
        !std::isfinite(seed.objective) ||
        !std::isfinite(seed.diversity) || seed.diversity < 0.0) {
        throw std::invalid_argument(
            "candidate seed is invalid or has a different configuration");
    }
    bool replacedStructuralClone = false;
    for (ScreenedConfigurationSeed& retained : record.diverseSeeds) {
        if (sameSeedStructure(retained, seed)) {
            if (seed.objective + policy_.objectiveTolerance >=
                retained.objective) {
                return false;
            }
            retained = std::move(seed);
            replacedStructuralClone = true;
            break;
        }
    }
    if (!replacedStructuralClone) {
        record.diverseSeeds.push_back(std::move(seed));
    }
    std::sort(
        record.diverseSeeds.begin(), record.diverseSeeds.end(),
        screenedConfigurationSeedHasHigherPriority);
    if (record.diverseSeeds.size() > policy_.seedLimit) {
        const auto best = record.diverseSeeds.front();
        std::sort(
            record.diverseSeeds.begin() + 1,
            record.diverseSeeds.end(),
            [](const ScreenedConfigurationSeed& left,
               const ScreenedConfigurationSeed& right) {
                if (left.diversity != right.diversity) {
                    return left.diversity > right.diversity;
                }
                if (left.objective != right.objective) {
                    return left.objective < right.objective;
                }
                return left.sourceIndex < right.sourceIndex;
            });
        record.diverseSeeds.resize(policy_.seedLimit);
        const auto bestFound = std::find_if(
            record.diverseSeeds.begin(), record.diverseSeeds.end(),
            [&best](const ScreenedConfigurationSeed& retained) {
                return retained.sourceIndex == best.sourceIndex &&
                    retained.objective == best.objective;
            });
        if (bestFound == record.diverseSeeds.end()) {
            record.diverseSeeds.back() = best;
        }
    }
    return true;
}

bool ConfigurationCandidateArchive::retainValidatedSeed(
        ScreenedConfigurationSeed seed) {
    const DepotConfigurationKey canonical =
        canonicalize(seed.configuration);
    auto found = records_.find(canonical);
    if (found == records_.end()) {
        throw std::logic_error(
            "cannot retain a seed without a working candidate record");
    }
    seed.configuration = canonical;
    return retainSeed(found->second, std::move(seed));
}

ConfigurationCandidateObservationResult
ConfigurationCandidateArchive::observe(
        const ConfigurationCandidateObservation& observation,
        const ConfigurationCandidateEligibilityContext& context) {
    if (!std::isfinite(observation.objective)) {
        throw std::invalid_argument(
            "candidate objective must be finite");
    }
    const DepotConfigurationKey canonical =
        canonicalize(observation.configuration);
    if (observation.source == ConfigurationDiscoverySource::Count) {
        throw std::invalid_argument(
            "candidate discovery source is invalid");
    }
    if (observation.beforeConfiguration.has_value()) {
        const DepotConfigurationKey canonicalBefore =
            makeDepotConfigurationKey(
                observation.beforeConfiguration->openDepots);
        if (canonicalBefore != *observation.beforeConfiguration) {
            throw std::invalid_argument(
                "candidate pre-discovery configuration is not canonical");
        }
    }
    if (observation.seed.has_value() &&
        !observation.independentlyValidated) {
        throw std::invalid_argument(
            "candidate seed requires independent validation");
    }
    const bool explicitEvidenceUnit =
        !observation.evidenceUnitId.empty();
    const std::string evidenceUnitId =
        !explicitEvidenceUnit
        ? "legacy_attempt:" + std::to_string(observation.attempt)
        : observation.evidenceUnitId;
    auto historyFound = observationHistory_.find(canonical);
    ConfigurationCandidateObservationResult result;
    if (historyFound == observationHistory_.end()) {
        ConfigurationObservationHistory history;
        history.firstSeenAttempt = observation.attempt;
        history.lastSeenAttempt = observation.attempt;
        historyFound = observationHistory_
            .emplace(canonical, std::move(history)).first;
        result.observationHistoryCreated = true;
    }
    ConfigurationObservationHistory& history = historyFound->second;
    if (history.totalObservations != 0U &&
        observation.attempt < history.lastSeenAttempt) {
        throw std::invalid_argument(
            "candidate observations must be submitted in attempt order");
    }
    if (history.totalObservations == 0U ||
        history.currentEvidenceAttempt != observation.attempt) {
        history.currentEvidenceAttempt = observation.attempt;
        history.currentAttemptEvidenceUnitIds.clear();
    }
    if (explicitEvidenceUnit) {
        if (!history.currentAttemptEvidenceUnitIds
                 .insert(evidenceUnitId).second) {
            result.duplicateEvidenceUnit = true;
            return result;
        }
        result.uniqueEvidenceUnitObservation = true;
    }
    const bool uniqueGlobalAttempt = history.totalObservations == 0U ||
        history.lastSeenAttempt != observation.attempt;
    if (uniqueGlobalAttempt) {
        ++history.distinctAttemptCount;
        history.lastAttemptBestObjective = observation.objective;
        result.uniqueAttemptObservation = true;
    } else {
        history.lastAttemptBestObjective = std::min(
            history.lastAttemptBestObjective, observation.objective);
    }
    if (explicitEvidenceUnit || uniqueGlobalAttempt) {
        ++history.totalObservations;
    }
    history.lastSeenAttempt = observation.attempt;
    history.bestObjective = std::min(
        history.bestObjective, observation.objective);
    const std::size_t sourceIndex =
        static_cast<std::size_t>(observation.source);
    ++history.observationsBySource[sourceIndex];
    if (observation.admittedToFree) {
        ++history.admittedFreeBySource[sourceIndex];
    }
    history.contributorMask |= observation.contributorMask |
        configurationDiscoverySourceBit(observation.source);
    history.lastSource = observation.source;
    history.lastEvidenceUnitId = evidenceUnitId;
    history.lastRouteAncestry = observation.routeAncestry;
    auto trialFound = trialHistory_.find(canonical);
    if (trialFound != trialHistory_.end()) {
        ConfigurationTrialHistory& trial = trialFound->second;
        const bool hasEvidenceBoundary =
            trial.totalRejections > 0U ||
            trial.totalResidentRetirements > 0U;
        if (hasEvidenceBoundary &&
            observation.attempt >
                trial.latestEvidenceBoundaryAttempt) {
            trial.bestObjectiveAfterLatestEvidenceBoundary = std::min(
                trial.bestObjectiveAfterLatestEvidenceBoundary,
                observation.objective);
        }
    }

    auto found = records_.find(canonical);
    if (found == records_.end()) {
        ConfigurationCandidateEligibilityContext evictionContext = context;
        evictionContext.attempt = observation.attempt;
        bool evicted = false;
        if (!makeRoomForNewRecord(evictionContext, evicted)) {
            result.capacityRejected = true;
            return result;
        }
        result.recordEvicted = evicted;
        ConfigurationCandidateRecord record;
        record.configuration = canonical;
        record.firstSeenAttempt = observation.attempt;
        record.lastSeenAttempt = observation.attempt;
        record.totalObservationCount = history.totalObservations;
        record.observationsBySource = history.observationsBySource;
        record.admittedFreeBySource = history.admittedFreeBySource;
        record.contributorMask = history.contributorMask;
        record.lastSource = history.lastSource;
        record.lastEvidenceUnitId = history.lastEvidenceUnitId;
        record.lastRouteAncestry = history.lastRouteAncestry;
        found = records_.emplace(canonical, std::move(record)).first;
        result.recordCreated = true;
        result.recordRecreated = !result.observationHistoryCreated;
    }

    ConfigurationCandidateRecord& record = found->second;
    if (record.observationCount != 0U &&
        observation.attempt < record.lastSeenAttempt) {
        throw std::invalid_argument(
            "candidate observations must be submitted in attempt order");
    }
    const bool uniqueRecordAttempt = record.observationCount == 0U ||
        record.lastSeenAttempt != observation.attempt;
    if (explicitEvidenceUnit || uniqueRecordAttempt) {
        ++record.observationCount;
        record.lastSeenAttempt = observation.attempt;
        record.lastAttemptBestObjective =
            history.lastAttemptBestObjective;
        record.bestObjective =
            std::min(record.bestObjective, observation.objective);
        record.topObjectives.push_back(observation.objective);
        std::sort(
            record.topObjectives.begin(), record.topObjectives.end());
        if (record.topObjectives.size() > policy_.topObjectiveLimit) {
            record.topObjectives.resize(policy_.topObjectiveLimit);
        }
    } else if (observation.objective <
               record.lastAttemptBestObjective) {
        const auto previous = std::find(
            record.topObjectives.begin(), record.topObjectives.end(),
            record.lastAttemptBestObjective);
        if (previous != record.topObjectives.end()) {
            record.topObjectives.erase(previous);
        }
        record.lastAttemptBestObjective = observation.objective;
        record.bestObjective =
            std::min(record.bestObjective, observation.objective);
        record.topObjectives.push_back(observation.objective);
        std::sort(
            record.topObjectives.begin(), record.topObjectives.end());
        if (record.topObjectives.size() > policy_.topObjectiveLimit) {
            record.topObjectives.resize(policy_.topObjectiveLimit);
        }
    }
    record.totalObservationCount = history.totalObservations;
    record.observationsBySource = history.observationsBySource;
    record.admittedFreeBySource = history.admittedFreeBySource;
    record.contributorMask = history.contributorMask;
    record.lastSource = history.lastSource;
    record.lastEvidenceUnitId = history.lastEvidenceUnitId;
    record.lastRouteAncestry = history.lastRouteAncestry;

    if (observation.admittedToFree) {
        ++record.admittedFreeCount;
        if (observation.seed.has_value()) {
            result.seedRetained = retainSeed(
                record, *observation.seed);
        }
    } else if (observation.seed.has_value()) {
        // A CRH/ST-CRH screening seed or a duplicate rejected by free-
        // population survivor selection is still independently validated
        // evidence. It may seed the challenger without claiming free admission.
        result.seedRetained = retainSeed(record, *observation.seed);
    }
    return result;
}

void ConfigurationCandidateArchive::recordTrialStarted(
        const DepotConfigurationKey& configuration,
        const std::uint64_t challengerGeneration) {
    const DepotConfigurationKey canonical = canonicalize(configuration);
    auto found = records_.find(canonical);
    if (found == records_.end()) {
        throw std::logic_error(
            "cannot start a trial for an unknown configuration");
    }
    ConfigurationTrialHistory& history = trialHistory_[canonical];
    ++history.totalTrials;
    history.lastTrialGeneration = challengerGeneration;
}

void ConfigurationCandidateArchive::recordRejection(
        const DepotConfigurationKey& configuration,
        const std::uint64_t rejectionAttempt,
        const std::uint64_t cooldownUntilAttempt) {
    const DepotConfigurationKey canonical = canonicalize(configuration);
    const auto observation = observationHistory_.find(canonical);
    if (observation == observationHistory_.end()) {
        throw std::logic_error(
            "cannot reject a configuration with no observation history");
    }
    ConfigurationTrialHistory& history = trialHistory_[canonical];
    if (cooldownUntilAttempt < rejectionAttempt) {
        throw std::invalid_argument(
            "candidate rejection cooldown precedes rejection");
    }
    if ((history.totalRejections > 0U ||
         history.totalResidentRetirements > 0U) &&
        rejectionAttempt < history.latestEvidenceBoundaryAttempt) {
        throw std::invalid_argument(
            "candidate rejection precedes the latest evidence boundary");
    }
    ++history.totalRejections;
    history.lastRejectionAttempt = rejectionAttempt;
    history.cooldownUntilAttempt = cooldownUntilAttempt;
    history.totalObservationsAtLastRejection =
        observation->second.totalObservations;
    history.bestObjectiveAtLastRejection =
        observation->second.bestObjective;
    history.latestEvidenceBoundaryAttempt = rejectionAttempt;
    history.latestEvidenceCooldownUntilAttempt = cooldownUntilAttempt;
    history.totalObservationsAtLatestEvidenceBoundary =
        observation->second.totalObservations;
    history.bestObjectiveAtLatestEvidenceBoundary =
        observation->second.bestObjective;
    history.bestObjectiveAfterLatestEvidenceBoundary =
        std::numeric_limits<double>::infinity();
}

void ConfigurationCandidateArchive::recordResidentRetirement(
        const DepotConfigurationKey& configuration,
        const std::uint64_t retirementAttempt,
        const std::uint64_t cooldownUntilAttempt,
        const double bestObjectiveAtRetirement) {
    if (!std::isfinite(bestObjectiveAtRetirement)) {
        throw std::invalid_argument(
            "resident retirement requires a finite best objective");
    }
    if (cooldownUntilAttempt < retirementAttempt) {
        throw std::invalid_argument(
            "resident retirement cooldown precedes retirement");
    }
    const DepotConfigurationKey canonical = canonicalize(configuration);
    const auto observation = observationHistory_.find(canonical);
    ConfigurationTrialHistory& history = trialHistory_[canonical];
    if ((history.totalRejections > 0U ||
         history.totalResidentRetirements > 0U) &&
        retirementAttempt < history.latestEvidenceBoundaryAttempt) {
        throw std::invalid_argument(
            "resident retirement precedes the latest evidence boundary");
    }
    ++history.totalResidentRetirements;
    history.lastResidentRetirementAttempt = retirementAttempt;
    history.retirementCooldownUntilAttempt = cooldownUntilAttempt;
    history.totalObservationsAtLastRetirement =
        observation == observationHistory_.end()
            ? 0U
            : observation->second.totalObservations;
    history.bestObjectiveAtLastRetirement = std::min(
        bestObjectiveAtRetirement,
        observation == observationHistory_.end()
            ? std::numeric_limits<double>::infinity()
            : observation->second.bestObjective);
    history.latestEvidenceBoundaryAttempt = retirementAttempt;
    history.latestEvidenceCooldownUntilAttempt = cooldownUntilAttempt;
    history.totalObservationsAtLatestEvidenceBoundary =
        history.totalObservationsAtLastRetirement;
    history.bestObjectiveAtLatestEvidenceBoundary =
        history.bestObjectiveAtLastRetirement;
    history.bestObjectiveAfterLatestEvidenceBoundary =
        std::numeric_limits<double>::infinity();
}

bool ConfigurationCandidateArchive::erasePromoted(
        const DepotConfigurationKey& configuration) {
    const DepotConfigurationKey canonical = canonicalize(configuration);
    ConfigurationTrialHistory& history = trialHistory_[canonical];
    ++history.totalPromotions;
    history.everPromoted = true;
    return records_.erase(canonical) != 0U;
}

void ConfigurationCandidateArchive::swap(
        ConfigurationCandidateArchive& other) noexcept {
    using std::swap;
    swap(policy_, other.policy_);
    records_.swap(other.records_);
    observationHistory_.swap(other.observationHistory_);
    trialHistory_.swap(other.trialHistory_);
}

}  // namespace hgasp
