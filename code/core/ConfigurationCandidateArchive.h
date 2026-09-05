#ifndef HGASP_CORE_CONFIGURATION_CANDIDATE_ARCHIVE_H_
#define HGASP_CORE_CONFIGURATION_CANDIDATE_ARCHIVE_H_

#include "ScreenedConfigurationSeed.h"
#include "RouteProvenance.h"

#include <cstddef>
#include <cstdint>
#include <array>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <string_view>
#include <string>
#include <vector>

namespace hgasp {

using ConfigurationDiscoverySourceCounts = std::array<
    std::uint64_t, configurationDiscoverySourceCount()>;

// Local-SP scopes maintain a fixed master configuration and are not depot-
// configuration discovery, even when their population admission target is
// implicit. Empty call ids are retained as the legacy Global-SP scope.
bool isGlobalSpConfigurationDiscoveryCall(
    std::string_view spCallId) noexcept;

// The unrestricted bootstrap is bulk-observed exactly once, when the static
// registry is first materialized. Restart generations are already attributed
// per admission in initialPop() and must not be traversed again.
class InitialFreePopulationObservationGate {
public:
    bool claimBulkObservation(
            bool registryAlreadyMaterialized) noexcept {
        if (registryAlreadyMaterialized || claimed_) {
            return false;
        }
        claimed_ = true;
        return true;
    }

private:
    bool claimed_ = false;
};

struct PostSpLocalSearchDiscoveryDecision {
    bool retainBeforeAuthoritativeRestore = false;
    bool restoreAuthoritativeCombination = false;
    ConfigurationDiscoverySource retainedSource =
        ConfigurationDiscoverySource::LocalSearch;
    ConfigurationDiscoveryContributorMask retainedContributors = 0U;
    ConfigurationDiscoveryContributorMask injectionContributors = 0U;
};

inline PostSpLocalSearchDiscoveryDecision
postSpLocalSearchDiscoveryDecision(
        const bool localSp,
        const bool configurationChanged,
        const bool worsenedRawSpObjective) noexcept {
    PostSpLocalSearchDiscoveryDecision decision;
    decision.restoreAuthoritativeCombination =
        worsenedRawSpObjective || (localSp && configurationChanged);
    decision.retainBeforeAuthoritativeRestore =
        configurationChanged &&
        decision.restoreAuthoritativeCombination;
    (void)localSp;
    decision.retainedSource =
        ConfigurationDiscoverySource::LocalSearch;
    if (configurationChanged) {
        decision.retainedContributors =
            configurationDiscoverySourceBit(
                ConfigurationDiscoverySource::LocalSearch);
        if (!decision.restoreAuthoritativeCombination) {
            decision.injectionContributors =
                configurationDiscoverySourceBit(
                    ConfigurationDiscoverySource::LocalSearch);
        }
    }
    return decision;
}

enum class ConfigurationChangeKind : std::uint8_t {
    Unchanged,
    Open,
    Close,
    Swap,
    Mixed,
};

const char* toString(ConfigurationChangeKind kind) noexcept;

ConfigurationChangeKind classifyConfigurationChange(
    const DepotConfigurationKey& before,
    const DepotConfigurationKey& after) noexcept;

struct ConfigurationCandidateArchivePolicy {
    std::size_t maximumRecords = 64U;
    std::size_t topObjectiveLimit = 3U;
    std::size_t seedLimit = 4U;
    std::uint64_t repeatedDiscoveryThreshold = 3U;
    std::uint64_t newObservationsAfterRejection = 3U;
    double objectiveTolerance = 1.0e-6;
};

struct ConfigurationCandidateRecord {
    DepotConfigurationKey configuration;
    std::uint64_t firstSeenAttempt = 0U;
    std::uint64_t lastSeenAttempt = 0U;
    std::uint64_t observationCount = 0U;
    std::uint64_t totalObservationCount = 0U;
    std::uint64_t admittedFreeCount = 0U;
    double bestObjective = std::numeric_limits<double>::infinity();
    double lastAttemptBestObjective =
        std::numeric_limits<double>::infinity();
    std::vector<double> topObjectives;
    std::vector<ScreenedConfigurationSeed> diverseSeeds;
    ConfigurationDiscoverySourceCounts observationsBySource{};
    ConfigurationDiscoverySourceCounts admittedFreeBySource{};
    ConfigurationDiscoveryContributorMask contributorMask = 0U;
    ConfigurationDiscoverySource lastSource =
        ConfigurationDiscoverySource::FreeInitialization;
    std::string lastEvidenceUnitId;
    std::string lastRouteAncestry;
    double top3Mean() const noexcept;
};

struct ConfigurationObservationHistory {
    std::uint64_t totalObservations = 0U;
    std::uint64_t distinctAttemptCount = 0U;
    std::uint64_t firstSeenAttempt = 0U;
    std::uint64_t lastSeenAttempt = 0U;
    double bestObjective = std::numeric_limits<double>::infinity();
    double lastAttemptBestObjective =
        std::numeric_limits<double>::infinity();
    ConfigurationDiscoverySourceCounts observationsBySource{};
    ConfigurationDiscoverySourceCounts admittedFreeBySource{};
    ConfigurationDiscoveryContributorMask contributorMask = 0U;
    ConfigurationDiscoverySource lastSource =
        ConfigurationDiscoverySource::FreeInitialization;
    std::string lastEvidenceUnitId;
    std::string lastRouteAncestry;
    std::uint64_t currentEvidenceAttempt = 0U;
    std::set<std::string> currentAttemptEvidenceUnitIds;
};

struct ConfigurationTrialHistory {
    std::uint64_t totalTrials = 0U;
    std::uint64_t totalPromotions = 0U;
    std::uint64_t totalRejections = 0U;
    std::uint64_t lastTrialGeneration = 0U;
    std::uint64_t lastRejectionAttempt = 0U;
    std::uint64_t cooldownUntilAttempt = 0U;
    std::uint64_t totalObservationsAtLastRejection = 0U;
    double bestObjectiveAtLastRejection =
        std::numeric_limits<double>::infinity();
    std::uint64_t totalResidentRetirements = 0U;
    std::uint64_t lastResidentRetirementAttempt = 0U;
    std::uint64_t retirementCooldownUntilAttempt = 0U;
    std::uint64_t totalObservationsAtLastRetirement = 0U;
    double bestObjectiveAtLastRetirement =
        std::numeric_limits<double>::infinity();
    // Eligibility after a rejection or resident retirement must be judged
    // with observations submitted after the latest evidence boundary.  The
    // lifetime observation best remains useful for provenance, but reusing it
    // here would let stale quality re-qualify a configuration after cooldown.
    std::uint64_t latestEvidenceBoundaryAttempt = 0U;
    std::uint64_t latestEvidenceCooldownUntilAttempt = 0U;
    std::uint64_t totalObservationsAtLatestEvidenceBoundary = 0U;
    double bestObjectiveAtLatestEvidenceBoundary =
        std::numeric_limits<double>::infinity();
    double bestObjectiveAfterLatestEvidenceBoundary =
        std::numeric_limits<double>::infinity();
    bool everPromoted = false;
};

struct ConfigurationCandidateObservation {
    DepotConfigurationKey configuration;
    std::uint64_t attempt = 0U;
    double objective = std::numeric_limits<double>::infinity();
    bool admittedToFree = false;
    std::optional<ScreenedConfigurationSeed> seed;
    ConfigurationDiscoverySource source =
        ConfigurationDiscoverySource::FreeInitialization;
    ConfigurationDiscoveryContributorMask contributorMask = 0U;
    std::string evidenceUnitId;
    std::string routeAncestry;
    std::optional<DepotConfigurationKey> beforeConfiguration;
    bool independentlyValidated = false;
    bool strictGlobalIncumbent = false;
};

struct ConfigurationCandidateObservationResult {
    bool recordCreated = false;
    bool recordRecreated = false;
    bool recordEvicted = false;
    bool observationHistoryCreated = false;
    bool uniqueAttemptObservation = false;
    bool uniqueEvidenceUnitObservation = false;
    bool duplicateEvidenceUnit = false;
    bool seedRetained = false;
    bool capacityRejected = false;
};

// Total deterministic order shared by transient pending discoveries and the
// persistent archive: lower objective, then higher structural diversity,
// canonical configuration and source index. Exact ties retain arrival order,
// preserving the existing archive behavior without route-key materialization.
// Returning true means `left` deserves retention ahead of `right`.
bool screenedConfigurationSeedHasHigherPriority(
    const ScreenedConfigurationSeed& left,
    const ScreenedConfigurationSeed& right);

struct ResidentQualitySnapshot {
    std::size_t populationIndex = 0U;
    double bestObjective = std::numeric_limits<double>::infinity();
    double eliteMean10 = std::numeric_limits<double>::infinity();
};

enum class ConfigurationCandidateEligibilityReason {
    ActiveResident,
    CurrentChallenger,
    MissingSeed,
    NoReplaceableResident,
    Cooldown,
    NoNewEvidenceAfterRejection,
    InsufficientObservations,
    QualityFloor,
    ExceptionalBest,
    RepeatedWithQualityFloor,
};

const char* toString(ConfigurationCandidateEligibilityReason reason) noexcept;

struct ConfigurationCandidateEligibilityDecision {
    bool eligible = false;
    ConfigurationCandidateEligibilityReason reason =
        ConfigurationCandidateEligibilityReason::InsufficientObservations;
};

struct ConfigurationCandidateEligibilityContext {
    std::uint64_t attempt = 0U;
    std::vector<DepotConfigurationKey> activeResidents;
    std::optional<DepotConfigurationKey> currentChallenger;
    std::optional<ResidentQualitySnapshot> weakestReplaceableResident;
    // Eligibility compares quality against the resident portfolio even when
    // every resident is temporarily protected.  Promotion authority remains
    // tied to weakestReplaceableResident and is evaluated after the trial.
    std::optional<ResidentQualitySnapshot> qualityReferenceResident;
};

// Configuration-aware metadata sidecar for the permanent unrestricted slot.
// It does not generate candidates and never owns Individuals or populations.
class ConfigurationCandidateArchive {
public:
    explicit ConfigurationCandidateArchive(
        ConfigurationCandidateArchivePolicy policy = {});

    const ConfigurationCandidateArchivePolicy& policy() const noexcept {
        return policy_;
    }

    std::size_t size() const noexcept {
        return records_.size();
    }

    const std::map<DepotConfigurationKey,
                   ConfigurationCandidateRecord>&
    records() const noexcept {
        return records_;
    }

    const std::map<DepotConfigurationKey,
                   ConfigurationObservationHistory>&
    observationHistory() const noexcept {
        return observationHistory_;
    }

    const std::map<DepotConfigurationKey,
                   ConfigurationTrialHistory>&
    trialHistory() const noexcept {
        return trialHistory_;
    }

    const ConfigurationObservationHistory* findObservationHistory(
        const DepotConfigurationKey& configuration) const;

    const ConfigurationTrialHistory* findTrialHistory(
        const DepotConfigurationKey& configuration) const;

    const ConfigurationCandidateRecord* find(
        const DepotConfigurationKey& configuration) const;

    ConfigurationCandidateObservationResult observe(
        const ConfigurationCandidateObservation& observation,
        const ConfigurationCandidateEligibilityContext& context = {});

    // Materializes route-heavy seed evidence only after observe() has kept a
    // working record. Observation history and capacity decisions therefore do
    // not pay for a snapshot that the bounded archive will immediately reject.
    bool retainValidatedSeed(ScreenedConfigurationSeed seed);

    ConfigurationCandidateEligibilityDecision evaluateEligibility(
        const ConfigurationCandidateRecord& record,
        const ConfigurationCandidateEligibilityContext& context) const;

    bool isEligible(
        const ConfigurationCandidateRecord& record,
        const ConfigurationCandidateEligibilityContext& context) const {
        return evaluateEligibility(record, context).eligible;
    }

    std::vector<const ConfigurationCandidateRecord*> eligibleRecords(
        const ConfigurationCandidateEligibilityContext& context) const;

    void recordTrialStarted(
        const DepotConfigurationKey& configuration,
        std::uint64_t challengerGeneration);

    void recordRejection(
        const DepotConfigurationKey& configuration,
        std::uint64_t rejectionAttempt,
        std::uint64_t cooldownUntilAttempt);

    // A promoted or initial resident becomes a non-resident hypothesis again
    // only after a fresh evidence boundary. This history is persistent even
    // when no working candidate record existed before retirement.
    void recordResidentRetirement(
        const DepotConfigurationKey& configuration,
        std::uint64_t retirementAttempt,
        std::uint64_t cooldownUntilAttempt,
        double bestObjectiveAtRetirement);

    bool erasePromoted(const DepotConfigurationKey& configuration);

    void swap(ConfigurationCandidateArchive& other) noexcept;

private:
    using RecordMap = std::map<DepotConfigurationKey,
                               ConfigurationCandidateRecord>;

    DepotConfigurationKey canonicalize(
        const DepotConfigurationKey& configuration) const;

    bool makeRoomForNewRecord(
        const ConfigurationCandidateEligibilityContext& context,
        bool& evicted);

    bool retainSeed(
        ConfigurationCandidateRecord& record,
        ScreenedConfigurationSeed seed) const;

    ConfigurationCandidateArchivePolicy policy_;
    RecordMap records_;
    std::map<DepotConfigurationKey, ConfigurationObservationHistory>
        observationHistory_;
    std::map<DepotConfigurationKey, ConfigurationTrialHistory>
        trialHistory_;
};

}  // namespace hgasp

#endif  // HGASP_CORE_CONFIGURATION_CANDIDATE_ARCHIVE_H_
