#ifndef HGASP_MODELS_ROUTE_PROVENANCE_H_
#define HGASP_MODELS_ROUTE_PROVENANCE_H_

#include "RouteKey.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace hgasp {

enum class ConfigurationDiscoverySource : std::uint8_t {
    CrhInitialization,
    StCrhInitialization,
    FreeInitialization,
    MdEaxRecombination,
    ConstraintFeasibleReconstruction,
    CapacityRepair,
    Mutation,
    LocalSearch,
    GlobalSp,
    Count,
};

constexpr std::size_t configurationDiscoverySourceCount() noexcept {
    return static_cast<std::size_t>(
        ConfigurationDiscoverySource::Count);
}

using ConfigurationDiscoveryContributorMask = std::uint32_t;

constexpr ConfigurationDiscoveryContributorMask
configurationDiscoverySourceBit(
        const ConfigurationDiscoverySource source) noexcept {
    return source == ConfigurationDiscoverySource::Count
        ? 0U
        : ConfigurationDiscoveryContributorMask{1U}
            << static_cast<unsigned int>(source);
}

const char* toString(ConfigurationDiscoverySource source) noexcept;

enum class RouteOriginKind {
    InitialConstruction,
    EaxOffspring,
    Mutation,
    LocalSearch,
    SpInjection,
};

enum class RoutePopulationRole {
    Unspecified,
    FixedResident,
    UnrestrictedFree,
    DynamicChallenger,
    InactiveOrSpare,
};

enum class SpFeedbackScope {
    ConfigurationLocal,
    Global,
};

// Frozen at parent selection, before mdEAX can create a multi-child
// batch.  Every field is owned by value so a sibling may retire, replace, or
// restart the physical population without changing later-child lineage.
struct ParentLineageSnapshot {
    std::uint64_t individualId = 0U;
    int populationIndex = -1;
    RoutePopulationRole populationRole =
        RoutePopulationRole::Unspecified;
    DepotConfigurationKey actualConfiguration;
    std::uint64_t populationGeneration = 0U;
    int challengerSlot = -1;
    std::uint64_t challengerGeneration = 0U;
    std::string spAncestryCallId;

    bool valid() const noexcept {
        return individualId != 0U && populationIndex >= 0;
    }
};

struct OffspringBatchLineageSnapshot {
    ParentLineageSnapshot first;
    ParentLineageSnapshot second;
    bool initialized = false;
};

inline DepotConfigurationKey offspringConfigurationDiscoveryBaseline(
        const OffspringBatchLineageSnapshot& batch,
        const DepotConfigurationKey& offspring) {
    if (!batch.initialized) {
        return offspring;
    }
    if (offspring == batch.first.actualConfiguration ||
        offspring == batch.second.actualConfiguration) {
        return offspring;
    }
    return batch.first.actualConfiguration.openDepots.empty()
        ? offspring
        : batch.first.actualConfiguration;
}

struct ChallengerRouteOrigin {
    int slot = -1;
    std::uint64_t generation = 0U;
    DepotConfigurationKey configuration;

    bool valid() const noexcept {
        return slot >= 0 && generation != 0U;
    }

    bool operator==(const ChallengerRouteOrigin& other) const {
        return slot == other.slot && generation == other.generation &&
            configuration == other.configuration;
    }
};

struct RouteSubmissionContext {
    std::uint64_t traceId = 0U;
    RouteOriginKind origin =
        RouteOriginKind::InitialConstruction;
    int iteration = 0;
    int firstPopulation = -1;
    int secondPopulation = -1;
    RoutePopulationRole firstPopulationRole =
        RoutePopulationRole::Unspecified;
    RoutePopulationRole secondPopulationRole =
        RoutePopulationRole::Unspecified;
    DepotConfigurationKey firstOriginConfiguration;
    DepotConfigurationKey secondOriginConfiguration;
    std::uint64_t firstPopulationGeneration = 0U;
    std::uint64_t secondPopulationGeneration = 0U;
    // The physical challenger slot is reused; generation distinguishes its
    // independently bounded trials without conflating them with legacy TW
    // island identity.
    std::uint64_t firstChallengerGeneration = 0U;
    std::uint64_t secondChallengerGeneration = 0U;
    int firstChallengerSlot = -1;
    int secondChallengerSlot = -1;
    std::uint64_t firstParentIndividualId = 0U;
    std::uint64_t secondParentIndividualId = 0U;
    std::string spAncestryCallId;
    // Stable identity of one configuration-discovery evidence unit.  HGA
    // uses batch+child, SP uses call+candidate+raw/ls, and initialization
    // uses a deterministic seed/construction identity.
    std::string configurationEvidenceUnitId;
    bool mutationCommitted = false;
    bool configurationChanged = false;
    ConfigurationDiscoverySource configurationDiscoverySource =
        ConfigurationDiscoverySource::Count;
    ConfigurationDiscoveryContributorMask
        configurationDiscoveryContributors = 0U;
    DepotConfigurationKey configurationBefore;
    DepotConfigurationKey configurationAfter;
};

inline void enrichConfigurationDiscoverySubmission(
        RouteSubmissionContext& context,
        const DepotConfigurationKey& before,
        const DepotConfigurationKey& after,
        const ConfigurationDiscoverySource source,
        const ConfigurationDiscoveryContributorMask contributors,
        std::string evidenceUnitId) {
    context.configurationBefore = before;
    context.configurationAfter = after;
    context.configurationChanged = before != after;
    context.configurationDiscoverySource = context.configurationChanged
        ? source
        : ConfigurationDiscoverySource::Count;
    context.configurationDiscoveryContributors =
        context.configurationChanged ? contributors : 0U;
    context.configurationEvidenceUnitId = std::move(evidenceUnitId);
}

struct ColumnProvenanceStats {
    struct LocalUsefulScoreBaseline {
        DepotConfigurationKey configuration;
        double bestObservedUsefulScore = 0.0;
        std::uint64_t lastTouchedFeedback = 0U;
    };

    RouteSubmissionContext bestRouteOrigin;
    // Live source ownership remains bestRouteOrigin. These bounded sidecars
    // retain challenger ancestry after the physical slot is retired or a
    // resident/free route becomes the best representative of the column.
    ChallengerRouteOrigin firstChallengerOrigin;
    ChallengerRouteOrigin bestRouteOriginChallenger;
    std::vector<ChallengerRouteOrigin> historicalChallengerOrigins;
    double lpUsageEma = 0.0;
    double reducedCostQualityEma = 0.0;
    double spSelectionFrequencyEma = 0.0;
    double globalBestObservedUsefulScore = 0.0;
    std::vector<LocalUsefulScoreBaseline> localUsefulScoreBaselines;
    double cumulativeUsefulCredit = 0.0;
    int lastSelectedIteration = -1;
};

// Bounded historical-only sidecar for master columns retained by the route
// archive after active-master pruning. It deliberately excludes live credit,
// LP feedback and current population ownership.
struct ArchivedChallengerColumnProvenance {
    ChallengerRouteOrigin firstChallengerOrigin;
    ChallengerRouteOrigin bestRouteOriginChallenger;
    std::vector<ChallengerRouteOrigin> historicalChallengerOrigins;
};

struct OriginCredit {
    int population = -1;
    std::uint64_t traceId = 0U;
    std::uint64_t challengerGeneration = 0U;
    double credit = 0.0;
    double selectedColumns = 0.0;
    double historicalChallengerSelectedColumns = 0.0;
    DepotConfigurationKey challengerConfiguration;
};

inline bool isActiveHistoricalChallengerOrigin(
        const OriginCredit& credit,
        const int activeSlot,
        const std::uint64_t activeGeneration) noexcept {
    return credit.historicalChallengerSelectedColumns > 0.0 &&
        credit.challengerGeneration != 0U &&
        credit.population == activeSlot &&
        credit.challengerGeneration == activeGeneration;
}

}  // namespace hgasp

#endif  // HGASP_MODELS_ROUTE_PROVENANCE_H_
