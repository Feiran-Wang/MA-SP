#ifndef HGASP_CORE_STATIC_HGAMP_POPULATION_POLICY_H_
#define HGASP_CORE_STATIC_HGAMP_POPULATION_POLICY_H_

#include "RouteKey.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace hgasp {

inline constexpr std::size_t
    kStaticCrossConfigurationSelectionMaxAttempts = 64U;

enum class StaticMatingScope {
    ResidentReproduction,
    ConfigurationDiscovery,
};

// Ordinary resident reproduction follows the HGAMP tournament and may select
// both parents from the same population. A distinct-configuration contract is
// reserved for an explicitly scheduled discovery/challenger turn.
bool requiresDistinctConfigurationMating(
    StaticMatingScope scope,
    bool strictDiscoveryMatingEnabled) noexcept;

struct StaticPopulationTournamentCandidate {
    std::size_t populationIndex = 0U;
    bool active = true;
    bool nonEmpty = true;
    double meanObjective = 0.0;
};

struct StaticPopulationTournamentResult {
    bool selected = false;
    std::size_t populationIndex = 0U;
    std::size_t eligiblePopulationCount = 0U;
};

// Selects the authoritative candidate source used by static HGAMP
// preprocessing. An explicit depot-configuration file takes precedence over
// legacy preprocessed candidates. Every result is canonical and unique while
// retaining first-occurrence order.
std::vector<DepotConfigurationKey> selectStaticConfigurationCandidates(
    const std::vector<std::vector<int>>& fileCandidates,
    const std::vector<DepotConfigurationKey>& legacyCandidates,
    std::size_t depotCount);

// Performs one HGAMP-compatible two-way objective tournament. Random draws are
// supplied by the owner, so this pure policy neither consumes nor reorders the
// HGA random stream. For N eligible candidates, firstDraw addresses [0,N) and
// secondDraw addresses the compressed [0,N-1) range excluding the first.
StaticPopulationTournamentResult selectStaticPopulationTournament(
    const std::vector<StaticPopulationTournamentCandidate>& populations,
    std::uint64_t firstDraw,
    std::uint64_t secondDraw,
    std::optional<std::size_t> excludedPopulation = std::nullopt);

struct ScheduledRotationCandidate {
    std::size_t populationIndex = 0U;
    bool active = true;
    bool carriesGlobalIncumbent = false;
    double bestObjective = 0.0;
};

// A checkpoint lease replaces the weakest active resident while preserving
// the resident that currently carries the global incumbent. Ties are stable
// by physical population index.
std::optional<std::size_t> selectScheduledRotationTarget(
    const std::vector<ScheduledRotationCandidate>& candidates,
    double objectiveTolerance);

struct StaticParentConfigurationPair {
    std::size_t firstPopulationIndex = 0U;
    std::size_t secondPopulationIndex = 0U;
    DepotConfigurationKey firstActualConfiguration;
    DepotConfigurationKey secondActualConfiguration;
};

enum class CrossConfigurationSelectionAction {
    Accept,
    Retry,
    SameConfigurationFallback,
};

enum class CrossConfigurationFallbackReason {
    None,
    OnlyOneDistinctConfiguration,
    CandidatePopulationsTooSmall,
    AttemptsExhausted,
};

const char* crossConfigurationFallbackReasonName(
    CrossConfigurationFallbackReason reason) noexcept;

struct CrossConfigurationSelectionDecision {
    CrossConfigurationSelectionAction action =
        CrossConfigurationSelectionAction::Retry;
    CrossConfigurationFallbackReason fallbackReason =
        CrossConfigurationFallbackReason::None;
    std::size_t attemptsUsed = 0U;

    bool acceptedDistinctConfiguration() const noexcept {
        return action == CrossConfigurationSelectionAction::Accept;
    }

    bool usedFallback() const noexcept {
        return action ==
            CrossConfigurationSelectionAction::SameConfigurationFallback;
    }
};

// Evaluates the actual parent configurations, not just population identities.
// attemptsUsed is one-based and includes the pair being evaluated.
CrossConfigurationSelectionDecision evaluateCrossConfigurationPair(
    const StaticParentConfigurationPair& pair,
    std::size_t distinctAvailableConfigurationCount,
    std::size_t eligiblePopulationCount,
    std::size_t attemptsUsed,
    std::size_t maximumAttempts =
        kStaticCrossConfigurationSelectionMaxAttempts);

}  // namespace hgasp

#endif  // HGASP_CORE_STATIC_HGAMP_POPULATION_POLICY_H_
