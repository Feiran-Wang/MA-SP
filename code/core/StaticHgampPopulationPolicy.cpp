#include "StaticHgampPopulationPolicy.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace hgasp {

bool requiresDistinctConfigurationMating(
        const StaticMatingScope scope,
        const bool strictDiscoveryMatingEnabled) noexcept {
    return strictDiscoveryMatingEnabled &&
        scope == StaticMatingScope::ConfigurationDiscovery;
}

std::vector<DepotConfigurationKey> selectStaticConfigurationCandidates(
        const std::vector<std::vector<int>>& fileCandidates,
        const std::vector<DepotConfigurationKey>& legacyCandidates,
        const std::size_t depotCount) {
    if (depotCount == 0U) {
        throw std::invalid_argument(
            "static configuration selection requires at least one depot");
    }
    std::vector<DepotConfigurationKey> selected;
    std::set<DepotConfigurationKey> seen;
    const auto admit = [&](std::vector<int> depots,
                           const char* source) {
        const DepotConfigurationKey key =
            makeDepotConfigurationKey(std::move(depots));
        if (key.openDepots.empty()) {
            throw std::invalid_argument(
                std::string(source) +
                " contains an empty depot configuration");
        }
        for (const int depot : key.openDepots) {
            if (depot < 0 ||
                static_cast<std::size_t>(depot) >= depotCount) {
                throw std::invalid_argument(
                    std::string(source) +
                    " contains an out-of-range depot index");
            }
        }
        if (seen.insert(key).second) {
            selected.push_back(key);
        }
    };
    if (!fileCandidates.empty()) {
        selected.reserve(fileCandidates.size());
        for (const std::vector<int>& candidate : fileCandidates) {
            admit(candidate, "depot configuration file");
        }
        return selected;
    }
    selected.reserve(legacyCandidates.size());
    for (const DepotConfigurationKey& candidate : legacyCandidates) {
        admit(candidate.openDepots, "legacy configuration candidates");
    }
    return selected;
}

StaticPopulationTournamentResult selectStaticPopulationTournament(
        const std::vector<StaticPopulationTournamentCandidate>& populations,
        const std::uint64_t firstDraw,
        const std::uint64_t secondDraw,
        const std::optional<std::size_t> excludedPopulation) {
    std::vector<StaticPopulationTournamentCandidate> eligible;
    eligible.reserve(populations.size());
    for (const StaticPopulationTournamentCandidate& population :
         populations) {
        if (!population.active || !population.nonEmpty ||
            (excludedPopulation.has_value() &&
             population.populationIndex == *excludedPopulation)) {
            continue;
        }
        if (std::isnan(population.meanObjective)) {
            throw std::invalid_argument(
                "static population tournament received NaN objective");
        }
        eligible.push_back(population);
    }
    std::sort(
        eligible.begin(),
        eligible.end(),
        [](const StaticPopulationTournamentCandidate& left,
           const StaticPopulationTournamentCandidate& right) {
            return left.populationIndex < right.populationIndex;
        });

    StaticPopulationTournamentResult result;
    result.eligiblePopulationCount = eligible.size();
    if (eligible.empty()) {
        return result;
    }
    const std::size_t first = static_cast<std::size_t>(
        firstDraw % eligible.size());
    std::size_t second = first;
    if (eligible.size() > 1U) {
        const std::size_t compressed = static_cast<std::size_t>(
            secondDraw % (eligible.size() - 1U));
        second = compressed >= first
            ? compressed + 1U
            : compressed;
    }
    const StaticPopulationTournamentCandidate* winner = &eligible[first];
    const StaticPopulationTournamentCandidate& challenger = eligible[second];
    if (challenger.meanObjective < winner->meanObjective ||
        (challenger.meanObjective == winner->meanObjective &&
         challenger.populationIndex < winner->populationIndex)) {
        winner = &challenger;
    }
    result.selected = true;
    result.populationIndex = winner->populationIndex;
    return result;
}

std::optional<std::size_t> selectScheduledRotationTarget(
        const std::vector<ScheduledRotationCandidate>& candidates,
        const double objectiveTolerance) {
    if (!std::isfinite(objectiveTolerance) || objectiveTolerance < 0.0) {
        throw std::invalid_argument(
            "scheduled rotation tolerance must be finite and nonnegative");
    }
    std::optional<std::size_t> selected;
    double weakestObjective =
        -std::numeric_limits<double>::infinity();
    for (const ScheduledRotationCandidate& candidate : candidates) {
        if (!candidate.active || candidate.carriesGlobalIncumbent) {
            continue;
        }
        if (!std::isfinite(candidate.bestObjective)) {
            throw std::invalid_argument(
                "scheduled rotation received a non-finite objective");
        }
        if (!selected.has_value() ||
            candidate.bestObjective >
                weakestObjective + objectiveTolerance ||
            (std::fabs(candidate.bestObjective - weakestObjective) <=
                 objectiveTolerance &&
             candidate.populationIndex < *selected)) {
            selected = candidate.populationIndex;
            weakestObjective = candidate.bestObjective;
        }
    }
    return selected;
}

const char* crossConfigurationFallbackReasonName(
        const CrossConfigurationFallbackReason reason) noexcept {
    switch (reason) {
        case CrossConfigurationFallbackReason::None:
            return "none";
        case CrossConfigurationFallbackReason::OnlyOneDistinctConfiguration:
            return "only_one_distinct_configuration";
        case CrossConfigurationFallbackReason::CandidatePopulationsTooSmall:
            return "candidate_populations_too_small";
        case CrossConfigurationFallbackReason::AttemptsExhausted:
            return "bounded_attempts_exhausted";
    }
    return "unknown";
}

CrossConfigurationSelectionDecision evaluateCrossConfigurationPair(
        const StaticParentConfigurationPair& pair,
        const std::size_t distinctAvailableConfigurationCount,
        const std::size_t eligiblePopulationCount,
        const std::size_t attemptsUsed,
        const std::size_t maximumAttempts) {
    if (maximumAttempts == 0U || attemptsUsed == 0U ||
        attemptsUsed > maximumAttempts) {
        throw std::invalid_argument(
            "cross-configuration attempts must be in [1, maximum]");
    }
    const DepotConfigurationKey first =
        makeDepotConfigurationKey(
            pair.firstActualConfiguration.openDepots);
    const DepotConfigurationKey second =
        makeDepotConfigurationKey(
            pair.secondActualConfiguration.openDepots);
    if (first.openDepots.empty() || second.openDepots.empty()) {
        throw std::invalid_argument(
            "parent actual configuration opens no depot");
    }

    CrossConfigurationSelectionDecision decision;
    decision.attemptsUsed = attemptsUsed;
    if (first != second) {
        decision.action = CrossConfigurationSelectionAction::Accept;
        return decision;
    }
    if (distinctAvailableConfigurationCount <= 1U) {
        decision.action =
            CrossConfigurationSelectionAction::SameConfigurationFallback;
        decision.fallbackReason =
            CrossConfigurationFallbackReason::OnlyOneDistinctConfiguration;
        return decision;
    }
    if (eligiblePopulationCount < 2U) {
        decision.action =
            CrossConfigurationSelectionAction::SameConfigurationFallback;
        decision.fallbackReason =
            CrossConfigurationFallbackReason::CandidatePopulationsTooSmall;
        return decision;
    }
    if (attemptsUsed == maximumAttempts) {
        decision.action =
            CrossConfigurationSelectionAction::SameConfigurationFallback;
        decision.fallbackReason =
            CrossConfigurationFallbackReason::AttemptsExhausted;
        return decision;
    }
    decision.action = CrossConfigurationSelectionAction::Retry;
    return decision;
}

}  // namespace hgasp
