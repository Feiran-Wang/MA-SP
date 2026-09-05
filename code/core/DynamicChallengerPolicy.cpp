#include "DynamicChallengerPolicy.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace hgasp {

DynamicChallengerPolicy::DynamicChallengerPolicy(
        DynamicChallengerParameters parameters)
    : parameters_(std::move(parameters)) {
    if (parameters_.selectionIntervalAttempts == 0U ||
        parameters_.clrpTrialAttempts == 0U ||
        parameters_.twTrialAttempts == 0U ||
        parameters_.protectedPrimaryPeriod == 0U ||
        parameters_.eliteMeanCount == 0U ||
        parameters_.targetMinimumSize == 0U ||
        parameters_.targetMaximumSize < parameters_.targetMinimumSize ||
        parameters_.targetPopulationDivisor == 0U ||
        parameters_.maximumConstructionMultiplier == 0U ||
        parameters_.minimumViableNumerator == 0U ||
        parameters_.minimumViableDenominator == 0U ||
        parameters_.minimumViableNumerator >
            parameters_.minimumViableDenominator ||
        !std::isfinite(parameters_.clrpLatestLaunchFraction) ||
        parameters_.clrpLatestLaunchFraction <= 0.0 ||
        parameters_.clrpLatestLaunchFraction > 1.0 ||
        !std::isfinite(parameters_.objectiveTolerance) ||
        parameters_.objectiveTolerance < 0.0) {
        throw std::invalid_argument(
            "dynamic challenger parameters are invalid");
    }
}

bool DynamicChallengerPolicy::shouldAttemptSelection(
        const ChallengerLaunchContext& context) const noexcept {
    if (!context.slotEmpty ||
        context.attempt < parameters_.staticMinimumAgeAttempts ||
        context.attempt % parameters_.selectionIntervalAttempts != 0U) {
        return false;
    }
    if (context.profile == ChallengerProblemProfile::TimeWindows) {
        return !context.reduction2Completed;
    }
    if (context.phaseHorizonAttempts == 0U) {
        return false;
    }
    const double completedFraction =
        static_cast<double>(context.attempt) /
        static_cast<double>(context.phaseHorizonAttempts);
    return completedFraction < parameters_.clrpLatestLaunchFraction;
}

std::size_t DynamicChallengerPolicy::targetPopulationSize(
        const std::size_t populationMinimum) const noexcept {
    return std::max<std::size_t>(
        parameters_.targetMinimumSize,
        std::min<std::size_t>(
            parameters_.targetMaximumSize,
            populationMinimum / parameters_.targetPopulationDivisor));
}

std::size_t DynamicChallengerPolicy::maximumConstructionAttempts(
        const std::size_t populationMinimum) const noexcept {
    return parameters_.maximumConstructionMultiplier *
        targetPopulationSize(populationMinimum);
}

std::size_t DynamicChallengerPolicy::minimumViablePopulationSize(
        const std::size_t populationMinimum) const noexcept {
    const std::size_t target = targetPopulationSize(populationMinimum);
    return (
        parameters_.minimumViableNumerator * target +
        parameters_.minimumViableDenominator - 1U) /
        parameters_.minimumViableDenominator;
}

std::uint64_t DynamicChallengerPolicy::trialAttempts(
        const ChallengerProblemProfile profile) const noexcept {
    return profile == ChallengerProblemProfile::TimeWindows
        ? parameters_.twTrialAttempts
        : parameters_.clrpTrialAttempts;
}

bool DynamicChallengerPolicy::isProtectedPrimaryTurn(
        const std::uint64_t matingOpportunity) const noexcept {
    return matingOpportunity != 0U &&
        matingOpportunity % parameters_.protectedPrimaryPeriod == 0U;
}

bool DynamicChallengerPolicy::candidateWins(
        const ConfigurationCandidateRecord& left,
        const ConfigurationCandidateRecord& right) const noexcept {
    if (left.top3Mean() != right.top3Mean()) {
        return left.top3Mean() < right.top3Mean();
    }
    if (left.bestObjective != right.bestObjective) {
        return left.bestObjective < right.bestObjective;
    }
    if (left.totalObservationCount != right.totalObservationCount) {
        return left.totalObservationCount > right.totalObservationCount;
    }
    return left.configuration < right.configuration;
}

const ConfigurationCandidateRecord*
DynamicChallengerPolicy::selectTwoWayTournament(
        const std::vector<const ConfigurationCandidateRecord*>& eligible,
        const std::size_t firstIndex,
        const std::size_t secondIndex) const {
    if (eligible.empty()) {
        return nullptr;
    }
    for (const ConfigurationCandidateRecord* record : eligible) {
        if (record == nullptr) {
            throw std::invalid_argument(
                "challenger tournament contains a null record");
        }
    }
    if (eligible.size() == 1U) {
        return eligible.front();
    }
    if (firstIndex >= eligible.size() ||
        secondIndex >= eligible.size() || firstIndex == secondIndex) {
        throw std::invalid_argument(
            "challenger tournament requires two distinct valid indices");
    }
    return candidateWins(*eligible[firstIndex], *eligible[secondIndex])
        ? eligible[firstIndex]
        : eligible[secondIndex];
}

std::pair<std::size_t, std::size_t>
DynamicChallengerPolicy::drawDistinctPair(
        const std::size_t candidateCount,
        const std::function<std::size_t(std::size_t)>& boundedDraw) const {
    if (candidateCount < 2U || !boundedDraw) {
        throw std::invalid_argument(
            "distinct challenger draw requires at least two candidates");
    }
    const std::size_t first = boundedDraw(candidateCount);
    const std::size_t compactSecond = boundedDraw(candidateCount - 1U);
    if (first >= candidateCount ||
        compactSecond >= candidateCount - 1U) {
        throw std::out_of_range(
            "challenger RNG returned a value outside its requested bound");
    }
    const std::size_t second = compactSecond >= first
        ? compactSecond + 1U
        : compactSecond;
    return {first, second};
}

std::optional<ResidentQualitySnapshot>
DynamicChallengerPolicy::weakestReplaceableResident(
        const std::vector<ChallengerResidentCandidate>& residents) const {
    std::optional<ResidentQualitySnapshot> weakest;
    for (const ChallengerResidentCandidate& resident : residents) {
        if (!resident.active || resident.unrestricted ||
            resident.challenger || resident.protectedResident ||
            !resident.minimumAgeSatisfied || resident.objectives.empty()) {
            continue;
        }
        const double mean = eliteMean(resident.objectives);
        if (!weakest.has_value() || mean > weakest->eliteMean10 ||
            (mean == weakest->eliteMean10 &&
             resident.populationIndex < weakest->populationIndex)) {
            ResidentQualitySnapshot snapshot;
            snapshot.populationIndex = resident.populationIndex;
            snapshot.bestObjective = *std::min_element(
                resident.objectives.begin(), resident.objectives.end());
            snapshot.eliteMean10 = mean;
            weakest = snapshot;
        }
    }
    return weakest;
}

double DynamicChallengerPolicy::eliteMean(
        const std::vector<double>& objectives) const {
    if (objectives.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    std::vector<double> sorted = objectives;
    if (std::any_of(
            sorted.begin(), sorted.end(),
            [](const double objective) {
                return !std::isfinite(objective);
            })) {
        throw std::invalid_argument(
            "population objectives must be finite");
    }
    std::sort(sorted.begin(), sorted.end());
    const std::size_t count =
        std::min(parameters_.eliteMeanCount, sorted.size());
    return std::accumulate(sorted.begin(), sorted.begin() + count, 0.0) /
        static_cast<double>(count);
}

ChallengerPromotionDecision
DynamicChallengerPolicy::evaluatePromotion(
        const std::vector<double>& challengerObjectives,
        const std::vector<ChallengerResidentCandidate>& residents) const {
    ChallengerPromotionDecision decision;
    if (challengerObjectives.empty()) {
        return decision;
    }
    decision.challengerBest = *std::min_element(
        challengerObjectives.begin(), challengerObjectives.end());
    decision.challengerEliteMean = eliteMean(challengerObjectives);

    const auto worst = weakestReplaceableResident(residents);
    if (!worst.has_value()) {
        return decision;
    }
    decision.hasReplaceableResident = true;
    decision.targetPopulationIndex = worst->populationIndex;
    decision.residentEliteMean = worst->eliteMean10;
    decision.residentBest = worst->bestObjective;
    decision.promote =
        decision.challengerEliteMean + parameters_.objectiveTolerance <
            decision.residentEliteMean &&
        decision.challengerBest <=
            decision.residentBest + parameters_.objectiveTolerance;
    return decision;
}

}  // namespace hgasp
