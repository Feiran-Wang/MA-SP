#ifndef HGASP_CORE_DYNAMIC_CHALLENGER_POLICY_H_
#define HGASP_CORE_DYNAMIC_CHALLENGER_POLICY_H_

#include "ConfigurationCandidateArchive.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <vector>

namespace hgasp {

enum class ChallengerState {
    Empty,
    Filling,
    ProtectedTrial,
    ReadyForEvaluation,
    WaitingForReplaceableResident,
};

struct ChallengerToken {
    std::uint64_t generation = 0U;

    bool operator==(const ChallengerToken& other) const noexcept {
        return generation == other.generation;
    }
};

struct ChallengerRuntime {
    ChallengerState state = ChallengerState::Empty;
    ChallengerToken token;
    DepotConfigurationKey configuration;
    std::uint64_t selectedAttempt = 0U;
    std::uint64_t trialStartAttempt = 0U;
    std::uint64_t trialEndAttempt = 0U;
    std::uint64_t primarySelections = 0U;
    std::uint64_t offspringProduced = 0U;
    std::uint64_t matchingConfigurationOffspring = 0U;
    ConfigurationCandidateEligibilityReason selectionReason =
        ConfigurationCandidateEligibilityReason::
            InsufficientObservations;
    std::uint64_t observationsBeforeTrial = 0U;
    std::size_t selectionTargetPopulationIndex = 0U;
    bool hadSelectionTarget = false;
    std::size_t fillSeedCount = 0U;
    std::size_t fillConstructionAttempts = 0U;
    std::size_t fillPopulationSize = 0U;
    double founderObjective = std::numeric_limits<double>::infinity();
    double top3MeanBeforeTrial = std::numeric_limits<double>::infinity();
    double selectionTargetBest = std::numeric_limits<double>::infinity();
    double selectionTargetEliteMean =
        std::numeric_limits<double>::infinity();
    double fillBestObjective = std::numeric_limits<double>::infinity();
    double bestObjective = std::numeric_limits<double>::infinity();
    double fillCpuSeconds = 0.0;
    double trialHgaCpuSeconds = 0.0;

    bool active() const noexcept {
        return state != ChallengerState::Empty;
    }
};

enum class ChallengerProblemProfile {
    Clrp,
    TimeWindows,
};

struct DynamicChallengerParameters {
    std::uint64_t staticMinimumAgeAttempts = 5000U;
    std::uint64_t selectionIntervalAttempts = 500U;
    std::uint64_t clrpTrialAttempts = 2000U;
    std::uint64_t twTrialAttempts = 1000U;
    std::uint64_t protectedPrimaryPeriod = 10U;
    double clrpLatestLaunchFraction = 0.9;
    std::size_t eliteMeanCount = 10U;
    double objectiveTolerance = 1.0e-6;
    std::size_t targetMinimumSize = 6U;
    std::size_t targetMaximumSize = 10U;
    std::size_t targetPopulationDivisor = 3U;
    std::size_t maximumConstructionMultiplier = 3U;
    std::size_t minimumViableNumerator = 3U;
    std::size_t minimumViableDenominator = 5U;
};

struct ChallengerLaunchContext {
    ChallengerProblemProfile profile = ChallengerProblemProfile::Clrp;
    std::uint64_t attempt = 0U;
    std::uint64_t phaseHorizonAttempts = 0U;
    bool reduction2Completed = false;
    bool slotEmpty = true;
};

struct ChallengerResidentCandidate {
    std::size_t populationIndex = 0U;
    bool active = false;
    bool unrestricted = false;
    bool challenger = false;
    bool protectedResident = false;
    bool minimumAgeSatisfied = false;
    std::vector<double> objectives;
};

struct ChallengerPromotionDecision {
    bool hasReplaceableResident = false;
    bool promote = false;
    std::size_t targetPopulationIndex = 0U;
    double challengerBest = std::numeric_limits<double>::infinity();
    double challengerEliteMean = std::numeric_limits<double>::infinity();
    double residentBest = std::numeric_limits<double>::infinity();
    double residentEliteMean = std::numeric_limits<double>::infinity();
};

class DynamicChallengerPolicy {
public:
    explicit DynamicChallengerPolicy(
        DynamicChallengerParameters parameters = {});

    const DynamicChallengerParameters& parameters() const noexcept {
        return parameters_;
    }

    bool shouldAttemptSelection(
        const ChallengerLaunchContext& context) const noexcept;

    std::size_t targetPopulationSize(
        std::size_t populationMinimum) const noexcept;

    std::size_t maximumConstructionAttempts(
        std::size_t populationMinimum) const noexcept;

    std::size_t minimumViablePopulationSize(
        std::size_t populationMinimum) const noexcept;

    std::uint64_t trialAttempts(
        ChallengerProblemProfile profile) const noexcept;

    bool isProtectedPrimaryTurn(
        std::uint64_t matingOpportunity) const noexcept;

    const ConfigurationCandidateRecord* selectTwoWayTournament(
        const std::vector<const ConfigurationCandidateRecord*>& eligible,
        std::size_t firstIndex,
        std::size_t secondIndex) const;

    std::pair<std::size_t, std::size_t> drawDistinctPair(
        std::size_t candidateCount,
        const std::function<std::size_t(std::size_t)>& boundedDraw) const;

    std::optional<ResidentQualitySnapshot> weakestReplaceableResident(
        const std::vector<ChallengerResidentCandidate>& residents) const;

    double eliteMean(
        const std::vector<double>& objectives) const;

    ChallengerPromotionDecision evaluatePromotion(
        const std::vector<double>& challengerObjectives,
        const std::vector<ChallengerResidentCandidate>& residents) const;

private:
    bool candidateWins(
        const ConfigurationCandidateRecord& left,
        const ConfigurationCandidateRecord& right) const noexcept;

    DynamicChallengerParameters parameters_;
};

}  // namespace hgasp

#endif  // HGASP_CORE_DYNAMIC_CHALLENGER_POLICY_H_
