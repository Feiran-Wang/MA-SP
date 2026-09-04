#ifndef HGASP_CORE_POPULATION_REDUCTION_POLICY_H_
#define HGASP_CORE_POPULATION_REDUCTION_POLICY_H_

#include "StaticConfigurationRegistry.h"

#include <cstddef>
#include <optional>
#include <vector>

namespace hgasp {

struct PopulationReductionWeights {
    double objectiveQuality = 0.55;
    double hgaImprovementPerCpu = 0.20;
    double diversity = 0.15;
    double globalSpContribution = 0.10;
};

struct PopulationReductionCandidate {
    std::size_t populationIndex = 0U;
    bool active = true;
    bool unrestricted = false;
    bool protectedPopulation = false;
    std::optional<StaticPopulationToken> token;
    DepotConfigurationKey configuration;
    std::optional<double> bestObjective;
    std::optional<double> hgaImprovementPerCpu;
    std::optional<double> diversity;
    std::optional<double> globalSpSelectedColumnContribution;
};

struct PopulationReductionScore {
    std::size_t populationIndex = 0U;
    double objectiveQuality = 0.0;
    double hgaImprovementPerCpu = 0.0;
    double diversity = 0.0;
    double globalSpContribution = 0.0;
    double weightedScore = 0.0;
};

struct PopulationReductionDecision {
    std::size_t fixedPopulationCountBefore = 0U;
    std::size_t requestedFixedSurvivorCount = 0U;
    std::size_t actualFixedSurvivorCount = 0U;
    std::size_t unrestrictedPopulationIndex = 0U;
    std::vector<std::size_t> survivingPopulationIndices;
    std::vector<std::size_t> removedPopulationIndices;
    std::vector<PopulationReductionScore> scores;
};

class PopulationReductionPolicy {
public:
    explicit PopulationReductionPolicy(
        PopulationReductionWeights weights = {});

    const PopulationReductionWeights& weights() const noexcept {
        return weights_;
    }

    static std::size_t halfSurvivorCount(
        std::size_t currentFixedPopulationCount) noexcept;

    // Ranks the active fixed populations with the same deterministic policy
    // as reduceHalf(), but retains the explicitly requested number.  The
    // request is clamped to the number of active fixed populations so a
    // profile can express the HGAMP 15 -> 8 -> 5 schedule while smaller
    // portfolios remain unchanged at an early phase.
    PopulationReductionDecision reduceToTarget(
        const std::vector<PopulationReductionCandidate>& candidates,
        std::size_t requestedFixedSurvivorCount) const;

    // Exactly one active unrestricted population is required and is always
    // retained. Only active fixed populations are scored and reduced.
    PopulationReductionDecision reduceHalf(
        const std::vector<PopulationReductionCandidate>& candidates) const;

private:
    PopulationReductionWeights weights_;
};

}  // namespace hgasp

#endif  // HGASP_CORE_POPULATION_REDUCTION_POLICY_H_
