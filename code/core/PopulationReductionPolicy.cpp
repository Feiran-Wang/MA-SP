#include "PopulationReductionPolicy.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>
#include <utility>

namespace hgasp {
namespace {

constexpr double kTolerance = 1.0e-12;

double optionalNonNegative(
        const std::optional<double>& value,
        const char* field) {
    if (!value.has_value()) {
        return 0.0;
    }
    if (!std::isfinite(*value) || *value < 0.0) {
        throw std::invalid_argument(field);
    }
    return *value;
}

std::vector<double> normalizeHigherIsBetter(
        const std::vector<double>& raw) {
    if (raw.empty()) {
        return {};
    }
    const auto bounds = std::minmax_element(raw.begin(), raw.end());
    const double minimum = *bounds.first;
    const double maximum = *bounds.second;
    std::vector<double> normalized(raw.size(), 0.0);
    if (maximum - minimum <= kTolerance) {
        if (maximum > kTolerance) {
            std::fill(normalized.begin(), normalized.end(), 1.0);
        }
        return normalized;
    }
    for (std::size_t index = 0U; index < raw.size(); ++index) {
        normalized[index] =
            (raw[index] - minimum) / (maximum - minimum);
    }
    return normalized;
}

std::vector<double> normalizeObjectiveQuality(
        const std::vector<std::optional<double>>& objectives) {
    std::optional<double> minimum;
    std::optional<double> maximum;
    for (const std::optional<double>& objective : objectives) {
        if (!objective.has_value()) {
            continue;
        }
        if (!std::isfinite(*objective)) {
            throw std::invalid_argument(
                "population reduction objective must be finite");
        }
        minimum = minimum.has_value()
            ? std::min(*minimum, *objective)
            : objective;
        maximum = maximum.has_value()
            ? std::max(*maximum, *objective)
            : objective;
    }
    std::vector<double> normalized(objectives.size(), 0.0);
    if (!minimum.has_value()) {
        return normalized;
    }
    if (*maximum - *minimum <= kTolerance) {
        for (std::size_t index = 0U;
             index < objectives.size();
             ++index) {
            if (objectives[index].has_value()) {
                normalized[index] = 1.0;
            }
        }
        return normalized;
    }
    for (std::size_t index = 0U;
         index < objectives.size();
         ++index) {
        if (objectives[index].has_value()) {
            normalized[index] =
                (*maximum - *objectives[index]) /
                (*maximum - *minimum);
        }
    }
    return normalized;
}

}  // namespace

PopulationReductionPolicy::PopulationReductionPolicy(
        PopulationReductionWeights weights)
    : weights_(weights) {
    if (!std::isfinite(weights_.objectiveQuality) ||
        !std::isfinite(weights_.hgaImprovementPerCpu) ||
        !std::isfinite(weights_.diversity) ||
        !std::isfinite(weights_.globalSpContribution) ||
        weights_.objectiveQuality < 0.0 ||
        weights_.hgaImprovementPerCpu < 0.0 ||
        weights_.diversity < 0.0 ||
        weights_.globalSpContribution < 0.0) {
        throw std::invalid_argument(
            "population reduction weights must be finite and non-negative");
    }
    const double sum = weights_.objectiveQuality +
        weights_.hgaImprovementPerCpu +
        weights_.diversity +
        weights_.globalSpContribution;
    if (sum <= 0.0) {
        throw std::invalid_argument(
            "population reduction weights must have positive total");
    }
    weights_.objectiveQuality /= sum;
    weights_.hgaImprovementPerCpu /= sum;
    weights_.diversity /= sum;
    weights_.globalSpContribution /= sum;
}

std::size_t PopulationReductionPolicy::halfSurvivorCount(
        const std::size_t currentFixedPopulationCount) noexcept {
    return (currentFixedPopulationCount + 1U) / 2U;
}

PopulationReductionDecision PopulationReductionPolicy::reduceHalf(
        const std::vector<PopulationReductionCandidate>& candidates) const {
    std::size_t activeFixedCount = 0U;
    for (const PopulationReductionCandidate& candidate : candidates) {
        if (candidate.active && !candidate.unrestricted) {
            ++activeFixedCount;
        }
    }
    return reduceToTarget(
        candidates, halfSurvivorCount(activeFixedCount));
}

PopulationReductionDecision PopulationReductionPolicy::reduceToTarget(
        const std::vector<PopulationReductionCandidate>& candidates,
        const std::size_t requestedFixedSurvivorCount) const {
    if (requestedFixedSurvivorCount == 0U) {
        throw std::invalid_argument(
            "population reduction survivor target must be positive");
    }
    std::set<std::size_t> populationIndices;
    std::set<DepotConfigurationKey> configurations;
    const PopulationReductionCandidate* unrestricted = nullptr;
    std::vector<const PopulationReductionCandidate*> fixed;
    fixed.reserve(candidates.size());

    for (const PopulationReductionCandidate& candidate : candidates) {
        if (!populationIndices.insert(candidate.populationIndex).second) {
            throw std::invalid_argument(
                "population reduction received a duplicate population "
                "index");
        }
        if (candidate.unrestricted) {
            if (unrestricted != nullptr || !candidate.active ||
                candidate.token.has_value() ||
                !candidate.configuration.openDepots.empty()) {
                throw std::invalid_argument(
                    "population reduction requires one active, token-free "
                    "unrestricted identity");
            }
            unrestricted = &candidate;
            continue;
        }
        if (!candidate.active) {
            continue;
        }
        if (!candidate.token.has_value()) {
            throw std::invalid_argument(
                "active fixed reduction candidate has no token");
        }
        if (candidate.token->slot != candidate.populationIndex) {
            throw std::invalid_argument(
                "fixed reduction token does not match its population slot");
        }
        const DepotConfigurationKey canonical =
            makeDepotConfigurationKey(candidate.configuration.openDepots);
        if (canonical.openDepots.empty() ||
            canonical != candidate.configuration ||
            !configurations.insert(canonical).second) {
            throw std::invalid_argument(
                "active fixed reduction configuration is invalid or "
                "duplicated");
        }
        fixed.push_back(&candidate);
    }
    if (unrestricted == nullptr) {
        throw std::invalid_argument(
            "population reduction cannot remove or omit unrestricted "
            "population");
    }

    std::sort(
        fixed.begin(),
        fixed.end(),
        [](const PopulationReductionCandidate* left,
           const PopulationReductionCandidate* right) {
            return left->populationIndex < right->populationIndex;
        });
    std::vector<std::optional<double>> objectives;
    std::vector<double> hgaImprovement;
    std::vector<double> diversity;
    std::vector<double> spContribution;
    objectives.reserve(fixed.size());
    hgaImprovement.reserve(fixed.size());
    diversity.reserve(fixed.size());
    spContribution.reserve(fixed.size());
    for (const PopulationReductionCandidate* candidate : fixed) {
        objectives.push_back(candidate->bestObjective);
        hgaImprovement.push_back(optionalNonNegative(
            candidate->hgaImprovementPerCpu,
            "population reduction HGA productivity is invalid"));
        diversity.push_back(optionalNonNegative(
            candidate->diversity,
            "population reduction diversity is invalid"));
        // Missing contribution is explicitly zero; it is not imputed.
        spContribution.push_back(optionalNonNegative(
            candidate->globalSpSelectedColumnContribution,
            "population reduction SP contribution is invalid"));
    }

    const std::vector<double> objectiveQuality =
        normalizeObjectiveQuality(objectives);
    const std::vector<double> normalizedHga =
        normalizeHigherIsBetter(hgaImprovement);
    const std::vector<double> normalizedDiversity =
        normalizeHigherIsBetter(diversity);
    const std::vector<double> normalizedSp =
        normalizeHigherIsBetter(spContribution);

    PopulationReductionDecision decision;
    decision.fixedPopulationCountBefore = fixed.size();
    decision.requestedFixedSurvivorCount =
        std::min(requestedFixedSurvivorCount, fixed.size());
    decision.unrestrictedPopulationIndex =
        unrestricted->populationIndex;
    decision.scores.reserve(fixed.size());
    std::map<std::size_t, double> scoreByPopulation;
    for (std::size_t index = 0U; index < fixed.size(); ++index) {
        PopulationReductionScore score;
        score.populationIndex = fixed[index]->populationIndex;
        score.objectiveQuality = objectiveQuality[index];
        score.hgaImprovementPerCpu = normalizedHga[index];
        score.diversity = normalizedDiversity[index];
        score.globalSpContribution = normalizedSp[index];
        score.weightedScore =
            weights_.objectiveQuality * score.objectiveQuality +
            weights_.hgaImprovementPerCpu *
                score.hgaImprovementPerCpu +
            weights_.diversity * score.diversity +
            weights_.globalSpContribution *
                score.globalSpContribution;
        scoreByPopulation.emplace(
            score.populationIndex, score.weightedScore);
        decision.scores.push_back(score);
    }

    std::vector<const PopulationReductionCandidate*> ranked = fixed;
    std::sort(
        ranked.begin(),
        ranked.end(),
        [&scoreByPopulation](
                const PopulationReductionCandidate* left,
                const PopulationReductionCandidate* right) {
            if (left->protectedPopulation !=
                right->protectedPopulation) {
                return left->protectedPopulation;
            }
            const double leftScore =
                scoreByPopulation.at(left->populationIndex);
            const double rightScore =
                scoreByPopulation.at(right->populationIndex);
            if (leftScore != rightScore) {
                return leftScore > rightScore;
            }
            if (left->configuration != right->configuration) {
                return left->configuration < right->configuration;
            }
            return left->populationIndex < right->populationIndex;
        });

    std::set<std::size_t> survivors;
    for (const PopulationReductionCandidate* candidate : ranked) {
        if (candidate->protectedPopulation) {
            survivors.insert(candidate->populationIndex);
        }
    }
    for (const PopulationReductionCandidate* candidate : ranked) {
        if (survivors.size() >=
            decision.requestedFixedSurvivorCount) {
            break;
        }
        survivors.insert(candidate->populationIndex);
    }
    decision.actualFixedSurvivorCount = survivors.size();
    decision.survivingPopulationIndices.assign(
        survivors.begin(), survivors.end());
    decision.survivingPopulationIndices.push_back(
        unrestricted->populationIndex);
    std::sort(
        decision.survivingPopulationIndices.begin(),
        decision.survivingPopulationIndices.end());
    for (const PopulationReductionCandidate* candidate : fixed) {
        if (survivors.find(candidate->populationIndex) ==
            survivors.end()) {
            decision.removedPopulationIndices.push_back(
                candidate->populationIndex);
        }
    }
    return decision;
}

}  // namespace hgasp
