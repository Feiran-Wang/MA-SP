#ifndef HGASP_CORE_ADAPTIVE_SEARCH_ALLOCATOR_H_
#define HGASP_CORE_ADAPTIVE_SEARCH_ALLOCATOR_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

namespace hgasp {

struct AdaptiveSearchPolicy {
    double exploration = 0.25;
    double diversityWeight = 0.10;
    double columnWeight = 0.05;
    double emaWeight = 0.20;
    int mutationMinimumPercent = 5;
    int mutationMaximumPercent = 40;
};

struct SearchArmObservation {
    double objectiveImprovement = 0.0;
    double cpuSeconds = 0.0;
    double newColumns = 0.0;
};

class AdaptiveSearchAllocator {
public:
    AdaptiveSearchAllocator() = default;
    explicit AdaptiveSearchAllocator(AdaptiveSearchPolicy policy) {
        configure(policy);
    }

    void configure(AdaptiveSearchPolicy policy) {
        if (policy.exploration < 0.0 ||
            policy.diversityWeight < 0.0 ||
            policy.columnWeight < 0.0 ||
            policy.emaWeight <= 0.0 ||
            policy.emaWeight > 1.0 ||
            policy.mutationMinimumPercent < 0 ||
            policy.mutationMaximumPercent >
                100 ||
            policy.mutationMinimumPercent >
                policy.mutationMaximumPercent) {
            throw std::invalid_argument(
                "invalid adaptive search-allocation policy");
        }
        policy_ = policy;
    }

    int selectPopulation(
            const std::vector<int>& eligible,
            const std::vector<double>& diversity) {
        if (eligible.empty()) {
            return -1;
        }
        int maximumIndex =
            *std::max_element(eligible.begin(), eligible.end());
        if (maximumIndex < 0) {
            throw std::invalid_argument(
                "eligible population indices must be non-negative");
        }
        ensureSize(static_cast<std::size_t>(maximumIndex + 1));
        if (!diversity.empty() &&
            diversity.size() < arms_.size()) {
            throw std::invalid_argument(
                "population diversity vector is too small");
        }

        int selected = -1;
        double selectedScore =
            -std::numeric_limits<double>::infinity();
        const double logVisits =
            std::log(static_cast<double>(totalReports_) + 2.0);
        for (const int index : eligible) {
            ArmState& arm = arms_[static_cast<std::size_t>(index)];
            if (arm.reports == 0U) {
                return index;
            }
            const double exploration =
                policy_.exploration *
                std::sqrt(
                    logVisits /
                    static_cast<double>(arm.reports));
            const double diversityValue =
                diversity.empty()
                    ? 0.0
                    : diversity[
                          static_cast<std::size_t>(index)];
            const double score =
                arm.improvementPerCpuEma +
                exploration +
                policy_.diversityWeight * diversityValue +
                policy_.columnWeight *
                    arm.columnsPerCpuEma;
            if (selected < 0 ||
                score > selectedScore + 1.0e-12 ||
                (std::fabs(score - selectedScore) <=
                     1.0e-12 &&
                 index < selected)) {
                selected = index;
                selectedScore = score;
            }
        }
        return selected;
    }

    void reportPopulation(
            const int index,
            const SearchArmObservation& observation) {
        if (index < 0 ||
            !std::isfinite(observation.objectiveImprovement) ||
            !std::isfinite(observation.cpuSeconds) ||
            !std::isfinite(observation.newColumns) ||
            observation.objectiveImprovement < 0.0 ||
            observation.cpuSeconds < 0.0 ||
            observation.newColumns < 0.0) {
            throw std::invalid_argument(
                "invalid adaptive population observation");
        }
        ensureSize(static_cast<std::size_t>(index + 1));
        ArmState& arm = arms_[static_cast<std::size_t>(index)];
        const double denominator =
            std::max(observation.cpuSeconds, 1.0e-9);
        updateEma(
            arm.improvementPerCpuEma,
            observation.objectiveImprovement / denominator,
            arm.reports == 0U);
        updateEma(
            arm.columnsPerCpuEma,
            observation.newColumns / denominator,
            arm.reports == 0U);
        ++arm.reports;
        ++totalReports_;
    }

    void reportSharedPopulations(
            const int first,
            const int second,
            const SearchArmObservation& observation) {
        if (first == second) {
            reportPopulation(first, observation);
            return;
        }
        SearchArmObservation shared = observation;
        shared.objectiveImprovement *= 0.5;
        shared.newColumns *= 0.5;
        // Both parents consumed the complete crossover CPU interval. Keeping
        // that denominator for each arm gives each exactly half of the joint
        // improvement/column credit without double-counting productivity.
        reportPopulation(first, shared);
        reportPopulation(second, shared);
    }

    void resetPopulation(const int index) {
        if (index < 0) {
            throw std::invalid_argument(
                "population index must be non-negative");
        }
        ensureSize(static_cast<std::size_t>(index + 1));
        arms_[static_cast<std::size_t>(index)] = ArmState{};
    }

    void resetAllPopulations() noexcept {
        for (ArmState& arm : arms_) {
            arm = ArmState{};
        }
    }

    std::size_t populationReports(const int index) const noexcept {
        if (index < 0 ||
            static_cast<std::size_t>(index) >= arms_.size()) {
            return 0U;
        }
        return arms_[static_cast<std::size_t>(index)].reports;
    }

    void reportMutation(
            const bool committed,
            const double objectiveImprovement,
            const double cpuSeconds) {
        if (!std::isfinite(objectiveImprovement) ||
            !std::isfinite(cpuSeconds) ||
            objectiveImprovement < 0.0 ||
            cpuSeconds < 0.0) {
            throw std::invalid_argument(
                "invalid adaptive mutation observation");
        }
        const bool first = mutationReports_ == 0U;
        updateEma(
            mutationCommitEma_,
            committed ? 1.0 : 0.0,
            first);
        updateEma(
            mutationImprovementPerCpuEma_,
            objectiveImprovement /
                std::max(cpuSeconds, 1.0e-9),
            first);
        ++mutationReports_;
    }

    int mutationProbability(
            const int baselinePercent,
            const double diversity,
            const double stagnationFraction,
            const bool spJustImproved) const {
        double probability =
            static_cast<double>(baselinePercent);
        probability +=
            12.0 *
            std::clamp(stagnationFraction, 0.0, 1.0);
        probability +=
            10.0 *
            std::clamp(0.20 - diversity, 0.0, 0.20) /
            0.20;
        if (mutationReports_ > 0U) {
            probability += 8.0 *
                (mutationCommitEma_ - 0.5);
            if (mutationImprovementPerCpuEma_ > 0.0) {
                probability += 3.0;
            }
        }
        if (spJustImproved) {
            probability -= 8.0;
        }
        return static_cast<int>(std::lround(std::clamp(
            probability,
            static_cast<double>(
                policy_.mutationMinimumPercent),
            static_cast<double>(
                policy_.mutationMaximumPercent))));
    }

    double populationImprovementEma(const int index) const {
        if (index < 0 ||
            static_cast<std::size_t>(index) >= arms_.size()) {
            return 0.0;
        }
        return arms_[static_cast<std::size_t>(index)]
            .improvementPerCpuEma;
    }

private:
    struct ArmState {
        std::size_t reports = 0U;
        double improvementPerCpuEma = 0.0;
        double columnsPerCpuEma = 0.0;
    };

    void ensureSize(const std::size_t size) {
        if (arms_.size() < size) {
            arms_.resize(size);
        }
    }

    void updateEma(
            double& target,
            const double value,
            const bool first) {
        target = first
            ? value
            : policy_.emaWeight * value +
                  (1.0 - policy_.emaWeight) * target;
    }

    AdaptiveSearchPolicy policy_;
    std::vector<ArmState> arms_;
    std::size_t totalReports_ = 0U;
    std::size_t mutationReports_ = 0U;
    double mutationCommitEma_ = 0.0;
    double mutationImprovementPerCpuEma_ = 0.0;
};

}  // namespace hgasp

#endif  // HGASP_CORE_ADAPTIVE_SEARCH_ALLOCATOR_H_
