#ifndef HGASP_CORE_ADAPTIVE_SEARCH_ALLOCATOR_H_
#define HGASP_CORE_ADAPTIVE_SEARCH_ALLOCATOR_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

namespace hgasp {

// Population-level responsibility is intentionally separate from operator
// adaptation. In legacy/static-population search the allocator owns both the
// primary and mate decisions. Dynamic TW islands own the primary-island UCB,
// so their allocator should be configured as MateOnly and retain only the
// second-parent share of a joint HGA outcome. Mutation/operator feedback is
// always owned by this allocator in both modes.
enum class PopulationAllocationResponsibility {
    LegacyPrimaryAndMate,
    MateOnly,
};

// Delayed route credit can be attributed explicitly when the caller retains
// parent-role provenance. Unspecified preserves the historical API contract.
enum class PopulationCreditRole {
    Unspecified,
    Primary,
    Mate,
};

struct AdaptiveSearchPolicy {
    double exploration = 0.25;
    double diversityWeight = 0.10;
    double columnWeight = 0.05;
    double emaWeight = 0.20;
    int mutationMinimumPercent = 5;
    int mutationMaximumPercent = 40;
    PopulationAllocationResponsibility populationResponsibility =
        PopulationAllocationResponsibility::LegacyPrimaryAndMate;
};

struct SearchArmObservation {
    double objectiveImprovement = 0.0;
    double cpuSeconds = 0.0;
    double newColumns = 0.0;
    double usefulColumnCredit = 0.0;
};

class AdaptiveSearchAllocator {
public:
    AdaptiveSearchAllocator() = default;
    explicit AdaptiveSearchAllocator(AdaptiveSearchPolicy policy) {
        configure(policy);
    }

    void configure(AdaptiveSearchPolicy policy) {
        switch (policy.populationResponsibility) {
            case PopulationAllocationResponsibility::
                LegacyPrimaryAndMate:
            case PopulationAllocationResponsibility::MateOnly:
                break;
            default:
                throw std::invalid_argument(
                    "invalid population-allocation responsibility");
        }
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

    PopulationAllocationResponsibility
    populationResponsibility() const noexcept {
        return policy_.populationResponsibility;
    }

    bool ownsPrimaryPopulationSelection() const noexcept {
        return policy_.populationResponsibility ==
            PopulationAllocationResponsibility::
                LegacyPrimaryAndMate;
    }

    bool ownsMatePopulationSelection() const noexcept {
        return true;
    }

    // Backward-compatible population selection. New call sites should use the
    // role-specific entry points below so ownership is evident in code.
    int selectPopulation(
            const std::vector<int>& eligible,
            const std::vector<double>& diversity) {
        return selectPopulationByScore(eligible, diversity);
    }

    int selectPrimaryPopulation(
            const std::vector<int>& eligible,
            const std::vector<double>& diversity) {
        if (!ownsPrimaryPopulationSelection()) {
            throw std::logic_error(
                "MateOnly allocator cannot select the primary population");
        }
        return selectPopulationByScore(eligible, diversity);
    }

    int selectMatePopulation(
            const std::vector<int>& eligible,
            const std::vector<double>& diversity) {
        return selectPopulationByScore(eligible, diversity);
    }

    void reportPopulation(
            const int index,
            const SearchArmObservation& observation) {
        validateObservation(index, observation);
        reportPopulationUnchecked(index, observation);
    }

    // Explicit ordered-parent feedback. The primary and mate shares are
    // accounted independently so population selection receives one bounded
    // observation per role.
    void reportMatingOutcome(
            const int primary,
            const int mate,
            const SearchArmObservation& observation) {
        validateObservation(primary, observation);
        if (mate < 0) {
            throw std::invalid_argument(
                "mate population index must be non-negative");
        }
        if (policy_.populationResponsibility ==
            PopulationAllocationResponsibility::MateOnly) {
            if (primary == mate) {
                return;
            }
            reportPopulationUnchecked(
                mate, sharedObservation(observation));
            return;
        }
        if (primary == mate) {
            reportPopulationUnchecked(primary, observation);
            return;
        }
        const SearchArmObservation shared =
            sharedObservation(observation);
        reportPopulationUnchecked(primary, shared);
        reportPopulationUnchecked(mate, shared);
    }

    // Historical name retained for source compatibility. Its parameters are
    // now explicitly interpreted as ordered primary and mate populations.
    void reportSharedPopulations(
            const int first,
            const int second,
            const SearchArmObservation& observation) {
        reportMatingOutcome(first, second, observation);
    }

    void resetPopulation(const int index) {
        if (index < 0) {
            throw std::invalid_argument(
                "population index must be non-negative");
        }
        ensureSize(static_cast<std::size_t>(index + 1));
        ArmState& arm = arms_[static_cast<std::size_t>(index)];
        totalReports_ =
            arm.reports > totalReports_
                ? 0U
                : totalReports_ - arm.reports;
        arm = ArmState{};
    }

    void resetAllPopulations() noexcept {
        for (ArmState& arm : arms_) {
            arm = ArmState{};
        }
        totalReports_ = 0U;
        improvementRateScale_ = 0.0;
        columnsRateScale_ = 0.0;
        usefulCreditRateScale_ = 0.0;
        hasImprovementRateScale_ = false;
        hasColumnsRateScale_ = false;
        hasUsefulCreditRateScale_ = false;
    }

    void reportUsefulColumnCredit(
            const int index,
            const double credit,
            const double originCpuSeconds,
            const PopulationCreditRole role =
                PopulationCreditRole::Unspecified) {
        switch (role) {
            case PopulationCreditRole::Unspecified:
            case PopulationCreditRole::Primary:
            case PopulationCreditRole::Mate:
                break;
            default:
                throw std::invalid_argument(
                    "invalid delayed-credit population role");
        }
        if (index < 0 ||
            !std::isfinite(credit) ||
            credit < 0.0 ||
            !std::isfinite(originCpuSeconds) ||
            originCpuSeconds < 0.0) {
            throw std::invalid_argument(
                "invalid delayed useful-column observation");
        }
        if (policy_.populationResponsibility ==
                PopulationAllocationResponsibility::MateOnly &&
            role == PopulationCreditRole::Primary) {
            return;
        }
        ensureSize(static_cast<std::size_t>(index + 1));
        ArmState& arm = arms_[static_cast<std::size_t>(index)];
        const double value =
            safePositiveRate(credit, originCpuSeconds);
        const bool first = !arm.hasUsefulCredit;
        updateEma(
            arm.usefulCreditPerCpuEma,
            value,
            first);
        updatePositiveScale(
            usefulCreditRateScale_,
            hasUsefulCreditRateScale_,
            value);
        arm.hasUsefulCredit = true;
    }

    double populationUsefulCreditEma(
            const int index) const noexcept {
        if (index < 0 ||
            static_cast<std::size_t>(index) >= arms_.size()) {
            return 0.0;
        }
        return arms_[static_cast<std::size_t>(index)]
            .usefulCreditPerCpuEma;
    }

    std::size_t populationReports(const int index) const noexcept {
        if (index < 0 ||
            static_cast<std::size_t>(index) >= arms_.size()) {
            return 0U;
        }
        return arms_[static_cast<std::size_t>(index)].reports;
    }

    std::size_t totalPopulationReports() const noexcept {
        return totalReports_;
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
        // Retain the committed flag in the public observation contract, but
        // do not interpret feasibility/acceptance alone as operator reward.
        static_cast<void>(committed);
        const bool first = mutationReports_ == 0U;
        updateEma(
            mutationImprovementPerCpuEma_,
            safePositiveRate(objectiveImprovement, cpuSeconds),
            first);
        updatePositiveScale(
            mutationImprovementRateScale_,
            hasMutationImprovementRateScale_,
            safePositiveRate(objectiveImprovement, cpuSeconds));
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
            // A committed ruin/recreate move is not evidence that increasing
            // its search share is useful. Only realized objective progress
            // contributes a positive operator reward.
            probability += 6.0 * boundedRate(
                mutationImprovementPerCpuEma_,
                mutationImprovementRateScale_);
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
        double usefulCreditPerCpuEma = 0.0;
        bool hasUsefulCredit = false;
    };

    int selectPopulationByScore(
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
                boundedRate(
                    arm.improvementPerCpuEma,
                    improvementRateScale_) +
                exploration +
                policy_.diversityWeight * diversityValue +
                policy_.columnWeight *
                    (boundedRate(
                         arm.usefulCreditPerCpuEma,
                         usefulCreditRateScale_) +
                     0.10 * boundedRate(
                         arm.columnsPerCpuEma,
                         columnsRateScale_));
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

    static SearchArmObservation sharedObservation(
            const SearchArmObservation& observation) {
        SearchArmObservation shared = observation;
        shared.objectiveImprovement *= 0.5;
        shared.newColumns *= 0.5;
        shared.usefulColumnCredit *= 0.5;
        return shared;
    }

    static void validateObservation(
            const int index,
            const SearchArmObservation& observation) {
        if (index < 0 ||
            !std::isfinite(observation.objectiveImprovement) ||
            !std::isfinite(observation.cpuSeconds) ||
            !std::isfinite(observation.newColumns) ||
            !std::isfinite(observation.usefulColumnCredit) ||
            observation.objectiveImprovement < 0.0 ||
            observation.cpuSeconds < 0.0 ||
            observation.newColumns < 0.0 ||
            observation.usefulColumnCredit < 0.0) {
            throw std::invalid_argument(
                "invalid adaptive population observation");
        }
    }

    void reportPopulationUnchecked(
            const int index,
            const SearchArmObservation& observation) {
        ensureSize(static_cast<std::size_t>(index + 1));
        ArmState& arm = arms_[static_cast<std::size_t>(index)];
        const double improvementRate = safePositiveRate(
            observation.objectiveImprovement, observation.cpuSeconds);
        const double columnsRate = safePositiveRate(
            observation.newColumns, observation.cpuSeconds);
        const double usefulCreditRate = safePositiveRate(
            observation.usefulColumnCredit, observation.cpuSeconds);
        updateEma(
            arm.improvementPerCpuEma,
            improvementRate,
            arm.reports == 0U);
        updateEma(
            arm.columnsPerCpuEma,
            columnsRate,
            arm.reports == 0U);
        updateEma(
            arm.usefulCreditPerCpuEma,
            usefulCreditRate,
            arm.reports == 0U);
        updatePositiveScale(
            improvementRateScale_,
            hasImprovementRateScale_,
            improvementRate);
        updatePositiveScale(
            columnsRateScale_,
            hasColumnsRateScale_,
            columnsRate);
        updatePositiveScale(
            usefulCreditRateScale_,
            hasUsefulCreditRateScale_,
            usefulCreditRate);
        ++arm.reports;
        ++totalReports_;
    }

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

    static constexpr double kMinimumCpuSeconds = 1.0e-3;

    static double boundedCpuSeconds(
            const double cpuSeconds) noexcept {
        return std::max(cpuSeconds, kMinimumCpuSeconds);
    }

    static double boundedRate(
            const double value,
            const double scale) noexcept {
        if (!std::isfinite(value) || value <= 0.0) {
            return 0.0;
        }
        const double positiveScale =
            std::max(scale, 1.0e-12);
        if (value >= positiveScale) {
            return 1.0 / (1.0 + positiveScale / value);
        }
        const double ratio = value / positiveScale;
        return ratio / (1.0 + ratio);
    }

    static double safePositiveRate(
            const double value,
            const double cpuSeconds) noexcept {
        if (!std::isfinite(value) || value <= 0.0) {
            return 0.0;
        }
        const double denominator = boundedCpuSeconds(cpuSeconds);
        if (value >
            std::numeric_limits<double>::max() * denominator) {
            return std::numeric_limits<double>::max();
        }
        return value / denominator;
    }

    void updatePositiveScale(
            double& scale,
            bool& initialized,
            const double value) noexcept {
        if (!std::isfinite(value) || value <= 0.0) {
            return;
        }
        updateEma(scale, value, !initialized);
        initialized = true;
    }

    AdaptiveSearchPolicy policy_;
    std::vector<ArmState> arms_;
    std::size_t totalReports_ = 0U;
    std::size_t mutationReports_ = 0U;
    double mutationImprovementPerCpuEma_ = 0.0;
    double improvementRateScale_ = 0.0;
    double columnsRateScale_ = 0.0;
    double usefulCreditRateScale_ = 0.0;
    double mutationImprovementRateScale_ = 0.0;
    bool hasImprovementRateScale_ = false;
    bool hasColumnsRateScale_ = false;
    bool hasUsefulCreditRateScale_ = false;
    bool hasMutationImprovementRateScale_ = false;
};

}  // namespace hgasp

#endif  // HGASP_CORE_ADAPTIVE_SEARCH_ALLOCATOR_H_
