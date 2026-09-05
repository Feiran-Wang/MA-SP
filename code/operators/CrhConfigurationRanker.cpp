#include "CrhConfigurationRanker.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <set>
#include <stdexcept>
#include <utility>

namespace hgasp {
namespace {

constexpr double kTolerance = 1.0e-12;

void requireFiniteNonNegative(
        const double value,
        const char* field) {
    if (!std::isfinite(value) || value < 0.0) {
        throw std::invalid_argument(field);
    }
}

void validateSpatialInstance(const CrhSpatialInstance& instance) {
    const std::size_t customerCount = instance.customerDemand.size();
    const std::size_t depotCount = instance.depotCapacity.size();
    if (depotCount == 0U ||
        instance.depotOpeningCost.size() != depotCount ||
        instance.depotToCustomerCost.size() != depotCount ||
        instance.customerToCustomerCost.size() != customerCount) {
        throw std::invalid_argument(
            "CRH instance dimensions are inconsistent");
    }
    requireFiniteNonNegative(
        instance.vehicleFixedCost,
        "CRH vehicle fixed cost must be finite and non-negative");
    if (!std::isfinite(instance.vehicleCapacity) ||
        instance.vehicleCapacity <= 0.0) {
        throw std::invalid_argument(
            "CRH vehicle capacity must be finite and positive");
    }
    for (const double demand : instance.customerDemand) {
        requireFiniteNonNegative(
            demand,
            "CRH customer demand must be finite and non-negative");
    }
    for (std::size_t depot = 0U; depot < depotCount; ++depot) {
        requireFiniteNonNegative(
            instance.depotCapacity[depot],
            "CRH depot capacity must be finite and non-negative");
        requireFiniteNonNegative(
            instance.depotOpeningCost[depot],
            "CRH depot opening cost must be finite and non-negative");
        if (instance.depotToCustomerCost[depot].size() !=
            customerCount) {
            throw std::invalid_argument(
                "CRH depot-customer matrix dimensions are inconsistent");
        }
        for (const double cost :
             instance.depotToCustomerCost[depot]) {
            requireFiniteNonNegative(
                cost,
                "CRH depot-customer cost must be finite and non-negative");
        }
    }
    for (const std::vector<double>& row :
         instance.customerToCustomerCost) {
        if (row.size() != customerCount) {
            throw std::invalid_argument(
                "CRH customer matrix dimensions are inconsistent");
        }
        for (const double cost : row) {
            requireFiniteNonNegative(
                cost,
                "CRH customer cost must be finite and non-negative");
        }
    }
}

CrhDepotEstimate estimateDepot(
        const CrhSpatialInstance& instance,
        const int depot) {
    const std::size_t customerCount = instance.customerDemand.size();
    std::vector<bool> included(customerCount, false);
    std::vector<double> lowCost =
        instance.depotToCustomerCost[static_cast<std::size_t>(depot)];
    CrhDepotEstimate estimate;
    estimate.depot = depot;
    double currentRouteLoad = 0.0;

    for (std::size_t inclusionCount = 0U;
         inclusionCount < customerCount;
         ++inclusionCount) {
        std::size_t selected = customerCount;
        double selectedCost = std::numeric_limits<double>::infinity();
        for (std::size_t customer = 0U;
             customer < customerCount;
             ++customer) {
            if (included[customer]) {
                continue;
            }
            const double demand = instance.customerDemand[customer];
            if (demand > instance.vehicleCapacity + kTolerance ||
                estimate.coveredDemand + demand >
                    instance.depotCapacity[static_cast<std::size_t>(depot)] +
                    kTolerance) {
                continue;
            }
            if (lowCost[customer] < selectedCost ||
                (lowCost[customer] == selectedCost &&
                 customer < selected)) {
                selected = customer;
                selectedCost = lowCost[customer];
            }
        }
        if (selected == customerCount) {
            break;
        }

        const double demand = instance.customerDemand[selected];
        if (estimate.estimatedVehicleCount == 0U ||
            currentRouteLoad + demand >
                instance.vehicleCapacity + kTolerance) {
            ++estimate.estimatedVehicleCount;
            currentRouteLoad = 0.0;
        }
        currentRouteLoad += demand;
        estimate.coveredDemand += demand;
        estimate.mstLikeRoutingCost += selectedCost;
        estimate.coveredCustomers.push_back(
            static_cast<int>(selected));
        included[selected] = true;

        for (std::size_t customer = 0U;
             customer < customerCount;
             ++customer) {
            if (!included[customer]) {
                lowCost[customer] = std::min(
                    lowCost[customer],
                    instance.customerToCustomerCost[customer][selected]);
            }
        }
    }
    std::sort(
        estimate.coveredCustomers.begin(),
        estimate.coveredCustomers.end());
    estimate.estimatedCost =
        instance.depotOpeningCost[static_cast<std::size_t>(depot)] +
        estimate.mstLikeRoutingCost +
        static_cast<double>(estimate.estimatedVehicleCount) *
            instance.vehicleFixedCost;
    return estimate;
}

std::uint64_t splitMix64(std::uint64_t value) noexcept {
    value += 0x9e3779b97f4a7c15ULL;
    value = (value ^ (value >> 30U)) *
        0xbf58476d1ce4e5b9ULL;
    value = (value ^ (value >> 27U)) *
        0x94d049bb133111ebULL;
    return value ^ (value >> 31U);
}

double deterministicUnit(const std::uint64_t value) noexcept {
    return static_cast<double>(splitMix64(value) >> 11U) *
        (1.0 / 9007199254740992.0);
}

double coverageOverlap(
        const std::vector<int>& coverage,
        const std::set<int>& covered) {
    if (coverage.empty()) {
        return 1.0;
    }
    const std::size_t overlap =
        static_cast<std::size_t>(std::count_if(
            coverage.begin(),
            coverage.end(),
            [&covered](const int customer) {
                return covered.find(customer) != covered.end();
            }));
    return static_cast<double>(overlap) /
        static_cast<double>(coverage.size());
}

void validateScreeningPolicy(
        const ProgressiveConfigurationScreeningPolicy& policy) {
    if (policy.cheapLimit < policy.firstStageLimit ||
        policy.firstStageLimit < policy.finalistLimit ||
        policy.finalistLimit < policy.finalLimit ||
        policy.firstStageSamples == 0U ||
        policy.totalSamples < policy.firstStageSamples) {
        throw std::invalid_argument(
            "progressive configuration screening policy is inconsistent");
    }
}

DepotConfigurationKey validateScreeningCandidate(
        const ProgressiveConfigurationCandidate& candidate) {
    DepotConfigurationKey canonical =
        makeDepotConfigurationKey(candidate.configuration.openDepots);
    if (canonical.openDepots.empty()) {
        throw std::invalid_argument(
            "configuration screening candidate opens no depot");
    }
    if (!std::isfinite(candidate.cheapScore) ||
        !std::isfinite(candidate.diversity) ||
        candidate.diversity < 0.0) {
        throw std::invalid_argument(
            "configuration screening score is invalid");
    }
    for (const double objective : candidate.successfulObjectives) {
        if (!std::isfinite(objective)) {
            throw std::invalid_argument(
                "configuration screening objective is not finite");
        }
    }
    if (candidate.successfulObjectives.size() >
        candidate.attemptedSamples) {
        throw std::invalid_argument(
            "configuration has more successes than attempted samples");
    }
    return canonical;
}

double meanObjective(
        const ProgressiveConfigurationCandidate& candidate) {
    return std::accumulate(
               candidate.successfulObjectives.begin(),
               candidate.successfulObjectives.end(),
               0.0) /
        static_cast<double>(candidate.successfulObjectives.size());
}

double bestObjective(
        const ProgressiveConfigurationCandidate& candidate) {
    return *std::min_element(
        candidate.successfulObjectives.begin(),
        candidate.successfulObjectives.end());
}

bool evaluatedCandidateLess(
        const ProgressiveConfigurationCandidate& left,
        const ProgressiveConfigurationCandidate& right,
        const EvaluatedConfigurationRanking ranking) {
    if (left.successfulObjectives.size() !=
        right.successfulObjectives.size()) {
        return left.successfulObjectives.size() >
            right.successfulObjectives.size();
    }
    const double leftBest = bestObjective(left);
    const double rightBest = bestObjective(right);
    const double leftMean = meanObjective(left);
    const double rightMean = meanObjective(right);
    if (ranking == EvaluatedConfigurationRanking::BestObjectiveFirst) {
        if (leftBest != rightBest) {
            return leftBest < rightBest;
        }
        if (leftMean != rightMean) {
            return leftMean < rightMean;
        }
    } else {
        if (leftMean != rightMean) {
            return leftMean < rightMean;
        }
        if (leftBest != rightBest) {
            return leftBest < rightBest;
        }
    }
    if (left.diversity != right.diversity) {
        return left.diversity > right.diversity;
    }
    if (left.cheapScore != right.cheapScore) {
        return left.cheapScore < right.cheapScore;
    }
    if (left.configuration != right.configuration) {
        return left.configuration < right.configuration;
    }
    return left.sourceIndex < right.sourceIndex;
}

std::vector<ProgressiveConfigurationCandidate> selectEvaluated(
        std::vector<ProgressiveConfigurationCandidate> candidates,
        const std::size_t requiredAttempts,
        const std::size_t limit,
        const EvaluatedConfigurationRanking ranking) {
    for (ProgressiveConfigurationCandidate& candidate : candidates) {
        candidate.configuration = validateScreeningCandidate(candidate);
        if (candidate.attemptedSamples < requiredAttempts) {
            throw std::invalid_argument(
                "configuration screening stage is incomplete");
        }
    }
    candidates.erase(
        std::remove_if(
            candidates.begin(),
            candidates.end(),
            [](const ProgressiveConfigurationCandidate& candidate) {
                return candidate.successfulObjectives.empty();
            }),
        candidates.end());
    std::sort(
        candidates.begin(),
        candidates.end(),
        [ranking](const ProgressiveConfigurationCandidate& left,
                  const ProgressiveConfigurationCandidate& right) {
            return evaluatedCandidateLess(left, right, ranking);
        });

    std::set<DepotConfigurationKey> seen;
    std::vector<ProgressiveConfigurationCandidate> selected;
    selected.reserve(std::min(limit, candidates.size()));
    for (ProgressiveConfigurationCandidate& candidate : candidates) {
        if (!seen.insert(candidate.configuration).second) {
            continue;
        }
        selected.push_back(std::move(candidate));
        if (selected.size() == limit) {
            break;
        }
    }
    return selected;
}

}  // namespace

CrhConfigurationRanker::CrhConfigurationRanker(
        CrhSpatialInstance instance)
    : instance_(std::move(instance)) {
    validateSpatialInstance(instance_);
    totalDemand_ = std::accumulate(
        instance_.customerDemand.begin(),
        instance_.customerDemand.end(),
        0.0);
    depotEstimates_.reserve(instance_.depotCapacity.size());
    for (std::size_t depot = 0U;
         depot < instance_.depotCapacity.size();
         ++depot) {
        depotEstimates_.push_back(estimateDepot(
            instance_, static_cast<int>(depot)));
    }
}

DepotConfigurationKey
CrhConfigurationRanker::validateConfiguration(
        const DepotConfigurationKey& configuration) const {
    DepotConfigurationKey canonical =
        makeDepotConfigurationKey(configuration.openDepots);
    if (canonical.openDepots.empty()) {
        throw std::invalid_argument(
            "CRH configuration opens no depot");
    }
    for (const int depot : canonical.openDepots) {
        if (static_cast<std::size_t>(depot) >=
            instance_.depotCapacity.size()) {
            throw std::invalid_argument(
                "CRH configuration has an out-of-range depot");
        }
    }
    return canonical;
}

CrhConfigurationScore CrhConfigurationRanker::score(
        const DepotConfigurationKey& configuration) const {
    CrhConfigurationScore result;
    result.configuration = validateConfiguration(configuration);
    std::set<int> uniqueCoverage;
    std::size_t coverageOccurrences = 0U;
    for (const int depot : result.configuration.openDepots) {
        const std::size_t index = static_cast<std::size_t>(depot);
        result.aggregateCapacity += instance_.depotCapacity[index];
        result.estimatedCost += depotEstimates_[index].estimatedCost;
        coverageOccurrences +=
            depotEstimates_[index].coveredCustomers.size();
        uniqueCoverage.insert(
            depotEstimates_[index].coveredCustomers.begin(),
            depotEstimates_[index].coveredCustomers.end());
    }
    result.aggregateCapacityFeasible =
        result.aggregateCapacity + kTolerance >= totalDemand_;
    if (coverageOccurrences > 0U) {
        result.spatialCoverageOverlap =
            static_cast<double>(
                coverageOccurrences - uniqueCoverage.size()) /
            static_cast<double>(coverageOccurrences);
    }
    return result;
}

std::vector<CrhConfigurationScore> CrhConfigurationRanker::rank(
        const std::vector<DepotConfigurationKey>& configurations) const {
    std::set<DepotConfigurationKey> seen;
    std::vector<CrhConfigurationScore> scores;
    scores.reserve(configurations.size());
    for (const DepotConfigurationKey& configuration : configurations) {
        CrhConfigurationScore candidate = score(configuration);
        if (seen.insert(candidate.configuration).second) {
            scores.push_back(std::move(candidate));
        }
    }
    std::sort(
        scores.begin(),
        scores.end(),
        [](const CrhConfigurationScore& left,
           const CrhConfigurationScore& right) {
            if (left.aggregateCapacityFeasible !=
                right.aggregateCapacityFeasible) {
                return left.aggregateCapacityFeasible;
            }
            if (left.estimatedCost != right.estimatedCost) {
                return left.estimatedCost < right.estimatedCost;
            }
            if (left.spatialCoverageOverlap !=
                right.spatialCoverageOverlap) {
                return left.spatialCoverageOverlap <
                    right.spatialCoverageOverlap;
            }
            return left.configuration < right.configuration;
        });
    return scores;
}

std::vector<DepotConfigurationKey>
CrhConfigurationRanker::constructCandidates(
        const std::size_t requestedLimit,
        const std::size_t samplesPerStartingDepot,
        const std::uint64_t deterministicSeed,
        const double rankBiasExponent) const {
    if (requestedLimit == 0U || samplesPerStartingDepot == 0U) {
        return {};
    }
    if (!std::isfinite(rankBiasExponent) ||
        rankBiasExponent <= 0.0) {
        throw std::invalid_argument(
            "CRH rank bias exponent must be finite and positive");
    }
    const double totalCapacity = std::accumulate(
        instance_.depotCapacity.begin(),
        instance_.depotCapacity.end(),
        0.0);
    if (totalCapacity + kTolerance < totalDemand_) {
        return {};
    }

    std::vector<int> startOrder(instance_.depotCapacity.size());
    std::iota(startOrder.begin(), startOrder.end(), 0);
    std::sort(
        startOrder.begin(),
        startOrder.end(),
        [this](const int left, const int right) {
            const double leftCost =
                depotEstimates_[static_cast<std::size_t>(left)].estimatedCost;
            const double rightCost =
                depotEstimates_[static_cast<std::size_t>(right)].estimatedCost;
            if (leftCost != rightCost) {
                return leftCost < rightCost;
            }
            return left < right;
        });

    std::set<DepotConfigurationKey> unique;
    for (std::size_t sample = 0U;
         sample < samplesPerStartingDepot &&
         unique.size() < requestedLimit;
         ++sample) {
        for (std::size_t startOrdinal = 0U;
             startOrdinal < startOrder.size() &&
             unique.size() < requestedLimit;
             ++startOrdinal) {
            const int startDepot = startOrder[startOrdinal];
            std::vector<int> open{startDepot};
            std::vector<bool> isOpen(
                instance_.depotCapacity.size(), false);
            isOpen[static_cast<std::size_t>(startDepot)] = true;
            double capacity = instance_.depotCapacity[
                static_cast<std::size_t>(startDepot)];
            std::set<int> covered(
                depotEstimates_[static_cast<std::size_t>(startDepot)]
                    .coveredCustomers.begin(),
                depotEstimates_[static_cast<std::size_t>(startDepot)]
                    .coveredCustomers.end());
            std::size_t step = 0U;

            while (capacity + kTolerance < totalDemand_) {
                struct Addition {
                    int depot = -1;
                    double overlap = 0.0;
                    double cost = 0.0;
                };
                std::vector<Addition> remaining;
                std::vector<Addition> eligible;
                for (std::size_t depot = 0U;
                     depot < instance_.depotCapacity.size();
                     ++depot) {
                    if (isOpen[depot]) {
                        continue;
                    }
                    Addition addition;
                    addition.depot = static_cast<int>(depot);
                    addition.overlap = coverageOverlap(
                        depotEstimates_[depot].coveredCustomers,
                        covered);
                    addition.cost = depotEstimates_[depot].estimatedCost;
                    remaining.push_back(addition);
                    const double afterCapacity =
                        capacity + instance_.depotCapacity[depot];
                    const double threshold = totalDemand_ > kTolerance
                        ? 0.1 + 0.5 * (afterCapacity / totalDemand_)
                        : 0.6;
                    if (addition.overlap <= threshold + kTolerance) {
                        eligible.push_back(addition);
                    }
                }
                if (remaining.empty()) {
                    break;
                }
                if (eligible.empty()) {
                    // The overlap filter is guidance, not a feasibility cut.
                    eligible = remaining;
                }
                std::sort(
                    eligible.begin(),
                    eligible.end(),
                    [](const Addition& left, const Addition& right) {
                        if (left.cost != right.cost) {
                            return left.cost < right.cost;
                        }
                        if (left.overlap != right.overlap) {
                            return left.overlap < right.overlap;
                        }
                        return left.depot < right.depot;
                    });
                const std::uint64_t drawKey = deterministicSeed ^
                    (static_cast<std::uint64_t>(sample + 1U) << 48U) ^
                    (static_cast<std::uint64_t>(startOrdinal + 1U) << 24U) ^
                    static_cast<std::uint64_t>(step + 1U);
                const double biased = std::pow(
                    deterministicUnit(drawKey), rankBiasExponent);
                std::size_t rank = static_cast<std::size_t>(
                    biased * static_cast<double>(eligible.size()));
                rank = std::min(rank, eligible.size() - 1U);
                const int selected = eligible[rank].depot;
                open.push_back(selected);
                isOpen[static_cast<std::size_t>(selected)] = true;
                capacity += instance_.depotCapacity[
                    static_cast<std::size_t>(selected)];
                covered.insert(
                    depotEstimates_[static_cast<std::size_t>(selected)]
                        .coveredCustomers.begin(),
                    depotEstimates_[static_cast<std::size_t>(selected)]
                        .coveredCustomers.end());
                ++step;
            }
            if (capacity + kTolerance >= totalDemand_) {
                unique.insert(makeDepotConfigurationKey(std::move(open)));
            }
        }
    }
    return {unique.begin(), unique.end()};
}

ProgressiveCheapScreeningResult screenConfigurationCheapStage(
        std::vector<ProgressiveConfigurationCandidate> candidates,
        const ProgressiveConfigurationScreeningPolicy& policy) {
    validateScreeningPolicy(policy);
    for (ProgressiveConfigurationCandidate& candidate : candidates) {
        candidate.configuration = validateScreeningCandidate(candidate);
    }
    candidates.erase(
        std::remove_if(
            candidates.begin(),
            candidates.end(),
            [](const ProgressiveConfigurationCandidate& candidate) {
                return !candidate.aggregateCapacityFeasible;
            }),
        candidates.end());
    std::sort(
        candidates.begin(),
        candidates.end(),
        [](const ProgressiveConfigurationCandidate& left,
           const ProgressiveConfigurationCandidate& right) {
            if (left.cheapScore != right.cheapScore) {
                return left.cheapScore < right.cheapScore;
            }
            if (left.configuration != right.configuration) {
                return left.configuration < right.configuration;
            }
            return left.sourceIndex < right.sourceIndex;
        });

    ProgressiveCheapScreeningResult result;
    std::set<DepotConfigurationKey> seen;
    result.cheapPool.reserve(std::min(policy.cheapLimit, candidates.size()));
    for (ProgressiveConfigurationCandidate& candidate : candidates) {
        if (!seen.insert(candidate.configuration).second) {
            continue;
        }
        result.cheapPool.push_back(std::move(candidate));
        if (result.cheapPool.size() == policy.cheapLimit) {
            break;
        }
    }
    const std::size_t firstStageSize = std::min(
        policy.firstStageLimit, result.cheapPool.size());
    result.firstStage.assign(
        result.cheapPool.begin(),
        result.cheapPool.begin() +
            static_cast<std::ptrdiff_t>(firstStageSize));
    return result;
}

std::vector<ProgressiveConfigurationCandidate>
selectConfigurationFinalists(
        std::vector<ProgressiveConfigurationCandidate> candidates,
        const ProgressiveConfigurationScreeningPolicy& policy) {
    validateScreeningPolicy(policy);
    return selectEvaluated(
        std::move(candidates),
        policy.firstStageSamples,
        policy.finalistLimit,
        policy.ranking);
}

std::vector<ProgressiveConfigurationCandidate>
selectStaticFixedConfigurations(
        std::vector<ProgressiveConfigurationCandidate> candidates,
        const ProgressiveConfigurationScreeningPolicy& policy) {
    validateScreeningPolicy(policy);
    return selectEvaluated(
        std::move(candidates),
        policy.totalSamples,
        policy.finalLimit,
        policy.ranking);
}

}  // namespace hgasp
