#include "SpatioTemporalCrh.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace hgasp {
namespace {

constexpr double kTolerance = 1.0e-12;

void validateWindow(
        const CrhTimeWindow& window,
        const char* field) {
    if (!std::isfinite(window.ready) ||
        !std::isfinite(window.due) ||
        !std::isfinite(window.service) ||
        window.ready > window.due ||
        window.service < 0.0) {
        throw std::invalid_argument(field);
    }
}

void validateTravelMatrix(
        const std::vector<std::vector<double>>& matrix,
        const std::size_t rows,
        const std::size_t columns,
        const char* field) {
    if (matrix.size() != rows) {
        throw std::invalid_argument(field);
    }
    for (const std::vector<double>& row : matrix) {
        if (row.size() != columns) {
            throw std::invalid_argument(field);
        }
        for (const double value : row) {
            if (!std::isfinite(value) || value < 0.0) {
                throw std::invalid_argument(field);
            }
        }
    }
}

void validateTemporalInput(
        const CrhConfigurationRanker& classic,
        const SpatioTemporalCrhInput& temporal) {
    if (!temporal.hardTimeWindows) {
        return;
    }
    const std::size_t customerCount =
        classic.instance().customerDemand.size();
    const std::size_t depotCount =
        classic.instance().depotCapacity.size();
    if (temporal.customerWindows.size() != customerCount ||
        temporal.depotWindows.size() != depotCount) {
        throw std::invalid_argument(
            "ST-CRH time-window dimensions are inconsistent");
    }
    for (const CrhTimeWindow& window : temporal.customerWindows) {
        validateWindow(
            window,
            "ST-CRH customer time window is invalid");
    }
    for (const CrhTimeWindow& window : temporal.depotWindows) {
        validateWindow(
            window,
            "ST-CRH depot time window is invalid");
    }
    validateTravelMatrix(
        temporal.depotToCustomerTravelTime,
        depotCount,
        customerCount,
        "ST-CRH depot-customer travel matrix is invalid");
    validateTravelMatrix(
        temporal.customerToDepotTravelTime,
        customerCount,
        depotCount,
        "ST-CRH customer-depot travel matrix is invalid");
    validateTravelMatrix(
        temporal.customerToCustomerTravelTime,
        customerCount,
        customerCount,
        "ST-CRH customer travel matrix is invalid");
}

struct DirectTemporalObservation {
    bool feasible = false;
    double earliestServiceSlack =
        -std::numeric_limits<double>::infinity();
    double returnToDepotSlack =
        -std::numeric_limits<double>::infinity();

    double limitingSlack() const noexcept {
        return std::min(
            earliestServiceSlack, returnToDepotSlack);
    }
};

DirectTemporalObservation directObservation(
        const SpatioTemporalCrhInput& temporal,
        const std::size_t depot,
        const std::size_t customer) {
    const CrhTimeWindow& depotWindow = temporal.depotWindows[depot];
    const CrhTimeWindow& customerWindow =
        temporal.customerWindows[customer];
    const double serviceStart = std::max(
        customerWindow.ready,
        depotWindow.ready +
            temporal.depotToCustomerTravelTime[depot][customer]);
    DirectTemporalObservation observation;
    observation.earliestServiceSlack =
        customerWindow.due - serviceStart;
    observation.returnToDepotSlack =
        depotWindow.due -
        (serviceStart + customerWindow.service +
         temporal.customerToDepotTravelTime[customer][depot]);
    observation.feasible =
        observation.earliestServiceSlack >= -kTolerance &&
        observation.returnToDepotSlack >= -kTolerance;
    return observation;
}

bool directedPairFeasible(
        const CrhConfigurationRanker& classic,
        const SpatioTemporalCrhInput& temporal,
        const std::size_t depot,
        const std::size_t first,
        const std::size_t second) {
    const CrhSpatialInstance& spatial = classic.instance();
    if (spatial.customerDemand[first] +
            spatial.customerDemand[second] >
            spatial.vehicleCapacity + kTolerance ||
        spatial.customerDemand[first] +
                spatial.customerDemand[second] >
            spatial.depotCapacity[depot] + kTolerance) {
        return false;
    }
    const CrhTimeWindow& depotWindow = temporal.depotWindows[depot];
    const CrhTimeWindow& firstWindow = temporal.customerWindows[first];
    const CrhTimeWindow& secondWindow = temporal.customerWindows[second];
    const double firstStart = std::max(
        firstWindow.ready,
        depotWindow.ready +
            temporal.depotToCustomerTravelTime[depot][first]);
    if (firstStart > firstWindow.due + kTolerance) {
        return false;
    }
    const double secondStart = std::max(
        secondWindow.ready,
        firstStart + firstWindow.service +
            temporal.customerToCustomerTravelTime[first][second]);
    if (secondStart > secondWindow.due + kTolerance) {
        return false;
    }
    return secondStart + secondWindow.service +
            temporal.customerToDepotTravelTime[second][depot] <=
        depotWindow.due + kTolerance;
}

double slackRisk(
        const double limitingSlack,
        const CrhTimeWindow& window) {
    const double scale = std::max(1.0, window.due - window.ready);
    if (limitingSlack >= 0.0) {
        return 1.0 / (1.0 + limitingSlack / scale);
    }
    return 1.0 + (-limitingSlack / scale);
}

struct GreedyRouteState {
    std::size_t lastCustomer = 0U;
    double lastServiceStart = 0.0;
    double load = 0.0;
};

std::size_t estimateRouteCount(
        const CrhConfigurationRanker& classic,
        const SpatioTemporalCrhInput& temporal,
        const DepotConfigurationKey& configuration,
        const std::vector<int>& assignedDepot) {
    const CrhSpatialInstance& spatial = classic.instance();
    std::size_t totalRoutes = 0U;
    for (const int depotValue : configuration.openDepots) {
        const std::size_t depot = static_cast<std::size_t>(depotValue);
        std::vector<std::size_t> customers;
        for (std::size_t customer = 0U;
             customer < assignedDepot.size();
             ++customer) {
            if (assignedDepot[customer] == depotValue) {
                customers.push_back(customer);
            }
        }
        std::sort(
            customers.begin(),
            customers.end(),
            [&temporal](const std::size_t left,
                        const std::size_t right) {
                const CrhTimeWindow& leftWindow =
                    temporal.customerWindows[left];
                const CrhTimeWindow& rightWindow =
                    temporal.customerWindows[right];
                if (leftWindow.due != rightWindow.due) {
                    return leftWindow.due < rightWindow.due;
                }
                if (leftWindow.ready != rightWindow.ready) {
                    return leftWindow.ready < rightWindow.ready;
                }
                return left < right;
            });

        std::vector<GreedyRouteState> routes;
        for (const std::size_t customer : customers) {
            const CrhTimeWindow& window =
                temporal.customerWindows[customer];
            bool appended = false;
            for (GreedyRouteState& route : routes) {
                if (route.load + spatial.customerDemand[customer] >
                    spatial.vehicleCapacity + kTolerance) {
                    continue;
                }
                const double serviceStart = std::max(
                    window.ready,
                    route.lastServiceStart +
                        temporal.customerWindows[route.lastCustomer].service +
                        temporal.customerToCustomerTravelTime[
                            route.lastCustomer][customer]);
                if (serviceStart > window.due + kTolerance ||
                    serviceStart + window.service +
                            temporal.customerToDepotTravelTime[customer][depot] >
                        temporal.depotWindows[depot].due + kTolerance) {
                    continue;
                }
                route.lastCustomer = customer;
                route.lastServiceStart = serviceStart;
                route.load += spatial.customerDemand[customer];
                appended = true;
                break;
            }
            if (!appended) {
                const double serviceStart = std::max(
                    window.ready,
                    temporal.depotWindows[depot].ready +
                        temporal.depotToCustomerTravelTime[depot][customer]);
                routes.push_back({
                    customer,
                    serviceStart,
                    spatial.customerDemand[customer],
                });
            }
        }
        totalRoutes += routes.size();
    }
    return totalRoutes;
}

}  // namespace

SpatioTemporalCrh::SpatioTemporalCrh(
        CrhConfigurationRanker classicRanker,
        SpatioTemporalCrhInput temporalInput)
    : classicRanker_(std::move(classicRanker)),
      temporalInput_(std::move(temporalInput)) {
    validateTemporalInput(classicRanker_, temporalInput_);
}

SpatioTemporalCrhScore SpatioTemporalCrh::score(
        const DepotConfigurationKey& configuration) const {
    SpatioTemporalCrhScore result;
    result.classic = classicRanker_.score(configuration);
    if (!temporalInput_.hardTimeWindows) {
        return result;
    }

    const CrhSpatialInstance& spatial = classicRanker_.instance();
    const std::size_t customerCount = spatial.customerDemand.size();
    std::vector<int> assignedDepot(customerCount, -1);
    std::vector<double> customerRisk(customerCount, 0.0);
    std::vector<std::vector<bool>> coveredByDepot(
        spatial.depotCapacity.size(),
        std::vector<bool>(customerCount, false));

    for (std::size_t customer = 0U;
         customer < customerCount;
         ++customer) {
        double bestObservedSlack =
            -std::numeric_limits<double>::infinity();
        double bestFeasibleSlack =
            -std::numeric_limits<double>::infinity();
        int bestDepot = -1;
        for (const int depotValue : result.classic.configuration.openDepots) {
            const std::size_t depot =
                static_cast<std::size_t>(depotValue);
            const DirectTemporalObservation observation =
                directObservation(temporalInput_, depot, customer);
            const bool depotCanServeCustomer =
                spatial.customerDemand[customer] <=
                spatial.depotCapacity[depot] + kTolerance;
            const bool spatiallyCovered = std::binary_search(
                classicRanker_.depotEstimates()[depot]
                    .coveredCustomers.begin(),
                classicRanker_.depotEstimates()[depot]
                    .coveredCustomers.end(),
                static_cast<int>(customer));
            coveredByDepot[depot][customer] =
                observation.feasible && depotCanServeCustomer &&
                spatiallyCovered;
            const double limitingSlack = observation.limitingSlack();
            bestObservedSlack = std::max(
                bestObservedSlack, limitingSlack);
            if (observation.feasible && depotCanServeCustomer &&
                (limitingSlack > bestFeasibleSlack ||
                 (limitingSlack == bestFeasibleSlack &&
                  (bestDepot < 0 || depotValue < bestDepot)))) {
                bestFeasibleSlack = limitingSlack;
                bestDepot = depotValue;
            }
        }
        if (bestDepot < 0 ||
            spatial.customerDemand[customer] >
                spatial.vehicleCapacity + kTolerance) {
            ++result.uninsertedCustomerCount;
        } else {
            assignedDepot[customer] = bestDepot;
        }
        customerRisk[customer] = slackRisk(
            bestDepot >= 0 ? bestFeasibleSlack : bestObservedSlack,
            temporalInput_.customerWindows[customer]);
        result.temporalRisk += customerRisk[customer];
    }

    for (std::size_t first = 0U; first < customerCount; ++first) {
        for (std::size_t second = 0U;
             second < customerCount;
             ++second) {
            if (first == second) {
                continue;
            }
            bool compatible = false;
            for (const int depotValue :
                 result.classic.configuration.openDepots) {
                if (directedPairFeasible(
                        classicRanker_,
                        temporalInput_,
                        static_cast<std::size_t>(depotValue),
                        first,
                        second)) {
                    compatible = true;
                    break;
                }
            }
            if (compatible) {
                ++result.directedCompatiblePairCount;
            }
        }
    }
    const std::size_t directedPairCount =
        customerCount > 1U
        ? customerCount * (customerCount - 1U)
        : 0U;
    if (customerCount > 0U) {
        result.temporalRisk /= static_cast<double>(customerCount);
    }
    if (directedPairCount > 0U) {
        result.temporalRisk += 1.0 -
            static_cast<double>(result.directedCompatiblePairCount) /
                static_cast<double>(directedPairCount);
    }

    result.estimatedTwRouteCount = estimateRouteCount(
        classicRanker_,
        temporalInput_,
        result.classic.configuration,
        assignedDepot);

    double overlapSum = 0.0;
    std::size_t depotPairCount = 0U;
    for (std::size_t leftOrdinal = 0U;
         leftOrdinal < result.classic.configuration.openDepots.size();
         ++leftOrdinal) {
        const std::size_t left = static_cast<std::size_t>(
            result.classic.configuration.openDepots[leftOrdinal]);
        for (std::size_t rightOrdinal = leftOrdinal + 1U;
             rightOrdinal <
                 result.classic.configuration.openDepots.size();
             ++rightOrdinal) {
            const std::size_t right = static_cast<std::size_t>(
                result.classic.configuration.openDepots[rightOrdinal]);
            double intersection = 0.0;
            double unionWeight = 0.0;
            for (std::size_t customer = 0U;
                 customer < customerCount;
                 ++customer) {
                const bool inLeft = coveredByDepot[left][customer];
                const bool inRight = coveredByDepot[right][customer];
                if (!inLeft && !inRight) {
                    continue;
                }
                const double weight = 1.0 + customerRisk[customer];
                unionWeight += weight;
                if (inLeft && inRight) {
                    intersection += weight;
                }
            }
            if (unionWeight > 0.0) {
                overlapSum += intersection / unionWeight;
            }
            ++depotPairCount;
        }
    }
    if (depotPairCount > 0U) {
        result.spatioTemporalCoverageOverlap =
            overlapSum / static_cast<double>(depotPairCount);
    }
    return result;
}

std::vector<SpatioTemporalCrhScore> SpatioTemporalCrh::rank(
        const std::vector<DepotConfigurationKey>& configurations) const {
    std::vector<SpatioTemporalCrhScore> scores;
    if (!temporalInput_.hardTimeWindows) {
        const std::vector<CrhConfigurationScore> classicScores =
            classicRanker_.rank(configurations);
        scores.reserve(classicScores.size());
        for (const CrhConfigurationScore& classicScore : classicScores) {
            SpatioTemporalCrhScore candidate;
            candidate.classic = classicScore;
            scores.push_back(std::move(candidate));
        }
        return scores;
    }

    std::set<DepotConfigurationKey> seen;
    scores.reserve(configurations.size());
    for (const DepotConfigurationKey& configuration : configurations) {
        SpatioTemporalCrhScore candidate = score(configuration);
        if (seen.insert(candidate.classic.configuration).second) {
            scores.push_back(std::move(candidate));
        }
    }
    std::sort(
        scores.begin(),
        scores.end(),
        [](const SpatioTemporalCrhScore& left,
           const SpatioTemporalCrhScore& right) {
            if (left.classic.aggregateCapacityFeasible !=
                right.classic.aggregateCapacityFeasible) {
                return left.classic.aggregateCapacityFeasible;
            }
            if (left.uninsertedCustomerCount !=
                right.uninsertedCustomerCount) {
                return left.uninsertedCustomerCount <
                    right.uninsertedCustomerCount;
            }
            if (left.classic.estimatedCost !=
                right.classic.estimatedCost) {
                return left.classic.estimatedCost <
                    right.classic.estimatedCost;
            }
            if (left.estimatedTwRouteCount !=
                right.estimatedTwRouteCount) {
                return left.estimatedTwRouteCount <
                    right.estimatedTwRouteCount;
            }
            if (left.temporalRisk != right.temporalRisk) {
                return left.temporalRisk < right.temporalRisk;
            }
            if (left.spatioTemporalCoverageOverlap !=
                right.spatioTemporalCoverageOverlap) {
                return left.spatioTemporalCoverageOverlap <
                    right.spatioTemporalCoverageOverlap;
            }
            return left.classic.configuration <
                right.classic.configuration;
        });
    return scores;
}

}  // namespace hgasp
