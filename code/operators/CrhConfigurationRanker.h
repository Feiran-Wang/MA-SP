#ifndef HGASP_OPERATORS_CRH_CONFIGURATION_RANKER_H_
#define HGASP_OPERATORS_CRH_CONFIGURATION_RANKER_H_

#include "RouteKey.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace hgasp {

inline constexpr std::size_t kV114ConfigurationCheapLimit = 100U;
inline constexpr std::size_t kV114ConfigurationFirstStageLimit = 50U;
inline constexpr std::size_t kV114ConfigurationFinalistLimit = 20U;
inline constexpr std::size_t kV114StaticFixedPopulationCount = 15U;
inline constexpr std::size_t kV114ConfigurationFirstSamples = 3U;
inline constexpr std::size_t kV114ConfigurationTotalSamples = 10U;

// Matrix conventions preserve the public HGAMP CRH code: depot-customer
// labels start from depotToCustomerCost[d][c], then an included customer v
// relaxes a candidate customer c with customerToCustomerCost[c][v].
struct CrhSpatialInstance {
    std::vector<double> customerDemand;
    std::vector<double> depotCapacity;
    std::vector<double> depotOpeningCost;
    double vehicleCapacity = 0.0;
    double vehicleFixedCost = 0.0;
    std::vector<std::vector<double>> depotToCustomerCost;
    std::vector<std::vector<double>> customerToCustomerCost;
};

struct CrhDepotEstimate {
    int depot = -1;
    std::vector<int> coveredCustomers;
    double coveredDemand = 0.0;
    std::size_t estimatedVehicleCount = 0U;
    double mstLikeRoutingCost = 0.0;
    double estimatedCost = 0.0;
};

struct CrhConfigurationScore {
    DepotConfigurationKey configuration;
    bool aggregateCapacityFeasible = false;
    double aggregateCapacity = 0.0;
    double estimatedCost = 0.0;
    double spatialCoverageOverlap = 0.0;
};

class CrhConfigurationRanker {
public:
    explicit CrhConfigurationRanker(CrhSpatialInstance instance);

    const CrhSpatialInstance& instance() const noexcept {
        return instance_;
    }

    double totalDemand() const noexcept {
        return totalDemand_;
    }

    const std::vector<CrhDepotEstimate>& depotEstimates() const noexcept {
        return depotEstimates_;
    }

    CrhConfigurationScore score(
        const DepotConfigurationKey& configuration) const;

    std::vector<CrhConfigurationScore> rank(
        const std::vector<DepotConfigurationKey>& configurations) const;

    // Reconstructs the public CRH overlap/cost intent with bounded,
    // deterministic rank-biased choices. It returns only unique, non-empty
    // configurations satisfying the aggregate-capacity necessary condition.
    std::vector<DepotConfigurationKey> constructCandidates(
        std::size_t requestedLimit,
        std::size_t samplesPerStartingDepot = 3U,
        std::uint64_t deterministicSeed = 0U,
        double rankBiasExponent = 6.0) const;

private:
    DepotConfigurationKey validateConfiguration(
        const DepotConfigurationKey& configuration) const;

    CrhSpatialInstance instance_;
    double totalDemand_ = 0.0;
    std::vector<CrhDepotEstimate> depotEstimates_;
};

struct ProgressiveConfigurationCandidate {
    std::size_t sourceIndex = 0U;
    DepotConfigurationKey configuration;
    bool aggregateCapacityFeasible = false;
    double cheapScore = 0.0;
    std::size_t attemptedSamples = 0U;
    std::vector<double> successfulObjectives;
    double diversity = 0.0;
};

enum class EvaluatedConfigurationRanking {
    StabilityFirst,
    BestObjectiveFirst,
};

struct ProgressiveConfigurationScreeningPolicy {
    std::size_t cheapLimit = kV114ConfigurationCheapLimit;
    std::size_t firstStageLimit = kV114ConfigurationFirstStageLimit;
    std::size_t finalistLimit = kV114ConfigurationFinalistLimit;
    std::size_t finalLimit = kV114StaticFixedPopulationCount;
    std::size_t firstStageSamples = kV114ConfigurationFirstSamples;
    std::size_t totalSamples = kV114ConfigurationTotalSamples;
    EvaluatedConfigurationRanking ranking =
        EvaluatedConfigurationRanking::StabilityFirst;
};

// Cheap ranking retains at most cheapLimit unique candidates; the returned
// firstStage vector is its first firstStageLimit entries.
struct ProgressiveCheapScreeningResult {
    std::vector<ProgressiveConfigurationCandidate> cheapPool;
    std::vector<ProgressiveConfigurationCandidate> firstStage;
};

ProgressiveCheapScreeningResult screenConfigurationCheapStage(
    std::vector<ProgressiveConfigurationCandidate> candidates,
    const ProgressiveConfigurationScreeningPolicy& policy = {});

// Requires firstStageSamples attempted evaluations per candidate, removes
// zero-success configurations, and keeps at most finalistLimit entries.
std::vector<ProgressiveConfigurationCandidate>
selectConfigurationFinalists(
    std::vector<ProgressiveConfigurationCandidate> candidates,
    const ProgressiveConfigurationScreeningPolicy& policy = {});

// Requires totalSamples attempted evaluations per candidate, removes
// zero-success configurations, and keeps at most finalLimit unique entries.
std::vector<ProgressiveConfigurationCandidate>
selectStaticFixedConfigurations(
    std::vector<ProgressiveConfigurationCandidate> candidates,
    const ProgressiveConfigurationScreeningPolicy& policy = {});

}  // namespace hgasp

#endif  // HGASP_OPERATORS_CRH_CONFIGURATION_RANKER_H_
