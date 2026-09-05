#ifndef HGASP_OPERATORS_SPATIO_TEMPORAL_CRH_H_
#define HGASP_OPERATORS_SPATIO_TEMPORAL_CRH_H_

#include "CrhConfigurationRanker.h"

#include <cstddef>
#include <vector>

namespace hgasp {

struct CrhTimeWindow {
    double ready = 0.0;
    double due = 0.0;
    double service = 0.0;
};

struct SpatioTemporalCrhInput {
    bool hardTimeWindows = false;
    std::vector<CrhTimeWindow> customerWindows;
    std::vector<CrhTimeWindow> depotWindows;
    std::vector<std::vector<double>> depotToCustomerTravelTime;
    std::vector<std::vector<double>> customerToDepotTravelTime;
    std::vector<std::vector<double>> customerToCustomerTravelTime;
};

struct SpatioTemporalCrhScore {
    CrhConfigurationScore classic;
    std::size_t uninsertedCustomerCount = 0U;
    std::size_t estimatedTwRouteCount = 0U;
    std::size_t directedCompatiblePairCount = 0U;
    double temporalRisk = 0.0;
    double spatioTemporalCoverageOverlap = 0.0;

    // This score is ranking evidence only. It never certifies full route or
    // configuration feasibility.
    bool isFeasibilityCertificate() const noexcept {
        return false;
    }
};

class SpatioTemporalCrh {
public:
    SpatioTemporalCrh(
        CrhConfigurationRanker classicRanker,
        SpatioTemporalCrhInput temporalInput);

    SpatioTemporalCrhScore score(
        const DepotConfigurationKey& configuration) const;

    // With hardTimeWindows=false this is exactly the classic CRH ordering.
    // With hard windows, every valid input candidate remains in the output;
    // temporal metrics only change its position.
    std::vector<SpatioTemporalCrhScore> rank(
        const std::vector<DepotConfigurationKey>& configurations) const;

private:
    CrhConfigurationRanker classicRanker_;
    SpatioTemporalCrhInput temporalInput_;
};

}  // namespace hgasp

#endif  // HGASP_OPERATORS_SPATIO_TEMPORAL_CRH_H_
