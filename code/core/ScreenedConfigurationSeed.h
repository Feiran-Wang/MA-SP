#ifndef HGASP_CORE_SCREENED_CONFIGURATION_SEED_H_
#define HGASP_CORE_SCREENED_CONFIGURATION_SEED_H_

#include "RouteKey.h"

#include <cstddef>
#include <limits>
#include <vector>

namespace hgasp {

// Lightweight, ownership-independent snapshot of an exact-configuration
// individual.  Population builders reconstruct an Individual from routes;
// the archive never owns or aliases population objects.
struct ScreenedConfigurationSeed {
    DepotConfigurationKey configuration;
    std::vector<StandaloneRoute> routes;
    double objective = std::numeric_limits<double>::infinity();
    // Supplied by the existing broken-pairs/route-structure calculation.
    double diversity = 0.0;
    std::size_t sourceIndex = 0U;
};

}  // namespace hgasp

#endif  // HGASP_CORE_SCREENED_CONFIGURATION_SEED_H_
