#ifndef HGASP_MODELS_ROUTE_H_
#define HGASP_MODELS_ROUTE_H_

#include <cstddef>
#include <functional>
#include <vector>

// Value representation exchanged between the route generator, route pool,
// and SP module. Customer and depot identifiers are zero-based.
struct StandaloneRoute {
    int locId = -1;
    double routeCost = 0.0;
    int totalLoad = 0;
    std::vector<int> sequence;

    bool operator==(const StandaloneRoute& other) const noexcept {
        return locId == other.locId && sequence == other.sequence;
    }
};

// Sequence order is intentional: HGAMP permits asymmetric travel costs, so a
// route and its reversal are not generally equivalent.
struct StandaloneRouteHash {
    std::size_t operator()(const StandaloneRoute& route) const noexcept {
        std::size_t seed = route.sequence.size();
        seed ^= std::hash<int>()(route.locId) +
                0x9e3779b9U +
                (seed << 6U) +
                (seed >> 2U);
        for (const int customer : route.sequence) {
            seed ^= std::hash<int>()(customer) +
                    0x9e3779b9U +
                    (seed << 6U) +
                    (seed >> 2U);
        }
        return seed;
    }
};

#endif  // HGASP_MODELS_ROUTE_H_
