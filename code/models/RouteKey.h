#ifndef HGASP_MODELS_ROUTE_KEY_H_
#define HGASP_MODELS_ROUTE_KEY_H_

#include "Route.h"

#include <algorithm>
#include <cstddef>
#include <functional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace hgasp {

// Canonical identity of a restricted-master column. Visit order is excluded:
// for a fixed depot and customer set, only the cheapest feasible ordering is
// relevant to the set-partitioning master.
struct RouteKey {
    int depot = -1;
    std::vector<int> customers;

    bool operator==(const RouteKey& other) const noexcept {
        return depot == other.depot && customers == other.customers;
    }

    bool operator!=(const RouteKey& other) const noexcept {
        return !(*this == other);
    }

    bool operator<(const RouteKey& other) const noexcept {
        if (depot != other.depot) {
            return depot < other.depot;
        }
        return customers < other.customers;
    }
};

struct RouteKeyHash {
    std::size_t operator()(const RouteKey& key) const noexcept {
        std::size_t seed = key.customers.size();
        seed ^= std::hash<int>()(key.depot) +
                0x9e3779b9U +
                (seed << 6U) +
                (seed >> 2U);
        for (const int customer : key.customers) {
            seed ^= std::hash<int>()(customer) +
                    0x9e3779b9U +
                    (seed << 6U) +
                    (seed >> 2U);
        }
        return seed;
    }
};

inline RouteKey makeRouteKey(const StandaloneRoute& route) {
    RouteKey key;
    key.depot = route.locId;
    key.customers = route.sequence;
    std::sort(key.customers.begin(), key.customers.end());
    return key;
}

struct DepotConfigurationKey {
    std::vector<int> openDepots;

    bool operator==(
            const DepotConfigurationKey& other) const noexcept {
        return openDepots == other.openDepots;
    }

    bool operator!=(
            const DepotConfigurationKey& other) const noexcept {
        return !(*this == other);
    }

    bool operator<(
            const DepotConfigurationKey& other) const noexcept {
        return openDepots < other.openDepots;
    }
};

struct DepotConfigurationKeyHash {
    std::size_t operator()(
            const DepotConfigurationKey& key) const noexcept {
        std::size_t seed = key.openDepots.size();
        for (const int depot : key.openDepots) {
            seed ^= std::hash<int>()(depot) +
                    0x9e3779b9U +
                    (seed << 6U) +
                    (seed >> 2U);
        }
        return seed;
    }
};

inline DepotConfigurationKey makeDepotConfigurationKey(
        std::vector<int> openDepots) {
    for (const int depot : openDepots) {
        if (depot < 0) {
            throw std::invalid_argument(
                "depot configuration contains a negative index");
        }
    }
    std::sort(openDepots.begin(), openDepots.end());
    openDepots.erase(
        std::unique(openDepots.begin(), openDepots.end()),
        openDepots.end());
    return {std::move(openDepots)};
}

inline DepotConfigurationKey makeDepotConfigurationKey(
        const std::vector<bool>& openDepots) {
    std::vector<int> indices;
    indices.reserve(openDepots.size());
    for (std::size_t depot = 0;
         depot < openDepots.size();
         ++depot) {
        if (openDepots[depot]) {
            indices.push_back(static_cast<int>(depot));
        }
    }
    return {std::move(indices)};
}

}  // namespace hgasp

#endif  // HGASP_MODELS_ROUTE_KEY_H_
