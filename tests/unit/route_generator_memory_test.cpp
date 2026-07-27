#include "Individual.h"
#include "read_data.h"

#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <utility>
#include <vector>

std::mt19937 global_rng;

namespace {

int fail(const std::string& message) {
    std::cerr << "route_generator_memory_test: " << message << std::endl;
    return 1;
}

StandaloneRoute makeRoute(
    const int depot,
    std::vector<int> customers) {
    StandaloneRoute route;
    route.locId = depot;
    route.totalLoad = static_cast<int>(customers.size());
    route.routeCost = static_cast<double>(customers.size() + 1);
    route.sequence = std::move(customers);
    return route;
}

double referenceBrokenPairsDistance(
    Individual& individual,
    Individual& other,
    const int numV,
    const int numC) {
    double distance = 0.0;
    for (int i = numV; i < numC; ++i) {
        if (!individual.location[i - numV].used) {
            continue;
        }
        for (int j = 0;
             j < individual.location[i - numV].maxVisit;
             ++j) {
            if (!individual.location[i - numV].depots[j].route->used) {
                continue;
            }

            Node* node = &individual.location[i - numV].depots[j];
            Node* first = node->next;
            if (other.client[first->cour]
                    .route->whichLocation->cour !=
                node->route->whichLocation->cour) {
                distance += 10.0;
            }

            bool isFirst = true;
            while (isFirst || !node->next->isDepot) {
                Node& compared = other.client[node->next->cour];
                if (compared.pre->cour == node->cour ||
                    compared.next->cour == node->cour) {
                    distance += 1.0;
                } else if (
                    compared.route->cour == node->route->cour) {
                    distance += 3.0;
                } else if (
                    compared.route->whichLocation->cour !=
                    node->route->whichLocation->cour) {
                    distance += 10.0;
                }
                node = node->next;
                node = node->next;
                isFirst = false;
            }

            Node& compared = other.client[node->cour];
            if (compared.pre->cour == node->next->cour ||
                compared.next->cour == node->next->cour) {
                distance += 1.0;
            } else if (
                compared.route->cour == node->next->route->cour) {
                distance += 3.0;
            } else if (
                compared.route->whichLocation->cour !=
                node->next->route->whichLocation->cour) {
                distance += 10.0;
            }
        }
    }
    return -distance;
}

}  // namespace

int main() {
    read_data data;
    data.numV = 4;
    data.numL = 2;
    data.numC = 6;
    data.define();

    for (int row = 0; row < data.numC; ++row) {
        if (data.D[row] !=
            data.D[0] +
                static_cast<std::size_t>(row) *
                    static_cast<std::size_t>(data.numC)) {
            return fail("distance-matrix rows are not contiguous");
        }
    }

    for (int customer = 0; customer < data.numV; ++customer) {
        data.demCus[customer] = 1;
    }
    data.totalDemand = 4;
    data.vehCap = 4;
    for (int depot = 0; depot < data.numL; ++depot) {
        const int node = data.numV + depot;
        data.invLoc[node] = 10;
        data.openCostLoc[node] = 0.0;
    }
    data.usingCostVeh = 0.0;

    for (int from = 0; from < data.numC; ++from) {
        for (int to = 0; to < data.numC; ++to) {
            data.D[from][to] = from == to ? 0.0 : 1.0;
        }
    }

    Individual first;
    first.define(&data);
    first.rebuildFromSP(
        {makeRoute(0, {0, 1}), makeRoute(1, {2, 3})});

    Individual second;
    second.define(&data);
    second.rebuildFromSP(
        {makeRoute(0, {0, 2}), makeRoute(1, {1, 3})});

    const double expected = referenceBrokenPairsDistance(
        first, second, data.numV, data.numC);
    const double cached = first.brokenPairsDistance(&second);
    if (std::fabs(expected - cached) > 1.0e-9) {
        return fail(
            "compact diversity cache changed the distance: expected " +
            std::to_string(expected) + ", received " +
            std::to_string(cached));
    }
    if (std::fabs(cached - first.brokenPairsDistance(&second)) >
        1.0e-9) {
        return fail("reusing the diversity cache changed the distance");
    }

    // Rebuilding a solution must invalidate and refresh its compact cache.
    second.rebuildFromSP({makeRoute(0, {0, 1, 2, 3})});
    const double rebuiltExpected = referenceBrokenPairsDistance(
        first, second, data.numV, data.numC);
    const double rebuiltCached = first.brokenPairsDistance(&second);
    if (std::fabs(rebuiltExpected - rebuiltCached) > 1.0e-9) {
        return fail("diversity cache was stale after a route rebuild");
    }

    std::cout << "route_generator_memory_test: PASS" << std::endl;
    return 0;
}
