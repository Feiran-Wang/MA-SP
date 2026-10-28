// Author: Feiran
// Extract, deduplicate, and retain route columns for set partitioning.

#include "RoutePool.h"
#include "Individual.h"
#include <iostream>
#include <algorithm>

RoutePool::RoutePool(read_data *data) : data(data), maxPoolSize(data->spMaxPoolSize) {}

RoutePool::~RoutePool() {
    uniqueRoutes.clear();
    poolList.clear();
}

void RoutePool::extractRoutes(Individual *indiv) {

    for (int i = 0; i < data->numL; i++) {
        const NodeDepot &depot = indiv->location[i];

        if (depot.used) {

            for (int j = 0; j < depot.maxVisit; j++) {
                const Node &start = depot.depots[j];

                if (start.route->used &&
                    (data->hardTimeWindows ? !start.next->isDepot : start.route->load > 0)) {

                    StandaloneRoute candidate;
                    candidate.locId = i;
                    candidate.totalLoad = start.route->load;

                    candidate.routeCost = 0.0;
                    Node *current = start.next;
                    Node *previous = &depot.depots[j];

                    while (!current->isDepot) {
                        candidate.sequence.push_back(current->cour);

                        candidate.routeCost += data->D[previous->cour][current->cour];

                        previous = current;
                        current = current->next;
                    }

                    candidate.routeCost += data->D[previous->cour][current->cour];

                    if (data->hardTimeWindows &&
                        (candidate.totalLoad > data->vehCap ||
                         !data->routeTimeFeasible(candidate.locId, candidate.sequence)))
                        continue;
                    auto result = uniqueRoutes.insert(candidate);
                    if (result.second) {

                        poolList.push_back(candidate);
                    }
                }
            }
        }
    }

    if (poolList.size() > maxPoolSize) {
        prunePool();
    }
}

void RoutePool::prunePool() {
    std::cout << "[RoutePool] Pruning pool. Current size: " << poolList.size() << std::endl;

    auto compare = [](const StandaloneRoute &a, const StandaloneRoute &b) {
        double efficiencyA = (a.routeCost > 1e-6) ? (double)a.totalLoad / a.routeCost : 1e9;
        double efficiencyB = (b.routeCost > 1e-6) ? (double)b.totalLoad / b.routeCost : 1e9;
        return efficiencyA > efficiencyB;
    };

    std::sort(poolList.begin(), poolList.end(), compare);

    int targetSize = maxPoolSize * 0.8;
    poolList.resize(targetSize);

    uniqueRoutes.clear();
    for (const auto &sr : poolList) {
        uniqueRoutes.insert(sr);
    }

    std::cout << "[RoutePool] Pruning complete. New size: " << poolList.size() << std::endl;
}
