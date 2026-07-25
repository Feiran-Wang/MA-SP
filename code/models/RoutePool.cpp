/******************************************************************************
 * File: RoutePool.cpp
 * Author: Feiran Wang
 * Date: 2026-06-25
 * Description: 
 *   Central pool for deduplicating and retaining feasible routes.
 ******************************************************************************/

#include "RoutePool.h"
#include "Individual.h"
#include <iostream>
#include <algorithm>

RoutePool::RoutePool(read_data* data) {
    this->data = data;
    this->maxPoolSize = data->spMaxPoolSize;
}

RoutePool::~RoutePool() {
    uniqueRoutes.clear();
    poolList.clear();
}

void RoutePool::extractRoutes(const Individual* indiv) {
    // Callers admit only feasible individuals; each used non-empty route is
    // converted to the value representation shared with the SP solver.
    for (int i = 0; i < data->numL; i++) {
        if (indiv->location[i].used) {
            for (int j = 0; j < indiv->location[i].maxVisit; j++) {
                if (indiv->location[i].depots[j].route->used && 
                    indiv->location[i].depots[j].route->load > 0) {
                    
                    StandaloneRoute sr;
                    sr.locId = i;
                    sr.totalLoad = 0;
                    
                    // routeCost contains travel cost only. Vehicle and depot
                    // fixed costs are added by the SP objective.
                    sr.routeCost = 0.0;
                    const Node* current =
                        indiv->location[i].depots[j].next;
                    const Node* previous =
                        &indiv->location[i].depots[j];

                    while (!current->isDepot) {
                        sr.sequence.push_back(current->cour);
                        sr.totalLoad += current->dem;
                        // Preserve route direction for asymmetric instances.
                        sr.routeCost +=
                            data->D[previous->cour][current->cour];
                        
                        previous = current;
                        current = current->next;
                    }
                    sr.routeCost +=
                        data->D[previous->cour][current->cour];
                    
                    auto result = uniqueRoutes.insert(sr);
                    if (result.second) {
                        poolList.push_back(sr);
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
    
    auto compare = [](const StandaloneRoute& a, const StandaloneRoute& b) {
        // Zero-cost routes receive the highest retention score.
        double efficiencyA = (a.routeCost > 1e-6) ? (double)a.totalLoad / a.routeCost : 1e9;
        double efficiencyB = (b.routeCost > 1e-6) ? (double)b.totalLoad / b.routeCost : 1e9;
        return efficiencyA > efficiencyB; 
    };

    // Retain routes with the highest delivered-load/travel-cost ratio.
    std::sort(poolList.begin(), poolList.end(), compare);

    // Keep 80% of the configured limit to leave room for new routes.
    const std::size_t targetSize =
        std::max<std::size_t>(1, maxPoolSize * 4 / 5);
    poolList.resize(targetSize);

    // Rebuild uniqueRoutes from the retained pool entries.
    uniqueRoutes.clear();
    for (const auto& sr : poolList) {
        uniqueRoutes.insert(sr);
    }
    
    std::cout << "[RoutePool] Pruning complete. New size: " << poolList.size() << std::endl;
}

void RoutePool::printStatus() const {
    std::cout << "--- Route Pool Status ---" << std::endl;
    std::cout << "Unique Routes Count: " << uniqueRoutes.size() << std::endl;
    std::cout << "Vector Size:         " << poolList.size() << std::endl;
    std::cout << "-------------------------" << std::endl;
}
