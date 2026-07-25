/******************************************************************************
 * File: RoutePool.h
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description: 
 *   Central pool for deduplicating and retaining feasible routes.
 ******************************************************************************/

#ifndef ROUTEPOOL_H_
#define ROUTEPOOL_H_

#include <vector>
#include <unordered_set>
#include "Route.h"
#include "read_data.h"

class Individual;

class RoutePool {
public:
    RoutePool(read_data* data);
    ~RoutePool();

    // Extract used routes from a feasible individual.
    void extractRoutes(const Individual* indiv);

    // Return the current deduplicated route snapshot.
    const std::vector<StandaloneRoute>& getPool() const { return poolList; }

    // Print current pool statistics.
    void printStatus() const;

    // Track the mean objective used by the admission rule.
    void recordSolutionFitness(double fitness) {
        totalFitnessSum += fitness;
        solutionCount++;
    }

    double getAverageFitness() const {
        if (solutionCount == 0) return 1e30;
        return totalFitnessSum / (double)solutionCount;
    }

private:
    read_data* data;
    
    // Admission statistics.
    double totalFitnessSum = 0.0;
    int solutionCount = 0;

    // Hash index for duplicate detection.
    std::unordered_set<StandaloneRoute, StandaloneRouteHash> uniqueRoutes;
    
    // Stable contiguous view consumed by the SP solver.
    std::vector<StandaloneRoute> poolList;

    // Configured pool-size bound.
    std::size_t maxPoolSize;

    // Retain the highest-scoring routes when the bound is exceeded.
    void prunePool();
};

#endif /* ROUTEPOOL_H_ */
