// Author: Feiran
// Define standalone route identity and the bounded route-pool interface.

#ifndef ROUTEPOOL_H_
#define ROUTEPOOL_H_

#include <vector>
#include <unordered_set>
#include "read_data.h"

class Individual;

struct StandaloneRoute {
    int locId;
    // Travel cost excludes vehicle and depot opening charges, which the master adds separately.
    double routeCost;
    int totalLoad;
    std::vector<int> sequence;

    // Depot and visit order define identity; cost and load are checked separately.
    bool operator==(const StandaloneRoute &other) const {
        if (locId != other.locId)
            return false;
        if (sequence.size() != other.sequence.size())
            return false;
        for (size_t i = 0; i < sequence.size(); ++i) {
            if (sequence[i] != other.sequence[i])
                return false;
        }
        return true;
    }
};

struct StandaloneRouteHash {
    std::size_t operator()(const StandaloneRoute &r) const {
        std::size_t seed = r.sequence.size();
        seed ^= std::hash<int>()(r.locId) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        for (int node : r.sequence) {
            seed ^= std::hash<int>()(node) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

class RoutePool {
  public:
    RoutePool(read_data *data);
    ~RoutePool();

    void extractRoutes(Individual *indiv);

    const std::vector<StandaloneRoute> &getPool() const {
        return poolList;
    }

    void recordSolutionFitness(double fitness) {
        totalFitnessSum += fitness;
        solutionCount++;
    }

    double getAverageFitness() const {
        if (solutionCount == 0)
            return 1e30;
        return totalFitnessSum / (double)solutionCount;
    }

  private:
    read_data *data;

    double totalFitnessSum = 0.0;
    int solutionCount = 0;

    std::unordered_set<StandaloneRoute, StandaloneRouteHash> uniqueRoutes;

    std::vector<StandaloneRoute> poolList;

    std::size_t maxPoolSize;

    void prunePool();
};

#endif
