/******************************************************************************
 * File: RoutePool.h
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description: 
 *   Central pool for deduplicating and retaining feasible routes.
 ******************************************************************************/

#ifndef ROUTEPOOL_H_
#define ROUTEPOOL_H_

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "Route.h"
#include "read_data.h"

class Individual;

enum class RouteProtection {
    Backbone,
    GlobalIncumbent,
    LastSpIncumbent,
    PopulationElite,
};

// SP columns with the same depot and customer set have identical coefficients
// in every master constraint. Customer order deliberately does not belong to
// this key; the cheapest feasible order exactly dominates all dearer orders.
struct MasterColumnKey {
    int locId = -1;
    std::vector<int> customers;

    bool operator==(const MasterColumnKey& other) const noexcept {
        return locId == other.locId && customers == other.customers;
    }
};

struct MasterColumnKeyHash {
    std::size_t operator()(const MasterColumnKey& key) const noexcept;
};

class RoutePool {
public:
    RoutePool(read_data* data);
    ~RoutePool();

    // Extract used routes from a feasible individual.
    void extractRoutes(const Individual* indiv);
    std::vector<StandaloneRoute> exportRoutes(
        const Individual* indiv) const;
    void setProtectedRoutes(
        RouteProtection protection,
        const std::vector<StandaloneRoute>& routes);
    void pruneIfNeeded();
    void updateDualFeedback(
        const std::vector<StandaloneRoute>& snapshot,
        const std::vector<double>& customerDuals,
        const std::vector<double>& routeReducedCosts);
    const std::vector<double>& getCustomerDuals() const noexcept {
        return latestCustomerDuals;
    }
    double reducedCostFor(
        const StandaloneRoute& route) const noexcept;

    // Submit a complete feasible solution to the route layer independently
    // from population admission. Every feasible HGA/SP candidate reaches this
    // boundary exactly once, even when it is later rejected as a population
    // duplicate or loses a resident-population replacement decision.
    void submitFeasibleIndividual(const Individual* indiv);

    // Return the exact-dominance-compressed columns consumed by SP.
    const std::vector<StandaloneRoute>& getPool() const { return poolList; }

    // Return every distinct ordered route retained for HGA/search diagnostics.
    const std::vector<StandaloneRoute>& getSearchArchive() const {
        return searchArchive;
    }

    // Print current pool statistics.
    void printStatus() const;

    double getAverageFitness() const {
        if (feasibleSubmissionCount == 0) return 1e30;
        return totalFitnessSum /
            static_cast<double>(feasibleSubmissionCount);
    }

    std::size_t getFeasibleSubmissionCount() const noexcept {
        return feasibleSubmissionCount;
    }

    std::size_t getDominanceReplacementCount() const noexcept {
        return dominanceReplacementCount;
    }

    std::size_t getDominanceDiscardCount() const noexcept {
        return dominanceDiscardCount;
    }

    std::uint64_t getMasterVersion() const noexcept {
        return masterVersion;
    }

    std::size_t getLastPruneProtectedCount() const noexcept {
        return lastPruneProtectedCount;
    }

    std::size_t getLastPruneRarityCount() const noexcept {
        return lastPruneRarityCount;
    }
    std::size_t getBestArchivedColumnCount() const noexcept {
        return bestArchivedColumns.size();
    }
    std::size_t getSearchArchiveDiscardCount() const noexcept {
        return searchArchiveDiscardCount;
    }
    std::size_t getBestArchiveDiscardCount() const noexcept {
        return bestArchiveDiscardCount;
    }

private:
    read_data* data;
    
    // Feasible route-source statistics. These are diagnostics only and never
    // gate route or population admission.
    double totalFitnessSum = 0.0;
    std::size_t feasibleSubmissionCount = 0U;

    // Ordered routes serve the HGA/search side, where route direction and
    // sequence remain genetic material.
    std::unordered_set<StandaloneRoute, StandaloneRouteHash>
        uniqueSearchRoutes;
    std::vector<StandaloneRoute> searchArchive;
    std::unordered_map<
        MasterColumnKey,
        StandaloneRoute,
        MasterColumnKeyHash> bestArchivedColumns;
    
    // Compressed master columns serve SP. The map points to stable vector
    // positions until pruning, after which it is rebuilt.
    std::vector<StandaloneRoute> poolList;
    std::unordered_map<
        MasterColumnKey,
        std::size_t,
        MasterColumnKeyHash> masterColumnIndex;

    std::size_t dominanceReplacementCount = 0U;
    std::size_t dominanceDiscardCount = 0U;
    std::size_t searchArchiveDiscardCount = 0U;
    std::size_t bestArchiveDiscardCount = 0U;
    std::uint64_t masterVersion = 0U;
    std::vector<double> latestCustomerDuals;
    std::unordered_map<
        MasterColumnKey,
        double,
        MasterColumnKeyHash> latestReducedCosts;

    using ProtectedKeySet = std::unordered_set<
        MasterColumnKey,
        MasterColumnKeyHash>;
    ProtectedKeySet backboneKeys;
    ProtectedKeySet globalIncumbentKeys;
    ProtectedKeySet lastSpIncumbentKeys;
    ProtectedKeySet populationEliteKeys;
    std::size_t lastPruneProtectedCount = 0U;
    std::size_t lastPruneRarityCount = 0U;
    std::size_t lastPruneDepotDiversityCount = 0U;
    std::size_t lastPruneOrdinaryCount = 0U;

    // Configured pool-size bound.
    std::size_t maxPoolSize;

    // Retain the highest-scoring routes when the bound is exceeded.
    void prunePool();

    static MasterColumnKey masterKey(const StandaloneRoute& route);
    static bool preferredMasterRoute(
        const StandaloneRoute& candidate,
        const StandaloneRoute& incumbent);
    void admitMasterColumn(const StandaloneRoute& route);
    bool archiveOrderedRoute(const StandaloneRoute& route);
    StandaloneRoute archivedBestOrCandidate(
        const MasterColumnKey& key,
        const StandaloneRoute& candidate);
    void rebuildMasterIndex();
    bool isProtected(const MasterColumnKey& key) const;
    ProtectedKeySet& protectionSet(RouteProtection protection);
};

#endif /* ROUTEPOOL_H_ */
