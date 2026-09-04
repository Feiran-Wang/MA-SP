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
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include "ColumnRetentionPolicy.h"
#include "Route.h"
#include "RouteKey.h"
#include "RouteProvenance.h"
#include "SpTypes.h"
#include "SolverContext.h"

class Individual;

namespace hgasp {
struct ConfigurationCoverageSummary {
    bool completeCustomerCoverage = false;
    std::uint64_t signature = 0U;
    std::uint64_t version = 0U;
};
}

enum class RouteProtection {
    Backbone,
    GlobalIncumbent,
    LastSpIncumbent,
    PopulationElite,
};

// SP columns with the same depot and customer set have identical coefficients
// in every master constraint. Customer order deliberately does not belong to
// this key; the cheapest feasible order exactly dominates all dearer orders.
using MasterColumnKey = hgasp::RouteKey;
using MasterColumnKeyHash = hgasp::RouteKeyHash;

class RoutePool {
public:
    struct DepotReassignmentClosure {
        std::vector<StandaloneRoute> snapshot;
        std::size_t candidatesEvaluated = 0U;
        std::size_t columnsAppended = 0U;
        std::size_t representativesReplaced = 0U;
    };

    struct ExploratoryActiveView {
        std::vector<StandaloneRoute> active;
        std::size_t reserveCount = 0U;
        std::size_t mandatoryCount = 0U;
        std::size_t protectedCount = 0U;
        std::size_t lpSupportCount = 0U;
        bool reduced = false;
        bool dualFeedbackUsable = false;
    };

    struct FinalMasterDiagnostic {
        std::size_t activeColumnCount = 0U;
        std::size_t archivedColumnCount = 0U;
        std::size_t sameKeyActiveRepresentativeViolations = 0U;
        std::size_t sameKeyArchiveRepresentativeViolations = 0U;
        std::size_t dominanceReplacementCount = 0U;
        double dominanceReplacementTotalGain = 0.0;
        std::size_t finalIncumbentRoutesWithoutMatchingMasterKey = 0U;
        std::size_t finalIncumbentRoutesWithCheaperMatchingActiveKey = 0U;
        std::size_t finalIncumbentRoutesWithCheaperMatchingArchiveKey = 0U;
    };

    using SubmissionObserver = std::function<void(
        const Individual*,
        const std::vector<StandaloneRoute>&,
        const hgasp::RouteSubmissionContext&)>;

    RoutePool(
        SolverContext* data,
        std::size_t capacityOverride = 0U,
        std::optional<hgasp::DepotConfigurationKey>
            fixedConfiguration = std::nullopt);
    ~RoutePool();

    // Extract used routes from a feasible individual.
    void extractRoutes(const Individual* indiv);
    std::vector<StandaloneRoute> exportRoutes(
        const Individual* indiv) const;
    // Add the exact depot-assignment closure of a small set of route
    // sequences to one call-local global-SP snapshot.  The active RoutePool
    // and the HGA trajectory are not mutated.
    DepotReassignmentClosure buildDepotReassignmentClosure(
        std::vector<StandaloneRoute> snapshot,
        const std::vector<StandaloneRoute>& seedRoutes) const;
    // Build a call-local active MIP view after an LP over the full immutable
    // snapshot. Available aligned duals lead the deterministic ranking; when
    // the bounded LP has no usable dual solution, mandatory/protected columns,
    // coverage rarity, load efficiency and exact master identity provide the
    // fallback ranking. The reserve is the untouched complement; neither view
    // mutates central ownership or route-admission state.
    ExploratoryActiveView buildExploratoryActiveView(
        const std::vector<StandaloneRoute>& snapshot,
        const SpResult& lpResult,
        const std::vector<StandaloneRoute>& mandatoryRoutes,
        std::size_t targetSize) const;
    void setProtectedRoutes(
        RouteProtection protection,
        const std::vector<StandaloneRoute>& routes);
    std::vector<StandaloneRoute> protectedRoutes(
        RouteProtection protection) const;
    void pruneIfNeeded();
    std::size_t updateDualFeedback(
        const std::vector<StandaloneRoute>& snapshot,
        const std::vector<double>& customerDuals,
        const std::vector<double>& routeReducedCosts,
        bool allowEphemeralColumns = false);
    std::vector<hgasp::OriginCredit>
    applySpFeedbackAndCollectCredits(
        const std::vector<StandaloneRoute>& snapshot,
        const SpResult& result,
        int iteration,
        hgasp::SpFeedbackScope scope,
        std::optional<hgasp::DepotConfigurationKey>
            configuration = std::nullopt);
    const std::vector<double>& getCustomerDuals() const noexcept {
        return latestCustomerDuals;
    }
    double reducedCostFor(
        const StandaloneRoute& route) const noexcept;
    std::optional<hgasp::MasterColumnEvaluationState>
    columnEvaluationStateFor(
        const StandaloneRoute& route) const;
    std::size_t newColumnGraceCapacity() const noexcept {
        return hgasp::newColumnGraceCapacity(maxPoolSize);
    }

    // Submit a complete feasible solution to the route layer independently
    // from population admission. Every feasible HGA/SP candidate reaches this
    // boundary exactly once, even when it is later rejected as a population
    // duplicate or loses a resident-population replacement decision.
    void submitFeasibleIndividual(
        const Individual* indiv,
        const hgasp::RouteSubmissionContext& context =
            hgasp::RouteSubmissionContext{});
    // Reuse a caller's already validated route materialization. Population-
    // local pools use this boundary so central and local admission never
    // extract the same solution twice.
    void submitPreparedFeasibleIndividual(
        const Individual* indiv,
        const std::vector<StandaloneRoute>& routes,
        const hgasp::RouteSubmissionContext& context =
            hgasp::RouteSubmissionContext{});
    void setSubmissionObserver(SubmissionObserver observer) {
        submissionObserver = std::move(observer);
    }

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
    // Scheduling counters deliberately separate genuinely new master
    // columns from cheaper representatives of existing coefficients. They
    // are monotone and complement masterVersion, which also advances on
    // representative replacement and pruning.
    std::uint64_t getMasterAppendCount() const noexcept {
        return masterAppendCount;
    }
    std::uint64_t
    getMasterRepresentativeReplacementCount() const noexcept {
        return masterRepresentativeReplacementCount;
    }
    double getMasterRepresentativeReplacementCostGain() const noexcept {
        return masterRepresentativeReplacementCostGain;
    }
    std::uint64_t getConfigurationMasterVersion(
        const hgasp::DepotConfigurationKey& configuration) const;
    hgasp::ConfigurationCoverageSummary
    getConfigurationCoverageSummary(
        const hgasp::DepotConfigurationKey& configuration) const;
    std::uint64_t getConfigurationMasterAppendCount(
        const hgasp::DepotConfigurationKey& configuration) const;
    std::uint64_t getConfigurationMasterRepresentativeReplacementCount(
        const hgasp::DepotConfigurationKey& configuration) const;
    double getConfigurationMasterRepresentativeReplacementCostGain(
        const hgasp::DepotConfigurationKey& configuration) const;

    std::size_t getLastPruneProtectedCount() const noexcept {
        return lastPruneProtectedCount;
    }

    std::size_t getLastPruneRarityCount() const noexcept {
        return lastPruneRarityCount;
    }
    std::size_t getLastPruneGraceCount() const noexcept {
        return lastPruneGraceCount;
    }
    std::size_t getLastPruneDepotDiversityCount() const noexcept {
        return lastPruneDepotDiversityCount;
    }
    std::size_t getLastPruneOrdinaryCount() const noexcept {
        return lastPruneOrdinaryCount;
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
    std::size_t getProvenanceColumnCount() const noexcept {
        return columnProvenance.size();
    }
    std::size_t getArchivedProvenanceColumnCount() const noexcept {
        return archivedColumnProvenance.size();
    }
    std::size_t getLocalCreditBaselineCapacity() const noexcept {
        return localCreditBaselineCapacity;
    }
    std::uint64_t
    getLocalCreditBaselineEvictionCount() const noexcept {
        return localCreditBaselineEvictionCount;
    }
    FinalMasterDiagnostic summarizeFinalMasterAgainstIncumbent(
        const Individual& incumbent) const;

private:
    SolverContext* data;
    
    // Feasible route-source statistics. These are diagnostics only and never
    // gate route or population admission.
    double totalFitnessSum = 0.0;
    std::size_t feasibleSubmissionCount = 0U;

    // Ordered routes serve the HGA/search side, where route direction and
    // sequence remain genetic material.
    // Exact ordered identity -> already constructed canonical master key.
    // This replaces the former membership-only set: recurrence keeps the
    // same archive semantics while avoiding another sequence copy+sort.
    std::unordered_map<
        StandaloneRoute,
        MasterColumnKey,
        StandaloneRouteHash> uniqueSearchRouteKeys;
    std::vector<StandaloneRoute> searchArchive;
    // The ordered search archive is deliberately bounded, but forgetting an
    // overflow identity immediately caused every recurrence to cross the
    // considerably more expensive master-admission boundary again.  Keep an
    // equally bounded exact FIFO sidecar for discarded ordered identities.
    // This is only an admission memo: callers still restore a master key when
    // pruning removed it from the active pool.
    std::unordered_map<
        StandaloneRoute,
        MasterColumnKey,
        StandaloneRouteHash> overflowSearchRouteKeys;
    std::deque<StandaloneRoute> overflowSearchRouteFifo;
    std::unordered_map<
        MasterColumnKey,
        StandaloneRoute,
        MasterColumnKeyHash> bestArchivedColumns;
    
    // Compressed master columns serve SP. The map points to stable vector
    // positions until pruning, after which it is rebuilt.
    std::vector<StandaloneRoute> poolList;
    // Exact canonical keys are kept in lockstep with poolList. This bounded
    // sidecar avoids repeatedly copying and sorting every route sequence
    // during pruning and index rebuilds; it is never used without exact key
    // equality in the owning maps/sets.
    std::vector<MasterColumnKey> poolMasterKeys;
    std::unordered_map<
        MasterColumnKey,
        std::size_t,
        MasterColumnKeyHash> masterColumnIndex;

    std::size_t dominanceReplacementCount = 0U;
    std::size_t dominanceDiscardCount = 0U;
    std::size_t searchArchiveDiscardCount = 0U;
    std::size_t bestArchiveDiscardCount = 0U;
    std::uint64_t masterVersion = 0U;
    // Depot-scoped versions advance on admission, representative replacement,
    // and pruning that removes columns belonging to the depot.
    std::vector<std::uint64_t> depotMasterVersions;
    // Active-master coverage is maintained by canonical key, independently
    // of the representative route order stored in poolList.
    std::vector<std::vector<std::uint32_t>>
        depotCustomerColumnCounts;
    std::vector<std::uint64_t> depotMasterKeySignatures;
    std::uint64_t masterAppendCount = 0U;
    std::uint64_t masterRepresentativeReplacementCount = 0U;
    double masterRepresentativeReplacementCostGain = 0.0;
    std::vector<std::uint64_t> depotMasterAppendCounts;
    std::vector<std::uint64_t>
        depotMasterRepresentativeReplacementCounts;
    std::vector<double>
        depotMasterRepresentativeReplacementCostGains;
    std::vector<double> latestCustomerDuals;
    std::unordered_map<
        MasterColumnKey,
        double,
        MasterColumnKeyHash> latestReducedCosts;
    std::unordered_map<
        MasterColumnKey,
        hgasp::MasterColumnEvaluationState,
        MasterColumnKeyHash> columnEvaluationStates;
    std::unordered_map<
        MasterColumnKey,
        hgasp::ColumnProvenanceStats,
        MasterColumnKeyHash> columnProvenance;
    std::unordered_map<
        MasterColumnKey,
        hgasp::ArchivedChallengerColumnProvenance,
        MasterColumnKeyHash> archivedColumnProvenance;
    // Retain the formal V1.17.2 bounded LRU behavior independently of the
    // removed dynamic-island configuration surface.
    std::size_t localCreditBaselineCapacity = 6U;
    std::uint64_t creditFeedbackSequence = 0U;
    std::uint64_t localCreditBaselineEvictionCount = 0U;

    using ProtectedKeySet = std::unordered_set<
        MasterColumnKey,
        MasterColumnKeyHash>;
    ProtectedKeySet backboneKeys;
    ProtectedKeySet globalIncumbentKeys;
    ProtectedKeySet lastSpIncumbentKeys;
    ProtectedKeySet populationEliteKeys;
    std::size_t lastPruneProtectedCount = 0U;
    std::size_t lastPruneRarityCount = 0U;
    std::size_t lastPruneGraceCount = 0U;
    std::size_t lastPruneDepotDiversityCount = 0U;
    std::size_t lastPruneOrdinaryCount = 0U;

    struct PruneMetric {
        std::size_t index = 0U;
        double reducedCost = 0.0;
        double efficiency = 0.0;
        double rarity = 0.0;
        double routeCost = 0.0;
        int depot = -1;
        bool protectedColumn = false;
        bool hasEvaluationState = false;
        bool lpEvaluated = false;
        std::uint64_t insertedMasterVersion = 0U;
    };
    // Pruning is called thousands of times in large runs. These buffers are
    // owned by one RoutePool and reused serially; they change allocation
    // behavior only, never column ranking or retention order.
    std::vector<int> pruneAvailabilityScratch;
    std::vector<int> pruneRetainedCoverageScratch;
    std::vector<std::uint8_t> pruneRetainedScratch;
    std::vector<std::size_t> pruneCandidateScratch;
    std::vector<std::size_t> pruneRetainedIndexScratch;
    std::vector<PruneMetric> pruneMetricScratch;

    // Configured pool-size bound.
    std::size_t maxPoolSize;
    std::optional<hgasp::DepotConfigurationKey> fixedConfiguration;
    SubmissionObserver submissionObserver;

    // Retain the highest-scoring routes when the bound is exceeded.
    void prunePool();

    static MasterColumnKey masterKey(const StandaloneRoute& route);
    static std::uint64_t stableMasterKeyDigest(
        const MasterColumnKey& key) noexcept;
    void addActiveMasterKey(const MasterColumnKey& key);
    void removeActiveMasterKey(const MasterColumnKey& key);
    static bool preferredMasterRoute(
        const StandaloneRoute& candidate,
        const StandaloneRoute& incumbent);
    enum class MasterAdmission {
        Discarded,
        NewColumn,
        DominanceReplacement,
    };
    MasterAdmission admitMasterColumn(
        const StandaloneRoute& route,
        const MasterColumnKey& key,
        const hgasp::RouteSubmissionContext* context = nullptr);
    const MasterColumnKey* archivedOrderedMasterKey(
        const StandaloneRoute& route) const noexcept;
    bool archiveOrderedRoute(
        const StandaloneRoute& route,
        const MasterColumnKey& key);
    StandaloneRoute archivedBestOrCandidate(
        const MasterColumnKey& key,
        const StandaloneRoute& candidate);
    void rebuildMasterIndex();
    void pruneColumnSidecars();
    bool isProtected(const MasterColumnKey& key) const;
    ProtectedKeySet& protectionSet(RouteProtection protection);
    void recordContributor(
        const MasterColumnKey& key,
        const hgasp::RouteSubmissionContext& context,
        bool bestOrigin);
    void restoreArchivedChallengerProvenance(
        const MasterColumnKey& key);
    double& usefulScoreBaseline(
        hgasp::ColumnProvenanceStats& stats,
        hgasp::SpFeedbackScope scope,
        const std::optional<hgasp::DepotConfigurationKey>&
            configuration);
};

#endif /* ROUTEPOOL_H_ */
