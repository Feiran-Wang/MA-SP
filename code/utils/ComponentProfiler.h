#ifndef HGASP_COMPONENT_PROFILER_H_
#define HGASP_COMPONENT_PROFILER_H_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <vector>

namespace hgasp {

// Fixed identifiers keep hot-path profiling allocation-free and make the
// emitted schema stable across runs. Timings are collected on the algorithm
// control thread; Gurobi worker CPU is nevertheless included by std::clock().
enum class TimedComponent : std::size_t {
    InstanceLoading = 0,
    AlgorithmSetup,
    DepotConfigurationPreprocess,
    InitialPopulation,
    ParentAndPopulationSelection,
    EaxCrossover,
    ConstraintFeasibleReconstruction,
    CapacityRepair,
    Mutation,
    CapacityRouteLocalSearch,
    HardTwRouteLocalSearch,
    RoutePoolAdmission,
    RoutePoolPruning,
    PopulationAdmission,
    SearchControl,
    SpLocalGlobal,
    SpInjection,
    Count,
};

struct ComponentTiming {
    std::uint64_t calls = 0U;
    double inclusiveCpuSeconds = 0.0;
    double exclusiveCpuSeconds = 0.0;
    double inclusiveWallSeconds = 0.0;
    double exclusiveWallSeconds = 0.0;
    double maximumInclusiveCpuSeconds = 0.0;
    double maximumInclusiveWallSeconds = 0.0;
};

// Fixed diagnostic work counters explain where a semantic-preserving
// optimization saves work. Calls are compiled out unless the dedicated CMake
// option is enabled, so publication Release timing never pays this overhead.
enum class HotPathCounter : std::size_t {
    EvaluatorCheckedCalls = 0,
    EvaluatorTrustedCalls,
    EvaluatorCustomersScanned,
    EvaluatorFullScheduleCalls,
    RouteLimitRequests,
    RouteLimitFullScans,
    RouteLimitCacheHits,
    RouteLimitDemandEntriesScanned,
    LsCapacityCalls,
    LsHardTwCalls,
    LsZeroAcceptCalls,
	LsCandidateMoves,
	LsAcceptedMoves,
	LsCandidateM1,
	LsCandidateM2,
	LsCandidateM3,
	LsCandidateM4,
	LsCandidateM5,
	LsCandidateM6,
	LsCandidateM7,
	LsCandidateM8,
	LsCandidateM9,
	LsCandidateM10,
	LsCandidateSplitPrefix,
	LsCandidateSplitSuffix,
	LsAcceptedM1,
	LsAcceptedM2,
	LsAcceptedM3,
	LsAcceptedM4,
	LsAcceptedM5,
	LsAcceptedM6,
	LsAcceptedM7,
	LsAcceptedM8,
	LsAcceptedM9,
	LsAcceptedM10,
	LsAcceptedSplitPrefix,
	LsAcceptedSplitSuffix,
	LsLegacyResetCells,
    LsFullSolutionEvaluations,
    LsSummaryRebuilds,
	LsCanonicalPairCacheQueries,
	LsCanonicalPairCacheHits,
	LsDirectedPairFailureSkips,
    ReconstructionBuilds,
    ReconstructionInsertionUnits,
    ReconstructionCandidateSlots,
    ReconstructionSequenceMaterializations,
    ReconstructionRouteEvaluations,
    ReconstructionFeasibleCandidates,
    ReconstructionSortedCandidates,
    ReconstructionGuidedFallbacks,
    PopulationAdmissionCalls,
    PopulationValidationCalls,
    PopulationValidationCacheHits,
    PopulationCopyCalls,
    PopulationActiveRoutesCopied,
    PopulationEmptySlotsInitialized,
    PopulationStructureKeyBuilds,
    PopulationObjectAllocations,
    PopulationObjectReuses,
    RoutePoolSubmissions,
    RoutePoolRoutesEvaluated,
    RoutePoolMasterKeyBuilds,
    RoutePoolMasterKeyCustomerItems,
    RoutePoolPruneCalls,
    RoutePoolPruneColumnsScanned,
    RoutePoolEvaluationCertificateHits,
    RoutePoolEvaluationFallbacks,
	RoutePoolMasterKeyCacheHits,
	RoutePoolMasterKeyCacheEvictions,
	RoutePoolProvenanceNoopSkips,
    RepairCalls,
    RepairFastPathCalls,
    RepairSearchCalls,
    RepairChangedCalls,
    RepairRejectedCalls,
	RepairLegacyTabuResetCells,
	RepairTabuResetCells,
	RepairRouteTraversalRequests,
	RepairLegacyRouteSlotsScanned,
	RepairActiveRoutesScanned,
	LsDepotCandidateResolutionRequests,
	LsDepotCandidateResolutionBuilds,
	ActiveRouteViewRequests,
	ActiveRouteViewRebuilds,
	ActiveRouteViewCustomersScanned,
	Count,
};

class HotPathCounters {
public:
    static HotPathCounters& instance() noexcept;

    void reset() noexcept;
    void add(HotPathCounter counter, std::uint64_t amount = 1U) noexcept;
    std::uint64_t value(HotPathCounter counter) const noexcept;
    std::array<
        std::uint64_t,
        static_cast<std::size_t>(HotPathCounter::Count)>
    snapshot() const noexcept;

private:
    std::array<
        std::uint64_t,
        static_cast<std::size_t>(HotPathCounter::Count)> counters_{};
};

constexpr bool hotPathCountersCompiled() noexcept {
#if defined(HGASP_ENABLE_HOT_PATH_COUNTERS)
    return true;
#else
    return false;
#endif
}

inline void recordHotPathCounter(
        const HotPathCounter counter,
        const std::uint64_t amount = 1U) noexcept {
#if defined(HGASP_ENABLE_HOT_PATH_COUNTERS)
    HotPathCounters::instance().add(counter, amount);
#else
    (void)counter;
    (void)amount;
#endif
}

class ComponentProfiler {
public:
    static ComponentProfiler& instance() noexcept;

    void configureFromEnvironment() noexcept;
    void reset(
        bool enabled = true,
        std::uint64_t snapshotIntervalAttempts = 10000U) noexcept;
    bool enabled() const noexcept;
    std::uint64_t snapshotIntervalAttempts() const noexcept;
    bool shouldEmitSnapshot(std::uint64_t iteration) const noexcept;
    void record(
        TimedComponent component,
        double inclusiveCpuSeconds,
        double exclusiveCpuSeconds,
        double inclusiveWallSeconds,
        double exclusiveWallSeconds) noexcept;
    ComponentTiming timing(TimedComponent component) const noexcept;
    std::vector<ComponentTiming> snapshot() const;

private:
    std::array<
        ComponentTiming,
        static_cast<std::size_t>(TimedComponent::Count)> timings_{};
    bool enabled_ = true;
    std::uint64_t snapshotIntervalAttempts_ = 10000U;
};

class ScopedComponentTimer {
public:
    explicit ScopedComponentTimer(TimedComponent component);
    ~ScopedComponentTimer();

    ScopedComponentTimer(const ScopedComponentTimer&) = delete;
    ScopedComponentTimer& operator=(const ScopedComponentTimer&) = delete;

private:
    TimedComponent component_;
    std::clock_t cpuStarted_ = 0;
    std::chrono::steady_clock::time_point wallStarted_{};
    double nestedCpuSeconds_ = 0.0;
    double nestedWallSeconds_ = 0.0;
    bool active_ = false;

    static thread_local std::vector<ScopedComponentTimer*> stack_;
};

const char* timedComponentName(TimedComponent component) noexcept;
const char* timedComponentScope(TimedComponent component) noexcept;
const char* routeLocalSearchComponentName(
    bool hardTimeWindows) noexcept;

// Periodic snapshots contain compact cumulative exclusive CPU totals in one
// event. Final output adds one detailed event per active component followed by
// an auditable completion marker.
void emitComponentTimingSnapshot(
    std::uint64_t iteration,
    double searchCpuSeconds) noexcept;
void emitComponentTimingFinal(
    std::uint64_t iteration,
    double searchCpuSeconds,
    bool completed) noexcept;
// Mirrors the cumulative diagnostic work counters at the same low-frequency
// attempt checkpoints as component timing. This preserves useful evidence
// when a long run is interrupted before the terminal event is emitted.
void emitHotPathCountersSnapshot(std::uint64_t iteration) noexcept;
void emitHotPathCountersFinal(bool completed) noexcept;

}  // namespace hgasp

#endif  // HGASP_COMPONENT_PROFILER_H_
