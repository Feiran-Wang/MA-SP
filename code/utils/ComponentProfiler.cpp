#include "ComponentProfiler.h"

#include "RunLogger.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <string>
#include <utility>

namespace hgasp {

namespace {

constexpr std::size_t componentCount =
    static_cast<std::size_t>(TimedComponent::Count);
constexpr std::size_t hotPathCounterCount =
    static_cast<std::size_t>(HotPathCounter::Count);

#if defined(HGASP_ENABLE_HOT_PATH_COUNTERS)
struct HotPathCounterDescriptor {
    HotPathCounter counter;
    const char* group;
    const char* name;
};

constexpr std::array<HotPathCounterDescriptor, hotPathCounterCount>
    hotPathCounterDescriptors{{
        {HotPathCounter::EvaluatorCheckedCalls, "evaluator", "checked_calls"},
        {HotPathCounter::EvaluatorTrustedCalls, "evaluator", "trusted_calls"},
        {HotPathCounter::EvaluatorCustomersScanned, "evaluator", "customers_scanned"},
        {HotPathCounter::EvaluatorFullScheduleCalls, "evaluator", "full_schedule_calls"},
        {HotPathCounter::RouteLimitRequests, "route_limit", "requests"},
        {HotPathCounter::RouteLimitFullScans, "route_limit", "full_scans"},
        {HotPathCounter::RouteLimitCacheHits, "route_limit", "cache_hits"},
        {HotPathCounter::RouteLimitDemandEntriesScanned, "route_limit", "demand_entries_scanned"},
        {HotPathCounter::LsCapacityCalls, "local_search", "capacity_calls"},
        {HotPathCounter::LsHardTwCalls, "local_search", "hard_tw_calls"},
        {HotPathCounter::LsZeroAcceptCalls, "local_search", "zero_accept_calls"},
        {HotPathCounter::LsCandidateMoves, "local_search", "candidate_moves"},
        {HotPathCounter::LsAcceptedMoves, "local_search", "accepted_moves"},
		{HotPathCounter::LsCandidateM1, "local_search", "candidate_m1"},
		{HotPathCounter::LsCandidateM2, "local_search", "candidate_m2"},
		{HotPathCounter::LsCandidateM3, "local_search", "candidate_m3"},
		{HotPathCounter::LsCandidateM4, "local_search", "candidate_m4"},
		{HotPathCounter::LsCandidateM5, "local_search", "candidate_m5"},
		{HotPathCounter::LsCandidateM6, "local_search", "candidate_m6"},
		{HotPathCounter::LsCandidateM7, "local_search", "candidate_m7"},
		{HotPathCounter::LsCandidateM8, "local_search", "candidate_m8"},
		{HotPathCounter::LsCandidateM9, "local_search", "candidate_m9"},
		{HotPathCounter::LsCandidateM10, "local_search", "candidate_m10"},
		{HotPathCounter::LsCandidateSplitPrefix, "local_search", "candidate_split_prefix"},
		{HotPathCounter::LsCandidateSplitSuffix, "local_search", "candidate_split_suffix"},
		{HotPathCounter::LsAcceptedM1, "local_search", "accepted_m1"},
		{HotPathCounter::LsAcceptedM2, "local_search", "accepted_m2"},
		{HotPathCounter::LsAcceptedM3, "local_search", "accepted_m3"},
		{HotPathCounter::LsAcceptedM4, "local_search", "accepted_m4"},
		{HotPathCounter::LsAcceptedM5, "local_search", "accepted_m5"},
		{HotPathCounter::LsAcceptedM6, "local_search", "accepted_m6"},
		{HotPathCounter::LsAcceptedM7, "local_search", "accepted_m7"},
		{HotPathCounter::LsAcceptedM8, "local_search", "accepted_m8"},
		{HotPathCounter::LsAcceptedM9, "local_search", "accepted_m9"},
		{HotPathCounter::LsAcceptedM10, "local_search", "accepted_m10"},
		{HotPathCounter::LsAcceptedSplitPrefix, "local_search", "accepted_split_prefix"},
		{HotPathCounter::LsAcceptedSplitSuffix, "local_search", "accepted_split_suffix"},
        {HotPathCounter::LsLegacyResetCells, "local_search", "legacy_reset_cells"},
        {HotPathCounter::LsFullSolutionEvaluations, "local_search", "full_solution_evaluations"},
        {HotPathCounter::LsSummaryRebuilds, "local_search", "summary_rebuilds"},
        {HotPathCounter::LsCanonicalPairCacheQueries, "local_search", "canonical_pair_cache_queries"},
        {HotPathCounter::LsCanonicalPairCacheHits, "local_search", "canonical_pair_cache_hits"},
		{HotPathCounter::LsDirectedPairFailureSkips, "local_search", "directed_pair_failure_skips"},
        {HotPathCounter::ReconstructionBuilds, "reconstruction", "builds"},
        {HotPathCounter::ReconstructionInsertionUnits, "reconstruction", "insertion_units"},
        {HotPathCounter::ReconstructionCandidateSlots, "reconstruction", "candidate_slots"},
        {HotPathCounter::ReconstructionSequenceMaterializations, "reconstruction", "sequence_materializations"},
        {HotPathCounter::ReconstructionRouteEvaluations, "reconstruction", "route_evaluations"},
        {HotPathCounter::ReconstructionFeasibleCandidates, "reconstruction", "feasible_candidates"},
        {HotPathCounter::ReconstructionSortedCandidates, "reconstruction", "sorted_candidates"},
        {HotPathCounter::ReconstructionGuidedFallbacks, "reconstruction", "guided_fallbacks"},
        {HotPathCounter::PopulationAdmissionCalls, "population", "admission_calls"},
        {HotPathCounter::PopulationValidationCalls, "population", "validation_calls"},
        {HotPathCounter::PopulationValidationCacheHits, "population", "validation_cache_hits"},
        {HotPathCounter::PopulationCopyCalls, "population", "copy_calls"},
        {HotPathCounter::PopulationActiveRoutesCopied, "population", "active_routes_copied"},
        {HotPathCounter::PopulationEmptySlotsInitialized, "population", "empty_slots_initialized"},
        {HotPathCounter::PopulationStructureKeyBuilds, "population", "structure_key_builds"},
        {HotPathCounter::PopulationObjectAllocations, "population", "object_allocations"},
        {HotPathCounter::PopulationObjectReuses, "population", "object_reuses"},
        {HotPathCounter::RoutePoolSubmissions, "route_pool", "submissions"},
        {HotPathCounter::RoutePoolRoutesEvaluated, "route_pool", "routes_evaluated"},
        {HotPathCounter::RoutePoolMasterKeyBuilds, "route_pool", "master_key_builds"},
        {HotPathCounter::RoutePoolMasterKeyCustomerItems, "route_pool", "master_key_customer_items"},
        {HotPathCounter::RoutePoolPruneCalls, "route_pool", "prune_calls"},
        {HotPathCounter::RoutePoolPruneColumnsScanned, "route_pool", "prune_columns_scanned"},
        {HotPathCounter::RoutePoolEvaluationCertificateHits, "route_pool", "evaluation_certificate_hits"},
        {HotPathCounter::RoutePoolEvaluationFallbacks, "route_pool", "evaluation_fallbacks"},
        {HotPathCounter::RoutePoolMasterKeyCacheHits, "route_pool", "master_key_cache_hits"},
        {HotPathCounter::RoutePoolMasterKeyCacheEvictions, "route_pool", "master_key_cache_evictions"},
		{HotPathCounter::RoutePoolProvenanceNoopSkips, "route_pool", "provenance_noop_skips"},
        {HotPathCounter::RepairCalls, "capacity_repair", "calls"},
        {HotPathCounter::RepairFastPathCalls, "capacity_repair", "fast_path_calls"},
        {HotPathCounter::RepairSearchCalls, "capacity_repair", "search_calls"},
        {HotPathCounter::RepairChangedCalls, "capacity_repair", "changed_calls"},
        {HotPathCounter::RepairRejectedCalls, "capacity_repair", "rejected_calls"},
        {HotPathCounter::RepairLegacyTabuResetCells, "capacity_repair", "legacy_tabu_reset_cells"},
        {HotPathCounter::RepairTabuResetCells, "capacity_repair", "tabu_reset_cells"},
		{HotPathCounter::RepairRouteTraversalRequests, "capacity_repair", "route_traversal_requests"},
		{HotPathCounter::RepairLegacyRouteSlotsScanned, "capacity_repair", "legacy_route_slots_scanned"},
		{HotPathCounter::RepairActiveRoutesScanned, "capacity_repair", "active_routes_scanned"},
        {HotPathCounter::LsDepotCandidateResolutionRequests, "local_search", "depot_candidate_resolution_requests"},
        {HotPathCounter::LsDepotCandidateResolutionBuilds, "local_search", "depot_candidate_resolution_builds"},
		{HotPathCounter::ActiveRouteViewRequests, "representation", "active_route_view_requests"},
		{HotPathCounter::ActiveRouteViewRebuilds, "representation", "active_route_view_rebuilds"},
		{HotPathCounter::ActiveRouteViewCustomersScanned, "representation", "active_route_view_customers_scanned"},
    }};

constexpr std::array<const char*, 8U> hotPathCounterGroups{{
    "evaluator",
    "route_limit",
    "local_search",
    "reconstruction",
    "population",
    "route_pool",
	"capacity_repair",
	"representation",
}};
#endif

bool configuredBoolean(
        const char* const name,
        const bool fallback) noexcept {
    const char* raw = std::getenv(name);
    if (raw == nullptr || *raw == '\0') {
        return fallback;
    }
    const std::string value(raw);
    if (value == "1" || value == "true" || value == "on") {
        return true;
    }
    if (value == "0" || value == "false" || value == "off") {
        return false;
    }
    return fallback;
}

std::uint64_t configuredInterval() noexcept {
    const char* raw =
        std::getenv("HGASP_COMPONENT_TIMING_INTERVAL");
    if (raw == nullptr || *raw == '\0') {
        return 10000U;
    }
    errno = 0;
    char* end = nullptr;
    const unsigned long long value =
        std::strtoull(raw, &end, 10);
    if (errno != 0 || end == raw || *end != '\0') {
        return 10000U;
    }
    return std::min<std::uint64_t>(
        static_cast<std::uint64_t>(value),
        1000000000U);
}

double nonnegativeElapsed(
        const std::clock_t start,
        const std::clock_t finish) noexcept {
    if (finish < start) {
        return 0.0;
    }
    return static_cast<double>(finish - start) /
        static_cast<double>(CLOCKS_PER_SEC);
}

double nonnegativeWall(
        const std::chrono::steady_clock::time_point start,
        const std::chrono::steady_clock::time_point finish) noexcept {
    return std::max(
        0.0,
        std::chrono::duration<double>(finish - start).count());
}

}  // namespace

thread_local std::vector<ScopedComponentTimer*>
    ScopedComponentTimer::stack_;

ComponentProfiler& ComponentProfiler::instance() noexcept {
    static ComponentProfiler profiler;
    return profiler;
}

HotPathCounters& HotPathCounters::instance() noexcept {
    static HotPathCounters counters;
    return counters;
}

void HotPathCounters::reset() noexcept {
    counters_ = {};
}

void HotPathCounters::add(
        const HotPathCounter counter,
        const std::uint64_t amount) noexcept {
    const std::size_t index = static_cast<std::size_t>(counter);
    if (index < counters_.size()) {
        counters_[index] += amount;
    }
}

std::uint64_t HotPathCounters::value(
        const HotPathCounter counter) const noexcept {
    const std::size_t index = static_cast<std::size_t>(counter);
    return index < counters_.size() ? counters_[index] : 0U;
}

std::array<std::uint64_t, hotPathCounterCount>
HotPathCounters::snapshot() const noexcept {
    return counters_;
}

#if defined(HGASP_ENABLE_HOT_PATH_COUNTERS)
namespace {

std::uint64_t emitHotPathCounterGroups(
        const char* const event,
        const std::uint64_t iteration,
        const bool includeIteration) {
    const auto values = HotPathCounters::instance().snapshot();
    std::uint64_t emittedGroups = 0U;
    for (const char* const group : hotPathCounterGroups) {
        EventFields fields;
        fields.emplace_back("group", group);
        if (includeIteration) {
            fields.emplace_back(
                "iteration",
                static_cast<unsigned long long>(iteration));
        }
        bool hasValues = false;
        for (const HotPathCounterDescriptor& descriptor :
             hotPathCounterDescriptors) {
            if (std::string(descriptor.group) != group) {
                continue;
            }
            const std::uint64_t value = values[
                static_cast<std::size_t>(descriptor.counter)];
            fields.emplace_back(
                descriptor.name,
                static_cast<unsigned long long>(value));
            hasValues = hasValues || value != 0U;
        }
        if (!hasValues) {
            continue;
        }
        ++emittedGroups;
        RunLogger::event(
            LogLevel::info,
            "timing",
            event,
            {},
            std::move(fields));
    }
    return emittedGroups;
}

}  // namespace
#endif

void ComponentProfiler::configureFromEnvironment() noexcept {
    HotPathCounters::instance().reset();
    reset(
        configuredBoolean(
            "HGASP_COMPONENT_TIMING_ENABLED", true),
        configuredInterval());
}

void emitHotPathCountersFinal(const bool completed) noexcept {
#if defined(HGASP_ENABLE_HOT_PATH_COUNTERS)
    const std::uint64_t emittedGroups = emitHotPathCounterGroups(
        "hot_path_counters_final", 0U, false);
    RunLogger::event(
        completed ? LogLevel::info : LogLevel::warning,
        "timing",
        "hot_path_counters_complete",
        {},
        {
            {"completed", completed},
            {"groups",
                static_cast<unsigned long long>(emittedGroups)},
            {"compiled", true},
        });
#else
    (void)completed;
#endif
}

void emitHotPathCountersSnapshot(
        const std::uint64_t iteration) noexcept {
#if defined(HGASP_ENABLE_HOT_PATH_COUNTERS)
    const std::uint64_t emittedGroups = emitHotPathCounterGroups(
        "hot_path_counters_snapshot", iteration, true);
    RunLogger::event(
        LogLevel::info,
        "timing",
        "hot_path_counters_snapshot_complete",
        {},
        {
            {"iteration", static_cast<unsigned long long>(iteration)},
            {"groups", static_cast<unsigned long long>(emittedGroups)},
            {"compiled", true},
        });
#else
    (void)iteration;
#endif
}

void ComponentProfiler::reset(
        const bool enabled,
        const std::uint64_t snapshotIntervalAttempts) noexcept {
    timings_ = {};
    enabled_ = enabled;
    snapshotIntervalAttempts_ = snapshotIntervalAttempts;
}

bool ComponentProfiler::enabled() const noexcept {
    return enabled_;
}

std::uint64_t
ComponentProfiler::snapshotIntervalAttempts() const noexcept {
    return snapshotIntervalAttempts_;
}

bool ComponentProfiler::shouldEmitSnapshot(
        const std::uint64_t iteration) const noexcept {
    return enabled_ &&
        snapshotIntervalAttempts_ > 0U &&
        iteration > 0U &&
        iteration % snapshotIntervalAttempts_ == 0U;
}

void ComponentProfiler::record(
        const TimedComponent component,
        const double inclusiveCpuSeconds,
        const double exclusiveCpuSeconds,
        const double inclusiveWallSeconds,
        const double exclusiveWallSeconds) noexcept {
    if (!enabled_) {
        return;
    }
    const std::size_t index =
        static_cast<std::size_t>(component);
    if (index >= componentCount) {
        return;
    }
    ComponentTiming& timing = timings_[index];
    ++timing.calls;
    const double inclusiveCpu =
        std::max(0.0, inclusiveCpuSeconds);
    const double exclusiveCpu =
        std::clamp(exclusiveCpuSeconds, 0.0, inclusiveCpu);
    const double inclusiveWall =
        std::max(0.0, inclusiveWallSeconds);
    const double exclusiveWall =
        std::clamp(exclusiveWallSeconds, 0.0, inclusiveWall);
    timing.inclusiveCpuSeconds += inclusiveCpu;
    timing.exclusiveCpuSeconds += exclusiveCpu;
    timing.inclusiveWallSeconds += inclusiveWall;
    timing.exclusiveWallSeconds += exclusiveWall;
    timing.maximumInclusiveCpuSeconds =
        std::max(
            timing.maximumInclusiveCpuSeconds,
            inclusiveCpu);
    timing.maximumInclusiveWallSeconds =
        std::max(
            timing.maximumInclusiveWallSeconds,
            inclusiveWall);
}

ComponentTiming ComponentProfiler::timing(
        const TimedComponent component) const noexcept {
    const std::size_t index =
        static_cast<std::size_t>(component);
    return index < componentCount
        ? timings_[index]
        : ComponentTiming{};
}

std::vector<ComponentTiming> ComponentProfiler::snapshot() const {
    return std::vector<ComponentTiming>(
        timings_.begin(), timings_.end());
}

ScopedComponentTimer::ScopedComponentTimer(
        const TimedComponent component)
    : component_(component) {
    if (!ComponentProfiler::instance().enabled()) {
        return;
    }
    cpuStarted_ = std::clock();
    wallStarted_ = std::chrono::steady_clock::now();
    active_ = true;
    stack_.push_back(this);
}

ScopedComponentTimer::~ScopedComponentTimer() {
    if (!active_) {
        return;
    }
    const std::clock_t cpuFinished = std::clock();
    const auto wallFinished =
        std::chrono::steady_clock::now();
    const double inclusiveCpu =
        nonnegativeElapsed(cpuStarted_, cpuFinished);
    const double inclusiveWall =
        nonnegativeWall(wallStarted_, wallFinished);
    const double exclusiveCpu =
        std::max(0.0, inclusiveCpu - nestedCpuSeconds_);
    const double exclusiveWall =
        std::max(0.0, inclusiveWall - nestedWallSeconds_);
    ComponentProfiler::instance().record(
        component_,
        inclusiveCpu,
        exclusiveCpu,
        inclusiveWall,
        exclusiveWall);

    if (!stack_.empty() && stack_.back() == this) {
        stack_.pop_back();
        if (!stack_.empty()) {
            stack_.back()->nestedCpuSeconds_ += inclusiveCpu;
            stack_.back()->nestedWallSeconds_ += inclusiveWall;
        }
    } else {
        // A non-LIFO destruction would invalidate exclusive attribution.
        // Clear the per-thread stack and retain safe inclusive measurements.
        stack_.clear();
    }
}

const char* timedComponentName(
        const TimedComponent component) noexcept {
    switch (component) {
    case TimedComponent::InstanceLoading:
        return "instance_loading";
    case TimedComponent::AlgorithmSetup:
        return "algorithm_setup";
    case TimedComponent::DepotConfigurationPreprocess:
        return "depot_configuration_preprocess";
    case TimedComponent::InitialPopulation:
        return "initial_population";
    case TimedComponent::ParentAndPopulationSelection:
        return "parent_population_selection";
    case TimedComponent::EaxCrossover:
        return "eax_crossover";
    case TimedComponent::ConstraintFeasibleReconstruction:
        return "constraint_feasible_reconstruction";
    case TimedComponent::CapacityRepair:
        return "capacity_repair";
    case TimedComponent::Mutation:
        return "mutation";
    case TimedComponent::CapacityRouteLocalSearch:
        return "route_local_search.capacity";
    case TimedComponent::HardTwRouteLocalSearch:
        return "route_local_search.hard_tw";
    case TimedComponent::RoutePoolAdmission:
        return "route_pool_admission";
    case TimedComponent::RoutePoolPruning:
        return "route_pool_pruning";
    case TimedComponent::PopulationAdmission:
        return "population_admission";
    case TimedComponent::SearchControl:
        return "search_control";
    case TimedComponent::SpLocalGlobal:
        return "sp_local_global";
    case TimedComponent::SpInjection:
        return "sp_injection";
    case TimedComponent::Count:
        break;
    }
    return "unknown";
}

const char* routeLocalSearchComponentName(
        const bool hardTimeWindows) noexcept {
    return timedComponentName(
        hardTimeWindows
            ? TimedComponent::HardTwRouteLocalSearch
            : TimedComponent::CapacityRouteLocalSearch);
}

const char* timedComponentScope(
        const TimedComponent component) noexcept {
    return component == TimedComponent::InstanceLoading
        ? "pre_search"
        : "search_after_instance_loading";
}

void emitComponentTimingSnapshot(
        const std::uint64_t iteration,
        const double searchCpuSeconds) noexcept {
    ComponentProfiler& profiler =
        ComponentProfiler::instance();
    if (!profiler.enabled()) {
        return;
    }
    EventFields fields;
    fields.reserve(componentCount + 4U);
    fields.emplace_back(
        "iteration",
        static_cast<unsigned long long>(iteration));
    fields.emplace_back(
        "search_cpu_seconds",
        std::max(0.0, searchCpuSeconds));
    fields.emplace_back("basis", "process_cpu");
    fields.emplace_back("aggregation", "cumulative_exclusive");
    for (std::size_t index = 0U;
         index < componentCount;
         ++index) {
        const TimedComponent component =
            static_cast<TimedComponent>(index);
        const ComponentTiming timing =
            profiler.timing(component);
        if (timing.calls == 0U) {
            continue;
        }
        fields.emplace_back(
            std::string(timedComponentName(component)) +
                "_cpu_seconds",
            timing.exclusiveCpuSeconds);
    }
    RunLogger::event(
        LogLevel::info,
        "timing",
        "component_timing_snapshot",
        {},
        std::move(fields));
}

void emitComponentTimingFinal(
        const std::uint64_t iteration,
        const double searchCpuSeconds,
        const bool completed) noexcept {
    ComponentProfiler& profiler =
        ComponentProfiler::instance();
    if (!profiler.enabled()) {
        return;
    }
    std::uint64_t activeComponents = 0U;
    double exclusiveCpuTotal = 0.0;
    double searchExclusiveCpuTotal = 0.0;
    double preSearchExclusiveCpuTotal = 0.0;
    for (std::size_t index = 0U;
         index < componentCount;
         ++index) {
        const TimedComponent component =
            static_cast<TimedComponent>(index);
        const ComponentTiming timing =
            profiler.timing(component);
        if (timing.calls == 0U) {
            continue;
        }
        ++activeComponents;
        exclusiveCpuTotal += timing.exclusiveCpuSeconds;
        if (component == TimedComponent::InstanceLoading) {
            preSearchExclusiveCpuTotal +=
                timing.exclusiveCpuSeconds;
        } else {
            searchExclusiveCpuTotal +=
                timing.exclusiveCpuSeconds;
        }
        RunLogger::event(
            LogLevel::info,
            "timing",
            "component_timing_final",
            {},
            {
                {"iteration",
                    static_cast<unsigned long long>(
                        iteration)},
                {"component",
                    timedComponentName(component)},
                {"scope",
                    timedComponentScope(component)},
                {"calls",
                    static_cast<unsigned long long>(
                        timing.calls)},
                {"inclusive_cpu_seconds",
                    timing.inclusiveCpuSeconds},
                {"exclusive_cpu_seconds",
                    timing.exclusiveCpuSeconds},
                {"inclusive_wall_seconds",
                    timing.inclusiveWallSeconds},
                {"exclusive_wall_seconds",
                    timing.exclusiveWallSeconds},
                {"maximum_inclusive_cpu_seconds",
                    timing.maximumInclusiveCpuSeconds},
                {"maximum_inclusive_wall_seconds",
                    timing.maximumInclusiveWallSeconds},
                {"cpu_basis", "process_cpu"},
                {"wall_basis", "steady_clock"},
            });
    }
    RunLogger::event(
        completed ? LogLevel::info : LogLevel::warning,
        "timing",
        "component_timing_complete",
        {},
        {
            {"iteration",
                static_cast<unsigned long long>(iteration)},
            {"completed", completed},
            {"active_components",
                static_cast<unsigned long long>(
                    activeComponents)},
            {"instrumented_exclusive_cpu_seconds",
                exclusiveCpuTotal},
            {"search_scope_exclusive_cpu_seconds",
                searchExclusiveCpuTotal},
            {"pre_search_exclusive_cpu_seconds",
                preSearchExclusiveCpuTotal},
            {"search_cpu_seconds",
                std::max(0.0, searchCpuSeconds)},
            {"cpu_basis", "process_cpu"},
            {"wall_basis", "steady_clock"},
        });
}

}  // namespace hgasp
