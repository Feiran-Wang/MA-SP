#include "SearchComponents.h"

#include <array>
#include <atomic>
#include <utility>

namespace hgasp {

namespace {

std::array<std::atomic<std::uint64_t>, kV114ComponentCount>
    v114ActivationCounts{};

ComponentActivation activation(
        std::string name,
        const bool configured,
        const bool effective,
        std::string disabledReason,
        const std::uint64_t activationCount = 0U) {
    return {
        std::move(name),
        configured,
        effective,
        effective ? std::string() : std::move(disabledReason),
        effective ? activationCount : 0U,
    };
}

}  // namespace

const char* v114ComponentName(
        const V114Component component) noexcept {
    switch (component) {
        case V114Component::StaticHgampPopulations:
            return "static_hgamp_populations";
        case V114Component::SpatioTemporalCrh:
            return "st_crh";
        case V114Component::StrictCrossConfigurationMating:
            return "strict_cross_configuration_mating";
        case V114Component::PopulationLocalSpRescue:
            return "population_local_sp_rescue";
        case V114Component::PreReductionGlobalSp:
            return "pre_reduction_global_sp";
        case V114Component::PopulationReductionPhase1:
            return "population_reduction_phase_1";
        case V114Component::PopulationReductionPhase2:
            return "population_reduction_phase_2";
        case V114Component::FinalGlobalSp:
            return "final_global_sp";
        case V114Component::Count:
            break;
    }
    return "unknown";
}

V114ComponentActivationConfiguration
makeV114ComponentActivationConfiguration(
        const bool staticHgampMode,
        const bool hardTimeWindows,
        const bool strictCrossConfigurationMating,
        const bool populationLocalSpEnabled,
        const bool preReductionGlobalSpEnabled) {
    V114ComponentActivationConfiguration result;
    const auto set = [&result](
            const V114Component component,
            const bool configured,
            std::string disabledReason) {
        const std::size_t index = static_cast<std::size_t>(component);
        result.configured[index] = configured;
        result.disabledReasons[index] = configured
            ? "not_activated_in_run"
            : std::move(disabledReason);
    };
    set(
        V114Component::StaticHgampPopulations,
        staticHgampMode,
        "population_mode_not_static_hgamp");
    set(
        V114Component::SpatioTemporalCrh,
        staticHgampMode && hardTimeWindows,
        !staticHgampMode
            ? "population_mode_not_static_hgamp"
            : "profile_has_no_hard_time_windows");
    set(
        V114Component::StrictCrossConfigurationMating,
        staticHgampMode && strictCrossConfigurationMating,
        !staticHgampMode
            ? "population_mode_not_static_hgamp"
            : "disabled_by_configuration");
    set(
        V114Component::PopulationLocalSpRescue,
        staticHgampMode && populationLocalSpEnabled,
        !staticHgampMode
            ? "population_mode_not_static_hgamp"
            : "disabled_by_configuration");
    set(
        V114Component::PreReductionGlobalSp,
        staticHgampMode && preReductionGlobalSpEnabled,
        !staticHgampMode
            ? "population_mode_not_static_hgamp"
            : "disabled_by_configuration");
    set(
        V114Component::PopulationReductionPhase1,
        staticHgampMode,
        "population_mode_not_static_hgamp");
    set(
        V114Component::PopulationReductionPhase2,
        staticHgampMode,
        "population_mode_not_static_hgamp");
    set(
        V114Component::FinalGlobalSp,
        true,
        "not_activated_in_run");
    return result;
}

ComponentActivationSummary makeComponentActivationSummary(
        const bool hardTimeWindows,
        const V114ComponentActivationConfiguration& v114) {
    ComponentActivationSummary result;
    result.components.reserve(2U + kV114ComponentCount);
    result.components.push_back(activation(
        "route_local_search.capacity",
        true,
        !hardTimeWindows,
        "hard_time_window_profile_uses_route_local_search.hard_tw",
        !hardTimeWindows ? 1U : 0U));
    result.components.push_back(activation(
        "route_local_search.hard_tw",
        true,
        hardTimeWindows,
        "profile_has_no_hard_time_windows",
        hardTimeWindows ? 1U : 0U));
    for (std::size_t index = 0U;
         index < kV114ComponentCount;
         ++index) {
        const V114Component component =
            static_cast<V114Component>(index);
        const std::uint64_t count =
            v114ActivationCounts[index].load(
                std::memory_order_acquire);
        const bool configured = v114.configured[index];
        result.components.push_back(activation(
            v114ComponentName(component),
            configured,
            configured && count > 0U,
            v114.disabledReasons[index].empty()
                ? std::string("not_activated_in_run")
                : v114.disabledReasons[index],
            count));
    }
    return result;
}

void recordV114ComponentActivation(
        const V114Component component,
        const std::uint64_t count) noexcept {
    const std::size_t index = static_cast<std::size_t>(component);
    if (index >= kV114ComponentCount || count == 0U) {
        return;
    }
    v114ActivationCounts[index].fetch_add(
        count, std::memory_order_acq_rel);
}

std::uint64_t v114ComponentActivationCount(
        const V114Component component) noexcept {
    const std::size_t index = static_cast<std::size_t>(component);
    if (index >= kV114ComponentCount) {
        return 0U;
    }
    return v114ActivationCounts[index].load(
        std::memory_order_acquire);
}

const ComponentActivation* findComponentActivation(
        const ComponentActivationSummary& summary,
        const std::string& name) noexcept {
    for (const ComponentActivation& component : summary.components) {
        if (component.name == name) {
            return &component;
        }
    }
    return nullptr;
}

}  // namespace hgasp
