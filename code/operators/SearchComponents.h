#ifndef HGASP_OPERATORS_SEARCHCOMPONENTS_H_
#define HGASP_OPERATORS_SEARCHCOMPONENTS_H_

#include "Version.h"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace hgasp {

struct ComponentActivation {
    std::string name;
    bool configured = false;
    bool effective = false;
    std::string disabledReason;
    std::uint64_t activationCount = 0U;
};

struct ComponentActivationSummary {
    std::vector<ComponentActivation> components;
};

enum class V114Component : std::size_t {
    StaticHgampPopulations = 0U,
    SpatioTemporalCrh,
    StrictCrossConfigurationMating,
    PopulationLocalSpRescue,
    PreReductionGlobalSp,
    PopulationReductionPhase1,
    PopulationReductionPhase2,
    FinalGlobalSp,
    Count,
};

inline constexpr std::size_t kV114ComponentCount =
    static_cast<std::size_t>(V114Component::Count);

struct V114ComponentActivationConfiguration {
    std::array<bool, kV114ComponentCount> configured{};
    std::array<std::string, kV114ComponentCount> disabledReasons{};
};

V114ComponentActivationConfiguration
makeV114ComponentActivationConfiguration(
    bool staticHgampMode,
    bool hardTimeWindows,
    bool strictCrossConfigurationMating,
    bool populationLocalSpEnabled,
    bool preReductionGlobalSpEnabled);

ComponentActivationSummary makeComponentActivationSummary(
    bool hardTimeWindows,
    const V114ComponentActivationConfiguration& v114 = {});

void recordV114ComponentActivation(
    V114Component component,
    std::uint64_t count = 1U) noexcept;

std::uint64_t v114ComponentActivationCount(
    V114Component component) noexcept;

const char* v114ComponentName(V114Component component) noexcept;

const ComponentActivation* findComponentActivation(
    const ComponentActivationSummary& summary,
    const std::string& name) noexcept;

}  // namespace hgasp

#endif  // HGASP_OPERATORS_SEARCHCOMPONENTS_H_
