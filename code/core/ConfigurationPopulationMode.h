#ifndef HGASP_CORE_CONFIGURATION_POPULATION_MODE_H_
#define HGASP_CORE_CONFIGURATION_POPULATION_MODE_H_

#include <stdexcept>
#include <string>

namespace hgasp {

enum class ConfigurationPopulationMode {
    StaticHgamp,
    FreeOnly,
};

inline const char* toString(
        const ConfigurationPopulationMode mode) noexcept {
    switch (mode) {
        case ConfigurationPopulationMode::StaticHgamp:
            return "static_hgamp";
        case ConfigurationPopulationMode::FreeOnly:
            return "free_only";
    }
    return "unknown";
}

inline ConfigurationPopulationMode parseConfigurationPopulationMode(
        const std::string& value) {
    if (value == "static_hgamp") {
        return ConfigurationPopulationMode::StaticHgamp;
    }
    if (value == "free_only") {
        return ConfigurationPopulationMode::FreeOnly;
    }
    throw std::invalid_argument(
        "population mode must be static_hgamp or free_only");
}

enum class ConfigurationRankerMode {
    ProfileAwareCrh,
};

inline const char* toString(
        const ConfigurationRankerMode mode) noexcept {
    switch (mode) {
        case ConfigurationRankerMode::ProfileAwareCrh:
            return "profile_aware_crh";
    }
    return "unknown";
}

inline ConfigurationRankerMode parseConfigurationRankerMode(
        const std::string& value) {
    if (value == "profile_aware_crh") {
        return ConfigurationRankerMode::ProfileAwareCrh;
    }
    throw std::invalid_argument(
        "configuration ranker must be profile_aware_crh");
}

}  // namespace hgasp

#endif  // HGASP_CORE_CONFIGURATION_POPULATION_MODE_H_
