#include "ProblemProfile.h"

#include <stdexcept>

namespace hgasp {

const char* toString(const TimeWindowPolicy policy) noexcept {
    switch (policy) {
        case TimeWindowPolicy::None:
            return "none";
        case TimeWindowPolicy::Hard:
            return "hard";
    }
    return "unknown";
}

const char* toString(const NumericRounding rounding) noexcept {
    switch (rounding) {
        case NumericRounding::Exact:
            return "exact";
        case NumericRounding::Floor:
            return "floor";
        case NumericRounding::CeilAfterScalingBy100:
            return "ceil-after-scaling-by-100";
    }
    return "unknown";
}

const char* toString(const InstanceFormat format) noexcept {
    switch (format) {
        case InstanceFormat::LegacyDat:
            return "legacy-dat";
        case InstanceFormat::SetSJson:
            return "set-s-json";
        case InstanceFormat::FarhamText:
            return "farham-text";
    }
    return "unknown";
}

const char* toString(
        const CustomerDepotCandidatePolicy policy) noexcept {
    switch (policy) {
        case CustomerDepotCandidatePolicy::LegacyNearest:
            return "legacy-nearest";
        case CustomerDepotCandidatePolicy::
                CapacityCompatibleAllDepots:
            return "capacity-compatible-all-depots";
    }
    return "unknown";
}

const char* toString(const DepotConfigurationPolicy policy) noexcept {
    switch (policy) {
        case DepotConfigurationPolicy::LegacyPreprocessed:
            return "legacy-preprocessed";
        case DepotConfigurationPolicy::FreeDiscovery:
            return "free-discovery";
    }
    return "unknown";
}

ProblemProfile makeSetSProblemProfile(
        const InstanceFormat format,
        const NumericRounding travelCostRounding) {
    ProblemProfile profile;
    profile.benchmarkFamily = BenchmarkFamily::SetS;
    profile.format = format;
    profile.constraints.timeWindows = TimeWindowPolicy::None;
    profile.matrix.travelCostRounding = travelCostRounding;
    profile.matrix.travelTimeRounding = NumericRounding::Exact;
    profile.customerDepotCandidates =
        CustomerDepotCandidatePolicy::CapacityCompatibleAllDepots;
    profile.depotConfigurations =
        DepotConfigurationPolicy::FreeDiscovery;
    if (travelCostRounding ==
            NumericRounding::CeilAfterScalingBy100) {
        profile.reporting.metricPolicyId = kMetricPolicyHe2025;
        profile.reporting.reportingPolicyId =
            kReportingPolicyInteger;
        profile.reporting.objectiveDecimals = 0;
    } else if (travelCostRounding == NumericRounding::Exact) {
        profile.reporting.metricPolicyId =
            kMetricPolicyLegacySetSExact;
        profile.reporting.reportingPolicyId =
            kReportingPolicyDecimal2;
        profile.reporting.objectiveDecimals = 2;
    } else {
        throw std::invalid_argument(
            "Set S does not support the requested travel-cost rounding");
    }
    validateProblemProfile(profile);
    return profile;
}

ProblemProfile makeFarham2018ProblemProfile() {
    ProblemProfile profile;
    profile.benchmarkFamily = BenchmarkFamily::SetF;
    profile.format = InstanceFormat::FarhamText;
    profile.constraints.timeWindows = TimeWindowPolicy::Hard;
    profile.matrix.travelCostRounding = NumericRounding::Floor;
    profile.matrix.travelTimeRounding = NumericRounding::Floor;
    profile.reporting.metricPolicyId = kMetricPolicyFarham2018;
    profile.reporting.reportingPolicyId = kReportingPolicyInteger;
    profile.reporting.objectiveDecimals = 0;
    profile.customerDepotCandidates =
        CustomerDepotCandidatePolicy::
            CapacityCompatibleAllDepots;
    profile.depotConfigurations =
        DepotConfigurationPolicy::FreeDiscovery;
    validateProblemProfile(profile);
    return profile;
}

void validateProblemProfile(const ProblemProfile& profile) {
    if (profile.family != kProblemFamilyClrp) {
        throw std::invalid_argument(
            "unsupported problem family '" + profile.family + "'");
    }
    if (profile.reporting.metricPolicyId.empty() ||
        profile.reporting.reportingPolicyId.empty()) {
        throw std::invalid_argument(
            "problem profile requires metric and reporting policy IDs");
    }
    if (profile.reporting.objectiveDecimals < 0 ||
        profile.reporting.objectiveDecimals > 12) {
        throw std::invalid_argument(
            "objective reporting decimals must be between 0 and 12");
    }
    if (profile.benchmarkFamily == BenchmarkFamily::SetS) {
        if (profile.format == InstanceFormat::FarhamText ||
            profile.constraints.hasHardTimeWindows() ||
            profile.matrix.travelTimeRounding != NumericRounding::Exact ||
            (profile.matrix.travelCostRounding !=
                 NumericRounding::CeilAfterScalingBy100 &&
             profile.matrix.travelCostRounding != NumericRounding::Exact)) {
            throw std::invalid_argument(
                "Set S profile has an incompatible format, constraint, or "
                "matrix policy");
        }
    } else if (profile.benchmarkFamily == BenchmarkFamily::SetF) {
        if (profile.format != InstanceFormat::FarhamText ||
            !profile.constraints.hasHardTimeWindows() ||
            profile.matrix.travelCostRounding != NumericRounding::Floor ||
            profile.matrix.travelTimeRounding != NumericRounding::Floor) {
            throw std::invalid_argument(
                "Set F profile requires Farham format, hard time windows, "
                "and per-arc floor cost/time matrices");
        }
    }
}

std::vector<std::string> activeConstraintIds(
        const ProblemProfile& profile) {
    std::vector<std::string> result{
        "vehicle_capacity",
        "depot_capacity",
        "customer_exact_cover",
    };
    if (profile.constraints.hasHardTimeWindows()) {
        result.push_back("hard_time_windows");
    }
    return result;
}

}  // namespace hgasp
