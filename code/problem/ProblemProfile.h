#ifndef HGASP_PROBLEM_PROBLEMPROFILE_H_
#define HGASP_PROBLEM_PROBLEMPROFILE_H_

#include "BenchmarkFamily.h"
#include "ConstraintProfile.h"
#include "ReportingPolicy.h"

#include <string>
#include <vector>

namespace hgasp {

inline constexpr const char* kProblemFamilyClrp = "clrp";
inline constexpr const char* kMetricPolicyFarham2018 =
    "farham-2018-set-f-floor-euclidean";
inline constexpr const char* kMetricPolicyHe2025 =
    "he-2025-set-s-hgamp-ceil100";
inline constexpr const char* kMetricPolicyLegacySetSExact =
    "legacy-set-s-exact-euclidean";
inline constexpr const char* kReportingPolicyInteger =
    "integer-objective";
inline constexpr const char* kReportingPolicyDecimal2 =
    "decimal-2-objective";

enum class InstanceFormat {
    LegacyDat,
    SetSJson,
    FarhamText,
};

enum class CustomerDepotCandidatePolicy {
    LegacyNearest,
    CapacityCompatibleAllDepots,
};

enum class DepotConfigurationPolicy {
    LegacyPreprocessed,
    FreeDiscovery,
};

struct ProblemProfile {
    std::string family = kProblemFamilyClrp;
    BenchmarkFamily benchmarkFamily = BenchmarkFamily::SetS;
    ConstraintProfile constraints;
    MatrixPolicy matrix;
    ReportingPolicy reporting;
    InstanceFormat format = InstanceFormat::LegacyDat;
    CustomerDepotCandidatePolicy customerDepotCandidates =
        CustomerDepotCandidatePolicy::CapacityCompatibleAllDepots;
    DepotConfigurationPolicy depotConfigurations =
        DepotConfigurationPolicy::FreeDiscovery;
};

ProblemProfile makeSetSProblemProfile(
    InstanceFormat format,
    NumericRounding travelCostRounding =
        NumericRounding::CeilAfterScalingBy100);
ProblemProfile makeFarham2018ProblemProfile();
void validateProblemProfile(const ProblemProfile& profile);
std::vector<std::string> activeConstraintIds(
    const ProblemProfile& profile);

const char* toString(InstanceFormat format) noexcept;
const char* toString(CustomerDepotCandidatePolicy policy) noexcept;
const char* toString(DepotConfigurationPolicy policy) noexcept;

}  // namespace hgasp

#endif  // HGASP_PROBLEM_PROBLEMPROFILE_H_
