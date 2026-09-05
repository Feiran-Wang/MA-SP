#ifndef HGASP_PROBLEM_REPORTINGPOLICY_H_
#define HGASP_PROBLEM_REPORTINGPOLICY_H_

#include <string>

namespace hgasp {

enum class NumericRounding {
    Exact,
    Floor,
    CeilAfterScalingBy100,
};

struct MatrixPolicy {
    NumericRounding travelCostRounding = NumericRounding::Exact;
    NumericRounding travelTimeRounding = NumericRounding::Exact;
    bool zeroDepotToDepotEntries = true;
};

struct ReportingPolicy {
    std::string metricPolicyId;
    std::string reportingPolicyId;
    int objectiveDecimals = 0;
};

const char* toString(NumericRounding rounding) noexcept;

}  // namespace hgasp

#endif  // HGASP_PROBLEM_REPORTINGPOLICY_H_
