#ifndef HGASP_CORE_NUMERICPOLICY_H_
#define HGASP_CORE_NUMERICPOLICY_H_

#include "ReportingPolicy.h"

#include <cmath>

namespace hgasp {

inline int objectiveReportingDecimals(
        const ReportingPolicy& policy) noexcept {
    return policy.objectiveDecimals;
}

inline double roundObjectiveForReporting(
    const double value,
    const ReportingPolicy& policy) noexcept {
    double scale = 1.0;
    for (int decimal = 0;
         decimal < policy.objectiveDecimals;
         ++decimal) {
        scale *= 10.0;
    }
    return std::round(value * scale) / scale;
}

inline bool strictlyImprovesObjective(
        const double previous,
        const double candidate,
        const double tolerance) noexcept {
    return candidate + tolerance < previous;
}

inline const char* objectiveReportingRounding() noexcept {
    return "nearest-half-away-from-zero";
}

}  // namespace hgasp

#endif  // HGASP_CORE_NUMERICPOLICY_H_
