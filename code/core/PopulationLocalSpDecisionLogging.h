#ifndef HGASP_CORE_POPULATION_LOCAL_SP_DECISION_LOGGING_H_
#define HGASP_CORE_POPULATION_LOCAL_SP_DECISION_LOGGING_H_

#include <cstdint>
#include <cstddef>
#include <string_view>

namespace hgasp {

struct SpColumnTelemetry {
    double usefulColumnCredit = 0.0;
    std::uint64_t snapshotColumnCount = 0U;
};

inline SpColumnTelemetry makeSpColumnTelemetry(
        const double usefulColumnCredit,
        const std::size_t snapshotColumnCount) noexcept {
    return {
        usefulColumnCredit,
        static_cast<std::uint64_t>(snapshotColumnCount),
    };
}

// Run-level telemetry for population Local-SP control decisions.  This class
// only classifies decisions after the controller (or an existing coordinator
// prefilter) has run; it never gates or otherwise changes dispatch behavior.
struct PopulationLocalSpDecisionLogSummary {
    std::uint64_t controllerDecisions = 0U;
    std::uint64_t notStagnant = 0U;
    std::uint64_t snapshotConsumed = 0U;
    std::uint64_t insufficientChange = 0U;
    std::uint64_t familyInterval = 0U;
    std::uint64_t cpuShare = 0U;
    std::uint64_t incompleteCoverage = 0U;
    std::uint64_t eligible = 0U;
    std::uint64_t selected = 0U;
    std::uint64_t otherReject = 0U;

    void recordSuppressedReason(const std::string_view reason) noexcept {
        if (reason == "population_not_stagnant") {
            ++notStagnant;
        } else if (reason == "snapshot_already_consumed") {
            ++snapshotConsumed;
        } else if (reason == "insufficient_meaningful_master_change") {
            ++insufficientChange;
        } else if (reason == "family_minimum_interval") {
            ++familyInterval;
        } else if (reason == "family_cpu_share") {
            ++cpuShare;
        } else if (reason == "incomplete_customer_coverage") {
            ++incompleteCoverage;
        } else {
            ++otherReject;
        }
    }

    bool recordDecision(
            const bool isEligible,
            const std::string_view reason) noexcept {
        ++controllerDecisions;
        if (isEligible) {
            ++eligible;
            return true;
        }
        recordSuppressedReason(reason);
        return false;
    }

    void recordSelected() noexcept {
        ++selected;
    }
};

}  // namespace hgasp

#endif  // HGASP_CORE_POPULATION_LOCAL_SP_DECISION_LOGGING_H_
