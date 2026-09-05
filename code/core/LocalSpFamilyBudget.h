#ifndef HGASP_CORE_LOCAL_SP_FAMILY_BUDGET_H_
#define HGASP_CORE_LOCAL_SP_FAMILY_BUDGET_H_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace hgasp {

struct LocalSpFamilyPolicy {
    std::uint64_t minimumIntervalAttempts = 750U;
    double cpuShareLimit = 0.08;
};

struct LocalSpFamilyContext {
    std::uint64_t currentAttempt = 0U;
    double elapsedSearchCpuSeconds = 0.0;
    // The caller supplies a conservative process-CPU projection. This class
    // deliberately does not read a clock or reinterpret a solver wall limit.
    double expectedCallCpuSeconds = 0.0;
};

enum class LocalSpFamilyGateReason {
    Eligible,
    MinimumInterval,
    CpuShare,
};

const char* toString(LocalSpFamilyGateReason reason) noexcept;

struct LocalSpFamilyDecision {
    bool eligible = false;
    LocalSpFamilyGateReason reason =
        LocalSpFamilyGateReason::CpuShare;
    std::uint64_t attemptsSinceLastCall = 0U;
    double consumedCpuSeconds = 0.0;
    double projectedCpuSeconds = 0.0;
    double projectedSearchCpuSeconds = 0.0;
    double projectedCpuShare = 0.0;
};

// Run-level gate shared by all population-local rescue calls. It owns only
// deterministic accounting supplied by the search; it never reads wall time,
// process CPU, solver state, or a Gurobi environment itself.
class LocalSpFamilyBudget {
public:
    explicit LocalSpFamilyBudget(
        LocalSpFamilyPolicy policy = {});

    void configure(LocalSpFamilyPolicy policy);

    LocalSpFamilyDecision decide(
        const LocalSpFamilyContext& context) const;

    // Records the start attempt and complete outer process-CPU cost exactly
    // once after a dispatched call, including deferred solver-start failures.
    // A record that violates the interval is a coordinator contract error.
    void recordCall(
        std::uint64_t startAttempt,
        double outerCpuSeconds);

    std::optional<std::uint64_t> lastCallAttempt() const noexcept {
        return lastCallAttempt_;
    }

    std::size_t callCount() const noexcept {
        return callCount_;
    }

    double consumedCpuSeconds() const noexcept {
        return consumedCpuSeconds_;
    }

    double cpuShare(double elapsedSearchCpuSeconds) const;

    const LocalSpFamilyPolicy& policy() const noexcept {
        return policy_;
    }

private:
    static void validatePolicy(const LocalSpFamilyPolicy& policy);
    static void validateContext(const LocalSpFamilyContext& context);

    LocalSpFamilyPolicy policy_;
    std::optional<std::uint64_t> lastCallAttempt_;
    std::size_t callCount_ = 0U;
    double consumedCpuSeconds_ = 0.0;
};

}  // namespace hgasp

#endif  // HGASP_CORE_LOCAL_SP_FAMILY_BUDGET_H_
