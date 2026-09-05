#include "LocalSpFamilyBudget.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace hgasp {

const char* toString(
        const LocalSpFamilyGateReason reason) noexcept {
    switch (reason) {
    case LocalSpFamilyGateReason::Eligible:
        return "eligible";
    case LocalSpFamilyGateReason::MinimumInterval:
        return "family_minimum_interval";
    case LocalSpFamilyGateReason::CpuShare:
        return "family_cpu_share";
    }
    return "unknown";
}

LocalSpFamilyBudget::LocalSpFamilyBudget(
        const LocalSpFamilyPolicy policy) {
    configure(policy);
}

void LocalSpFamilyBudget::validatePolicy(
        const LocalSpFamilyPolicy& policy) {
    if (!std::isfinite(policy.cpuShareLimit) ||
        policy.cpuShareLimit < 0.0 ||
        policy.cpuShareLimit > 1.0) {
        throw std::invalid_argument(
            "Local-SP family CPU share must be in [0,1]");
    }
}

void LocalSpFamilyBudget::validateContext(
        const LocalSpFamilyContext& context) {
    if (!std::isfinite(context.elapsedSearchCpuSeconds) ||
        context.elapsedSearchCpuSeconds < 0.0 ||
        !std::isfinite(context.expectedCallCpuSeconds) ||
        context.expectedCallCpuSeconds < 0.0) {
        throw std::invalid_argument(
            "invalid Local-SP family budget context");
    }
}

void LocalSpFamilyBudget::configure(
        const LocalSpFamilyPolicy policy) {
    validatePolicy(policy);
    policy_ = policy;
}

LocalSpFamilyDecision LocalSpFamilyBudget::decide(
        const LocalSpFamilyContext& context) const {
    validateContext(context);
    LocalSpFamilyDecision decision;
    decision.consumedCpuSeconds = consumedCpuSeconds_;
    decision.projectedCpuSeconds =
        consumedCpuSeconds_ + context.expectedCallCpuSeconds;
    decision.projectedSearchCpuSeconds = std::max(
        context.elapsedSearchCpuSeconds +
            context.expectedCallCpuSeconds,
        decision.projectedCpuSeconds);
    decision.projectedCpuShare =
        decision.projectedSearchCpuSeconds > 0.0
        ? decision.projectedCpuSeconds /
            decision.projectedSearchCpuSeconds
        : 0.0;

    if (lastCallAttempt_.has_value()) {
        if (context.currentAttempt < *lastCallAttempt_) {
            throw std::invalid_argument(
                "Local-SP family attempt counter moved backwards");
        }
        decision.attemptsSinceLastCall =
            context.currentAttempt - *lastCallAttempt_;
        if (decision.attemptsSinceLastCall <
            policy_.minimumIntervalAttempts) {
            decision.reason =
                LocalSpFamilyGateReason::MinimumInterval;
            return decision;
        }
    } else {
        decision.attemptsSinceLastCall =
            std::numeric_limits<std::uint64_t>::max();
    }

    if (decision.projectedCpuShare >
        policy_.cpuShareLimit + 1.0e-12) {
        decision.reason = LocalSpFamilyGateReason::CpuShare;
        return decision;
    }
    decision.eligible = true;
    decision.reason = LocalSpFamilyGateReason::Eligible;
    return decision;
}

void LocalSpFamilyBudget::recordCall(
        const std::uint64_t startAttempt,
        const double outerCpuSeconds) {
    if (!std::isfinite(outerCpuSeconds) || outerCpuSeconds < 0.0) {
        throw std::invalid_argument(
            "Local-SP family CPU observation must be finite and "
            "non-negative");
    }
    if (lastCallAttempt_.has_value() &&
        (startAttempt < *lastCallAttempt_ ||
         startAttempt - *lastCallAttempt_ <
             policy_.minimumIntervalAttempts)) {
        throw std::logic_error(
            "Local-SP family call violated the minimum interval");
    }
    lastCallAttempt_ = startAttempt;
    ++callCount_;
    consumedCpuSeconds_ += outerCpuSeconds;
}

double LocalSpFamilyBudget::cpuShare(
        const double elapsedSearchCpuSeconds) const {
    if (!std::isfinite(elapsedSearchCpuSeconds) ||
        elapsedSearchCpuSeconds < 0.0) {
        throw std::invalid_argument(
            "invalid elapsed search CPU for Local-SP family share");
    }
    if (elapsedSearchCpuSeconds <= 0.0) {
        return consumedCpuSeconds_ > 0.0
            ? std::numeric_limits<double>::infinity()
            : 0.0;
    }
    return consumedCpuSeconds_ / elapsedSearchCpuSeconds;
}

}  // namespace hgasp
