#ifndef HGASP_CORE_GLOBAL_SP_PROMOTION_TRANSACTION_H_
#define HGASP_CORE_GLOBAL_SP_PROMOTION_TRANSACTION_H_

#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>

namespace hgasp {

// Static HGAMP keeps stable internal slots after the permanent free
// population: free+1 is the optional challenger and free+2 is transactional
// promotion staging. Staging is required by strict Global-SP even when the
// challenger feature is disabled.
constexpr std::size_t staticHgampPromotionStagingPopulationIndex(
        const std::size_t unrestrictedPopulationIndex) noexcept {
    return unrestrictedPopulationIndex + 2U;
}

constexpr std::size_t staticHgampRequiredPopulationSlotCount(
        const std::size_t unrestrictedPopulationIndex) noexcept {
    return staticHgampPromotionStagingPopulationIndex(
               unrestrictedPopulationIndex) + 1U;
}

constexpr bool permanentFreePopulationMeetsEstablishedFloor(
        const std::size_t currentSize,
        const std::size_t establishedFloor) noexcept {
    return currentSize >= (establishedFloor == 0U ? 1U : establishedFloor);
}

enum class PromotionFailureKind {
    None,
    RecoverableBuildFailure,
    RecoverableAdmissionFailure,
    ContractViolation,
};

enum class GlobalSpPromotionFallbackTarget {
    UnrestrictedFree,
    ActiveChallenger,
};

struct DirectPromotionAttempt {
    bool promoted = false;
    int populationIndex = -1;
    PromotionFailureKind failure = PromotionFailureKind::None;
    std::string reason;
};

struct PromotionFallbackAdmission {
    bool admitted = false;
    int populationIndex = -1;
};

struct GlobalSpDirectPromotionResult {
    bool promoted = false;
    bool fallbackAttempted = false;
    bool fallbackAdmitted = false;
    int populationIndex = -1;
    PromotionFailureKind failure = PromotionFailureKind::None;
    GlobalSpPromotionFallbackTarget fallbackTarget =
        GlobalSpPromotionFallbackTarget::UnrestrictedFree;
    std::string reason;

    bool populationAdmitted() const noexcept {
        return promoted || fallbackAdmitted;
    }
};

// Orders the two independent transactions for a validated, strictly
// improving, unseen Global-SP solution. Incumbent and route preservation are
// completed before resident promotion is attempted. Only explicitly
// recoverable portfolio failures enter fallback; contract failures remain
// fail-fast.
template <typename AcceptIncumbent,
          typename PreserveRoutes,
          typename AttemptPromotion,
          typename AdmitFallback>
GlobalSpDirectPromotionResult executeGlobalSpDirectPromotion(
        const bool configurationMatchesActiveChallenger,
        AcceptIncumbent&& acceptIncumbent,
        PreserveRoutes&& preserveRoutes,
        AttemptPromotion&& attemptPromotion,
        AdmitFallback&& admitFallback) {
    std::forward<AcceptIncumbent>(acceptIncumbent)();
    std::forward<PreserveRoutes>(preserveRoutes)();

    const DirectPromotionAttempt attempt =
        std::forward<AttemptPromotion>(attemptPromotion)();
    if (attempt.promoted) {
        if (attempt.failure != PromotionFailureKind::None ||
            attempt.populationIndex < 0) {
            throw std::logic_error(
                "successful Global-SP promotion returned inconsistent "
                "status");
        }
        GlobalSpDirectPromotionResult result;
        result.promoted = true;
        result.populationIndex = attempt.populationIndex;
        return result;
    }
    if (attempt.failure == PromotionFailureKind::None ||
        attempt.failure == PromotionFailureKind::ContractViolation) {
        throw std::logic_error(
            attempt.reason.empty()
                ? "Global-SP promotion contract violation"
                : attempt.reason);
    }

    GlobalSpDirectPromotionResult result;
    result.failure = attempt.failure;
    result.reason = attempt.reason;
    result.fallbackAttempted = true;
    result.fallbackTarget = configurationMatchesActiveChallenger
        ? GlobalSpPromotionFallbackTarget::ActiveChallenger
        : GlobalSpPromotionFallbackTarget::UnrestrictedFree;
    const PromotionFallbackAdmission fallback =
        std::forward<AdmitFallback>(admitFallback)(result.fallbackTarget);
    result.fallbackAdmitted = fallback.admitted;
    result.populationIndex = fallback.populationIndex;
    return result;
}

inline const char* toString(const PromotionFailureKind kind) noexcept {
    switch (kind) {
        case PromotionFailureKind::None:
            return "none";
        case PromotionFailureKind::RecoverableBuildFailure:
            return "recoverable_build_failure";
        case PromotionFailureKind::RecoverableAdmissionFailure:
            return "recoverable_admission_failure";
        case PromotionFailureKind::ContractViolation:
            return "contract_violation";
    }
    return "unknown";
}

inline const char* toString(
        const GlobalSpPromotionFallbackTarget target) noexcept {
    return target == GlobalSpPromotionFallbackTarget::ActiveChallenger
        ? "active_challenger"
        : "unrestricted_free";
}

}  // namespace hgasp

#endif  // HGASP_CORE_GLOBAL_SP_PROMOTION_TRANSACTION_H_
