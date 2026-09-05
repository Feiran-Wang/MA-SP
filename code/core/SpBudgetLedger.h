#ifndef HGASP_CORE_SP_BUDGET_LEDGER_H_
#define HGASP_CORE_SP_BUDGET_LEDGER_H_

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace hgasp {

inline double conservativeProcessCpuProjection(
        const double solverWallLimitSeconds,
        const int solverThreads,
        const double previousOuterCpuSeconds = 0.0) {
    if (!std::isfinite(solverWallLimitSeconds) ||
        solverWallLimitSeconds < 0.0 ||
        solverThreads < 1 ||
        !std::isfinite(previousOuterCpuSeconds) ||
        previousOuterCpuSeconds < 0.0) {
        throw std::invalid_argument(
            "invalid SP process-CPU projection input");
    }
    return std::max(
        solverWallLimitSeconds *
            static_cast<double>(solverThreads),
        previousOuterCpuSeconds);
}

inline bool projectedCpuShareWithinTarget(
        const double consumedCpuSeconds,
        const double expectedCpuSeconds,
        const double elapsedSearchCpuSeconds,
        const double targetShare) {
    if (!std::isfinite(consumedCpuSeconds) ||
        consumedCpuSeconds < 0.0 ||
        !std::isfinite(expectedCpuSeconds) ||
        expectedCpuSeconds < 0.0 ||
        !std::isfinite(elapsedSearchCpuSeconds) ||
        elapsedSearchCpuSeconds < 0.0 ||
        !std::isfinite(targetShare) ||
        targetShare < 0.0 ||
        targetShare > 1.0) {
        throw std::invalid_argument(
            "CPU-share projection contains an invalid value");
    }
    if (targetShare <= 0.0) {
        return expectedCpuSeconds <= 0.0;
    }
    const double projectedComponent =
        consumedCpuSeconds + expectedCpuSeconds;
    const double projectedSearch =
        std::max(
            elapsedSearchCpuSeconds + expectedCpuSeconds,
            projectedComponent);
    return projectedComponent <=
        targetShare * projectedSearch + 1.0e-9;
}

inline double remainingOuterSpBudget(
        const double declaredSeconds,
        const double consumedWallSeconds) {
    if (!std::isfinite(declaredSeconds) ||
        declaredSeconds < 0.0 ||
        !std::isfinite(consumedWallSeconds) ||
        consumedWallSeconds < 0.0) {
        throw std::invalid_argument(
            "invalid SP outer wall-clock budget");
    }
    return std::max(0.0, declaredSeconds - consumedWallSeconds);
}

enum class SpScope {
    Local,
    Global,
};

// Phase accounting is deliberately separate from SpScope accounting. Scope
// totals retain the complete outer CPU cost exactly once, while phase totals
// expose which solver activity consumed that cost. This prevents LP probes
// from silently exhausting the budget intended for small integer
// recombination calls.
enum class SpBudgetPhase {
    LpProbe,
    LocalMip,
    GlobalMip,
    FinalGlobal,
};

// A call token makes settlement idempotent without coupling accounting to a
// particular execution mode.  The orchestrator owns the token for the whole
// post-start scope and settles it from both normal and exceptional exits.
struct SpBudgetCallToken {
    bool settled = false;
};

// Accounts process CPU consumed by Local and Global SP phases. The ledger is
// a controller guard, not a stopping rule: mandatory calls may exceed the
// target share while every recorded value remains observable.
class SpBudgetLedger {
public:
    explicit SpBudgetLedger(const double targetShare = 0.15) {
        configure(targetShare);
    }

    void configure(const double targetShare) {
        if (!std::isfinite(targetShare) ||
            targetShare < 0.0 ||
            targetShare > 1.0) {
            throw std::invalid_argument(
                "SP CPU target share must be in [0,1]");
        }
        targetShare_ = targetShare;
    }

    void configurePhaseTargets(
            const double lpProbeShare,
            const double localMipShare,
            const double globalMipShare) {
        const double values[] = {
            lpProbeShare,
            localMipShare,
            globalMipShare,
        };
        for (const double value : values) {
            if (!std::isfinite(value) || value < 0.0 || value > 1.0) {
                throw std::invalid_argument(
                    "SP phase CPU target share must be in [0,1]");
            }
        }
        lpProbeTargetShare_ = lpProbeShare;
        localMipTargetShare_ = localMipShare;
        globalMipTargetShare_ = globalMipShare;
    }

    void record(
            const SpScope scope,
            const double cpuSeconds,
            const bool mandatoryExemption = false) {
        if (!std::isfinite(cpuSeconds) || cpuSeconds < 0.0) {
            throw std::invalid_argument(
                "SP CPU observation must be finite and non-negative");
        }
        switch (scope) {
        case SpScope::Local:
            localCpuSeconds_ += cpuSeconds;
            lastLocalCpuSeconds_ = cpuSeconds;
            if (!mandatoryExemption) {
                guardableLocalCpuSeconds_ += cpuSeconds;
            }
            break;
        case SpScope::Global:
            globalCpuSeconds_ += cpuSeconds;
            lastGlobalCpuSeconds_ = cpuSeconds;
            if (!mandatoryExemption) {
                guardableGlobalCpuSeconds_ += cpuSeconds;
            }
            break;
        }
        if (mandatoryExemption) {
            mandatoryExemptCpuSeconds_ += cpuSeconds;
        }
    }

    bool settle(
            SpBudgetCallToken& token,
            const SpScope scope,
            const double cpuSeconds,
            const bool mandatoryExemption = false) {
        if (token.settled) {
            return false;
        }
        record(scope, cpuSeconds, mandatoryExemption);
        token.settled = true;
        return true;
    }

    void recordPhase(
            const SpBudgetPhase phase,
            const double cpuSeconds,
            const bool mandatoryExemption = false) {
        if (!std::isfinite(cpuSeconds) || cpuSeconds < 0.0) {
            throw std::invalid_argument(
                "SP phase CPU observation must be finite and non-negative");
        }
        switch (phase) {
        case SpBudgetPhase::LpProbe:
            lpProbeCpuSeconds_ += cpuSeconds;
            lastLpProbeCpuSeconds_ = cpuSeconds;
            if (!mandatoryExemption) {
                guardableLpProbeCpuSeconds_ += cpuSeconds;
            }
            break;
        case SpBudgetPhase::LocalMip:
            localMipCpuSeconds_ += cpuSeconds;
            lastLocalMipCpuSeconds_ = cpuSeconds;
            if (!mandatoryExemption) {
                guardableLocalMipCpuSeconds_ += cpuSeconds;
            }
            break;
        case SpBudgetPhase::GlobalMip:
            globalMipCpuSeconds_ += cpuSeconds;
            lastGlobalMipCpuSeconds_ = cpuSeconds;
            if (!mandatoryExemption) {
                guardableGlobalMipCpuSeconds_ += cpuSeconds;
            }
            break;
        case SpBudgetPhase::FinalGlobal:
            finalGlobalCpuSeconds_ += cpuSeconds;
            lastFinalGlobalCpuSeconds_ = cpuSeconds;
            if (!mandatoryExemption) {
                guardableFinalGlobalCpuSeconds_ += cpuSeconds;
            }
            break;
        }
    }

    bool canSpend(
            const SpScope scope,
            const double expectedFutureCpuSeconds,
            const double elapsedSearchCpuSeconds,
            const bool hardMandatory,
            const double globalReserveShare = 0.0,
            const double unrecordedElapsedCpuSeconds = 0.0) const {
        if (!std::isfinite(expectedFutureCpuSeconds) ||
            expectedFutureCpuSeconds < 0.0 ||
            !std::isfinite(elapsedSearchCpuSeconds) ||
            elapsedSearchCpuSeconds < 0.0 ||
            !std::isfinite(globalReserveShare) ||
            globalReserveShare < 0.0 ||
            globalReserveShare > 1.0 ||
            !std::isfinite(unrecordedElapsedCpuSeconds) ||
            unrecordedElapsedCpuSeconds < 0.0 ||
            targetShare_ + globalReserveShare > 1.0 + 1.0e-12) {
            throw std::invalid_argument(
                "SP budget query contains an invalid CPU value");
        }
        if (hardMandatory) {
            return true;
        }
        // elapsedSearchCpuSeconds already contains in-flight work. Include
        // that work only in the projected component numerator; only future
        // work is added to the projected search denominator.
        const double consumedIncludingInFlight =
            guardableCpuSeconds() + unrecordedElapsedCpuSeconds;
        const bool withinSharedTarget =
            projectedCpuShareWithinTarget(
                consumedIncludingInFlight,
                expectedFutureCpuSeconds,
                elapsedSearchCpuSeconds,
                targetShare_);
        if (withinSharedTarget ||
            scope != SpScope::Global ||
            globalReserveShare <= 0.0) {
            return withinSharedTarget;
        }

        // Only Global calls may reach this enlarged family cap. Global work
        // already paid from the shared target must not be charged against the
        // reserve a second time; the reserve is precisely the family share
        // above targetShare_.
        return projectedCpuShareWithinTarget(
            consumedIncludingInFlight,
            expectedFutureCpuSeconds,
            elapsedSearchCpuSeconds,
            targetShare_ + globalReserveShare);
    }

    bool canSpendPhase(
            const SpBudgetPhase phase,
            const double expectedFutureCpuSeconds,
            const double elapsedSearchCpuSeconds,
            const bool hardMandatory,
            const double mipRescueReserveShare = 0.0,
            const double unrecordedFamilyCpuSeconds = 0.0,
            const double unrecordedPhaseCpuSeconds = 0.0) const {
        if (!std::isfinite(mipRescueReserveShare) ||
            mipRescueReserveShare < 0.0 ||
            mipRescueReserveShare > 1.0) {
            throw std::invalid_argument(
                "SP MIP rescue reserve must be in [0,1]");
        }
        if (hardMandatory || phase == SpBudgetPhase::FinalGlobal) {
            return true;
        }
        const bool mipPhase =
            phase == SpBudgetPhase::LocalMip ||
            phase == SpBudgetPhase::GlobalMip;
        const double phaseShare = phaseTargetShare(phase);
        const double enlargedPhaseShare =
            std::min(
                1.0,
                phaseShare +
                    (mipPhase ? mipRescueReserveShare : 0.0));
        if (!projectedCpuShareWithinTarget(
                phaseCpuSecondsForGuard(phase) +
                    unrecordedPhaseCpuSeconds,
                expectedFutureCpuSeconds,
                elapsedSearchCpuSeconds,
                enlargedPhaseShare)) {
            return false;
        }
        if (mipPhase && mipRescueReserveShare > 0.0) {
            return projectedCpuShareWithinTarget(
                guardableCpuSeconds() +
                    unrecordedFamilyCpuSeconds,
                expectedFutureCpuSeconds,
                elapsedSearchCpuSeconds,
                std::min(
                    1.0,
                    targetShare_ + mipRescueReserveShare));
        }
        return canSpend(
            phaseScope(phase),
            expectedFutureCpuSeconds,
            elapsedSearchCpuSeconds,
            false,
            0.0,
            unrecordedFamilyCpuSeconds);
    }

    double localCpuSeconds() const noexcept {
        return localCpuSeconds_;
    }
    double globalCpuSeconds() const noexcept {
        return globalCpuSeconds_;
    }
    double totalCpuSeconds() const noexcept {
        return localCpuSeconds_ +
            globalCpuSeconds_;
    }
    double guardableCpuSeconds() const noexcept {
        return guardableLocalCpuSeconds_ +
            guardableGlobalCpuSeconds_;
    }
    double mandatoryExemptCpuSeconds() const noexcept {
        return mandatoryExemptCpuSeconds_;
    }
    double guardableLocalCpuSeconds() const noexcept {
        return guardableLocalCpuSeconds_;
    }
    double guardableGlobalCpuSeconds() const noexcept {
        return guardableGlobalCpuSeconds_;
    }
    double observedShare(
            const double elapsedSearchCpuSeconds) const noexcept {
        return elapsedSearchCpuSeconds > 0.0
            ? totalCpuSeconds() / elapsedSearchCpuSeconds
            : 0.0;
    }
    double guardableShare(
            const double elapsedSearchCpuSeconds) const noexcept {
        return elapsedSearchCpuSeconds > 0.0
            ? guardableCpuSeconds() / elapsedSearchCpuSeconds
            : 0.0;
    }
    double mandatoryExemptShare(
            const double elapsedSearchCpuSeconds) const noexcept {
        return elapsedSearchCpuSeconds > 0.0
            ? mandatoryExemptCpuSeconds_ / elapsedSearchCpuSeconds
            : 0.0;
    }
    double phaseCpuSeconds(
            const SpBudgetPhase phase) const noexcept {
        switch (phase) {
        case SpBudgetPhase::LpProbe:
            return lpProbeCpuSeconds_;
        case SpBudgetPhase::LocalMip:
            return localMipCpuSeconds_;
        case SpBudgetPhase::GlobalMip:
            return globalMipCpuSeconds_;
        case SpBudgetPhase::FinalGlobal:
            return finalGlobalCpuSeconds_;
        }
        return 0.0;
    }
    double guardablePhaseCpuSeconds(
            const SpBudgetPhase phase) const noexcept {
        return phaseCpuSecondsForGuard(phase);
    }
    double observedPhaseShare(
            const SpBudgetPhase phase,
            const double elapsedSearchCpuSeconds) const noexcept {
        return elapsedSearchCpuSeconds > 0.0
            ? phaseCpuSeconds(phase) / elapsedSearchCpuSeconds
            : 0.0;
    }
    double guardablePhaseShare(
            const SpBudgetPhase phase,
            const double elapsedSearchCpuSeconds) const noexcept {
        return elapsedSearchCpuSeconds > 0.0
            ? guardablePhaseCpuSeconds(phase) /
                  elapsedSearchCpuSeconds
            : 0.0;
    }
    double configuredPhaseTargetShare(
            const SpBudgetPhase phase) const noexcept {
        return phaseTargetShare(phase);
    }
    double lastPhaseCpuSeconds(
            const SpBudgetPhase phase) const noexcept {
        switch (phase) {
        case SpBudgetPhase::LpProbe:
            return lastLpProbeCpuSeconds_;
        case SpBudgetPhase::LocalMip:
            return lastLocalMipCpuSeconds_;
        case SpBudgetPhase::GlobalMip:
            return lastGlobalMipCpuSeconds_;
        case SpBudgetPhase::FinalGlobal:
            return lastFinalGlobalCpuSeconds_;
        }
        return 0.0;
    }
    double lastCpuSeconds(const SpScope scope) const noexcept {
        switch (scope) {
        case SpScope::Local:
            return lastLocalCpuSeconds_;
        case SpScope::Global:
            return lastGlobalCpuSeconds_;
        }
        return 0.0;
    }
    double share(const double elapsedSearchCpuSeconds) const noexcept {
        return observedShare(elapsedSearchCpuSeconds);
    }

private:
    double phaseCpuSecondsForGuard(
            const SpBudgetPhase phase) const noexcept {
        switch (phase) {
        case SpBudgetPhase::LpProbe:
            return guardableLpProbeCpuSeconds_;
        case SpBudgetPhase::LocalMip:
            return guardableLocalMipCpuSeconds_;
        case SpBudgetPhase::GlobalMip:
            return guardableGlobalMipCpuSeconds_;
        case SpBudgetPhase::FinalGlobal:
            return guardableFinalGlobalCpuSeconds_;
        }
        return 0.0;
    }

    static SpScope phaseScope(
            const SpBudgetPhase phase) noexcept {
        switch (phase) {
        case SpBudgetPhase::LpProbe:
        case SpBudgetPhase::LocalMip:
            return SpScope::Local;
        case SpBudgetPhase::GlobalMip:
        case SpBudgetPhase::FinalGlobal:
            return SpScope::Global;
        }
        return SpScope::Global;
    }

    double phaseTargetShare(
            const SpBudgetPhase phase) const noexcept {
        switch (phase) {
        case SpBudgetPhase::LpProbe:
            return lpProbeTargetShare_;
        case SpBudgetPhase::LocalMip:
            return localMipTargetShare_;
        case SpBudgetPhase::GlobalMip:
            return globalMipTargetShare_;
        case SpBudgetPhase::FinalGlobal:
            return 1.0;
        }
        return 0.0;
    }

    double targetShare_ = 0.15;
    double lpProbeTargetShare_ = 0.04;
    double localMipTargetShare_ = 0.02;
    double globalMipTargetShare_ = 0.05;
    double localCpuSeconds_ = 0.0;
    double globalCpuSeconds_ = 0.0;
    double guardableLocalCpuSeconds_ = 0.0;
    double guardableGlobalCpuSeconds_ = 0.0;
    double mandatoryExemptCpuSeconds_ = 0.0;
    double lastLocalCpuSeconds_ = 0.0;
    double lastGlobalCpuSeconds_ = 0.0;
    double lpProbeCpuSeconds_ = 0.0;
    double localMipCpuSeconds_ = 0.0;
    double globalMipCpuSeconds_ = 0.0;
    double finalGlobalCpuSeconds_ = 0.0;
    double guardableLpProbeCpuSeconds_ = 0.0;
    double guardableLocalMipCpuSeconds_ = 0.0;
    double guardableGlobalMipCpuSeconds_ = 0.0;
    double guardableFinalGlobalCpuSeconds_ = 0.0;
    double lastLpProbeCpuSeconds_ = 0.0;
    double lastLocalMipCpuSeconds_ = 0.0;
    double lastGlobalMipCpuSeconds_ = 0.0;
    double lastFinalGlobalCpuSeconds_ = 0.0;
};

}  // namespace hgasp

#endif  // HGASP_CORE_SP_BUDGET_LEDGER_H_
