#ifndef HGASP_CORE_POPULATION_LOCAL_SP_CONTROLLER_H_
#define HGASP_CORE_POPULATION_LOCAL_SP_CONTROLLER_H_

#include "LocalSpFamilyBudget.h"
#include "PopulationLocalSpSnapshot.h"

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace hgasp {

enum class PopulationLocalSpPhase {
    Active,
    Eligible,
    Running,
    Improved,
    Exhausted,
    Retired,
};

const char* toString(PopulationLocalSpPhase phase) noexcept;

enum class PopulationLocalSpOutcomeKind {
    Improved,
    ExhaustedLpBound,
    ExhaustedMipOptimalNoImprovement,
    ExhaustedTimeLimitNoImprovement,
    ExhaustedTerminalNoImprovement,
    DeferredIncompleteCoverage,
    DeferredCpuBudget,
    DeferredEnvironmentFailure,
    DeferredStalePopulation,
    DeferredSnapshotChanged,
    DeferredUnsupportedStatus,
};

const char* toString(PopulationLocalSpOutcomeKind outcome) noexcept;

struct PopulationLocalSpPolicy {
    std::uint64_t stagnationAttempts = 10000U;
    std::uint64_t minimumAgeAttempts = 5000U;
    std::uint64_t minimumNewCustomerSets = 250U;
    double minimumReplacementGainRatio = 0.0001;
    double lpTimeLimitSeconds = 2.0;
    double mipTimeLimitSeconds = 3.0;
    std::size_t maximumCallsPerLifetime = 3U;
};

struct PopulationLocalSpObservation {
    PopulationLocalSpSnapshot snapshot;
    std::uint64_t currentAttempt = 0U;
    std::uint64_t createdAttempt = 0U;
    std::uint64_t lastPopulationBestImprovementAttempt = 0U;
    // Cumulative event count. A retained true/false pulse is intentionally
    // avoided so one injection can reactivate a snapshot at most once.
    std::uint64_t verifiedGlobalInjectionCount = 0U;
    bool populationActive = true;
    bool completeCustomerCoverage = false;
};

struct PopulationLocalSpDecision {
    PopulationLocalSpSnapshot snapshot;
    bool eligible = false;
    std::string reason = "not_observed";
    PopulationLocalSpPhase phase = PopulationLocalSpPhase::Active;
    std::uint64_t currentAttempt = 0U;
    std::uint64_t populationAgeAttempts = 0U;
    std::uint64_t stagnationAttempts = 0U;
    std::uint64_t newCustomerSets = 0U;
    std::uint64_t representativeReplacements = 0U;
    double replacementGainRatio = 0.0;
    std::uint64_t verifiedGlobalInjectionCount = 0U;
    std::size_t callCount = 0U;
    std::size_t productiveCallCount = 0U;
    std::optional<std::uint64_t> lastCallAttempt;
    double historicalDirectImprovement = 0.0;
    double lpTimeLimitSeconds = 0.0;
    double mipTimeLimitSeconds = 0.0;
    std::uint64_t snapshotId = 0U;
};

struct PopulationLocalSpOutcome {
    PopulationLocalSpOutcomeKind kind =
        PopulationLocalSpOutcomeKind::DeferredUnsupportedStatus;
    double outerCpuSeconds = 0.0;
    double directObjectiveImprovement = 0.0;
    double postLocalSearchImprovement = 0.0;
};

struct PopulationLocalSpCompletion {
    PopulationLocalSpOutcomeKind effectiveOutcome =
        PopulationLocalSpOutcomeKind::DeferredUnsupportedStatus;
    PopulationLocalSpPhase phase = PopulationLocalSpPhase::Active;
    bool snapshotConsumed = false;
    bool stalePopulation = false;
    bool staleSnapshot = false;
    std::uint64_t consumedSnapshotId = 0U;
};

struct PopulationLocalSpStateView {
    PopulationLocalSpToken population;
    DepotConfigurationKey configuration;
    PopulationLocalSpPhase phase = PopulationLocalSpPhase::Active;
    double populationBestAtLastCall = 0.0;
    std::uint64_t lastPopulationBestImprovementAttempt = 0U;
    std::uint64_t observedConfigurationAppendCount = 0U;
    std::uint64_t observedRepresentativeReplacementCount = 0U;
    double observedRepresentativeReplacementGain = 0.0;
    std::optional<std::uint64_t> lastCallAttempt;
    std::optional<PopulationLocalSpOutcomeKind> lastStatus;
    std::optional<PopulationLocalSpSnapshot> exhaustedSnapshot;
    std::size_t callCount = 0U;
    std::size_t productiveCallCount = 0U;
    std::size_t reactivationCount = 0U;
    double cpuSeconds = 0.0;
    double directImprovement = 0.0;
    std::string lastReactivationReason;
};

// Owns independent Local-SP state for each generation-qualified population.
// It decides and accounts; RoutePool export, LP/MIP execution, reconstruction,
// validation, LS, and admission remain coordinator responsibilities.
class PopulationLocalSpController {
public:
    explicit PopulationLocalSpController(
        PopulationLocalSpPolicy policy = {});

    void configure(PopulationLocalSpPolicy policy);

    PopulationLocalSpDecision decide(
        const PopulationLocalSpObservation& observation,
        const LocalSpFamilyDecision& familyDecision);

    std::optional<PopulationLocalSpDecision> selectCandidate(
        const std::vector<PopulationLocalSpDecision>& decisions) const;

    void start(const PopulationLocalSpDecision& decision);

    PopulationLocalSpCompletion complete(
        PopulationLocalSpToken population,
        const PopulationLocalSpSnapshot& currentSnapshot,
        const PopulationLocalSpOutcome& outcome);

    void retire(PopulationLocalSpToken population) noexcept;
    void retireAll() noexcept;

    std::optional<PopulationLocalSpStateView> state(
        PopulationLocalSpToken population) const;

    const PopulationLocalSpPolicy& policy() const noexcept {
        return policy_;
    }

private:
    struct State {
        PopulationLocalSpToken population;
        DepotConfigurationKey configuration;
        PopulationLocalSpPhase phase = PopulationLocalSpPhase::Active;
        std::uint64_t createdAttempt = 0U;
        double bestObservedObjective = 0.0;
        std::uint64_t lastPopulationBestImprovementAttempt = 0U;

        PopulationLocalSpSnapshot referenceSnapshot;
        std::uint64_t referenceGlobalInjectionCount = 0U;
        PopulationLocalSpSnapshot latestSnapshot;
        std::uint64_t latestGlobalInjectionCount = 0U;
        std::optional<PopulationLocalSpSnapshot> consumedSnapshot;
        std::optional<PopulationLocalSpSnapshot> runningSnapshot;
        std::uint64_t runningGlobalInjectionCount = 0U;
        bool meaningfulChangePending = false;

        double populationBestAtLastCall = 0.0;
        std::optional<std::uint64_t> lastCallAttempt;
        std::optional<PopulationLocalSpOutcomeKind> lastStatus;
        std::size_t callCount = 0U;
        std::size_t productiveCallCount = 0U;
        std::size_t reactivationCount = 0U;
        double cpuSeconds = 0.0;
        double directImprovement = 0.0;
        std::string lastReactivationReason;
    };

    static void validatePolicy(const PopulationLocalSpPolicy& policy);
    static void validateObservation(
        const PopulationLocalSpObservation& observation);
    static void validateOutcome(const PopulationLocalSpOutcome& outcome);
    static bool isExhaustedOutcome(
        PopulationLocalSpOutcomeKind outcome) noexcept;
    static bool isDeferredOutcome(
        PopulationLocalSpOutcomeKind outcome) noexcept;

    State& stateFor(const PopulationLocalSpObservation& observation);
    void retireOlderSlotGenerations(
        PopulationLocalSpToken population) noexcept;
    PopulationLocalSpDecision makeDecision(
        const PopulationLocalSpObservation& observation,
        const State& state) const;

    PopulationLocalSpPolicy policy_;
    std::map<PopulationLocalSpToken, State> states_;
};

}  // namespace hgasp

#endif  // HGASP_CORE_POPULATION_LOCAL_SP_CONTROLLER_H_
