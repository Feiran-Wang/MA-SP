#ifndef HGASP_CORE_STATIC_CONFIGURATION_REGISTRY_H_
#define HGASP_CORE_STATIC_CONFIGURATION_REGISTRY_H_

#include "RouteKey.h"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

namespace hgasp {

// Stable identity of a fixed-configuration population. The unrestricted
// population deliberately has no configuration token.
struct StaticPopulationToken {
    std::size_t slot = 0U;
    std::uint64_t generation = 0U;

    bool operator==(
            const StaticPopulationToken& other) const noexcept {
        return slot == other.slot && generation == other.generation;
    }

    bool operator!=(
            const StaticPopulationToken& other) const noexcept {
        return !(*this == other);
    }
};

struct StaticPopulationMetrics {
    double bestObjective = std::numeric_limits<double>::infinity();
    double meanObjective = std::numeric_limits<double>::infinity();
    double diversity = 0.0;
    std::uint64_t lastImprovementAttempt = 0U;
    double hgaImprovementPerCpu = 0.0;
    double globalSpSelectedColumnCredit = 0.0;
};

struct StaticConfigurationSlotState {
    std::size_t slot = 0U;
    std::uint64_t generation = 0U;
    bool active = false;
    DepotConfigurationKey configuration;
    std::uint64_t createdAttempt = 0U;
    std::uint64_t protectedUntilAttempt = 0U;
    StaticPopulationMetrics metrics;

    StaticPopulationToken token() const noexcept {
        return {slot, generation};
    }
};

enum class StaticPopulationTargetKind {
    Fixed,
    Unrestricted,
};

struct StaticPopulationTarget {
    StaticPopulationTargetKind kind =
        StaticPopulationTargetKind::Unrestricted;
    std::size_t populationIndex = 0U;
    std::optional<StaticPopulationToken> fixedToken;

    bool isFixed() const noexcept {
        return kind == StaticPopulationTargetKind::Fixed;
    }

    bool isUnrestricted() const noexcept {
        return kind == StaticPopulationTargetKind::Unrestricted;
    }
};

// Replacement authority is explicit: an HGA challenger is admitted only after
// its protected trial and resident-quality gate, whereas an independently
// validated strict Global-SP incumbent is stronger evidence and may bypass
// resident protection.
enum class StaticReplacementAuthority {
    HgaChallengerValidated,
    GlobalSpStrictIncumbent,
    ScheduledExplorationLease,
};

struct StaticConfigurationReplacementRequest {
    DepotConfigurationKey configuration;
    StaticReplacementAuthority authority =
        StaticReplacementAuthority::GlobalSpStrictIncumbent;
    bool independentlyValidated = false;
    bool challengerTrialCompleted = false;
    bool beatsWeakestResident = false;
    bool strictlyImprovesPriorGlobal = false;
    std::uint64_t attempt = 0U;
    std::uint64_t protectionAttempts = 0U;
};

// Owns only logical population identities and scalar state. Population
// objects and Individuals remain owned by popManager.
class StaticConfigurationRegistry {
public:
    explicit StaticConfigurationRegistry(
        std::size_t requestedFixedPopulations = 15U,
        std::optional<std::size_t> depotCount = std::nullopt);

    void initialize(
        const std::vector<DepotConfigurationKey>& rankedConfigurations,
        std::uint64_t createdAttempt = 0U,
        std::uint64_t protectionAttempts = 0U);

    std::size_t requestedFixedPopulationCount() const noexcept {
        return requestedFixedPopulations_;
    }

    std::size_t actualFixedPopulationCount() const noexcept;

    std::size_t fixedSlotCount() const noexcept {
        return slots_.size();
    }

    // The unrestricted population is laid out after all physical fixed slots.
    // Reductions deactivate fixed slots without changing this stable index.
    std::size_t unrestrictedPopulationIndex() const noexcept {
        return slots_.size();
    }

    const std::vector<StaticConfigurationSlotState>& slots() const noexcept {
        return slots_;
    }

    std::vector<StaticPopulationToken> activeTokens() const;

    std::optional<StaticPopulationToken> tokenForConfiguration(
        const DepotConfigurationKey& configuration) const;

    StaticPopulationTarget classify(
        const DepotConfigurationKey& actualConfiguration) const;

    bool isCurrent(StaticPopulationToken token) const noexcept;

    const StaticConfigurationSlotState& state(
        StaticPopulationToken token) const;

    // Critical runtime invariant for fixed-population admission.
    void requireMatchingConfiguration(
        StaticPopulationToken token,
        const DepotConfigurationKey& actualConfiguration) const;

    void updateMetrics(
        StaticPopulationToken token,
        const StaticPopulationMetrics& metrics);

    StaticPopulationToken replaceConfiguration(
        StaticPopulationToken target,
        const StaticConfigurationReplacementRequest& request);

    void deactivate(StaticPopulationToken token);

    // Restart preserves configurations while invalidating every prior token.
    void restartActive(
        std::uint64_t createdAttempt,
        std::uint64_t protectionAttempts = 0U);

    // A prepared registry copy is committed with this no-throw swap only
    // after the successor population and every allocating metadata update have
    // succeeded.
    void swap(StaticConfigurationRegistry& other) noexcept;

private:
    DepotConfigurationKey validateAndCanonicalize(
        const DepotConfigurationKey& configuration) const;

    StaticConfigurationSlotState& mutableState(
        StaticPopulationToken token);

    std::size_t requestedFixedPopulations_ = 15U;
    std::optional<std::size_t> depotCount_;
    std::vector<StaticConfigurationSlotState> slots_;
    bool initialized_ = false;
};

}  // namespace hgasp

#endif  // HGASP_CORE_STATIC_CONFIGURATION_REGISTRY_H_
