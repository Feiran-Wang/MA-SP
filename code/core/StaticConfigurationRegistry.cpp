#include "StaticConfigurationRegistry.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace hgasp {
namespace {

std::uint64_t protectedUntil(
        const std::uint64_t attempt,
        const std::uint64_t protectionAttempts) noexcept {
    if (protectionAttempts >
        std::numeric_limits<std::uint64_t>::max() - attempt) {
        return std::numeric_limits<std::uint64_t>::max();
    }
    return attempt + protectionAttempts;
}

void validateMetrics(const StaticPopulationMetrics& metrics) {
    if (!std::isfinite(metrics.bestObjective) ||
        !std::isfinite(metrics.meanObjective) ||
        !std::isfinite(metrics.diversity) ||
        !std::isfinite(metrics.hgaImprovementPerCpu) ||
        !std::isfinite(metrics.globalSpSelectedColumnCredit) ||
        metrics.diversity < 0.0 ||
        metrics.hgaImprovementPerCpu < 0.0 ||
        metrics.globalSpSelectedColumnCredit < 0.0) {
        throw std::invalid_argument(
            "static population metrics must be finite and non-negative "
            "outside objective fields");
    }
}

}  // namespace

StaticConfigurationRegistry::StaticConfigurationRegistry(
        const std::size_t requestedFixedPopulations,
        const std::optional<std::size_t> depotCount)
    : requestedFixedPopulations_(requestedFixedPopulations),
      depotCount_(depotCount) {
    if (depotCount_.has_value() && *depotCount_ == 0U) {
        throw std::invalid_argument(
            "static registry depot count must be positive when provided");
    }
}

DepotConfigurationKey
StaticConfigurationRegistry::validateAndCanonicalize(
        const DepotConfigurationKey& configuration) const {
    DepotConfigurationKey canonical =
        makeDepotConfigurationKey(configuration.openDepots);
    if (canonical.openDepots.empty()) {
        throw std::invalid_argument(
            "fixed population configuration opens no depot");
    }
    if (depotCount_.has_value()) {
        for (const int depot : canonical.openDepots) {
            if (static_cast<std::size_t>(depot) >= *depotCount_) {
                throw std::invalid_argument(
                    "fixed population configuration has an out-of-range "
                    "depot");
            }
        }
    }
    return canonical;
}

void StaticConfigurationRegistry::initialize(
        const std::vector<DepotConfigurationKey>& rankedConfigurations,
        const std::uint64_t createdAttempt,
        const std::uint64_t protectionAttempts) {
    if (initialized_) {
        throw std::logic_error(
            "static registry cannot be initialized twice; use restartActive");
    }
    initialized_ = true;
    slots_.clear();
    slots_.reserve(std::min(
        requestedFixedPopulations_, rankedConfigurations.size()));
    std::set<DepotConfigurationKey> seen;
    for (const DepotConfigurationKey& candidate : rankedConfigurations) {
        if (slots_.size() >= requestedFixedPopulations_) {
            break;
        }
        DepotConfigurationKey canonical =
            validateAndCanonicalize(candidate);
        if (!seen.insert(canonical).second) {
            continue;
        }
        StaticConfigurationSlotState state;
        state.slot = slots_.size();
        state.generation = 1U;
        state.active = true;
        state.configuration = std::move(canonical);
        state.createdAttempt = createdAttempt;
        state.protectedUntilAttempt = protectedUntil(
            createdAttempt, protectionAttempts);
        slots_.push_back(std::move(state));
    }
}

std::size_t
StaticConfigurationRegistry::actualFixedPopulationCount() const noexcept {
    return static_cast<std::size_t>(std::count_if(
        slots_.begin(),
        slots_.end(),
        [](const StaticConfigurationSlotState& state) {
            return state.active;
        }));
}

std::vector<StaticPopulationToken>
StaticConfigurationRegistry::activeTokens() const {
    std::vector<StaticPopulationToken> tokens;
    tokens.reserve(actualFixedPopulationCount());
    for (const StaticConfigurationSlotState& state : slots_) {
        if (state.active) {
            tokens.push_back(state.token());
        }
    }
    return tokens;
}

std::optional<StaticPopulationToken>
StaticConfigurationRegistry::tokenForConfiguration(
        const DepotConfigurationKey& configuration) const {
    const DepotConfigurationKey canonical =
        validateAndCanonicalize(configuration);
    for (const StaticConfigurationSlotState& state : slots_) {
        if (state.active && state.configuration == canonical) {
            return state.token();
        }
    }
    return std::nullopt;
}

StaticPopulationTarget StaticConfigurationRegistry::classify(
        const DepotConfigurationKey& actualConfiguration) const {
    const std::optional<StaticPopulationToken> token =
        tokenForConfiguration(actualConfiguration);
    if (!token.has_value()) {
        return {
            StaticPopulationTargetKind::Unrestricted,
            unrestrictedPopulationIndex(),
            std::nullopt,
        };
    }
    return {
        StaticPopulationTargetKind::Fixed,
        token->slot,
        token,
    };
}

bool StaticConfigurationRegistry::isCurrent(
        const StaticPopulationToken token) const noexcept {
    return token.slot < slots_.size() &&
        slots_[token.slot].active &&
        slots_[token.slot].generation == token.generation;
}

const StaticConfigurationSlotState&
StaticConfigurationRegistry::state(
        const StaticPopulationToken token) const {
    if (!isCurrent(token)) {
        throw std::logic_error(
            "stale or inactive static population token");
    }
    return slots_[token.slot];
}

void StaticConfigurationRegistry::requireMatchingConfiguration(
        const StaticPopulationToken token,
        const DepotConfigurationKey& actualConfiguration) const {
    const StaticConfigurationSlotState& slotState = state(token);
    const DepotConfigurationKey canonical =
        validateAndCanonicalize(actualConfiguration);
    if (canonical != slotState.configuration) {
        throw std::logic_error(
            "configuration mismatch on fixed population admission");
    }
}

StaticConfigurationSlotState&
StaticConfigurationRegistry::mutableState(
        const StaticPopulationToken token) {
    if (!isCurrent(token)) {
        throw std::logic_error(
            "stale or inactive static population token");
    }
    return slots_[token.slot];
}

void StaticConfigurationRegistry::updateMetrics(
        const StaticPopulationToken token,
        const StaticPopulationMetrics& metrics) {
    validateMetrics(metrics);
    mutableState(token).metrics = metrics;
}

StaticPopulationToken
StaticConfigurationRegistry::replaceConfiguration(
        const StaticPopulationToken target,
        const StaticConfigurationReplacementRequest& request) {
    StaticConfigurationSlotState& targetState = mutableState(target);
    if (!request.independentlyValidated) {
        throw std::invalid_argument(
            "static replacement requires independent validation");
    }
    switch (request.authority) {
    case StaticReplacementAuthority::HgaChallengerValidated:
        if (!request.challengerTrialCompleted ||
            !request.beatsWeakestResident) {
            throw std::invalid_argument(
                "HGA challenger replacement requires a completed trial "
                "and weakest-resident quality gate");
        }
        if (request.strictlyImprovesPriorGlobal) {
            throw std::invalid_argument(
                "HGA challenger authority cannot claim Global-SP strict "
                "incumbent evidence");
        }
        if (request.attempt < targetState.protectedUntilAttempt) {
            throw std::logic_error(
                "HGA challenger cannot replace a protected resident");
        }
        break;
    case StaticReplacementAuthority::GlobalSpStrictIncumbent:
        if (!request.strictlyImprovesPriorGlobal) {
            throw std::invalid_argument(
                "Global-SP replacement requires strict prior-global "
                "improvement");
        }
        if (request.challengerTrialCompleted ||
            request.beatsWeakestResident) {
            throw std::invalid_argument(
                "Global-SP authority cannot claim HGA trial evidence");
        }
        // A validated strict Global-SP incumbent deliberately bypasses
        // resident age/protection; this is the higher-confidence authority.
        break;
    case StaticReplacementAuthority::ScheduledExplorationLease:
        if (request.strictlyImprovesPriorGlobal ||
            request.challengerTrialCompleted ||
            request.beatsWeakestResident) {
            throw std::invalid_argument(
                "scheduled exploration replacement cannot claim Global-SP "
                "improvement or challenger-trial evidence");
        }
        // Checkpoint ownership is the evidence boundary. The leased slot may
        // intentionally explore a weaker founder and therefore bypasses the
        // resident age and quality gates.
        break;
    }
    DepotConfigurationKey canonical =
        validateAndCanonicalize(request.configuration);
    if (canonical == targetState.configuration ||
        tokenForConfiguration(canonical).has_value()) {
        throw std::invalid_argument(
            "static replacement configuration is already registered");
    }
    if (targetState.generation ==
        std::numeric_limits<std::uint64_t>::max()) {
        throw std::overflow_error(
            "static population generation overflow");
    }
    ++targetState.generation;
    targetState.configuration = std::move(canonical);
    targetState.createdAttempt = request.attempt;
    targetState.protectedUntilAttempt = protectedUntil(
        request.attempt, request.protectionAttempts);
    targetState.metrics = {};
    return targetState.token();
}

void StaticConfigurationRegistry::deactivate(
        const StaticPopulationToken token) {
    StaticConfigurationSlotState& state = mutableState(token);
    if (state.generation ==
        std::numeric_limits<std::uint64_t>::max()) {
        throw std::overflow_error(
            "static population generation overflow");
    }
    ++state.generation;
    state.active = false;
    state.metrics = {};
}

void StaticConfigurationRegistry::restartActive(
        const std::uint64_t createdAttempt,
        const std::uint64_t protectionAttempts) {
    for (StaticConfigurationSlotState& state : slots_) {
        if (!state.active) {
            continue;
        }
        if (state.generation ==
            std::numeric_limits<std::uint64_t>::max()) {
            throw std::overflow_error(
                "static population generation overflow");
        }
        ++state.generation;
        state.createdAttempt = createdAttempt;
        state.protectedUntilAttempt = protectedUntil(
            createdAttempt, protectionAttempts);
        state.metrics = {};
    }
}

void StaticConfigurationRegistry::swap(
        StaticConfigurationRegistry& other) noexcept {
    using std::swap;
    swap(requestedFixedPopulations_, other.requestedFixedPopulations_);
    swap(depotCount_, other.depotCount_);
    slots_.swap(other.slots_);
    swap(initialized_, other.initialized_);
}

}  // namespace hgasp
