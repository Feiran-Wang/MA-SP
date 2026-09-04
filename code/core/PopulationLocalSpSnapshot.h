#ifndef HGASP_CORE_POPULATION_LOCAL_SP_SNAPSHOT_H_
#define HGASP_CORE_POPULATION_LOCAL_SP_SNAPSHOT_H_

#include "RouteKey.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>

namespace hgasp {

// Stable identity of one population lifetime. Slots may be reused only after
// incrementing generation; this keeps an asynchronous Local-SP result from
// being applied to a replacement population.
struct PopulationLocalSpToken {
    std::size_t slot = 0U;
    std::uint64_t generation = 0U;

    bool operator==(
            const PopulationLocalSpToken& other) const noexcept {
        return slot == other.slot && generation == other.generation;
    }

    bool operator!=(
            const PopulationLocalSpToken& other) const noexcept {
        return !(*this == other);
    }

    bool operator<(
            const PopulationLocalSpToken& other) const noexcept {
        return slot != other.slot
            ? slot < other.slot
            : generation < other.generation;
    }
};

// Immutable view consumed by one Local-SP call. Counter deltas decide whether
// an exhausted view has changed meaningfully; the complete structure decides
// whether a result was solved against the still-current view. In particular,
// rawMasterVersion is never sufficient to reactivate a population.
struct PopulationLocalSpSnapshot {
    PopulationLocalSpToken population;
    DepotConfigurationKey configuration;
    double populationBestObjective = 0.0;
    std::uint64_t configurationAppendCount = 0U;
    std::uint64_t configurationRepresentativeReplacementCount = 0U;
    double configurationRepresentativeReplacementGain = 0.0;
    std::uint64_t eligibleRouteCoverageSignature = 0U;
    std::uint64_t eligibleRouteCoverageVersion = 0U;
    std::uint64_t rawMasterVersion = 0U;

    bool operator==(
            const PopulationLocalSpSnapshot& other) const noexcept {
        return population == other.population &&
            configuration == other.configuration &&
            populationBestObjective == other.populationBestObjective &&
            configurationAppendCount ==
                other.configurationAppendCount &&
            configurationRepresentativeReplacementCount ==
                other.configurationRepresentativeReplacementCount &&
            configurationRepresentativeReplacementGain ==
                other.configurationRepresentativeReplacementGain &&
            eligibleRouteCoverageSignature ==
                other.eligibleRouteCoverageSignature &&
            eligibleRouteCoverageVersion ==
                other.eligibleRouteCoverageVersion &&
            rawMasterVersion == other.rawMasterVersion;
    }

    bool operator!=(
            const PopulationLocalSpSnapshot& other) const noexcept {
        return !(*this == other);
    }
};

inline bool isCanonicalPopulationLocalSpConfiguration(
        const DepotConfigurationKey& configuration) noexcept {
    if (configuration.openDepots.empty()) {
        return false;
    }
    for (std::size_t index = 0U;
         index < configuration.openDepots.size();
         ++index) {
        if (configuration.openDepots[index] < 0 ||
            (index > 0U &&
             configuration.openDepots[index - 1U] >=
                 configuration.openDepots[index])) {
            return false;
        }
    }
    return true;
}

inline void validatePopulationLocalSpSnapshot(
        const PopulationLocalSpSnapshot& snapshot) {
    if (snapshot.population.generation == 0U ||
        !isCanonicalPopulationLocalSpConfiguration(
            snapshot.configuration) ||
        !std::isfinite(snapshot.populationBestObjective) ||
        snapshot.populationBestObjective < 0.0 ||
        !std::isfinite(
            snapshot.configurationRepresentativeReplacementGain) ||
        snapshot.configurationRepresentativeReplacementGain < 0.0) {
        throw std::invalid_argument(
            "invalid population Local-SP snapshot");
    }
}

namespace detail {

inline std::uint64_t populationLocalSpHashWord(
        std::uint64_t hash,
        const std::uint64_t word) noexcept {
    // FNV-1a is used only for stable diagnostics. Controller identity and
    // stale-snapshot checks always compare the complete typed fields above.
    for (unsigned int byte = 0U; byte < 8U; ++byte) {
        hash ^= (word >> (byte * 8U)) & 0xffU;
        hash *= 1099511628211ULL;
    }
    return hash;
}

inline std::uint64_t populationLocalSpDoubleBits(
        const double value) noexcept {
    const double normalized = value == 0.0 ? 0.0 : value;
    std::uint64_t bits = 0U;
    std::memcpy(&bits, &normalized, sizeof(bits));
    return bits;
}

}  // namespace detail

inline std::uint64_t populationLocalSpSnapshotId(
        const PopulationLocalSpSnapshot& snapshot) noexcept {
    std::uint64_t hash = 1469598103934665603ULL;
    hash = detail::populationLocalSpHashWord(
        hash, static_cast<std::uint64_t>(snapshot.population.slot));
    hash = detail::populationLocalSpHashWord(
        hash, snapshot.population.generation);
    hash = detail::populationLocalSpHashWord(
        hash,
        static_cast<std::uint64_t>(
            snapshot.configuration.openDepots.size()));
    for (const int depot : snapshot.configuration.openDepots) {
        hash = detail::populationLocalSpHashWord(
            hash, static_cast<std::uint64_t>(depot));
    }
    hash = detail::populationLocalSpHashWord(
        hash,
        detail::populationLocalSpDoubleBits(
            snapshot.populationBestObjective));
    hash = detail::populationLocalSpHashWord(
        hash, snapshot.configurationAppendCount);
    hash = detail::populationLocalSpHashWord(
        hash,
        snapshot.configurationRepresentativeReplacementCount);
    hash = detail::populationLocalSpHashWord(
        hash,
        detail::populationLocalSpDoubleBits(
            snapshot.configurationRepresentativeReplacementGain));
    hash = detail::populationLocalSpHashWord(
        hash, snapshot.eligibleRouteCoverageSignature);
    hash = detail::populationLocalSpHashWord(
        hash, snapshot.eligibleRouteCoverageVersion);
    return detail::populationLocalSpHashWord(
        hash, snapshot.rawMasterVersion);
}

}  // namespace hgasp

#endif  // HGASP_CORE_POPULATION_LOCAL_SP_SNAPSHOT_H_
