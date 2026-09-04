#ifndef HGASP_CORE_SP_CONTRIBUTION_ATTRIBUTION_H_
#define HGASP_CORE_SP_CONTRIBUTION_ATTRIBUTION_H_

namespace hgasp {

enum class SpContribution {
    SpDirect,
    SpPlusLocalSearch,
    SpSeededLocalSearch,
    DiverseOnly,
    Rejected,
};

inline SpContribution classifySpContribution(
        const double globalBefore,
        const double spRaw,
        const double postLocalSearch,
        const bool admitted) noexcept {
    if (spRaw < globalBefore) {
        return postLocalSearch < spRaw
            ? SpContribution::SpPlusLocalSearch
            : SpContribution::SpDirect;
    }
    if (postLocalSearch < globalBefore) {
        return SpContribution::SpSeededLocalSearch;
    }
    return admitted
        ? SpContribution::DiverseOnly
        : SpContribution::Rejected;
}

inline const char* toString(
        const SpContribution contribution) noexcept {
    switch (contribution) {
        case SpContribution::SpDirect:
            return "SP_DIRECT";
        case SpContribution::SpPlusLocalSearch:
            return "SP_PLUS_LS";
        case SpContribution::SpSeededLocalSearch:
            return "SP_SEEDED_LS";
        case SpContribution::DiverseOnly:
            return "DIVERSE_ONLY";
        case SpContribution::Rejected:
            return "REJECTED";
    }
    return "REJECTED";
}

}  // namespace hgasp

#endif  // HGASP_CORE_SP_CONTRIBUTION_ATTRIBUTION_H_
