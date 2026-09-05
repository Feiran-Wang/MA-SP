#ifndef HGASP_OPERATORS_PROMISING_ARC_CANDIDATES_H_
#define HGASP_OPERATORS_PROMISING_ARC_CANDIDATES_H_

#include <cassert>
#include <cstddef>
#include <vector>

class SolverContext;

namespace hgasp {

struct PromisingArcCandidateGraph {
    std::vector<std::vector<int>> customerCandidates;
    std::vector<std::vector<int>> depotCandidates;
    // Direct byte lookup is faster than packed-bit extraction for the very
    // high-frequency, effectively random M1--M10 probes. At the supported
    // instance scale the exact matrix remains bounded and cache-resident.
    std::vector<unsigned char> arcMembership;
    int customerCount = 0;
    int depotCount = 0;
    int vertexCount = 0;
    std::size_t maximumCustomerCandidates = 0U;
    std::size_t maximumDepotCandidates = 0U;

    inline bool containsArc(const int from, const int to) const noexcept {
        return from >= 0 && to >= 0 && from < vertexCount &&
            to < vertexCount && containsKnownArc(from, to);
    }
    inline bool containsKnownArc(const int from, const int to) const noexcept {
        assert(from >= 0 && to >= 0 && from < vertexCount &&
            to < vertexCount);
        return arcMembership[static_cast<std::size_t>(from) *
            static_cast<std::size_t>(vertexCount) +
            static_cast<std::size_t>(to)] != 0U;
    }
    bool containsCustomerArc(int from, int to) const noexcept;
    bool containsDepotArc(int customer, int depot) const noexcept;
};

PromisingArcCandidateGraph buildPromisingArcCandidateGraph(
    const SolverContext& data,
    int customerLimit,
    int depotLimit,
    double temporalPenaltyWeight,
    double waitingTimeWeight);

}  // namespace hgasp

#endif  // HGASP_OPERATORS_PROMISING_ARC_CANDIDATES_H_
