#include "PromisingArcCandidates.h"

#include "SolverContext.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace hgasp {
namespace {

double directedCorrelation(
        const SolverContext& data,
        const int from,
        const int to,
        const double temporalPenaltyWeight,
        const double waitingTimeWeight) {
    const double travelTime = data.T[from][to];
    const double waiting = std::max(
        0.0,
        data.readyTime[static_cast<std::size_t>(to)] -
            data.serviceTime[static_cast<std::size_t>(from)] - travelTime -
            data.dueTime[static_cast<std::size_t>(from)]);
    const double timeWarp = std::max(
        0.0,
        data.readyTime[static_cast<std::size_t>(from)] +
            data.serviceTime[static_cast<std::size_t>(from)] + travelTime -
            data.dueTime[static_cast<std::size_t>(to)]);
    return data.D[from][to] + waitingTimeWeight * waiting +
        temporalPenaltyWeight * timeWarp;
}

}  // namespace

bool PromisingArcCandidateGraph::containsCustomerArc(
        const int from,
        const int to) const noexcept {
    return from >= 0 && to >= 0 && from < customerCount &&
        to < customerCount && containsArc(from, to);
}

bool PromisingArcCandidateGraph::containsDepotArc(
        const int customer,
        const int depot) const noexcept {
    return customer >= 0 && depot >= 0 && customer < customerCount &&
        depot < depotCount && containsArc(customer, customerCount + depot);
}

PromisingArcCandidateGraph buildPromisingArcCandidateGraph(
        const SolverContext& data,
        const int customerLimit,
        const int depotLimit,
        const double temporalPenaltyWeight,
        const double waitingTimeWeight) {
    if (customerLimit < 0 || depotLimit < 0 ||
        !std::isfinite(temporalPenaltyWeight) ||
        !std::isfinite(waitingTimeWeight) || temporalPenaltyWeight < 0.0 ||
        waitingTimeWeight < 0.0) {
        throw std::invalid_argument("invalid promising-arc policy");
    }
    PromisingArcCandidateGraph graph;
    const std::size_t customers = static_cast<std::size_t>(data.numV);
    const std::size_t depots = static_cast<std::size_t>(data.numL);
    graph.customerCount = data.numV;
    graph.depotCount = data.numL;
    graph.vertexCount = data.numC;
    graph.customerCandidates.resize(customers);
    graph.depotCandidates.resize(customers);
    graph.arcMembership.assign(
        static_cast<std::size_t>(graph.vertexCount) *
            static_cast<std::size_t>(graph.vertexCount),
        0U);
    const auto markArc = [&graph](const int from, const int to) {
        graph.arcMembership[static_cast<std::size_t>(from) *
                static_cast<std::size_t>(graph.vertexCount) +
                static_cast<std::size_t>(to)] = 1U;
    };

    for (int customer = 0; customer < data.numV; ++customer) {
        std::vector<std::pair<double, int>> rankedCustomers;
        rankedCustomers.reserve(customers > 0U ? customers - 1U : 0U);
        for (int other = 0; other < data.numV; ++other) {
            if (other == customer) continue;
            rankedCustomers.emplace_back(
                directedCorrelation(
                    data,
                    customer,
                    other,
                    temporalPenaltyWeight,
                    waitingTimeWeight),
                other);
        }
        std::sort(rankedCustomers.begin(), rankedCustomers.end());
        const std::size_t customerCount = std::min(
            static_cast<std::size_t>(customerLimit),
            rankedCustomers.size());
        auto& selectedCustomers = graph.customerCandidates[
            static_cast<std::size_t>(customer)];
        selectedCustomers.reserve(customerCount);
        for (std::size_t index = 0U; index < customerCount; ++index) {
            const int other = rankedCustomers[index].second;
            selectedCustomers.push_back(other);
            markArc(customer, other);
        }
        graph.maximumCustomerCandidates = std::max(
            graph.maximumCustomerCandidates, selectedCustomers.size());

        std::vector<std::pair<double, int>> rankedDepots;
        rankedDepots.reserve(depots);
        for (int depot = 0; depot < data.numL; ++depot) {
            const int depotNode = data.numV + depot;
            if (data.demCus[customer] > data.invLoc[depotNode]) continue;
            rankedDepots.emplace_back(data.D[customer][depotNode], depotNode);
        }
        std::sort(rankedDepots.begin(), rankedDepots.end());
        const std::size_t depotCount = std::min(
            static_cast<std::size_t>(depotLimit), rankedDepots.size());
        auto& selectedDepots = graph.depotCandidates[
            static_cast<std::size_t>(customer)];
        selectedDepots.reserve(depotCount);
        for (std::size_t index = 0U; index < depotCount; ++index) {
            const int depotNode = rankedDepots[index].second;
            selectedDepots.push_back(depotNode);
            // Depot candidates are an HGA-SP adaptation and historically
            // match either orientation of the customer/depot boundary.
            markArc(customer, depotNode);
            markArc(depotNode, customer);
        }
        graph.maximumDepotCandidates = std::max(
            graph.maximumDepotCandidates, selectedDepots.size());
    }
    return graph;
}

}  // namespace hgasp
