/******************************************************************************
 * File: LS.cpp
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description: 
 *   Local Search algorithms for route optimization and refinement.
 ******************************************************************************/

#include "LS.h"
#include "basic.h"
#include "ComponentProfiler.h"
#include "Individual.h"
#include "RouteEvaluator.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

#if defined(__GNUC__) || defined(__clang__)
#define HGASP_LS_ALWAYS_INLINE inline __attribute__((always_inline))
#else
#define HGASP_LS_ALWAYS_INLINE inline
#endif

namespace {

std::vector<StandaloneRoute> extractStandaloneRoutes(
		const Individual& individual,
		const SolverContext& data) {
	std::vector<StandaloneRoute> routes;
	for (int depot = 0; depot < data.numL; ++depot) {
		for (int slot = 0;
			 slot < individual.location[depot].maxVisit;
			 ++slot) {
			const Route* route =
				individual.location[depot].depots[slot].route;
			if (!routeIsOccupied(*route))
				continue;
			StandaloneRoute valueRoute;
			valueRoute.locId = depot;
			const Node* node =
				individual.location[depot].depots[slot].next;
			while (!node->isDepot) {
				valueRoute.sequence.push_back(node->cour);
				node = node->next;
			}
			if (!valueRoute.sequence.empty())
				routes.push_back(std::move(valueRoute));
		}
	}
	return routes;
}

bool evaluateTimeWindowRouteSet(
		const SolverContext& data,
		std::vector<StandaloneRoute>& routes,
		double& objective) {
	RouteEvaluator evaluator(
		data, data.problemContext().temporalConstraint());
	std::vector<int> depotLoads(
		static_cast<std::size_t>(data.numL), 0);
	std::vector<int> customerUse(
		static_cast<std::size_t>(data.numV), 0);
	std::set<int> openedDepots;
	objective =
		data.usingCostVeh * static_cast<double>(routes.size());

	for (StandaloneRoute& route : routes) {
		if (route.locId < 0 || route.locId >= data.numL ||
			route.sequence.empty()) {
			return false;
		}
		for (const int customer : route.sequence) {
			if (customer < 0 || customer >= data.numV ||
				++customerUse[static_cast<std::size_t>(customer)] != 1) {
				return false;
			}
		}
		const RouteEvaluation evaluation =
			evaluator.evaluate(
				route.locId,
				route.sequence,
				EvaluationDetail::Summary);
		if (!evaluation.feasible())
			return false;
		depotLoads[static_cast<std::size_t>(route.locId)] +=
			evaluation.load;
		if (depotLoads[static_cast<std::size_t>(route.locId)] >
			data.invLoc[data.numV + route.locId]) {
			return false;
		}
		route.totalLoad = evaluation.load;
		route.routeCost = evaluation.travelCost;
		route.returnTime = evaluation.returnTime;
		route.totalWaitingTime = evaluation.totalWaitingTime;
		route.temporalFeasible = evaluation.temporalFeasible;
		route.temporalViolationCustomer =
			evaluation.violatingCustomer;
		objective += evaluation.travelCost;
		openedDepots.insert(route.locId);
	}
	if (std::any_of(
			customerUse.begin(),
			customerUse.end(),
			[](const int useCount) { return useCount != 1; })) {
		return false;
	}
	for (const int depot : openedDepots)
		objective += data.openCostLoc[data.numV + depot];
	return true;
}

double routeTravelCost(
		const SolverContext& data,
		const int depot,
		const std::vector<int>& sequence) {
	if (sequence.empty()) {
		return 0.0;
	}
	const int depotNode = data.numV + depot;
	double cost = data.D[depotNode][sequence.front()];
	for (std::size_t index = 1U; index < sequence.size(); ++index) {
		cost += data.D[sequence[index - 1U]][sequence[index]];
	}
	cost += data.D[sequence.back()][depotNode];
	return cost;
}

double routeTravelObjectiveContribution(
		const SolverContext& data,
		const StandaloneRoute& route) {
	if (route.sequence.empty()) {
		return 0.0;
	}
	return data.usingCostVeh +
		routeTravelCost(data, route.locId, route.sequence);
}

bool exactDeltaCannotImprove(
		const double predictedObjective,
		const double bestObjective) {
	// This is only a rejection gate. It must never authorize a zero-gain or
	// worsening move at large objective scales. Writing this as
	// `predicted > best - Min` is unsafe when `best - Min` rounds back to best.
	// The authoritative affected-route evaluation repeats this strict test.
	return !(predictedObjective < bestObjective) ||
		bestObjective - predictedObjective < Min;
}

std::size_t hgampMoveMetricIndex(const LS::HgampMoveKind kind) {
	const int rawKind = static_cast<int>(kind);
	if (rawKind < static_cast<int>(LS::HgampMoveKind::M1) ||
		rawKind > static_cast<int>(LS::HgampMoveKind::SplitSuffix)) {
		throw std::logic_error("invalid HGAMP move metric kind");
	}
	return static_cast<std::size_t>(rawKind - 1);
}

void assignRouteEvaluation(
		StandaloneRoute& route,
		std::vector<int> sequence,
		const RouteEvaluation& evaluation) {
	route.sequence = std::move(sequence);
	route.totalLoad = evaluation.load;
	route.routeCost = evaluation.travelCost;
	route.returnTime = evaluation.returnTime;
	route.totalWaitingTime = evaluation.totalWaitingTime;
	route.temporalFeasible = evaluation.temporalFeasible;
	route.temporalViolationCustomer = evaluation.violatingCustomer;
}

struct RouteSlice {
	const std::vector<int>* sequence = nullptr;
	std::size_t begin = 0U;
	std::size_t end = 0U;
	bool reversed = false;
};

struct RouteComposition {
	int locId = -1;
	std::array<RouteSlice, 8U> slices{};
	std::size_t sliceCount = 0U;
};

void appendSlice(
		RouteComposition& composition,
		const std::vector<int>& sequence,
		const std::size_t begin,
		const std::size_t end,
		const bool reversed = false) {
	if (begin >= end) {
		return;
	}
	if (end > sequence.size() ||
		composition.sliceCount >= composition.slices.size()) {
		throw std::logic_error("invalid local-search route composition");
	}
	composition.slices[composition.sliceCount++] =
		RouteSlice{&sequence, begin, end, reversed};
}

template <typename Visitor>
void visitComposition(
		const RouteComposition& composition,
		Visitor&& visitor) {
	for (std::size_t sliceIndex = 0U;
		 sliceIndex < composition.sliceCount;
		 ++sliceIndex) {
		const RouteSlice& slice = composition.slices[sliceIndex];
		if (slice.sequence == nullptr || slice.begin > slice.end ||
			slice.end > slice.sequence->size()) {
			throw std::logic_error(
				"HGAMP composition contains an invalid route slice");
		}
		if (!slice.reversed) {
			for (std::size_t index = slice.begin;
				 index < slice.end;
				 ++index) {
				visitor((*slice.sequence)[index]);
			}
		} else {
			for (std::size_t index = slice.end;
				 index > slice.begin;
				 --index) {
				visitor((*slice.sequence)[index - 1U]);
			}
		}
	}
}

std::size_t compositionSize(const RouteComposition& composition) {
	std::size_t size = 0U;
	for (std::size_t index = 0U;
		 index < composition.sliceCount;
		 ++index) {
		size += composition.slices[index].end -
			composition.slices[index].begin;
	}
	return size;
}

struct SequencePrefixSummary {
	std::vector<int> loadPrefix;
	std::vector<double> forwardArcPrefix;
	std::vector<double> reverseArcPrefix;
	std::vector<int> reversePromisingPrefix;
};

struct TemporalSegmentSummary {
	bool empty = true;
	bool feasible = true;
	int first = -1;
	int last = -1;
	double duration = 0.0;
	double earliestDeparture = 0.0;
	double latestArrival = 0.0;
};

TemporalSegmentSummary singletonTemporalSegment(
		const SolverContext& data,
		const int customer) {
	TemporalSegmentSummary result;
	result.empty = false;
	result.first = customer;
	result.last = customer;
	result.duration = data.serviceTime[customer];
	result.earliestDeparture =
		data.readyTime[customer] + data.serviceTime[customer];
	result.latestArrival = data.dueTime[customer];
	return result;
}

TemporalSegmentSummary concatenateTemporalSegments(
		const SolverContext& data,
		const TemporalSegmentSummary& left,
		const TemporalSegmentSummary& right) {
	if (left.empty) return right;
	if (right.empty) return left;
	const double travel = data.T[left.last][right.first];
	TemporalSegmentSummary result;
	result.empty = false;
	result.feasible = left.feasible && right.feasible &&
		left.earliestDeparture + travel <=
			right.latestArrival +
			HardTimeWindowConstraint::kDefaultEpsilon;
	result.first = left.first;
	result.last = right.last;
	result.duration = left.duration + travel + right.duration;
	result.earliestDeparture = std::max(
		left.earliestDeparture + travel + right.duration,
		right.earliestDeparture);
	result.latestArrival = std::min(
		left.latestArrival,
		right.latestArrival - travel - left.duration);
	return result;
}

class TemporalDisjointSparseTable {
public:
	TemporalDisjointSparseTable() = default;

	TemporalDisjointSparseTable(
			const SolverContext& data,
			const std::vector<int>& nodes,
			const bool reverseView = false)
		: base_(nodes.size()) {
		for (std::size_t index = 0U; index < nodes.size(); ++index) {
			const std::size_t sourceIndex = reverseView
				? nodes.size() - 1U - index
				: index;
			base_[index] = singletonTemporalSegment(
				data, nodes[sourceIndex]);
		}
		for (std::size_t half = 1U; half < nodes.size(); half <<= 1U) {
			const std::size_t block = half << 1U;
			levels_.emplace_back(nodes.size());
			std::vector<TemporalSegmentSummary>& level = levels_.back();
			for (std::size_t start = 0U;
				 start < nodes.size();
				 start += block) {
				const std::size_t middle =
					std::min(start + half, nodes.size());
				const std::size_t end =
					std::min(start + block, nodes.size());
				if (middle > start) {
					level[middle - 1U] = base_[middle - 1U];
					for (std::size_t index = middle - 1U;
						 index > start;
						 --index) {
						level[index - 1U] = concatenateTemporalSegments(
							data, base_[index - 1U], level[index]);
					}
				}
				if (middle < end) {
					level[middle] = base_[middle];
					for (std::size_t index = middle + 1U;
						 index < end;
						 ++index) {
						level[index] = concatenateTemporalSegments(
							data, level[index - 1U], base_[index]);
					}
				}
			}
		}
	}

	TemporalSegmentSummary query(
			const SolverContext& data,
			const std::size_t begin,
			const std::size_t end) const {
		if (begin >= end) return {};
		if (end > base_.size()) {
			throw std::logic_error("invalid temporal range query");
		}
		if (end - begin == 1U) return base_[begin];
		const std::size_t right = end - 1U;
		std::size_t differing = begin ^ right;
		std::size_t level = 0U;
		while (differing > 1U) {
			differing >>= 1U;
			++level;
		}
		if (level >= levels_.size()) {
			throw std::logic_error("temporal sparse-table level missing");
		}
		return concatenateTemporalSegments(
			data, levels_[level][begin], levels_[level][right]);
	}

private:
	std::vector<TemporalSegmentSummary> base_;
	std::vector<std::vector<TemporalSegmentSummary>> levels_;
};

struct TemporalRouteSummary {
	std::size_t size = 0U;
	TemporalDisjointSparseTable forward;
	TemporalDisjointSparseTable reverse;
};

TemporalRouteSummary buildTemporalRouteSummary(
		const SolverContext& data,
		const std::vector<int>& sequence) {
	TemporalRouteSummary summary;
	summary.size = sequence.size();
	summary.forward = TemporalDisjointSparseTable(data, sequence);
	summary.reverse = TemporalDisjointSparseTable(data, sequence, true);
	return summary;
}

struct TemporalSummaryIndex {
	const std::vector<StandaloneRoute>* routes = nullptr;
	const std::vector<std::size_t>* customerRoute = nullptr;
	std::vector<TemporalRouteSummary> values;

	const TemporalRouteSummary& forSequence(
			const std::vector<int>* sequence) const {
		if (sequence == nullptr || sequence->empty() || routes == nullptr ||
			customerRoute == nullptr) {
			throw std::logic_error("missing HGAMP temporal route summary");
		}
		const int customer = sequence->front();
		if (customer < 0 ||
			customer >= static_cast<int>(customerRoute->size())) {
			throw std::logic_error("invalid HGAMP temporal summary customer");
		}
		const std::size_t routeIndex =
			(*customerRoute)[static_cast<std::size_t>(customer)];
		if (routeIndex >= values.size() || routeIndex >= routes->size() ||
			&(*routes)[routeIndex].sequence != sequence) {
			throw std::logic_error("stale HGAMP temporal route summary");
		}
		return values[routeIndex];
	}
};

TemporalSummaryIndex buildTemporalSummaries(
		const SolverContext& data,
		const std::vector<StandaloneRoute>& routes,
		const std::vector<std::size_t>& customerRoute) {
	TemporalSummaryIndex result;
	result.routes = &routes;
	result.customerRoute = &customerRoute;
	result.values.reserve(routes.size());
	for (const StandaloneRoute& route : routes) {
		result.values.push_back(
			buildTemporalRouteSummary(data, route.sequence));
	}
	return result;
}

TemporalSegmentSummary temporalSliceSummary(
		const SolverContext& data,
		const RouteSlice& slice,
		const TemporalSummaryIndex& summaries) {
	const TemporalRouteSummary& route =
		summaries.forSequence(slice.sequence);
	if (!slice.reversed) {
		return route.forward.query(data, slice.begin, slice.end);
	}
	return route.reverse.query(
		data,
		route.size - slice.end,
		route.size - slice.begin);
}

bool temporalCompositionFeasible(
		const SolverContext& data,
		const RouteComposition& composition,
		const TemporalSummaryIndex& summaries) {
	if (compositionSize(composition) == 0U) return true;
	TemporalSegmentSummary aggregate;
	for (std::size_t index = 0U;
		 index < composition.sliceCount;
		 ++index) {
		aggregate = concatenateTemporalSegments(
			data,
			aggregate,
			temporalSliceSummary(data, composition.slices[index], summaries));
		if (!aggregate.feasible) return false;
	}
	const int depotNode = data.numV + composition.locId;
	const double arrival = data.depotOpenTime[composition.locId] +
		data.T[depotNode][aggregate.first];
	if (arrival > aggregate.latestArrival +
			HardTimeWindowConstraint::kDefaultEpsilon) {
		return false;
	}
	const double departure = std::max(
		arrival + aggregate.duration,
		aggregate.earliestDeparture);
	return departure + data.T[aggregate.last][depotNode] <=
		data.depotCloseTime[composition.locId] +
		HardTimeWindowConstraint::kDefaultEpsilon;
}

struct SequenceSummaryIndex {
	const std::vector<StandaloneRoute>* routes = nullptr;
	const std::vector<std::size_t>* customerRoute = nullptr;
	std::vector<SequencePrefixSummary> values;

	const SequencePrefixSummary& forSequence(
			const std::vector<int>* sequence) const {
		if (sequence == nullptr || sequence->empty() || routes == nullptr ||
			customerRoute == nullptr) {
			throw std::logic_error("missing HGAMP route prefix summary");
		}
		const int customer = sequence->front();
		if (customer < 0 ||
			customer >= static_cast<int>(customerRoute->size())) {
			throw std::logic_error("invalid HGAMP route summary customer");
		}
		const std::size_t routeIndex =
			(*customerRoute)[static_cast<std::size_t>(customer)];
		if (routeIndex >= values.size() || routeIndex >= routes->size() ||
			&(*routes)[routeIndex].sequence != sequence) {
			throw std::logic_error("stale HGAMP route prefix summary");
		}
		return values[routeIndex];
	}
};

SequencePrefixSummary buildSequencePrefixSummary(
		const SolverContext& data,
		const std::vector<int>& sequence,
		const hgasp::PromisingArcCandidateGraph* promisingGraph = nullptr) {
	SequencePrefixSummary summary;
	summary.loadPrefix.assign(sequence.size() + 1U, 0);
	summary.forwardArcPrefix.assign(sequence.size(), 0.0);
	summary.reverseArcPrefix.assign(sequence.size(), 0.0);
	summary.reversePromisingPrefix.assign(sequence.size(), 0);
	for (std::size_t index = 0U; index < sequence.size(); ++index) {
		if (sequence[index] < 0 || sequence[index] >= data.numV) {
			throw std::logic_error(
				"HGAMP route summary contains an invalid customer");
		}
		summary.loadPrefix[index + 1U] =
			summary.loadPrefix[index] + data.demCus[sequence[index]];
		if (index != 0U) {
			summary.forwardArcPrefix[index] =
				summary.forwardArcPrefix[index - 1U] +
					data.D[sequence[index - 1U]][sequence[index]];
			summary.reverseArcPrefix[index] =
				summary.reverseArcPrefix[index - 1U] +
					data.D[sequence[index]][sequence[index - 1U]];
			summary.reversePromisingPrefix[index] =
				summary.reversePromisingPrefix[index - 1U] +
				(promisingGraph != nullptr &&
				 promisingGraph->containsCustomerArc(
					 sequence[index], sequence[index - 1U]) ? 1 : 0);
		}
	}
	return summary;
}

SequenceSummaryIndex buildSequenceSummaries(
		const SolverContext& data,
		const std::vector<StandaloneRoute>& routes,
		const std::vector<std::size_t>& customerRoute,
		const hgasp::PromisingArcCandidateGraph* promisingGraph = nullptr) {
	SequenceSummaryIndex result;
	result.routes = &routes;
	result.customerRoute = &customerRoute;
	result.values.reserve(routes.size());
	for (const StandaloneRoute& route : routes) {
		result.values.push_back(
			buildSequencePrefixSummary(data, route.sequence, promisingGraph));
	}
	return result;
}

const SequencePrefixSummary& summaryFor(
		const SequenceSummaryIndex& summaries,
		const std::vector<int>* sequence) {
	return summaries.forSequence(sequence);
}

int compositionLoad(
		const RouteComposition& composition,
		const SequenceSummaryIndex& summaries) {
	int load = 0;
	for (std::size_t sliceIndex = 0U;
		 sliceIndex < composition.sliceCount;
		 ++sliceIndex) {
		const RouteSlice& slice = composition.slices[sliceIndex];
		const SequencePrefixSummary& summary =
			summaryFor(summaries, slice.sequence);
		if (slice.begin > slice.end ||
			slice.end >= summary.loadPrefix.size()) {
			throw std::logic_error(
				"HGAMP composition load slice exceeds its route summary");
		}
		load += summary.loadPrefix[slice.end] -
			summary.loadPrefix[slice.begin];
	}
	return load;
}

double compositionTravelCost(
		const SolverContext& data,
		const RouteComposition& composition,
		const SequenceSummaryIndex& summaries) {
	if (compositionSize(composition) == 0U) {
		return 0.0;
	}
	const int depotNode = data.numV + composition.locId;
	int previous = depotNode;
	double cost = 0.0;
	for (std::size_t sliceIndex = 0U;
		 sliceIndex < composition.sliceCount;
		 ++sliceIndex) {
		const RouteSlice& slice = composition.slices[sliceIndex];
		const std::vector<int>& sequence = *slice.sequence;
		const SequencePrefixSummary& summary =
			summaryFor(summaries, slice.sequence);
		if (slice.begin >= slice.end || slice.end > sequence.size() ||
			slice.end > summary.forwardArcPrefix.size() ||
			slice.end > summary.reverseArcPrefix.size()) {
			throw std::logic_error(
				"HGAMP composition travel slice exceeds its route summary");
		}
		const int first = slice.reversed
			? sequence[slice.end - 1U]
			: sequence[slice.begin];
		const int last = slice.reversed
			? sequence[slice.begin]
			: sequence[slice.end - 1U];
		cost += data.D[previous][first];
		if (slice.end - slice.begin > 1U) {
			cost += slice.reversed
				? summary.reverseArcPrefix[slice.end - 1U] -
					summary.reverseArcPrefix[slice.begin]
				: summary.forwardArcPrefix[slice.end - 1U] -
					summary.forwardArcPrefix[slice.begin];
		}
		previous = last;
	}
	cost += data.D[previous][depotNode];
	return cost;
}

void materializeComposition(
		const RouteComposition& composition,
		std::vector<int>& scratch) {
	scratch.clear();
	scratch.reserve(compositionSize(composition));
	visitComposition(
		composition,
		[&scratch](const int customer) {
			scratch.push_back(customer);
		});
}

RouteComposition sameRouteSegmentExchange(
		const StandaloneRoute& route,
		const std::size_t firstStart,
		const std::size_t firstLength,
		const std::size_t secondStart,
		const std::size_t secondLength,
		const bool reverseFirstAtSecond = false,
		const bool reverseSecondAtFirst = false) {
	if (firstStart + firstLength > route.sequence.size() ||
		secondStart + secondLength > route.sequence.size() ||
		!(firstStart + firstLength <= secondStart ||
		  secondStart + secondLength <= firstStart)) {
		throw std::logic_error("overlapping HGAMP exchange segments");
	}
	RouteComposition result;
	result.locId = route.locId;
	if (firstStart < secondStart) {
		appendSlice(result, route.sequence, 0U, firstStart);
		appendSlice(
			result,
			route.sequence,
			secondStart,
			secondStart + secondLength,
			reverseSecondAtFirst);
		appendSlice(
			result,
			route.sequence,
			firstStart + firstLength,
			secondStart);
		appendSlice(
			result,
			route.sequence,
			firstStart,
			firstStart + firstLength,
			reverseFirstAtSecond);
		appendSlice(
			result,
			route.sequence,
			secondStart + secondLength,
			route.sequence.size());
	} else {
		appendSlice(result, route.sequence, 0U, secondStart);
		appendSlice(
			result,
			route.sequence,
			firstStart,
			firstStart + firstLength,
			reverseFirstAtSecond);
		appendSlice(
			result,
			route.sequence,
			secondStart + secondLength,
			firstStart);
		appendSlice(
			result,
			route.sequence,
			secondStart,
			secondStart + secondLength,
			reverseSecondAtFirst);
		appendSlice(
			result,
			route.sequence,
			firstStart + firstLength,
			route.sequence.size());
	}
	return result;
}

void installLinkedRouteCustomers(
		Route* route,
		const std::vector<Node*>& customers) {
	Node* previous = route->depot;
	for (Node* customer : customers) {
		previous->next = customer;
		customer->pre = previous;
		customer->route = route;
		previous = customer;
	}
	previous->next = route->endDepot;
	route->endDepot->pre = previous;
}

}  // namespace

void LS::linkedRouteCustomers(
		Route* route,
		std::vector<Node*>& output) const {
	if (route == nullptr) {
		throw std::logic_error("cannot read an invalid linked route");
	}
	output.clear();
	for (Node* node = route->depot->next;
		 !node->isDepot;
		 node = node->next) {
		output.push_back(node);
	}
}

std::size_t LS::nodePosition(
		const std::vector<Node*>& customers,
		const Node* target) const {
	if (target == nullptr || target->isDepot || target->position <= 0) {
		throw std::logic_error("HGAMP move node is absent from its route");
	}
	const std::size_t position =
		static_cast<std::size_t>(target->position - 1);
	if (position >= customers.size() || customers[position] != target) {
		throw std::logic_error("HGAMP move node position is stale");
	}
	return position;
}

void LS::applyLinkedSegmentExchange(
		Route* firstRoute,
		Node* firstStart,
		const std::size_t firstLength,
		Route* secondRoute,
		Node* secondStart,
		const std::size_t secondLength,
		const bool reverseFirstAtSecond) {
	std::vector<Node*>& first = linkedMoveScratch_.first;
	linkedRouteCustomers(firstRoute, first);
	const std::size_t firstPosition = nodePosition(first, firstStart);
	if (firstPosition + firstLength > first.size()) {
		throw std::logic_error("HGAMP first exchange segment is truncated");
	}
	if (firstRoute == secondRoute) {
		const std::size_t secondPosition = nodePosition(first, secondStart);
		if (secondPosition + secondLength > first.size() ||
			!(firstPosition + firstLength <= secondPosition ||
			  secondPosition + secondLength <= firstPosition)) {
			throw std::logic_error("HGAMP same-route segments overlap");
		}
		std::vector<Node*>& result = linkedMoveScratch_.result;
		result.clear();
		auto append = [&result, &first](
				const std::size_t begin,
				const std::size_t end,
				const bool reversed = false) {
			if (!reversed) {
				result.insert(
					result.end(), first.begin() + begin, first.begin() + end);
			} else {
				for (std::size_t index = end; index > begin; --index) {
					result.push_back(first[index - 1U]);
				}
			}
		};
		if (firstPosition < secondPosition) {
			append(0U, firstPosition);
			append(secondPosition, secondPosition + secondLength);
			append(firstPosition + firstLength, secondPosition);
			append(
				firstPosition,
				firstPosition + firstLength,
				reverseFirstAtSecond);
			append(secondPosition + secondLength, first.size());
		} else {
			append(0U, secondPosition);
			append(
				firstPosition,
				firstPosition + firstLength,
				reverseFirstAtSecond);
			append(secondPosition + secondLength, firstPosition);
			append(secondPosition, secondPosition + secondLength);
			append(firstPosition + firstLength, first.size());
		}
		installLinkedRouteCustomers(firstRoute, result);
		return;
	}

	std::vector<Node*>& second = linkedMoveScratch_.second;
	linkedRouteCustomers(secondRoute, second);
	const std::size_t secondPosition = nodePosition(second, secondStart);
	if (secondPosition + secondLength > second.size()) {
		throw std::logic_error("HGAMP second exchange segment is truncated");
	}
	std::vector<Node*>& newFirst = linkedMoveScratch_.newFirst;
	newFirst.clear();
	newFirst.insert(
		newFirst.end(), first.begin(), first.begin() + firstPosition);
	newFirst.insert(
		newFirst.end(),
		second.begin() + secondPosition,
		second.begin() + secondPosition + secondLength);
	newFirst.insert(
		newFirst.end(),
		first.begin() + firstPosition + firstLength,
		first.end());

	std::vector<Node*>& newSecond = linkedMoveScratch_.newSecond;
	newSecond.clear();
	newSecond.insert(
		newSecond.end(), second.begin(), second.begin() + secondPosition);
	if (!reverseFirstAtSecond) {
		newSecond.insert(
			newSecond.end(),
			first.begin() + firstPosition,
			first.begin() + firstPosition + firstLength);
	} else {
		for (std::size_t index = firstPosition + firstLength;
			 index > firstPosition;
			 --index) {
			newSecond.push_back(first[index - 1U]);
		}
	}
	newSecond.insert(
		newSecond.end(),
		second.begin() + secondPosition + secondLength,
		second.end());
	installLinkedRouteCustomers(firstRoute, newFirst);
	installLinkedRouteCustomers(secondRoute, newSecond);
}

void LS::applyLinkedReversedPairBefore(Node* first, Node* destination) {
	Node* const second = first->next;
	Route* const sourceRoute = first->route;
	Route* const destinationRoute = destination->route;
	std::vector<Node*>& source = linkedMoveScratch_.first;
	linkedRouteCustomers(sourceRoute, source);
	const std::size_t sourcePosition = nodePosition(source, first);
	std::size_t destinationPosition = 0U;
	if (sourceRoute == destinationRoute) {
		destinationPosition = nodePosition(source, destination);
		if (destinationPosition == sourcePosition ||
			destinationPosition == sourcePosition + 1U) {
			throw std::logic_error("HGAMP M3 destination overlaps moved pair");
		}
	}
	source.erase(
		source.begin() + static_cast<std::ptrdiff_t>(sourcePosition),
		source.begin() + static_cast<std::ptrdiff_t>(sourcePosition + 2U));
	if (sourceRoute == destinationRoute) {
		const std::size_t position = destinationPosition > sourcePosition
			? destinationPosition - 2U
			: destinationPosition;
		source.insert(
			source.begin() + static_cast<std::ptrdiff_t>(position),
			{second, first});
		installLinkedRouteCustomers(sourceRoute, source);
		return;
	}
	std::vector<Node*>& target = linkedMoveScratch_.second;
	linkedRouteCustomers(destinationRoute, target);
	std::size_t position = 0U;
	if (!destination->isDepot) {
		position = nodePosition(target, destination);
	}
	target.insert(
		target.begin() + static_cast<std::ptrdiff_t>(position),
		{second, first});
	installLinkedRouteCustomers(sourceRoute, source);
	installLinkedRouteCustomers(destinationRoute, target);
}

void LS::applyLinkedM9(Node* u, Node* v) {
	Route* const routeA = u->route;
	Route* const routeB = v->route;
	std::vector<Node*>& a = linkedMoveScratch_.first;
	std::vector<Node*>& b = linkedMoveScratch_.second;
	linkedRouteCustomers(routeA, a);
	linkedRouteCustomers(routeB, b);
	const std::size_t uPosition = nodePosition(a, u);
	const std::size_t vPosition = nodePosition(b, v);
	std::vector<Node*>& newA = linkedMoveScratch_.newFirst;
	newA.clear();
	newA.insert(newA.end(), a.begin(), a.begin() + uPosition + 1U);
	newA.insert(newA.end(), b.begin() + vPosition, b.end());
	std::vector<Node*>& newB = linkedMoveScratch_.newSecond;
	newB.clear();
	newB.insert(newB.end(), b.begin(), b.begin() + vPosition);
	newB.insert(newB.end(), a.begin() + uPosition + 1U, a.end());
	installLinkedRouteCustomers(routeA, newA);
	installLinkedRouteCustomers(routeB, newB);
}

void LS::applyLinkedM10(Node* u, Node* v) {
	Route* const routeA = u->route;
	Route* const routeB = v->route;
	std::vector<Node*>& a = linkedMoveScratch_.first;
	std::vector<Node*>& b = linkedMoveScratch_.second;
	linkedRouteCustomers(routeA, a);
	linkedRouteCustomers(routeB, b);
	const std::size_t uPosition = nodePosition(a, u);
	const std::size_t vPosition = nodePosition(b, v);
	std::vector<Node*>& newA = linkedMoveScratch_.newFirst;
	newA.clear();
	newA.insert(newA.end(), a.begin(), a.begin() + uPosition + 1U);
	for (std::size_t index = vPosition + 1U; index > 0U; --index)
		newA.push_back(b[index - 1U]);
	std::vector<Node*>& newB = linkedMoveScratch_.newSecond;
	newB.clear();
	for (std::size_t index = a.size(); index > uPosition + 1U; --index)
		newB.push_back(a[index - 1U]);
	newB.insert(newB.end(), b.begin() + vPosition + 1U, b.end());
	installLinkedRouteCustomers(routeA, newA);
	installLinkedRouteCustomers(routeB, newB);
}

LS::LS(SolverContext * data, RepairProcedure *repair) {
	this->data=data;
	(void)repair;
	this->numV = data->numV;
	this->numC = data->numC;
	this->numL = data->numL;
	for (int i=0;i<numV;i++)
		orderNodes.push_back(i);
	linkedMoveScratch_.reserve(static_cast<std::size_t>(numV));
}
double LS::compute(bool istrue, double value){
	if (istrue)	return 0.;
	else return value;
}

int LS::effectiveLocalSearchAlpha() const noexcept {
	if (data->parameters.profile ==
		hgasp::SearchParameterProfile::Hgamp2ParameterCompatibleSetS) {
		return std::min(
			data->parameters.localSearch.constructionCorrelationAlpha,
			std::max(0, numV - 2));
	}
	return std::min(
		data->localSearchCustomerAlpha,
		std::max(0, numV - 1));
}

int LS::effectiveDepotCandidateBeta() const noexcept {
	if (hgasp::isSetSProductionSearchProfile(data->parameters.profile)) {
		return std::min(
			std::max(0, data->localSearchDepotBeta),
			std::max(0, numL));
	}
	if (data->parameters.profile ==
		hgasp::SearchParameterProfile::Hgamp2ParameterCompatibleSetS) {
		return std::min(
			std::max(0, numL),
			std::max(0, numL / 2 + 1));
	}
	// Preserve depot exploration for the single-customer boundary. Customer
	// alpha is necessarily zero there, but HGAMP granularity remains one.
	const int granularity = std::min(
		data->localSearchDepotBeta,
		std::max(1, numV - 1));
	return std::min(granularity, std::max(0, numL));
}

bool LS::fixedConfigurationAllowsDepotState(
		const NodeDepot* const depot,
		const bool usedAfterMove) const noexcept {
	if (!fixedConfigurationMode_) return true;
	if (depot == nullptr) return false;
	const int depotIndex = depot->cour - numV;
	return depotIndex >= 0 && depotIndex < numL &&
		static_cast<std::size_t>(depotIndex) < fixedOpenDepots_.size() &&
		fixedOpenDepots_[static_cast<std::size_t>(depotIndex)] ==
			usedAfterMove;
}

void LS::resetHgampMetrics() {
#ifndef NDEBUG
	// Preserve the trace allocation across the many LS invocations in a run.
	// Moving it aside lets aggregate assignment reset every scalar and array
	// without retaining stale values or releasing the vector's storage.
	std::vector<HgampMoveKind> acceptedMoveTrace =
		std::move(lastTwMetrics_.acceptedMoveTrace);
	lastTwMetrics_ = {};
	lastTwMetrics_.acceptedMoveTrace = std::move(acceptedMoveTrace);
	lastTwMetrics_.acceptedMoveTrace.clear();
#else
	// Release-diagnostic keeps aggregate operator counters but not a dynamic
	// per-move trace; the latter distorted the component whose cost it measured.
	lastTwMetrics_ = {};
#endif
#if !defined(NDEBUG) || defined(HGASP_ENABLE_HOT_PATH_COUNTERS)
	lastTwMetrics_.effectiveAlpha =
		static_cast<std::size_t>(effectiveLocalSearchAlpha());
	lastTwMetrics_.effectiveDepotBeta =
		static_cast<std::size_t>(effectiveDepotCandidateBeta());
	lastTwMetrics_.maximumCandidateVertices =
		staticMaximumCustomerCandidates_;
	lastTwMetrics_.maximumDepotCandidates =
		staticMaximumDepotCandidates_;
#else
	// The publication binary keeps acceptedMoves because it controls final
	// objective refresh and no-improvement handling. Per-move counters and the
	// acceptedMoveTrace belong to dev and release-diagnostic builds.
#endif
}

void LS::recordHgampCandidate(const HgampMoveKind kind) {
#if !defined(NDEBUG) || defined(HGASP_ENABLE_HOT_PATH_COUNTERS)
	++lastTwMetrics_.candidateMoves;
	++lastTwMetrics_.candidateByHgampMove[hgampMoveMetricIndex(kind)];
#else
	(void)kind;
#endif
}

void LS::recordHgampAccepted(const HgampMoveKind kind) {
	if (promisingArcFilterEnabled()) advanceCanonicalTopologyEpoch();
#if !defined(NDEBUG) || defined(HGASP_ENABLE_HOT_PATH_COUNTERS)
	if (lastTwMetrics_.acceptedMoves == 0U) {
		lastTwMetrics_.firstAcceptedMoveKind = kind;
	}
	++lastTwMetrics_.acceptedMoves;
#ifndef NDEBUG
	lastTwMetrics_.acceptedMoveTrace.push_back(kind);
#endif
	++lastTwMetrics_.acceptedByHgampMove[hgampMoveMetricIndex(kind)];
	switch (kind) {
	case HgampMoveKind::M1:
		++lastTwMetrics_.acceptedRelocateOneMoves;
		break;
	case HgampMoveKind::M2:
		++lastTwMetrics_.acceptedOrOptTwoMoves;
		break;
	case HgampMoveKind::M3:
		break;
	case HgampMoveKind::M4:
		++lastTwMetrics_.acceptedSwapMoves;
		break;
	case HgampMoveKind::M5:
	case HgampMoveKind::M6:
	case HgampMoveKind::M7:
		break;
	case HgampMoveKind::M8:
		++lastTwMetrics_.acceptedTwoOptMoves;
		break;
	case HgampMoveKind::M9:
	case HgampMoveKind::M10:
		++lastTwMetrics_.acceptedTwoOptStarMoves;
		break;
	case HgampMoveKind::SplitPrefix:
		++lastTwMetrics_.acceptedSplitPrefixMoves;
		break;
	case HgampMoveKind::SplitSuffix:
		++lastTwMetrics_.acceptedSplitSuffixMoves;
		break;
	case HgampMoveKind::None:
	case HgampMoveKind::OrOptThree:
		throw std::logic_error("accepted a non-HGAMP move");
	}
#else
	++lastTwMetrics_.acceptedMoves;
	(void)kind;
#endif
}

void LS::advanceCanonicalTopologyEpoch() {
	constexpr std::uint32_t maximumPackedEpoch = 0x0FFFFFFFU;
	if (canonicalTopologyEpoch_ == maximumPackedEpoch) {
		std::fill(canonicalPairStates_.begin(),
			canonicalPairStates_.end(), 0U);
		canonicalTopologyEpoch_ = 1U;
	} else {
		++canonicalTopologyEpoch_;
	}
}

bool LS::canonicalPairAlreadyEvaluated(
		const HgampMoveKind kind,
		const int firstBlockStart,
		const int secondBlockStart) noexcept {
	// V2.2 and Set F retain their sealed traversal exactly. Role-aware Set S uses a
	// directed Gamma graph, so a simple `u < v` guard is not safe: only an
	// actual second occurrence of the same exchange may be suppressed.
	if (!promisingArcFilterEnabled() || firstBlockStart < 0 ||
		secondBlockStart < 0 || firstBlockStart >= numV ||
		secondBlockStart >= numV || firstBlockStart == secondBlockStart) {
		return false;
	}
	if (canonicalTopologyEpoch_ == 0U) {
		advanceCanonicalTopologyEpoch();
	}
#if !defined(NDEBUG) || defined(HGASP_ENABLE_HOT_PATH_COUNTERS)
	++lastTwMetrics_.canonicalPairCacheQueries;
#endif
	const int lower = std::min(firstBlockStart, secondBlockStart);
	const int upper = std::max(firstBlockStart, secondBlockStart);
	const std::size_t index = static_cast<std::size_t>(lower) *
		static_cast<std::size_t>(numV) + static_cast<std::size_t>(upper);
	std::uint32_t operatorBit = 0U;
	switch (kind) {
	case HgampMoveKind::M4:
		operatorBit = 0x1U;
		break;
	case HgampMoveKind::M6:
		operatorBit = 0x2U;
		break;
	case HgampMoveKind::M7:
		// M7 reverses the first block only.  Swapping the two block roles is
		// a different route outcome and must remain available.  The orientation
		// bit deduplicates only a genuinely repeated directed outcome.
		operatorBit = firstBlockStart == lower ? 0x4U : 0x8U;
		break;
	default:
		return false;
	}
	if (index >= canonicalPairStates_.size()) {
		return false;
	}
	constexpr std::uint32_t operatorMask = 0xFU;
	const std::uint32_t epochTag = canonicalTopologyEpoch_ << 4U;
	std::uint32_t& state = canonicalPairStates_[index];
	if ((state & ~operatorMask) != epochTag) {
		state = epochTag | operatorBit;
		return false;
	}
	if ((state & operatorBit) != 0U) {
#if !defined(NDEBUG) || defined(HGASP_ENABLE_HOT_PATH_COUNTERS)
		++lastTwMetrics_.canonicalPairCacheHits;
		++lastTwMetrics_.canonicalPairDuplicateSkips;
#endif
		return true;
	}
	state |= operatorBit;
	return false;
}

HGASP_LS_ALWAYS_INLINE int LS::linkedDependencyVersion(
		const Route* first,
		const Route* second) const noexcept {
	int version = std::max(
		routeModificationVersion(first),
		routeModificationVersion(second));
	for (const Route* route : {first, second}) {
		if (route == nullptr || route->whichLocation == nullptr) {
			continue;
		}
		const int depot = route->whichLocation->cour - numV;
		if (depot >= 0 &&
			depot < static_cast<int>(depotModificationVersions_.size())) {
			const std::size_t depotIndex =
				static_cast<std::size_t>(depot);
			if (depotModificationGenerations_[depotIndex] ==
					currentInvocationGeneration_) {
				version = std::max(
					version,
					depotModificationVersions_[depotIndex]);
			}
		}
	}
	return version;
}

int LS::routeModificationVersion(const Route* route) const noexcept {
	if (route == nullptr || route->cour < 0) {
		return -1;
	}
	const std::size_t routeIndex =
		static_cast<std::size_t>(route->cour);
	if (routeIndex >= routeModificationVersions_.size() ||
		routeModificationGenerations_[routeIndex] !=
			currentInvocationGeneration_) {
		return -1;
	}
	return routeModificationVersions_[routeIndex];
}

LS::CapacityRouteBeforeState LS::captureRouteBefore(
		Route* const route) const {
	if (route == nullptr || route->whichLocation == nullptr) {
		throw std::logic_error(
			"cannot capture an invalid capacity route");
	}
	return {route, route->whichLocation, route->load, route->used};
}

void LS::refreshChangedRoutesAndDepots(
		const std::array<CapacityRouteBeforeState, 2U>& states,
		const std::size_t count) {
	if (count == 0U || count > states.size()) {
		throw std::logic_error(
			"capacity route refresh requires one or two routes");
	}
	const bool hadCurrentFeasibilityCertificate =
		s->hasCurrentFeasibilityCertificate();
	std::array<CapacityRouteBeforeState, 2U> uniqueRoutes{};
	const auto validateState = [](const CapacityRouteBeforeState& state) {
		if (state.route == nullptr || state.depot == nullptr ||
			state.route->whichLocation != state.depot) {
			throw std::logic_error(
				"capacity route refresh received stale ownership");
		}
	};
	validateState(states[0]);
	uniqueRoutes[0] = states[0];
	std::size_t uniqueRouteCount = 1U;
	if (count == 2U) {
		validateState(states[1]);
		if (states[1].route == states[0].route) {
			if (states[1].depot != states[0].depot ||
				states[1].oldLoad != states[0].oldLoad ||
				states[1].oldUsed != states[0].oldUsed) {
				throw std::logic_error(
					"duplicate capacity route snapshots disagree");
			}
		} else {
			uniqueRoutes[1] = states[1];
			uniqueRouteCount = 2U;
		}
	}

	bool routeOccupancyChanged = false;
	struct DepotDelta {
		NodeDepot* depot = nullptr;
		int load = 0;
		int routes = 0;
	};
	std::array<DepotDelta, 2U> depotDeltas{};
	std::size_t depotCount = 0U;
	for (std::size_t index = 0U;
		 index < uniqueRouteCount;
		 ++index) {
		const CapacityRouteBeforeState& state = uniqueRoutes[index];
		updateModifiedRoute(state.route);
		if (index == 0U && hadCurrentFeasibilityCertificate &&
			s->hasCurrentFeasibilityCertificate()) {
			throw std::logic_error(
				"route refresh left a feasibility certificate current");
		}
		const bool newUsed = routeIsOccupied(*state.route);
		state.route->used = newUsed;
		routeOccupancyChanged = routeOccupancyChanged ||
			state.oldUsed != newUsed;
		std::size_t depotIndex = 0U;
		if (depotCount == 0U) {
			depotDeltas[0].depot = state.depot;
			depotCount = 1U;
		} else if (depotDeltas[0].depot != state.depot) {
			if (depotCount != 1U) {
				throw std::logic_error(
					"capacity route refresh exceeded two depots");
			}
			depotIndex = 1U;
			depotDeltas[1].depot = state.depot;
			depotCount = 2U;
		}
		DepotDelta& depot = depotDeltas[depotIndex];
		depot.load += state.route->load - state.oldLoad;
		depot.routes += static_cast<int>(newUsed) -
			static_cast<int>(state.oldUsed);
	}

	std::array<NodeDepot*, 2U> affectedDepots{};
	for (std::size_t index = 0U; index < depotCount; ++index) {
		DepotDelta& delta = depotDeltas[index];
		delta.depot->remainInven += delta.load;
		delta.depot->splitTime += delta.routes;
		delta.depot->used = delta.depot->splitTime > 0;
		if (delta.depot->remainInven < 0 ||
			delta.depot->remainInven > delta.depot->inventory ||
			delta.depot->splitTime < 0 ||
			delta.depot->splitTime > delta.depot->maxVisit) {
			throw std::logic_error(
				"incremental capacity depot state exceeded its bounds");
		}
		affectedDepots[index] = delta.depot;
		// The two changed routes already carry fresh route versions.  Invalidate
		// every other route linked to this depot only when a depot-wide quantity
		// that can change feasibility or objective actually changed.  Intra-depot
		// relocations and exchanges normally have zero net load/route-count delta;
		// marking the depot for those moves defeats the route-scoped move memory
		// and causes a full depot rescan without changing any decision input.
		if (delta.load != 0 || delta.routes != 0) {
			markLinkedDepotsModified(delta.depot, nullptr);
		}
	}
	if (routeOccupancyChanged) {
		advanceDepotOccupancyGeneration();
	}

#if !defined(NDEBUG)
	assertIncrementalDepotStateMatchesFullScan(
		affectedDepots, depotCount);
#endif
}

void LS::assertIncrementalDepotStateMatchesFullScan(
		const std::array<NodeDepot*, 2U>& depots,
		const std::size_t count) const {
	for (std::size_t index = 0U; index < count; ++index) {
		const NodeDepot* const depot = depots[index];
		if (depot == nullptr) {
			throw std::logic_error(
				"incremental capacity oracle received a null depot");
		}
		int recomputedLoad = 0;
		int recomputedRoutes = 0;
		std::set<int> recomputedEmptyRoutes;
		for (int slot = 0; slot < depot->maxVisit; ++slot) {
			const Route* const route = depot->depots[slot].route;
			const bool expectedUsed = routeIsOccupied(*route);
			if (route->used != expectedUsed) {
				throw std::logic_error(
					"incremental capacity oracle found stale route usage");
			}
			if (expectedUsed) {
				recomputedLoad += route->load;
				++recomputedRoutes;
			} else {
				recomputedEmptyRoutes.insert(route->cour);
			}
		}
		if (recomputedLoad != depot->remainInven ||
			recomputedRoutes != depot->splitTime ||
			depot->used != (depot->splitTime > 0) ||
			recomputedEmptyRoutes != depot->emptyRoutes) {
			throw std::logic_error(
				"incremental capacity depot state disagrees with full scan");
		}
	}
}

void LS::updateModifiedRoute(Route* route) {
	if (route == nullptr || route->cour < 0 ||
		static_cast<std::size_t>(route->cour) >=
			routeModificationVersions_.size()) {
		throw std::logic_error(
			"local search modified an invalid route slot");
	}
	int routeIndex = route->cour;
	s->updateRouteInfor(routeIndex, nbMoves);
	const std::size_t index = static_cast<std::size_t>(routeIndex);
	routeModificationVersions_[index] = nbMoves;
	routeModificationGenerations_[index] =
		currentInvocationGeneration_;
}

void LS::markLinkedDepotsModified(
		const NodeDepot* first,
		const NodeDepot* second) noexcept {
	for (const NodeDepot* location : {first, second}) {
		if (location == nullptr) {
			continue;
		}
		const int depot = location->cour - numV;
		if (depot >= 0 &&
			depot < static_cast<int>(depotModificationVersions_.size())) {
			const std::size_t depotIndex =
				static_cast<std::size_t>(depot);
			depotModificationVersions_[depotIndex] = nbMoves;
			depotModificationGenerations_[depotIndex] =
				currentInvocationGeneration_;
		}
	}
}

void LS::advanceDepotOccupancyGeneration() noexcept {
	if (depotOccupancyGeneration_ ==
		std::numeric_limits<std::uint64_t>::max()) {
		depotOccupancyGeneration_ = 1U;
		resolvedDepotCandidateCustomer_ = -1;
		resolvedDepotCandidateInvocationGeneration_ = 0U;
		resolvedDepotCandidateOccupancyGeneration_ = 0U;
	} else {
		++depotOccupancyGeneration_;
	}
}

void LS::beginInvocationGeneration() {
	resolvedDepotCandidateCustomer_ = -1;
	resolvedDepotCandidateInvocationGeneration_ = 0U;
	resolvedDepotCandidateOccupancyGeneration_ = 0U;
	depotOccupancyGeneration_ = 1U;
	if (currentInvocationGeneration_ ==
		std::numeric_limits<std::uint64_t>::max()) {
		std::fill(
			depotModificationGenerations_.begin(),
			depotModificationGenerations_.end(), 0U);
		std::fill(
			customerLastTestedGenerations_.begin(),
			customerLastTestedGenerations_.end(), 0U);
		std::fill(
			routeModificationGenerations_.begin(),
			routeModificationGenerations_.end(), 0U);
		std::fill(
			reverseArcPrefixInvocationGenerations_.begin(),
			reverseArcPrefixInvocationGenerations_.end(), 0U);
		currentInvocationGeneration_ = 1U;
	} else {
		++currentInvocationGeneration_;
	}
	if (promisingArcFilterEnabled()) advanceCanonicalTopologyEpoch();

	const std::size_t depotCount = static_cast<std::size_t>(numL);
	if (depotModificationVersions_.size() != depotCount ||
		depotModificationGenerations_.size() != depotCount) {
		depotModificationVersions_.assign(depotCount, -1);
		depotModificationGenerations_.assign(depotCount, 0U);
	}
	const std::size_t customerCount = static_cast<std::size_t>(numV);
	if (customerLastTestedVersions_.size() != customerCount ||
		customerLastTestedGenerations_.size() != customerCount) {
		customerLastTestedVersions_.assign(customerCount, -1);
		customerLastTestedGenerations_.assign(customerCount, 0U);
	}
	const std::size_t routeCount = static_cast<std::size_t>(s->maxVnum);
	if (routeModificationVersions_.size() != routeCount ||
		routeModificationGenerations_.size() != routeCount) {
		routeModificationVersions_.assign(routeCount, -1);
		routeModificationGenerations_.assign(routeCount, 0U);
	}
	if (reverseArcPrefix_.size() != customerCount) {
		reverseArcPrefix_.resize(customerCount);
	}
	if (reversePromisingArcPrefix_.size() != customerCount) {
		reversePromisingArcPrefix_.resize(customerCount);
	}
	if (reverseArcPrefixGeneration_.size() != routeCount ||
		reverseArcPrefixInvocationGenerations_.size() != routeCount) {
		reverseArcPrefixGeneration_.assign(routeCount, -1);
		reverseArcPrefixInvocationGenerations_.assign(routeCount, 0U);
	}
}

void LS::ensureLocalSearchCandidates() {
	if (localSearchCandidatesInitialized_) {
		return;
	}
	const int alpha = effectiveLocalSearchAlpha();
	const int beta = effectiveDepotCandidateBeta();
	localSearchCustomerCandidates_.assign(
		static_cast<std::size_t>(numV), {});
	localSearchDepotCandidates_.assign(
		static_cast<std::size_t>(numV), {});
	const auto finalizeCandidateCaches = [this]() {
		localSearchDepotCandidateDescriptors_.assign(
			static_cast<std::size_t>(numV), {});
		for (int customer = 0; customer < numV; ++customer) {
			const auto& nodes = localSearchDepotCandidates_[
				static_cast<std::size_t>(customer)];
			auto& descriptors = localSearchDepotCandidateDescriptors_[
				static_cast<std::size_t>(customer)];
			descriptors.reserve(nodes.size());
			for (const int node : nodes) {
				const int depot = node - numV;
				if (depot < 0 || depot >= numL) {
					throw std::logic_error(
						"local-search depot candidate is outside the instance");
				}
				descriptors.push_back({node, depot});
			}
		}
		const std::size_t pairCells = static_cast<std::size_t>(numV) *
			static_cast<std::size_t>(numV);
		if (hgasp::isSetSProductionSearchProfile(
				data->parameters.profile)) {
			canonicalPairStates_.assign(pairCells, 0U);
		} else {
			canonicalPairStates_.clear();
		}
		localSearchCandidatesInitialized_ = true;
	};
	std::size_t maximumCustomerCandidates = 0U;
	std::size_t maximumDepotCandidates = 0U;
	const bool hgamp2CompatibleCandidates =
		data->parameters.profile ==
			hgasp::SearchParameterProfile::Hgamp2ParameterCompatibleSetS;
	const bool promisingArcCandidates =
		hgasp::isSetSProductionSearchProfile(data->parameters.profile);
	if (promisingArcCandidates) {
		promisingArcGraph_ = hgasp::buildPromisingArcCandidateGraph(
			*data,
			alpha,
			beta,
			data->localSearchTemporalPenaltyWeight,
			data->localSearchWaitingTimeWeight);
		localSearchCustomerCandidates_ =
			promisingArcGraph_.customerCandidates;
		localSearchDepotCandidates_ = promisingArcGraph_.depotCandidates;
		staticMaximumCustomerCandidates_ =
			promisingArcGraph_.maximumCustomerCandidates;
		staticMaximumDepotCandidates_ =
			promisingArcGraph_.maximumDepotCandidates;
		finalizeCandidateCaches();
		return;
	}
	for (int customer = 0; customer < numV; ++customer) {
		std::vector<int>& customerCandidates =
			localSearchCustomerCandidates_[static_cast<std::size_t>(customer)];
		customerCandidates.reserve(
			hgamp2CompatibleCandidates
				? (customer < static_cast<int>(data->edgeNode.size())
					? data->edgeNode[static_cast<std::size_t>(customer)].size()
					: 0U)
				: static_cast<std::size_t>(alpha));
		std::vector<bool> selected(
			static_cast<std::size_t>(numV), false);
		if (hgamp2CompatibleCandidates) {
			if (customer >= static_cast<int>(data->edgeNode.size())) {
				throw std::logic_error(
					"HGAMP2-compatible customer candidate graph is incomplete");
			}
			// SolverContext builds edgeNode with the original HGAMP2 rule: rank
			// all customer/depot vertices, select min(alpha,n-2), then take
			// the symmetric union. Depot vertices may consume an outgoing
			// slot and are ignored by the customer-customer LS scan.
			for (const int vertex :
				 data->edgeNode[static_cast<std::size_t>(customer)]) {
				if (vertex < 0 || vertex >= numV || vertex == customer ||
					selected[static_cast<std::size_t>(vertex)]) {
					continue;
				}
				selected[static_cast<std::size_t>(vertex)] = true;
				customerCandidates.push_back(vertex);
			}
		}
		// Production nearCity is the stable (D[customer][vertex], vertex)
		// outgoing ranking. Filter it into a customer-only alpha list, keeping
		// depots from consuming customer slots and tolerating defensive fixture
		// duplicates. Fill only malformed/truncated inputs from the same directed
		// ordering.
		if (!hgamp2CompatibleCandidates &&
			customer < static_cast<int>(data->nearCity.size())) {
			for (const int vertex : data->nearCity[customer]) {
				if (vertex < 0 || vertex >= numV || vertex == customer ||
					selected[static_cast<std::size_t>(vertex)]) {
					continue;
				}
				selected[static_cast<std::size_t>(vertex)] = true;
				customerCandidates.push_back(vertex);
				if (static_cast<int>(customerCandidates.size()) == alpha) break;
			}
		}
		if (!hgamp2CompatibleCandidates &&
			static_cast<int>(customerCandidates.size()) < alpha) {
			std::vector<std::pair<double, int>> remainder;
			for (int vertex = 0; vertex < numV; ++vertex) {
				if (vertex != customer &&
					!selected[static_cast<std::size_t>(vertex)]) {
					remainder.emplace_back(
						data->D[customer][vertex], vertex);
				}
			}
			std::sort(remainder.begin(), remainder.end());
			for (const auto& candidate : remainder) {
				customerCandidates.push_back(candidate.second);
				if (static_cast<int>(customerCandidates.size()) == alpha) break;
			}
		}
		maximumCustomerCandidates = std::max(
			maximumCustomerCandidates,
			customerCandidates.size());

		std::vector<std::pair<double, int>> rankedDepots;
		rankedDepots.reserve(static_cast<std::size_t>(numL));
		for (int depot = 0; depot < numL; ++depot) {
			const int depotNode = numV + depot;
			if (hgamp2CompatibleCandidates ||
				data->demCus[customer] <= data->invLoc[depotNode]) {
				rankedDepots.emplace_back(
					data->D[customer][depotNode], depotNode);
			}
		}
		const int actualBeta = std::min(
			beta, static_cast<int>(rankedDepots.size()));
		const auto depotEnd = rankedDepots.begin() + actualBeta;
		std::partial_sort(
			rankedDepots.begin(), depotEnd, rankedDepots.end());
		std::vector<int>& depotCandidates =
			localSearchDepotCandidates_[static_cast<std::size_t>(customer)];
		depotCandidates.reserve(static_cast<std::size_t>(actualBeta));
		for (auto iterator = rankedDepots.begin();
			 iterator != depotEnd;
			 ++iterator) {
			depotCandidates.push_back(iterator->second);
		}
		maximumDepotCandidates = std::max(
			maximumDepotCandidates, depotCandidates.size());
	}
	staticMaximumCustomerCandidates_ = maximumCustomerCandidates;
	staticMaximumDepotCandidates_ = maximumDepotCandidates;
	finalizeCandidateCaches();
}

const std::vector<LS::ResolvedDepotCandidate>&
LS::resolvedDepotCandidates(const int customer) {
#if !defined(NDEBUG) || defined(HGASP_ENABLE_HOT_PATH_COUNTERS)
	++lastTwMetrics_.depotCandidateResolutionRequests;
#endif
	if (customer < 0 || customer >= numV) {
		throw std::out_of_range(
			"local-search depot state requested for an invalid customer");
	}
	if (resolvedDepotCandidateCustomer_ == customer &&
		resolvedDepotCandidateInvocationGeneration_ ==
			currentInvocationGeneration_ &&
		resolvedDepotCandidateOccupancyGeneration_ ==
			depotOccupancyGeneration_) {
		return resolvedDepotCandidateScratch_;
	}
#if !defined(NDEBUG) || defined(HGASP_ENABLE_HOT_PATH_COUNTERS)
	++lastTwMetrics_.depotCandidateResolutionBuilds;
#endif
	const auto& descriptors = localSearchDepotCandidateDescriptors_[
		static_cast<std::size_t>(customer)];
	resolvedDepotCandidateScratch_.clear();
	resolvedDepotCandidateScratch_.reserve(descriptors.size());
	for (const DepotCandidateDescriptor& descriptor : descriptors) {
		if (descriptor.depot < 0 || descriptor.depot >= numL) {
			throw std::out_of_range(
				"local-search depot state requested for an invalid depot");
		}
		NodeDepot* const location = &s->location[descriptor.depot];
		const bool allowed = !fixedConfigurationMode_ ||
			fixedOpenDepots_[static_cast<std::size_t>(descriptor.depot)];
		Node* emptyRouteDepot = nullptr;
		if (allowed && !location->emptyRoutes.empty()) {
			emptyRouteDepot = s->route[
				*location->emptyRoutes.begin()].depot;
		}
		resolvedDepotCandidateScratch_.push_back({
			descriptor.node,
			descriptor.depot,
			location,
			emptyRouteDepot,
			allowed,
		});
	}
	resolvedDepotCandidateCustomer_ = customer;
	resolvedDepotCandidateInvocationGeneration_ =
		currentInvocationGeneration_;
	resolvedDepotCandidateOccupancyGeneration_ =
		depotOccupancyGeneration_;
	return resolvedDepotCandidateScratch_;
}

void LS::srun(){
	ensureLocalSearchCandidates();
	RandomShuffle(orderNodes);
	const int candidateShuffleDenominator =
		data->parameters.localSearch.candidateShuffleDenominator;
	for (int i=0; i<numV; i++)
		if (RandomInt(0, candidateShuffleDenominator - 1) == 0)
			RandomShuffle(localSearchCustomerCandidates_[i]);
        
	searchComplete=false;
	for (loopID=0;!searchComplete;loopID++){
		searchComplete=true;
		for (int posU=0;posU<numV;posU++){// only num_v-1 clients
			nodeU=&s->client[orderNodes[posU]];
			// nodeU's linked neighborhood is immutable until an accepted move
			// increments nbMoves. Reuse these six pointer/index loads across all
			// customer and depot candidates for the same topology.
			int loadedUTopology = -1;
			const std::size_t customerIndex =
				static_cast<std::size_t>(nodeU->cour);
			const int lastTestRINodeU =
				customerLastTestedGenerations_[customerIndex] ==
					currentInvocationGeneration_
					? customerLastTestedVersions_[customerIndex]
					: -1;
			customerLastTestedVersions_[customerIndex] = nbMoves;
			customerLastTestedGenerations_[customerIndex] =
				currentInvocationGeneration_;
			for (int posV = 0;
				 posV < static_cast<int>(
					 localSearchCustomerCandidates_[nodeU->cour].size());
				 ++posV){
				const int candidate =
					localSearchCustomerCandidates_[nodeU->cour][posV];
#if !defined(NDEBUG) || defined(HGASP_ENABLE_HOT_PATH_COUNTERS)
				++lastTwMetrics_.candidateVertices;
#endif
				nodeV=&s->client[candidate];
				if (loopID==0 ||
					linkedDependencyVersion(nodeU->route, nodeV->route) >
						lastTestRINodeU){
					if (loadedUTopology != nbMoves) {
						setLocalVariableRouteU();
						loadedUTopology = nbMoves;
					}
					setLocalVariableRouteV();
					//
					if (Relocate_one())continue;//10 insert operator
					if (Relocate_two())continue;//20 insert operator
					if (Relocate_three())continue;//02 insert operator
					if (Relocate_four()) continue; // M4: x <-> v
					if (Relocate_five())continue;//21 swap
					if (Relocate_six()) continue; // M6
					if (Relocate_seven()) continue; // M7
					if (routeU == routeV && Relocate_eight()) continue;
					if (routeU != routeV && Relocate_nine()) continue;
					if (routeU != routeV && Relocate_ten()) continue;
					// Trying moves that insert nodeU directly after the depot
					if (nodeV->pre->isDepot){
						nodeV = nodeV->pre;
						setLocalVariableRouteV();
						if (Relocate_one()) continue; // RELOCATE
						if (Relocate_two()) continue; // RELOCATE
					}
				}
			}
			const std::vector<ResolvedDepotCandidate>* resolvedDepots = nullptr;
			std::uint64_t resolvedDepotOccupancyGeneration = 0U;
			const std::size_t depotCandidateCount =
				localSearchDepotCandidateDescriptors_[nodeU->cour].size();
			for (std::size_t depotCandidateIndex = 0U;
				 depotCandidateIndex < depotCandidateCount;
				 ++depotCandidateIndex) {
				if (resolvedDepots == nullptr ||
					resolvedDepotOccupancyGeneration !=
						depotOccupancyGeneration_) {
					resolvedDepots =
						&resolvedDepotCandidates(nodeU->cour);
					resolvedDepotOccupancyGeneration =
						depotOccupancyGeneration_;
				}
				const ResolvedDepotCandidate candidate =
					(*resolvedDepots)[depotCandidateIndex];
#if !defined(NDEBUG) || defined(HGASP_ENABLE_HOT_PATH_COUNTERS)
				++lastTwMetrics_.candidateVertices;
#endif
				if (!candidate.fixedConfigurationAllowed ||
					candidate.emptyRouteDepot == nullptr) {
					continue;
				}
				nodeV = candidate.emptyRouteDepot;
				if (loadedUTopology != nbMoves) {
					setLocalVariableRouteU();
					loadedUTopology = nbMoves;
				}
				setLocalVariableRouteV();
				if (Relocate_one()) continue;
				if (Relocate_two()) continue;
				if (Relocate_three()) continue;
			}
			//begin split operators// the main function is to add routes
			if (loadedUTopology != nbMoves) {
				setLocalVariableRouteU();
				loadedUTopology = nbMoves;
			}
			if (resolvedDepots == nullptr ||
				resolvedDepotOccupancyGeneration !=
					depotOccupancyGeneration_) {
				resolvedDepots = &resolvedDepotCandidates(nodeU->cour);
				resolvedDepotOccupancyGeneration =
					depotOccupancyGeneration_;
			}
			if (Splitb(*resolvedDepots))continue;
			if (Splitf(*resolvedDepots))continue;
		}/// end the CVRP neighborhoods search
	}
}

bool LS::Splitf(){
	return Splitf(resolvedDepotCandidates(nodeUIndex));
}

bool LS::Splitf(
		const std::vector<ResolvedDepotCandidate>& candidates) {
	for (const ResolvedDepotCandidate& candidate : candidates) {
		const int loc = candidate.node;
		if (loc == nodeU->route->whichLocation->cour) continue;
		if (!candidate.fixedConfigurationAllowed) continue;
#if !defined(NDEBUG) || defined(HGASP_ENABLE_HOT_PATH_COUNTERS)
		++lastTwMetrics_.candidateVertices;
#endif
		if (SplitfForDepot(candidate)) return true;
	}
	return false;
}

HGASP_LS_ALWAYS_INLINE bool LS::SplitfForDepot(
		const ResolvedDepotCandidate& candidate) {
	const int loc = candidate.node;
	recordHgampCandidate(HgampMoveKind::SplitPrefix);
	Node* const beginU = nodeU->route->depot->next;
	const int movedLoad = nodeU->cumulatedLoad;
	NodeDepot* const sourceLocation = nodeU->route->whichLocation;
	NodeDepot* const targetLocation = candidate.location;
	if (targetLocation->remainInven + movedLoad >
		targetLocation->inventory) {
		return false;
	}
	if (candidate.emptyRouteDepot == nullptr) return false;
	const bool sourceRouteRemoved = nodeU->next->isDepot;
	if (!hasPromisingArc({
			{sourceLocation->cour, nodeXIndex},
			{loc, beginU->cour},
			{nodeUIndex, loc}})) return false;
	if (!fixedConfigurationAllowsDepotState(
			sourceLocation,
			!(sourceLocation->splitTime == 1 && sourceRouteRemoved)) ||
		!fixedConfigurationAllowsDepotState(targetLocation, true)) {
		return false;
	}
	moveGain = compute(s->location[loc-numV].used, data->openCostLoc[loc])
		- compute(!(sourceLocation->splitTime == 1 && sourceRouteRemoved),
			data->openCostLoc[sourceLocation->cour])
		- compute(!sourceRouteRemoved, data->usingCostVeh)
		+ data->usingCostVeh
		// Directed boundary replacement:
		// S->begin ... U->X ... S becomes S->X ... S and
		// T->begin ... U->T.
		+ data->D[sourceLocation->cour][nodeXIndex]
		- data->D[sourceLocation->cour][beginU->cour]
		- data->D[nodeUIndex][nodeXIndex]
		+ data->D[loc][beginU->cour]
		+ data->D[nodeUIndex][loc];

	if (moveGain > -Min) return false;

	Node* xx = nodeU;
	Route* const rr = candidate.emptyRouteDepot->route;
	const std::array<CapacityRouteBeforeState, 2U> before{{
		captureRouteBefore(routeU),
		captureRouteBefore(rr),
	}};
	while (!xx->isDepot) {
		xx->route = rr;
		xx = xx->pre;
	}
	Node* const pre = beginU->pre;
	Node* const next = nodeX;
	pre->next = next;
	next->pre = pre;
	beginU->pre = rr->depot;
	rr->depot->next = beginU;
	nodeU->next = rr->endDepot;
	rr->endDepot->pre = nodeU;
	++nbMoves;
	searchComplete = false;
	refreshChangedRoutesAndDepots(before, 2U);
	recordHgampAccepted(HgampMoveKind::SplitPrefix);
	return true;
}
bool LS::Splitb(){
	return Splitb(resolvedDepotCandidates(nodeUIndex));
}

bool LS::Splitb(
		const std::vector<ResolvedDepotCandidate>& candidates) {
	for (const ResolvedDepotCandidate& candidate : candidates) {
		const int loc = candidate.node;
		if (loc == nodeU->route->whichLocation->cour) continue;
		if (!candidate.fixedConfigurationAllowed) continue;
#if !defined(NDEBUG) || defined(HGASP_ENABLE_HOT_PATH_COUNTERS)
		++lastTwMetrics_.candidateVertices;
#endif
		if (SplitbForDepot(candidate)) return true;
	}
	return false;
}

HGASP_LS_ALWAYS_INLINE bool LS::SplitbForDepot(
		const ResolvedDepotCandidate& candidate) {
	const int loc = candidate.node;
	recordHgampCandidate(HgampMoveKind::SplitSuffix);
	Node* const lastU = nodeU->route->endDepot->pre;
	const int movedLoad =
		nodeU->route->load - nodeU->pre->cumulatedLoad;
	NodeDepot* const sourceLocation = nodeU->route->whichLocation;
	NodeDepot* const targetLocation = candidate.location;
	if (targetLocation->remainInven + movedLoad >
		targetLocation->inventory) {
		return false;
	}
	if (candidate.emptyRouteDepot == nullptr) return false;
	const bool sourceRouteRemoved = nodeU->pre->isDepot;
	if (!hasPromisingArc({
			{nodeUPrevIndex, sourceLocation->cour},
			{loc, nodeUIndex},
			{lastU->cour, loc}})) return false;
	if (!fixedConfigurationAllowsDepotState(
			sourceLocation,
			!(sourceLocation->splitTime == 1 && sourceRouteRemoved)) ||
		!fixedConfigurationAllowsDepotState(targetLocation, true)) {
		return false;
	}
	moveGain = compute(s->location[loc-numV].used, data->openCostLoc[loc])
		- compute(!(sourceLocation->splitTime == 1 && sourceRouteRemoved),
			data->openCostLoc[sourceLocation->cour])
		- compute(!sourceRouteRemoved, data->usingCostVeh)
		+ data->usingCostVeh
		// Directed boundary replacement:
		// S ... P->U ... last->S becomes S ... P->S and
		// T->U ... last->T.
		+ data->D[nodeUPrevIndex][sourceLocation->cour]
		- data->D[nodeUPrevIndex][nodeUIndex]
		- data->D[lastU->cour][sourceLocation->cour]
		+ data->D[loc][nodeUIndex]
		+ data->D[lastU->cour][loc];

	if (moveGain > -Min) return false;
	Node* xx = nodeU;
	Route* const rr = candidate.emptyRouteDepot->route;
	const std::array<CapacityRouteBeforeState, 2U> before{{
		captureRouteBefore(routeU),
		captureRouteBefore(rr),
	}};
	while (!xx->isDepot) {
		xx->route = rr;
		xx = xx->next;
	}
	Node* const pre = nodeU->pre;
	Node* const next = lastU->next;
	pre->next = next;
	next->pre = pre;
	nodeU->pre = rr->depot;
	rr->depot->next = nodeU;
	lastU->next = rr->endDepot;
	rr->endDepot->pre = lastU;
	++nbMoves;
	searchComplete = false;
	refreshChangedRoutesAndDepots(before, 2U);
	recordHgampAccepted(HgampMoveKind::SplitSuffix);
	return true;
}
HGASP_LS_ALWAYS_INLINE bool LS::Relocate_one(){
	recordHgampCandidate(HgampMoveKind::M1);
    // Reject overlapping nodes that would create a self-loop.
	if (nodeUIndex == nodeYIndex || nodeUIndex == nodeVIndex) return false;
    // A cross-depot move must respect the destination depot capacity.
    if (nodeU->route->whichLocation != nodeV->route->whichLocation) {
	    if (!(nodeV->route->whichLocation->remainInven + nodeU->dem <= nodeV->route->whichLocation->inventory))
		    return false;
		if (!fixedConfigurationAllowsDepotState(
				nodeU->route->whichLocation,
				!(nodeU->route->whichLocation->splitTime == 1 &&
				  nodeU->route->nbClients == 1)) ||
			!fixedConfigurationAllowsDepotState(
				nodeV->route->whichLocation, true)) return false;
    }
	if(nodeV->route != nodeU->route && nodeV->route->load + nodeU->dem > data->vehCap)return false;// route feasible
	if (!hasPromisingArc({
			{nodeUPrevIndex, nodeXIndex},
			{nodeVIndex, nodeUIndex},
			{nodeUIndex, nodeYIndex}})) return false;

	moveGain = compute(nodeV->route->whichLocation->used, data->openCostLoc[nodeV->route->whichLocation->cour])
			+ compute(!(nodeV->isDepot && nodeY->isDepot), data->usingCostVeh)
			- compute(!(nodeU->route->whichLocation->splitTime == 1 && nodeU->route->nbClients == 1 && nodeU->route->whichLocation != nodeV->route->whichLocation), data->openCostLoc[nodeU->route->whichLocation->cour])
			- compute(nodeU->route->nbClients != 1, data->usingCostVeh)
			+ data->D[nodeUPrevIndex][nodeXIndex]-data->D[nodeUPrevIndex][nodeUIndex]-data->D[nodeUIndex][nodeXIndex]
			+ data->D[nodeVIndex][nodeUIndex]+data->D[nodeUIndex][nodeYIndex]-data->D[nodeVIndex][nodeYIndex];
	//
	if (moveGain > -Min)return false;
	const std::array<CapacityRouteBeforeState, 2U> before{{
		captureRouteBefore(routeU),
		captureRouteBefore(routeV),
	}};
	insertNode(nodeU,nodeV);

	nbMoves++;
	searchComplete=false;
	refreshChangedRoutesAndDepots(before, 2U);
	recordHgampAccepted(HgampMoveKind::M1);
	return true;
}

HGASP_LS_ALWAYS_INLINE bool LS::Relocate_two(){
	recordHgampCandidate(HgampMoveKind::M2);
    // Reject overlapping nodes and two-customer blocks ending at a depot.
	if (nodeUIndex == nodeYIndex || nodeXIndex == nodeVIndex || nodeUIndex == nodeVIndex || nodeX->isDepot) return false;
    // Same-depot moves do not change depot load.
	if (nodeU->route->whichLocation != nodeV->route->whichLocation) {
        if (!(nodeV->route->whichLocation->remainInven + nodeU->dem + nodeX->dem <= nodeV->route->whichLocation->inventory))
            return false;
		if (!fixedConfigurationAllowsDepotState(
				nodeU->route->whichLocation,
				!(nodeU->route->whichLocation->splitTime == 1 &&
				  nodeU->route->nbClients == 2)) ||
			!fixedConfigurationAllowsDepotState(
				nodeV->route->whichLocation, true)) return false;
    }
	if(nodeV->route != nodeU->route && nodeV->route->load + nodeU->dem + nodeX->dem > data->vehCap)return false;
	if (!hasPromisingArc({
			{nodeUPrevIndex, nodeXNextIndex},
			{nodeVIndex, nodeUIndex},
			{nodeXIndex, nodeYIndex}})) return false;
	
    // Evaluate insertion of the ordered block U -> X after V.
	moveGain = compute(nodeV->route->whichLocation->used, data->openCostLoc[nodeV->route->whichLocation->cour])
			+ compute(!(nodeV->isDepot && nodeY->isDepot), data->usingCostVeh)
			- compute(!(nodeU->route->whichLocation->splitTime == 1 && nodeU->route->nbClients == 2 && nodeU->route->whichLocation != nodeV->route->whichLocation), data->openCostLoc[nodeU->route->whichLocation->cour])
			- compute(nodeU->route->nbClients != 2, data->usingCostVeh)
			+ data->D[nodeUPrevIndex][nodeXNextIndex]-data->D[nodeUPrevIndex][nodeUIndex]-data->D[nodeXIndex][nodeXNextIndex]
			+ data->D[nodeVIndex][nodeUIndex]+data->D[nodeXIndex][nodeYIndex]-data->D[nodeVIndex][nodeYIndex];
	
	if (moveGain>-Min)return false;
	const std::array<CapacityRouteBeforeState, 2U> before{{
		captureRouteBefore(routeU),
		captureRouteBefore(routeV),
	}};
    
    // Preserve the U -> X order while inserting the block after V.
	insertNode(nodeU,nodeV);
	insertNode(nodeX,nodeU);

	nbMoves++;
	searchComplete=false;
	refreshChangedRoutesAndDepots(before, 2U);
	recordHgampAccepted(HgampMoveKind::M2);
	return true;
}

HGASP_LS_ALWAYS_INLINE bool LS::Relocate_three(){
	recordHgampCandidate(HgampMoveKind::M3);
    // Reject overlapping nodes and two-customer blocks ending at a depot.
	if (nodeXIndex == nodeVIndex || nodeUIndex == nodeVIndex ||
		nodeX->isDepot) return false;
	
    // Same-depot moves do not change depot load.
    if (nodeU->route->whichLocation != nodeV->route->whichLocation) {
        if (!(nodeV->route->whichLocation->remainInven + nodeU->dem + nodeX->dem <= nodeV->route->whichLocation->inventory))
            return false;
		if (!fixedConfigurationAllowsDepotState(
				nodeU->route->whichLocation,
				!(nodeU->route->whichLocation->splitTime == 1 &&
				  nodeU->route->nbClients == 2)) ||
			!fixedConfigurationAllowsDepotState(
				nodeV->route->whichLocation, true)) return false;
    }
	if(nodeV->route != nodeU->route && nodeV->route->load + nodeU->dem + nodeX->dem > data->vehCap)return false;
	
	Node* const targetPredecessor =
		nodeV->isDepot ? nodeV : nodeV->pre;
	if (routeU == routeV && nodeV == nodeX->next) {
		if (!hasPromisingArc({
				{nodeUPrevIndex, nodeXIndex},
				{nodeXIndex, nodeUIndex},
				{nodeUIndex, nodeVIndex}})) return false;
	} else if (routeU == routeV && nodeV == nodeU->pre) {
		if (!hasPromisingArc({
				{nodeVPrevIndex, nodeXIndex},
				{nodeXIndex, nodeUIndex},
				{nodeUIndex, nodeVIndex},
				{nodeVIndex, nodeXNextIndex}})) return false;
	} else if (!hasPromisingArc({
			{nodeUPrevIndex, nodeXNextIndex},
			{targetPredecessor->cour, nodeXIndex},
			{nodeXIndex, nodeUIndex},
			{nodeUIndex, nodeVIndex}})) {
		return false;
	}
	// M3 places reversed (x,u) before v. The official implementation places
	// it after v and also leaves moveGain uninitialized on one branch; use the
	// paper topology and one total directed formula here.
	if (routeU == routeV && nodeV == nodeX->next) {
		moveGain = data->D[nodeUPrevIndex][nodeXIndex] +
			data->D[nodeXIndex][nodeUIndex] +
			data->D[nodeUIndex][nodeVIndex] -
			data->D[nodeUPrevIndex][nodeUIndex] -
			data->D[nodeUIndex][nodeXIndex] -
			data->D[nodeXIndex][nodeVIndex];
	} else if (routeU == routeV && nodeV == nodeU->pre) {
		moveGain = data->D[nodeVPrevIndex][nodeXIndex] +
			data->D[nodeXIndex][nodeUIndex] +
			data->D[nodeUIndex][nodeVIndex] +
			data->D[nodeVIndex][nodeXNextIndex] -
			data->D[nodeVPrevIndex][nodeVIndex] -
			data->D[nodeVIndex][nodeUIndex] -
			data->D[nodeUIndex][nodeXIndex] -
			data->D[nodeXIndex][nodeXNextIndex];
	} else {
		moveGain = compute(nodeV->route->whichLocation->used, data->openCostLoc[nodeV->route->whichLocation->cour])
			+ compute(!(nodeV->isDepot && nodeY->isDepot), data->usingCostVeh)
			- compute(!(nodeU->route->whichLocation->splitTime == 1 && nodeU->route->nbClients == 2 && nodeU->route->whichLocation != nodeV->route->whichLocation), data->openCostLoc[nodeU->route->whichLocation->cour])
			- compute(nodeU->route->nbClients != 2, data->usingCostVeh)
			+ data->D[nodeUPrevIndex][nodeXNextIndex]
			- data->D[nodeUPrevIndex][nodeUIndex]
			- data->D[nodeUIndex][nodeXIndex]
			- data->D[nodeXIndex][nodeXNextIndex]
			+ data->D[targetPredecessor->cour][nodeXIndex]
			+ data->D[nodeXIndex][nodeUIndex]
			+ data->D[nodeUIndex][nodeVIndex]
			- data->D[targetPredecessor->cour][nodeVIndex];
	}
	
	if (moveGain>-Min)return false;
	const std::array<CapacityRouteBeforeState, 2U> before{{
		captureRouteBefore(routeU),
		captureRouteBefore(routeV),
	}};
    
	applyLinkedReversedPairBefore(nodeU, nodeV);

	nbMoves++;
	searchComplete=false;
	refreshChangedRoutesAndDepots(before, 2U);
	recordHgampAccepted(HgampMoveKind::M3);
	return true;
}

HGASP_LS_ALWAYS_INLINE bool LS::Relocate_four(){
	recordHgampCandidate(HgampMoveKind::M4);
	Node* const a = nodeX; // HGAMP M4 exchanges x=succ(u), not u.
	Node* const b = nodeV;
	if (a->isDepot || b->isDepot || a == b) return false;
	Route* const ra = a->route;
	Route* const rb = b->route;
	if (ra != rb) {
		if (ra->load - a->dem + b->dem > data->vehCap ||
			rb->load - b->dem + a->dem > data->vehCap) return false;
		if (ra->whichLocation != rb->whichLocation &&
			(ra->whichLocation->remainInven - a->dem + b->dem >
				ra->whichLocation->inventory ||
			 rb->whichLocation->remainInven - b->dem + a->dem >
				rb->whichLocation->inventory)) return false;
	}
	if (a->next == b) {
		if (!hasPromisingArc({
				{a->pre->cour, b->cour}, {b->cour, a->cour},
				{a->cour, b->next->cour}})) return false;
	} else if (b->next == a) {
		if (!hasPromisingArc({
				{b->pre->cour, a->cour}, {a->cour, b->cour},
				{b->cour, a->next->cour}})) return false;
	} else if (!hasPromisingArc({
			{a->pre->cour, b->cour}, {b->cour, a->next->cour},
			{b->pre->cour, a->cour}, {a->cour, b->next->cour}})) {
		return false;
	}
	// The reverse exchange can only occur when predecessor(b) also lists a.
	// Avoid touching the dense pair cache for the overwhelmingly common
	// one-direction-only candidates.
	if (promisingArcFilterEnabled() && !b->pre->isDepot &&
		promisingArcGraph_.containsKnownArc(b->pre->cour, a->cour) &&
		canonicalPairAlreadyEvaluated(
			HgampMoveKind::M4, a->cour, b->cour)) return false;
	if (a->next == b) {
		moveGain = data->D[a->pre->cour][b->cour] +
			data->D[b->cour][a->cour] + data->D[a->cour][b->next->cour] -
			data->D[a->pre->cour][a->cour] - data->D[a->cour][b->cour] -
			data->D[b->cour][b->next->cour];
	} else if (b->next == a) {
		moveGain = data->D[b->pre->cour][a->cour] +
			data->D[a->cour][b->cour] + data->D[b->cour][a->next->cour] -
			data->D[b->pre->cour][b->cour] - data->D[b->cour][a->cour] -
			data->D[a->cour][a->next->cour];
	} else {
		moveGain = data->D[a->pre->cour][b->cour] +
			data->D[b->cour][a->next->cour] + data->D[b->pre->cour][a->cour] +
			data->D[a->cour][b->next->cour] - data->D[a->pre->cour][a->cour] -
			data->D[a->cour][a->next->cour] - data->D[b->pre->cour][b->cour] -
			data->D[b->cour][b->next->cour];
	}
	if (moveGain > -Min) return false;
	const std::array<CapacityRouteBeforeState, 2U> before{{
		captureRouteBefore(ra),
		captureRouteBefore(rb),
	}};
	applyLinkedSegmentExchange(ra, a, 1U, rb, b, 1U);
	++nbMoves; searchComplete = false;
	refreshChangedRoutesAndDepots(before, 2U);
	recordHgampAccepted(HgampMoveKind::M4);
	return true;
}

HGASP_LS_ALWAYS_INLINE bool LS::Relocate_five(){
	recordHgampCandidate(HgampMoveKind::M5);
	Node* const a = nodeX; // HGAMP M5: (x,x+) <-> v.
	Node* const b = nodeV;
	if (a->isDepot || a->next->isDepot || b->isDepot) return false;
	Node* const z = a->next;
	if (b == a || b == z) return false;
	Route* const ra = a->route;
	Route* const rb = b->route;
	const int loadA = a->dem + z->dem;
	if (ra != rb) {
		if (ra->load - loadA + b->dem > data->vehCap ||
			rb->load - b->dem + loadA > data->vehCap) return false;
		if (ra->whichLocation != rb->whichLocation &&
			(ra->whichLocation->remainInven - loadA + b->dem >
				ra->whichLocation->inventory ||
			 rb->whichLocation->remainInven - b->dem + loadA >
				rb->whichLocation->inventory)) return false;
	}
	if (z->next == b) {
		if (!hasPromisingArc({
				{a->pre->cour, b->cour}, {b->cour, a->cour},
				{z->cour, b->next->cour}})) return false;
	} else if (b->next == a) {
		if (!hasPromisingArc({
				{b->pre->cour, a->cour}, {z->cour, b->cour},
				{b->cour, z->next->cour}})) return false;
	} else if (!hasPromisingArc({
			{a->pre->cour, b->cour}, {b->cour, z->next->cour},
			{b->pre->cour, a->cour}, {z->cour, b->next->cour}})) {
		return false;
	}
	if (z->next == b) {
		moveGain = data->D[a->pre->cour][b->cour] +
			data->D[b->cour][a->cour] + data->D[z->cour][b->next->cour] -
			data->D[a->pre->cour][a->cour] - data->D[z->cour][b->cour] -
			data->D[b->cour][b->next->cour];
	} else if (b->next == a) {
		moveGain = data->D[b->pre->cour][a->cour] +
			data->D[z->cour][b->cour] + data->D[b->cour][z->next->cour] -
			data->D[b->pre->cour][b->cour] - data->D[b->cour][a->cour] -
			data->D[z->cour][z->next->cour];
	} else {
		moveGain = data->D[a->pre->cour][b->cour] +
			data->D[b->cour][z->next->cour] + data->D[b->pre->cour][a->cour] +
			data->D[z->cour][b->next->cour] - data->D[a->pre->cour][a->cour] -
			data->D[z->cour][z->next->cour] - data->D[b->pre->cour][b->cour] -
			data->D[b->cour][b->next->cour];
	}
	if (moveGain > -Min) return false;
	const std::array<CapacityRouteBeforeState, 2U> before{{
		captureRouteBefore(ra),
		captureRouteBefore(rb),
	}};
	applyLinkedSegmentExchange(ra, a, 2U, rb, b, 1U);
	++nbMoves; searchComplete = false;
	refreshChangedRoutesAndDepots(before, 2U);
	recordHgampAccepted(HgampMoveKind::M5);
	return true;
}

HGASP_LS_ALWAYS_INLINE bool LS::Relocate_six(){
	recordHgampCandidate(HgampMoveKind::M6);
	Node* const a = nodeX; // HGAMP M6: (x,x+) <-> (v,y).
	Node* const b = nodeV;
	if (a->isDepot || a->next->isDepot || b->isDepot || b->next->isDepot)
		return false;
	Node* const z = a->next;
	Node* const y = b->next;
	if (a == b || a == y || z == b || z == y) return false;
	Route* const ra = a->route;
	Route* const rb = b->route;
	const int loadA = a->dem + z->dem;
	const int loadB = b->dem + y->dem;
	if (ra != rb) {
		if (ra->load - loadA + loadB > data->vehCap ||
			rb->load - loadB + loadA > data->vehCap) return false;
		if (ra->whichLocation != rb->whichLocation &&
			(ra->whichLocation->remainInven - loadA + loadB >
				ra->whichLocation->inventory ||
			 rb->whichLocation->remainInven - loadB + loadA >
				rb->whichLocation->inventory)) return false;
	}
	if (z->next == b) {
		if (!hasPromisingArc({
				{a->pre->cour, b->cour}, {y->cour, a->cour},
				{z->cour, y->next->cour}})) return false;
	} else if (y->next == a) {
		if (!hasPromisingArc({
				{b->pre->cour, a->cour}, {z->cour, b->cour},
				{y->cour, z->next->cour}})) return false;
	} else if (!hasPromisingArc({
			{a->pre->cour, b->cour}, {y->cour, z->next->cour},
			{b->pre->cour, a->cour}, {z->cour, y->next->cour}})) {
		return false;
	}
	if (promisingArcFilterEnabled() && !b->pre->isDepot &&
		promisingArcGraph_.containsKnownArc(b->pre->cour, a->cour) &&
		canonicalPairAlreadyEvaluated(
			HgampMoveKind::M6, a->cour, b->cour)) return false;
	if (z->next == b) {
		moveGain = data->D[a->pre->cour][b->cour] +
			data->D[y->cour][a->cour] + data->D[z->cour][y->next->cour] -
			data->D[a->pre->cour][a->cour] - data->D[z->cour][b->cour] -
			data->D[y->cour][y->next->cour];
	} else if (y->next == a) {
		moveGain = data->D[b->pre->cour][a->cour] +
			data->D[z->cour][b->cour] + data->D[y->cour][z->next->cour] -
			data->D[b->pre->cour][b->cour] - data->D[y->cour][a->cour] -
			data->D[z->cour][z->next->cour];
	} else {
		moveGain = data->D[a->pre->cour][b->cour] +
			data->D[y->cour][z->next->cour] + data->D[b->pre->cour][a->cour] +
			data->D[z->cour][y->next->cour] - data->D[a->pre->cour][a->cour] -
			data->D[z->cour][z->next->cour] - data->D[b->pre->cour][b->cour] -
			data->D[y->cour][y->next->cour];
	}
	if (moveGain > -Min) return false;
	const std::array<CapacityRouteBeforeState, 2U> before{{
		captureRouteBefore(ra),
		captureRouteBefore(rb),
	}};
	applyLinkedSegmentExchange(ra, a, 2U, rb, b, 2U);
	++nbMoves; searchComplete = false;
	refreshChangedRoutesAndDepots(before, 2U);
	recordHgampAccepted(HgampMoveKind::M6);
	return true;
}

HGASP_LS_ALWAYS_INLINE bool LS::Relocate_seven(){
	recordHgampCandidate(HgampMoveKind::M7);
	Node* const a = nodeX; // HGAMP M7: reversed (x+,x) <-> (v,y).
	Node* const b = nodeV;
	if (a->isDepot || a->next->isDepot || b->isDepot || b->next->isDepot)
		return false;
	Node* const z = a->next;
	Node* const y = b->next;
	if (a == b || a == y || z == b || z == y) return false;
	Route* const ra = a->route;
	Route* const rb = b->route;
	const int loadA = a->dem + z->dem;
	const int loadB = b->dem + y->dem;
	if (ra != rb) {
		if (ra->load - loadA + loadB > data->vehCap ||
			rb->load - loadB + loadA > data->vehCap) return false;
		if (ra->whichLocation != rb->whichLocation &&
			(ra->whichLocation->remainInven - loadA + loadB >
				ra->whichLocation->inventory ||
			 rb->whichLocation->remainInven - loadB + loadA >
				rb->whichLocation->inventory)) return false;
	}
	if (z->next == b) {
		if (!hasPromisingArc({
				{a->pre->cour, b->cour}, {y->cour, z->cour},
				{z->cour, a->cour}, {a->cour, y->next->cour}})) return false;
	} else if (y->next == a) {
		if (!hasPromisingArc({
				{b->pre->cour, z->cour}, {z->cour, a->cour},
				{a->cour, b->cour}, {y->cour, z->next->cour}})) return false;
	} else if (!hasPromisingArc({
			{a->pre->cour, b->cour}, {y->cour, z->next->cour},
			{b->pre->cour, z->cour}, {z->cour, a->cour},
			{a->cour, y->next->cour}})) {
		return false;
	}
	// M7 reverses only its first block, so swapping the two blocks is not an
	// equivalent move. The directed candidate lists contain no duplicates;
	// consequently each semantics-equivalent oriented M7 key is already
	// canonical by construction and needs no runtime pair-cache probe.
	if (z->next == b) {
		moveGain = data->D[a->pre->cour][b->cour] +
			data->D[y->cour][z->cour] + data->D[z->cour][a->cour] +
			data->D[a->cour][y->next->cour] -
			data->D[a->pre->cour][a->cour] - data->D[a->cour][z->cour] -
			data->D[z->cour][b->cour] - data->D[y->cour][y->next->cour];
	} else if (y->next == a) {
		moveGain = data->D[b->pre->cour][z->cour] +
			data->D[z->cour][a->cour] + data->D[a->cour][b->cour] +
			data->D[y->cour][z->next->cour] -
			data->D[b->pre->cour][b->cour] - data->D[y->cour][a->cour] -
			data->D[a->cour][z->cour] - data->D[z->cour][z->next->cour];
	} else {
		moveGain = data->D[a->pre->cour][b->cour] +
			data->D[y->cour][z->next->cour] + data->D[b->pre->cour][z->cour] +
			data->D[z->cour][a->cour] + data->D[a->cour][y->next->cour] -
			data->D[a->pre->cour][a->cour] - data->D[a->cour][z->cour] -
			data->D[z->cour][z->next->cour] - data->D[b->pre->cour][b->cour] -
			data->D[y->cour][y->next->cour];
	}
	if (moveGain > -Min) return false;
	const std::array<CapacityRouteBeforeState, 2U> before{{
		captureRouteBefore(ra),
		captureRouteBefore(rb),
	}};
	applyLinkedSegmentExchange(ra, a, 2U, rb, b, 2U, true);
	++nbMoves; searchComplete = false;
	refreshChangedRoutesAndDepots(before, 2U);
	recordHgampAccepted(HgampMoveKind::M7);
	return true;
}

HGASP_LS_ALWAYS_INLINE bool LS::Relocate_eight(){
	recordHgampCandidate(HgampMoveKind::M8);
    // The 2-opt segment must be non-overlapping and belong to one route.
    if (nodeU->route != nodeV->route) return false;
    if (nodeX->isDepot || nodeV->isDepot || nodeU->isDepot) return false;
    if (nodeU == nodeV || nodeX == nodeV || nodeU == nodeY || nodeX == nodeY) return false;

	// Positions are refreshed with route information and give route order in
	// O(1), including valid zero-demand instances. The successor y may be the
	// end depot when v is the route's final customer.
	if (nodeX->position > nodeV->position) return false;
	ensureReverseArcPrefix(nodeU->route);
	const bool reversedSegmentPromising = !promisingArcFilterEnabled() ||
		reversePromisingArcPrefix_[static_cast<std::size_t>(nodeV->cour)] -
			reversePromisingArcPrefix_[static_cast<std::size_t>(nodeX->cour)] > 0;
	if (!hasPromisingArc({
			{nodeUIndex, nodeVIndex}, {nodeXIndex, nodeYIndex}}) &&
		!reversedSegmentPromising) return false;

    // Evaluate replacement boundary edges.
    moveGain = data->D[nodeUIndex][nodeVIndex] + data->D[nodeXIndex][nodeYIndex]
             - data->D[nodeUIndex][nodeXIndex] - data->D[nodeVIndex][nodeYIndex];

    // Both directed internal sums are O(1): updateRouteInfor already stores
    // the forward prefix, while this route-versioned prefix stores reverse
    // arcs. Only an accepted reversal walks the segment to change links.
	const double forwardInternal =
		nodeV->cumulatedDis - nodeX->cumulatedDis;
    const double reverseInternal =
        reverseArcPrefix_[static_cast<std::size_t>(nodeV->cour)] -
        reverseArcPrefix_[static_cast<std::size_t>(nodeX->cour)];
    moveGain += reverseInternal - forwardInternal;

    if (moveGain > -Min) return false;
	const std::array<CapacityRouteBeforeState, 2U> before{{
		captureRouteBefore(nodeU->route),
		CapacityRouteBeforeState{},
	}};

    // Reverse the segment's links.
    Node* curr = nodeX;
    Node* stopNode = nodeY;
    while (curr != stopNode) {
        Node* nxt = curr->next;
        Node* temp = curr->next;
        curr->next = curr->pre;
        curr->pre = temp;
        curr = nxt;
    }

    // Reconnect the segment boundaries.
    nodeU->next = nodeV;
    nodeV->pre  = nodeU;

    nodeX->next = nodeY;
    nodeY->pre  = nodeX;

	nbMoves++;
	searchComplete = false;
	refreshChangedRoutesAndDepots(before, 1U);
	recordHgampAccepted(HgampMoveKind::M8);
	return true;
}

HGASP_LS_ALWAYS_INLINE bool LS::Relocate_nine() {
	recordHgampCandidate(HgampMoveKind::M9);
	if (routeU == routeV || nodeV->isDepot) return false;
	Node* const lastA = routeU->endDepot->pre;
	Node* const lastB = routeV->endDepot->pre;
	const int tailA = routeU->load - nodeU->cumulatedLoad;
	const int tailB = routeV->load - nodeV->pre->cumulatedLoad;
	const int newLoadA = nodeU->cumulatedLoad + tailB;
	const int newLoadB = nodeV->pre->cumulatedLoad + tailA;
	if (newLoadA > data->vehCap || newLoadB > data->vehCap) return false;
	NodeDepot* const locA = routeU->whichLocation;
	NodeDepot* const locB = routeV->whichLocation;
	if (locA != locB &&
		(locA->remainInven - tailA + tailB > locA->inventory ||
		 locB->remainInven - tailB + tailA > locB->inventory)) return false;
	if (fixedConfigurationMode_ && locA != locB) {
		// updateRouteInfor keeps one-based customer positions and nbClients
		// authoritative after every accepted move.  M9 leaves the prefix before
		// v in B and appends the suffix strictly after u from A, so its resulting
		// occupancy is available in O(1).  Do not materialize both full routes
		// for every rejected candidate in fixed-population local search.
		const int newCountB = nodeV->position - 1 +
			routeU->nbClients - nodeU->position;
		if (newCountB < 0) {
			throw std::logic_error("M9 derived a negative route occupancy");
		}
		if (!fixedConfigurationAllowsDepotState(
				locB, locB->splitTime > 1 || newCountB > 0)) return false;
	}
	const int depotA = locA->cour;
	const int depotB = locB->cour;
	Node* const beforeV = nodeV->pre;
	if (!hasPromisingArc({
			{nodeU->cour, nodeV->cour}, {lastB->cour, depotA},
			{beforeV->cour, nodeX->cour}, {lastA->cour, depotB}})) {
		return false;
	}
	const double newCostA = nodeU->cumulatedDis +
		data->D[nodeU->cour][nodeV->cour] +
		(lastB->cumulatedDis - nodeV->cumulatedDis) +
		data->D[lastB->cour][depotA];
	double newCostB = 0.0;
	if (newLoadB > 0) {
		newCostB = beforeV->isDepot ? 0.0 : beforeV->cumulatedDis;
		if (!nodeX->isDepot) {
			newCostB += data->D[beforeV->cour][nodeX->cour] +
				(lastA->cumulatedDis - nodeX->cumulatedDis) +
				data->D[lastA->cour][depotB];
		} else {
			newCostB += data->D[beforeV->cour][depotB];
		}
	}
	moveGain = newCostA + newCostB - routeU->dis - routeV->dis;
	if (newLoadB == 0) {
		moveGain -= data->usingCostVeh;
		if (locA != locB && locB->splitTime == 1)
			moveGain -= data->openCostLoc[depotB];
	}
	if (moveGain > -Min) return false;
	Route* const firstRoute = routeU;
	Route* const secondRoute = routeV;
	const std::array<CapacityRouteBeforeState, 2U> before{{
		captureRouteBefore(firstRoute),
		captureRouteBefore(secondRoute),
	}};
	applyLinkedM9(nodeU, nodeV);
	++nbMoves; searchComplete = false;
	refreshChangedRoutesAndDepots(before, 2U);
	recordHgampAccepted(HgampMoveKind::M9);
	return true;
}

HGASP_LS_ALWAYS_INLINE bool LS::Relocate_ten() {
	recordHgampCandidate(HgampMoveKind::M10);
	if (routeU == routeV || nodeV->isDepot) return false;
	Node* const firstB = routeV->depot->next;
	Node* const lastA = routeU->endDepot->pre;
	Node* const lastB = routeV->endDepot->pre;
	const int prefixA = nodeU->cumulatedLoad;
	const int tailA = routeU->load - prefixA;
	const int prefixB = nodeV->cumulatedLoad;
	const int tailB = routeV->load - prefixB;
	const int newLoadA = prefixA + prefixB;
	const int newLoadB = tailA + tailB;
	if (newLoadA > data->vehCap || newLoadB > data->vehCap) return false;
	NodeDepot* const locA = routeU->whichLocation;
	NodeDepot* const locB = routeV->whichLocation;
	if (locA != locB &&
		(locA->remainInven - tailA + prefixB > locA->inventory ||
		 locB->remainInven - prefixB + tailA > locB->inventory)) return false;
	if (fixedConfigurationMode_ && locA != locB) {
		// M10 places the suffix strictly after u and the suffix strictly after v
		// in B.  The maintained one-based positions make the exact structural
		// occupancy available without two route walks and temporary vectors.
		const int newCountB =
			routeU->nbClients - nodeU->position +
			routeV->nbClients - nodeV->position;
		if (newCountB < 0) {
			throw std::logic_error("M10 derived a negative route occupancy");
		}
		if (!fixedConfigurationAllowsDepotState(
				locB, locB->splitTime > 1 || newCountB > 0)) return false;
	}
	ensureReverseArcPrefix(routeU);
	ensureReverseArcPrefix(routeV);
	const int depotA = locA->cour;
	const int depotB = locB->cour;
	const bool promisingBoundary = hasPromisingArc({
		{nodeU->cour, nodeV->cour}, {firstB->cour, depotA},
		{depotB, lastA->cour}, {nodeX->cour, nodeY->cour},
		{lastB->cour, depotB}});
	const bool promisingReversedPrefix = !promisingArcFilterEnabled() ||
		reversePromisingArcPrefix_[static_cast<std::size_t>(nodeV->cour)] -
			reversePromisingArcPrefix_[static_cast<std::size_t>(firstB->cour)] > 0;
	const bool promisingReversedTail = !nodeX->isDepot &&
		(!promisingArcFilterEnabled() ||
		 reversePromisingArcPrefix_[static_cast<std::size_t>(lastA->cour)] -
			reversePromisingArcPrefix_[static_cast<std::size_t>(nodeX->cour)] > 0);
	if (!promisingBoundary && !promisingReversedPrefix &&
		!promisingReversedTail) return false;
	const double reversedPrefixB =
		reverseArcPrefix_[static_cast<std::size_t>(nodeV->cour)] -
		reverseArcPrefix_[static_cast<std::size_t>(firstB->cour)];
	const double newCostA = nodeU->cumulatedDis +
		data->D[nodeU->cour][nodeV->cour] + reversedPrefixB +
		data->D[firstB->cour][depotA];
	double newCostB = 0.0;
	if (newLoadB > 0) {
		if (!nodeX->isDepot) {
			const double reversedTailA =
				reverseArcPrefix_[static_cast<std::size_t>(lastA->cour)] -
				reverseArcPrefix_[static_cast<std::size_t>(nodeX->cour)];
			newCostB = data->D[depotB][lastA->cour] + reversedTailA;
			if (!nodeY->isDepot) {
				newCostB += data->D[nodeX->cour][nodeY->cour] +
					(lastB->cumulatedDis - nodeY->cumulatedDis) +
					data->D[lastB->cour][depotB];
			} else {
				newCostB += data->D[nodeX->cour][depotB];
			}
		} else {
			newCostB = data->D[depotB][nodeY->cour] +
				(lastB->cumulatedDis - nodeY->cumulatedDis) +
				data->D[lastB->cour][depotB];
		}
	}
	moveGain = newCostA + newCostB - routeU->dis - routeV->dis;
	if (newLoadB == 0) {
		moveGain -= data->usingCostVeh;
		if (locA != locB && locB->splitTime == 1)
			moveGain -= data->openCostLoc[depotB];
	}
	if (moveGain > -Min) return false;
	Route* const firstRoute = routeU;
	Route* const secondRoute = routeV;
	const std::array<CapacityRouteBeforeState, 2U> before{{
		captureRouteBefore(firstRoute),
		captureRouteBefore(secondRoute),
	}};
	applyLinkedM10(nodeU, nodeV);
	++nbMoves; searchComplete = false;
	refreshChangedRoutesAndDepots(before, 2U);
	recordHgampAccepted(HgampMoveKind::M10);
	return true;
}

void LS::setLocalVariableRouteU(){
	routeU=nodeU->route;
	nodeX=nodeU->next;
	nodeXNextIndex=nodeX->next->cour;
	nodeUIndex = nodeU->cour;
	nodeUPrevIndex = nodeU->pre->cour;
	nodeXIndex = nodeX->cour;
}
void LS::setLocalVariableRouteV(){
	routeV=nodeV->route;
	nodeY=nodeV->next;
	nodeVIndex=nodeV->cour;
	nodeVPrevIndex=nodeV->pre->cour;
	nodeYIndex=nodeY->cour;
}
void LS::ensureReverseArcPrefix(Route* route) {
	if (route == nullptr || route->cour < 0 ||
		static_cast<std::size_t>(route->cour) >=
			reverseArcPrefixGeneration_.size()) {
		throw std::logic_error(
			"cannot build a reverse prefix for an invalid route slot");
	}
	const std::size_t routeIndex =
		static_cast<std::size_t>(route->cour);
	const int modificationVersion = routeModificationVersion(route);
	if (reverseArcPrefixInvocationGenerations_[routeIndex] ==
			currentInvocationGeneration_ &&
		reverseArcPrefixGeneration_[routeIndex] == modificationVersion) {
		return;
	}
	double prefix = 0.0;
	int promisingPrefix = 0;
	Node* previous = route->depot;
	for (Node* node = previous->next;
		 !node->isDepot;
		 previous = node, node = node->next) {
		prefix += data->D[node->cour][previous->cour];
		promisingPrefix += isPromisingArc(node->cour, previous->cour) ? 1 : 0;
		reverseArcPrefix_[static_cast<std::size_t>(node->cour)] = prefix;
		reversePromisingArcPrefix_[static_cast<std::size_t>(node->cour)] =
			promisingPrefix;
	}
	reverseArcPrefixGeneration_[routeIndex] = modificationVersion;
	reverseArcPrefixInvocationGenerations_[routeIndex] =
		currentInvocationGeneration_;
}

HGASP_LS_ALWAYS_INLINE bool LS::promisingArcFilterEnabled() const noexcept {
	return data != nullptr &&
		hgasp::isSetSProductionSearchProfile(data->parameters.profile);
}

HGASP_LS_ALWAYS_INLINE bool LS::isPromisingArc(
		const int from, const int to) const noexcept {
	if (!promisingArcFilterEnabled()) return true;
	return from != to && promisingArcGraph_.containsKnownArc(from, to);
}

HGASP_LS_ALWAYS_INLINE bool LS::hasPromisingArc(
		const std::initializer_list<std::pair<int, int>> arcs) const noexcept {
	if (!promisingArcFilterEnabled()) return true;
	for (const auto& arc : arcs) {
		if (arc.first != arc.second &&
			promisingArcGraph_.containsKnownArc(
				arc.first, arc.second)) return true;
	}
	return false;
}

void LS::insertNode(Node *U, Node *V){
	U->pre->next = U->next;
	U->next->pre = U->pre;
	V->next->pre = U;
	U->pre = V;
	U->next = V->next;
	V->next = U;
	U->route = V->route;
}
void LS::loadSolution(){
	hgasp::recordHotPathCounter(
		hgasp::HotPathCounter::LsLegacyResetCells,
		static_cast<std::uint64_t>(numL) +
		3U * static_cast<std::uint64_t>(numV) +
		2U * static_cast<std::uint64_t>(s->maxVnum));
	beginInvocationGeneration();
	nbMoves = 0;
}
void LS::local_search_run(Individual *s, double, double){
	runLocalSearch(s, true);
}

void LS::local_search_run_deferred_objective(
		Individual* const individual,
		double,
		double) {
	if (data->hasHardTimeWindows() ||
		!hgasp::isSetSProductionSearchProfile(
			data->parameters.profile)) {
		throw std::logic_error(
			"deferred LS objective refresh requires a role-aware Set S profile");
	}
	runLocalSearch(individual, false);
}

void LS::runLocalSearch(
		Individual* const individual,
		const bool finalizeCapacityObjective) {
	hgasp::ScopedComponentTimer timer(
		data->hasHardTimeWindows()
			? hgasp::TimedComponent::HardTwRouteLocalSearch
			: hgasp::TimedComponent::CapacityRouteLocalSearch);
		this->s = individual;
		if (this->s == nullptr) {
			throw std::invalid_argument(
				"local search requires a non-null individual");
		}
		const bool hardTimeWindows = data->hasHardTimeWindows();
		if (hardTimeWindows) {
			hardTwLocalSearch();
		} else {
			capacityLocalSearch(finalizeCapacityObjective);
		}
		hgasp::recordHotPathCounter(
			hardTimeWindows
				? hgasp::HotPathCounter::LsHardTwCalls
				: hgasp::HotPathCounter::LsCapacityCalls);
		if (lastTwMetrics_.acceptedMoves == 0U) {
			hgasp::recordHotPathCounter(
				hgasp::HotPathCounter::LsZeroAcceptCalls);
		}
		hgasp::recordHotPathCounter(
			hgasp::HotPathCounter::LsCandidateMoves,
			static_cast<std::uint64_t>(lastTwMetrics_.candidateMoves));
		hgasp::recordHotPathCounter(
			hgasp::HotPathCounter::LsAcceptedMoves,
			static_cast<std::uint64_t>(lastTwMetrics_.acceptedMoves));
		static_assert(
			static_cast<std::size_t>(hgasp::HotPathCounter::LsAcceptedM1) ==
				static_cast<std::size_t>(
					hgasp::HotPathCounter::LsCandidateM1) + 12U,
			"LS operator diagnostic counters must remain contiguous");
		for (std::size_t index = 0U;
			 index < lastTwMetrics_.candidateByHgampMove.size();
			 ++index) {
			hgasp::recordHotPathCounter(
				static_cast<hgasp::HotPathCounter>(
					static_cast<std::size_t>(
						hgasp::HotPathCounter::LsCandidateM1) + index),
				static_cast<std::uint64_t>(
					lastTwMetrics_.candidateByHgampMove[index]));
			hgasp::recordHotPathCounter(
				static_cast<hgasp::HotPathCounter>(
					static_cast<std::size_t>(
						hgasp::HotPathCounter::LsAcceptedM1) + index),
				static_cast<std::uint64_t>(
					lastTwMetrics_.acceptedByHgampMove[index]));
		}
		hgasp::recordHotPathCounter(
			hgasp::HotPathCounter::LsFullSolutionEvaluations,
			static_cast<std::uint64_t>(
				lastTwMetrics_.fullRouteSetEvaluations));
		hgasp::recordHotPathCounter(
			hgasp::HotPathCounter::LsSummaryRebuilds,
			static_cast<std::uint64_t>(lastTwMetrics_.summaryRebuilds));
		hgasp::recordHotPathCounter(
			hgasp::HotPathCounter::LsCanonicalPairCacheQueries,
			static_cast<std::uint64_t>(
				lastTwMetrics_.canonicalPairCacheQueries));
		hgasp::recordHotPathCounter(
			hgasp::HotPathCounter::LsCanonicalPairCacheHits,
			static_cast<std::uint64_t>(
				lastTwMetrics_.canonicalPairCacheHits));
		hgasp::recordHotPathCounter(
			hgasp::HotPathCounter::LsDirectedPairFailureSkips,
			static_cast<std::uint64_t>(
				lastTwMetrics_.directedPairFailureSkips));
		hgasp::recordHotPathCounter(
			hgasp::HotPathCounter::LsDepotCandidateResolutionRequests,
			static_cast<std::uint64_t>(
				lastTwMetrics_.depotCandidateResolutionRequests));
		hgasp::recordHotPathCounter(
			hgasp::HotPathCounter::LsDepotCandidateResolutionBuilds,
			static_cast<std::uint64_t>(
				lastTwMetrics_.depotCandidateResolutionBuilds));
}

void LS::local_search_fixed_configuration(
		Individual* const individual,
		const std::vector<int>& openDepots,
		const double penaltyDepot,
		const double penaltyVehicle) {
	runFixedConfigurationLocalSearch(
		individual, openDepots, penaltyDepot, penaltyVehicle, true);
}

void LS::local_search_fixed_configuration_deferred_objective(
		Individual* const individual,
		const std::vector<int>& openDepots,
		const double penaltyDepot,
		const double penaltyVehicle) {
	if (data->hasHardTimeWindows() ||
		!hgasp::isSetSProductionSearchProfile(
			data->parameters.profile)) {
		throw std::logic_error(
			"deferred fixed LS objective refresh requires a role-aware Set S profile");
	}
	runFixedConfigurationLocalSearch(
		individual, openDepots, penaltyDepot, penaltyVehicle, false);
}

void LS::runFixedConfigurationLocalSearch(
		Individual* const individual,
		const std::vector<int>& openDepots,
		const double penaltyDepot,
		const double penaltyVehicle,
		const bool finalizeCapacityObjective) {
	if (fixedConfigurationMode_) {
		throw std::logic_error(
			"fixed-configuration local search cannot be nested");
	}
	if (individual == nullptr || openDepots.empty()) {
		throw std::invalid_argument(
			"fixed-configuration local search requires an individual and depot");
	}
	std::vector<bool> required(static_cast<std::size_t>(numL), false);
	for (const int depot : openDepots) {
		if (depot < 0 || depot >= numL ||
			required[static_cast<std::size_t>(depot)]) {
			throw std::invalid_argument(
				"fixed-configuration local search received an invalid depot set");
		}
		required[static_cast<std::size_t>(depot)] = true;
	}
	for (int depot = 0; depot < numL; ++depot) {
		if (individual->location[depot].used !=
			required[static_cast<std::size_t>(depot)]) {
			throw std::invalid_argument(
				"fixed-configuration local search input has a different depot set");
		}
	}
	fixedOpenDepots_ = std::move(required);
	fixedConfigurationMode_ = true;
	try {
		(void)penaltyDepot;
		(void)penaltyVehicle;
		runLocalSearch(individual, finalizeCapacityObjective);
	} catch (...) {
		fixedConfigurationMode_ = false;
		fixedOpenDepots_.clear();
		throw;
	}
	fixedConfigurationMode_ = false;
	for (int depot = 0; depot < numL; ++depot) {
		if (individual->location[depot].used !=
			fixedOpenDepots_[static_cast<std::size_t>(depot)]) {
			fixedOpenDepots_.clear();
			throw std::logic_error(
				"fixed-configuration local search changed its depot set");
		}
	}
	fixedOpenDepots_.clear();
}

void LS::capacityLocalSearch(const bool finalizeObjective) {
//	s->isRight();
	ensureLocalSearchCandidates();
	resetHgampMetrics();
	loadSolution();
	srun();
	if (finalizeObjective && lastTwMetrics_.acceptedMoves != 0U) {
		++lastTwMetrics_.fullRouteSetEvaluations;
		s->evaluationDis();
	}

//	s->outputSolution();



	double totalOpenCost = 0.0;
	const bool hasRemovableDepotCapacity =
		collectClrpPostSearchSummary(totalOpenCost);
	if (lastTwMetrics_.acceptedMoves == 0U) {
		// fit/dis remain authoritative when no topology changed. Preserve the
		// only side effect of evaluationDis() without walking every route.
		data->probFixedCost =
			std::abs(s->fit) > Min ? totalOpenCost / s->fit : 0.0;
	}
	if (hasRemovableDepotCapacity)
		data->loopp++;


	}

bool LS::collectClrpPostSearchSummary(
		double& totalOpenCost) const noexcept {
	int totalInventory = 0;
	int minimumUsedDepotInventory = std::numeric_limits<int>::max();
	bool hasUsedDepot = false;
	totalOpenCost = 0.0;
	for (int depot = 0; depot < numL; ++depot) {
		if (!s->location[depot].used) continue;
		hasUsedDepot = true;
		const int inventory = s->location[depot].inventory;
		totalInventory += inventory;
		minimumUsedDepotInventory = std::min(
			minimumUsedDepotInventory, inventory);
		totalOpenCost +=
			data->openCostLoc[s->location[depot].cour];
	}
	return hasUsedDepot &&
		totalInventory - minimumUsedDepotInventory > data->totalDemand;
}

void LS::hardTwLocalSearch() {
	ensureLocalSearchCandidates();
	resetHgampMetrics();

	std::vector<StandaloneRoute> incumbent =
		extractStandaloneRoutes(*s, *data);
	double incumbentObjective = 0.0;
	++lastTwMetrics_.fullRouteSetEvaluations;
	if (!evaluateTimeWindowRouteSet(
			*data, incumbent, incumbentObjective)) {
		throw std::runtime_error(
			"HGAMP hard-TW local search received an infeasible incumbent");
	}
	if (!data->equalTwoVar(incumbentObjective, s->fit)) {
		throw std::logic_error(
			"HGAMP hard-TW local-search incumbent objective mismatch");
	}

	RandomShuffle(orderNodes);
	const int candidateShuffleDenominator =
		data->parameters.localSearch.candidateShuffleDenominator;
	for (int customer = 0; customer < numV; ++customer) {
		if (RandomInt(0, candidateShuffleDenominator - 1) == 0) {
			RandomShuffle(localSearchCustomerCandidates_[customer]);
		}
	}

	RouteEvaluator evaluator(
		*data, data->problemContext().temporalConstraint());
	std::vector<int> sourceScratch;
	std::vector<int> destinationScratch;
	HgampMoveKind acceptedKind = HgampMoveKind::None;
	std::vector<int> twCustomerStateVersions(
		static_cast<std::size_t>(numV), -1);
	std::vector<int> twCustomerLastTestedVersions(
		static_cast<std::size_t>(numV), -1);
	std::vector<int> twDepotStateVersions(
		static_cast<std::size_t>(numL), -1);
	int twModificationVersion = 0;
	int twScanPass = 0;
	constexpr std::size_t kNoRoute =
		std::numeric_limits<std::size_t>::max();

	auto recordAccepted = [this](const HgampMoveKind kind) {
		recordHgampAccepted(kind);
	};

	std::vector<std::size_t> customerRoute(
		static_cast<std::size_t>(numV), kNoRoute);
	std::vector<std::size_t> customerPosition(
		static_cast<std::size_t>(numV), kNoRoute);
	std::vector<int> depotRouteCounts(
		static_cast<std::size_t>(numL), 0);
	std::vector<int> depotLoads(
		static_cast<std::size_t>(numL), 0);
	for (std::size_t routeIndex = 0U;
		 routeIndex < incumbent.size();
		 ++routeIndex) {
		const StandaloneRoute& route = incumbent[routeIndex];
		++depotRouteCounts[static_cast<std::size_t>(route.locId)];
		depotLoads[static_cast<std::size_t>(route.locId)] += route.totalLoad;
		for (std::size_t position = 0U;
			 position < route.sequence.size();
			 ++position) {
			const int customer = route.sequence[position];
			customerRoute[static_cast<std::size_t>(customer)] = routeIndex;
			customerPosition[static_cast<std::size_t>(customer)] = position;
		}
	}
	SequenceSummaryIndex sequenceSummaries =
		buildSequenceSummaries(
			*data,
			incumbent,
			customerRoute,
			promisingArcFilterEnabled() ? &promisingArcGraph_ : nullptr);
	TemporalSummaryIndex temporalSummaries =
		buildTemporalSummaries(*data, incumbent, customerRoute);
	lastTwMetrics_.summaryRebuilds = incumbent.size();
	enum class TwScanPhase {
		CustomerPairs,
		EmptyRoutes,
		SplitSuffix,
		SplitPrefix,
	};
	std::size_t twSourceCursor = 0U;
	std::size_t twCandidateCursor = 0U;
	TwScanPhase twScanPhase = TwScanPhase::CustomerPairs;
	bool twSourceStarted = false;
	bool twPassImproved = false;
	bool twAdvanceSourceAfterAcceptance = false;
	int twSourceLastTestedVersion = -1;
	std::vector<std::vector<unsigned char>> depotFirstArcs(
		static_cast<std::size_t>(numL),
		std::vector<unsigned char>(static_cast<std::size_t>(numV), 0U));
	auto rebuildDepotFirstArcs = [&]() {
		for (auto& row : depotFirstArcs) {
			std::fill(row.begin(), row.end(), 0U);
		}
		for (const StandaloneRoute& route : incumbent) {
			if (!route.sequence.empty()) {
				depotFirstArcs[static_cast<std::size_t>(route.locId)]
					[static_cast<std::size_t>(route.sequence.front())] = 1U;
			}
		}
	};
	rebuildDepotFirstArcs();
	auto isOldDirectedArc = [&](const int from, const int to) {
		if (from >= numV) {
			const int depot = from - numV;
			return depot >= 0 && depot < numL && to >= 0 && to < numV &&
				depotFirstArcs[static_cast<std::size_t>(depot)]
					[static_cast<std::size_t>(to)] != 0U;
		}
		if (from < 0 || from >= numV) return false;
		const std::size_t routeIndex =
			customerRoute[static_cast<std::size_t>(from)];
		const std::size_t position =
			customerPosition[static_cast<std::size_t>(from)];
		if (routeIndex >= incumbent.size() ||
			position >= incumbent[routeIndex].sequence.size()) return false;
		const StandaloneRoute& route = incumbent[routeIndex];
		const int oldTo = position + 1U < route.sequence.size()
			? route.sequence[position + 1U]
			: numV + route.locId;
		return oldTo == to;
	};
	auto compositionIntroducesPromisingArc =
		[&](const RouteComposition& composition) {
			if (!promisingArcFilterEnabled()) return true;
			if (compositionSize(composition) == 0U) return false;
			int previous = numV + composition.locId;
			for (std::size_t sliceIndex = 0U;
				 sliceIndex < composition.sliceCount;
				 ++sliceIndex) {
				const RouteSlice& slice = composition.slices[sliceIndex];
				const std::vector<int>& sequence = *slice.sequence;
				const int first = slice.reversed
					? sequence[slice.end - 1U]
					: sequence[slice.begin];
				if (!isOldDirectedArc(previous, first) &&
					isPromisingArc(previous, first)) return true;
				if (slice.reversed && slice.end - slice.begin > 1U) {
					const SequencePrefixSummary& summary =
						summaryFor(sequenceSummaries, slice.sequence);
					if (summary.reversePromisingPrefix[slice.end - 1U] -
						summary.reversePromisingPrefix[slice.begin] > 0) {
						return true;
					}
				}
				previous = slice.reversed
					? sequence[slice.begin]
					: sequence[slice.end - 1U];
			}
			const int depotNode = numV + composition.locId;
			return !isOldDirectedArc(previous, depotNode) &&
				isPromisingArc(previous, depotNode);
		};

	while (true) {
		auto tryCandidate = [this,
				&evaluator,
				&incumbent,
				&incumbentObjective,
				&depotRouteCounts,
				&depotLoads,
				&customerRoute,
				&customerPosition,
				&sequenceSummaries,
				&temporalSummaries,
				&sourceScratch,
				&destinationScratch,
				&acceptedKind,
				&twCustomerStateVersions,
				&twDepotStateVersions,
				&twModificationVersion,
				&compositionIntroducesPromisingArc,
				&rebuildDepotFirstArcs,
				kNoRoute](
				const std::size_t sourceIndex,
				const std::size_t destinationIndex,
				const RouteComposition& newSource,
				const std::optional<RouteComposition>& newDestination,
				const HgampMoveKind kind) {
			recordHgampCandidate(kind);
			const std::size_t metricIndex = hgampMoveMetricIndex(kind);
			if (sourceIndex >= incumbent.size() ||
				(destinationIndex != kNoRoute &&
				 destinationIndex >= incumbent.size()) ||
				(destinationIndex == sourceIndex)) {
				throw std::logic_error("invalid HGAMP move route index");
			}
			if (!compositionIntroducesPromisingArc(newSource) &&
				(!newDestination.has_value() ||
				 !compositionIntroducesPromisingArc(*newDestination))) {
				return false;
			}

			const int newSourceLoad =
				compositionLoad(newSource, sequenceSummaries);
			if (newSourceLoad > data->vehCap) {
				return false;
			}
			int newDestinationLoad = 0;
				if (newDestination.has_value()) {
				newDestinationLoad =
					compositionLoad(*newDestination, sequenceSummaries);
				if (newDestinationLoad > data->vehCap) {
					return false;
				}
			}

			std::array<int, 4U> affectedDepots{
				incumbent[sourceIndex].locId,
				newSource.locId,
				-1,
				-1};
			if (destinationIndex != kNoRoute) {
				affectedDepots[2] = incumbent[destinationIndex].locId;
			}
			if (newDestination.has_value()) {
				affectedDepots[3] = newDestination->locId;
			}
			std::sort(affectedDepots.begin(), affectedDepots.end());

			auto newDepotRouteCount = [&](const int depot) {
				int count = depotRouteCounts[static_cast<std::size_t>(depot)];
				if (incumbent[sourceIndex].locId == depot) --count;
				if (destinationIndex != kNoRoute &&
					incumbent[destinationIndex].locId == depot) --count;
				if (newSource.locId == depot &&
					compositionSize(newSource) != 0U) ++count;
				if (newDestination.has_value() &&
					newDestination->locId == depot &&
					compositionSize(*newDestination) != 0U) ++count;
				return count;
			};
			auto newDepotLoad = [&](const int depot) {
				int load = depotLoads[static_cast<std::size_t>(depot)];
				if (incumbent[sourceIndex].locId == depot) {
					load -= incumbent[sourceIndex].totalLoad;
				}
				if (destinationIndex != kNoRoute &&
					incumbent[destinationIndex].locId == depot) {
					load -= incumbent[destinationIndex].totalLoad;
				}
				if (newSource.locId == depot) load += newSourceLoad;
				if (newDestination.has_value() &&
					newDestination->locId == depot) {
					load += newDestinationLoad;
				}
				return load;
			};

			if (fixedConfigurationMode_) {
				int previousFixedDepot = -1;
				for (const int depot : affectedDepots) {
					if (depot < 0 || depot == previousFixedDepot) continue;
					previousFixedDepot = depot;
					if (!fixedConfigurationAllowsDepotState(
							&s->location[depot],
							newDepotRouteCount(depot) > 0)) {
						return false;
					}
				}
			}

			int previousDepot = -1;
			for (const int depot : affectedDepots) {
				if (depot < 0 || depot == previousDepot) continue;
				previousDepot = depot;
				const int routeCount = newDepotRouteCount(depot);
				if (routeCount < 0 ||
					routeCount > s->location[depot].maxVisit ||
					newDepotLoad(depot) >
						data->invLoc[data->numV + depot]) {
					return false;
				}
			}

			double predictedObjective = incumbentObjective -
				routeTravelObjectiveContribution(
					*data, incumbent[sourceIndex]);
			if (destinationIndex != kNoRoute) {
				predictedObjective -= routeTravelObjectiveContribution(
					*data, incumbent[destinationIndex]);
			}
			if (compositionSize(newSource) != 0U) {
				predictedObjective += data->usingCostVeh +
					compositionTravelCost(
						*data, newSource, sequenceSummaries);
			}
			if (newDestination.has_value() &&
				compositionSize(*newDestination) != 0U) {
				predictedObjective += data->usingCostVeh +
					compositionTravelCost(
						*data, *newDestination, sequenceSummaries);
			}
			previousDepot = -1;
			for (const int depot : affectedDepots) {
				if (depot < 0 || depot == previousDepot) continue;
				previousDepot = depot;
				const bool wasOpen =
					depotRouteCounts[static_cast<std::size_t>(depot)] > 0;
				const bool isOpen = newDepotRouteCount(depot) > 0;
				if (wasOpen != isOpen) {
					predictedObjective += isOpen
						? data->openCostLoc[data->numV + depot]
						: -data->openCostLoc[data->numV + depot];
				}
			}
			if (exactDeltaCannotImprove(
					predictedObjective, incumbentObjective)) {
				++lastTwMetrics_.exactDeltaRejectedMoves;
				return false;
			}
			if (!temporalCompositionFeasible(
					*data, newSource, temporalSummaries) ||
				(newDestination.has_value() &&
				 !temporalCompositionFeasible(
					 *data, *newDestination, temporalSummaries))) {
				++lastTwMetrics_.temporalSummaryRejectedMoves;
				++lastTwMetrics_.temporalRejectedByHgampMove[metricIndex];
				return false;
			}

			++lastTwMetrics_.evaluatorCheckedByHgampMove[metricIndex];
			materializeComposition(newSource, sourceScratch);
			materializeComposition(
				newDestination.value_or(RouteComposition{}),
				destinationScratch);
			RouteEvaluation sourceEvaluation;
			if (!sourceScratch.empty()) {
				++lastTwMetrics_.acceptedRouteEvaluations;
				sourceEvaluation = evaluator.evaluate(
					newSource.locId,
					sourceScratch,
					EvaluationDetail::Summary);
				++lastTwMetrics_.affectedRouteEvaluations;
				if (!sourceEvaluation.feasible()) return false;
			}
			RouteEvaluation destinationEvaluation;
			if (newDestination.has_value() &&
				!destinationScratch.empty()) {
				++lastTwMetrics_.acceptedRouteEvaluations;
				destinationEvaluation = evaluator.evaluate(
					newDestination->locId,
					destinationScratch,
					EvaluationDetail::Summary);
				++lastTwMetrics_.affectedRouteEvaluations;
				if (!destinationEvaluation.feasible()) return false;
			}

			double authoritativeObjective = incumbentObjective -
				routeTravelObjectiveContribution(
					*data, incumbent[sourceIndex]);
			if (destinationIndex != kNoRoute) {
				authoritativeObjective -= routeTravelObjectiveContribution(
					*data, incumbent[destinationIndex]);
			}
			if (!sourceScratch.empty()) {
				authoritativeObjective +=
					data->usingCostVeh + sourceEvaluation.travelCost;
			}
			if (newDestination.has_value() &&
				!destinationScratch.empty()) {
				authoritativeObjective +=
					data->usingCostVeh + destinationEvaluation.travelCost;
			}
			previousDepot = -1;
			for (const int depot : affectedDepots) {
				if (depot < 0 || depot == previousDepot) continue;
				previousDepot = depot;
				const bool wasOpen =
					depotRouteCounts[static_cast<std::size_t>(depot)] > 0;
				const bool isOpen = newDepotRouteCount(depot) > 0;
				if (wasOpen != isOpen) {
					authoritativeObjective += isOpen
						? data->openCostLoc[data->numV + depot]
						: -data->openCostLoc[data->numV + depot];
				}
			}
			if (!data->equalTwoVar(
					authoritativeObjective, predictedObjective)) {
				throw std::logic_error(
					"HGAMP hard-TW composition delta disagrees with "
					"authoritative affected-route objective");
			}
			if (exactDeltaCannotImprove(
					authoritativeObjective, incumbentObjective)) {
				++lastTwMetrics_.exactDeltaRejectedMoves;
				return false;
			}

			// RouteComposition stores non-owning slices into the current
			// incumbent. Mark the affected customer state before replacing either
			// route, otherwise these slices would be dangling after assignment.
			++twModificationVersion;
			visitComposition(
				newSource,
				[&twCustomerStateVersions, twModificationVersion](
						const int customer) {
					twCustomerStateVersions[
						static_cast<std::size_t>(customer)] =
						twModificationVersion;
				});
			if (newDestination.has_value()) {
				visitComposition(
					*newDestination,
					[&twCustomerStateVersions, twModificationVersion](
							const int customer) {
						twCustomerStateVersions[
							static_cast<std::size_t>(customer)] =
							twModificationVersion;
					});
				}
				for (const int customer : incumbent[sourceIndex].sequence) {
					customerRoute[static_cast<std::size_t>(customer)] = kNoRoute;
					customerPosition[static_cast<std::size_t>(customer)] = kNoRoute;
				}
				if (destinationIndex != kNoRoute) {
					for (const int customer :
						 incumbent[destinationIndex].sequence) {
						customerRoute[static_cast<std::size_t>(customer)] = kNoRoute;
						customerPosition[static_cast<std::size_t>(customer)] = kNoRoute;
					}
				}
				std::array<int, 4U> replacementRouteCounts{};
				std::array<int, 4U> replacementDepotLoads{};
				for (std::size_t index = 0U;
					 index < affectedDepots.size();
					 ++index) {
					const int depot = affectedDepots[index];
					if (depot >= 0 &&
						(index == 0U || depot != affectedDepots[index - 1U])) {
						replacementRouteCounts[index] =
							newDepotRouteCount(depot);
						replacementDepotLoads[index] = newDepotLoad(depot);
					}
				}

				StandaloneRoute acceptedSource;
			acceptedSource.locId = newSource.locId;
			if (!sourceScratch.empty()) {
				assignRouteEvaluation(
					acceptedSource,
					std::move(sourceScratch),
					sourceEvaluation);
			}
				incumbent[sourceIndex] = std::move(acceptedSource);
				std::size_t acceptedDestinationIndex = destinationIndex;
				if (newDestination.has_value()) {
				StandaloneRoute acceptedDestination;
				acceptedDestination.locId = newDestination->locId;
				if (!destinationScratch.empty()) {
					assignRouteEvaluation(
						acceptedDestination,
						std::move(destinationScratch),
						destinationEvaluation);
				}
					if (destinationIndex == kNoRoute) {
						acceptedDestinationIndex = incumbent.size();
						incumbent.push_back(
						std::move(acceptedDestination));
				} else {
					incumbent[destinationIndex] =
						std::move(acceptedDestination);
					}
				}
				auto indexRoute = [this,
						&incumbent,
						&customerRoute,
						&customerPosition](const std::size_t routeIndex) {
					const auto& sequence = incumbent[routeIndex].sequence;
					for (std::size_t position = 0U;
						 position < sequence.size();
						 ++position) {
						const int customer = sequence[position];
						if (customer < 0 || customer >= numV) {
							throw std::logic_error(
								"HGAMP accepted route contains invalid customer");
						}
						customerRoute[static_cast<std::size_t>(customer)] =
							routeIndex;
						customerPosition[static_cast<std::size_t>(customer)] =
							position;
					}
				};
				indexRoute(sourceIndex);
				if (newDestination.has_value()) {
					indexRoute(acceptedDestinationIndex);
				}
				sequenceSummaries.values[sourceIndex] =
					buildSequencePrefixSummary(
						*data,
						incumbent[sourceIndex].sequence,
						promisingArcFilterEnabled() ? &promisingArcGraph_ : nullptr);
				temporalSummaries.values[sourceIndex] =
					buildTemporalRouteSummary(
						*data, incumbent[sourceIndex].sequence);
				++lastTwMetrics_.summaryRebuilds;
				if (newDestination.has_value()) {
					SequencePrefixSummary destinationSequenceSummary =
						buildSequencePrefixSummary(
							*data,
							incumbent[acceptedDestinationIndex].sequence,
							promisingArcFilterEnabled()
								? &promisingArcGraph_
								: nullptr);
					TemporalRouteSummary destinationTemporalSummary =
						buildTemporalRouteSummary(
							*data,
							incumbent[acceptedDestinationIndex].sequence);
					if (acceptedDestinationIndex ==
							sequenceSummaries.values.size()) {
						sequenceSummaries.values.push_back(
							std::move(destinationSequenceSummary));
						temporalSummaries.values.push_back(
							std::move(destinationTemporalSummary));
					} else {
						sequenceSummaries.values[acceptedDestinationIndex] =
							std::move(destinationSequenceSummary);
						temporalSummaries.values[acceptedDestinationIndex] =
							std::move(destinationTemporalSummary);
					}
					++lastTwMetrics_.summaryRebuilds;
				}
				rebuildDepotFirstArcs();
				for (std::size_t index = 0U;
					 index < affectedDepots.size();
					 ++index) {
					const int depot = affectedDepots[index];
					if (depot >= 0 &&
						(index == 0U || depot != affectedDepots[index - 1U])) {
						depotRouteCounts[static_cast<std::size_t>(depot)] =
							replacementRouteCounts[index];
						depotLoads[static_cast<std::size_t>(depot)] =
							replacementDepotLoads[index];
						twDepotStateVersions[static_cast<std::size_t>(depot)] =
							twModificationVersion;
					}
				}
				incumbentObjective = authoritativeObjective;
			acceptedKind = kind;
			return true;
		};

		bool improved = false;
		while (twSourceCursor < orderNodes.size()) {
			const int u = orderNodes[twSourceCursor];
			const std::size_t sourceIndex =
				customerRoute[static_cast<std::size_t>(u)];
			const std::size_t uPosition =
				customerPosition[static_cast<std::size_t>(u)];
			if (sourceIndex == kNoRoute || uPosition == kNoRoute) {
				throw std::logic_error(
					"hard-TW LS customer is absent from incumbent");
			}
			if (!twSourceStarted) {
				twSourceLastTestedVersion =
					twCustomerLastTestedVersions[static_cast<std::size_t>(u)];
				twCustomerLastTestedVersions[static_cast<std::size_t>(u)] =
					twModificationVersion;
				twSourceStarted = true;
			}
			const int lastTestedVersion = twSourceLastTestedVersion;
			const StandaloneRoute& source = incumbent[sourceIndex];
			const auto& candidates =
				localSearchCustomerCandidates_[static_cast<std::size_t>(u)];
			const auto& depotCandidates =
				localSearchDepotCandidates_[static_cast<std::size_t>(u)];

			if (twScanPhase == TwScanPhase::CustomerPairs) {
			while (twCandidateCursor < candidates.size()) {
				const int vertex = candidates[twCandidateCursor++];
#if !defined(NDEBUG) || defined(HGASP_ENABLE_HOT_PATH_COUNTERS)
				++lastTwMetrics_.candidateVertices;
#endif
				const std::size_t destinationIndex =
					customerRoute[static_cast<std::size_t>(vertex)];
				const std::size_t vPosition =
					customerPosition[static_cast<std::size_t>(vertex)];
				const StandaloneRoute& destination =
					incumbent[destinationIndex];
				const bool sameRoute = sourceIndex == destinationIndex;
				const int dependencyVersion = std::max({
					twCustomerStateVersions[static_cast<std::size_t>(u)],
					twCustomerStateVersions[static_cast<std::size_t>(vertex)],
					twDepotStateVersions[static_cast<std::size_t>(source.locId)],
					twDepotStateVersions[
						static_cast<std::size_t>(destination.locId)]});
				if (twScanPass > 0 && dependencyVersion <= lastTestedVersion) {
					continue;
				}

				// M1: u is inserted after v.
				if (u != vertex) {
					RouteComposition newSource;
					newSource.locId = source.locId;
					if (sameRoute) {
						if (uPosition < vPosition) {
							appendSlice(newSource, source.sequence, 0U, uPosition);
							appendSlice(newSource, source.sequence, uPosition + 1U, vPosition + 1U);
							appendSlice(newSource, source.sequence, uPosition, uPosition + 1U);
							appendSlice(newSource, source.sequence, vPosition + 1U, source.sequence.size());
						} else if (vPosition + 1U != uPosition) {
							appendSlice(newSource, source.sequence, 0U, vPosition + 1U);
							appendSlice(newSource, source.sequence, uPosition, uPosition + 1U);
							appendSlice(newSource, source.sequence, vPosition + 1U, uPosition);
							appendSlice(newSource, source.sequence, uPosition + 1U, source.sequence.size());
						}
						if (newSource.sliceCount != 0U &&
							tryCandidate(sourceIndex, kNoRoute, newSource, std::nullopt, HgampMoveKind::M1)) {
							improved = true;
							break;
						}
					} else {
						appendSlice(newSource, source.sequence, 0U, uPosition);
						appendSlice(newSource, source.sequence, uPosition + 1U, source.sequence.size());
						RouteComposition newDestination;
						newDestination.locId = destination.locId;
						appendSlice(newDestination, destination.sequence, 0U, vPosition + 1U);
						appendSlice(newDestination, source.sequence, uPosition, uPosition + 1U);
						appendSlice(newDestination, destination.sequence, vPosition + 1U, destination.sequence.size());
						if (tryCandidate(sourceIndex, destinationIndex, newSource, newDestination, HgampMoveKind::M1)) {
							improved = true;
							break;
						}
					}
				}

				// M2/M3: move (u,x) after v, or reversed (x,u) before v.
				if (uPosition + 1U < source.sequence.size()) {
					const int x = source.sequence[uPosition + 1U];
					if (vertex != u && vertex != x) {
						RouteComposition reduced;
						reduced.locId = source.locId;
						appendSlice(reduced, source.sequence, 0U, uPosition);
						appendSlice(reduced, source.sequence, uPosition + 2U, source.sequence.size());
						if (!sameRoute) {
							RouteComposition after;
							after.locId = destination.locId;
							appendSlice(after, destination.sequence, 0U, vPosition + 1U);
							appendSlice(after, source.sequence, uPosition, uPosition + 2U);
							appendSlice(after, destination.sequence, vPosition + 1U, destination.sequence.size());
							if (tryCandidate(sourceIndex, destinationIndex, reduced, after, HgampMoveKind::M2)) {
								improved = true;
								break;
							}
							RouteComposition before;
							before.locId = destination.locId;
							appendSlice(before, destination.sequence, 0U, vPosition);
							appendSlice(before, source.sequence, uPosition, uPosition + 2U, true);
							appendSlice(before, destination.sequence, vPosition, destination.sequence.size());
							if (tryCandidate(sourceIndex, destinationIndex, reduced, before, HgampMoveKind::M3)) {
								improved = true;
								break;
							}
						} else {
							RouteComposition after;
							after.locId = source.locId;
							if (uPosition + 2U <= vPosition) {
								appendSlice(after, source.sequence, 0U, uPosition);
								appendSlice(after, source.sequence, uPosition + 2U, vPosition + 1U);
								appendSlice(after, source.sequence, uPosition, uPosition + 2U);
								appendSlice(after, source.sequence, vPosition + 1U, source.sequence.size());
							} else if (vPosition + 1U < uPosition) {
								appendSlice(after, source.sequence, 0U, vPosition + 1U);
								appendSlice(after, source.sequence, uPosition, uPosition + 2U);
								appendSlice(after, source.sequence, vPosition + 1U, uPosition);
								appendSlice(after, source.sequence, uPosition + 2U, source.sequence.size());
							}
							if (after.sliceCount != 0U &&
								tryCandidate(sourceIndex, kNoRoute, after, std::nullopt, HgampMoveKind::M2)) {
								improved = true;
								break;
							}
							RouteComposition before;
							before.locId = source.locId;
							if (uPosition + 2U <= vPosition) {
								appendSlice(before, source.sequence, 0U, uPosition);
								appendSlice(before, source.sequence, uPosition + 2U, vPosition);
								appendSlice(before, source.sequence, uPosition, uPosition + 2U, true);
								appendSlice(before, source.sequence, vPosition, source.sequence.size());
							} else if (vPosition < uPosition) {
								appendSlice(before, source.sequence, 0U, vPosition);
								appendSlice(before, source.sequence, uPosition, uPosition + 2U, true);
								appendSlice(before, source.sequence, vPosition, uPosition);
								appendSlice(before, source.sequence, uPosition + 2U, source.sequence.size());
							}
							if (before.sliceCount != 0U &&
								tryCandidate(sourceIndex, kNoRoute, before, std::nullopt, HgampMoveKind::M3)) {
								improved = true;
								break;
							}
						}
					}
				}

				// M4-M7 use x (the successor of u), exactly as defined in
				// HGAMP, rather than accidentally using u as in the reference code.
				if (uPosition + 1U < source.sequence.size()) {
					const std::size_t xPosition = uPosition + 1U;
					if (!(sameRoute && xPosition == vPosition)) {
						if (sameRoute) {
							RouteComposition exchanged = sameRouteSegmentExchange(source, xPosition, 1U, vPosition, 1U);
							if (tryCandidate(sourceIndex, kNoRoute, exchanged, std::nullopt, HgampMoveKind::M4)) {
								improved = true;
								break;
							}
						} else {
							RouteComposition newSource;
							newSource.locId = source.locId;
							appendSlice(newSource, source.sequence, 0U, xPosition);
							appendSlice(newSource, destination.sequence, vPosition, vPosition + 1U);
							appendSlice(newSource, source.sequence, xPosition + 1U, source.sequence.size());
							RouteComposition newDestination;
							newDestination.locId = destination.locId;
							appendSlice(newDestination, destination.sequence, 0U, vPosition);
							appendSlice(newDestination, source.sequence, xPosition, xPosition + 1U);
							appendSlice(newDestination, destination.sequence, vPosition + 1U, destination.sequence.size());
							if (tryCandidate(sourceIndex, destinationIndex, newSource, newDestination, HgampMoveKind::M4)) {
								improved = true;
								break;
							}
						}
					}

					if (xPosition + 1U < source.sequence.size() &&
						!(sameRoute && vPosition >= xPosition && vPosition <= xPosition + 1U)) {
						if (sameRoute) {
							RouteComposition exchanged = sameRouteSegmentExchange(source, xPosition, 2U, vPosition, 1U);
							if (tryCandidate(sourceIndex, kNoRoute, exchanged, std::nullopt, HgampMoveKind::M5)) {
								improved = true;
								break;
							}
						} else {
							RouteComposition newSource;
							newSource.locId = source.locId;
							appendSlice(newSource, source.sequence, 0U, xPosition);
							appendSlice(newSource, destination.sequence, vPosition, vPosition + 1U);
							appendSlice(newSource, source.sequence, xPosition + 2U, source.sequence.size());
							RouteComposition newDestination;
							newDestination.locId = destination.locId;
							appendSlice(newDestination, destination.sequence, 0U, vPosition);
							appendSlice(newDestination, source.sequence, xPosition, xPosition + 2U);
							appendSlice(newDestination, destination.sequence, vPosition + 1U, destination.sequence.size());
							if (tryCandidate(sourceIndex, destinationIndex, newSource, newDestination, HgampMoveKind::M5)) {
								improved = true;
								break;
							}
						}
					}

					if (xPosition + 1U < source.sequence.size() &&
						vPosition + 1U < destination.sequence.size() &&
						!(sameRoute &&
						  !(xPosition + 2U <= vPosition || vPosition + 2U <= xPosition))) {
						if (sameRoute) {
							RouteComposition exchanged = sameRouteSegmentExchange(source, xPosition, 2U, vPosition, 2U);
							if (tryCandidate(sourceIndex, kNoRoute, exchanged, std::nullopt, HgampMoveKind::M6)) {
								improved = true;
								break;
							}
							RouteComposition reversed = sameRouteSegmentExchange(source, xPosition, 2U, vPosition, 2U, true, false);
							if (tryCandidate(sourceIndex, kNoRoute, reversed, std::nullopt, HgampMoveKind::M7)) {
								improved = true;
								break;
							}
						} else {
							for (const bool reverseX : {false, true}) {
								RouteComposition newSource;
								newSource.locId = source.locId;
								appendSlice(newSource, source.sequence, 0U, xPosition);
								appendSlice(newSource, destination.sequence, vPosition, vPosition + 2U);
								appendSlice(newSource, source.sequence, xPosition + 2U, source.sequence.size());
								RouteComposition newDestination;
								newDestination.locId = destination.locId;
								appendSlice(newDestination, destination.sequence, 0U, vPosition);
								appendSlice(newDestination, source.sequence, xPosition, xPosition + 2U, reverseX);
								appendSlice(newDestination, destination.sequence, vPosition + 2U, destination.sequence.size());
								const HgampMoveKind kind = reverseX ? HgampMoveKind::M7 : HgampMoveKind::M6;
								if (tryCandidate(sourceIndex, destinationIndex, newSource, newDestination, kind)) {
									improved = true;
									break;
								}
							}
							if (improved) break;
						}
					}
				}

				// M8: same-route 2-opt replaces (u,x),(v,y) by (u,v),(x,y).
				if (sameRoute && uPosition + 1U < vPosition) {
					RouteComposition twoOpt;
					twoOpt.locId = source.locId;
					appendSlice(twoOpt, source.sequence, 0U, uPosition + 1U);
					appendSlice(twoOpt, source.sequence, uPosition + 1U, vPosition + 1U, true);
					appendSlice(twoOpt, source.sequence, vPosition + 1U, source.sequence.size());
					if (tryCandidate(sourceIndex, kNoRoute, twoOpt, std::nullopt, HgampMoveKind::M8)) {
						improved = true;
						break;
					}
				}

				if (!sameRoute) {
					// M9 exchanges (x,u_l) and (v,v_l).
					RouteComposition m9Source;
					m9Source.locId = source.locId;
					appendSlice(m9Source, source.sequence, 0U, uPosition + 1U);
					appendSlice(m9Source, destination.sequence, vPosition, destination.sequence.size());
					RouteComposition m9Destination;
					m9Destination.locId = destination.locId;
					appendSlice(m9Destination, destination.sequence, 0U, vPosition);
					appendSlice(m9Destination, source.sequence, uPosition + 1U, source.sequence.size());
					if (tryCandidate(sourceIndex, destinationIndex, m9Source, m9Destination, HgampMoveKind::M9)) {
						improved = true;
						break;
					}

					// M10 exchanges (x,u_l) and (v_b,v). The transferred
					// substrings are reversed to preserve depot-to-depot route
					// orientation, matching the reference topology without its
					// case-specific pointer bugs.
					RouteComposition m10Source;
					m10Source.locId = source.locId;
					appendSlice(m10Source, source.sequence, 0U, uPosition + 1U);
					appendSlice(m10Source, destination.sequence, 0U, vPosition + 1U, true);
					RouteComposition m10Destination;
					m10Destination.locId = destination.locId;
					appendSlice(m10Destination, source.sequence, uPosition + 1U, source.sequence.size(), true);
					appendSlice(m10Destination, destination.sequence, vPosition + 1U, destination.sequence.size());
					if (tryCandidate(sourceIndex, destinationIndex, m10Source, m10Destination, HgampMoveKind::M10)) {
						improved = true;
						break;
					}
				}

				// Author-source depot anchor: when v is the first customer, also
				// try M1/M2 immediately after that route's depot. M3-before-v is
				// already covered above; the apparent source M9/M10 depot calls are
				// dead and intentionally are not copied.
				if (vPosition == 0U && u != vertex) {
					RouteComposition anchorSource;
					anchorSource.locId = source.locId;
					RouteComposition anchorDestination;
					anchorDestination.locId = destination.locId;
					if (sameRoute) {
						if (uPosition > 0U) {
							appendSlice(anchorSource, source.sequence, uPosition, uPosition + 1U);
							appendSlice(anchorSource, source.sequence, 0U, uPosition);
							appendSlice(anchorSource, source.sequence, uPosition + 1U, source.sequence.size());
							if (tryCandidate(sourceIndex, kNoRoute, anchorSource, std::nullopt, HgampMoveKind::M1)) {
								improved = true;
								break;
							}
						}
					} else {
						appendSlice(anchorSource, source.sequence, 0U, uPosition);
						appendSlice(anchorSource, source.sequence, uPosition + 1U, source.sequence.size());
						appendSlice(anchorDestination, source.sequence, uPosition, uPosition + 1U);
						appendSlice(anchorDestination, destination.sequence, 0U, destination.sequence.size());
						if (tryCandidate(sourceIndex, destinationIndex, anchorSource, anchorDestination, HgampMoveKind::M1)) {
							improved = true;
							break;
						}
					}

					if (uPosition + 1U < source.sequence.size()) {
						RouteComposition anchorTwoSource;
						anchorTwoSource.locId = source.locId;
						RouteComposition anchorTwoDestination;
						anchorTwoDestination.locId = destination.locId;
						if (sameRoute) {
							if (uPosition > 0U) {
								appendSlice(anchorTwoSource, source.sequence, uPosition, uPosition + 2U);
								appendSlice(anchorTwoSource, source.sequence, 0U, uPosition);
								appendSlice(anchorTwoSource, source.sequence, uPosition + 2U, source.sequence.size());
								if (tryCandidate(sourceIndex, kNoRoute, anchorTwoSource, std::nullopt, HgampMoveKind::M2)) {
									improved = true;
									break;
								}
							}
						} else {
							appendSlice(anchorTwoSource, source.sequence, 0U, uPosition);
							appendSlice(anchorTwoSource, source.sequence, uPosition + 2U, source.sequence.size());
							appendSlice(anchorTwoDestination, source.sequence, uPosition, uPosition + 2U);
							appendSlice(anchorTwoDestination, destination.sequence, 0U, destination.sequence.size());
							if (tryCandidate(sourceIndex, destinationIndex, anchorTwoSource, anchorTwoDestination, HgampMoveKind::M2)) {
								improved = true;
								break;
							}
						}
					}
					}
				}
				if (improved) break;
				twScanPhase = TwScanPhase::EmptyRoutes;
				twCandidateCursor = 0U;
			}

				// Empty-route M1-M3 use the independently bounded depot-beta list;
			// this phase is separate from customer-alpha and allocates no merged
			// candidate vector on the hot path.
				if (twScanPhase == TwScanPhase::EmptyRoutes) {
				while (twCandidateCursor < depotCandidates.size()) {
					const int depotNode = depotCandidates[twCandidateCursor++];
#if !defined(NDEBUG) || defined(HGASP_ENABLE_HOT_PATH_COUNTERS)
				++lastTwMetrics_.candidateVertices;
#endif
				const int targetDepot = depotNode - numV;
				RouteComposition reduced;
				reduced.locId = source.locId;
				appendSlice(reduced, source.sequence, 0U, uPosition);
				appendSlice(
					reduced,
					source.sequence,
					uPosition + 1U,
					source.sequence.size());
				RouteComposition inserted;
				inserted.locId = targetDepot;
				appendSlice(
					inserted,
					source.sequence,
					uPosition,
					uPosition + 1U);
				if (tryCandidate(
						sourceIndex,
						kNoRoute,
						reduced,
						inserted,
						HgampMoveKind::M1)) {
					improved = true;
					break;
				}

				if (uPosition + 1U >= source.sequence.size()) continue;
				RouteComposition reducedTwo;
				reducedTwo.locId = source.locId;
				appendSlice(reducedTwo, source.sequence, 0U, uPosition);
				appendSlice(
					reducedTwo,
					source.sequence,
					uPosition + 2U,
					source.sequence.size());
				RouteComposition forward;
				forward.locId = targetDepot;
				appendSlice(
					forward,
					source.sequence,
					uPosition,
					uPosition + 2U);
				if (tryCandidate(
						sourceIndex,
						kNoRoute,
						reducedTwo,
						forward,
						HgampMoveKind::M2)) {
					improved = true;
					break;
				}
				RouteComposition reversed;
				reversed.locId = targetDepot;
				appendSlice(
					reversed,
					source.sequence,
					uPosition,
					uPosition + 2U,
					true);
				if (tryCandidate(
						sourceIndex,
						kNoRoute,
						reducedTwo,
						reversed,
						HgampMoveKind::M3)) {
					improved = true;
					break;
					}
				}
				if (improved) break;
				twScanPhase = TwScanPhase::SplitSuffix;
				twCandidateCursor = 0U;
			}

			// Split uses the independent bounded depot list. Preserve the author
			// source order across the complete list: all Splitb (suffix) candidates
			// precede every Splitf (prefix) candidate.
				if (twScanPhase == TwScanPhase::SplitSuffix) {
				while (twCandidateCursor < depotCandidates.size()) {
					const int vertex = depotCandidates[twCandidateCursor++];
				if (vertex - numV == source.locId) continue;
#if !defined(NDEBUG) || defined(HGASP_ENABLE_HOT_PATH_COUNTERS)
				++lastTwMetrics_.candidateVertices;
#endif
				const int targetDepot = vertex - numV;
				RouteComposition suffixRemainder;
				suffixRemainder.locId = source.locId;
				appendSlice(suffixRemainder, source.sequence, 0U, uPosition);
				RouteComposition suffix;
				suffix.locId = targetDepot;
				appendSlice(suffix, source.sequence, uPosition, source.sequence.size());
					if (tryCandidate(sourceIndex, kNoRoute, suffixRemainder, suffix, HgampMoveKind::SplitSuffix)) {
						improved = true;
						twAdvanceSourceAfterAcceptance = true;
						break;
					}
				}
				if (improved) break;
				twScanPhase = TwScanPhase::SplitPrefix;
				twCandidateCursor = 0U;
				}
				if (twScanPhase == TwScanPhase::SplitPrefix) {
				while (twCandidateCursor < depotCandidates.size()) {
					const int vertex = depotCandidates[twCandidateCursor++];
				if (vertex - numV == source.locId) continue;
#if !defined(NDEBUG) || defined(HGASP_ENABLE_HOT_PATH_COUNTERS)
				++lastTwMetrics_.candidateVertices;
#endif
				const int targetDepot = vertex - numV;
				RouteComposition prefixRemainder;
				prefixRemainder.locId = source.locId;
				appendSlice(prefixRemainder, source.sequence, uPosition + 1U, source.sequence.size());
				RouteComposition prefix;
				prefix.locId = targetDepot;
				appendSlice(prefix, source.sequence, 0U, uPosition + 1U);
					if (tryCandidate(sourceIndex, kNoRoute, prefixRemainder, prefix, HgampMoveKind::SplitPrefix)) {
						improved = true;
						twAdvanceSourceAfterAcceptance = true;
						break;
					}
				}
				if (improved) break;
				}
				++twSourceCursor;
				twCandidateCursor = 0U;
				twScanPhase = TwScanPhase::CustomerPairs;
				twSourceStarted = false;
			}

			if (!improved) {
				if (twSourceCursor == orderNodes.size() && twPassImproved) {
					twSourceCursor = 0U;
					twCandidateCursor = 0U;
					twScanPhase = TwScanPhase::CustomerPairs;
					twSourceStarted = false;
					twPassImproved = false;
					++twScanPass;
					continue;
				}
				break;
		}
		#ifndef NDEBUG
		// The production hot path relies on the affected-route evaluator and
		// incremental coverage/load indices above. Debug/test builds retain an
		// independent whole-solution oracle after every accepted move.
		std::vector<StandaloneRoute> validationRoutes;
		validationRoutes.reserve(incumbent.size());
		for (const StandaloneRoute& route : incumbent) {
			if (!route.sequence.empty()) validationRoutes.push_back(route);
		}
		double validatedObjective = 0.0;
		++lastTwMetrics_.fullRouteSetEvaluations;
		if (!evaluateTimeWindowRouteSet(
				*data, validationRoutes, validatedObjective) ||
			!data->equalTwoVar(validatedObjective, incumbentObjective)) {
			throw std::logic_error(
				"HGAMP hard-TW accepted move failed full validation");
		}
		incumbentObjective = validatedObjective;
		#endif
			recordAccepted(acceptedKind);
			twPassImproved = true;
			if (twAdvanceSourceAfterAcceptance) {
				++twSourceCursor;
				twCandidateCursor = 0U;
				twScanPhase = TwScanPhase::CustomerPairs;
				twSourceStarted = false;
				twAdvanceSourceAfterAcceptance = false;
			}
			if (twSourceCursor == orderNodes.size()) {
				twSourceCursor = 0U;
				twCandidateCursor = 0U;
				twScanPhase = TwScanPhase::CustomerPairs;
				twSourceStarted = false;
				twPassImproved = false;
				++twScanPass;
			}
		}

	if (lastTwMetrics_.acceptedMoves == 0U) {
		return;
	}
	incumbent.erase(
		std::remove_if(
			incumbent.begin(),
			incumbent.end(),
			[](const StandaloneRoute& route) {
				return route.sequence.empty();
			}),
		incumbent.end());
	double boundaryObjective = 0.0;
	++lastTwMetrics_.fullRouteSetEvaluations;
	if (!evaluateTimeWindowRouteSet(
			*data, incumbent, boundaryObjective) ||
		!data->equalTwoVar(boundaryObjective, incumbentObjective)) {
		throw std::logic_error(
			"HGAMP hard-TW local-search boundary validation failed");
	}
	incumbentObjective = boundaryObjective;
	++lastTwMetrics_.rebuilds;
	s->rebuildFromSP(incumbent);
	s->evaluationDis();
	s->decideFeasible();
	if (!s->isFeasible) {
		throw std::logic_error(
			"HGAMP hard-TW local-search commit failed validation");
	}
	s->isRight();
}
