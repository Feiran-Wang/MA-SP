/******************************************************************************
 * File: FeasibleInsertionFinder.cpp
 * Description:
 *   Exhaustive candidate enumeration for hard-time-window insertion.
 ******************************************************************************/

#include "FeasibleInsertionFinder.h"

#include "ComponentProfiler.h"
#include "Individual.h"
#include "RouteEvaluator.h"
#include "basic.h"
#include "SolverContext.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace hgasp {
namespace {

struct Candidate {
	bool opensRoute = false;
	int routeIndex = -1;
	int depotIndex = -1;
	std::size_t insertionPosition = 0;
	double objectiveDelta = 0.0;
	RouteEvaluation evaluation;
	std::vector<int> sequence;
};

constexpr double kCandidateDeltaEpsilon = 1.0e-12;

bool candidateLess(const Candidate& lhs, const Candidate& rhs) {
	if (std::fabs(lhs.objectiveDelta - rhs.objectiveDelta) >
		kCandidateDeltaEpsilon) {
		return lhs.objectiveDelta < rhs.objectiveDelta;
	}
	if (lhs.depotIndex != rhs.depotIndex) {
		return lhs.depotIndex < rhs.depotIndex;
	}
	if (lhs.opensRoute != rhs.opensRoute) {
		// Prefer extending a route when objective deltas are identical.
		return !lhs.opensRoute;
	}
	if (lhs.routeIndex != rhs.routeIndex) {
		return lhs.routeIndex < rhs.routeIndex;
	}
	return lhs.insertionPosition < rhs.insertionPosition;
}

void materializeInsertion(
		const std::vector<int>& base,
		const std::vector<int>& unit,
		const std::size_t position,
		std::vector<int>& sequence) {
	if (position > base.size()) {
		throw std::logic_error(
			"reconstruction insertion position exceeds its route");
	}
	sequence.clear();
	sequence.reserve(base.size() + unit.size());
	sequence.insert(
		sequence.end(), base.begin(),
		base.begin() + static_cast<std::ptrdiff_t>(position));
	sequence.insert(sequence.end(), unit.begin(), unit.end());
	sequence.insert(
		sequence.end(),
		base.begin() + static_cast<std::ptrdiff_t>(position),
		base.end());
}

RouteEvaluation evaluateFinalizedCapacityInsertion(
		const SolverContext& data,
		const int depot,
		const std::vector<int>& base,
		const std::vector<int>& unit,
		const std::size_t position,
		const int load) {
	RouteEvaluation evaluation;
	evaluation.customerCount = base.size() + unit.size();
	evaluation.load = load;
	evaluation.capacityFeasible = load <= data.vehCap;
	const int depotNode = data.numV + depot;
	int previous = depotNode;
	auto appendCustomer = [&](const int customer) {
		evaluation.travelCost += data.D[previous][customer];
		previous = customer;
	};
	for (std::size_t index = 0U; index <= base.size(); ++index) {
		if (index == position) {
			for (const int customer : unit) {
				appendCustomer(customer);
			}
		}
		if (index < base.size()) {
			appendCustomer(base[index]);
		}
	}
	evaluation.travelCost += data.D[previous][depotNode];
	return evaluation;
}

std::size_t selectCandidateIndex(
	std::size_t candidateCount,
	int rankPop,
	std::size_t maximumWidth,
	double rankExponent);

std::size_t selectedCandidatePosition(
		const std::vector<Candidate>& candidates,
		const int rankPop,
		const std::size_t maximumWidth,
		const double rankExponent,
		std::size_t& sortedCandidates) {
	if (candidates.empty()) {
		return 0U;
	}
	const std::size_t selectedRank =
		selectCandidateIndex(
			candidates.size(), rankPop, maximumWidth, rankExponent);
	const std::size_t retained =
		std::min(maximumWidth, candidates.size());
	std::vector<std::size_t> order(candidates.size());
	std::iota(order.begin(), order.end(), 0U);
	const auto indexLess = [&candidates](
			const std::size_t lhs,
			const std::size_t rhs) {
		return candidateLess(candidates[lhs], candidates[rhs]);
	};

	// The V1.16.1 comparator deliberately treats objective differences up to
	// 1e-12 as ties.  It is a strict weak ordering for the discrete Set S
	// objective deltas (distinct values differ by at least one), but not for an
	// arbitrary chain of floating-point values.  Never expose the new bounded
	// partial sort to that unproven case: non-discrete data keeps the legacy
	// full-sort ordering path.
	const bool hasDiscreteObjectiveDeltas = std::all_of(
		candidates.begin(), candidates.end(), [](const Candidate& candidate) {
			return std::isfinite(candidate.objectiveDelta) &&
				std::trunc(candidate.objectiveDelta) ==
					candidate.objectiveDelta;
		});
	if (!hasDiscreteObjectiveDeltas) {
		std::sort(order.begin(), order.end(), indexLess);
		sortedCandidates = candidates.size();
	} else {
		std::partial_sort(
			order.begin(),
			order.begin() + static_cast<std::ptrdiff_t>(retained),
			order.end(),
			indexLess);
		sortedCandidates = retained;
	}
	return order[selectedRank];
}

std::size_t selectCandidateIndex(
	const std::size_t candidateCount,
	const int rankPop,
	const std::size_t maximumWidth,
	const double rankExponent) {
	if (candidateCount == 0U || rankPop < 0) {
		return 0U;
	}

	// A bounded RCL gives population diversity without turning construction
	// into an unconstrained random assignment.  rankPop=0 remains the exact
	// cheapest-insertion policy and therefore provides a deterministic anchor.
	const std::size_t width = std::min(
		candidateCount,
		1U + static_cast<std::size_t>(rankPop) % maximumWidth);
	if (width == 1U) {
		return 0U;
	}

	const double biased =
		std::pow(RandomDouble(0.0, 1.0), rankExponent);
	return std::min(
		width - 1U,
		static_cast<std::size_t>(
			biased * static_cast<double>(width)));
}

void validatePermutation(
	const SolverContext& data,
	const std::vector<int>& customerOrder) {
	if (customerOrder.size() !=
		static_cast<std::size_t>(data.numV)) {
		throw std::invalid_argument(
			"TW feasible insertion requires a complete customer "
			"permutation");
	}

	std::vector<unsigned char> seen(
		static_cast<std::size_t>(data.numV), 0U);
	for (const int customer : customerOrder) {
		if (customer < 0 || customer >= data.numV) {
			throw std::out_of_range(
				"TW feasible insertion customer index is out of range");
		}
		if (seen[static_cast<std::size_t>(customer)] != 0U) {
			throw std::invalid_argument(
				"TW feasible insertion permutation contains a duplicate "
				"customer");
		}
		seen[static_cast<std::size_t>(customer)] = 1U;
	}
}

void validateAllowedDepots(
		const SolverContext& data,
		const std::vector<bool>& allowedDepots) {
	if (allowedDepots.empty()) {
		return;
	}
	if (allowedDepots.size() !=
		static_cast<std::size_t>(data.numL)) {
		throw std::invalid_argument(
			"TW allowed-depot mask has the wrong size");
	}
	if (std::none_of(
			allowedDepots.begin(),
			allowedDepots.end(),
			[](const bool allowed) { return allowed; })) {
		throw std::invalid_argument(
			"TW allowed-depot mask cannot exclude every depot");
	}
}

bool depotAllowed(
		const std::vector<bool>& allowedDepots,
		const int depot) noexcept {
	return allowedDepots.empty() ||
		allowedDepots[static_cast<std::size_t>(depot)];
}

std::string describeFailure(
	const SolverContext& data,
	const int customer,
	const std::size_t insertedCount,
	const RouteEvaluator& evaluator,
	const std::vector<bool>& allowedDepots) {
	std::ostringstream message;
	message << "no feasible insertion for customer "
			<< data.originalCustomerIds.at(
				   static_cast<std::size_t>(customer))
			<< " (internal index " << customer << ") after "
			<< insertedCount << " customers; demand="
			<< data.demCus[customer] << ", TW=["
			<< data.readyTime.at(static_cast<std::size_t>(customer))
			<< ","
			<< data.dueTime.at(static_cast<std::size_t>(customer))
			<< "]. Singleton-feasible depots:";

	bool anySingleton = false;
	for (int depot = 0; depot < data.numL; ++depot) {
		if (!depotAllowed(allowedDepots, depot)) {
			continue;
		}
		if (data.demCus[customer] >
			data.invLoc[data.numV + depot]) {
			continue;
		}
		const RouteEvaluation singleton =
			evaluator.evaluate(depot, std::vector<int>{customer});
		recordHotPathCounter(
			HotPathCounter::ReconstructionRouteEvaluations);
		if (singleton.feasible()) {
			anySingleton = true;
			message << " "
					<< data.originalDepotIds.at(
						   static_cast<std::size_t>(depot));
		}
	}
	if (!anySingleton) {
		message << " none";
	}
	if (!allowedDepots.empty()) {
		message << " (restricted depot set)";
	}
	return message.str();
}

void validateCompletePlan(
	const SolverContext& data,
	const std::vector<StandaloneRoute>& routes,
	const RouteEvaluator& evaluator,
	const std::vector<bool>& allowedDepots) {
	std::vector<int> coverage(
		static_cast<std::size_t>(data.numV), 0);
	std::vector<int> depotLoad(
		static_cast<std::size_t>(data.numL), 0);
	std::vector<int> depotRoutes(
		static_cast<std::size_t>(data.numL), 0);

	for (const StandaloneRoute& route : routes) {
		if (route.locId < 0 || route.locId >= data.numL ||
			route.sequence.empty()) {
			throw std::logic_error(
				"TW feasible insertion produced an invalid route shell");
		}
		if (!depotAllowed(allowedDepots, route.locId)) {
			throw std::logic_error(
				"TW feasible insertion used a depot excluded by its "
				"heuristic mask");
		}
		const RouteEvaluation evaluation =
			evaluator.evaluate(route.locId, route.sequence);
		recordHotPathCounter(
			HotPathCounter::ReconstructionRouteEvaluations);
		if (!evaluation.feasible() ||
			evaluation.load != route.totalLoad ||
			std::fabs(
				evaluation.travelCost - route.routeCost) >= Min) {
			throw std::logic_error(
				"TW feasible insertion produced a route that failed "
				"independent validation");
		}

		depotLoad[static_cast<std::size_t>(route.locId)] +=
			evaluation.load;
		++depotRoutes[static_cast<std::size_t>(route.locId)];
		for (const int customer : route.sequence) {
			if (++coverage[static_cast<std::size_t>(customer)] != 1) {
				throw std::logic_error(
					"TW feasible insertion produced duplicate coverage");
			}
		}
	}

	for (int customer = 0; customer < data.numV; ++customer) {
		if (coverage[static_cast<std::size_t>(customer)] != 1) {
			throw std::logic_error(
				"TW feasible insertion did not cover every customer");
		}
	}
	for (int depot = 0; depot < data.numL; ++depot) {
		if (depotLoad[static_cast<std::size_t>(depot)] >
			data.invLoc[data.numV + depot]) {
			throw std::logic_error(
				"TW feasible insertion exceeded depot capacity");
		}
		if (depotRoutes[static_cast<std::size_t>(depot)] >
			Individual::representationRouteLimit(data, depot)) {
			throw std::logic_error(
				"TW feasible insertion exceeded the route representation "
				"limit");
		}
	}
}

}  // namespace

FeasibleInsertionFinder::FeasibleInsertionFinder(
		const SolverContext& data,
		const FeasibleInsertionExecution execution)
		: data_(data), execution_(execution) {
}

bool FeasibleInsertionFinder::build(
	const std::vector<int>& customerOrder,
	const int rankPop,
	FeasibleInsertionResult& result,
	std::string* const diagnostic) const {
	return build(
		customerOrder,
		rankPop,
		std::vector<std::vector<int>>{},
		std::vector<bool>{},
		result,
		diagnostic);
}

bool FeasibleInsertionFinder::build(
	const std::vector<int>& customerOrder,
	const int rankPop,
	const std::vector<bool>& allowedDepots,
	FeasibleInsertionResult& result,
	std::string* const diagnostic) const {
	return build(
		customerOrder,
		rankPop,
		std::vector<std::vector<int>>{},
		allowedDepots,
		result,
		diagnostic);
}

bool FeasibleInsertionFinder::build(
	const std::vector<int>& customerOrder,
	const int rankPop,
	const std::vector<std::vector<int>>& trustedBlocks,
	FeasibleInsertionResult& result,
	std::string* const diagnostic) const {
	return build(
		customerOrder,
		rankPop,
		trustedBlocks,
		std::vector<bool>{},
		result,
		diagnostic);
}

bool FeasibleInsertionFinder::build(
	const std::vector<int>& customerOrder,
	const int rankPop,
	const std::vector<std::vector<int>>& trustedBlocks,
	const std::vector<bool>& allowedDepots,
	FeasibleInsertionResult& result,
	std::string* const diagnostic) const {
	recordHotPathCounter(HotPathCounter::ReconstructionBuilds);
	validatePermutation(data_, customerOrder);
	validateAllowedDepots(data_, allowedDepots);
	std::vector<std::vector<int>> insertionUnits;
	std::vector<int> blockAt(
		static_cast<std::size_t>(data_.numV), -1);
	std::vector<unsigned char> blockMember(
		static_cast<std::size_t>(data_.numV), 0U);
	for (std::size_t blockIndex = 0;
		 blockIndex < trustedBlocks.size();
		 ++blockIndex) {
		const std::vector<int>& block = trustedBlocks[blockIndex];
		if (block.size() < 2U || block.size() > 5U) {
			throw std::invalid_argument(
				"TW trusted blocks must contain 2 to 5 customers");
		}
		for (const int customer : block) {
			if (customer < 0 || customer >= data_.numV ||
				blockMember[static_cast<std::size_t>(customer)] != 0U) {
				throw std::invalid_argument(
					"TW trusted blocks overlap or contain an invalid "
					"customer");
			}
			blockMember[static_cast<std::size_t>(customer)] = 1U;
		}
		blockAt[static_cast<std::size_t>(block.front())] =
			static_cast<int>(blockIndex);
	}
	for (std::size_t orderIndex = 0U;
		 orderIndex < customerOrder.size();) {
		const int customer = customerOrder[orderIndex];
		const int blockIndex =
			blockAt[static_cast<std::size_t>(customer)];
		if (blockIndex < 0) {
			if (blockMember[static_cast<std::size_t>(customer)] != 0U) {
				throw std::invalid_argument(
					"TW trusted block is not contiguous in customer "
					"order");
			}
			insertionUnits.push_back({customer});
			++orderIndex;
			continue;
		}
		const std::vector<int>& block =
			trustedBlocks[static_cast<std::size_t>(blockIndex)];
		if (orderIndex + block.size() > customerOrder.size() ||
			!std::equal(
				block.begin(),
				block.end(),
				customerOrder.begin() +
					static_cast<std::ptrdiff_t>(orderIndex))) {
			throw std::invalid_argument(
				"TW trusted block is not contiguous in customer order");
		}
		insertionUnits.push_back(block);
		orderIndex += block.size();
	}
	recordHotPathCounter(
		HotPathCounter::ReconstructionInsertionUnits,
		static_cast<std::uint64_t>(insertionUnits.size()));
	result.routes.clear();
	result.objective = 0.0;
	if (diagnostic != nullptr) {
		diagnostic->clear();
	}

	RouteEvaluator evaluator(
		data_, data_.problemContext().temporalConstraint());
	std::vector<int> depotLoad(
		static_cast<std::size_t>(data_.numL), 0);
	std::vector<int> depotRouteCount(
		static_cast<std::size_t>(data_.numL), 0);
	std::vector<unsigned char> depotOpen(
		static_cast<std::size_t>(data_.numL), 0U);
	const bool exhaustiveReference =
		execution_ == FeasibleInsertionExecution::ExhaustiveReference;
	const bool exhaustiveCandidatePath =
		exhaustiveReference || data_.hasHardTimeWindows();
	const bool finalizedCapacityFastPath =
		!exhaustiveCandidatePath &&
		data_.problemDataFinalizedForSearch() &&
		!data_.hasHardTimeWindows();
	const std::vector<int> emptySequence;
	std::vector<int> sequenceScratch;
	std::vector<int> selectedSequence;

	std::size_t insertedCustomers = 0U;
	for (const std::vector<int>& unit : insertionUnits) {
		const int customer = unit.front();
		int unitDemand = 0;
		for (const int member : unit) {
			unitDemand += data_.demCus[member];
		}
		std::vector<Candidate> candidates;

		for (std::size_t routeIndex = 0;
			routeIndex < result.routes.size();
			++routeIndex) {
			const StandaloneRoute& route = result.routes[routeIndex];
			const int depot = route.locId;
			if (!depotAllowed(allowedDepots, depot) ||
				depotLoad[static_cast<std::size_t>(depot)] +
					unitDemand >
				data_.invLoc[data_.numV + depot] ||
				route.totalLoad + unitDemand >
					data_.vehCap) {
				continue;
			}

			for (std::size_t position = 0;
				position <= route.sequence.size();
				++position) {
				recordHotPathCounter(
					HotPathCounter::ReconstructionCandidateSlots);
				RouteEvaluation evaluation;
				std::vector<int> exhaustiveSequence;
				if (finalizedCapacityFastPath) {
					evaluation = evaluateFinalizedCapacityInsertion(
						data_,
						depot,
						route.sequence,
						unit,
						position,
						route.totalLoad + unitDemand);
				} else if (exhaustiveCandidatePath) {
					materializeInsertion(
						route.sequence,
						unit,
						position,
						exhaustiveSequence);
					recordHotPathCounter(
						HotPathCounter::
							ReconstructionSequenceMaterializations);
					evaluation =
						evaluator.evaluate(depot, exhaustiveSequence);
				} else {
					materializeInsertion(
						route.sequence,
						unit,
						position,
						sequenceScratch);
					recordHotPathCounter(
						HotPathCounter::
							ReconstructionSequenceMaterializations);
					evaluation =
						evaluator.evaluate(depot, sequenceScratch);
				}
				recordHotPathCounter(
					HotPathCounter::ReconstructionRouteEvaluations);
				if (!evaluation.feasible()) {
					continue;
				}

				Candidate candidate;
				candidate.opensRoute = false;
				candidate.routeIndex =
					static_cast<int>(routeIndex);
				candidate.depotIndex = depot;
				candidate.insertionPosition = position;
				candidate.objectiveDelta =
					evaluation.travelCost - route.routeCost;
				candidate.evaluation = std::move(evaluation);
				if (exhaustiveCandidatePath) {
					candidate.sequence =
						std::move(exhaustiveSequence);
				}
				candidates.push_back(std::move(candidate));
				recordHotPathCounter(
					HotPathCounter::ReconstructionFeasibleCandidates);
			}
		}

		for (int depot = 0; depot < data_.numL; ++depot) {
			if (!depotAllowed(allowedDepots, depot) ||
				depotLoad[static_cast<std::size_t>(depot)] +
					unitDemand >
				data_.invLoc[data_.numV + depot] ||
				depotRouteCount[static_cast<std::size_t>(depot)] >=
					Individual::representationRouteLimit(data_, depot)) {
				continue;
			}

			recordHotPathCounter(
				HotPathCounter::ReconstructionCandidateSlots);
			RouteEvaluation evaluation;
			if (finalizedCapacityFastPath) {
				evaluation = evaluateFinalizedCapacityInsertion(
					data_,
					depot,
					emptySequence,
					unit,
					0U,
					unitDemand);
			} else {
				evaluation = evaluator.evaluate(depot, unit);
			}
			recordHotPathCounter(
				HotPathCounter::ReconstructionRouteEvaluations);
			if (!evaluation.feasible()) {
				continue;
			}

			Candidate candidate;
			candidate.opensRoute = true;
			candidate.depotIndex = depot;
			candidate.objectiveDelta =
				evaluation.travelCost + data_.usingCostVeh;
			if (depotOpen[static_cast<std::size_t>(depot)] == 0U) {
				candidate.objectiveDelta +=
					data_.openCostLoc[data_.numV + depot];
			}
			candidate.evaluation = std::move(evaluation);
			if (exhaustiveCandidatePath) {
				recordHotPathCounter(
					HotPathCounter::
						ReconstructionSequenceMaterializations);
				candidate.sequence = unit;
			}
			candidates.push_back(std::move(candidate));
			recordHotPathCounter(
				HotPathCounter::ReconstructionFeasibleCandidates);
		}

		if (candidates.empty()) {
			if (!trustedBlocks.empty()) {
				// Trusted blocks are guidance, never a feasibility
				// restriction. A feasible early block can also make a later
				// singleton impossible, so any guided failure restarts through
				// the exact legacy path.
				recordHotPathCounter(
					HotPathCounter::ReconstructionGuidedFallbacks);
				return build(
					customerOrder,
					rankPop,
					std::vector<std::vector<int>>{},
					allowedDepots,
					result,
					diagnostic);
			}
			if (diagnostic != nullptr) {
				*diagnostic = describeFailure(
					data_,
					customer,
					insertedCustomers,
					evaluator,
					allowedDepots);
			}
			result.routes.clear();
			result.objective = 0.0;
			return false;
		}

		std::size_t selectedIndex = 0U;
		std::size_t sortedCandidates = 0U;
		const std::size_t insertionCandidateListWidth =
			static_cast<std::size_t>(
				data_.parameters.construction.insertionCandidateListWidth);
		const double insertionRankExponent =
			data_.parameters.construction.insertionRankExponent;
		if (exhaustiveCandidatePath) {
			std::sort(
				candidates.begin(), candidates.end(), candidateLess);
			selectedIndex = selectCandidateIndex(
				candidates.size(),
				rankPop,
				insertionCandidateListWidth,
				insertionRankExponent);
			sortedCandidates = candidates.size();
		} else {
			selectedIndex = selectedCandidatePosition(
				candidates,
				rankPop,
				insertionCandidateListWidth,
				insertionRankExponent,
				sortedCandidates);
		}
		recordHotPathCounter(
			HotPathCounter::ReconstructionSortedCandidates,
			static_cast<std::uint64_t>(sortedCandidates));
		Candidate& selected = candidates[selectedIndex];
		if (exhaustiveCandidatePath) {
			selectedSequence = std::move(selected.sequence);
		} else if (selected.opensRoute) {
			selectedSequence.assign(unit.begin(), unit.end());
			recordHotPathCounter(
				HotPathCounter::ReconstructionSequenceMaterializations);
		} else {
			materializeInsertion(
				result.routes.at(
					static_cast<std::size_t>(selected.routeIndex)).sequence,
				unit,
				selected.insertionPosition,
				selectedSequence);
			recordHotPathCounter(
				HotPathCounter::ReconstructionSequenceMaterializations);
		}

		const int depot = selected.depotIndex;
		depotLoad[static_cast<std::size_t>(depot)] +=
			unitDemand;
		insertedCustomers += unit.size();
		result.objective += selected.objectiveDelta;

		if (selected.opensRoute) {
			StandaloneRoute route;
			route.locId = depot;
			route.routeCost = selected.evaluation.travelCost;
			route.totalLoad = selected.evaluation.load;
			route.sequence = std::move(selectedSequence);
			result.routes.push_back(std::move(route));
			++depotRouteCount[static_cast<std::size_t>(depot)];
			depotOpen[static_cast<std::size_t>(depot)] = 1U;
		} else {
			StandaloneRoute& route = result.routes.at(
				static_cast<std::size_t>(selected.routeIndex));
			route.routeCost = selected.evaluation.travelCost;
			route.totalLoad = selected.evaluation.load;
			route.sequence = std::move(selectedSequence);
		}
	}

	validateCompletePlan(
		data_, result.routes, evaluator, allowedDepots);
	return true;
}

}  // namespace hgasp
