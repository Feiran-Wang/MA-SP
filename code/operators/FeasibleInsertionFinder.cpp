/******************************************************************************
 * File: FeasibleInsertionFinder.cpp
 * Description:
 *   Exhaustive candidate enumeration for hard-time-window insertion.
 ******************************************************************************/

#include "FeasibleInsertionFinder.h"

#include "Individual.h"
#include "ProblemMode.h"
#include "RouteEvaluator.h"
#include "basic.h"
#include "read_data.h"

#include <algorithm>
#include <cmath>
#include <limits>
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

bool candidateLess(const Candidate& lhs, const Candidate& rhs) {
	constexpr double epsilon = 1.0e-12;
	if (std::fabs(lhs.objectiveDelta - rhs.objectiveDelta) > epsilon) {
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

std::size_t selectCandidateIndex(
	const std::size_t candidateCount,
	const int rankPop) {
	if (candidateCount == 0U || rankPop < 0) {
		return 0U;
	}

	// A bounded RCL gives population diversity without turning construction
	// into an unconstrained random assignment.  rankPop=0 remains the exact
	// cheapest-insertion policy and therefore provides a deterministic anchor.
	const std::size_t maximumWidth = 8U;
	const std::size_t width = std::min(
		candidateCount,
		1U + static_cast<std::size_t>(rankPop) % maximumWidth);
	if (width == 1U) {
		return 0U;
	}

	const double biased =
		std::pow(RandomDouble(0.0, 1.0), 6.0);
	return std::min(
		width - 1U,
		static_cast<std::size_t>(
			biased * static_cast<double>(width)));
}

void validatePermutation(
	const read_data& data,
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

std::string describeFailure(
	const read_data& data,
	const int customer,
	const std::size_t insertedCount,
	const RouteEvaluator& evaluator) {
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
		if (data.demCus[customer] >
			data.invLoc[data.numV + depot]) {
			continue;
		}
		const RouteEvaluation singleton =
			evaluator.evaluate(depot, std::vector<int>{customer});
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
	return message.str();
}

void validateCompletePlan(
	const read_data& data,
	const std::vector<StandaloneRoute>& routes,
	const RouteEvaluator& evaluator) {
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
		const RouteEvaluation evaluation =
			evaluator.evaluate(route.locId, route.sequence);
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
	const read_data& data)
	: data_(data) {
	if (data_.problemMode != ProblemMode::ClrpTw) {
		throw std::invalid_argument(
			"hard-time-window feasible insertion requires clrp-tw mode");
	}
}

bool FeasibleInsertionFinder::build(
	const std::vector<int>& customerOrder,
	const int rankPop,
	FeasibleInsertionResult& result,
	std::string* const diagnostic) const {
	validatePermutation(data_, customerOrder);
	result.routes.clear();
	result.objective = 0.0;
	if (diagnostic != nullptr) {
		diagnostic->clear();
	}

	HardTimeWindowConstraint timeWindows;
	RouteEvaluator evaluator(data_, timeWindows);
	std::vector<int> depotLoad(
		static_cast<std::size_t>(data_.numL), 0);
	std::vector<int> depotRouteCount(
		static_cast<std::size_t>(data_.numL), 0);
	std::vector<unsigned char> depotOpen(
		static_cast<std::size_t>(data_.numL), 0U);

	for (std::size_t orderIndex = 0;
			orderIndex < customerOrder.size();
			++orderIndex) {
		const int customer = customerOrder[orderIndex];
		std::vector<Candidate> candidates;

		for (std::size_t routeIndex = 0;
			routeIndex < result.routes.size();
			++routeIndex) {
			const StandaloneRoute& route = result.routes[routeIndex];
			const int depot = route.locId;
			if (depotLoad[static_cast<std::size_t>(depot)] +
					data_.demCus[customer] >
				data_.invLoc[data_.numV + depot] ||
				route.totalLoad + data_.demCus[customer] >
					data_.vehCap) {
				continue;
			}

			for (std::size_t position = 0;
				position <= route.sequence.size();
				++position) {
				std::vector<int> sequence;
				sequence.reserve(route.sequence.size() + 1U);
				for (std::size_t index = 0;
					index <= route.sequence.size();
					++index) {
					if (index == position) {
						sequence.push_back(customer);
					}
					if (index < route.sequence.size()) {
						sequence.push_back(route.sequence[index]);
					}
				}
				RouteEvaluation evaluation =
					evaluator.evaluate(depot, sequence);
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
				candidate.sequence = std::move(sequence);
				candidates.push_back(std::move(candidate));
			}
		}

		for (int depot = 0; depot < data_.numL; ++depot) {
			if (depotLoad[static_cast<std::size_t>(depot)] +
					data_.demCus[customer] >
				data_.invLoc[data_.numV + depot] ||
				depotRouteCount[static_cast<std::size_t>(depot)] >=
					Individual::representationRouteLimit(data_, depot)) {
				continue;
			}

			std::vector<int> singleton{customer};
			RouteEvaluation evaluation =
				evaluator.evaluate(depot, singleton);
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
			candidate.sequence = std::move(singleton);
			candidates.push_back(std::move(candidate));
		}

		if (candidates.empty()) {
			if (diagnostic != nullptr) {
				*diagnostic = describeFailure(
					data_, customer, orderIndex, evaluator);
			}
			result.routes.clear();
			result.objective = 0.0;
			return false;
		}

		std::sort(
			candidates.begin(), candidates.end(), candidateLess);
		Candidate selected = std::move(
			candidates[selectCandidateIndex(
				candidates.size(), rankPop)]);

		const int depot = selected.depotIndex;
		depotLoad[static_cast<std::size_t>(depot)] +=
			data_.demCus[customer];
		result.objective += selected.objectiveDelta;

		if (selected.opensRoute) {
			StandaloneRoute route;
			route.locId = depot;
			route.routeCost = selected.evaluation.travelCost;
			route.totalLoad = selected.evaluation.load;
			route.sequence = std::move(selected.sequence);
			result.routes.push_back(std::move(route));
			++depotRouteCount[static_cast<std::size_t>(depot)];
			depotOpen[static_cast<std::size_t>(depot)] = 1U;
		} else {
			StandaloneRoute& route = result.routes.at(
				static_cast<std::size_t>(selected.routeIndex));
			route.routeCost = selected.evaluation.travelCost;
			route.totalLoad = selected.evaluation.load;
			route.sequence = std::move(selected.sequence);
		}
	}

	validateCompletePlan(data_, result.routes, evaluator);
	return true;
}

}  // namespace hgasp
