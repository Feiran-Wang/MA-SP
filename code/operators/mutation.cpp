/******************************************************************************
 * File: mutation.cpp
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description: 
 *   Mutation operators for genetic algorithm diversification.
 ******************************************************************************/

#include "mutation.h"
#include "ComponentProfiler.h"
#include "RouteEvaluator.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

struct TimeWindowInsertionCandidate {
	bool opensRoute = false;
	int routeIndex = -1;
	int depotIndex = -1;
	std::size_t insertionPosition = 0U;
	double objectiveDelta = 0.0;
	double selectionScore = 0.0;
	RouteEvaluation evaluation;
	std::vector<int> sequence;
};

struct TimeWindowPartialPlanState {
	std::vector<int> depotLoads;
	std::vector<int> depotRouteCounts;
	std::vector<unsigned char> depotOpen;
};

bool candidateLess(
		const TimeWindowInsertionCandidate& lhs,
		const TimeWindowInsertionCandidate& rhs) {
	// Use an exact primary comparison: tolerance-based comparisons are not a
	// strict weak ordering and therefore are invalid for lower_bound/sort.
	if (lhs.selectionScore != rhs.selectionScore)
		return lhs.selectionScore < rhs.selectionScore;
	if (lhs.objectiveDelta != rhs.objectiveDelta)
		return lhs.objectiveDelta < rhs.objectiveDelta;
	if (lhs.depotIndex != rhs.depotIndex)
		return lhs.depotIndex < rhs.depotIndex;
	if (lhs.opensRoute != rhs.opensRoute)
		return !lhs.opensRoute;
	if (lhs.routeIndex != rhs.routeIndex)
		return lhs.routeIndex < rhs.routeIndex;
	return lhs.insertionPosition < rhs.insertionPosition;
}

void retainTopInsertionCandidate(
		std::vector<TimeWindowInsertionCandidate>& candidates,
		TimeWindowInsertionCandidate candidate,
		const std::size_t restrictedCandidateListWidth) {
	if (!std::isfinite(candidate.objectiveDelta) ||
		!std::isfinite(candidate.selectionScore)) {
		throw std::logic_error(
			"constraint-aware mutation produced a non-finite insertion delta");
	}
	const auto position = std::lower_bound(
		candidates.begin(),
		candidates.end(),
		candidate,
		candidateLess);
	if (position == candidates.end() &&
		candidates.size() >= restrictedCandidateListWidth) {
		return;
	}
	candidates.insert(position, std::move(candidate));
	if (candidates.size() > restrictedCandidateListWidth)
		candidates.pop_back();
}

std::vector<StandaloneRoute> extractTimeWindowRoutes(
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
			int guard = 0;
			while (node != nullptr && !node->isDepot &&
				   guard++ <= data.numV) {
				valueRoute.sequence.push_back(node->cour);
				node = node->next;
			}
			if (node == nullptr || guard > data.numV ||
				valueRoute.sequence.empty()) {
				return {};
			}
			routes.push_back(std::move(valueRoute));
		}
	}
	return routes;
}

bool evaluateCompleteTimeWindowPlan(
		const SolverContext& data,
		std::vector<StandaloneRoute>& routes,
		double& objective,
		std::vector<double>* const serviceStarts = nullptr,
		std::vector<int>* const customerRoutes = nullptr) {
	RouteEvaluator evaluator(
		data, data.problemContext().temporalConstraint());
	std::vector<int> coverage(
		static_cast<std::size_t>(data.numV), 0);
	std::vector<int> depotLoads(
		static_cast<std::size_t>(data.numL), 0);
	std::vector<int> depotRouteCounts(
		static_cast<std::size_t>(data.numL), 0);
	std::vector<unsigned char> openedDepots(
		static_cast<std::size_t>(data.numL), 0U);

	if (serviceStarts != nullptr) {
		serviceStarts->assign(
			static_cast<std::size_t>(data.numV),
			std::numeric_limits<double>::quiet_NaN());
	}
	if (customerRoutes != nullptr) {
		customerRoutes->assign(
			static_cast<std::size_t>(data.numV), -1);
	}

	objective =
		data.usingCostVeh * static_cast<double>(routes.size());
	for (std::size_t routeIndex = 0;
		 routeIndex < routes.size();
		 ++routeIndex) {
		StandaloneRoute& route = routes[routeIndex];
		if (route.locId < 0 || route.locId >= data.numL ||
			route.sequence.empty()) {
			return false;
		}

		const EvaluationDetail detail =
			serviceStarts == nullptr
				? EvaluationDetail::Summary
				: EvaluationDetail::FullSchedule;
		const RouteEvaluation evaluation =
			evaluator.evaluate(route.locId, route.sequence, detail);
		if (!evaluation.feasible())
			return false;

		const std::size_t depot =
			static_cast<std::size_t>(route.locId);
		depotLoads[depot] += evaluation.load;
		if (depotLoads[depot] >
			data.invLoc[data.numV + route.locId]) {
			return false;
		}
		if (++depotRouteCounts[depot] >
			Individual::representationRouteLimit(
				data, route.locId)) {
			return false;
		}

		for (const int customer : route.sequence) {
			if (customer < 0 || customer >= data.numV ||
				++coverage[static_cast<std::size_t>(customer)] != 1) {
				return false;
			}
			if (customerRoutes != nullptr) {
				(*customerRoutes)[
					static_cast<std::size_t>(customer)] =
					static_cast<int>(routeIndex);
			}
		}
			if (serviceStarts != nullptr) {
				if (!evaluation.schedule.empty()) {
					for (const CustomerSchedule& schedule :
						 evaluation.schedule) {
						(*serviceStarts)[
							static_cast<std::size_t>(
								schedule.customer)] =
							schedule.serviceStartTime;
					}
				} else if (
					data.readyTime.size() ==
						static_cast<std::size_t>(data.numV) &&
					data.serviceTime.size() ==
						static_cast<std::size_t>(data.numV) &&
					data.depotOpenTime.size() ==
						static_cast<std::size_t>(data.numL)) {
					// A no-op temporal constraint deliberately returns no
					// schedule. Finite temporal instance data may still guide
					// Shaw relatedness, so propagate the same earliest-service
					// values without turning them into feasibility constraints.
					double currentTime = data.depotOpenTime[depot];
					int previous = data.numV + route.locId;
					for (const int customer : route.sequence) {
						const double travel = data.T[previous][customer];
						if (!std::isfinite(currentTime) ||
							!std::isfinite(travel) ||
							!std::isfinite(data.readyTime[customer]) ||
							!std::isfinite(data.serviceTime[customer])) {
							break;
						}
						currentTime = std::max(
							currentTime + travel,
							data.readyTime[customer]);
						(*serviceStarts)[
							static_cast<std::size_t>(customer)] =
							currentTime;
						currentTime += data.serviceTime[customer];
						previous = customer;
					}
				}
			}

		route.totalLoad = evaluation.load;
		route.routeCost = evaluation.travelCost;
		route.returnTime = evaluation.returnTime;
		route.totalWaitingTime = evaluation.totalWaitingTime;
		route.temporalFeasible = evaluation.temporalFeasible;
		route.temporalViolationCustomer =
			evaluation.violatingCustomer;
		objective += evaluation.travelCost;
		openedDepots[depot] = 1U;
	}

	for (const int useCount : coverage) {
		if (useCount != 1)
			return false;
	}
	for (int depot = 0; depot < data.numL; ++depot) {
		if (openedDepots[static_cast<std::size_t>(depot)] != 0U)
			objective += data.openCostLoc[data.numV + depot];
	}
	return true;
}

bool equivalentRoutePlans(
		const std::vector<StandaloneRoute>& lhs,
		const std::vector<StandaloneRoute>& rhs) {
	if (lhs.size() != rhs.size())
		return false;

	std::vector<unsigned char> matched(rhs.size(), 0U);
	for (const StandaloneRoute& leftRoute : lhs) {
		bool found = false;
		for (std::size_t index = 0;
			 index < rhs.size();
			 ++index) {
			if (matched[index] == 0U &&
				leftRoute.locId == rhs[index].locId &&
				leftRoute.sequence == rhs[index].sequence) {
				matched[index] = 1U;
				found = true;
				break;
			}
		}
		if (!found)
			return false;
	}
	return true;
}

void removeCustomers(
		std::vector<StandaloneRoute>& routes,
		const std::vector<unsigned char>& removed) {
	for (StandaloneRoute& route : routes) {
		route.sequence.erase(
			std::remove_if(
				route.sequence.begin(),
				route.sequence.end(),
				[&removed](const int customer) {
					return removed[
						static_cast<std::size_t>(customer)] != 0U;
				}),
			route.sequence.end());
	}
	routes.erase(
		std::remove_if(
			routes.begin(),
			routes.end(),
			[](const StandaloneRoute& route) {
				return route.sequence.empty();
			}),
		routes.end());
}

bool refreshPartialPlan(
		const SolverContext& data,
		std::vector<StandaloneRoute>& routes,
		const RouteEvaluator& evaluator,
		TimeWindowPartialPlanState& state) {
	state.depotLoads.assign(
		static_cast<std::size_t>(data.numL), 0);
	state.depotRouteCounts.assign(
		static_cast<std::size_t>(data.numL), 0);
	state.depotOpen.assign(
		static_cast<std::size_t>(data.numL), 0U);

	for (StandaloneRoute& route : routes) {
		if (route.locId < 0 || route.locId >= data.numL ||
			route.sequence.empty()) {
			throw std::logic_error(
				"constraint-aware mutation produced an invalid partial route");
		}
		const RouteEvaluation evaluation =
			evaluator.evaluate(route.locId, route.sequence);
		if (!evaluation.capacityFeasible) {
			throw std::logic_error(
				"constraint-aware mutation customer removal increased a "
				"route load");
		}
		if (!evaluation.temporalFeasible)
			return false;

		const std::size_t depot =
			static_cast<std::size_t>(route.locId);
		state.depotLoads[depot] += evaluation.load;
		if (state.depotLoads[depot] >
			data.invLoc[data.numV + route.locId]) {
			throw std::logic_error(
				"constraint-aware mutation customer removal increased a "
				"depot load");
		}
		if (++state.depotRouteCounts[depot] >
			Individual::representationRouteLimit(
				data, route.locId)) {
			throw std::logic_error(
				"constraint-aware mutation partial plan exceeds its route slots");
		}
		state.depotOpen[depot] = 1U;

		route.totalLoad = evaluation.load;
		route.routeCost = evaluation.travelCost;
		route.returnTime = evaluation.returnTime;
		route.totalWaitingTime = evaluation.totalWaitingTime;
		route.temporalFeasible = evaluation.temporalFeasible;
		route.temporalViolationCustomer =
			evaluation.violatingCustomer;
	}
	return true;
}

std::vector<TimeWindowInsertionCandidate>
collectTimeWindowInsertionCandidates(
		const SolverContext& data,
		const int customer,
		std::vector<StandaloneRoute>& routes,
		const RouteEvaluator& evaluator,
		const TimeWindowPartialPlanState& state) {
	std::vector<TimeWindowInsertionCandidate> candidates;
	// One spare slot permits insert-then-trim without reallocating.
	const std::size_t restrictedCandidateListWidth =
		static_cast<std::size_t>(
			data.parameters.mutation.insertionCandidateListWidth);
	candidates.reserve(restrictedCandidateListWidth + 1U);

	for (std::size_t routeIndex = 0;
		 routeIndex < routes.size();
		 ++routeIndex) {
		const StandaloneRoute& route = routes[routeIndex];
		const int depot = route.locId;
		if (route.totalLoad + data.demCus[customer] >
				data.vehCap ||
			state.depotLoads[static_cast<std::size_t>(depot)] +
					data.demCus[customer] >
				data.invLoc[data.numV + depot]) {
			continue;
		}

		for (std::size_t position = 0;
			 position <= route.sequence.size();
			 ++position) {
			// Build by indexed append instead of iterator arithmetic. Besides
			// making the [0,size] insertion contract explicit, this avoids a
			// GCC release-build false positive that treated begin()+position
			// as a possible -1 subscript after inlining.
			std::vector<int> sequence;
			sequence.reserve(route.sequence.size() + 1U);
			for (std::size_t index = 0;
				 index <= route.sequence.size();
				 ++index) {
				if (index == position) {
					sequence.push_back(customer);
				}
				if (index < route.sequence.size()) {
					sequence.push_back(
						route.sequence[index]);
				}
			}
			RouteEvaluation evaluation =
				evaluator.evaluate(depot, sequence);
			if (!evaluation.capacityFeasible) {
				throw std::logic_error(
					"constraint-aware insertion capacity precheck disagrees "
					"with route evaluation");
			}
			if (!evaluation.temporalFeasible)
				continue;

			TimeWindowInsertionCandidate candidate;
			candidate.opensRoute = false;
			candidate.routeIndex =
				static_cast<int>(routeIndex);
			candidate.depotIndex = depot;
			candidate.insertionPosition = position;
			candidate.objectiveDelta =
				evaluation.travelCost - route.routeCost;
			candidate.selectionScore = candidate.objectiveDelta;
			candidate.evaluation = std::move(evaluation);
			candidate.sequence = std::move(sequence);
			retainTopInsertionCandidate(
				candidates,
				std::move(candidate),
				restrictedCandidateListWidth);
		}
	}

	for (int depot = 0; depot < data.numL; ++depot) {
		const std::size_t depotIndex =
			static_cast<std::size_t>(depot);
		if (state.depotLoads[depotIndex] +
				data.demCus[customer] >
				data.invLoc[data.numV + depot] ||
			state.depotRouteCounts[depotIndex] >=
				Individual::representationRouteLimit(data, depot)) {
			continue;
		}

		std::vector<int> sequence{customer};
		RouteEvaluation evaluation =
			evaluator.evaluate(depot, sequence);
		if (!evaluation.capacityFeasible) {
			throw std::logic_error(
				"constraint-aware singleton insertion exceeds vehicle "
				"capacity despite a feasible incumbent");
		}
		if (!evaluation.temporalFeasible)
			continue;

		TimeWindowInsertionCandidate candidate;
		candidate.opensRoute = true;
		candidate.depotIndex = depot;
		candidate.objectiveDelta =
			evaluation.travelCost + data.usingCostVeh;
		if (state.depotOpen[depotIndex] == 0U) {
			candidate.objectiveDelta +=
				data.openCostLoc[data.numV + depot];
		}
		candidate.selectionScore = candidate.objectiveDelta;
		candidate.evaluation = std::move(evaluation);
		candidate.sequence = std::move(sequence);
		retainTopInsertionCandidate(
			candidates,
			std::move(candidate),
			restrictedCandidateListWidth);
	}

	return candidates;
}

bool insertCustomerTimeWindowFeasible(
		const SolverContext& data,
		const int customer,
		std::vector<StandaloneRoute>& routes,
		const RouteEvaluator& evaluator,
		TimeWindowPartialPlanState& state) {
	std::vector<TimeWindowInsertionCandidate> candidates =
		collectTimeWindowInsertionCandidates(
			data,
			customer,
			routes,
			evaluator,
			state);
	if (candidates.empty())
		return false;

	// Shaw-style recreate uses a small randomized restricted candidate list.
	// The vector already contains exactly the configured globally best
	// candidates,
	// so no full candidate materialization or sort is needed.
	const std::size_t width = candidates.size();
	const double biased =
		std::pow(
			RandomDouble(0.0, 1.0),
			data.parameters.mutation.insertionRankExponent);
	const std::size_t selectedIndex =
		std::min(
			width - 1U,
			static_cast<std::size_t>(
				biased * static_cast<double>(width)));
	TimeWindowInsertionCandidate selected =
		std::move(candidates[selectedIndex]);

	if (selected.opensRoute) {
		StandaloneRoute route;
		route.locId = selected.depotIndex;
		route.sequence = std::move(selected.sequence);
		route.totalLoad = selected.evaluation.load;
		route.routeCost = selected.evaluation.travelCost;
		route.returnTime = selected.evaluation.returnTime;
		route.totalWaitingTime =
			selected.evaluation.totalWaitingTime;
		route.temporalFeasible =
			selected.evaluation.temporalFeasible;
		route.temporalViolationCustomer =
			selected.evaluation.violatingCustomer;
		routes.push_back(std::move(route));
		const std::size_t depot =
			static_cast<std::size_t>(selected.depotIndex);
		state.depotLoads[depot] +=
			selected.evaluation.load;
		++state.depotRouteCounts[depot];
		state.depotOpen[depot] = 1U;
	} else {
		StandaloneRoute& route = routes.at(
			static_cast<std::size_t>(selected.routeIndex));
		const int previousLoad = route.totalLoad;
		route.sequence = std::move(selected.sequence);
		route.totalLoad = selected.evaluation.load;
		route.routeCost = selected.evaluation.travelCost;
		route.returnTime = selected.evaluation.returnTime;
		route.totalWaitingTime =
			selected.evaluation.totalWaitingTime;
		route.temporalFeasible =
			selected.evaluation.temporalFeasible;
		route.temporalViolationCustomer =
			selected.evaluation.violatingCustomer;
		state.depotLoads[static_cast<std::size_t>(
			selected.depotIndex)] +=
			selected.evaluation.load - previousLoad;
	}
	return true;
}

}  // namespace

mutation::mutation(SolverContext * data) {
	this->data = data;
	this->numV = data->numV;
	this->numL = data->numL;
	auxArray1.resize(static_cast<std::size_t>(numV));
	this->perturbLength = data->ratioPerturbLength*numV;
	cityRank.resize(static_cast<std::size_t>(numV));
}
void mutation::validateTimeWindowCommit(Individual *target) const {
	if (target == nullptr) {
		throw std::invalid_argument(
			"mutation commit validation requires a target");
	}
	target->evaluationDis();
	target->decideFeasible();
	if (!target->isFeasible) {
		throw std::logic_error(
			"mutation commit failed active-constraint validation");
	}
	target->isRight();
}
void mutation::initializeTimeWindowNormalization() {
	if (timeWindowNormalizationInitialized)
		return;

	maximumCustomerDistance = 0.0;
	for (int first = 0; first < numV; ++first) {
		for (int second = first + 1;
			 second < numV;
			 ++second) {
			maximumCustomerDistance = std::max(
				maximumCustomerDistance,
				0.5 * (
					data->D[first][second] +
					data->D[second][first]));
		}
	}
		maximumCustomerDistance =
			std::max(maximumCustomerDistance, 1.0);
		temporalHeuristicFeaturesAvailable =
			data->readyTime.size() == static_cast<std::size_t>(numV) &&
			data->dueTime.size() == static_cast<std::size_t>(numV) &&
			data->serviceTime.size() == static_cast<std::size_t>(numV) &&
			data->depotOpenTime.size() == static_cast<std::size_t>(numL) &&
			data->depotCloseTime.size() == static_cast<std::size_t>(numL);
		if (temporalHeuristicFeaturesAvailable) {
			for (int customer = 0; customer < numV; ++customer) {
				const std::size_t index =
					static_cast<std::size_t>(customer);
				temporalHeuristicFeaturesAvailable =
					temporalHeuristicFeaturesAvailable &&
					std::isfinite(data->readyTime[index]) &&
					std::isfinite(data->dueTime[index]) &&
					std::isfinite(data->serviceTime[index]) &&
					data->readyTime[index] <= data->dueTime[index] &&
					data->serviceTime[index] >= 0.0;
			}
			for (int depot = 0; depot < numL; ++depot) {
				const std::size_t index =
					static_cast<std::size_t>(depot);
				temporalHeuristicFeaturesAvailable =
					temporalHeuristicFeaturesAvailable &&
					std::isfinite(data->depotOpenTime[index]) &&
					std::isfinite(data->depotCloseTime[index]) &&
					data->depotOpenTime[index] <=
						data->depotCloseTime[index];
			}
		}
		if (!temporalHeuristicFeaturesAvailable) {
			timeWindowCentres.assign(
				static_cast<std::size_t>(numV), 0.0);
			timeWindowTimeScale = 1.0;
			timeWindowDemandScale =
				static_cast<double>(std::max(1, data->vehCap));
			timeWindowNormalizationInitialized = true;
			return;
		}

		double earliestTime =
		std::numeric_limits<double>::infinity();
	double latestTime =
		-std::numeric_limits<double>::infinity();
	timeWindowCentres.assign(
		static_cast<std::size_t>(numV), 0.0);
	for (int customer = 0; customer < numV; ++customer) {
		const std::size_t index =
			static_cast<std::size_t>(customer);
		earliestTime = std::min(
			earliestTime, data->readyTime[index]);
		latestTime = std::max(
			latestTime, data->dueTime[index]);
		timeWindowCentres[index] =
			0.5 * (
				data->readyTime[index] +
				data->dueTime[index]);
	}
	for (int depot = 0; depot < numL; ++depot) {
		const std::size_t index =
			static_cast<std::size_t>(depot);
		earliestTime = std::min(
			earliestTime, data->depotOpenTime[index]);
		latestTime = std::max(
			latestTime, data->depotCloseTime[index]);
	}
		timeWindowTimeScale =
		std::max(1.0, latestTime - earliestTime);
	timeWindowDemandScale =
		static_cast<double>(std::max(1, data->vehCap));
	timeWindowNormalizationInitialized = true;
}
double mutation::compute(bool istrue, double value){
	if (istrue)	return 0.;
	else return value;
}
bool mutation::shawRemoval(){
	// the first step is to select customers
	for (int i=0;i<numV;i++)auxArray1[i]=0;
	int seed = selectRemovalSeed();
	auxArray1[seed]=1;
	listRemove.push_back(seed);
	int iter;double y;int rank;
	while((int)listRemove.size() < perturbLength){//select more 1/3 cities and select randomly
		seed = RandomInt(0, listRemove.size() - 1);
		iter=0;
		for (int i=0;i<numV-1;i++){// not the location
			if (data->nearCity[listRemove[seed]][i] < numV && auxArray1[data->nearCity[listRemove[seed]][i]]==0){
				cityRank[iter]=data->nearCity[listRemove[seed]][i];
				iter++;
			}
		}
		if (iter == 0)
			return false;
		y = RandomDouble(0.0, 1.0);
		rank=int(pow(y,data->determine_para)*iter);
		listRemove.push_back(cityRank[rank]);
		auxArray1[cityRank[rank]]=1;
	}
	//the second step is to remove customers
	Node *p = NULL;
	Node *pre = NULL, *next = NULL;
	for (int i=0;i<(int)listRemove.size();i++){
		p = &s->client[listRemove[i]];
		pre = p->pre;
		next = p->next;
		pre->next = next;
		next->pre = pre;
		p->pre = NULL;
		p->next = NULL;
	}
	for (int i=0;i<numL;i++){
		for (int j=0;j<s->location[i].maxVisit;j++)
			s->updateRouteInfor(s->location[i].depots[j].route->cour, -1);
		int ll = i+numV;
		s->updateLocationInfor(ll);
	}
//	if (data->iteration == 313)
//		s->outputSolution();

	//the third step is to add customers
	RandomShuffle(listRemove);// randomize all removed customers
	double delta, min_delta;
	Node *node, *nodeU, *nodeX, *pos;
	while ((int)listRemove.size() != 0){
		min_delta = Max; //listRemove
		pos = NULL;
		node = &s->client[listRemove[0]];
		for (int i=0;i<(int)data->edgeNode[node->cour].size();i++){
			if (data->edgeNode[node->cour][i] >= numV) continue;
			if (auxArray1[data->edgeNode[node->cour][i]] == 0){
				nodeU = &s->client[data->edgeNode[node->cour][i]];
				if (nodeU->route->load + node->dem > data->vehCap || nodeU->route->whichLocation->remainInven + node->dem > nodeU->route->whichLocation->inventory)continue;
				if (nodeU->route->cour == node->route->cour)continue;
				delta = data->D[nodeU->cour][node->cour] + data->D[node->cour][nodeU->next->cour] - data->D[nodeU->cour][nodeU->next->cour];
				if (delta < min_delta){
					pos = nodeU;
					min_delta = delta;
				}
			}
		}
		for (int i=0; i<(int)data->customerToLoc[node->cour].size(); i++){
			if (s->location[data->customerToLoc[node->cour][i] - numV].remainInven + node->dem > s->location[data->customerToLoc[node->cour][i] - numV].inventory)continue;
			if (s->location[data->customerToLoc[node->cour][i] - numV].emptyRoutes.empty())continue;
			nodeU = s->route[*s->location[data->customerToLoc[node->cour][i] - numV].emptyRoutes.begin()].depot;
			nodeX = nodeU->next;
			delta = compute(nodeU->route->whichLocation->used, data->openCostLoc[nodeU->route->whichLocation->cour])
					+ compute(!(nodeU->isDepot && nodeX->isDepot), data->usingCostVeh)
					+ data->D[nodeU->cour][node->cour] * 2;
			if (delta < min_delta){
				pos = nodeU;
				min_delta = delta;
			}
		}
		// remedy strategy
		if (min_delta == Max){
			for (int i=0; i<numL; i++){
				if (s->location[i].used){
					if (s->location[i].remainInven + node->dem > s->location[i].inventory)continue;
					if (s->location[i].emptyRoutes.empty())continue;
					nodeU = s->route[*s->location[i].emptyRoutes.begin()].depot;
					nodeX = nodeU->next;
					delta = compute(nodeU->route->whichLocation->used, data->openCostLoc[nodeU->route->whichLocation->cour])
							+ compute(!(nodeU->isDepot && nodeX->isDepot), data->usingCostVeh)
							+ data->D[nodeU->cour][node->cour] * 2;
					if (delta < min_delta){
						pos = nodeU;
						min_delta = delta;
					}
				}
			}
			if (min_delta == Max){
				for (int i=0; i<numL; i++){
					if (!s->location[i].used){
						if (s->location[i].remainInven + node->dem > s->location[i].inventory)continue;
						if (s->location[i].emptyRoutes.empty())continue;
						nodeU = s->route[*s->location[i].emptyRoutes.begin()].depot;
						nodeX = nodeU->next;
						delta = compute(nodeU->route->whichLocation->used, data->openCostLoc[nodeU->route->whichLocation->cour])
								+ compute(!(nodeU->isDepot && nodeX->isDepot), data->usingCostVeh)
								+ data->D[nodeU->cour][node->cour] * 2;
						if (delta < min_delta){
							pos = nodeU;
							min_delta = delta;
						}
					}
				}
			}
		}

		// Greedy recreation can consume a route or depot option needed by a
		// later removed customer.  That is a rejected mutation, not a valid
		// insertion at a null anchor; shawMutation restores the incumbent.
		if (pos == NULL)
			return false;
		insertNode(node,pos);
//		if (data->iteration == 313)
//			s->outputSolution();
		auxArray1[node->cour] = 0;
		listRemove.erase(listRemove.begin());
		s->updateRouteInfor(pos->route->cour,-1);
		s->updateLocationInfor(pos->route->whichLocation->cour);
	}
	return true;
}
void mutation::insertNode(Node *U, Node *V){
	V->next->pre = U;
	U->next = V->next;
	V->next = U;
	U->pre = V;
	U->route = V->route;
}
bool mutation::constraintAwareRuinRecreate() {
	if (s == nullptr) {
		throw std::logic_error(
			"constraint-aware mutation has no target individual");
	}
	if (numV <= 1)
		return false;

	std::vector<StandaloneRoute> incumbent =
		extractTimeWindowRoutes(*s, *data);
	double incumbentObjective = 0.0;
	std::vector<double> serviceStarts;
	std::vector<int> customerRoutes;
	if (incumbent.empty()) {
		throw std::logic_error(
			"constraint-aware mutation could not extract its incumbent "
			"route plan");
	}
	if (!evaluateCompleteTimeWindowPlan(
			*data,
			incumbent,
			incumbentObjective,
			&serviceStarts,
			&customerRoutes)) {
		throw std::logic_error(
			"constraint-aware mutation received an infeasible or malformed "
			"incumbent");
	}
	if (!data->equalTwoVar(incumbentObjective, s->fit)) {
		throw std::logic_error(
			"constraint-aware mutation incumbent objective does not match "
			"its route plan");
	}
	initializeTimeWindowNormalization();

	// Related removal follows Shaw's spatial/temporal/demand/route
	// relatedness principle.  Every term is normalized to keep the choice
	// meaningful when benchmark coordinate, time, and demand scales differ.
	auto relatedness =
		[this,
		 &serviceStarts,
		 &customerRoutes](
			const int first,
			const int second) {
			const double spatial =
				0.5 * (
					data->D[first][second] +
					data->D[second][first]) /
				maximumCustomerDistance;
				const double demandProximity =
					std::fabs(
						static_cast<double>(data->demCus[first]) -
						static_cast<double>(data->demCus[second])) /
					timeWindowDemandScale;
				const double differentRoute =
					customerRoutes[
						static_cast<std::size_t>(first)] ==
					customerRoutes[
						static_cast<std::size_t>(second)]
						? 0.0
						: 1.0;
				if (!temporalHeuristicFeaturesAvailable) {
					// Preserve the relative weights of the non-temporal Shaw
					// features when an instance supplies no finite time data.
					return (
						0.40 * spatial +
						0.10 * demandProximity +
						0.05 * differentRoute) / 0.55;
				}
				const double serviceTimeProximity =
					std::fabs(
					serviceStarts[
						static_cast<std::size_t>(first)] -
					serviceStarts[
						static_cast<std::size_t>(second)]) /
				timeWindowTimeScale;
			const double firstWindowCentre =
				timeWindowCentres[
					static_cast<std::size_t>(first)];
			const double secondWindowCentre =
				timeWindowCentres[
					static_cast<std::size_t>(second)];
			const double windowProximity =
				std::fabs(
					firstWindowCentre -
					secondWindowCentre) /
				timeWindowTimeScale;
				return 0.40 * spatial +
				0.30 * serviceTimeProximity +
				0.15 * windowProximity +
				0.10 * demandProximity +
				0.05 * differentRoute;
		};

	const int removalCount = std::min(
		numV,
		std::max(2, perturbLength));
	std::vector<unsigned char> removed(
		static_cast<std::size_t>(numV), 0U);
	std::vector<int> removedCustomers;
	removedCustomers.reserve(
		static_cast<std::size_t>(removalCount));
	const int seed = selectRemovalSeed();
	removed[static_cast<std::size_t>(seed)] = 1U;
	removedCustomers.push_back(seed);

	while (static_cast<int>(removedCustomers.size()) <
		   removalCount) {
		const int reference =
			removedCustomers[static_cast<std::size_t>(
				RandomInt(
					0,
					static_cast<int>(
						removedCustomers.size()) -
						1))];
		std::vector<std::pair<double, int>> candidates;
		candidates.reserve(
			static_cast<std::size_t>(
				numV -
				static_cast<int>(removedCustomers.size())));
		for (int customer = 0; customer < numV; ++customer) {
			if (removed[static_cast<std::size_t>(customer)] == 0U) {
				candidates.push_back({
					relatedness(reference, customer),
					customer});
			}
		}
		if (candidates.empty()) {
			throw std::logic_error(
				"constraint-aware mutation related-removal candidate set "
				"became empty");
		}
		const double biased =
			std::pow(
				RandomDouble(0.0, 1.0),
				data->determine_para);
		const std::size_t rank = std::min(
			candidates.size() - 1U,
			static_cast<std::size_t>(
				biased *
					static_cast<double>(candidates.size())));
		std::nth_element(
			candidates.begin(),
			candidates.begin() +
				static_cast<std::ptrdiff_t>(rank),
			candidates.end());
		const int customer = candidates[rank].second;
		removed[static_cast<std::size_t>(customer)] = 1U;
		removedCustomers.push_back(customer);
	}

	// Work exclusively on detached StandaloneRoute values.  The linked-list
	// individual is not touched until a complete plan has independently
	// passed all constraints.
	for (int attempt = 0;
		 attempt < data->parameters.mutation.recreateAttempts;
		 ++attempt) {
		std::vector<StandaloneRoute> candidate = incumbent;
		removeCustomers(candidate, removed);
		RouteEvaluator evaluator(
			*data, data->problemContext().temporalConstraint());
		TimeWindowPartialPlanState partialState;
		if (!refreshPartialPlan(
				*data,
				candidate,
				evaluator,
				partialState)) {
			// Removing customers can make a non-metric shortcut violate a
			// hard time window.  This is an expected failed recreation, not
			// a corrupt incumbent.
			continue;
		}

		std::vector<int> insertionOrder = removedCustomers;
			if (attempt == 0 &&
				temporalHeuristicFeaturesAvailable) {
			std::stable_sort(
				insertionOrder.begin(),
				insertionOrder.end(),
				[this](const int lhs, const int rhs) {
					const double lhsDue =
						data->dueTime[
							static_cast<std::size_t>(lhs)];
					const double rhsDue =
						data->dueTime[
							static_cast<std::size_t>(rhs)];
					if (lhsDue != rhsDue)
						return lhsDue < rhsDue;
					return lhs < rhs;
				});
		} else {
			RandomShuffle(insertionOrder);
		}

		bool recreated = true;
		for (const int customer : insertionOrder) {
			if (!insertCustomerTimeWindowFeasible(
					*data,
					customer,
					candidate,
					evaluator,
					partialState)) {
				recreated = false;
				break;
			}
		}
		if (!recreated)
			continue;

		double candidateObjective = 0.0;
		if (!evaluateCompleteTimeWindowPlan(
				*data,
				candidate,
				candidateObjective)) {
			throw std::logic_error(
				"constraint-aware recreation passed incremental checks but "
				"failed complete-plan validation");
		}
		if (equivalentRoutePlans(candidate, incumbent)) {
			continue;
		}

		try {
			s->rebuildFromSP(candidate);
			validateTimeWindowCommit(s);
			return true;
		} catch (...) {
			// Validation or commit failures are internal errors, not ordinary
			// "no mutation found" outcomes.  Restore the exact incumbent and
			// propagate the original failure so the experiment cannot
			// silently continue with a degraded operator.
			const std::exception_ptr commitFailure =
				std::current_exception();
			try {
				s->rebuildFromSP(incumbent);
				s->evaluationDis();
				s->decideFeasible();
				if (!s->isFeasible)
					throw std::logic_error(
						"restored mutation incumbent is infeasible");
				s->isRight();
			} catch (const std::exception& restoreError) {
				throw std::runtime_error(
					std::string(
						"mutation commit failed and incumbent "
						"restoration also failed: ") +
					restoreError.what());
			}
			std::rethrow_exception(commitFailure);
		}
	}
	return false;
}

std::optional<hgasp::TimeWindowInsertionPreview>
mutation::previewBestTimeWindowInsertion(
			const int customer,
			const std::vector<StandaloneRoute>& partialRoutes) {
		if (customer < 0 || customer >= numV) {
		throw std::out_of_range(
			"TW insertion preview customer is out of range");
	}

	initializeTimeWindowNormalization();
	std::vector<StandaloneRoute> routes = partialRoutes;
	RouteEvaluator evaluator(
		*data, data->problemContext().temporalConstraint());
	TimeWindowPartialPlanState partialState;
	if (!refreshPartialPlan(
			*data,
			routes,
			evaluator,
			partialState)) {
		return std::nullopt;
	}
	const std::vector<TimeWindowInsertionCandidate> candidates =
		collectTimeWindowInsertionCandidates(
			*data,
			customer,
			routes,
			evaluator,
			partialState);
	if (candidates.empty())
		return std::nullopt;

	const TimeWindowInsertionCandidate& best = candidates.front();
	return hgasp::TimeWindowInsertionPreview{
		best.opensRoute,
		best.routeIndex,
		best.depotIndex,
		best.insertionPosition,
		best.objectiveDelta,
		best.selectionScore};
}

int mutation::selectRemovalSeed() const {
	return RandomInt(0, numV - 1);
}

bool mutation::mutationRun(Individual *s){
	hgasp::ScopedComponentTimer timer(
		hgasp::TimedComponent::Mutation);
	if (s == nullptr) {
		throw std::invalid_argument(
			"mutationRun requires a non-null individual");
	}
		this->s=s;
		if (data->parameters.mutation.policy ==
			hgasp::MutationPolicy::ShawOnly) {
			return shawMutation();
		}
		const bool useConstraintAware =
			(mutationRunCount_++ % 2U) == 0U;
		if (useConstraintAware)
			return constraintAwareRuinRecreate();
		return shawMutation();
}

bool mutation::shawMutation() {
	// Shaw removal is the HGAMP baseline operator. Active-constraint
	// validation makes the in-place operator transactional.
		if (numV <= 1)
			return false;
		std::vector<StandaloneRoute> incumbent =
			extractTimeWindowRoutes(*s, *data);
		double incumbentObjective = 0.0;
		if (incumbent.empty() ||
			!evaluateCompleteTimeWindowPlan(
				*data, incumbent, incumbentObjective) ||
			!data->equalTwoVar(incumbentObjective, s->fit)) {
			throw std::logic_error(
				"Shaw mutation received an infeasible or malformed "
				"incumbent");
		}
		auto restoreIncumbent = [this, &incumbent]() {
			listRemove.clear();
			s->rebuildFromSP(incumbent);
			s->evaluationDis();
			s->decideFeasible();
			if (!s->isFeasible) {
				throw std::logic_error(
					"Shaw rollback restored an infeasible incumbent");
			}
			s->isRight();
		};

		listRemove.clear();
		try {
			if (!shawRemoval()) {
				restoreIncumbent();
				return false;
			}
			s->evaluationDis();
			s->decideFeasible();
			if (!s->isFeasible) {
				restoreIncumbent();
				return false;
			}
			s->isRight();
			return true;
		} catch (...) {
			const std::exception_ptr failure =
				std::current_exception();
			restoreIncumbent();
			std::rethrow_exception(failure);
		}
}
