/******************************************************************************
 * File: initialsol.cpp
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description: 
 *   Heuristics for generating the initial population and starting routes.
 ******************************************************************************/

#include "basic.h"
#include "ComponentProfiler.h"
#include "FeasibleInsertionFinder.h"
#include "RouteEvaluator.h"
#include "initialsol.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <numeric>
#include <set>
#include <stdexcept>
#include <utility>

namespace {

struct ExactSeedEdge {
	double roundTripCost = 0.0;
	std::size_t depotOrdinal = 0U;
	int customer = -1;
};

bool exactSeedEdgeLess(
		const ExactSeedEdge& left,
		const ExactSeedEdge& right) noexcept {
	if (left.roundTripCost != right.roundTripCost) {
		return left.roundTripCost < right.roundTripCost;
	}
	if (left.depotOrdinal != right.depotOrdinal) {
		return left.depotOrdinal < right.depotOrdinal;
	}
	return left.customer < right.customer;
}

struct ExactInsertionCandidate {
	double objectiveDelta = 0.0;
	bool opensRoute = false;
	std::size_t routeIndex = 0U;
	int depot = -1;
	std::size_t insertionPosition = 0U;
};

bool exactInsertionCandidateLess(
		const ExactInsertionCandidate& left,
		const ExactInsertionCandidate& right) noexcept {
	constexpr double epsilon = 1.0e-12;
	if (std::abs(left.objectiveDelta - right.objectiveDelta) > epsilon) {
		return left.objectiveDelta < right.objectiveDelta;
	}
	if (left.opensRoute != right.opensRoute) {
		return !left.opensRoute;
	}
	if (left.depot != right.depot) {
		return left.depot < right.depot;
	}
	if (left.routeIndex != right.routeIndex) {
		return left.routeIndex < right.routeIndex;
	}
	return left.insertionPosition < right.insertionPosition;
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

void clearFailedExactConfigurationSolution(Individual* const target) {
	if (target == nullptr) {
		return;
	}
	target->initilization();
	target->evaluationDis();
	target->decideFeasible();
	// Complete coverage, not the legacy capacity-only predicate, determines
	// whether an initial seed is usable.
	target->isFeasible = false;
}

}  // namespace

void hgasp::requireValidExactInsertion(
		const std::size_t routeCount,
		const std::size_t routeIndex,
		const std::size_t routeSize,
		const std::size_t insertionPosition) {
	if (routeIndex >= routeCount) {
		throw std::logic_error(
			"exact-configuration insertion route index is out of range");
	}
	if (insertionPosition > routeSize) {
		throw std::logic_error(
			"exact-configuration insertion position is out of range");
	}
}

initial_sol::initial_sol(SolverContext * data, RepairProcedure *repair) {
	this->repair = repair;
	this->data=data;
	this->numV = data->numV;
	this->numL = data->numL;
	this->numC = data->numC;
	customer.resize(static_cast<std::size_t>(numV));
	for (int i=0;i<numV;i++)
		sque.push_back(i);
	locUsed.resize(static_cast<std::size_t>(numC));
	cusUsed.resize(static_cast<std::size_t>(numV));
}
void initial_sol::insertCustomerToLocation(int &customer, int &loc, int newCus){
	Node *U = &s->client[customer];
	if (newCus > 0){
		Node *V = &s->client[newCus];
		insertClient(U, V);
		s->updateRouteInfor(U->route->cour,-1);
		s->updateLocationInfor(U->route->whichLocation->cour);
		return;
	}
	for (int i=0;i<s->location[loc].maxVisit; i++){
		if (!(s->location[loc].depots[i].route->load + data->demCus[customer] > data->vehCap)){
			Node *p= s->location[loc].depots[i].next;
			if (p != NULL){
				Node *V = NULL;
				double delta=Max;
				double tem =  data->D[customer][s->location[loc].cour] + data->D[customer][p->cour] - data->D[p->cour][s->location[loc].cour];
				if ( tem < delta){
					delta = tem;
					V=&s->location[loc].depots[i];
				}
				while(1){
					p=p->next;
					if (p->next->isDepot)break;
					tem = data->D[p->pre->cour][customer] + data->D[customer][p->cour] - data->D[p->pre->cour][p->cour];
					if (tem < delta){
						delta = tem;
						V = p->pre;
					}
				}
		//		tem = data->D[p->cour][customer] + data->D[customer][s->location[loc].cour] - data->D[p->cour][s->location[loc].cour];
		//		if (tem < delta){
		//			delta = tem;
		//			V = p;
		//		}
				insertClient(U, V);
				s->updateRouteInfor(U->route->cour,-1);
				s->updateLocationInfor(U->route->whichLocation->cour);
			}
			else{// open a new route
				Node *V = &s->location[loc].depots[i];
				insertClient(U, V);
				U->route->used = true;
				s->location[loc].used = true;
				s->location[loc].splitTime++;
				s->updateRouteInfor(U->route->cour,-1);
				s->updateLocationInfor(U->route->whichLocation->cour);
			}
			return;
		}
	}
}
void initial_sol::greedyInsert(){
	s->initilization();
	for (int i=0; i<numV; i++)	customer[i]=0;
	RandomShuffle(sque);// to random all customers
	int cus;
	for (int i=0;i<numV;i++){
		cus=sque[i];
		if (! customer[cus]){
			int whichLocation = -1;// three conditions: if the location is open and has enough free capacity, if all locations are closed.
			if ((int)data->customerToLoc[cus].size() > 0){
				// find the most near depot.
				for (int j = 0; j< (int)data->customerToLoc[cus].size(); j++){
					int location = data->customerToLoc[cus][j] - numV;
					if (!s->location[location].used && whichLocation < 0){
						whichLocation = location;
						break;
					}
					if (s->location[location].used){
						if (s->location[location].remainInven + data->demCus[cus] > s->location[location].inventory)
							continue;
						else{
							whichLocation = location;
							break;
						}
					}
				}
				insertCustomerToLocation(cus, whichLocation, -1);
				customer[cus] = 1;
				int currentCus = cus;
				int targetCus;
				while(1){
					int j;
					for (j=0; j<(int)data->edgeNode[currentCus].size();j++){
						targetCus = data->edgeNode[currentCus][j];
						if (targetCus >= numV) continue;// target the depot
						if (customer[targetCus] == 0){
							if (s->client[currentCus].route->whichLocation->remainInven + data->demCus[targetCus] < s->client[currentCus].route->whichLocation->inventory){
								if (s->client[currentCus].route->load + data->demCus[targetCus] < data->vehCap){
									insertCustomerToLocation(targetCus, whichLocation, currentCus);
									customer[targetCus] = 1;
									currentCus = targetCus;
									break;
								}
								else{
									if (std::find(data->customerToLoc[targetCus].begin(), data->customerToLoc[targetCus].end(), whichLocation+numV) != data->customerToLoc[targetCus].end()){
										insertCustomerToLocation(targetCus, whichLocation, -1);
										customer [targetCus] = 1;
										currentCus = targetCus;
										break;
									}
									else{
										whichLocation =-1;
										for (int k = 0; k< (int)data->customerToLoc[targetCus].size(); k++){
											int location = data->customerToLoc[targetCus][k];
											if (!s->location[location-numV].used ){
												whichLocation = location - numV;
												break;
											}
											else{
												if (s->location[location-numV].remainInven + data->demCus[targetCus] > s->location[location-numV].inventory)
													continue;
												else{
													whichLocation = location - numV;
													break;
												}
											}
										}
										if (whichLocation == -1)break;
										insertCustomerToLocation(targetCus, whichLocation, -1);
										customer [targetCus] = 1;
										currentCus = targetCus;
										break;
									}
								}
							}
						}
					}
					if (whichLocation ==-1 || j == (int)data->edgeNode[currentCus].size())break;
				}
			}
		}
	}
	//to save all remaining unrouted customers
	double moveGain = Max, minGain = Max; // 初始化 moveGain
	Node *nodeV = NULL;
	for (int i=0;i<numV;i++){
		if (customer[i] == 0){
			minGain = Max;
			for (int j=0;j<(int)data->edgeNode[i].size();j++){
				if (data->edgeNode[i][j] >= numV || customer[data->edgeNode[i][j]])continue;
				if (s->client[data->edgeNode[i][j]].route->whichLocation->remainInven + data->demCus[i] > s->client[data->edgeNode[i][j]].route->whichLocation->inventory
						|| s->client[data->edgeNode[i][j]].route->load + data->demCus[i] > data->vehCap) continue;
				if (customer[data->edgeNode[i][j]] != 0){
					if (s->client[data->edgeNode[i][j]].next->isDepot)
						moveGain = data->D[s->client[data->edgeNode[i][j]].next->cour][i] + data->D[i][data->edgeNode[i][j]] - data->D[data->edgeNode[i][j]][s->client[data->edgeNode[i][j]].next->cour];
					else
						moveGain = data->D[data->edgeNode[i][j]][i] + data->D[i][s->client[data->edgeNode[i][j]].next->cour] - data->D[data->edgeNode[i][j]][s->client[data->edgeNode[i][j]].next->cour];
				}
				if (moveGain < minGain){
					minGain = moveGain;
					nodeV = &s->client[data->edgeNode[i][j]];
				}
			}
			Node * nodeU = &s->client[i];
			insertClient(nodeU, nodeV);
			s->updateRouteInfor(nodeU->route->cour,-1);
			s->updateLocationInfor(nodeU->route->whichLocation->cour);
		}
	}
}
void initial_sol::insertClient(Node *U, Node *V){
	if (!V->route->used){// insert into a empty route;
		V->pre->pre = U;// to locate the endDepot
		V->next = U;
		U->pre = V;
		U->next = V->pre;
	}
	else{
		V->next->pre = U;
		U->pre = V;
		U->next = V->next;
		V->next = U;
	}
	U->route = V->route;
	remainDem = remainDem - U->dem;
}
void initial_sol::newGreedyInsert(){
	data->validateDepotSearchSupport();
	s->initilization();

	bool isTotalSame = true;
	for (int i=1;i<numL;i++)
		if (data->invLoc[numV] != data->invLoc[i+numV])
			isTotalSame = false;
	if (isTotalSame && data->invLoc[numV] > data->totalDemand)
		RandomShuffle(data->locRatio);

	remainDem = data->totalDemand;
	for (int i=0; i<numV; i++)
		RandomShuffle(data->edgeNode[i]);

	for (int i=numV; i<numC;i++) locUsed[i] = false;
	// if the inventory of a location can meet all demands, we choose this location in first step;
	int tarLoc = -1;
	for (int i=numV;i<numC;i++){// if there is unlimited capacity location
		if (data->invLoc[i] >= data->totalDemand){
			tarLoc = i;break;
		}
	}
	//// existing perfect compact location configurations
	for (int i=0;i<numV;i++) cusUsed[i] = false;
	double y;int rank;
	if (rankPop > -1 && rankPop < data->numLocConfiguration && (int)data->compactLocConfigure[rankPop].size() > 0){
	//	y=double(rand()%10000/10000.0);
	//	rank=int(pow(y, data->parameters.construction
	//		.depotSelectionRankExponent) *
	//		(int)data->compactLocConfigure.size());
		auto it = data->compactLocConfigure[rankPop].begin();
		std::vector<int>location;
		while (it != data->compactLocConfigure[rankPop].end()){
			location.push_back(*it);
			it ++;
		}
		RandomShuffle(location);
		for (int i=0; i<(int)location.size();i++)
			insertCusToLoc(location[i]);
		int usedInventory = 0;
		for (int i=0;i<numL;i++)if (locUsed[i+numV]) usedInventory += s->location[i].inventory;

		if (usedInventory >= data->totalDemand && remainDem != 0)// the most key method to guarantee the feasible
			remedyTheSolution(location);
	}
	// without the perfect compact location configuration
	/*
	std::uniform_int_distribution<int> dist_3(0, 2);
	int rand_num;
	if (dist_3(global_rng) == 1) rand_num = 1;
	else rand_num = 0;
	*/

	std::vector<int>listLoc1;
	while(remainDem != 0){
		for (int i=0;i<numL;i++)
			if (!locUsed[data->locRatio[i].second] && remainDem == s->location[data->locRatio[i].second - numV].inventory)
				listLoc1.push_back(data->locRatio[i].second);
		if ((int)listLoc1.size() == 0){
			for (int i=0;i<numL;i++)
				if (!locUsed[data->locRatio[i].second])listLoc1.push_back(data->locRatio[i].second);
			if (listLoc1.empty()) {
				throw std::runtime_error(
					"legacy initial solution exhausted all ranked depots "
					"before assigning total customer demand");
			}
			//
	//		if (rand_num)// add the randomness
	//			rank = rand()%(int)listLoc1.size();
	//		else{
				y = RandomDouble(0.0, 1.0);
				rank = int(pow(
					y,
					data->parameters.construction.depotSelectionRankExponent) *
					(int)listLoc1.size());
	//		}
			tarLoc = listLoc1[rank];
			insertCusToLoc(tarLoc);
		}
		else{
			y = RandomDouble(0.0, 1.0);
			rank = int(pow(
				y,
				data->parameters.construction.depotSelectionRankExponent) *
				(int)listLoc1.size());
			tarLoc = listLoc1[rank];
			insertCusToLoc(tarLoc);
		}
		int usedInventory = 0;
		for (int i=0;i<numL;i++)
			if (locUsed[i+numV]) usedInventory += s->location[i].inventory;


		if (usedInventory >= data->totalDemand && remainDem != 0){// the most key method to guarantee the feasible
			std::vector<int>location;
			for (int i=0;i<numL;i++)
				if (s->location[i].used)
					location.push_back(i + numV);
			remedyTheSolution(location);
		}
		listLoc1.clear();
	}
}
void initial_sol::insertCusToLoc(int loc){
	locUsed [loc] = true;
	RandomShuffle(data->edgeLocVec[loc-numV]);
	Node *p, *nodeU, *nodeV, *pos; bool isFirst,consider;
	double delta, min_delta;
	for (int i=0;i<(int)data->edgeLocVec[loc-numV].size();i++){
		p = &s->client[data->edgeLocVec[loc-numV][i]];
		if (! cusUsed[p->cour]){
			min_delta = Max;
			for (int j=0;j<s->location[loc-numV].maxVisit;j++){
				if (s->location[loc-numV].depots[j].route->load + p->dem <= data->vehCap && s->location[loc-numV].remainInven + p->dem <= s->location[loc-numV].inventory){
					nodeU = &s->location[loc-numV].depots[j];
					isFirst = true;
					while(isFirst || !nodeU->isDepot){
						delta = data->D[nodeU->cour][p->cour] + data->D[p->cour][nodeU->next->cour] - data->D[nodeU->cour][nodeU->next->cour];
						if (delta < min_delta){
							min_delta = delta;
							pos = nodeU;
						}
						isFirst = false;
						nodeU = nodeU->next;
					}
				}
			}
			if (min_delta == Max)return;// the customer cannot be inserted into the solution
			insertClient(p, pos);
			s->updateRouteInfor(pos->route->cour,-1);
			s->updateLocationInfor(pos->route->whichLocation->cour);
			cusUsed[p->cour] = true;
			//
			while(1){
				consider = false;
				for (int j=0;j<(int)data->edgeNode[p->cour].size();j++){
					if (data->edgeNode[p->cour][j] >= numV)continue;
					nodeV = &s->client[data->edgeNode[p->cour][j]];
					if (!cusUsed[nodeV->cour]){
						if (p->route->load + nodeV->dem <= data->vehCap && p->route->whichLocation->remainInven + nodeV->dem <= p->route->whichLocation->inventory){
							consider = true;
							insertClient(nodeV, p);
							s->updateRouteInfor(p->route->cour,-1);
							s->updateLocationInfor(p->route->whichLocation->cour);
							cusUsed[nodeV->cour] = true;
							p = nodeV;
							break;
						}
						else if (p->route->whichLocation->remainInven + nodeV->dem <= p->route->whichLocation->inventory){
							consider = true;
							nodeU = &s->location[loc-numV].depots[s->location[loc-numV].splitTime];
							insertClient(nodeV, nodeU);
							s->updateRouteInfor(nodeU->route->cour,-1);
							s->updateLocationInfor(nodeU->route->whichLocation->cour);
							cusUsed[nodeV->cour] = true;
							p = nodeV;
							break;
						}
					}
				}
				if (! consider) break;
			}
		}
	}
	// is there any customer can be inserted into the location
	for (int i=0;i<(int)data->nearCityLoca[loc-numV].size();i++){
		p = &s->client[data->nearCityLoca[loc-numV][i]];
		if (cusUsed[p->cour])continue;
		if (s->location[loc-numV].remainInven + p->dem > s->location[loc-numV].inventory)continue;
		min_delta = Max;
		for (int j=0;j<s->location[loc-numV].maxVisit;j++){
			if (s->location[loc-numV].depots[j].route->load + p->dem <= data->vehCap && s->location[loc-numV].remainInven + p->dem <= s->location[loc-numV].inventory){
				nodeU = &s->location[loc-numV].depots[j];
				isFirst = true;
				while(isFirst || !nodeU->isDepot){
					delta = data->D[nodeU->cour][p->cour] + data->D[p->cour][nodeU->next->cour] - data->D[nodeU->cour][nodeU->next->cour];
					if (delta < min_delta){
						min_delta = delta;
						pos = nodeU;
					}
					isFirst = false;
					nodeU = nodeU->next;
				}
			}
		}
		if (min_delta == Max)return;// the customer cannot be inserted into the solution
		insertClient(p, pos);
		s->updateRouteInfor(pos->route->cour,-1);
		s->updateLocationInfor(pos->route->whichLocation->cour);
		cusUsed[p->cour] = true;
	}
}
void initial_sol::remedyTheSolution(std::vector<int> location){// to use the infeasible framework to restore the solution.
	int remainCap, tarLoc;
	Node *p = NULL, *pos = NULL;
	double delta, min_delta;
	bool isFirst;
	for (int i=0;i<numV;i++){
		if (!cusUsed[i]){// adjust customers in locations and insert the customer i in targeted locations.
			remainCap = 0;
			tarLoc = -1;
			for (int i=0; i<(int)location.size();i++){
				if(std::max(s->location[location[i]-numV].inventory - s->location[location[i]-numV].remainInven, 0) > remainCap){
					remainCap = std::max(s->location[location[i]-numV].inventory - s->location[location[i]-numV].remainInven, 0);
					tarLoc = location[i] - numV;
				}
			}
			if (tarLoc == -1) {
                throw std::runtime_error(
                    "initial solution repair found no eligible depot");
            }
			min_delta = Max;
			for (int j=0; j<s->location[tarLoc].maxVisit;j++){
				p = &s->location[tarLoc].depots[j];
				isFirst = true;
				while(isFirst || !p->isDepot){
					delta = data->D[p->cour][i] + data->D[i][p->next->cour] - data->D[p->cour][p->next->cour];
					if (delta < min_delta){
						min_delta = delta;
						pos = p;
					}
					p = p->next;
					isFirst = false;
				}
			}
			p = &s->client[i];
			insertClient(p, pos);

			s->updateRouteInfor(pos->route->cour,-1);
			s->updateLocationInfor(pos->route->whichLocation->cour);
			cusUsed[p->cour] = true;
		}
	}
	//repair the infeasible solution
	s->evaluationDis();
//	s->outputSolution();
		repair->repairForInitialSolution(s);
//	s->outputSolution();
}
void initial_sol::clearFailedTimeWindowSolution(Individual *target) {
	if (target == nullptr) {
		return;
	}
	target->initilization();
	target->evaluationDis();
	target->decideFeasible();
	// An empty solution can satisfy the legacy capacity-only predicate, but it
	// is not a valid solution of a nonempty CLRP instance.
	target->isFeasible = false;
}

bool initial_sol::reconstructConstraintFeasiblePlan(
		Individual *target,
		const std::vector<int>& customerOrder,
		const int rankPop) {
	return reconstructConstraintFeasiblePlan(
		target,
		customerOrder,
		std::vector<std::vector<int>>{},
		std::vector<bool>{},
		rankPop);
}

bool initial_sol::reconstructConstraintFeasiblePlan(
		Individual *target,
		const std::vector<int>& customerOrder,
		const std::vector<bool>& allowedDepots,
		const int rankPop) {
	return reconstructConstraintFeasiblePlan(
		target,
		customerOrder,
		std::vector<std::vector<int>>{},
		allowedDepots,
		rankPop);
}

bool initial_sol::reconstructConstraintFeasiblePlan(
		Individual *target,
		const std::vector<int>& customerOrder,
		const std::vector<std::vector<int>>& trustedBlocks,
		const int rankPop) {
	return reconstructConstraintFeasiblePlan(
		target,
		customerOrder,
		trustedBlocks,
		std::vector<bool>{},
		rankPop);
}

bool initial_sol::reconstructConstraintFeasiblePlan(
		Individual *target,
		const std::vector<int>& customerOrder,
		const std::vector<std::vector<int>>& trustedBlocks,
		const std::vector<bool>& allowedDepots,
		const int rankPop) {
	hgasp::ScopedComponentTimer timer(
		hgasp::TimedComponent::ConstraintFeasibleReconstruction);
	if (target == nullptr) {
		throw std::invalid_argument(
			"cannot reconstruct a null CLRP individual");
	}

	hgasp::FeasibleInsertionFinder finder(*data);
	hgasp::FeasibleInsertionResult result;
	std::string diagnostic;
	if (!finder.build(
			customerOrder,
			rankPop,
			trustedBlocks,
			allowedDepots,
			result,
			&diagnostic)) {
		clearFailedTimeWindowSolution(target);
		return false;
	}

	// rebuildFromSP is the sole topology commit. Every route column has
	// already passed the active constraint evaluator.
	target->rebuildFromSP(result.routes);
	target->evaluationDis();
	target->decideFeasible();
	if (!target->isFeasible) {
		clearFailedTimeWindowSolution(target);
		return false;
	}
	target->isRight();
	return true;
}

bool initial_sol::initial_solution(Individual *s, int rankPop){
		if (s == nullptr) {
			throw std::invalid_argument(
				"initial solution requires a non-null individual");
		}
		if (data->hasHardTimeWindows()) {
			return initialConstraintAware(s, rankPop);
		}
		return initialCapacityOnly(s, rankPop);
}

bool initial_sol::initialCapacityOnlyExactConfiguration(
		Individual* const target,
		const int populationRank) {
	if (target == nullptr) {
		throw std::invalid_argument(
			"exact fixed-configuration construction requires a non-null "
			"individual");
	}
	if (data->hasHardTimeWindows()) {
		throw std::logic_error(
			"the capacity-only exact constructor cannot serve a hard-TW "
			"profile");
	}
	if (populationRank < 0 ||
		populationRank >= data->numLocConfiguration ||
		static_cast<std::size_t>(populationRank) >=
			data->compactLocConfigure.size()) {
		throw std::out_of_range(
			"exact fixed-configuration population index is out of range");
	}

	std::vector<int> depots;
	for (const int depotNode :
		 data->compactLocConfigure[
			 static_cast<std::size_t>(populationRank)]) {
		const int depot = depotNode - numV;
		if (depot < 0 || depot >= numL) {
			throw std::out_of_range(
				"exact fixed-configuration constructor received an invalid "
				"depot");
		}
		depots.push_back(depot);
	}
	return initialCapacityOnlyExactConfiguration(
		target, hgasp::makeDepotConfigurationKey(std::move(depots)));
}

bool initial_sol::initialCapacityOnlyExactConfiguration(
		Individual* const target,
		const hgasp::DepotConfigurationKey& configuration) {
	if (target == nullptr) {
		throw std::invalid_argument(
			"exact fixed-configuration construction requires a non-null "
			"individual");
	}
	if (data->hasHardTimeWindows()) {
		throw std::logic_error(
			"the capacity-only exact constructor cannot serve a hard-TW "
			"profile");
	}
	const hgasp::DepotConfigurationKey canonical =
		hgasp::makeDepotConfigurationKey(configuration.openDepots);
	if (canonical != configuration) {
		throw std::invalid_argument(
			"exact fixed-configuration constructor requires a canonical "
			"configuration key");
	}
	const std::vector<int>& requiredDepots = canonical.openDepots;
	for (const int depot : requiredDepots) {
		if (depot < 0 || depot >= numL) {
			throw std::out_of_range(
				"exact fixed-configuration constructor received an invalid "
				"depot");
		}
	}
	if (requiredDepots.empty()) {
		throw std::invalid_argument(
			"exact fixed-configuration constructor requires at least one "
			"depot");
	}
	if (requiredDepots.size() > static_cast<std::size_t>(numV)) {
		clearFailedExactConfigurationSolution(target);
		return false;
	}

	long long aggregateCapacity = 0;
	for (const int depot : requiredDepots) {
		aggregateCapacity += data->invLoc[numV + depot];
	}
	if (aggregateCapacity < static_cast<long long>(data->totalDemand)) {
		clearFailedExactConfigurationSolution(target);
		return false;
	}

	RouteEvaluator evaluator(
		*data, data->problemContext().temporalConstraint());
	std::vector<std::vector<int>> customerPreferences(
		requiredDepots.size());
	std::vector<ExactSeedEdge> seedEdges;
	seedEdges.reserve(
		requiredDepots.size() * static_cast<std::size_t>(numV));
	for (std::size_t ordinal = 0U;
		 ordinal < requiredDepots.size();
		 ++ordinal) {
		const int depot = requiredDepots[ordinal];
		const int depotNode = numV + depot;
		for (int customerIndex = 0;
			 customerIndex < numV;
			 ++customerIndex) {
			const int demand = data->demCus[customerIndex];
			// Occupancy is structural, so a zero-demand customer can seed a
			// required depot without weakening either capacity check.
			if (demand < 0 || demand > data->vehCap ||
				demand > data->invLoc[depotNode]) {
				continue;
			}
			seedEdges.push_back({
				data->D[depotNode][customerIndex] +
					data->D[customerIndex][depotNode],
				ordinal,
				customerIndex,
			});
		}
	}
	std::sort(seedEdges.begin(), seedEdges.end(), exactSeedEdgeLess);
	for (const ExactSeedEdge& edge : seedEdges) {
		customerPreferences[edge.depotOrdinal].push_back(edge.customer);
	}
	if (std::any_of(
			customerPreferences.begin(),
			customerPreferences.end(),
			[](const std::vector<int>& values) { return values.empty(); })) {
		clearFailedExactConfigurationSolution(target);
		return false;
	}

	std::vector<int> seedForDepot(requiredDepots.size(), -1);
	std::vector<int> depotForCustomer(
		static_cast<std::size_t>(numV), -1);
	// Take the cheapest disjoint pairs first, then use a bounded augmenting
	// pass so a greedy collision cannot reject an otherwise matchable set.
	for (const ExactSeedEdge& edge : seedEdges) {
		if (seedForDepot[edge.depotOrdinal] < 0 &&
			depotForCustomer[
				static_cast<std::size_t>(edge.customer)] < 0) {
			seedForDepot[edge.depotOrdinal] = edge.customer;
			depotForCustomer[
				static_cast<std::size_t>(edge.customer)] =
				static_cast<int>(edge.depotOrdinal);
		}
	}
	std::function<bool(std::size_t, std::vector<unsigned char>&)> augment;
	augment = [&](const std::size_t depotOrdinal,
				  std::vector<unsigned char>& seenCustomers) {
		for (const int customerIndex :
			 customerPreferences[depotOrdinal]) {
			if (seenCustomers[
					static_cast<std::size_t>(customerIndex)] != 0U) {
				continue;
			}
			seenCustomers[
				static_cast<std::size_t>(customerIndex)] = 1U;
			const int priorDepot = depotForCustomer[
				static_cast<std::size_t>(customerIndex)];
			if (priorDepot < 0 ||
				augment(static_cast<std::size_t>(priorDepot), seenCustomers)) {
				seedForDepot[depotOrdinal] = customerIndex;
				depotForCustomer[
					static_cast<std::size_t>(customerIndex)] =
					static_cast<int>(depotOrdinal);
				return true;
			}
		}
		return false;
	};
	for (std::size_t ordinal = 0U;
		 ordinal < requiredDepots.size();
		 ++ordinal) {
		if (seedForDepot[ordinal] >= 0) {
			continue;
		}
		std::vector<unsigned char> seenCustomers(
			static_cast<std::size_t>(numV), 0U);
		if (!augment(ordinal, seenCustomers)) {
			clearFailedExactConfigurationSolution(target);
			return false;
		}
	}

	std::vector<StandaloneRoute> routes;
	routes.reserve(requiredDepots.size());
	std::vector<int> depotLoad(static_cast<std::size_t>(numL), 0);
	std::vector<int> depotRouteCount(
		static_cast<std::size_t>(numL), 0);
	std::vector<unsigned char> assignedCustomer(
		static_cast<std::size_t>(numV), 0U);
	for (std::size_t ordinal = 0U;
		 ordinal < requiredDepots.size();
		 ++ordinal) {
		const int depot = requiredDepots[ordinal];
		const int customerIndex = seedForDepot[ordinal];
		const RouteEvaluation evaluation =
			evaluator.evaluate(depot, {customerIndex});
		if (!evaluation.feasible()) {
			throw std::logic_error(
				"exact fixed-configuration seed matching produced an "
				"infeasible singleton route");
		}
		StandaloneRoute route;
		route.locId = depot;
		assignRouteEvaluation(
			route, std::vector<int>{customerIndex}, evaluation);
		routes.push_back(std::move(route));
		depotLoad[static_cast<std::size_t>(depot)] += evaluation.load;
		++depotRouteCount[static_cast<std::size_t>(depot)];
		assignedCustomer[
			static_cast<std::size_t>(customerIndex)] = 1U;
	}

	std::vector<int> remainingCustomers;
	remainingCustomers.reserve(static_cast<std::size_t>(numV));
	for (int customerIndex = 0;
		 customerIndex < numV;
		 ++customerIndex) {
		if (assignedCustomer[
				static_cast<std::size_t>(customerIndex)] == 0U) {
			remainingCustomers.push_back(customerIndex);
		}
	}
	RandomShuffle(remainingCustomers);
	std::stable_sort(
		remainingCustomers.begin(),
		remainingCustomers.end(),
		[this](const int left, const int right) {
			return data->demCus[left] > data->demCus[right];
		});

	for (const int customerIndex : remainingCustomers) {
		const int demand = data->demCus[customerIndex];
		std::vector<ExactInsertionCandidate> candidates;
		for (std::size_t routeIndex = 0U;
			 routeIndex < routes.size();
			 ++routeIndex) {
			const StandaloneRoute& route = routes[routeIndex];
			const int depot = route.locId;
			if (depotLoad[static_cast<std::size_t>(depot)] + demand >
				data->invLoc[numV + depot] ||
				route.totalLoad + demand > data->vehCap) {
				continue;
			}
			for (std::size_t position = 0U;
				 position <= route.sequence.size();
				 ++position) {
				const int depotNode = numV + depot;
				const int previous = position == 0U
					? depotNode
					: route.sequence[position - 1U];
				const int next = position == route.sequence.size()
					? depotNode
					: route.sequence[position];
				ExactInsertionCandidate candidate;
				candidate.objectiveDelta =
					data->D[previous][customerIndex] +
					data->D[customerIndex][next] -
					data->D[previous][next];
				candidate.routeIndex = routeIndex;
				candidate.depot = depot;
				candidate.insertionPosition = position;
				candidates.push_back(std::move(candidate));
			}
		}
		for (const int depot : requiredDepots) {
			if (demand > data->vehCap ||
				depotLoad[static_cast<std::size_t>(depot)] + demand >
					data->invLoc[numV + depot] ||
				depotRouteCount[static_cast<std::size_t>(depot)] >=
					Individual::representationRouteLimit(*data, depot)) {
				continue;
			}
			const int depotNode = numV + depot;
			ExactInsertionCandidate candidate;
			candidate.objectiveDelta =
				data->usingCostVeh +
				data->D[depotNode][customerIndex] +
				data->D[customerIndex][depotNode];
			candidate.opensRoute = true;
			candidate.routeIndex = routes.size();
			candidate.depot = depot;
			candidates.push_back(std::move(candidate));
		}
		if (candidates.empty()) {
			clearFailedExactConfigurationSolution(target);
			return false;
		}
		std::sort(
			candidates.begin(),
			candidates.end(),
			exactInsertionCandidateLess);
		// Retain the legacy constructor's bounded diversification without
		// ever admitting a depot outside the requested configuration.
		const std::size_t width = std::min<std::size_t>(
			static_cast<std::size_t>(
				data->parameters.construction
					.exactInsertionCandidateListWidth),
			candidates.size());
		const double biased = std::pow(
			RandomDouble(0.0, 1.0),
			data->parameters.construction.exactInsertionRankExponent);
		const std::size_t selectedIndex = std::min(
			width - 1U,
			static_cast<std::size_t>(
				biased * static_cast<double>(width)));
		ExactInsertionCandidate selected =
			std::move(candidates[selectedIndex]);
		if (selected.opensRoute) {
			std::vector<int> sequence{customerIndex};
			const RouteEvaluation evaluation =
				evaluator.evaluate(selected.depot, sequence);
			if (!evaluation.feasible()) {
				throw std::logic_error(
					"exact fixed-configuration insertion admitted an "
					"infeasible new route");
			}
			StandaloneRoute route;
			route.locId = selected.depot;
			assignRouteEvaluation(
				route,
				std::move(sequence),
				evaluation);
			routes.push_back(std::move(route));
			++depotRouteCount[
				static_cast<std::size_t>(selected.depot)];
		} else {
			if (selected.routeIndex >= routes.size()) {
				throw std::logic_error(
					"exact-configuration insertion route index is out of "
					"range");
			}
			hgasp::requireValidExactInsertion(
				routes.size(),
				selected.routeIndex,
				routes[selected.routeIndex].sequence.size(),
				selected.insertionPosition);
			std::vector<int> sequence =
				routes.at(selected.routeIndex).sequence;
			std::vector<int> extendedSequence;
			extendedSequence.reserve(sequence.size() + 1U);
			for (std::size_t position = 0U;
				 position <= sequence.size(); ++position) {
				if (position == selected.insertionPosition) {
					extendedSequence.push_back(customerIndex);
				}
				if (position < sequence.size()) {
					extendedSequence.push_back(sequence[position]);
				}
			}
			sequence = std::move(extendedSequence);
			const RouteEvaluation evaluation =
				evaluator.evaluate(selected.depot, sequence);
			if (!evaluation.feasible()) {
				throw std::logic_error(
					"exact fixed-configuration insertion admitted an "
					"infeasible route extension");
			}
			assignRouteEvaluation(
				routes.at(selected.routeIndex),
				std::move(sequence),
				evaluation);
		}
		depotLoad[static_cast<std::size_t>(selected.depot)] += demand;
	}

	std::set<int> actualDepots;
	std::vector<int> coverage(static_cast<std::size_t>(numV), 0);
	std::fill(depotLoad.begin(), depotLoad.end(), 0);
	for (StandaloneRoute& route : routes) {
		const RouteEvaluation evaluation =
			evaluator.evaluate(route.locId, route.sequence);
		if (!evaluation.feasible()) {
			throw std::logic_error(
				"exact fixed-configuration constructor produced an "
				"infeasible route");
		}
		assignRouteEvaluation(route, route.sequence, evaluation);
		actualDepots.insert(route.locId);
		depotLoad[static_cast<std::size_t>(route.locId)] +=
			evaluation.load;
		for (const int customerIndex : route.sequence) {
			if (++coverage[
					static_cast<std::size_t>(customerIndex)] != 1) {
				throw std::logic_error(
					"exact fixed-configuration constructor duplicated a "
					"customer");
			}
		}
	}
	const std::set<int> expectedDepots(
		requiredDepots.begin(), requiredDepots.end());
	if (actualDepots != expectedDepots ||
		std::any_of(
			coverage.begin(),
			coverage.end(),
			[](const int count) { return count != 1; })) {
		throw std::logic_error(
			"exact fixed-configuration constructor violated its contract");
	}
	for (const int depot : requiredDepots) {
		if (depotLoad[static_cast<std::size_t>(depot)] >
			data->invLoc[numV + depot]) {
			throw std::logic_error(
				"exact fixed-configuration constructor exceeded depot "
				"capacity");
		}
	}

	// No linked-list mutation occurs before the complete value-level plan has
	// passed independent coverage and capacity validation.
	target->rebuildFromSP(routes);
	target->evaluationDis();
	target->decideFeasible();
	std::set<int> committedDepots;
	for (int depot = 0; depot < numL; ++depot) {
		if (target->location[depot].used) {
			committedDepots.insert(depot);
		}
	}
	if (!target->isFeasible || committedDepots != expectedDepots) {
		throw std::logic_error(
			"exact fixed-configuration constructor violated its contract");
	}
	target->isRight();
	return true;
}

bool initial_sol::initialConstraintAwareExactConfiguration(
		Individual* const target,
		const hgasp::DepotConfigurationKey& configuration) {
	if (target == nullptr) {
		throw std::invalid_argument(
			"exact TW construction requires a non-null individual");
	}
	if (!data->hasHardTimeWindows()) {
		throw std::logic_error(
			"the constraint-aware exact constructor requires a hard-TW "
			"profile");
	}
	const hgasp::DepotConfigurationKey canonical =
		hgasp::makeDepotConfigurationKey(configuration.openDepots);
	if (canonical != configuration || canonical.openDepots.empty()) {
		throw std::invalid_argument(
			"exact TW construction requires a non-empty canonical "
			"configuration key");
	}
	std::vector<bool> allowedDepots(
		static_cast<std::size_t>(numL), false);
	for (const int depot : canonical.openDepots) {
		if (depot < 0 || depot >= numL) {
			throw std::out_of_range(
				"exact TW constructor received an invalid depot");
		}
		allowedDepots[static_cast<std::size_t>(depot)] = true;
	}

	std::vector<int> order(static_cast<std::size_t>(numV));
	std::iota(order.begin(), order.end(), 0);
	const int attempts = std::max(16, std::min(128, 2 * numV));
	for (int attempt = 0; attempt < attempts; ++attempt) {
		if (attempt + 1 == attempts) {
			std::stable_sort(
				order.begin(), order.end(),
				[this](const int lhs, const int rhs) {
					const double lhsDue = data->dueTime[
						static_cast<std::size_t>(lhs)];
					const double rhsDue = data->dueTime[
						static_cast<std::size_t>(rhs)];
					return lhsDue != rhsDue ? lhsDue < rhsDue : lhs < rhs;
				});
		} else {
			RandomShuffle(order);
		}
		if (!reconstructConstraintFeasiblePlan(
				target, order, allowedDepots, attempt)) {
			continue;
		}
		std::vector<int> actualDepots;
		for (int depot = 0; depot < numL; ++depot) {
			if (target->location[depot].used) {
				actualDepots.push_back(depot);
			}
		}
		if (hgasp::makeDepotConfigurationKey(std::move(actualDepots)) ==
			canonical) {
			target->decideFeasible();
			if (!target->isFeasible) {
				throw std::logic_error(
					"exact TW constructor committed an infeasible solution");
			}
			target->isRight();
			return true;
		}
	}
	clearFailedTimeWindowSolution(target);
	return false;
}

bool initial_sol::initialConstraintAware(
		Individual *s,
		const int rankPop) {
	this->s=s;
	this->rankPop = rankPop;
	std::vector<bool> allowedDepots;
	if (rankPop >= 0 && rankPop < data->numLocConfiguration) {
		std::vector<int> depots;
		for (const int depotNode :
			 data->compactLocConfigure[
				 static_cast<std::size_t>(rankPop)]) {
			const int depot = depotNode - numV;
			if (depot < 0 || depot >= numL) {
				throw std::out_of_range(
					"fixed TW constructor received an invalid depot");
			}
			depots.push_back(depot);
		}
		return initialConstraintAwareExactConfiguration(
			s, hgasp::makeDepotConfigurationKey(std::move(depots)));
	}
	std::vector<int> order(static_cast<std::size_t>(numV));
	std::iota(order.begin(), order.end(), 0);

		// Repeated construction is intentionally bounded.  All failed attempts
		// leave an empty Individual, while the final due-time ordering provides
		// a deterministic urgency-first fallback.
	const int attempts =
		std::max(16, std::min(128, 2 * numV));
	for (int attempt = 0; attempt < attempts; ++attempt) {
			if (attempt + 1 == attempts) {
				std::stable_sort(
					order.begin(),
					order.end(),
					[this](const int lhs, const int rhs) {
						const double lhsDue =
							data->dueTime[
								static_cast<std::size_t>(lhs)];
						const double rhsDue =
							data->dueTime[
								static_cast<std::size_t>(rhs)];
						if (lhsDue != rhsDue) {
							return lhsDue < rhsDue;
						}
						return lhs < rhs;
					});
			} else {
				RandomShuffle(order);
			}

			const int diversifiedRank =
				rankPop < 0 ? -1 : rankPop + attempt;
			if (reconstructConstraintFeasiblePlan(
					s, order, allowedDepots, diversifiedRank)) {
				return true;
			}
	}
	clearFailedTimeWindowSolution(s);
	return false;
}

bool initial_sol::initialCapacityOnly(
		Individual *s,
		const int rankPop) {
	this->s = s;
	this->rankPop = rankPop;

//	if ((int)data->compactLocConfigure.size() == 0)
//	greedyInsert();
//	else
	newGreedyInsert();

/*	for (int i=0;i<numL;i++){
		if (s->location[i].used){
			std::cout<<i<<" ";
		}
	}
	std::cout<<std::endl;
*/
	s->evaluationDis();
	s->decideFeasible();
	if (!s->isFeasible)
		return false;
	s->isRight();
	return true;
}
