/******************************************************************************
 * File: Individual.cpp
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description: 
 *   Data structure representing a single solution (individual) in the population.
 ******************************************************************************/

#include "Individual.h"
#include "AlgorithmFingerprint.h"
#include "ComponentProfiler.h"
#include "Version.h"
#include "NumericPolicy.h"
#include "RouteEvaluator.h"
#include "SearchAttemptScheduler.h"
#include "SearchComponents.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

std::vector<int> customerSequence(const Route& route) {
	std::vector<int> sequence;
	const Node* node = route.depot->next;
	while (!node->isDepot) {
		sequence.push_back(node->cour);
		node = node->next;
	}
	return sequence;
}

using StructuralRoute = std::pair<int, std::vector<int>>;

std::vector<StructuralRoute> structuralRoutes(
		const Individual& individual,
		const int numDepots) {
	hgasp::recordHotPathCounter(
		hgasp::HotPathCounter::PopulationStructureKeyBuilds);
	std::vector<StructuralRoute> result;
	for (int depot = 0; depot < numDepots; ++depot) {
		if (individual.location[depot].used) {
			// An empty sequence is a depot-activation marker. It keeps the
			// predicate exact even for a transient individual that opens a
			// depot before attaching a non-empty route to it.
			result.push_back({depot, {}});
		}
		for (int slot = 0;
			 slot < individual.location[depot].maxVisit;
			 ++slot) {
			const Route& route =
				*individual.location[depot].depots[slot].route;
			if (!routeIsOccupied(route))
				continue;
			result.push_back({
				depot,
				customerSequence(route),
			});
		}
	}
	std::sort(result.begin(), result.end());
	return result;
}

RouteEvaluation evaluateRoute(
	const SolverContext& data,
	const int depotIndex,
	const std::vector<int>& sequence,
	const EvaluationDetail detail = EvaluationDetail::Summary) {
	return data.problemContext().evaluateRoute(
		data, depotIndex, sequence, detail);
}

int originalCustomerId(const SolverContext& data, const int customer) {
	if (customer >= 0 &&
		static_cast<std::size_t>(customer) <
			data.originalCustomerIds.size()) {
		return data.originalCustomerIds[static_cast<std::size_t>(customer)];
	}
	return customer;
}

int originalDepotId(const SolverContext& data, const int depot) {
	if (depot >= 0 &&
		static_cast<std::size_t>(depot) <
			data.originalDepotIds.size()) {
		return data.originalDepotIds[static_cast<std::size_t>(depot)];
	}
	return depot;
}

std::string jsonEscape(const std::string& value) {
	std::ostringstream escaped;
	escaped << '"';
	for (const unsigned char character : value) {
		switch (character) {
			case '"':
				escaped << "\\\"";
				break;
			case '\\':
				escaped << "\\\\";
				break;
			case '\b':
				escaped << "\\b";
				break;
			case '\f':
				escaped << "\\f";
				break;
			case '\n':
				escaped << "\\n";
				break;
			case '\r':
				escaped << "\\r";
				break;
			case '\t':
				escaped << "\\t";
				break;
			default:
				if (character < 0x20U) {
					escaped << "\\u"
							<< std::hex << std::setw(4)
							<< std::setfill('0')
							<< static_cast<unsigned int>(character)
							<< std::dec << std::setfill(' ');
				} else {
					escaped << static_cast<char>(character);
				}
		}
	}
	escaped << '"';
	return escaped.str();
}

}  // namespace

Individual::Individual()
	: client(nullptr),
	  location(nullptr),
	  route(nullptr),
	  inventory(0),
	  inventoryVeh(0),
	  dis(0.0),
	  capExcess(0),
	  penalty(0.0),
	  fit(Max),
		  biasedFit(0.0),
		  isFeasible(false),
		  searchIndividualId(0U),
		  sourceTraceId(0U),
		  spAncestryCallId(),
		  maxVnum(0),
	  data(nullptr),
	  numV(0),
	  numL(0),
	  numC(0),
	  checkArray(nullptr),
	  diversityCacheValid(false) {}

Individual::~Individual() {
	delete [] client;
	if (location != nullptr) {
		for (int i=0; i< numL;i++){
			delete [] location[i].depots;
			delete [] location[i].endDepots;
		}
	}
	delete [] location;
	delete [] route;
	delete [] checkArray;
}

int Individual::representationRouteLimit(
		const SolverContext& data,
		const int depotIndex) {
	hgasp::recordHotPathCounter(
		hgasp::HotPathCounter::RouteLimitRequests);
	if (depotIndex < 0 || depotIndex >= data.numL) {
		throw std::out_of_range(
			"representation route limit: depot index is out of range");
	}
	if (data.numV <= 0) {
		throw std::invalid_argument(
			"representation route limit: customer count must be positive");
	}
	if (data.problemDataFinalizedForSearch()) {
		hgasp::recordHotPathCounter(
			hgasp::HotPathCounter::RouteLimitCacheHits);
		return data.cachedRepresentationRouteLimit(depotIndex);
	}
	const int depotCapacity =
		data.invLoc[data.numV + depotIndex];
	if (depotCapacity < 0) {
		throw std::invalid_argument(
			"representation route limit: depot capacity is negative");
	}

	int zeroDemandCustomers = 0;
	int positiveDemandCustomers = 0;
	int minimumPositiveDemand = std::numeric_limits<int>::max();
	hgasp::recordHotPathCounter(
		hgasp::HotPathCounter::RouteLimitFullScans);
	for (int customer = 0; customer < data.numV; ++customer) {
		const int demand = data.demCus[customer];
		if (demand < 0) {
			throw std::invalid_argument(
				"representation route limit: customer demand is negative");
		}
		if (demand == 0) {
			++zeroDemandCustomers;
		} else {
			++positiveDemandCustomers;
			minimumPositiveDemand =
				std::min(minimumPositiveDemand, demand);
		}
	}
	hgasp::recordHotPathCounter(
		hgasp::HotPathCounter::RouteLimitDemandEntriesScanned,
		static_cast<std::uint64_t>(data.numV));

	// Each non-empty route contains either a zero-demand customer, of which
	// there are at most z, or at least one positive-demand customer. Since
	// depot load is bounded by Q_d, at most floor(Q_d/q_min) routes can be of
	// the latter kind. Exact cover also bounds the total by n. This is a safe
	// representation bound for CLRP and CLRP-TW; it does not impose a fleet
	// restriction on the mathematical problem.
	long long safeBound = zeroDemandCustomers;
	if (positiveDemandCustomers > 0) {
		safeBound += std::min<long long>(
			positiveDemandCustomers,
			depotCapacity / minimumPositiveDemand);
	}
	safeBound = std::min<long long>(data.numV, safeBound);

	// Existing route/depot storage assumes at least one slot for every
	// candidate depot, including a depot that cannot serve positive demand.
	return std::max(1, static_cast<int>(safeBound));
}

bool Individual::hasSameRouteStructure(
		const Individual& other) const {
	if (numV != other.numV || numL != other.numL)
		return false;
	ensureStructuralRouteCache();
	other.ensureStructuralRouteCache();
	return structuralRouteCache_ == other.structuralRouteCache_;
}

void Individual::define(SolverContext *data){
	if (data == nullptr) {
		throw std::invalid_argument(
			"cannot define an individual without problem data");
	}
	if (client != nullptr || location != nullptr ||
		route != nullptr || checkArray != nullptr) {
		throw std::logic_error(
			"an individual cannot be defined more than once");
	}
	this->data=data;
	this->numV=data->numV;
	this->numL = data->numL;
	this->numC = data->numC;
	diversityCache.resize(numV);
	diversityCacheValid = false;
	structuralRouteCache_.clear();
	structuralRouteCacheValid_ = false;
	activeRouteScratch_.reserve(static_cast<std::size_t>(numV));
	validationCustomerUseScratch_.resize(
		static_cast<std::size_t>(numV));
	validationRouteSequenceScratch_.reserve(
		static_cast<std::size_t>(numV));
	client=new Node [numV]{};
	for (int i=0;i<numV;i++){
		client[i].isDepot=false;
		client[i].cour=i;
		client[i].pre=NULL;
		client[i].next=NULL;
		client[i].route=NULL;
		client[i].dem=data->demCus[i];
		client[i].position=0;
		client[i].cumulatedLoad=0;
		client[i].cumulatedDis=0;
		client[i].whenLastTestedRI=-1;
	}
	location = new NodeDepot [numL]{};
	maxVnum=0;
	for (int i=0; i<numL; i++){
		location[i].used = false;
		location[i].cour = i+numV;
		location[i].inventory = data->invLoc[i+numV];
		location[i].remainInven = 0;
		location[i].splitTime = 0;
		if (location[i].inventory > data->totalDemand) location[i].iscapCon = false;// to decide the constraint in locations
		else location[i].iscapCon = true;
		location[i].maxVisit = representationRouteLimit(*data, i);
		maxVnum += location[i].maxVisit;
		location[i].depots =
			new Node [location[i].maxVisit]{};
		location[i].endDepots =
			new Node [location[i].maxVisit]{};
		for (int j=0; j<location[i].maxVisit;j++){
			location[i].depots[j].isDepot=true;
			location[i].depots[j].cour = i+numV;
			location[i].depots[j].position = 0;
			location[i].depots[j].dem = 0;
			location[i].depots[j].cumulatedLoad = 0;
			location[i].depots[j].cumulatedDis = 0;
			location[i].depots[j].pre = &location[i].endDepots[j];
			location[i].depots[j].next = &location[i].endDepots[j];
			location[i].endDepots[j].isDepot=true;
			location[i].endDepots[j].cour = i+numV;
			location[i].endDepots[j].position = 0;
			location[i].endDepots[j].dem = 0;
			location[i].endDepots[j].cumulatedLoad = 0;
			location[i].endDepots[j].cumulatedDis = 0;
			location[i].endDepots[j].pre = &location[i].depots[j];
			location[i].endDepots[j].next = &location[i].depots[j];
		}
	}
//	data->maxVnum = maxVnum;
	route=new Route [maxVnum]{};// the maxVnum is the maximum vehicles;
	for (int i=0; i<maxVnum; i++){
		route[i].used=false;
		route[i].cour=i;
		route[i].whenLastModified=-1;
		route[i].dis=0;
		route[i].load=0;
		route[i].nbClients=0;
		route[i].returnTime=0;
		route[i].waitingTime=0;
		route[i].temporalFeasible=true;
		route[i].temporalViolationCustomer=-1;
		route[i].penalty=0;
	}
	int rankRoute=0;
	for (int i=0;i<numL;i++){
		for (int j=0;j<location[i].maxVisit; j++){
			location[i].depots[j].route = &route[rankRoute];
			location[i].endDepots[j].route = &route[rankRoute];
			route[rankRoute].depot = &location[i].depots[j];
			route[rankRoute].endDepot = &location[i].endDepots[j];
			route[rankRoute].whichLocation = & location[i];
			location[i].emptyRoutes.insert(rankRoute);
			rankRoute++;
		}
	}
	checkArray = new int [numV]{};
}

void Individual::resetForPopulationCopy() {
	if (activeRouteScratchOccupancyGeneration_ == 0U ||
		activeRouteScratchOccupancyGeneration_ !=
			routeOccupancyGeneration_) {
		rebuildActiveRouteIndex(false);
	}
	const std::vector<int> occupiedRoutes = activeRouteScratch_;

	searchIndividualId = 0U;
	sourceTraceId = 0U;
	spAncestryCallId.clear();
	invalidateTopologyState();
	invalidateActiveRouteState();
	saturatLoc.clear();
	saturatRoute.clear();
	for (int customer = 0; customer < numV; ++customer) {
		client[customer].pre = nullptr;
		client[customer].next = nullptr;
		client[customer].route = nullptr;
		client[customer].position = 0;
		client[customer].cumulatedLoad = 0;
		client[customer].cumulatedDis = 0.0;
		client[customer].whenLastTestedRI = -1;
	}
	for (int depot = 0; depot < numL; ++depot) {
		location[depot].used = false;
		location[depot].inventory = data->invLoc[depot + numV];
		location[depot].remainInven = 0;
		location[depot].splitTime = 0;
	}
	for (const int routeIndex : occupiedRoutes) {
		Route& currentRoute = route[routeIndex];
		Node& depot = *currentRoute.depot;
		Node& endDepot = *currentRoute.endDepot;
		depot.pre = &endDepot;
		depot.next = &endDepot;
		depot.position = 0;
		depot.cumulatedLoad = 0;
		depot.cumulatedDis = 0.0;
		endDepot.pre = &depot;
		endDepot.next = &depot;
		endDepot.position = 0;
		endDepot.cumulatedLoad = 0;
		endDepot.cumulatedDis = 0.0;
		currentRoute.used = false;
		currentRoute.whenLastModified = -1;
		currentRoute.dis = 0.0;
		currentRoute.load = 0;
		currentRoute.nbClients = 0;
		currentRoute.returnTime = 0.0;
		currentRoute.waitingTime = 0.0;
		currentRoute.temporalFeasible = true;
		currentRoute.temporalViolationCustomer = -1;
		currentRoute.penalty = 0.0;
		currentRoute.whichLocation->emptyRoutes.insert(routeIndex);
	}

#ifndef NDEBUG
	for (int routeIndex = 0; routeIndex < maxVnum; ++routeIndex) {
		if (std::binary_search(
				occupiedRoutes.begin(), occupiedRoutes.end(), routeIndex)) {
			continue;
		}
		const Route& currentRoute = route[routeIndex];
		if (currentRoute.used || routeIsOccupied(currentRoute) ||
			currentRoute.depot->next != currentRoute.endDepot ||
			currentRoute.endDepot->pre != currentRoute.depot) {
			throw std::logic_error(
				"population copy reset found untracked route state");
		}
	}
#endif
}

void Individual::initilization(){
	searchIndividualId = 0U;
	sourceTraceId = 0U;
	spAncestryCallId.clear();
	invalidateTopologyState();
	invalidateActiveRouteState();
	saturatLoc.clear();
	saturatRoute.clear();
	for (int i=0;i<numV;i++){
		client[i].pre=NULL;
		client[i].next=NULL;
		client[i].route=NULL;
		client[i].position=0;
		client[i].cumulatedLoad=0;
		client[i].cumulatedDis=0;
		client[i].whenLastTestedRI=-1;
	}
	for (int i=0; i<numL; i++){
		location[i].used = false;
		location[i].inventory = data->invLoc[i+numV];
		location[i].remainInven = 0;
		location[i].splitTime = 0;
		for (int j=0; j<location[i].maxVisit;j++){
			location[i].depots[j].cour = i+numV;
			location[i].depots[j].dem = 0;
			location[i].depots[j].cumulatedLoad = 0;
			location[i].depots[j].cumulatedDis = 0;
			location[i].depots[j].pre = &location[i].endDepots[j];
			location[i].depots[j].next = &location[i].endDepots[j];
			location[i].endDepots[j].isDepot=true;
			location[i].endDepots[j].cour = i+numV;
			location[i].endDepots[j].position = 0;
			location[i].endDepots[j].dem = 0;
			location[i].endDepots[j].cumulatedLoad = 0;
			location[i].endDepots[j].cumulatedDis = 0;
			location[i].endDepots[j].pre = &location[i].depots[j];
			location[i].endDepots[j].next = &location[i].depots[j];
		}
	}
	for (int i=0; i<maxVnum; i++){
		route[i].used=false;
		route[i].cour=i;
		route[i].whenLastModified=-1;
		route[i].dis=0;
		route[i].load=0;
		route[i].nbClients=0;
		route[i].returnTime=0;
		route[i].waitingTime=0;
		route[i].temporalFeasible=true;
		route[i].temporalViolationCustomer=-1;
		route[i].penalty=0;
	}
	for (int i=numV;i<numC;i++){
		location[i-numV].emptyRoutes.clear();
		for (int j=0;j<location[i-numV].maxVisit;j++){
			int routee = location[i-numV].depots[j].route->cour;
			location[i-numV].emptyRoutes.insert(routee);
		}
	}
}
void Individual::evaluationDis(){
	double totalDis=0;
	double totalLoc=0;
	double totalVeh=0;
	Node *p;
	for (int i=numV; i < numC ;i++){
		if (location[i - numV].used){
			totalLoc += data->openCostLoc[location[i - numV].cour];
			totalVeh += data->usingCostVeh * location[i - numV].splitTime;
			for (int j=0; j < location[i - numV].maxVisit; j++){
				if (location[i-numV].depots[j].route->used){
					p = &location[i - numV].depots[j];
					do {
						totalDis += data->D[p->cour][p->next->cour];
						p = p->next;
					} while (!p->isDepot);
				}
			}
		}
	}

	fit = totalLoc + totalVeh + totalDis;
	dis = totalDis;

	data->probFixedCost =
		std::abs(fit) > Min ? totalLoc / fit : 0.0;
}
void Individual::getSaturationDegree(){
	saturatLoc.clear();
	saturatRoute.clear();
	for (int i=0;i<numL;i++){
		if (location[i].used){
			if (!data->isConstraintLoc)/////////////////////////the other cases and considered in other places
				saturatLoc.insert({double(location[i].remainInven) / double(location[i].inventory), i});
			for (int j=0;j<location[i].maxVisit;j++){
				if (location[i].depots[j].route->used){
					saturatRoute.insert({double(location[i].depots[j].route->load) / double(data->vehCap), location[i].depots[j].route->cour});
				}
			}
		}
	}
}


void Individual::updateRouteInfor(int &r, int nbMoves){
	invalidateTopologyState();
	Node *mynode=route[r].depot;
	mynode->cumulatedLoad=0;
	mynode->position=0;
	mynode->cumulatedDis = 0;
	do {
		Node* previous = mynode;
		mynode = mynode->next;
		mynode->position = previous->position + 1;
		mynode->cumulatedLoad =
			mynode->dem + previous->cumulatedLoad;
		mynode->cumulatedDis =
			data->D[previous->cour][mynode->cour] +
			previous->cumulatedDis;
	} while (!mynode->isDepot);
	route[r].dis=mynode->cumulatedDis;
	route[r].load=mynode->cumulatedLoad;
	route[r].nbClients=std::max(0, mynode->position - 1);
	if (data->hasHardTimeWindows()) {
		const int depotIndex =
			route[r].whichLocation->cour - numV;
		const RouteEvaluation evaluation = evaluateRoute(
			*data, depotIndex, customerSequence(route[r]));
		route[r].load = evaluation.load;
		route[r].dis = evaluation.travelCost;
		route[r].returnTime = evaluation.returnTime;
		route[r].waitingTime = evaluation.totalWaitingTime;
		route[r].temporalFeasible = evaluation.temporalFeasible;
		route[r].temporalViolationCustomer =
			evaluation.violatingCustomer;
	} else {
		route[r].returnTime = 0.0;
		route[r].waitingTime = 0.0;
		route[r].temporalFeasible = true;
		route[r].temporalViolationCustomer = -1;
	}

//		NodeDepot *loc = route[r].whichLocation;// update the location inventory
//		loc->remainInven=0;
//		for (int i=0; i<loc->splitTime;i++)
//			loc->remainInven += loc->depots[i].route->load;
	route[r].whenLastModified = nbMoves;
	const bool occupied = routeIsOccupied(route[r]);
	updateActiveRouteMembership(r, occupied);
	if (!occupied) {
		route[r].penalty = 0.0;
		route[r].whichLocation->emptyRoutes.insert(route[r].cour);
	} else {
		route[r].whichLocation->emptyRoutes.erase(route[r].cour);
	}
}
void Individual:: updateLocationInfor(int &ll){
	invalidateTopologyState();
	int l = ll - numV;
	location[l].splitTime = 0;
	location[l].remainInven = 0;
	for (int i=0; i<location[l].maxVisit; i++){
		Route& currentRoute = *location[l].depots[i].route;
		if (routeIsOccupied(currentRoute)){
			location[l].remainInven += currentRoute.load;
			currentRoute.used = true;
			location[l].splitTime++;
		}
		else currentRoute.used = false;
	}
//	if (location[l].remainInven > location[l].inventory)std::cout<<"report a wrong "
	if (location[l].splitTime == 0){
		location[l].used = false;
	}
	else location[l].used = true;
}

void Individual::refreshActiveRouteInformation() {
	(void)refreshActiveRouteInformationAndCheckCapacity();
}

Individual::ActiveRouteRefreshSummary
Individual::refreshActiveRouteInformationAndCheckCapacity() {
	ActiveRouteRefreshSummary summary;
	if (activeRouteScratchOccupancyGeneration_ == 0U ||
		activeRouteScratchOccupancyGeneration_ !=
			routeOccupancyGeneration_) {
		rebuildActiveRouteIndex(true);
	}
	std::vector<int>& activeRoutes = activeRouteScratch_;

	for (int depot = 0; depot < numL; ++depot) {
		location[depot].used = false;
		location[depot].remainInven = 0;
		location[depot].splitTime = 0;
	}
	double totalTravel = 0.0;
	for (const int routeIndex : activeRoutes) {
		int mutableRouteIndex = routeIndex;
		updateRouteInfor(mutableRouteIndex, -1);
		Route& currentRoute = route[routeIndex];
		if (!routeIsOccupied(currentRoute)) {
			throw std::logic_error(
				"route refresh found an empty customer route");
		}
		currentRoute.used = true;
		NodeDepot& currentLocation = *currentRoute.whichLocation;
		currentLocation.used = true;
		currentLocation.remainInven += currentRoute.load;
		++currentLocation.splitTime;
		totalTravel += currentRoute.dis;
		if (currentRoute.load > data->vehCap) {
			summary.vehicleCapacityFeasible = false;
		}
	}
	double totalOpeningCost = 0.0;
	double totalVehicleCost = 0.0;
	for (int depot = 0; depot < numL; ++depot) {
		if (!location[depot].used) continue;
		if (location[depot].remainInven > location[depot].inventory) {
			summary.depotCapacityFeasible = false;
		}
		totalOpeningCost += data->openCostLoc[numV + depot];
		totalVehicleCost +=
			data->usingCostVeh * location[depot].splitTime;
	}
	dis = totalTravel;
	fit = totalTravel + totalOpeningCost + totalVehicleCost;
	data->probFixedCost =
		std::abs(fit) > Min
			? totalOpeningCost / fit
			: 0.0;
#ifndef NDEBUG
	for (int routeIndex = 0; routeIndex < maxVnum; ++routeIndex) {
		const bool active = std::binary_search(
			activeRoutes.begin(), activeRoutes.end(), routeIndex);
		const Route& currentRoute = route[routeIndex];
		if (active != currentRoute.used ||
			active != routeIsOccupied(currentRoute)) {
			throw std::logic_error(
				"route refresh found untracked route occupancy");
		}
	}
#endif
	return summary;
}

const std::vector<int>& Individual::activeRouteIndices() const {
	hgasp::recordHotPathCounter(
		hgasp::HotPathCounter::ActiveRouteViewRequests);
	if (activeRouteScratchOccupancyGeneration_ == 0U ||
		activeRouteScratchOccupancyGeneration_ !=
			routeOccupancyGeneration_) {
		rebuildActiveRouteIndex(true);
	}
#ifndef NDEBUG
	std::vector<int> expected;
	expected.reserve(static_cast<std::size_t>(numV));
	for (int customer = 0; customer < numV; ++customer) {
		const Route* const customerRoute = client[customer].route;
		if (customerRoute == nullptr || customerRoute->cour < 0 ||
			customerRoute->cour >= maxVnum) {
			throw std::logic_error(
				"active-route occupancy oracle found invalid ownership");
		}
		expected.push_back(customerRoute->cour);
	}
	std::sort(expected.begin(), expected.end());
	expected.erase(
		std::unique(expected.begin(), expected.end()), expected.end());
	if (expected != activeRouteScratch_) {
		throw std::logic_error(
			"active-route occupancy cache is stale");
	}
#endif
	return activeRouteScratch_;
}

void Individual::rebuildActiveRouteIndex(
		const bool requireCompleteAssignment) const {
	hgasp::recordHotPathCounter(
		hgasp::HotPathCounter::ActiveRouteViewRebuilds);
	hgasp::recordHotPathCounter(
		hgasp::HotPathCounter::ActiveRouteViewCustomersScanned,
		static_cast<std::uint64_t>(numV));
	activeRouteScratch_.clear();
	activeRouteScratch_.reserve(static_cast<std::size_t>(numV));
	for (int customer = 0; customer < numV; ++customer) {
		const Route* const customerRoute = client[customer].route;
		if (customerRoute == nullptr) {
			if (requireCompleteAssignment) {
				throw std::logic_error(
					"active-route view found an unassigned customer");
			}
			continue;
		}
		if (customerRoute->cour < 0 || customerRoute->cour >= maxVnum) {
			throw std::logic_error(
				"active-route view found an invalid route index");
		}
		activeRouteScratch_.push_back(customerRoute->cour);
	}
	std::sort(activeRouteScratch_.begin(), activeRouteScratch_.end());
	activeRouteScratch_.erase(
		std::unique(
			activeRouteScratch_.begin(), activeRouteScratch_.end()),
		activeRouteScratch_.end());
	activeRouteScratchOccupancyGeneration_ =
		routeOccupancyGeneration_;
}

void Individual::outputSolution(){
	std::cout<<"------------------------------------The solution display is as follows-------------------------------------------"<<std::endl;
	Node *p;
	for (int loc=numV; loc < numC; loc++){
		if (location[loc - numV].used){
			std::cout<<"Loc "<<loc-numV<<" inventory =  "<<location[loc-numV].inventory<<"  used: "<<location[loc-numV].remainInven<<std::endl;
			for (int i=0;i<location[loc - numV].maxVisit;i++){
				if (!location[loc - numV].depots[i].route->used)continue;
				std::cout<<"Loc "<< location[loc - numV].cour<<" and route ";
				std::cout<< location[loc - numV].depots[i].route->cour<<" (cap "<<location[loc - numV].depots[i].route->load<<"): ";
				p = &location[loc - numV].depots[i];
				p=p->next;
				while(! p->next->isDepot){
					std::cout<<p->cour<<" ";
					p=p->next;
				}
				std::cout<<p->cour<<std::endl;
			}
		}
	}
}
void Individual::isRight(){// check the solution
	double totalDis=0,totalFit=0;
	for (int i=0;i<numV;i++)
		checkArray[i] = 0;
	for (int loc=numV; loc < numC; loc++){
		NodeDepot& depot = location[loc - numV];
		int capLoc = 0;
		int routeCount = 0;
		if (depot.used){
			totalFit += data->openCostLoc[loc];
		}
		for (int i=0;i<depot.maxVisit;i++){
			Route& currentRoute = *depot.depots[i].route;
			const std::vector<int> sequence =
				customerSequence(currentRoute);
			if (sequence.empty()) {
				if (currentRoute.used || routeIsOccupied(currentRoute) ||
					currentRoute.load != 0) {
					throw std::runtime_error(
						"individual validation failed: empty route state");
				}
				continue;
			}
			if (!depot.used || !currentRoute.used ||
				!routeIsOccupied(currentRoute)) {
				throw std::runtime_error(
					"individual validation failed: nonempty route state");
			}

			++routeCount;
			const RouteEvaluation evaluation = evaluateRoute(
				*data, loc - numV, sequence);
			for (const int customer : sequence) {
				checkArray[customer]++;
			}
			if (evaluation.load != currentRoute.load ||
				!evaluation.capacityFeasible) {
				throw std::runtime_error(
					"individual validation failed: route capacity");
			}
			if (data->hasHardTimeWindows()) {
				if (!evaluation.temporalFeasible) {
					throw std::runtime_error(
						"individual validation failed: route time window");
				}
				if (!currentRoute.temporalFeasible ||
					!data->equalTwoVar(
						evaluation.returnTime,
						currentRoute.returnTime) ||
					!data->equalTwoVar(
						evaluation.totalWaitingTime,
						currentRoute.waitingTime)) {
					throw std::runtime_error(
						"individual validation failed: temporal aggregate "
						"mismatch");
				}
			}
			capLoc += evaluation.load;
			totalDis += evaluation.travelCost;
		}
		if (depot.used) {
			totalFit += data->usingCostVeh * depot.splitTime;
			if (routeCount != depot.splitTime ||
				capLoc != depot.remainInven ||
				capLoc > depot.inventory) {
				throw std::runtime_error(
					"individual validation failed: depot capacity");
			}
		} else if (capLoc != 0 || depot.remainInven != 0) {
			throw std::runtime_error(
				"individual validation failed: unused depot state");
		}
	}
	for (int i=0;i<numV;i++)
		if (checkArray[i] != 1){
            throw std::runtime_error(
                "individual validation failed: customer coverage");
		}
	if (! data->equalTwoVar(totalDis, dis)){
        throw std::runtime_error(
            "individual validation failed: travel cost mismatch");
	}
	if (! data->equalTwoVar(totalFit+ totalDis, fit)){
        throw std::runtime_error(
            "individual validation failed: objective mismatch");
	}
}
bool Individual::hasCurrentFeasibilityCertificate() const noexcept {
	if (data == nullptr) {
		return false;
	}
	const std::uint64_t problemRevision =
		data->problemDataRevisionForSearch();
	return problemRevision != 0U &&
		validatedProblemRevision_ == problemRevision &&
		validatedTopologyGeneration_ == topologyGeneration_;
}

void Individual::stampFeasibilityCertificate(
		const bool feasible) noexcept {
	const std::uint64_t problemRevision =
		data == nullptr ? 0U : data->problemDataRevisionForSearch();
	if (problemRevision == 0U) {
		validatedTopologyGeneration_ = 0U;
		validatedProblemRevision_ = 0U;
		validatedFeasibility_ = false;
		return;
	}
	validatedTopologyGeneration_ = topologyGeneration_;
	validatedProblemRevision_ = problemRevision;
	validatedFeasibility_ = feasible;
}

bool Individual::copyFeasibilityCertificateFrom(
		const Individual& source) noexcept {
	if (data == nullptr || source.data != data ||
		!source.hasCurrentFeasibilityCertificate()) {
		validatedTopologyGeneration_ = 0U;
		validatedProblemRevision_ = 0U;
		validatedFeasibility_ = false;
		return false;
	}
	validatedTopologyGeneration_ = topologyGeneration_;
	validatedProblemRevision_ = data->problemDataRevisionForSearch();
	validatedFeasibility_ = source.validatedFeasibility_;
	isFeasible = validatedFeasibility_;
	return true;
}

void Individual::decideFeasible() {
	validateFeasibility(true);
}

void Individual::decideFeasibleAtTrustBoundary() {
	validateFeasibility(false);
}

Individual::PopulationValidationReceipt
Individual::validateForPopulationAdmission() {
	validateFeasibility(false);
	return PopulationValidationReceipt(
		this,
		data,
		data == nullptr
			? 0U
			: data->problemDataRevisionForSearch(),
		topologyGeneration_,
		dis,
		fit,
		penalty,
		capExcess,
		inventory,
		inventoryVeh,
		isFeasible);
}

Individual::PopulationValidationReceipt
Individual::normalizeAndValidateForPopulationAdmission() {
	if (data == nullptr || data->hasHardTimeWindows() ||
		!hgasp::isSetSProductionSearchProfile(
			data->parameters.profile)) {
		throw std::logic_error(
			"deferred objective normalization requires a role-aware Set S profile");
	}
	validateFeasibility(false, true);
	return PopulationValidationReceipt(
		this,
		data,
		data->problemDataRevisionForSearch(),
		topologyGeneration_,
		dis,
		fit,
		penalty,
		capExcess,
		inventory,
		inventoryVeh,
		isFeasible);
}

bool Individual::matchesPopulationValidationReceipt(
		const PopulationValidationReceipt& receipt) const noexcept {
	if (data == nullptr || receipt.individual_ != this ||
		receipt.problem_ != data || receipt.problemRevision_ == 0U ||
		receipt.problemRevision_ != data->problemDataRevisionForSearch() ||
		receipt.topologyGeneration_ != topologyGeneration_ ||
		!hasCurrentFeasibilityCertificate()) {
		return false;
	}
	return receipt.distance_ == dis &&
		receipt.objective_ == fit &&
		receipt.penalty_ == penalty &&
		receipt.capacityExcess_ == capExcess &&
		receipt.carriedInventory_ == inventory &&
		receipt.vehicleInventory_ == inventoryVeh &&
		receipt.feasible_ == isFeasible &&
		receipt.feasible_ == validatedFeasibility_;
}

void Individual::validateFeasibility(
		const bool allowCachedCertificate,
		const bool normalizeObjective) {
	if (allowCachedCertificate && normalizeObjective) {
		throw std::invalid_argument(
			"objective normalization requires an uncached validation");
	}
	if (allowCachedCertificate &&
		hasCurrentFeasibilityCertificate()) {
		hgasp::recordHotPathCounter(
			hgasp::HotPathCounter::PopulationValidationCacheHits);
		isFeasible = validatedFeasibility_;
		return;
	}
	hgasp::recordHotPathCounter(
		hgasp::HotPathCounter::PopulationValidationCalls);
	const auto finish = [this](const bool feasible) {
		isFeasible = feasible;
		stampFeasibilityCertificate(feasible);
	};
	std::fill(
		validationCustomerUseScratch_.begin(),
		validationCustomerUseScratch_.end(),
		0);
	std::vector<int>& customerUse = validationCustomerUseScratch_;
	std::vector<int>& sequence = validationRouteSequenceScratch_;
	double computedTravel = 0.0;
	double computedFixedCost = 0.0;
	double computedOpeningCost = 0.0;
	for (int i = 0; i < numL; ++i) {
		int computedDepotLoad = 0;
		int computedRouteCount = 0;
		for (int j = 0; j < location[i].maxVisit; ++j) {
			Route& currentRoute = *location[i].depots[j].route;
			sequence.clear();
			const Node* routeNode = currentRoute.depot->next;
			while (!routeNode->isDepot) {
				sequence.push_back(routeNode->cour);
				routeNode = routeNode->next;
			}
			if (sequence.empty()) {
				if (currentRoute.used || routeIsOccupied(currentRoute) ||
					currentRoute.load != 0) {
					finish(false);
					return;
				}
				continue;
			}
			if (!location[i].used || !currentRoute.used ||
				!routeIsOccupied(currentRoute)) {
				finish(false);
				return;
			}
			++computedRouteCount;
			const RouteEvaluation evaluation = evaluateRoute(
				*data, i, sequence);
			if (!evaluation.feasible() ||
				evaluation.load != currentRoute.load ||
				!data->equalTwoVar(
					evaluation.travelCost,
					currentRoute.dis)) {
				finish(false);
				return;
			}
			for (const int customer : sequence) {
				if (++customerUse[
						static_cast<std::size_t>(customer)] != 1) {
					finish(false);
					return;
				}
			}
			computedDepotLoad += evaluation.load;
			if (!allowCachedCertificate) {
				computedTravel += evaluation.travelCost;
			}
		}
		const bool shouldBeUsed = computedRouteCount != 0;
		if (location[i].used != shouldBeUsed ||
			location[i].splitTime != computedRouteCount ||
			computedDepotLoad > location[i].inventory ||
			computedDepotLoad != location[i].remainInven) {
			finish(false);
			return;
		}
		if (!allowCachedCertificate && shouldBeUsed) {
			computedOpeningCost += data->openCostLoc[numV + i];
			computedFixedCost += data->openCostLoc[numV + i];
			computedFixedCost +=
				data->usingCostVeh * computedRouteCount;
		}
	}
	for (const int useCount : customerUse) {
		if (useCount != 1) {
			finish(false);
			return;
		}
	}
	if (!allowCachedCertificate) {
		const double computedObjective =
			computedTravel + computedFixedCost;
		if (normalizeObjective) {
			dis = computedTravel;
			fit = computedObjective;
			data->probFixedCost =
				std::abs(fit) > Min ? computedOpeningCost / fit : 0.0;
		}
		if (!data->equalTwoVar(computedTravel, dis)) {
			throw IndividualObjectiveConsistencyError(
				"individual objective consistency failed: stored travel "
				"cost does not match the active evaluator");
		}
		if (!data->equalTwoVar(computedObjective, fit)) {
			throw IndividualObjectiveConsistencyError(
				"individual objective consistency failed: stored total "
				"objective does not match travel, depot, and vehicle costs");
		}
	}
	finish(true);
}
void Individual::outputBestSolution(){
	std::cout<<"----------Write Solution With Value  "<<fit<<" In: "<<data->pathSolution<<std::endl;
	decideFeasible();
	if (!isFeasible) {
		throw std::runtime_error(
			"solution output failed: solution is infeasible");
	}
	if (!data->searchTermination.internallyConsistent()) {
		throw std::logic_error(
			"solution output requires a consistent termination record");
	}
	isRight();

	double depotOpeningCost = 0.0;
	double vehicleFixedCost = 0.0;
	double travelCost = 0.0;
	for (int depotIndex = 0; depotIndex < numL; ++depotIndex) {
		if (!location[depotIndex].used) {
			continue;
		}
		depotOpeningCost +=
			data->openCostLoc[numV + depotIndex];
		vehicleFixedCost +=
			data->usingCostVeh * location[depotIndex].splitTime;
		for (int slot = 0;
			 slot < location[depotIndex].maxVisit;
			 ++slot) {
			const Route& currentRoute =
				*location[depotIndex].depots[slot].route;
			if (currentRoute.used) {
				travelCost += evaluateRoute(
					*data,
					depotIndex,
					customerSequence(currentRoute)).travelCost;
			}
		}
	}
	const double actualCost =
		depotOpeningCost + vehicleFixedCost + travelCost;
	const double reportedCost = hgasp::roundObjectiveForReporting(
		actualCost, data->problemProfile().reporting);
	const int reportingDecimals =
		hgasp::objectiveReportingDecimals(
			data->problemProfile().reporting);
	const double searchCpuSeconds = data->elapsedCpuSeconds();
	std::ostringstream rngState;
	rngState << global_rng;
	hgasp::AlgorithmFingerprint algorithmFingerprint =
		hgasp::runtimeAlgorithmFingerprint();
	const hgasp::ComponentActivationSummary componentActivation =
		hgasp::makeComponentActivationSummary(
			data->hasHardTimeWindows(),
			hgasp::makeV114ComponentActivationConfiguration(
				data->staticHgampPopulationsEnabled(),
				data->hasHardTimeWindows(),
				data->strictCrossConfigurationMating,
				data->populationLocalSpEnabled,
				data->preReductionGlobalSpEnabled));
	const auto effective = [&](const std::string& name) {
		const hgasp::ComponentActivation* const component =
			hgasp::findComponentActivation(componentActivation, name);
		return component != nullptr && component->effective;
	};
	algorithmFingerprint.hardTwRouteLocalSearch =
		effective("route_local_search.hard_tw");

	// Preserve the historical human-readable output for existing workflows.
	std::ofstream myfile(data->pathSolution);
	if (myfile.is_open()){
			myfile<<"The objective is: "<<std::endl;
			myfile << std::fixed << std::setprecision(reportingDecimals)
				   << reportedCost << std::endl;
			myfile << std::defaultfloat << std::setprecision(17);
			myfile<<"The raw objective is: "<<std::endl;
			myfile<<actualCost<<std::endl;
			myfile<<"The running time is: "<<std::endl;
			myfile<<searchCpuSeconds<<std::endl;
			myfile<<"The time meeting best is: "<<std::endl;
			myfile<<data->meetBestTime<<std::endl;
			myfile<<"The time basis is: "<<std::endl;
			myfile<<"process_cpu"<<std::endl;
			myfile<<"The time scope is: "<<std::endl;
			myfile<<"search_after_instance_loading"<<std::endl;
		Node *p;
		for (int loc=numV; loc < numC; loc++){
			if (location[loc - numV].used){
				int count=0;
				myfile<<"Location: "<<loc-numV<<"(load: "<<location[loc-numV].remainInven<<") "<<std::endl;
				for (int i=0;i<location[loc - numV].maxVisit;i++){
					if (!location[loc - numV].depots[i].route->used)continue;
					myfile<<" Route "<<count<<"(load: "<<location[loc - numV].depots[i].route->load<<") ";
					count++;
					p = &location[loc - numV].depots[i];
				//	myfile<<p->cour<<" ";
					p=p->next;
					while(! p->next->isDepot){
						myfile<<p->cour<<" ";
						p=p->next;
					}
				//	myfile<<p->cour<<" "<<p->next->cour<<std::endl;
					myfile<<p->cour<<" "<<std::endl;
				}
			}
		}
		myfile << "The actual cost is " << std::fixed
			   << std::setprecision(reportingDecimals)
			   << reportedCost << std::endl;
	}
	else std::cout << "----- IMPOSSIBLE TO OPEN: " << data->pathSolution << std::endl;
	myfile.close();

	// The machine-readable solution is a fixed-name sibling of solution.txt,
	// so experiment tooling can validate every problem profile uniformly.
	const std::filesystem::path textPath(data->pathSolution);
	const std::filesystem::path jsonPath =
		textPath.parent_path() / "solution.json";
	std::ofstream json(jsonPath);
	if (!json) {
		throw std::runtime_error(
			"solution output failed: cannot open " + jsonPath.string());
	}
	json << std::setprecision(17);
	json << "{\n";
	json << "  \"schema_version\": 2,\n";
	json << "  \"solver_version\": "
		 << jsonEscape(hgasp::kSolverVersion) << ",\n";
	json << "  \"algorithm_revision_id\": "
		 << jsonEscape(hgasp::kAlgorithmRevisionId) << ",\n";
	json << "  \"git_commit\": "
		 << jsonEscape(algorithmFingerprint.gitCommit) << ",\n";
	json << "  \"git_dirty\": "
		 << (algorithmFingerprint.gitDirty ? "true" : "false") << ",\n";
	json << "  \"feature_flags\": {\n";
	json << "    \"route_local_search.hard_tw\": "
		 << (algorithmFingerprint.hardTwRouteLocalSearch ? "true" : "false")
		 << "\n";
	json << "  },\n";
	json << "  \"component_activation\": {\n";
	for (std::size_t index = 0U;
		 index < componentActivation.components.size();
		 ++index) {
		const hgasp::ComponentActivation& component =
			componentActivation.components[index];
		json << "    " << jsonEscape(component.name) << ": {"
			 << "\"configured\":"
			 << (component.configured ? "true" : "false") << ","
			 << "\"effective\":"
			 << (component.effective ? "true" : "false") << ","
			 << "\"activation_count\":"
			 << component.activationCount << ","
			 << "\"disabled_reason\":";
		if (component.disabledReason.empty()) {
			json << "null";
		} else {
			json << jsonEscape(component.disabledReason);
		}
		json << "}"
			 << (index + 1U == componentActivation.components.size()
				 ? "\n"
				 : ",\n");
	}
	json << "  },\n";
	json << "  \"resolved_config_sha256\": "
		 << jsonEscape(algorithmFingerprint.resolvedConfigSha256) << ",\n";
	json << "  \"instance_sha256\": "
		 << jsonEscape(algorithmFingerprint.instanceSha256) << ",\n";
	json << "  \"search_policy_id\": "
		 << jsonEscape(hgasp::kSearchPolicyId) << ",\n";
	json << "  \"problem_family\": "
		 << jsonEscape(data->problemProfile().family) << ",\n";
	json << "  \"benchmark_family\": "
		 << jsonEscape(hgasp::toString(
			data->problemProfile().benchmarkFamily)) << ",\n";
	json << "  \"active_constraints\": [";
	const std::vector<std::string> activeConstraints =
		hgasp::activeConstraintIds(data->problemProfile());
	for (std::size_t index = 0;
		 index < activeConstraints.size();
		 ++index) {
		if (index != 0U) {
			json << ", ";
		}
		json << jsonEscape(activeConstraints[index]);
	}
	json << "],\n";
	json << "  \"instance_format\": "
		 << jsonEscape(hgasp::toString(data->instanceFormat)) << ",\n";
	json << "  \"metric_policy_id\": "
		 << jsonEscape(
			data->problemProfile().reporting.metricPolicyId) << ",\n";
	json << "  \"reporting_policy_id\": "
		 << jsonEscape(
			data->problemProfile().reporting.reportingPolicyId) << ",\n";
	json << "  \"eax_parent_mode\": "
		 << jsonEscape(hgasp::toString(data->eaxParentMode)) << ",\n";
	json << "  \"search_parameter_profile\": "
		 << jsonEscape(hgasp::toString(data->parameters.profile)) << ",\n";
	const bool hgamp2ParameterCompatible =
		data->parameters.profile ==
			hgasp::SearchParameterProfile::Hgamp2ParameterCompatibleSetS;
	const bool promisingArcProfile =
		hgasp::isSetSProductionSearchProfile(data->parameters.profile);
	const bool roleAwareProfile =
		hgasp::usesRoleAwareCheckpointSearch(data->parameters.profile);
	json << "  \"search_candidate_topology\": {\n";
	json << "    \"pool_layout\": "
		 << jsonEscape(
			hgamp2ParameterCompatible
				? "hgamp2-shared-ranking-plus-depot-pool"
				: (promisingArcProfile
					? "promising-arc-separate-customer-depot"
					: "separate-customer-depot"))
		 << ",\n";
	json << "    \"customer_rule\": "
		 << jsonEscape(
			hgamp2ParameterCompatible
				? "customer-origin-all-vertex-symmetric-alpha-union-filter-depots"
				: (promisingArcProfile
					? "vidal-directed-correlation"
					: "directed-fixed-width"))
		 << ",\n";
	json << "    \"depot_rule\": "
		 << jsonEscape(
			hgamp2ParameterCompatible
				? "nearest-half-plus-one"
				: (promisingArcProfile
					? "nearest-bounded-separate"
					: "nearest-fixed-width"))
		 << "\n";
	json << "  },\n";
	if (roleAwareProfile) {
		json << "  \"algorithm_semantics\": {\n";
		json << "    \"promising_arc_move_filter\": "
			 << jsonEscape(
				promisingArcProfile
					? "at-least-one-new-directed-promising-arc"
					: "disabled-v22-hard-tw-preservation")
			 << ",\n";
		json << "    \"temporal_penalty_weight\": "
			 << data->parameters.localSearch.temporalPenaltyWeight << ",\n";
		json << "    \"waiting_time_weight\": "
			 << data->parameters.localSearch.waitingTimeWeight << ",\n";
		json << "    \"mutation_policy\": "
			 << jsonEscape(
				data->parameters.mutation.policy ==
					hgasp::MutationPolicy::ShawOnly
					? "shaw-only"
					: "alternating")
			 << ",\n";
		json << "    \"sp_lp_policy\": "
			 << jsonEscape(
				"dual-feedback-at-50-and-feedback-gated-75-exploration")
			 << ",\n";
		json << "    \"route_pool_harvest_policy\": "
			 << jsonEscape(
				hgasp::isSetSProductionSearchProfile(
					data->parameters.profile)
					? "rolling-mean-with-exact-master-preservation"
					: "all-feasible") << ",\n";
		json << "    \"population_local_route_pool\": "
			 << jsonEscape("none")
			 << ",\n";
		json << "    \"local_sp_source\": "
			 << jsonEscape(
				"central-master-ephemeral-configuration-view") << ",\n";
		json << "    \"configuration_portfolio\": "
			 << jsonEscape(
				"ranked-checkpoint-rotation-with-permanent-free")
			 << ",\n";
		json << "    \"global_sp_schedule\": "
			 << jsonEscape(
				"configuration-aware-early-pre-reduction-barriers-and-final-phase")
			 << ",\n";
		json << "    \"global_sp_extension\": "
			 << jsonEscape(
				"budgeted-mip-and-feedback-gated-dual-exploration")
			 << ",\n";
		json << "    \"global_sp_effort_policy\": "
			 << jsonEscape(
				"canonical-stage-effort-with-early-sixfold-extension-and-family-specific-final")
			 << ",\n";
		json << "    \"exploratory_active_view_policy\": "
			 << jsonEscape("canonical-bounded-active-column-target")
			 << ",\n";
		json << "    \"global_sp_budget_scaling\": "
			 << jsonEscape("bounded-linear-n-over-500-floor-0.2-cap-1.0")
			 << ",\n";
		json << "    \"local_route_pool_capacity_policy\": "
			 << jsonEscape("none")
			 << ",\n";
		json << "    \"primary_population_roles\": "
			 << jsonEscape(
				"fixed-residents-role-preserving-ls-and-permanent-free")
			 << "\n";
		json << "  },\n";
	}
	json << "  \"search_parameters\": {\n";
	json << "    \"construction_correlation_alpha\": "
		 << data->alpha << ",\n";
	json << "    \"local_search_customer_alpha\": "
		 << data->localSearchCustomerAlpha << ",\n";
	json << "    \"local_search_depot_beta\": "
			 << data->localSearchDepotBeta << ",\n";
	json << "    \"local_search_candidate_shuffle_denominator\": "
			 << data->parameters.localSearch.candidateShuffleDenominator
			 << ",\n";
	json << "    \"construction_insertion_candidate_list_width\": "
			 << data->parameters.construction.insertionCandidateListWidth
			 << ",\n";
	json << "    \"construction_insertion_rank_exponent\": "
			 << data->parameters.construction.insertionRankExponent << ",\n";
	json << "    \"construction_exact_insertion_candidate_list_width\": "
			 << data->parameters.construction
				.exactInsertionCandidateListWidth << ",\n";
	json << "    \"construction_exact_insertion_rank_exponent\": "
			 << data->parameters.construction.exactInsertionRankExponent
			 << ",\n";
	json << "    \"construction_depot_selection_rank_exponent\": "
			 << data->parameters.construction.depotSelectionRankExponent
			 << ",\n";
	json << "    \"population_minimum_size\": "
		 << data->popMin << ",\n";
	json << "    \"population_offspring_count\": "
		 << data->parameters.population.offspringCount << ",\n";
	json << "    \"population_diversity_nearest_count\": "
		 << data->nbClost << ",\n";
	json << "    \"population_elite_count\": "
		 << data->nbElite << ",\n";
	json << "    \"static_fixed_population_count\": "
		 << data->staticFixedPopulationCount << ",\n";
	json << "    \"static_population_minimum_age_attempts\": "
		 << data->staticPopulationMinimumAgeAttempts << ",\n";
	json << "    \"population_resident_post_promotion_protection_attempts\": "
		 << data->residentPostPromotionProtectionAttempts << ",\n";
	json << "    \"population_resident_protection_attempts\": "
		 << data->popProtectionPeriod << ",\n";
	json << "    \"population_maximum_iterations_per_configuration\": "
			 << data->maxIterationPerLoc << ",\n";
	json << "    \"population_initial_candidate_multiplier\": "
			 << data->initialPopulationCandidateMultiplier << ",\n";
	json << "    \"population_incubation_minimum_construction_attempts\": "
			 << data->populationIncubationMinimumConstructionAttempts
			 << ",\n";
	json << "    \"population_incubation_target_construction_multiplier\": "
			 << data->populationIncubationTargetConstructionMultiplier
			 << ",\n";
	json << "    \"population_build_no_progress_minimum_attempts\": "
			 << data->populationBuildNoProgressMinimumAttempts << ",\n";
	json << "    \"population_build_no_progress_target_multiplier\": "
			 << data->populationBuildNoProgressTargetMultiplier << ",\n";
	json << "    \"population_promotion_construction_multiplier\": "
			 << data->populationPromotionConstructionMultiplier << ",\n";
	json << "    \"configuration_screening_cheap_limit\": "
		 << data->configurationScreeningCheapLimit << ",\n";
	json << "    \"configuration_screening_first_stage\": "
		 << data->configurationScreeningFirstStage << ",\n";
	json << "    \"configuration_screening_finalists\": "
		 << data->configurationScreeningFinalists << ",\n";
	json << "    \"configuration_evaluation_first_samples\": "
		 << data->configurationEvaluationFirstSamples << ",\n";
	json << "    \"configuration_evaluation_total_samples\": "
		 << data->configurationEvaluationTotalSamples << ",\n";
	json << "    \"cross_configuration_selection_max_attempts\": "
			 << data->crossConfigurationSelectionMaxAttempts << ",\n";
	json << "    \"configuration_expansion_multiplier\": "
			 << data->configurationExpansionMultiplier << ",\n";
	json << "    \"configuration_depot_selection_rank_exponent\": "
			 << data->parameters.configurationPortfolio
				.depotSelectionRankExponent << ",\n";
	json << "    \"population_phase_horizon_attempts\": "
		 << data->populationPhaseHorizonAttempts << ",\n";
	json << "    \"pre_reduction_global_1_fraction\": "
		 << data->preReductionGlobal1Fraction << ",\n";
	json << "    \"population_reduction_1_fraction\": "
		 << data->populationReduction1Fraction << ",\n";
	json << "    \"population_reduction_1_survivor_count\": "
		 << data->populationReduction1SurvivorCount << ",\n";
	json << "    \"pre_reduction_global_2_fraction\": "
		 << data->preReductionGlobal2Fraction << ",\n";
	json << "    \"population_reduction_2_fraction\": "
		 << data->populationReduction2Fraction << ",\n";
	json << "    \"population_reduction_2_survivor_count\": "
		 << data->populationReduction2SurvivorCount << ",\n";
	json << "    \"population_absorption_fraction\": "
		 << data->populationAbsorptionFraction << ",\n";
	json << "    \"population_reduction_objective_weight\": "
		 << data->populationReductionObjectiveWeight << ",\n";
	json << "    \"population_reduction_hga_improvement_weight\": "
		 << data->populationReductionHgaImprovementWeight << ",\n";
	json << "    \"population_reduction_diversity_weight\": "
		 << data->populationReductionDiversityWeight << ",\n";
	json << "    \"population_reduction_global_sp_credit_weight\": "
		 << data->populationReductionGlobalSpCreditWeight << ",\n";
	json << "    \"retired_configuration_cooldown_attempts\": "
		 << data->retiredConfigurationCooldownAttempts << ",\n";
	json << "    \"mutation_probability_percent\": "
		 << data->mutationProbabilityPercent << ",\n";
	json << "    \"mutation_ruin_fraction\": "
		 << data->ratioPerturbLength << ",\n";
	json << "    \"mutation_shaw_rank_exponent\": "
			 << data->determine_para << ",\n";
	json << "    \"mutation_insertion_candidate_list_width\": "
			 << data->parameters.mutation.insertionCandidateListWidth
			 << ",\n";
	json << "    \"mutation_insertion_rank_exponent\": "
			 << data->parameters.mutation.insertionRankExponent << ",\n";
	json << "    \"mutation_recreate_attempts\": "
			 << data->parameters.mutation.recreateAttempts << ",\n";
	json << "    \"restart_stagnation_attempts\": "
		 << data->maxIterNonProd << ",\n";
	json << "    \"restart_minimum_remaining_attempts\": "
		 << data->restartMinimumRemainingIterations << ",\n";
	json << "    \"restart_minimum_remaining_cpu_seconds\": "
		 << data->restartMinimumRemainingCpuSeconds << ",\n";
	json << "    \"challenger_selection_minimum_attempt\": "
		 << data->parameters.challenger.selectionMinimumAttempt << ",\n";
	json << "    \"challenger_selection_interval_attempts\": "
		 << data->parameters.challenger.selectionIntervalAttempts << ",\n";
	json << "    \"challenger_rejection_cooldown_attempts\": "
		 << data->parameters.challenger.rejectionCooldownAttempts << ",\n";
	json << "    \"challenger_clrp_trial_attempts\": "
		 << data->parameters.challenger.clrpTrialAttempts << ",\n";
	json << "    \"challenger_tw_trial_attempts\": "
		 << data->parameters.challenger.twTrialAttempts << ",\n";
	json << "    \"challenger_protected_primary_period\": "
		 << data->parameters.challenger.protectedPrimaryPeriod << ",\n";
	json << "    \"challenger_clrp_latest_launch_fraction\": "
		 << data->parameters.challenger.clrpLatestLaunchFraction << ",\n";
	json << "    \"challenger_elite_mean_count\": "
		 << data->parameters.challenger.eliteMeanCount << ",\n";
	json << "    \"challenger_target_minimum_size\": "
		 << data->parameters.challenger.targetMinimumSize << ",\n";
	json << "    \"challenger_target_maximum_size\": "
		 << data->parameters.challenger.targetMaximumSize << ",\n";
	json << "    \"challenger_target_population_divisor\": "
		 << data->parameters.challenger.targetPopulationDivisor << ",\n";
	json << "    \"challenger_maximum_construction_multiplier\": "
		 << data->parameters.challenger.maximumConstructionMultiplier
		 << ",\n";
	json << "    \"challenger_minimum_viable_numerator\": "
		 << data->parameters.challenger.minimumViableNumerator << ",\n";
	json << "    \"challenger_minimum_viable_denominator\": "
			 << data->parameters.challenger.minimumViableDenominator << ",\n";
	json << "    \"challenger_archive_capacity\": "
			 << data->parameters.challenger.archiveCapacity << ",\n";
	json << "    \"challenger_archive_top_objective_count\": "
			 << data->parameters.challenger.archiveTopObjectiveCount << ",\n";
	json << "    \"challenger_archive_seed_count\": "
			 << data->parameters.challenger.archiveSeedCount << ",\n";
	json << "    \"challenger_repeated_discovery_threshold\": "
			 << data->parameters.challenger.repeatedDiscoveryThreshold
			 << ",\n";
	json << "    \"challenger_new_observations_after_rejection\": "
			 << data->parameters.challenger.newObservationsAfterRejection
			 << ",\n";
	json << "    \"sp_global_trigger_frequency\": "
		 << data->spTriggerFrequency << ",\n";
	json << "    \"sp_local_trigger_frequency\": "
		 << data->spLocalTriggerFrequency << ",\n";
	json << "    \"sp_adaptive_minimum_interval\": "
		 << data->spAdaptiveMinimumInterval << ",\n";
	json << "    \"sp_cross_scope_local_minimum_interval\": "
		 << data->spCrossScopeLocalMinimumInterval << ",\n";
	json << "    \"sp_adaptive_local_new_columns\": "
		 << data->spAdaptiveLocalNewColumns << ",\n";
	json << "    \"sp_adaptive_local_stagnation\": "
		 << data->spAdaptiveLocalStagnation << ",\n";
	json << "    \"sp_adaptive_global_stagnation\": "
		 << data->spAdaptiveGlobalStagnation << ",\n";
	json << "    \"sp_adaptive_utility_ema_weight\": "
		 << data->spAdaptiveUtilityEmaWeight << ",\n";
	json << "    \"sp_adaptive_high_utility_threshold\": "
			 << data->spAdaptiveHighUtilityThreshold << ",\n";
	json << "    \"sp_early_global_fraction\": "
			 << data->spEarlyGlobalFraction << ",\n";
	json << "    \"sp_final_phase_start_fraction\": "
			 << data->spFinalPhaseStartFraction << ",\n";
	json << "    \"sp_global_minimum_interval\": "
		 << data->spGlobalMinimumInterval << ",\n";
	json << "    \"sp_local_zero_utility_patience\": "
		 << data->spLocalZeroUtilityPatience << ",\n";
	json << "    \"sp_global_zero_utility_patience\": "
		 << data->spGlobalZeroUtilityPatience << ",\n";
	json << "    \"population_local_sp_stagnation_attempts\": "
		 << data->populationLocalSpStagnationAttempts << ",\n";
	json << "    \"population_local_sp_minimum_age_attempts\": "
		 << data->populationLocalSpMinimumAgeAttempts << ",\n";
	json << "    \"population_local_sp_minimum_new_customer_sets\": "
		 << data->populationLocalSpMinimumNewCustomerSets << ",\n";
	json << "    \"population_local_sp_minimum_replacement_gain_ratio\": "
		 << data->populationLocalSpMinimumReplacementGainRatio << ",\n";
	json << "    \"population_local_sp_lp_time_limit_seconds\": "
		 << data->populationLocalSpLpTimeLimitSeconds << ",\n";
	json << "    \"population_local_sp_mip_time_limit_seconds\": "
		 << data->populationLocalSpMipTimeLimitSeconds << ",\n";
	json << "    \"population_local_sp_maximum_calls\": "
		 << data->populationLocalSpMaximumCalls << ",\n";
	json << "    \"population_local_sp_family_minimum_interval\": "
		 << data->populationLocalSpFamilyMinimumInterval << ",\n";
	json << "    \"population_local_sp_family_cpu_share_limit\": "
		 << data->populationLocalSpFamilyCpuShareLimit << ",\n";
	json << "    \"population_local_sp_injection_count\": "
		 << data->populationLocalSpInjectionCount << ",\n";
	json << "    \"pre_reduction_global_sp_lp_time_limit_seconds\": "
		 << data->preReductionGlobalSpLpTimeLimitSeconds << ",\n";
	json << "    \"pre_reduction_global_sp_mip_time_limit_seconds\": "
		 << data->preReductionGlobalSpMipTimeLimitSeconds << ",\n";
	json << "    \"pre_reduction_global_sp_injection_count\": "
		 << data->preReductionGlobalSpInjectionCount << ",\n";
	json << "    \"sp_exploratory_active_column_target\": "
		 << data->exploratoryActiveColumnTarget << ",\n";
	json << "    \"sp_global_outer_time_limit_seconds\": "
		 << data->spGlobalOuterTimeLimit << ",\n";
	json << "    \"sp_final_global_outer_time_limit_seconds\": "
		 << data->spFinalGlobalOuterTimeLimit << ",\n";
	json << "    \"sp_local_outer_time_limit_seconds\": "
		 << data->spLocalOuterTimeLimit << ",\n";
	json << "    \"sp_global_lp_time_limit_seconds\": "
		 << data->spGlobalLpTimeLimit << ",\n";
	json << "    \"sp_local_lp_time_limit_seconds\": "
		 << data->spLocalLpTimeLimit << ",\n";
	json << "    \"sp_online_global_mip_time_limit_seconds\": "
		 << data->spOnlineGlobalMipTimeLimit << ",\n";
	json << "    \"sp_online_local_mip_time_limit_seconds\": "
		 << data->spOnlineLocalMipTimeLimit << ",\n";
	json << "    \"sp_final_global_lp_time_limit_seconds\": "
		 << data->spFinalGlobalLpTimeLimit << ",\n";
	json << "    \"sp_final_global_mip_time_limit_seconds\": "
		 << data->spFinalGlobalMipTimeLimit << ",\n";
	json << "    \"sp_final_global_primal_idle_time_limit_seconds\": "
		 << data->spFinalGlobalPrimalIdleTimeLimit << ",\n";
	json << "    \"sp_tw_final_global_mip_time_limit_seconds\": "
		 << data->spTwFinalMipTimeLimit << ",\n";
	json << "    \"sp_online_global_injection_count\": "
		 << data->spOnlineGlobalInjectionCount << ",\n";
	json << "    \"sp_online_local_injection_count\": "
		 << data->spOnlineLocalInjectionCount << ",\n";
	json << "    \"sp_final_global_injection_count\": "
		 << data->spFinalGlobalInjectionCount << ",\n";
	json << "    \"sp_family_cpu_share_target\": "
		 << data->spCpuShareTarget << ",\n";
	json << "    \"sp_lp_probe_cpu_share_target\": "
		 << data->spLpProbeCpuShareTarget << ",\n";
	json << "    \"sp_local_mip_cpu_share_target\": "
		 << data->spLocalMipCpuShareTarget << ",\n";
	json << "    \"sp_global_mip_cpu_share_target\": "
		 << data->spGlobalMipCpuShareTarget << ",\n";
	json << "    \"sp_global_cpu_reserve_target\": "
		 << data->spGlobalCpuReserveTarget << ",\n";
	json << "    \"sp_mip_rescue_reserve_target\": "
		 << data->spMipRescueReserveTarget << ",\n";
	json << "    \"sp_global_solution_pool_size\": "
		 << data->spGlobalSolutionPoolSize << ",\n";
	json << "    \"sp_local_solution_pool_size\": "
		 << data->spLocalSolutionPoolSize << ",\n";
	json << "    \"sp_global_solution_pool_gap\": "
		 << data->spGlobalSolutionPoolGap << ",\n";
	json << "    \"sp_local_solution_pool_gap\": "
		 << data->spLocalSolutionPoolGap << ",\n";
	json << "    \"sp_global_solution_pool_search_mode\": "
		 << data->spGlobalSolutionPoolSearchMode << ",\n";
	json << "    \"sp_local_solution_pool_search_mode\": "
		 << data->spLocalSolutionPoolSearchMode << ",\n";
	json << "    \"route_pool_capacity\": "
		 << data->spMaxPoolSize << ",\n";
	json << "    \"sp_mip_focus\": "
		 << data->spMIPFocus << ",\n";
	json << "    \"sp_heuristics\": "
		 << data->spHeuristics << "\n";
	json << "  },\n";
	json << "  \"rng_state\": "
		 << jsonEscape(rngState.str()) << ",\n";
	json << "  \"termination\": {\n";
	json << "    \"requested_iterations\": "
		 << data->searchTermination.requestedIterations << ",\n";
	json << "    \"completed_iterations\": "
		 << data->searchTermination.completedIterations << ",\n";
	json << "    \"termination_reason\": "
		 << jsonEscape(std::string(hgasp::toString(
			data->searchTermination.reason))) << ",\n";
	json << "    \"early_stop_evidence_class\": "
		 << jsonEscape(std::string(hgasp::toString(
			data->searchTermination.evidenceClass))) << ",\n";
	json << "    \"lp_rigorous_no_improvement\": "
		 << (data->searchTermination.lpRigorousNoImprovement
			 ? "true"
			 : "false") << ",\n";
	json << "    \"mip_attempted\": "
		 << (data->searchTermination.mipAttempted ? "true" : "false")
		 << ",\n";
	json << "    \"mip_improved\": "
		 << (data->searchTermination.mipImproved ? "true" : "false")
		 << "\n";
	json << "  },\n";
	json << "  \"search_control\": {\n";
	json << "    \"restart\": {\n";
	json << "      \"stagnation_threshold\": "
		 << data->maxIterNonProd << ",\n";
	json << "      \"minimum_remaining_iterations\": "
		 << data->restartMinimumRemainingIterations << ",\n";
	json << "      \"minimum_remaining_cpu_seconds\": "
		 << data->restartMinimumRemainingCpuSeconds << ",\n";
	json << "      \"iteration_limit\": "
		 << data->iterLimit << ",\n";
	const bool restartIterationReachable =
		SearchAttemptScheduler::isRestartIterationReachable(
				data->maxIterNonProd,
				data->restartMinimumRemainingIterations,
				data->iterLimit);
	const long long restartReachabilityLeft =
		static_cast<long long>(data->maxIterNonProd) + 1LL +
		static_cast<long long>(data->restartMinimumRemainingIterations);
	json << "      \"iteration_reachable\": "
		 << (restartIterationReachable ? "true" : "false") << ",\n";
	json << "      \"iteration_reachability_inequality\": {\n";
	json << "        \"left_expression\": "
		 << jsonEscape(
			"stagnation_threshold + 1 + minimum_remaining_iterations")
		 << ",\n";
	json << "        \"left_value\": "
		 << restartReachabilityLeft << ",\n";
	json << "        \"operator\": \"<=\",\n";
	json << "        \"right_expression\": \"iteration_limit\",\n";
	json << "        \"right_value\": " << data->iterLimit << ",\n";
	json << "        \"satisfied\": "
		 << (restartIterationReachable ? "true" : "false") << ",\n";
	json << "        \"blocking_reason\": "
		 << jsonEscape(
			restartIterationReachable
				? "none"
				: "left_exceeds_iteration_limit") << "\n";
	json << "      }\n";
	json << "    }\n";
	json << "  },\n";
	json << "  \"instance_id\": "
		 << jsonEscape(
				std::filesystem::path(data->pathInstance).stem().string())
		 << ",\n";
	json << "  \"feasible\": true,\n";
	json << "  \"timing\": {\n";
	json << "    \"search_cpu_seconds\": " << searchCpuSeconds << ",\n";
	json << "    \"global_best_cpu_seconds\": "
		 << data->meetBestTime << ",\n";
	json << "    \"basis\": \"process_cpu\",\n";
	json << "    \"scope\": \"search_after_instance_loading\"\n";
	json << "  },\n";
	json << "  \"objective\": {\n";
	json << "    \"total\": " << std::fixed
		 << std::setprecision(reportingDecimals)
		 << reportedCost << ",\n";
	json << "    \"reported_total\": "
		 << reportedCost << ",\n";
	json << std::defaultfloat << std::setprecision(17);
	json << "    \"raw_total\": " << actualCost << ",\n";
	json << "    \"reporting_decimals\": " << reportingDecimals << ",\n";
	json << "    \"rounding\": "
		 << jsonEscape(hgasp::objectiveReportingRounding()) << ",\n";
	json << "    \"depot_opening\": " << depotOpeningCost << ",\n";
	json << "    \"vehicle_fixed\": " << vehicleFixedCost << ",\n";
	json << "    \"travel\": " << travelCost << "\n";
	json << "  },\n";
	json << "  \"depots\": [";

	bool firstDepot = true;
	for (int depotIndex = 0; depotIndex < numL; ++depotIndex) {
		if (!location[depotIndex].used) {
			continue;
		}
		json << (firstDepot ? "\n" : ",\n");
		firstDepot = false;
		json << "    {\n";
		json << "      \"depot_id\": "
			 << originalDepotId(*data, depotIndex) << ",\n";
		json << "      \"load\": "
			 << location[depotIndex].remainInven << ",\n";
		json << "      \"routes\": [";

		bool firstRoute = true;
		for (int slot = 0;
			 slot < location[depotIndex].maxVisit;
			 ++slot) {
			const Route& currentRoute =
				*location[depotIndex].depots[slot].route;
			if (!currentRoute.used) {
				continue;
			}
			const std::vector<int> sequence =
				customerSequence(currentRoute);
			const RouteEvaluation evaluation = evaluateRoute(
				*data,
				depotIndex,
				sequence,
				data->hasHardTimeWindows()
					? EvaluationDetail::FullSchedule
					: EvaluationDetail::Summary);

			json << (firstRoute ? "\n" : ",\n");
			firstRoute = false;
			json << "        {\n";
			json << "          \"load\": " << evaluation.load << ",\n";
			json << "          \"travel_cost\": "
				 << evaluation.travelCost << ",\n";
			json << "          \"return_time\": "
				 << evaluation.returnTime << ",\n";
			json << "          \"customers\": [";
			for (std::size_t position = 0;
				 position < sequence.size();
				 ++position) {
				if (position != 0) {
					json << ", ";
				}
				json << originalCustomerId(
					*data, sequence[position]);
			}
			json << "]";

			if (data->hasHardTimeWindows()) {
				json << ",\n          \"schedule\": [";
				for (std::size_t position = 0;
					 position < evaluation.schedule.size();
					 ++position) {
					if (position != 0) {
						json << ", ";
					}
					const CustomerSchedule& visit =
						evaluation.schedule[position];
					json << "{\"customer_id\": "
						 << originalCustomerId(*data, visit.customer)
						 << ", \"arrival_time\": "
						 << visit.arrivalTime
						 << ", \"waiting_time\": "
						 << visit.waitingTime
						 << ", \"service_start_time\": "
						 << visit.serviceStartTime
						 << ", \"departure_time\": "
						 << visit.departureTime << "}";
				}
				json << "]\n";
			} else {
				json << "\n";
			}
			json << "        }";
		}
		json << (firstRoute ? "]\n" : "\n      ]\n");
		json << "    }";
	}
	json << (firstDepot ? "]\n" : "\n  ]\n");
	json << "}\n";
	if (!json) {
		throw std::runtime_error(
			"solution output failed while writing " +
			jsonPath.string());
	}
}
void Individual::removeProximity(Individual * indiv){
    // 防御性编程：必须检查迭代器边界
    for (auto it = indivsPerProximity.begin(); it != indivsPerProximity.end(); ++it) {
        if (it->second == indiv) {
            indivsPerProximity.erase(it);
            break; // 找到后安全退出
        }
    }
}
double Individual::averageBrokenPairsDistanceClosest(int nbClosest){
	double result = 0 ;
	int maxSize = std::min<int>(nbClosest, indivsPerProximity.size());
	if (maxSize == 0) {
		return 0.0;
	}
	auto it = indivsPerProximity.begin();
	for (int i=0 ; i < maxSize; i++){
		result += it->first ;
		++it ;
	}
	return result/(double)maxSize ;
}

void Individual::invalidateTopologyState() noexcept {
	invalidateDiversityCache();
	if (topologyGeneration_ ==
		std::numeric_limits<std::uint64_t>::max()) {
		topologyGeneration_ = 1U;
	} else {
		++topologyGeneration_;
	}
	validatedTopologyGeneration_ = 0U;
	validatedProblemRevision_ = 0U;
	validatedFeasibility_ = false;
}

void Individual::invalidateActiveRouteState() noexcept {
	activeRouteScratchOccupancyGeneration_ = 0U;
	if (routeOccupancyGeneration_ ==
		std::numeric_limits<std::uint64_t>::max()) {
		routeOccupancyGeneration_ = 1U;
	} else {
		++routeOccupancyGeneration_;
	}
}

void Individual::updateActiveRouteMembership(
		const int routeIndex,
		const bool occupied) {
	if (activeRouteScratchOccupancyGeneration_ == 0U ||
		activeRouteScratchOccupancyGeneration_ !=
			routeOccupancyGeneration_) {
		return;
	}
	const auto position = std::lower_bound(
		activeRouteScratch_.begin(),
		activeRouteScratch_.end(),
		routeIndex);
	const bool tracked = position != activeRouteScratch_.end() &&
		*position == routeIndex;
	if (tracked == occupied) {
		return;
	}
	if (routeOccupancyGeneration_ ==
		std::numeric_limits<std::uint64_t>::max()) {
		routeOccupancyGeneration_ = 1U;
	} else {
		++routeOccupancyGeneration_;
	}
	if (occupied) {
		activeRouteScratch_.insert(position, routeIndex);
	} else {
		activeRouteScratch_.erase(position);
	}
	activeRouteScratchOccupancyGeneration_ =
		routeOccupancyGeneration_;
}

void Individual::invalidateDiversityCache() noexcept {
	diversityCacheValid = false;
	structuralRouteCacheValid_ = false;
}

void Individual::ensureStructuralRouteCache() const {
	if (structuralRouteCacheValid_) {
		return;
	}
	structuralRouteCache_ = structuralRoutes(*this, numL);
	structuralRouteCacheValid_ = true;
}

void Individual::ensureDiversityCache() {
	if (diversityCacheValid) {
		return;
	}
	for (int i = 0; i < numV; ++i) {
		const Node& node = client[i];
		DiversityNodeCache& cached = diversityCache[i];
		cached.predecessor = node.pre->cour;
		cached.successor = node.next->cour;
		cached.route = node.route->cour;
		cached.location = node.route->whichLocation->cour;
	}
	diversityCacheValid = true;
}

double Individual::brokenPairsDistance(Individual * indiv2){
	double hammingDis = 0;
	ensureDiversityCache();
	indiv2->ensureDiversityCache();
	const DiversityNodeCache* current = diversityCache.data();
	const DiversityNodeCache* comparison = indiv2->diversityCache.data();
	const auto scoreEdge = [&](const int customer, const int neighbor) {
		const DiversityNodeCache& own = current[customer];
		const DiversityNodeCache& other = comparison[customer];
		if (other.predecessor == neighbor ||
			other.successor == neighbor) {
			hammingDis += 1;
		} else if (other.route == own.route) {
			hammingDis += 3;
		} else if (other.location != own.location) {
			hammingDis += 10;
		}
	};
	for (int customer = 0; customer < numV; ++customer) {
		const DiversityNodeCache& own = current[customer];
		if (own.predecessor >= numV &&
			comparison[customer].location != own.location) {
			hammingDis += 10;
		}
		scoreEdge(customer, own.predecessor);
		if (own.successor >= numV) {
			scoreEdge(customer, own.successor);
		}
	}
	return -hammingDis;
}

void Individual::rebuildFromSP(const std::vector<StandaloneRoute>& spRoutes) {
    if (spRoutes.empty()) {
        throw std::invalid_argument(
            "SP reconstruction requires at least one route");
    }

    // Validate the complete value-level solution before mutating any linked
    // list. The solver also validates its incumbent, but this importer is an
    // independent trust boundary.
    std::vector<int> customerUse(numV, 0);
    std::vector<int> depotRouteCount(numL, 0);
    std::vector<int> depotLoad(numL, 0);
    for (const StandaloneRoute& sRoute : spRoutes) {
        if (sRoute.locId < 0 || sRoute.locId >= numL) {
            throw std::out_of_range(
                "SP reconstruction contains an invalid depot index");
        }
        if (sRoute.sequence.empty()) {
            throw std::invalid_argument(
                "SP reconstruction contains an empty route");
        }
        if (++depotRouteCount[sRoute.locId] >
            location[sRoute.locId].maxVisit) {
            throw std::invalid_argument(
                "SP reconstruction exceeds the fixed-slot representation "
                "route limit at depot " +
                std::to_string(sRoute.locId));
        }

        for (const int customer : sRoute.sequence) {
            if (customer < 0 || customer >= numV) {
                throw std::out_of_range(
                    "SP reconstruction contains an invalid customer index");
            }
            if (++customerUse[customer] != 1) {
                throw std::invalid_argument(
                    "SP reconstruction serves a customer more than once");
            }
        }
        const RouteEvaluation evaluation = evaluateRoute(
            *data, sRoute.locId, sRoute.sequence);
        const int computedLoad = evaluation.load;
        const double computedCost = evaluation.travelCost;

        if (computedLoad != sRoute.totalLoad) {
            throw std::invalid_argument(
                "SP reconstruction route load does not match the instance");
        }
        if (computedLoad > data->vehCap) {
            throw std::invalid_argument(
                "SP reconstruction route exceeds vehicle capacity");
        }
        if (!data->equalTwoVar(computedCost, sRoute.routeCost)) {
            throw std::invalid_argument(
                "SP reconstruction route cost does not match its sequence");
        }
        if (data->hasHardTimeWindows() &&
            !evaluation.temporalFeasible) {
            throw std::invalid_argument(
                "SP reconstruction contains a route that violates a hard "
                "time window or depot closing time");
        }
        depotLoad[sRoute.locId] += computedLoad;
        if (depotLoad[sRoute.locId] >
            location[sRoute.locId].inventory) {
            throw std::invalid_argument(
                "SP reconstruction exceeds depot capacity");
        }
    }
    for (const int useCount : customerUse) {
        if (useCount != 1) {
            throw std::invalid_argument(
                "SP reconstruction does not cover every customer exactly once");
        }
    }

    // Reset all links, routes and aggregate capacity state.
    this->initilization();

    for (const StandaloneRoute& sRoute : spRoutes) {
        const int locId = sRoute.locId;

        this->location[locId].used = true;

        // splitTime points to the next available route slot at this depot.
        const int currentSplitTime = this->location[locId].splitTime;
        
        Node* startDepot = &this->location[locId].depots[currentSplitTime];
        Node* endDepot   = &this->location[locId].endDepots[currentSplitTime];
        Route* currentRoute = startDepot->route;

        currentRoute->used = true;
        this->location[locId].emptyRoutes.erase(currentRoute->cour);

        // Build the route's doubly linked customer chain.
        Node* previousNode = startDepot;
        
        for (int custId : sRoute.sequence) {
            Node* currentNode = &this->client[custId];
            
            previousNode->next = currentNode;
            currentNode->pre = previousNode;
            currentNode->route = currentRoute;
            previousNode = currentNode;
        }

        previousNode->next = endDepot;
        endDepot->pre = previousNode;

        this->location[locId].splitTime++;

        // Recompute route load and travel cost from the rebuilt chain.
        this->updateRouteInfor(currentRoute->cour, -1);
        if (currentRoute->load != sRoute.totalLoad ||
            !data->equalTwoVar(currentRoute->dis, sRoute.routeCost)) {
            throw std::logic_error(
                "SP reconstruction produced inconsistent route aggregates");
        }
    }

    // Recompute aggregate state for every active depot.
    for (int i = 0; i < data->numL; i++) {
        if (this->location[i].used) {
            int locIndex = i + data->numV;
            this->updateLocationInfor(locIndex);
        }
    }

    // Recompute the objective and derive feasibility from the rebuilt
    // structure. Do not declare a solver result feasible by assignment.
    this->evaluationDis();

    this->capExcess = 0;
    this->penalty = 0.0;
    this->decideFeasible();

    if (!this->isFeasible) {
        throw std::logic_error(
            "SP reconstruction produced an infeasible individual");
    }
    this->isRight();
}
