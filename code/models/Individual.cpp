/******************************************************************************
 * File: Individual.cpp
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description: 
 *   Data structure representing a single solution (individual) in the population.
 ******************************************************************************/

#include "Individual.h"
#include "RouteEvaluator.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

bool hasHardTimeWindows(const read_data& data) noexcept {
	return data.problemMode == hgasp::ProblemMode::ClrpTw;
}

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
			if (!route.used || route.nbClients <= 0)
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
	const read_data& data,
	const int depotIndex,
	const std::vector<int>& sequence,
	const EvaluationDetail detail = EvaluationDetail::Summary) {
	if (hasHardTimeWindows(data)) {
		const HardTimeWindowConstraint hardTimeWindows;
		return RouteEvaluator(data, hardTimeWindows).evaluate(
			depotIndex, sequence, detail);
	}
	const NoTemporalConstraint noTemporalConstraint;
	return RouteEvaluator(data, noTemporalConstraint).evaluate(
		depotIndex, sequence, detail);
}

int originalCustomerId(const read_data& data, const int customer) {
	if (customer >= 0 &&
		static_cast<std::size_t>(customer) <
			data.originalCustomerIds.size()) {
		return data.originalCustomerIds[static_cast<std::size_t>(customer)];
	}
	return customer;
}

int originalDepotId(const read_data& data, const int depot) {
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
		const read_data& data,
		const int depotIndex) {
	if (depotIndex < 0 || depotIndex >= data.numL) {
		throw std::out_of_range(
			"representation route limit: depot index is out of range");
	}
	if (data.numV <= 0) {
		throw std::invalid_argument(
			"representation route limit: customer count must be positive");
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
	return structuralRoutes(*this, numL) ==
		structuralRoutes(other, other.numL);
}

void Individual::define(read_data *data){
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
			rankRoute++;
		}
	}
	checkArray = new int [numV]{};
}
void Individual::initilization(){
	invalidateDiversityCache();
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
	invalidateDiversityCache();
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
	if (hasHardTimeWindows(*data)) {
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
	if (route[r].load == 0)
		route[r].whichLocation->emptyRoutes.insert(route[r].cour);
	else
		route[r].whichLocation->emptyRoutes.erase(route[r].cour);
}
void Individual:: updateLocationInfor(int &ll){
	int l = ll - numV;
	location[l].splitTime = 0;
	location[l].remainInven = 0;
	for (int i=0; i<location[l].maxVisit; i++){
		if (location[l].depots[i].route->load > 0){
			location[l].remainInven += location[l].depots[i].route->load;
			location[l].depots[i].route->used = true;
			location[l].splitTime++;
		}
		else location[l].depots[i].route->used = false;
	}
//	if (location[l].remainInven > location[l].inventory)std::cout<<"report a wrong "
	if (location[l].remainInven == 0){
		location[l].used = false;
		location[l].splitTime=0;
	}
	else location[l].used = true;
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
				if (currentRoute.used || currentRoute.load != 0) {
					throw std::runtime_error(
						"individual validation failed: empty route state");
				}
				continue;
			}
			if (!depot.used || !currentRoute.used) {
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
			if (hasHardTimeWindows(*data)) {
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
void Individual::decideFeasible(){
	if (hasHardTimeWindows(*data)) {
		std::vector<int> customerUse(
			static_cast<std::size_t>(numV), 0);
		for (int i=0;i<numL;i++){
			int computedDepotLoad = 0;
			int computedRouteCount = 0;
			for (int j=0; j<location[i].maxVisit; j++){
				Route& currentRoute =
					*location[i].depots[j].route;
				const std::vector<int> sequence =
					customerSequence(currentRoute);
				if (sequence.empty()) {
					if (currentRoute.used ||
						currentRoute.load != 0) {
						isFeasible = false;
						return;
					}
					continue;
				}
				if (!location[i].used || !currentRoute.used) {
					isFeasible = false;
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
					isFeasible = false;
					return;
				}
				for (const int customer : sequence) {
					if (++customerUse[
							static_cast<std::size_t>(customer)] != 1) {
						isFeasible = false;
						return;
					}
				}
				computedDepotLoad += evaluation.load;
			}
			const bool shouldBeUsed = computedRouteCount != 0;
			if (location[i].used != shouldBeUsed ||
				location[i].splitTime != computedRouteCount ||
				computedDepotLoad > location[i].inventory ||
				computedDepotLoad != location[i].remainInven) {
				isFeasible = false;
				return;
			}
		}
		for (const int useCount : customerUse) {
			if (useCount != 1) {
				isFeasible = false;
				return;
			}
		}
		isFeasible = true;
		return;
	}

	// Preserve the legacy CLRP feasibility semantics.
	for (int i=0;i<numL;i++){
		if (location[i].used){
			for (int j=0; j<location[i].maxVisit; j++){
				if (location[i].depots[j].route->used){
					if (location[i].depots[j].route->load > data->vehCap){
						isFeasible = false;
						return;
					}
				}
			}
			if (location[i].remainInven > location[i].inventory){
				isFeasible = false;
				return;
			}
		}
	}
	isFeasible = true;
}
void Individual::outputBestSolution(){
	std::cout<<"----------Write Solution With Value  "<<fit<<" In: "<<data->pathSolution<<std::endl;
	decideFeasible();
	if (!isFeasible) {
		throw std::runtime_error(
			"solution output failed: solution is infeasible");
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
	const double searchCpuSeconds = data->elapsedCpuSeconds();

	// Preserve the historical human-readable output for existing workflows.
	std::ofstream myfile(data->pathSolution);
	if (myfile.is_open()){
		myfile << std::setprecision(17);
			myfile<<"The objective is: "<<std::endl;
			myfile<<fit<<std::endl;
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
		myfile<<"The actual cost is "<<actualCost<<std::endl;
	}
	else std::cout << "----- IMPOSSIBLE TO OPEN: " << data->pathSolution << std::endl;
	myfile.close();

	// The machine-readable solution is a fixed-name sibling of solution.txt,
	// so experiment tooling can validate either problem mode uniformly.
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
	json << "  \"schema_version\": 1,\n";
	json << "  \"problem_mode\": "
		 << jsonEscape(hgasp::toString(data->problemMode)) << ",\n";
	json << "  \"eax_parent_mode\": "
		 << jsonEscape(hgasp::toString(data->eaxParentMode)) << ",\n";
	json << "  \"instance_id\": "
		 << jsonEscape(
				std::filesystem::path(data->pathInstance).stem().string())
		 << ",\n";
	json << "  \"feasible\": true,\n";
	json << "  \"distance_policy\": "
		 << jsonEscape(
				hasHardTimeWindows(*data)
					? "euclidean-floor"
					: (data->instanceFormat ==
							   hgasp::InstanceFormat::LegacyDat &&
						   data->realOrInteger != 0
						   ? "euclidean"
						   : "euclidean-ceil-100"))
		 << ",\n";
	json << "  \"timing\": {\n";
	json << "    \"search_cpu_seconds\": " << searchCpuSeconds << ",\n";
	json << "    \"global_best_cpu_seconds\": "
		 << data->meetBestTime << ",\n";
	json << "    \"basis\": \"process_cpu\",\n";
	json << "    \"scope\": \"search_after_instance_loading\"\n";
	json << "  },\n";
	json << "  \"objective\": {\n";
	json << "    \"total\": " << actualCost << ",\n";
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
				hasHardTimeWindows(*data)
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

			if (hasHardTimeWindows(*data)) {
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

void Individual::invalidateDiversityCache() noexcept {
	diversityCacheValid = false;
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
	Node *p,*p1;bool isFirst;
	double hammingDis = 0;
	indiv2->ensureDiversityCache();
	const DiversityNodeCache* comparison = indiv2->diversityCache.data();
	for (int i=numV; i<numC; i++){
		if (location[i-numV].used){
			for (int j=0; j<location[i-numV].maxVisit; j++){
					if (location[i-numV].depots[j].route->used){
						p = &location[i-numV].depots[j];
						p1=p->next;
						if (p1 == nullptr || p1->isDepot) {
							throw std::logic_error(
								"diversity evaluation found a used empty "
								"route");
						}
						if (comparison[p1->cour].location !=
							p->route->whichLocation->cour) {
						hammingDis += 10;
					}
					isFirst = true;
					while (isFirst || !p->next->isDepot){
						const DiversityNodeCache& cached =
							comparison[p->next->cour];
						if (cached.predecessor == p->cour ||
							cached.successor == p->cour)
							hammingDis += 1;
						else if (cached.route == p->route->cour){
							hammingDis += 3;
						}
						else if (cached.location !=
							p->route->whichLocation->cour){
							hammingDis += 10;
						}
						// Advance once so every route edge is examined.  The
						// cache refactor must preserve the original
						// broken-pairs traversal; advancing twice skips half
						// the edges and can land on the end-depot sentinel.
						p = p->next;
						isFirst = false;
					}
					const DiversityNodeCache& cached = comparison[p->cour];
					if (cached.predecessor == p->next->cour ||
						cached.successor == p->next->cour)
						hammingDis += 1;
					else if (cached.route == p->next->route->cour){
						hammingDis += 3;
					}
					else if (cached.location !=
						p->next->route->whichLocation->cour){
						hammingDis += 10;
					}
				}
			}
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
        if (hasHardTimeWindows(*data) &&
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
