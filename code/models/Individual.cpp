/******************************************************************************
 * File: Individual.cpp
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description: 
 *   Data structure representing a single solution (individual) in the population.
 ******************************************************************************/

#include "Individual.h"
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

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
	  checkArray(nullptr) {}

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
	if (data.vehCap <= 0) {
		throw std::invalid_argument(
			"representation route limit: vehicle capacity must be positive");
	}

	const int inventory = data.invLoc[data.numV + depotIndex];
	if (inventory < 0 || data.totalDemand < 0) {
		throw std::invalid_argument(
			"representation route limit: capacities must be nonnegative");
	}

	const double globalBoundValue =
		std::ceil(1.2 * static_cast<double>(data.totalDemand) /
				  static_cast<double>(data.vehCap)) +
		5.0;
	const double depotBoundValue =
		3.0 * std::ceil(static_cast<double>(inventory) /
						static_cast<double>(data.vehCap));
	if (globalBoundValue >
			static_cast<double>(std::numeric_limits<int>::max()) ||
		depotBoundValue >
			static_cast<double>(std::numeric_limits<int>::max())) {
		throw std::overflow_error(
			"representation route limit exceeds the supported integer range");
	}
	return std::min(
		static_cast<int>(globalBoundValue),
		static_cast<int>(depotBoundValue));
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
	Node *p = NULL;
	int capLoc, capVeh;
	for (int i=0;i<numV;i++)
		checkArray[i] = 0;
	for (int loc=numV; loc < numC; loc++){
		if (location[loc - numV].used){
			capLoc = 0;
			totalFit += data->openCostLoc[loc];
			totalFit += data->usingCostVeh * location[loc - numV].splitTime;
			for (int i=0;i<location[loc - numV].maxVisit;i++){
				if (!location[loc - numV].depots[i].route->used)continue;
				capVeh = 0;
				p = &location[loc - numV].depots[i];
				totalDis += data->D[p->cour][p->next->cour];
				p=p->next;
				while(! p->next->isDepot){
					checkArray[p->cour] ++;
					capVeh += data->demCus[p->cour];
					totalDis += data->D[p->cour][p->next->cour];
					p=p->next;
				}
				checkArray[p->cour] ++;
				capVeh += data->demCus[p->cour];
				if (capVeh != p->route->load || capVeh > data->vehCap){
                    throw std::runtime_error(
                        "individual validation failed: route capacity");
				}
				capLoc += capVeh;
				totalDis += data->D[p->cour][p->next->cour];
			}
			if (capLoc != p->route->whichLocation->remainInven || capLoc > location[loc-numV].inventory){
                throw std::runtime_error(
                    "individual validation failed: depot capacity");
			}
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
	std::ofstream myfile(data->pathSolution);
	//double Distance=0;
	if (myfile.is_open()){
		myfile<<"The objective is: "<<std::endl;
		myfile<<fit<<std::endl;
		myfile<<"The running time is: "<<std::endl;
		myfile<<(double)clock()/(double)CLOCKS_PER_SEC<<std::endl;
		myfile<<"The time meeting best is: "<<std::endl;
		myfile<<data->meetBestTime<<std::endl;
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
		// computing the actual cost
		double totalDis=0,totalFit=0;
		int capLoc, capVeh;
		for (int loc=numV; loc < numC; loc++){
			if (location[loc - numV].used){
				capLoc = 0;
				totalFit += data->openCostLoc[loc];
				totalFit += data->usingCostVeh * location[loc - numV].splitTime;
				for (int i=0;i<location[loc - numV].maxVisit;i++){
					if (!location[loc - numV].depots[i].route->used)continue;
					capVeh = 0;
					p = &location[loc - numV].depots[i];
					totalDis += data->D[p->cour][p->next->cour];
					p=p->next;
					while(! p->next->isDepot){
						checkArray[p->cour] ++;
						capVeh += data->demCus[p->cour];
						totalDis += data->D[p->cour][p->next->cour];
						p=p->next;
					}
					checkArray[p->cour] ++;
					capVeh += data->demCus[p->cour];
					if (capVeh != p->route->load || capVeh > data->vehCap){
                        throw std::runtime_error(
                            "solution output failed: route capacity mismatch");
					}
					capLoc += capVeh;
					totalDis += data->D[p->cour][p->next->cour];
				}
			}
		}
		myfile<<"The actual cost is "<<totalFit + totalDis<<std::endl;


	/*	myfile<<numC<<std::endl;
		for (int i=0;i<numC;i++){
			myfile<<data->x[i]<<" "<<data->y[i]<<std::endl;
		}
		Node *p;
		for (int loc=numV; loc < numC; loc++){
			if (location[loc - numV].used){
				int count=0;
				for (int i=0;i<location[loc - numV].maxVisit;i++){
					if (!location[loc - numV].depots[i].route->used)continue;
					count++;
					p = &location[loc - numV].depots[i];
					myfile<<p->cour<<" ";
					p=p->next;
					while(! p->isDepot){
						myfile<<p->cour<<" ";
						p=p->next;
					}
				//	myfile<<p->cour<<" "<<p->next->cour<<std::endl;
					myfile<<p->cour<<" "<<std::endl;
				}
			}
		}
*/
	}
	else std::cout << "----- IMPOSSIBLE TO OPEN: " << data->pathSolution << std::endl;
	myfile.close();

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
double Individual::brokenPairsDistance(Individual * indiv2){
	Node *p,*p1;bool isFirst;
	double hammingDis = 0;
	for (int i=numV; i<numC; i++){
		if (location[i-numV].used){
			for (int j=0; j<location[i-numV].maxVisit; j++){
				if (location[i-numV].depots[j].route->used){
					p = &location[i-numV].depots[j];
					p1=p->next;
					if (indiv2->client[p1->cour].route->whichLocation->cour != p->route->whichLocation->cour)hammingDis += 10;
					isFirst = true;
					while (isFirst || !p->next->isDepot){
						if (indiv2->client[p->next->cour].pre->cour == p->cour || indiv2->client[p->next->cour].next->cour == p->cour)
							hammingDis += 1;
						else if (indiv2->client[p->next->cour].route->cour == p->route->cour){
							hammingDis += 3;
						}
						else if (indiv2->client[p->next->cour].route->whichLocation->cour != p->route->whichLocation->cour){
							hammingDis += 10;
						}
						p = p->next;
						isFirst = false;
					}
					if (indiv2->client[p->cour].pre->cour == p->next->cour || indiv2->client[p->cour].next->cour == p->next->cour)
						hammingDis += 1;
					else if (indiv2->client[p->cour].route->cour == p->next->route->cour){
						hammingDis += 3;
					}
					else if (indiv2->client[p->cour].route->whichLocation->cour != p->next->route->whichLocation->cour){
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

        int computedLoad = 0;
        double computedCost = 0.0;
        int previous = numV + sRoute.locId;
        for (const int customer : sRoute.sequence) {
            if (customer < 0 || customer >= numV) {
                throw std::out_of_range(
                    "SP reconstruction contains an invalid customer index");
            }
            if (++customerUse[customer] != 1) {
                throw std::invalid_argument(
                    "SP reconstruction serves a customer more than once");
            }
            computedLoad += data->demCus[customer];
            computedCost += data->D[previous][customer];
            previous = customer;
        }
        computedCost +=
            data->D[previous][numV + sRoute.locId];

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

    if (this->isFeasible) {
        this->isRight();
    }
}
