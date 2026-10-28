#include "TimeWindowOperators.h"
/*
 * initialsol.cpp
 *
 *  Created on: 17 Apr 2020
 *      Author: Pengfei He
 */
#include "basic.h"
#include "initialsol.h"
initial_sol::initial_sol(read_data * data, RepairProcedure *repair) {
	this->repair = repair;
	this->data=data;
	this->numV = data->numV;
	this->numL = data->numL;
	this->numC = data->numC;
	locUsed = new bool [numC];
	cusUsed = new bool [numV];
}
initial_sol::~initial_sol() {
	delete [] locUsed;
	delete [] cusUsed;
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
	//	rank=int(pow(y,data->determine_para)*(int)data->compactLocConfigure.size());// this location configuration
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

	std::vector<int>listLoc1;
	while(remainDem != 0){
		for (int i=0;i<numL;i++)
			if (!locUsed[data->locRatio[i].second] && remainDem == s->location[data->locRatio[i].second - numV].inventory)
				listLoc1.push_back(data->locRatio[i].second);
		if ((int)listLoc1.size() == 0){
			for (int i=0;i<numL;i++)
				if (!locUsed[data->locRatio[i].second])listLoc1.push_back(data->locRatio[i].second);
			//
				y = RandomDouble(0.0, 1.0);
				rank = int(pow(y, data->determine_para) * (int)listLoc1.size());
			tarLoc = listLoc1[rank];
			insertCusToLoc(tarLoc);
		}
		else{
			y = RandomDouble(0.0, 1.0);
			rank = int(pow(y, data->determine_para) * (int)listLoc1.size());
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
			if (tarLoc == -1){std::cout<<"there is a wrong problem"<<std::endl;exit(EXIT_FAILURE);}
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
	repair->repairForInitialSolution(s);
}
void initial_sol::initial_solution(Individual *s, int rankPop){
	if (data->hardTimeWindows) {
		s->isFeasible = masptw::construct(s, data, rankPop);
		return;
	}
	this->s=s;
	this->rankPop = rankPop;
	newGreedyInsert();

	s->evaluationDis();
	s->isRight();
}

