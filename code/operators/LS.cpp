/******************************************************************************
 * File: LS.cpp
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description: 
 *   Local Search algorithms for route optimization and refinement.
 ******************************************************************************/

#include "LS.h"
#include "basic.h"
#include "Individual.h"
LS::LS(read_data * data, RepairProcedure *repair) {
	this->data=data;
	this->repair = repair;
	this->numV = data->numV;
	this->numC = data->numC;
	this->numL = data->numL;
	for (int i=0;i<numV;i++)
		orderNodes.push_back(i);
	auxArray1 = new int [numV];
}
LS::~LS() {
	delete [] auxArray1;
}
double LS::compute(bool istrue, double value){
	if (istrue)	return 0.;
	else return value;
}
void LS::srun(){
	RandomShuffle(orderNodes);
	for (int i=0; i<numV; i++) 
		if (RandomInt(0, data->alpha - 1) == 0) 
			RandomShuffle(data->edgeNode[i]);
        
	searchComplete=false;
	for (loopID=0;!searchComplete;loopID++){
		searchComplete=true;
		for (int posU=0;posU<numV;posU++){// only num_v-1 clients
			nodeU=&s->client[orderNodes[posU]];
			int lastTestRINodeU = nodeU->whenLastTestedRI;
			nodeU->whenLastTestedRI=nbMoves;
			for (int posV = 0; posV<(int)data->edgeNode[nodeU->cour].size(); posV++){
				if (data->edgeNode[nodeU->cour][posV] >= numV)continue;// not consider locations
				nodeV=&s->client[data->edgeNode[nodeU->cour][posV]];
				if (loopID==0 || std::max<int>(nodeU->route->whenLastModified,nodeV->route->whenLastModified) > lastTestRINodeU){
					setLocalVariableRouteU();
					setLocalVariableRouteV();
					//
					if (Relocate_one())continue;//10 insert operator
					if (Relocate_two())continue;//20 insert operator
					if (Relocate_three())continue;//02 insert operator
					if (nodeUIndex < nodeVIndex && Relocate_four())continue;//11 swap operator
					if (Relocate_five())continue;//21 swap
					if (nodeUIndex <= nodeVIndex && Relocate_six()) continue; // SWAP 22
					if (nodeUIndex <= nodeVIndex && Relocate_seven()) continue;; // SWAP 2-2 reverse
					if (routeU == routeV && Relocate_eight()) continue; // 2-OPT (同路线)
					if (routeU != routeV && Relocate_nine()) continue; // 2-OPT* (逆向翻转交叉)
					if (routeU != routeV && Relocate_ten())  continue; // 2-OPT* (平移交叉)
					// Trying moves that insert nodeU directly after the depot
					if (nodeV->pre->isDepot){
						nodeV = nodeV->pre;
						setLocalVariableRouteV();
						if (Relocate_one()) continue; // RELOCATE
						if (Relocate_two()) continue; // RELOCATE
						if (Relocate_three()) continue; // RELOCATE
						if (routeU != routeV && Relocate_nine()) continue; // 2-OPT*
						if (routeU != routeV && Relocate_ten()) continue; // 2-OPT*
					}
				}
			}
			if (loopID > 0){
				for (int posV = 0; posV < (int)data->customerToLoc[nodeU->cour].size();posV++){
					if (!s->location[data->customerToLoc[nodeU->cour][posV] - numV].used)continue;
					if (!s->location[data->customerToLoc[nodeU->cour][posV] - numV].emptyRoutes.empty()){
						nodeV = s->route[*s->location[data->customerToLoc[nodeU->cour][posV] - numV].emptyRoutes.begin()].depot;
						setLocalVariableRouteU();
						setLocalVariableRouteV();
						if (Relocate_one()) continue; // RELOCATE
						if (Relocate_two()) continue; // RELOCATE
						if (Relocate_nine()) continue; // RELOCATE
						if (Relocate_ten()) continue; // 2-OPT*
					}
				}
			}
			//begin split operators// the main function is to add routes
			setLocalVariableRouteU();
			if (Splitb())continue;
			if (Splitf())continue;
		}/// end the CVRP neighborhoods search
	}
}
bool LS::Splitf(){
	for (int i=0;i<(int)data->customerToLoc[nodeUIndex].size(); i++){
		int loc = data->customerToLoc[nodeUIndex][i];
		if (loc != nodeU->route->whichLocation->cour){
			Node * beginU = nodeU->route->depot->next;
		//	if (nodeU == lastU)return false;
			if (s->location[loc-numV].remainInven + nodeU->cumulatedLoad > s->location[loc-numV].inventory)return false;
			if (s->location[loc - numV].emptyRoutes.empty())return false;
			moveGain = compute(s->location[loc-numV].used, data->openCostLoc[loc])
					- compute(!(nodeU->route->whichLocation->splitTime == 1 && nodeU->next->isDepot), data->openCostLoc[nodeU->route->whichLocation->cour])
					- compute(!nodeU->next->isDepot, data->usingCostVeh) + data->usingCostVeh
					+ data->D[nodeUIndex][loc] + data->D[loc][beginU->cour] + data->D[nodeXIndex][beginU->pre->cour]
					- data->D[nodeXIndex][nodeUIndex] - data->D[beginU->cour][beginU->pre->cour];

			if (moveGain > -Min)return false;

			Node *xx = nodeU;
			Route *rr = &s->route[*s->location[loc-numV].emptyRoutes.begin()];

			while(!xx->isDepot){
				xx->route = rr;//route
				xx=xx->pre;
			}
			Node *pre, *next;
			pre=beginU->pre;
			next = nodeX;
			pre->next = next;
			next->pre = pre;
			//
			beginU->pre = rr->depot;
			rr->depot->next = beginU;
			nodeU->next= rr->endDepot;
			rr->endDepot->pre = nodeU;
			s->updateRouteInfor(routeU->cour,nbMoves);
			s->updateRouteInfor(rr->cour, nbMoves);

			s->updateLocationInfor(routeU->whichLocation->cour);
			s->updateLocationInfor(nodeU->route->whichLocation->cour);
/*			s->outputSolution();
			s->isRight();
			distt += moveGain;
			s->evaluationDis();
				if (!data->equalTwoVar(distt,s->fit)){
					std::cout<<"the distance is wrong"<<std::endl;
				}
*/
			return true;
		}
	}
	return false;
}
bool LS::Splitb(){
	for (int i=0;i<(int)data->customerToLoc[nodeUIndex].size(); i++){
		int loc = data->customerToLoc[nodeUIndex][i];
		if (loc != nodeU->route->whichLocation->cour){
			Node * lastU = nodeU->route->endDepot->pre;
		//	if (nodeU == lastU)return false;
			if (s->location[loc-numV].remainInven + (nodeU->route->load - nodeU->pre->cumulatedLoad) > s->location[loc-numV].inventory)return false;
			if (s->location[loc - numV].emptyRoutes.empty())return false;
			moveGain = compute(s->location[loc-numV].used, data->openCostLoc[loc])
					- compute(!(nodeU->route->whichLocation->splitTime == 1 && nodeU->pre->isDepot), data->openCostLoc[nodeU->route->whichLocation->cour])
					- compute(!nodeU->pre->isDepot, data->usingCostVeh) + data->usingCostVeh
					+ data->D[nodeUIndex][loc] + data->D[loc][lastU->cour] + data->D[nodeUPrevIndex][lastU->next->cour]
					- data->D[nodeUPrevIndex][nodeUIndex] - data->D[lastU->cour][lastU->next->cour];

			if (moveGain > -Min)return false;
			Node *xx = nodeU;
			Route *rr = &s->route[*s->location[loc-numV].emptyRoutes.begin()];

			while(!xx->isDepot){
				xx->route = rr;
				xx=xx->next;
			}
			Node *pre, *next;
			pre=nodeU->pre;
			next = lastU->next;
			pre->next = next;
			next->pre = pre;
			//
			nodeU->pre = rr->depot;
			rr->depot->next = nodeU;
			lastU->next= rr->endDepot;
			rr->endDepot->pre = lastU;
			s->updateRouteInfor(routeU->cour,nbMoves);
			s->updateRouteInfor(rr->cour,nbMoves);

			s->updateLocationInfor(routeU->whichLocation->cour);
			s->updateLocationInfor(nodeU->route->whichLocation->cour);
/*			s->outputSolution();
			s->isRight();
			distt += moveGain;
			s->evaluationDis();
				if (!data->equalTwoVar(distt,s->fit)){
					std::cout<<"the distance is wrong"<<std::endl;
				}
*/
			return true;
		}
	}
	return false;
}
bool LS::Relocate_one(){
    // Reject overlapping nodes that would create a self-loop.
	if (nodeUIndex == nodeYIndex || nodeUIndex == nodeVIndex) return false;
	
    // A cross-depot move must respect the destination depot capacity.
    if (nodeU->route->whichLocation != nodeV->route->whichLocation) {
	    if (!(nodeV->route->whichLocation->remainInven + nodeU->dem <= nodeV->route->whichLocation->inventory))
		    return false;
    }
	if(nodeV->route != nodeU->route && nodeV->route->load + nodeU->dem > data->vehCap)return false;// route feasible

	moveGain = compute(nodeV->route->whichLocation->used, data->openCostLoc[nodeV->route->whichLocation->cour])
			+ compute(!(nodeV->isDepot && nodeY->isDepot), data->usingCostVeh)
			- compute(!(nodeU->route->whichLocation->splitTime == 1 && nodeU->route->nbClients == 1 && nodeU->route->whichLocation != nodeV->route->whichLocation), data->openCostLoc[nodeU->route->whichLocation->cour])
			- compute(nodeU->route->nbClients != 1, data->usingCostVeh)
			+ data->D[nodeUPrevIndex][nodeXIndex]-data->D[nodeUPrevIndex][nodeUIndex]-data->D[nodeUIndex][nodeXIndex]
			+ data->D[nodeVIndex][nodeUIndex]+data->D[nodeUIndex][nodeYIndex]-data->D[nodeVIndex][nodeYIndex];
	//
	if (moveGain > -Min)return false;
	insertNode(nodeU,nodeV);

	nbMoves++;
	searchComplete=false;
	s->updateRouteInfor(routeU->cour,nbMoves);
	if (routeU != routeV) s->updateRouteInfor(routeV->cour,nbMoves);
	s->updateLocationInfor(routeU->whichLocation->cour);
	if (routeU->whichLocation != routeV->whichLocation)
		s->updateLocationInfor(routeV->whichLocation->cour);

	return true;
}

bool LS::Relocate_two(){
    // Reject overlapping nodes and two-customer blocks ending at a depot.
	if (nodeUIndex == nodeYIndex || nodeXIndex == nodeVIndex || nodeUIndex == nodeVIndex || nodeX->isDepot) return false;

    // Same-depot moves do not change depot load.
	if (nodeU->route->whichLocation != nodeV->route->whichLocation) {
        if (!(nodeV->route->whichLocation->remainInven + nodeU->dem + nodeX->dem <= nodeV->route->whichLocation->inventory))
            return false;
    }
	if(nodeV->route != nodeU->route && nodeV->route->load + nodeU->dem + nodeX->dem > data->vehCap)return false;
	
    // Evaluate insertion of the ordered block U -> X after V.
	moveGain = compute(nodeV->route->whichLocation->used, data->openCostLoc[nodeV->route->whichLocation->cour])
			+ compute(!(nodeV->isDepot && nodeY->isDepot), data->usingCostVeh)
			- compute(!(nodeU->route->whichLocation->splitTime == 1 && nodeU->route->nbClients == 2 && nodeU->route->whichLocation != nodeV->route->whichLocation), data->openCostLoc[nodeU->route->whichLocation->cour])
			- compute(nodeU->route->nbClients != 2, data->usingCostVeh)
			+ data->D[nodeUPrevIndex][nodeXNextIndex]-data->D[nodeUPrevIndex][nodeUIndex]-data->D[nodeXIndex][nodeXNextIndex]
			+ data->D[nodeVIndex][nodeUIndex]+data->D[nodeXIndex][nodeYIndex]-data->D[nodeVIndex][nodeYIndex];
	
	if (moveGain>-Min)return false;
    
    // Preserve the U -> X order while inserting the block after V.
	insertNode(nodeU,nodeV);
	insertNode(nodeX,nodeU);

	nbMoves++;
	searchComplete=false;
	s->updateRouteInfor(routeU->cour,nbMoves);
	if (routeU != routeV) s->updateRouteInfor(routeV->cour,nbMoves);
	s->updateLocationInfor(routeU->whichLocation->cour);
	if (routeU->whichLocation != routeV->whichLocation)
		s->updateLocationInfor(routeV->whichLocation->cour);

	return true;
}

bool LS::Relocate_three(){
    // Reject overlapping nodes and two-customer blocks ending at a depot.
	if (nodeUIndex == nodeYIndex || nodeXIndex == nodeVIndex || nodeUIndex == nodeVIndex || nodeX->isDepot) return false;
	
    // Same-depot moves do not change depot load.
    if (nodeU->route->whichLocation != nodeV->route->whichLocation) {
        if (!(nodeV->route->whichLocation->remainInven + nodeU->dem + nodeX->dem <= nodeV->route->whichLocation->inventory))
            return false;
    }
	if(nodeV->route != nodeU->route && nodeV->route->load + nodeU->dem + nodeX->dem > data->vehCap)return false;
	
    // Compute moveGain uniformly, including the reversal cost.
	moveGain = compute(nodeV->route->whichLocation->used, data->openCostLoc[nodeV->route->whichLocation->cour])
			+ compute(!(nodeV->isDepot && nodeY->isDepot), data->usingCostVeh)
			- compute(!(nodeU->route->whichLocation->splitTime == 1 && nodeU->route->nbClients == 2 && nodeU->route->whichLocation != nodeV->route->whichLocation), data->openCostLoc[nodeU->route->whichLocation->cour])
			- compute(nodeU->route->nbClients != 2, data->usingCostVeh)
			+ data->D[nodeUPrevIndex][nodeXNextIndex]-data->D[nodeUPrevIndex][nodeUIndex]-data->D[nodeXIndex][nodeXNextIndex]
			+ data->D[nodeVIndex][nodeXIndex]+data->D[nodeUIndex][nodeYIndex]-data->D[nodeVIndex][nodeYIndex]
            + data->D[nodeXIndex][nodeUIndex]-data->D[nodeUIndex][nodeXIndex];
	
	if (moveGain>-Min)return false;
    
    // Insert the reversed block X -> U after V.
	insertNode(nodeX,nodeV);
	insertNode(nodeU,nodeX);

	nbMoves++;
	searchComplete=false;
	s->updateRouteInfor(routeU->cour,nbMoves);
	if (routeU != routeV) s->updateRouteInfor(routeV->cour,nbMoves);
	s->updateLocationInfor(routeU->whichLocation->cour);
	if (routeU->whichLocation != routeV->whichLocation)
		s->updateLocationInfor(routeV->whichLocation->cour);

	return true;
}

bool LS::Relocate_four(){//
	if (nodeUIndex == nodeVPrevIndex || nodeUIndex == nodeYIndex)return false;
	if (nodeU->route->whichLocation != nodeV->route->whichLocation)
		if ((nodeV->route->whichLocation->remainInven + nodeU->dem - nodeV->dem > nodeV->route->whichLocation->inventory) || (nodeU->route->whichLocation->remainInven + nodeV->dem - nodeU->dem > nodeU->route->whichLocation->inventory))return false;
	if (nodeU->route->load - nodeU->dem + nodeV->dem > data->vehCap || nodeV->route->load - nodeV->dem + nodeU->dem > data->vehCap)return false;

	moveGain=data->D[nodeUPrevIndex][nodeVIndex]+data->D[nodeVIndex][nodeXIndex]-data->D[nodeUPrevIndex][nodeUIndex]-data->D[nodeUIndex][nodeXIndex];
	moveGain+=data->D[nodeVPrevIndex][nodeUIndex]+data->D[nodeUIndex][nodeYIndex]-data->D[nodeVPrevIndex][nodeVIndex]-data->D[nodeVIndex][nodeYIndex];
	//
	if (moveGain > -Min)return false;
	swapNode(nodeU,nodeV);

	nbMoves++;
	searchComplete=false;
	s->updateRouteInfor(routeU->cour,nbMoves);
	if (routeU != routeV) s->updateRouteInfor(routeV->cour,nbMoves);
	s->updateLocationInfor(routeU->whichLocation->cour);
	if (routeU->whichLocation != routeV->whichLocation)
		s->updateLocationInfor(routeV->whichLocation->cour);

	return true;
}
bool LS::Relocate_five(){
	if (nodeU == nodeV->pre || nodeX == nodeV->pre || nodeU == nodeY || nodeX->isDepot) return false;
	if (nodeU->route->whichLocation != nodeV->route->whichLocation){
		if (nodeU->route->whichLocation->remainInven + nodeV->dem - nodeU->dem - nodeX->dem > nodeU->route->whichLocation->inventory)return false;
		if (nodeV->route->whichLocation->remainInven + nodeU->dem + nodeX->dem - nodeV->dem >nodeV->route->whichLocation->inventory)return false;
	}
	if (nodeU->route->load - nodeU->dem - nodeX->dem + nodeV->dem > data->vehCap || nodeV->route->load + nodeU->dem + nodeX->dem - nodeV->dem > data->vehCap)return false;

	moveGain=data->D[nodeUPrevIndex][nodeVIndex]+data->D[nodeVIndex][nodeXNextIndex]-data->D[nodeUPrevIndex][nodeUIndex]-data->D[nodeXIndex][nodeXNextIndex];
	moveGain+=data->D[nodeVPrevIndex][nodeUIndex]+data->D[nodeXIndex][nodeYIndex]-data->D[nodeVPrevIndex][nodeVIndex]-data->D[nodeVIndex][nodeYIndex];

	if (moveGain >-Min)return false;

	swapNode(nodeU, nodeV);
	insertNode(nodeX, nodeU);

	nbMoves++;
	searchComplete=false;
	s->updateRouteInfor(routeU->cour,nbMoves);
	if (routeU != routeV) s->updateRouteInfor(routeV->cour,nbMoves);
	s->updateLocationInfor(routeU->whichLocation->cour);
	if (routeU->whichLocation != routeV->whichLocation)
		s->updateLocationInfor(routeV->whichLocation->cour);

	return true;
}
bool LS::Relocate_six(){
	if (nodeX->isDepot || nodeY->isDepot || nodeY == nodeU->pre || nodeU == nodeY || nodeX == nodeV || nodeV == nodeX->next) return false;
	if (nodeU->route->whichLocation != nodeV->route->whichLocation){
    	if (nodeU->route->whichLocation->remainInven + nodeV->dem + nodeY->dem - nodeU->dem - nodeX->dem > nodeU->route->whichLocation->inventory) return false;
    	if (nodeV->route->whichLocation->remainInven + nodeU->dem + nodeX->dem - nodeV->dem - nodeY->dem > nodeV->route->whichLocation->inventory) return false;
	}
	if (nodeU->route->load - nodeU->dem - nodeX->dem + nodeV->dem + nodeY->dem > data->vehCap || nodeV->route->load - nodeV->dem - nodeY->dem + nodeU->dem + nodeX->dem > data->vehCap)return false;

	moveGain=data->D[nodeUPrevIndex][nodeVIndex]+data->D[nodeYIndex][nodeXNextIndex]-data->D[nodeUPrevIndex][nodeUIndex]-data->D[nodeXIndex][nodeXNextIndex];
	moveGain+=data->D[nodeVPrevIndex][nodeUIndex]+data->D[nodeXIndex][nodeYNextIndex]-data->D[nodeVPrevIndex][nodeVIndex]-data->D[nodeYIndex][nodeYNextIndex];

	if (moveGain > -Min)return false;
	swapNode(nodeU,nodeV);
	swapNode(nodeX,nodeY);

	nbMoves++;
	searchComplete=false;
	s->updateRouteInfor(routeU->cour,nbMoves);
	if (routeU != routeV) s->updateRouteInfor(routeV->cour,nbMoves);
	s->updateLocationInfor(routeU->whichLocation->cour);
	if (routeU->whichLocation != routeV->whichLocation)
		s->updateLocationInfor(routeV->whichLocation->cour);

	return true;
}

bool LS::Relocate_seven(){
    // Reject depot and overlapping-node cases.
    if (nodeX->isDepot || nodeY->isDepot || nodeY == nodeU->pre || nodeU == nodeY || nodeX == nodeV || nodeV == nodeX->next) return false;
    
    // Cross-depot exchanges must respect both depot capacities.
    if (nodeU->route->whichLocation != nodeV->route->whichLocation){
        if (nodeU->route->whichLocation->remainInven + nodeV->dem + nodeY->dem - nodeU->dem - nodeX->dem > nodeU->route->whichLocation->inventory) return false;
        if (nodeV->route->whichLocation->remainInven + nodeU->dem + nodeX->dem - nodeV->dem - nodeY->dem > nodeV->route->whichLocation->inventory) return false;
    }
    
    // Check vehicle capacities after exchanging the two blocks.
    if (nodeU->route->load - nodeU->dem - nodeX->dem + nodeV->dem + nodeY->dem > data->vehCap || nodeV->route->load - nodeV->dem - nodeY->dem + nodeU->dem + nodeX->dem > data->vehCap) return false;

    // Evaluate the replacement boundary edges.
    moveGain = data->D[nodeUPrevIndex][nodeYIndex] + data->D[nodeVIndex][nodeXNextIndex] 
             - data->D[nodeUPrevIndex][nodeUIndex] - data->D[nodeXIndex][nodeXNextIndex];
    moveGain += data->D[nodeVPrevIndex][nodeXIndex] + data->D[nodeUIndex][nodeYNextIndex] 
              - data->D[nodeVPrevIndex][nodeVIndex] - data->D[nodeYIndex][nodeYNextIndex];

    // Account for reversing U -> X and V -> Y into X -> U and Y -> V.
    moveGain += data->D[nodeYIndex][nodeVIndex] + data->D[nodeXIndex][nodeUIndex];
    moveGain -= data->D[nodeUIndex][nodeXIndex] + data->D[nodeVIndex][nodeYIndex];

    if (moveGain > -Min) return false;

    // Two swaps implement the reversed 2-2 exchange.
    swapNode(nodeX,nodeV);
    swapNode(nodeU,nodeY);

    // Refresh route and depot aggregates.
    nbMoves++;
    searchComplete=false;
    s->updateRouteInfor(routeU->cour,nbMoves);
    if (routeU != routeV) s->updateRouteInfor(routeV->cour,nbMoves);
    s->updateLocationInfor(routeU->whichLocation->cour);
    if (routeU->whichLocation != routeV->whichLocation)
        s->updateLocationInfor(routeV->whichLocation->cour);
        
    return true;
}

bool LS::Relocate_eight(){
    // The 2-opt segment must be non-overlapping and belong to one route.
    if (nodeU->route != nodeV->route) return false;
    if (nodeX->isDepot || nodeV->isDepot || nodeY->isDepot || nodeU->isDepot) return false;
    if (nodeU == nodeV || nodeX == nodeV || nodeU == nodeY || nodeX == nodeY) return false;

    // Confirm that V follows U without crossing a depot.
    bool isUbeforeV = false;
    Node* checkNode = nodeX;
    while (!checkNode->isDepot) {
        if (checkNode == nodeV) {
            isUbeforeV = true;
            break;
        }
        checkNode = checkNode->next;
    }
    if (!isUbeforeV) return false;

    // Evaluate replacement boundary edges.
    moveGain = data->D[nodeUIndex][nodeVIndex] + data->D[nodeXIndex][nodeYIndex] 
             - data->D[nodeUIndex][nodeXIndex] - data->D[nodeVIndex][nodeYIndex];

    // Accumulate every reversed internal edge for asymmetric costs.
    Node* curr = nodeX;
    while (curr != nodeV) {
        Node* nxt = curr->next;
        moveGain += data->D[nxt->cour][curr->cour] - data->D[curr->cour][nxt->cour];
        curr = nxt;
    }

    if (moveGain > -Min) return false;

    // Reverse the segment's links.
    curr = nodeX;
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
    
    // The route order and travel cost changed; load and customer count did not.
    s->updateRouteInfor(nodeU->route->cour, nbMoves);

    return true;
}

bool LS::Relocate_nine() {
    if (nodeU->route == nodeV->route) return false;
    if (nodeX->isDepot && nodeY->isDepot) return false;

    // Scan both tails with a cycle-safety bound.
    int safety_limit = data->numC + 2; 
    Node* curr = nodeX; Node* lastA = nullptr; int countA = 0; bool sameRoute = false;
    while (!curr->isDepot) {
        if (curr == nodeV || curr == nodeY) sameRoute = true;
        lastA = curr; countA++;
        if (countA > safety_limit) return false; 
        curr = curr->next;
    }
    if (sameRoute) return false; 
    Node* endDepotU = curr;

    curr = nodeY; Node* lastB = nullptr; int countB = 0;
    while (!curr->isDepot) {
        if (curr == nodeU || curr == nodeX) sameRoute = true;
        lastB = curr; countB++;
        if (countB > safety_limit) return false; 
        curr = curr->next;
    }
    if (sameRoute) return false; 
    Node* endDepotV = curr;

    // Check route and depot capacities after exchanging tails.
    double weightTailA = nodeU->route->load - nodeU->cumulatedLoad;
    double weightTailB = nodeV->route->load - nodeV->cumulatedLoad;

    if (nodeU->cumulatedLoad + weightTailB > data->vehCap) return false;
    if (nodeV->cumulatedLoad + weightTailA > data->vehCap) return false;

    if (nodeU->route->whichLocation != nodeV->route->whichLocation) {
        if (nodeU->route->whichLocation->remainInven - weightTailA + weightTailB > nodeU->route->whichLocation->inventory) return false;
        if (nodeV->route->whichLocation->remainInven - weightTailB + weightTailA > nodeV->route->whichLocation->inventory) return false;
    }

    // Evaluate directed travel cost, including reversed internal edges.
    moveGain = - data->D[nodeUIndex][nodeXIndex] - data->D[nodeVIndex][nodeYIndex];

    if (countB > 0) {
        moveGain += data->D[nodeUIndex][lastB->cour];
        moveGain += data->D[nodeYIndex][endDepotU->cour];
        moveGain -= data->D[lastB->cour][endDepotV->cour];
        Node* c = nodeY;
        while (c != endDepotV) {
            Node* nxt = c->next;
            if (nxt != endDepotV) moveGain += data->D[nxt->cour][c->cour] - data->D[c->cour][nxt->cour];
            c = nxt;
        }
    } else moveGain += data->D[nodeUIndex][endDepotU->cour];

    if (countA > 0) {
        moveGain += data->D[nodeVIndex][lastA->cour];
        moveGain += data->D[nodeXIndex][endDepotV->cour];
        moveGain -= data->D[lastA->cour][endDepotU->cour];
        Node* c = nodeX;
        while (c != endDepotU) {
            Node* nxt = c->next;
            if (nxt != endDepotU) moveGain += data->D[nxt->cour][c->cour] - data->D[c->cour][nxt->cour];
            c = nxt;
        }
    } else moveGain += data->D[nodeVIndex][endDepotV->cour];

    // Account for vehicle and depot activation changes.
    int old_nbU = nodeU->route->nbClients, old_nbV = nodeV->route->nbClients;
    int new_nbU = old_nbU - countA + countB, new_nbV = old_nbV - countB + countA;

    if (old_nbU > 0 && new_nbU == 0) {
        moveGain -= data->usingCostVeh;
        if (nodeU->route->whichLocation->splitTime == 1 && nodeU->route->whichLocation != nodeV->route->whichLocation) 
            moveGain -= data->openCostLoc[nodeU->route->whichLocation->cour];
    } else if (old_nbU == 0 && new_nbU > 0) {
        moveGain += data->usingCostVeh;
        if (!nodeU->route->whichLocation->used && nodeU->route->whichLocation != nodeV->route->whichLocation) 
            moveGain += data->openCostLoc[nodeU->route->whichLocation->cour];
    }

    if (old_nbV > 0 && new_nbV == 0) {
        moveGain -= data->usingCostVeh;
        if (nodeV->route->whichLocation->splitTime == 1 && nodeU->route->whichLocation != nodeV->route->whichLocation) 
            moveGain -= data->openCostLoc[nodeV->route->whichLocation->cour];
    } else if (old_nbV == 0 && new_nbV > 0) {
        moveGain += data->usingCostVeh;
        if (!nodeV->route->whichLocation->used && nodeU->route->whichLocation != nodeV->route->whichLocation) 
            moveGain += data->openCostLoc[nodeV->route->whichLocation->cour];
    }

    // Normalize numerical noise and admit only improving moves.
    if (std::abs(moveGain) < 1e-5) moveGain = 0.0;
    if (moveGain > -Min) return false;

    // Exchange and reverse the route tails.
    curr = nodeX; while (!curr->isDepot) { curr->route = nodeV->route; curr = curr->next; }
    curr = nodeY; while (!curr->isDepot) { curr->route = nodeU->route; curr = curr->next; }

    if (countA > 0) {
        curr = nodeX; Node* stopNode = endDepotU;
        while (curr != stopNode) {
            Node* nxt = curr->next; Node* temp = curr->next; curr->next = curr->pre; curr->pre = temp; curr = nxt;
        }
        nodeV->next = lastA; lastA->pre = nodeV;
        nodeX->next = endDepotV; endDepotV->pre = nodeX;
    } else { nodeV->next = endDepotV; endDepotV->pre = nodeV; }

    if (countB > 0) {
        curr = nodeY; Node* stopNode = endDepotV;
        while (curr != stopNode) {
            Node* nxt = curr->next; Node* temp = curr->next; curr->next = curr->pre; curr->pre = temp; curr = nxt;
        }
        nodeU->next = lastB; lastB->pre = nodeU;
        nodeY->next = endDepotU; endDepotU->pre = nodeY;
    } else { nodeU->next = endDepotU; endDepotU->pre = nodeU; }

    // Refresh route and depot aggregates.
    nodeU->route->nbClients = new_nbU; nodeV->route->nbClients = new_nbV;
    nbMoves++;
    searchComplete = false;
    
    s->updateRouteInfor(nodeU->route->cour, nbMoves);
    s->updateRouteInfor(nodeV->route->cour, nbMoves);
    s->updateLocationInfor(nodeU->route->whichLocation->cour);
    if (nodeU->route->whichLocation != nodeV->route->whichLocation) s->updateLocationInfor(nodeV->route->whichLocation->cour);

    return true;
}

bool LS::Relocate_ten() {
    if (nodeU->route == nodeV->route) return false;
    if (nodeX->isDepot && nodeY->isDepot) return false;

    // Scan both tails with a cycle-safety bound.
    int safety_limit = data->numC + 2; 
    Node* curr = nodeX; Node* lastA = nullptr; int countA = 0; bool sameRoute = false;
    while (!curr->isDepot) {
        if (curr == nodeV || curr == nodeY) sameRoute = true;
        lastA = curr; countA++;
        if (countA > safety_limit) return false; 
        curr = curr->next;
    }
    if (sameRoute) return false; 
    Node* endDepotU = curr;

    curr = nodeY; Node* lastB = nullptr; int countB = 0;
    while (!curr->isDepot) {
        if (curr == nodeU || curr == nodeX) sameRoute = true;
        lastB = curr; countB++;
        if (countB > safety_limit) return false; 
        curr = curr->next;
    }
    if (sameRoute) return false; 
    Node* endDepotV = curr;

    // Check route and depot capacities after exchanging tails.
    double weightTailA = nodeU->route->load - nodeU->cumulatedLoad;
    double weightTailB = nodeV->route->load - nodeV->cumulatedLoad;

    if (nodeU->cumulatedLoad + weightTailB > data->vehCap) return false;
    if (nodeV->cumulatedLoad + weightTailA > data->vehCap) return false;

    if (nodeU->route->whichLocation != nodeV->route->whichLocation) {
        if (nodeU->route->whichLocation->remainInven - weightTailA + weightTailB > nodeU->route->whichLocation->inventory) return false;
        if (nodeV->route->whichLocation->remainInven - weightTailB + weightTailA > nodeV->route->whichLocation->inventory) return false;
    }

    // Evaluate the directed travel-cost change.
    moveGain = - data->D[nodeUIndex][nodeXIndex] - data->D[nodeVIndex][nodeYIndex];

    if (countB > 0) {
        moveGain += data->D[nodeUIndex][nodeYIndex];
        moveGain -= data->D[lastB->cour][endDepotV->cour];
        moveGain += data->D[lastB->cour][endDepotU->cour];
    } else moveGain += data->D[nodeUIndex][endDepotU->cour];

    if (countA > 0) {
        moveGain += data->D[nodeVIndex][nodeXIndex];
        moveGain -= data->D[lastA->cour][endDepotU->cour];
        moveGain += data->D[lastA->cour][endDepotV->cour];
    } else moveGain += data->D[nodeVIndex][endDepotV->cour];

    // Account for vehicle and depot activation changes.
    int old_nbU = nodeU->route->nbClients, old_nbV = nodeV->route->nbClients;
    int new_nbU = old_nbU - countA + countB, new_nbV = old_nbV - countB + countA;

    if (old_nbU > 0 && new_nbU == 0) {
        moveGain -= data->usingCostVeh;
        if (nodeU->route->whichLocation->splitTime == 1 && nodeU->route->whichLocation != nodeV->route->whichLocation) 
            moveGain -= data->openCostLoc[nodeU->route->whichLocation->cour];
    } else if (old_nbU == 0 && new_nbU > 0) {
        moveGain += data->usingCostVeh;
        if (!nodeU->route->whichLocation->used && nodeU->route->whichLocation != nodeV->route->whichLocation) 
            moveGain += data->openCostLoc[nodeU->route->whichLocation->cour];
    }

    if (old_nbV > 0 && new_nbV == 0) {
        moveGain -= data->usingCostVeh;
        if (nodeV->route->whichLocation->splitTime == 1 && nodeU->route->whichLocation != nodeV->route->whichLocation) 
            moveGain -= data->openCostLoc[nodeV->route->whichLocation->cour];
    } else if (old_nbV == 0 && new_nbV > 0) {
        moveGain += data->usingCostVeh;
        if (!nodeV->route->whichLocation->used && nodeU->route->whichLocation != nodeV->route->whichLocation) 
            moveGain += data->openCostLoc[nodeV->route->whichLocation->cour];
    }

    // Normalize numerical noise and admit only improving moves.
    if (std::abs(moveGain) < 1e-5) moveGain = 0.0;
    if (moveGain > -Min) return false;

    // Exchange the route tails without reversing them.
    curr = nodeX; while (!curr->isDepot) { curr->route = nodeV->route; curr = curr->next; }
    curr = nodeY; while (!curr->isDepot) { curr->route = nodeU->route; curr = curr->next; }

    if (countB > 0) { nodeU->next = nodeY; nodeY->pre = nodeU; lastB->next = endDepotU; endDepotU->pre = lastB; } 
    else { nodeU->next = endDepotU; endDepotU->pre = nodeU; }

    if (countA > 0) { nodeV->next = nodeX; nodeX->pre = nodeV; lastA->next = endDepotV; endDepotV->pre = lastA; } 
    else { nodeV->next = endDepotV; endDepotV->pre = nodeV; }

    // Refresh route and depot aggregates.
    nodeU->route->nbClients = new_nbU; nodeV->route->nbClients = new_nbV;
    nbMoves++;
    searchComplete = false;

    s->updateRouteInfor(nodeU->route->cour, nbMoves);
    s->updateRouteInfor(nodeV->route->cour, nbMoves);
    s->updateLocationInfor(nodeU->route->whichLocation->cour);
    if (nodeU->route->whichLocation != nodeV->route->whichLocation) s->updateLocationInfor(nodeV->route->whichLocation->cour);

    return true;
}

void LS::setLocalVariableRouteU(){
	routeU=nodeU->route;
	nodeX=nodeU->next;
	nodeXNextIndex=nodeX->next->cour;
	nodeUIndex = nodeU->cour;
	nodeUPrevIndex = nodeU->pre->cour;
	nodeXIndex = nodeX->cour;
	loadU=nodeU->dem;
	loadX=nodeX->dem;
}
void LS::setLocalVariableRouteV(){
	routeV=nodeV->route;
	nodeY=nodeV->next;
	nodeYNextIndex=nodeY->next->cour;//the next two city.  U-X-this
	nodeVIndex=nodeV->cour;
	nodeVPrevIndex=nodeV->pre->cour;
	nodeYIndex=nodeY->cour;
	loadV=nodeV->dem;
	loadY=nodeY->dem;
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
void LS::swapNode(Node *U, Node *V){
	Node * myVPred = V->pre;
	Node * myVSuiv = V->next;
	Node * myUPred = U->pre;
	Node * myUSuiv = U->next;
	Route * myRouteU = U->route;
	Route * myRouteV = V->route;

	myUPred->next = V;
	myUSuiv->pre = V;
	myVPred->next = U;
	myVSuiv->pre = U;

	U->pre = myVPred;
	U->next = myVSuiv;
	V->pre = myUPred;
	V->next = myUSuiv;

	U->route = myRouteV;
	V->route = myRouteU;
}
void LS::loadSolution(){
	nbMoves = 0;
	for (int i=0;i<numV;i++)
		s->client[i].whenLastTestedRI = -1;
	for (int i=numV;i<numC;i++){
		for (int j=0;j<s->location[i-numV].maxVisit; j++){
			s->location[i-numV].depots[j].route->whenLastModified = -1;
		}
	}
}
void LS::local_search_run(Individual *s,double penaltyDepot, double penaltyVeh){
	this->penaltyDepot = penaltyDepot;
	this->penaltyDepot=Max;// no infeasible
	this->penaltyVeh = penaltyVeh;
	this->penaltyVeh = Max;
	this->s=s;
//	s->isRight();
	loadSolution();
	srun();
	s->evaluationDis();
	s->isRight();

//	s->outputSolution();



	int totalInv = 0;
	std::set<std::pair<int, int>>order;

	for (int i=0;i<numL;i++){
		if (s->location[i].used){
			order.insert({s->location[i].inventory,i});
			totalInv += s->location[i].inventory;
		}
	}
	if (totalInv > data->totalDemand){
		auto it = order.begin();
		if (totalInv - it->first > data->totalDemand)
			data->loopp++;
	}


}

/************************************************************************************************/
/************************************************************************************************/
/************************************************************************************************/
/************************************************************************************************/
/************************************************************************************************/
void LS::removeLocation(int removeLoc){
	for (int i=0;i<numV;i++)auxArray1[i]=0;
	Node *p;
	for (int i=0;i<s->location[removeLoc].maxVisit;i++){
		if (s->location[removeLoc].depots[i].route->used){
			p = s->location[removeLoc].depots[i].next;
			while(!p->isDepot){
				listRemove.push_back(p->cour);
				p = p->next;
			}
			s->location[removeLoc].depots[i].next = &s->location[removeLoc].endDepots[i];
			s->location[removeLoc].endDepots[i].pre = &s->location[removeLoc].depots[i];
			s->updateRouteInfor(s->location[removeLoc].depots[i].route->cour, -1);
		}
	}
	int ll = removeLoc + numV;
	s->updateLocationInfor(ll);
	//
	for (int i=0;i<(int)listRemove.size();i++){
		p = &s->client[listRemove[i]];
		p->pre = NULL;
		p->next = NULL;
		auxArray1[p->cour] = 1;
	}
	//
	RandomShuffle(listRemove); // randomize all removed customers
	double delta, min_delta;
	Node *node, *nodeU, *pos;bool isFirst;
	while ((int)listRemove.size() != 0){
		min_delta = Max; //listRemove
		pos = NULL;
		node = &s->client[listRemove[0]];
		for (int i=0;i<numL;i++){
			if (s->location[i].used){
				for (int j=0;j<s->location[i].maxVisit;j++){
					if (s->location[i].depots[j].route->used && (s->location[i].depots[j].route->load + node->dem <= data->vehCap) && (s->location[i].remainInven + node->dem <= s->location[i].inventory)){
						nodeU = &s->location[i].depots[j]; isFirst = true;
						while(isFirst || !nodeU->isDepot){
							delta = data->D[nodeU->cour][node->cour] + data->D[node->cour][nodeU->next->cour] - data->D[nodeU->cour][nodeU->next->cour];
							if (min_delta > delta){
								min_delta = delta;
								pos = nodeU;
							}
							nodeU = nodeU->next;
							isFirst = false;
						}
					}
				}
			}
		}
		//
		pos->next->pre = node;
		node->next = pos->next;
		pos->next = node;
		node->pre = pos;
		node->route = pos->route;
		//
		auxArray1[node->cour] = 0;
		listRemove.erase(listRemove.begin());
		s->updateRouteInfor(pos->route->cour,-1);
		s->updateLocationInfor(pos->route->whichLocation->cour);
	}
}
void LS::removeRoute(int removeRou){
	for (int i=0;i<numV;i++)auxArray1[i] = 0;
	Node *p = s->route[removeRou].depot->next;
	while(!p->isDepot){
		listRemove.push_back(p->cour);
		p = p->next;
	}
	s->route[removeRou].depot->next = s->route[removeRou].endDepot;
	s->route[removeRou].endDepot->pre = s->route[removeRou].depot;
	s->updateRouteInfor(removeRou, -1);
	int ll = s->route[removeRou].whichLocation->cour;
	s->updateLocationInfor(ll);
	//
	for (int i=0;i<(int)listRemove.size();i++){
		p = &s->client[listRemove[i]];
		p->pre = NULL;
		p->next = NULL;
		p->route = NULL;
		auxArray1[p->cour] = 1;
	}
	RandomShuffle(listRemove);  // randomize all removed customers
	double delta, min_delta;
	Node *node, *nodeU, *pos;
	while ((int)listRemove.size() != 0){
		min_delta = Max; //listRemove
		pos = NULL;
		node = &s->client[listRemove[0]];
		for (int i=0;i<(int)data->edgeNode[node->cour].size();i++){
			if (data->edgeNode[node->cour][i] < numV){
				nodeU = &s->client[data->edgeNode[node->cour][i]];
				if (nodeU->route != NULL){
					if (nodeU->route->load + node->dem <= data->vehCap && nodeU->route->whichLocation->remainInven + node->dem <= nodeU->route->whichLocation->inventory){
						delta = data->D[nodeU->cour][node->cour] + data->D[node->cour][nodeU->next->cour] - data->D[nodeU->cour][nodeU->next->cour];
						if (delta < min_delta){
							min_delta = delta;
							pos = nodeU;
						}
					}
				}
			}
		}
		if (min_delta == Max){
			for (int i=0;i<(int)data->edgeNode[node->cour].size();i++){
				if (data->edgeNode[node->cour][i] < numV){
					nodeU = &s->client[data->edgeNode[node->cour][i]];
					if (nodeU->route != NULL){
						delta = data->D[nodeU->cour][node->cour] + data->D[node->cour][nodeU->next->cour] - data->D[nodeU->cour][nodeU->next->cour];
						if (delta < min_delta){
							min_delta = delta;
							pos = nodeU;
						}
					}
				}
			}
			if (min_delta == Max){
				for (int i=0;i<numV;i++){
					if (s->client[i].route != NULL){
						nodeU = &s->client[i];
						delta = data->D[nodeU->cour][node->cour] + data->D[node->cour][nodeU->next->cour] - data->D[nodeU->cour][nodeU->next->cour];
						if (delta < min_delta){
							min_delta = delta;
							pos = nodeU;
						}
					}
				}
			}
		}
		//
		if (min_delta == Max)std::cout<<"report a bug at removing routes"<<std::endl;
//		s->outputSolution();
		pos->next->pre = node;
		node->next = pos->next;
		pos->next = node;
		node->pre = pos;
		node->route = pos->route;
		//
		auxArray1[node->cour] = 0;
		listRemove.erase(listRemove.begin());
		s->updateRouteInfor(pos->route->cour,-1);
		s->updateLocationInfor(pos->route->whichLocation->cour);
	}
	/***************************/
	//repair the infeasible solution
	s->evaluationDis();
	repair->repairForInitialSolution(s);
}
void LS::removeRoutesAndLocations(Individual *s){
	//
	this->s = s;
	s->getSaturationDegree();
//	s->outputSolution();
/*	int sumLoadLoc;
	while(1){
		sumLoadLoc = 0;locSat.clear();
		for (int i=0;i<numL;i++){
			if (s->location[i].used){
				sumLoadLoc += s->location[i].inventory;
				locSat.push_back({(double)s->location[i].remainInven / (double)s->location[i].inventory, i});
			}
		}
		std::sort(locSat.begin(),locSat.end());
		//
		if (sumLoadLoc - s->location[locSat[0].second].inventory >= data->totalDemand){// less saturated
			s->outputSolution();
			removeLocation(locSat[0].second);
		}
		else break;
	}
	*/
	int tarRoute;
	int sumLoadRou;
	while(1){// existing non-saturated routes
		auto it = s->saturatRoute.begin();
		if (it->first < 0.7){// saturated parameter
			tarRoute = it->second;// the order of the route.
			sumLoadRou = ((int)s->saturatRoute.size() - 1) * data->vehCap;
			if (sumLoadRou > 1. * data->totalDemand)
				removeRoute(tarRoute);
		}
		else
			break;
		break;
		s->getSaturationDegree();
	}
	s->decideFeasible();
	if (!s->isFeasible)return;
	s->evaluationDis();
	s->isRight();

}
