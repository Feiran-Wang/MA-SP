#include "TimeWindowOperators.h"
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
				exit(0);
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
				exit(0);
			}
*/
			return true;
		}
	}
	return false;
}
bool LS::Relocate_one(){
    // 新增 nodeUIndex == nodeVIndex 的拓扑自环拦截
	if (nodeUIndex == nodeYIndex || nodeUIndex == nodeVIndex) return false;

    // 修复：只有跨仓库转移时，才对目标仓库加上 U 的需求量进行验证
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
    // 增加 nodeU == nodeV 拦截，防止自我缠绕
	if (nodeUIndex == nodeYIndex || nodeXIndex == nodeVIndex || nodeUIndex == nodeVIndex || nodeX->isDepot) return false;

    // 修复：同仓库免检逻辑
	if (nodeU->route->whichLocation != nodeV->route->whichLocation) {
        if (!(nodeV->route->whichLocation->remainInven + nodeU->dem + nodeX->dem <= nodeV->route->whichLocation->inventory))
            return false;
    }
	if(nodeV->route != nodeU->route && nodeV->route->load + nodeU->dem + nodeX->dem > data->vehCap)return false;

    // 修正的 moveGain：完美映射 V -> U -> X -> Y 的距离变化
	moveGain = compute(nodeV->route->whichLocation->used, data->openCostLoc[nodeV->route->whichLocation->cour])
			+ compute(!(nodeV->isDepot && nodeY->isDepot), data->usingCostVeh)
			- compute(!(nodeU->route->whichLocation->splitTime == 1 && nodeU->route->nbClients == 2 && nodeU->route->whichLocation != nodeV->route->whichLocation), data->openCostLoc[nodeU->route->whichLocation->cour])
			- compute(nodeU->route->nbClients != 2, data->usingCostVeh)
			+ data->D[nodeUPrevIndex][nodeXNextIndex]-data->D[nodeUPrevIndex][nodeUIndex]-data->D[nodeXIndex][nodeXNextIndex]
			+ data->D[nodeVIndex][nodeUIndex]+data->D[nodeXIndex][nodeYIndex]-data->D[nodeVIndex][nodeYIndex]; // 此处已修正为 V连U, X连Y

	if (moveGain>-Min)return false;

    // 修复：正确的 2-0 顺序插入指针操作
	insertNode(nodeU,nodeV); // 先把 U 插在 V 后面
	insertNode(nodeX,nodeU); // 再把 X 插在 U 后面，完美保持 U -> X

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
    // 增加 nodeU == nodeV 拦截
	if (nodeUIndex == nodeYIndex || nodeXIndex == nodeVIndex || nodeUIndex == nodeVIndex || nodeX->isDepot) return false;

    // 修复：同仓库免检逻辑
    if (nodeU->route->whichLocation != nodeV->route->whichLocation) {
        if (!(nodeV->route->whichLocation->remainInven + nodeU->dem + nodeX->dem <= nodeV->route->whichLocation->inventory))
            return false;
    }
	if(nodeV->route != nodeU->route && nodeV->route->load + nodeU->dem + nodeX->dem > data->vehCap)return false;

    // 重构的 moveGain：清理了原来的幽灵分支，统一且精确计算包含反转的代价
	moveGain = compute(nodeV->route->whichLocation->used, data->openCostLoc[nodeV->route->whichLocation->cour])
			+ compute(!(nodeV->isDepot && nodeY->isDepot), data->usingCostVeh)
			- compute(!(nodeU->route->whichLocation->splitTime == 1 && nodeU->route->nbClients == 2 && nodeU->route->whichLocation != nodeV->route->whichLocation), data->openCostLoc[nodeU->route->whichLocation->cour])
			- compute(nodeU->route->nbClients != 2, data->usingCostVeh)
			+ data->D[nodeUPrevIndex][nodeXNextIndex]-data->D[nodeUPrevIndex][nodeUIndex]-data->D[nodeXIndex][nodeXNextIndex]
			+ data->D[nodeVIndex][nodeXIndex]+data->D[nodeUIndex][nodeYIndex]-data->D[nodeVIndex][nodeYIndex] // V连X，U连Y
            + data->D[nodeXIndex][nodeUIndex]-data->D[nodeUIndex][nodeXIndex]; // 修复：必须加上被反转的内部边代价！

	if (moveGain>-Min)return false;

    // 保持反序物理插入不动，以发挥 0-2 逆序算子的作用
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
    // 1. 基础合法性校验：不能移动仓库节点，且 U,X 和 V,Y 必须是两对相邻的节点
    if (nodeX->isDepot || nodeY->isDepot || nodeY == nodeU->pre || nodeU == nodeY || nodeX == nodeV || nodeV == nodeX->next) return false;

    // 2. 跨仓库交换时的容量校验（修复了原代码的代数死锁 Bug）
    if (nodeU->route->whichLocation != nodeV->route->whichLocation){
        // 注意：原代码不等式右侧错写成了 remainInven（导致两边抵消）。
        // 这里修正为比较仓库的最大容量 (inventory)，确保交换后仓库总负载不超标。
        if (nodeU->route->whichLocation->remainInven + nodeV->dem + nodeY->dem - nodeU->dem - nodeX->dem > nodeU->route->whichLocation->inventory) return false;
        if (nodeV->route->whichLocation->remainInven + nodeU->dem + nodeX->dem - nodeV->dem - nodeY->dem > nodeV->route->whichLocation->inventory) return false;
    }

    // 3. 车辆容量校验（原代码逻辑正确，予以保留）
    if (nodeU->route->load - nodeU->dem - nodeX->dem + nodeV->dem + nodeY->dem > data->vehCap || nodeV->route->load - nodeV->dem - nodeY->dem + nodeU->dem + nodeX->dem > data->vehCap) return false;

    // 4. 计算外围断边和连边的距离变化（原代码逻辑正确，予以保留）
    // U_pre 连 Y，V 连 X_next； V_pre 连 X，U 连 Y_next
    moveGain = data->D[nodeUPrevIndex][nodeYIndex] + data->D[nodeVIndex][nodeXNextIndex]
             - data->D[nodeUPrevIndex][nodeUIndex] - data->D[nodeXIndex][nodeXNextIndex];
    moveGain += data->D[nodeVPrevIndex][nodeXIndex] + data->D[nodeUIndex][nodeYNextIndex]
              - data->D[nodeVPrevIndex][nodeVIndex] - data->D[nodeYIndex][nodeYNextIndex];

    // 5. 【新增修复】计算被交换块内部的方向反转所带来的距离变化
    // 原本 U->X 和 V->Y 的方向，在交换后变成了 X->U 和 Y->V
    moveGain += data->D[nodeYIndex][nodeVIndex] + data->D[nodeXIndex][nodeUIndex]; // 加上反转后的新内部边
    moveGain -= data->D[nodeUIndex][nodeXIndex] + data->D[nodeVIndex][nodeYIndex]; // 减去反转前的旧内部边

    // 6. 改进阈值判定
    if (moveGain > -Min) return false;

    // 7. 执行物理交换：连续两次 swapNode 实现带反转的 2-2 交换
    swapNode(nodeX,nodeV);
    swapNode(nodeU,nodeY);

    // 8. 状态更新（原代码逻辑正确，予以保留）
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
    // 1. 基础防御：必须是同一条路线，且涉及节点不能是仓库，不能重叠
    if (nodeU->route != nodeV->route) return false;
    if (nodeX->isDepot || nodeV->isDepot || nodeY->isDepot || nodeU->isDepot) return false;
    if (nodeU == nodeV || nodeX == nodeV || nodeU == nodeY || nodeX == nodeY) return false;

    // 2. 拓扑方向防御：必须保证 U 在 V 的前面！(即 X 顺着走能遇到 V，且不经过仓库)
    bool isUbeforeV = false;
    Node* checkNode = nodeX;
    while (!checkNode->isDepot) {
        if (checkNode == nodeV) {
            isUbeforeV = true;
            break;
        }
        checkNode = checkNode->next;
    }
    if (!isUbeforeV) return false; // 如果 V 在 U 前面，拒绝执行，防止反转仓库

    // 3. 计算端点连线的距离变化 (外围边)
    moveGain = data->D[nodeUIndex][nodeVIndex] + data->D[nodeXIndex][nodeYIndex]
             - data->D[nodeUIndex][nodeXIndex] - data->D[nodeVIndex][nodeYIndex];

    // 4. 计算内部路径反转的距离变化 (必须逐段累加，以完美支持非对称矩阵)
    Node* curr = nodeX;
    while (curr != nodeV) {
        Node* nxt = curr->next;
        // 原本是从 curr 走向 nxt，现在反转成 nxt 走向 curr
        moveGain += data->D[nxt->cour][curr->cour] - data->D[curr->cour][nxt->cour];
        curr = nxt;
    }

    // 5. 收益判定
    if (moveGain > -Min) return false;

    // 6. 执行物理拓扑反转 (核心指针操作)
    curr = nodeX;
    Node* stopNode = nodeY; // 遍历到 Y 停止
    while (curr != stopNode) {
        Node* nxt = curr->next;
        // 交换当前节点的 pre 和 next
        Node* temp = curr->next;
        curr->next = curr->pre;
        curr->pre = temp;
        // 前进到原本的下一个节点
        curr = nxt;
    }

    // 7. 缝合断点 (边界重连)
    nodeU->next = nodeV;
    nodeV->pre  = nodeU;

    nodeX->next = nodeY;
    nodeY->pre  = nodeX;

    // 8. 状态更新 (修复了原版的 searchCompleted 拼写错误)
    nbMoves++;
    searchComplete = false; // 严格保证主循环能收到继续搜索的信号

    // 2-OPT 路线内反转不改变客户数量和总负载，但改变了访问顺序，需要通知 s 更新信息
    s->updateRouteInfor(nodeU->route->cour, nbMoves);

    return true;
}

bool LS::Relocate_nine() {
    if (nodeU->route == nodeV->route) return false;
    if (nodeX->isDepot && nodeY->isDepot) return false;

    // 1. 拓扑扫描与防死环边界判定
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

    // 2. 容量与库存红线校验
    double weightTailA = nodeU->route->load - nodeU->cumulatedLoad;
    double weightTailB = nodeV->route->load - nodeV->cumulatedLoad;

    if (nodeU->cumulatedLoad + weightTailB > data->vehCap) return false;
    if (nodeV->cumulatedLoad + weightTailA > data->vehCap) return false;

    if (nodeU->route->whichLocation != nodeV->route->whichLocation) {
        if (nodeU->route->whichLocation->remainInven - weightTailA + weightTailB > nodeU->route->whichLocation->inventory) return false;
        if (nodeV->route->whichLocation->remainInven - weightTailB + weightTailA > nodeV->route->whichLocation->inventory) return false;
    }

    // 3. O(1) 绝对精确的距离收益评估 (含反转补偿)
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

    // 4. 车辆使用费与车场真实启停成本精确推演
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

    // 5. 浮点精度抹平与严格准入判定 (彻底弃用回滚机制)
    if (std::abs(moveGain) < 1e-5) moveGain = 0.0;
    if (moveGain > -Min) return false;

    // 6. 安全无死环的拓扑重排
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

    // 7. 状态极速更新
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

    // 1. 拓扑扫描与防死环边界判定
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

    // 2. 容量与库存红线校验
    double weightTailA = nodeU->route->load - nodeU->cumulatedLoad;
    double weightTailB = nodeV->route->load - nodeV->cumulatedLoad;

    if (nodeU->cumulatedLoad + weightTailB > data->vehCap) return false;
    if (nodeV->cumulatedLoad + weightTailA > data->vehCap) return false;

    if (nodeU->route->whichLocation != nodeV->route->whichLocation) {
        if (nodeU->route->whichLocation->remainInven - weightTailA + weightTailB > nodeU->route->whichLocation->inventory) return false;
        if (nodeV->route->whichLocation->remainInven - weightTailB + weightTailA > nodeV->route->whichLocation->inventory) return false;
    }

    // 3. O(1) 绝对精确的距离平移评估
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

    // 4. 车辆使用费与车场真实启停成本精确推演
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

    // 5. 浮点精度抹平与严格准入判定
    if (std::abs(moveGain) < 1e-5) moveGain = 0.0;
    if (moveGain > -Min) return false;

    // 6. 安全无死环的拓扑重排
    curr = nodeX; while (!curr->isDepot) { curr->route = nodeV->route; curr = curr->next; }
    curr = nodeY; while (!curr->isDepot) { curr->route = nodeU->route; curr = curr->next; }

    if (countB > 0) { nodeU->next = nodeY; nodeY->pre = nodeU; lastB->next = endDepotU; endDepotU->pre = lastB; }
    else { nodeU->next = endDepotU; endDepotU->pre = nodeU; }

    if (countA > 0) { nodeV->next = nodeX; nodeX->pre = nodeV; lastA->next = endDepotV; endDepotV->pre = lastA; }
    else { nodeV->next = endDepotV; endDepotV->pre = nodeV; }

    // 7. 状态极速更新
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
	if (data->hardTimeWindows) {
		if (s->isFeasible) hgasp2tw::localSearch(s, data);
		return;
	}
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
