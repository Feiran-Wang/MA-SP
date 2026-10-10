#include "TimeWindowOperators.h"
#include "Individual.h"
#include "RoutePool.h"

Individual::Individual() {
}
Individual::~Individual() {
	delete [] client;
	for (int i=0; i< numL;i++){
		delete [] location[i].depots;
		delete [] location[i].endDepots;
	}
	delete [] location;
	delete [] route;
	delete [] checkArray;
}
void Individual::define(read_data *data){
	this->data=data;
	this->numV=data->numV;
	this->numL = data->numL;
	this->numC = data->numC;
	client=new Node [numV];
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
	location = new NodeDepot [numL];
	maxVnum=0;
	for (int i=0; i<numL; i++){
		location[i].used = false;
		location[i].cour = i+numV;
		location[i].inventory = data->invLoc[i+numV];
		location[i].remainInven = 0;
		location[i].splitTime = 0;
		int numOfVeh;
	//	if (data->totalDemand < location[i].inventory){
			numOfVeh =ceil(1.2*data->totalDemand/data->vehCap)+5;
	//	}
		if (location[i].inventory > data->totalDemand) location[i].iscapCon = false;// to decide the constraint in locations
		else location[i].iscapCon = true;
		location[i].maxVisit = data->safeRouteLimit(i);
		// Hard windows may require one vehicle per customer, regardless of demand.
		if (data->hardTimeWindows) location[i].maxVisit = numV;
		maxVnum += location[i].maxVisit;
		location[i].depots = new Node [location[i].maxVisit];
		location[i].endDepots = new Node [location[i].maxVisit];
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
	route=new Route [maxVnum];// the maxVnum is the maximum vehicles;
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
	checkArray = new int [numV];
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
					totalDis += data->D[p->next->cour][p->cour];
					p=p->next;
					while(1){
						p=p->next;
						if (p->isDepot){
							totalDis += data->D[p->pre->cour][p->cour];
							break;
						}
						totalDis += data->D[p->cour][p->pre->cour];
					}
				}
			}
		}
	}

	fit = totalLoc + totalVeh + totalDis;
	dis = totalDis;

	data->probFixedCost = totalLoc / fit;
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
		//
	bool isFirst = true;
	while(!mynode->isDepot || isFirst){
		mynode=mynode->next;
		mynode->position=mynode->pre->position+1;
		mynode->cumulatedLoad=mynode->dem+mynode->pre->cumulatedLoad;
		mynode->cumulatedDis=data->D[mynode->cour][mynode->pre->cour]+mynode->pre->cumulatedDis;
		isFirst = false;
	}
	route[r].dis=mynode->cumulatedDis + data->D[mynode->cour][mynode->next->cour];
	route[r].load=mynode->cumulatedLoad;
	route[r].nbClients=mynode->position;
	if (data->hardTimeWindows) route[r].nbClients = mynode->position - 1;

//		NodeDepot *loc = route[r].whichLocation;// update the location inventory
//		loc->remainInven=0;
//		for (int i=0; i<loc->splitTime;i++)
//			loc->remainInven += loc->depots[i].route->load;
	route[r].whenLastModified = nbMoves;
	if (data->hardTimeWindows ? route[r].depot->next->isDepot : route[r].load == 0)
		route[r].whichLocation->emptyRoutes.insert(route[r].cour);
	else
		route[r].whichLocation->emptyRoutes.erase(route[r].cour);
}
void Individual:: updateLocationInfor(int &ll){
	int l = ll - numV;
	location[l].splitTime = 0;
	location[l].remainInven = 0;
	for (int i=0; i<location[l].maxVisit; i++){
		if (data->hardTimeWindows ? !location[l].depots[i].next->isDepot : location[l].depots[i].route->load > 0){
			location[l].remainInven += location[l].depots[i].route->load;
			location[l].depots[i].route->used = true;
			location[l].splitTime++;
		}
		else location[l].depots[i].route->used = false;
	}
//	if (location[l].remainInven > location[l].inventory)std::cout<<"report a wrong "
	if (data->hardTimeWindows ? location[l].splitTime == 0 : location[l].remainInven == 0){
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
	if (data->hardTimeWindows) {
		if (!masptw::validate(this, data)) throw std::runtime_error("invalid hard-time-window solution");
		return;
	}
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
				totalDis += data->D[p->next->cour][p->cour];
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
					std::cout<<"The route "<<p->route->cour<<" capacity is wrong"<<std::endl;
					exit(EXIT_FAILURE);
				}
				capLoc += capVeh;
				totalDis += data->D[p->cour][p->next->cour];
			}
			if (capLoc != p->route->whichLocation->remainInven || capLoc > location[loc-numV].inventory){
				std::cout<<"The location "<<p->route->whichLocation->cour<<" capacity is wrong"<<std::endl;
				exit(EXIT_FAILURE);
			}
		}
	}
	for (int i=0;i<numV;i++)
		if (checkArray[i] != 1){
			std::cout<<"The vertex "<<i<<"  is wrong"<<std::endl;
			exit(EXIT_FAILURE);
		}
	if (! data->equalTwoVar(totalDis, dis)){
		std::cout<<"The traveling distance is wrong"<<std::endl;
		exit(EXIT_FAILURE);
	}
	if (! data->equalTwoVar(totalFit+ totalDis, fit)){
		std::cout<<"The fitness is wrong"<<std::endl;
		exit(EXIT_FAILURE);
	}
}
void Individual::decideFeasible(){
	if (data->hardTimeWindows) { isFeasible = masptw::validate(this, data); return; }
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
void Individual::writeStudyWitness(const std::string& path) const {
    std::ofstream out;
    out.exceptions(std::ios::failbit | std::ios::badbit);
    out.open(path);
    out << std::setprecision(17) << (data->hardTimeWindows
        ? "# Set F study candidate; requires independent validation\n"
        : "# Set S study candidate; requires independent validation\n")
        << "The objective is:\n" << fit << '\n'
        << "The running time is:\n" << static_cast<double>(clock()) / CLOCKS_PER_SEC << '\n';
    for (int l = 0; l < numL; ++l) if (location[l].used) {
        out << "Location: " << l << "(load: " << location[l].remainInven << ")\n";
        int count = 0;
        for (int k = 0; k < location[l].maxVisit; ++k) {
            const Node* start = &location[l].depots[k];
            if (!start->route->used) continue;
            out << "Route " << count++ << "(load: " << start->route->load << ") ";
            for (const Node* node = start->next; !node->isDepot; node = node->next) out << node->cour << ' ';
            out << '\n';
        }
    }
    out << "The actual cost is: " << fit << '\n';
    out.close();
}

void Individual::outputBestSolution(){
	if (data->hardTimeWindows) isRight();
	std::cout<<"----------Write Solution With Value  "<<fit<<" In: "<<data->pathSolution<<std::endl;
	std::ofstream myfile(data->pathSolution);
	myfile.exceptions(std::ios::failbit | std::ios::badbit);
	if (data->experimentMode && !data->hardTimeWindows) myfile << std::setprecision(17)
        << "# MA-SP Set S study; native instance metric; internal zero-based IDs\n";
	if (data->hardTimeWindows) myfile << std::setprecision(17)
		<< "# MA-SP Set F; hard windows; floor-euclidean cost/time; internal zero-based IDs\n";
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
					totalDis += data->D[p->next->cour][p->cour];
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
						std::cout<<"The route "<<p->route->cour<<" capacity is wrong"<<std::endl;
						exit(EXIT_FAILURE);
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
    if (data->hardTimeWindows) {
        std::vector<int> visits(numV, 0), loads(numL, 0), counts(numL, 0);
        for (const auto& sr : spRoutes) {
            if (sr.locId < 0 || sr.locId >= numL || sr.sequence.empty()
                || !data->routeTimeFeasible(sr.locId, sr.sequence))
                throw std::runtime_error("invalid Set F route before rebuild");
            int load = 0;
            for (int customer : sr.sequence) {
                if (customer < 0 || customer >= numV || ++visits[customer] != 1)
                    throw std::runtime_error("invalid Set F customer coverage before rebuild");
                load += data->demCus[customer];
            }
            loads[sr.locId] += load;
            if (load > data->vehCap || loads[sr.locId] > data->invLoc[sr.locId + numV]
                || ++counts[sr.locId] > location[sr.locId].maxVisit)
                throw std::runtime_error("invalid Set F capacity before rebuild");
        }
        for (int visit : visits) if (visit != 1)
            throw std::runtime_error("incomplete Set F customer coverage before rebuild");
    }
    // 1. 彻底清空当前个体的旧状态
    // 调用原生的初始化函数，这会重置所有的 pre/next 指针、route 状态以及 capacity 记录。
    this->initilization();

    // 2. 遍历 Gurobi 传回的每一条独立路线
    for (size_t r = 0; r < spRoutes.size(); ++r) {
        const StandaloneRoute& sRoute = spRoutes[r];
        int locId = sRoute.locId; // 获取场站索引 (0 到 data->numL - 1)

        // 场站被启用
        this->location[locId].used = true;

        // 获取该场站当前可用的空闲槽位 (splitTime)
        // 在原代码逻辑中，splitTime 表示该场站已经分派了多少条路线
        int currentSplitTime = this->location[locId].splitTime;

        // 防呆保护：确保不超过场站的路线数量上限 (通常是车辆数上限)
        if (currentSplitTime >= this->location[locId].maxVisit) {
            std::cerr << "[Error] Depot " << locId << " exceeded maxVisit capacity!" << std::endl;
            continue;
        }

        // 提取起始虚拟节点和终止虚拟节点
        Node* startDepot = &this->location[locId].depots[currentSplitTime];
        Node* endDepot   = &this->location[locId].endDepots[currentSplitTime];
        Route* currentRoute = startDepot->route;

        // 激活当前 Route
        currentRoute->used = true;
        // 注意：原代码的 route 数组中可能需要标记这条 route 被占用
        // 这里依赖底层逻辑，通常通过移除 emptyRoutes 中的记录来实现
        this->location[locId].emptyRoutes.erase(currentRoute->cour);

        // 3. 编织双向链表拓扑
        Node* previousNode = startDepot;

        for (int custId : sRoute.sequence) {
            Node* currentNode = &this->client[custId];

            // 建立与前一个节点的双向连接
            previousNode->next = currentNode;
            currentNode->pre = previousNode;

            // 绑定所属的 Route
            currentNode->route = currentRoute;

            // 标记节点被访问 (防止评估时漏掉)
            // 原代码可能没有显式的 isVisited 标记，而是靠链表遍历。

            previousNode = currentNode;
        }

        // 闭合路线：将最后一个客户连接回对应的 endDepot
        previousNode->next = endDepot;
        endDepot->pre = previousNode;

        // 更新场站的已分配路线计数
        this->location[locId].splitTime++;

        // 4. 更新该条独立路线的物理信息
        // 调用原有的 updateRouteInfor，这会计算这条 route 上的负载(load) 和距离(cost)
        // 传入参数通常是 (route 的索引, 一个增量/状态值，通常传 -1 表示全量刷新)
        this->updateRouteInfor(currentRoute->cour, -1);
    }

    // 5. 更新所有使用过的场站的宏观信息
    for (int i = 0; i < data->numL; i++) {
        if (this->location[i].used) {
            int locIndex = i + data->numV;
            this->updateLocationInfor(locIndex);
        }
    }

    // 6. 全局属性矫正与强制校验
    // 执行整体的距离评估，这会计算整张图的总距离并存入 this->dis
    this->evaluationDis();

    // 强制消除可行性违规。因为 Gurobi 的解在数学上保证了容量可行，
    // 我们必须将相关的惩罚清零，以防止原框架误杀优质解。
    this->capExcess = 0;
    this->penalty = 0.0;
    this->isFeasible = true;
    if (data->hardTimeWindows) this->decideFeasible();

    // (可选/防御性) 验证生成的解是否完全正确
    this->isRight();
}
