/******************************************************************************
 * File: popManager.cpp
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description: 
 *   Manages the population of individuals, sub-populations, and diversity logic.
 ******************************************************************************/

#include "popManager.h"
#include "RoutePool.h"
#include <cmath>

popManager::popManager(read_data *data) {
	// TODO Auto-generated constructor stub
	this->data=data;
	this->numV = data->numV;
	this->numL = data->numL;
	this->numC = data->numC;
	order = new int [numV];
	popula = std::vector <std::vector<Individual*>>(100);
	reverseReplacementCount = 0; // 初始化计数器
    popCreationIteration = std::vector<long long int>(100, 0); // 初始化种群代数记录器，默认容量为 100 
}

popManager::~popManager() {
	// TODO Auto-generated destructor stub
	for (int i = 0; i < (int)feasibleSubpopulation.size(); i++)
		delete feasibleSubpopulation[i];
	delete [] order;
	for (int i=0;i<data->numLocConfiguration+1;i++){
		for (Individual * indiv : popula[i]) delete indiv;
		popula[i].clear();
	}
}
void popManager:: copyAllData(Individual * s,Individual *ss){
	Node *nodeU;
	s->saturatLoc.clear();
	s->saturatRoute.clear();
	/*initialize solution s*/
	for (int i=0;i<numV; i++){
		s->client[i].pre = NULL;
		s->client[i].next = NULL;
		s->client[i].route = NULL;
	}
	for (int i=0;i<numL;i++){
		s->location[i].emptyRoutes.clear();
		for (int j=0;j<s->location[i].maxVisit;j++){
			s->location[i].depots[j].next = &s->location[i].endDepots[j];
			s->location[i].endDepots[j].pre = &s->location[i].depots[j];
			s->updateRouteInfor(s->location[i].depots[j].route->cour, -1);
		}
	//	s->updateLocationInfor(i + numV);
		s->location[i].used=false;
		s->location[i].emptyRoutes.clear();
	}
	//
	int count;
	Route * routeU;
	for (int i=numV; i<numC;i++){
		if (!ss->location[i-numV].used)continue;
		for (int j=0;j<ss->location[i-numV].maxVisit;j++){
			if (ss->location[i-numV].depots[j].route->used){
				count = 0;
				nodeU = ss->location[i-numV].depots[j].next;
				while(!nodeU->isDepot){
					order[count] = nodeU->cour;
					count++;
					nodeU=nodeU->next;
				}
				s->location[i-numV].depots[j].route->used = true;
				routeU = s->location[i-numV].depots[j].route;
				nodeU = &s->client[order[0]];
				nodeU->pre = routeU->depot;
				nodeU->route = routeU;
				routeU->depot->next = nodeU;
				for (int k=1; k<count; k++){
					s->client[order[k]].pre = nodeU;
					nodeU->next = &s->client[order[k]];
					s->client[order[k]].route = routeU;
					nodeU = &s->client[order[k]];
				}
				nodeU->next = routeU->endDepot;
				routeU->endDepot->pre = nodeU;
				s->updateRouteInfor(routeU->cour,-1);
			}
			else{
				s->location[i-numV].depots[j].next = &s->location[i-numV].endDepots[j];
				s->location[i-numV].endDepots[j].pre = &s->location[i-numV].depots[j];
				s->updateRouteInfor(s->location[i-numV].depots[j].route->cour,-1);
			}
		}
		s->updateLocationInfor(i);
	}
	s->dis=ss->dis;
	s->fit=ss->fit;
	s->biasedFit=ss->biasedFit;
	s->isFeasible=ss->isFeasible;
}
int popManager::addIndividual(Individual *indiv, int rankPop, bool isInitial, RoutePool* pool){
	if (isInitial){
		std::vector<int>depot;
		if (rankPop > -1 && rankPop < data->numLocConfiguration){
			for (int i=0;i<numL;i++){
				if (indiv->location[i].used)
					depot.push_back(i+numV);
			}
			auto it = data->compactLocConfigure[rankPop].begin();
			if ((int)depot.size() == (int)data->compactLocConfigure[rankPop].size()){
				for (int i=0;i<(int)depot.size();i++){
					if (*it != depot[i]){
						rankPop = data->numLocConfiguration;
					//	addInElitePop(indiv);
						break;
					}
					it ++;
				}
			}
		}
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	SubPopulation & subpop = popula[rankPop];
	//
	for (Individual * myIndividual : subpop)// no clone individuals
		if (myIndividual->fit + Min > indiv->fit && myIndividual->fit - Min < indiv->fit)
			return -1;
	//
	Individual * myIndividual = new Individual();
	myIndividual->define(data);
	myIndividual->initilization();
	copyAllData(myIndividual, indiv);
	for (Individual * myIndividual2 : subpop){
		double myDistance = myIndividual->brokenPairsDistance(myIndividual2);
		myIndividual2->indivsPerProximity.insert({ myDistance, myIndividual });
		myIndividual->indivsPerProximity.insert({ myDistance, myIndividual2 });
	}

	int place = (int)subpop.size();
	while (place > 0 && subpop[place - 1]->fit > indiv->fit - Min) place--;
	subpop.emplace(subpop.begin() + place, myIndividual);
	//
	std::vector<int>depot;
	bool isObtainNewPop = false;int index = -1;

	if (indiv->fit + Min < bestIndivRestart->fit){

		indiv->isRight();
		bestIndivRestart->initilization();
		copyAllData(bestIndivRestart, indiv);
		data->meetBestTime = (double)clock()/(double)CLOCKS_PER_SEC;
		data->nbIterNonImpro=0;
		if (indiv->fit + Min < global->fit){
			global->initilization();
			copyAllData(global, indiv);
			data->meetBestTime = float((double)clock()/(double)CLOCKS_PER_SEC);
		}
		//to replace a subPop if the new global solution's not belong to any subpop.
		if (!data->no_capacity_location){
			depot.clear();
			for (int i=0;i<numL;i++){
				if (global->location[i].used)
					depot.push_back(i+numV);
			}
			if (rankPop == data->numLocConfiguration && !isInitial){

				std::vector<std::pair<double, int>>avgCostPop;
				for (int i=0;i<data->numLocConfiguration; i++){
					if((int)popula[i].size() == 0)continue;
					double cost = 0;int count = 0;
					SubPopulation & sub = popula[i];
					for (Individual * myIndiv : sub){
						cost += myIndiv->fit;
						count ++;
					}
					if (count > 0)
						avgCostPop.push_back({cost / (double)count, i});
				}
				if ((int)avgCostPop.size() == 0)
					index = 0;
				else{
					double max_avg = 0;
					for (int i=0;i <(int)avgCostPop.size(); i++){
						if (max_avg < avgCostPop[i].first){
							max_avg = avgCostPop[i].first;
							index = avgCostPop[i].second;
						}
					}
				}
				SubPopulation & sub = popula[index];
				for (Individual * indiv : sub) delete indiv;
				sub.clear();
				data->compactLocConfigure[index].clear();
				for (int i=0;i<(int)depot.size();i++)
					data->compactLocConfigure[index].insert(depot[i]);
				Individual * myIndivi = new Individual();
				myIndivi->define(data);
				myIndivi->initilization();
				copyAllData(myIndivi, myIndividual);

				sub.emplace(sub.begin(),myIndivi);
                // 【新增】：游离解因全局最优建立新根据地时，提取初代路径
                if (pool != nullptr && myIndivi->isFeasible) {
                    pool->extractRoutes(myIndivi);
                }
				// generate the initial pop for the promising depot configuration
				isObtainNewPop = true;
				reverseReplacementCount++; // 成功触发逆袭机制，计数器+1

				// 【新增】：为因全局最优解而诞生的新种群，赋予禁忌保护期时间戳
				popCreationIteration[index] = data->iteration;
			}
		}
		////////////////////////////////////////////
	}
	// =========================================================
	// [新增机制] 游离种群延迟孵化与锦标赛替换 (Delayed Incubation & Tournament)
	// =========================================================
	if ((int)subpop.size() == data->popMax) {
		
		// 仅当目标为游离种群且非初始化阶段时触发判定
		if (rankPop == data->numLocConfiguration && !isInitial) {
			
			// 1. 提取游离种群的最好解 (Best Free)
			Individual* best_free = subpop[0];
			for (Individual* ind : subpop) {
				if (ind->fit < best_free->fit) best_free = ind;
			}

			// 2. 评估常驻种群以确定潜在被替代目标，并应用禁忌策略 (Evaluate incumbent populations with Tabu strategy)
			double max_avg_cost = -1.0;
			int worst_res_idx = -1;
			bool found_empty_slot = false; // 新增：空槽位标记标识

			for (int i = 0; i < data->numLocConfiguration; i++) {
				// 【核心修正】：捕获空余槽位并赋予绝对优先接管权
				if (popula[i].empty()) {
					worst_res_idx = i;
					found_empty_slot = true;
					break; // 发现空槽位，直接中断扫描，锁定目标
				}
				
                // 执行禁忌期审查 (Tabu period verification)
                if ((data->iteration - popCreationIteration[i]) < data->popProtectionPeriod) {
                    continue; // 拒绝评估处于保护期内的种群
                }

				double current_avg = 0.0;
				for (Individual* ind : popula[i]) current_avg += ind->fit;
				current_avg /= (double)popula[i].size();
				
				if (current_avg > max_avg_cost) {
					max_avg_cost = current_avg;
					worst_res_idx = i;
				}
			}

			// 3 & 4. 执行替换授权判定与数据覆写 (Execute Replacement Authorization and Data Overwrite)
			if (worst_res_idx != -1) {
				bool is_authorized_to_replace = false;
				double target_fit_for_log = 0.0;

				// 授权分支 A：发现空槽位，无条件授权免对决入驻
				if (found_empty_slot) {
					is_authorized_to_replace = true;
				} 
				// 授权分支 B：锁定劣势种群，需执行锦标赛对决 (Tournament Selection)
				else {
					Individual* best_worst_res = popula[worst_res_idx][0];
					for (Individual* ind : popula[worst_res_idx]) {
						if (ind->fit < best_worst_res->fit) best_worst_res = ind;
					}
					// 要求游离解具备严格优势方可覆写配置
					if (best_free->fit < best_worst_res->fit) {
						is_authorized_to_replace = true;
						target_fit_for_log = best_worst_res->fit;
					}
				}

				// 获得接管授权，执行物理覆写
				if (is_authorized_to_replace) {
					// 区分日志输出语义，保持严谨的终端监控反馈
					if (found_empty_slot) {
						std::cout << "\n>>> [Mechanism Monitor] 游离解发现系统空余槽位 (Empty Slot Detected)! (Free Best: " 
						          << std::fixed << best_free->fit 
						          << ")，直接进驻并激活槽位 [" << worst_res_idx << "]\n" << std::endl;
					} else {
						std::cout << "\n>>> [Mechanism Monitor] 游离解触发种群更迭 (Free Best: " << std::fixed << best_free->fit 
						          << " < Incumbent Worst: " << target_fit_for_log 
						          << ")，成功接管槽位 [" << worst_res_idx << "]\n" << std::endl;
					}

					// 【新增】：同步累加逆袭监控计数器，确保外层监控日志准确
					reverseReplacementCount++;

					// 重置该槽位的创立代数，启动新一轮禁忌保护 (Reset the epoch for Tabu protection)
					popCreationIteration[worst_res_idx] = data->iteration;

					SubPopulation& target_sub = popula[worst_res_idx];

					// 若原槽位非空，释放内存以防泄漏 (Safe memory deallocation)
					for (Individual* ind : target_sub) delete ind;
					target_sub.clear();

					// 覆写车场配置 (Depot Configuration)
					data->compactLocConfigure[worst_res_idx].clear();
					for (int j = 0; j < numL; j++) {
						if (best_free->location[j].used) {
							data->compactLocConfigure[worst_res_idx].insert(j + numV);
						}
					}

					// 游离始祖登基，作为新种群的种子
					Individual* seed_indiv = new Individual();
					seed_indiv->define(data);
					seed_indiv->initilization();
					copyAllData(seed_indiv, best_free);
					target_sub.emplace(target_sub.begin(), seed_indiv);
                    
                    // 【新增】：锦标赛逆袭成功，提取始祖解路径
                    if (pool != nullptr && seed_indiv->isFeasible) {
                        pool->extractRoutes(seed_indiv);
                    }

					// 清空原游离种群，释放算力并等待下一轮探索
					for (Individual* ind : subpop) delete ind;
					subpop.clear();

					// 返回替换的索引，通知外层 (solution.cpp -> main_loop) 继续调用快速孵化繁衍机制
					return worst_res_idx; 
				}
			}
		}

		// 若未触发替换，或非游离种群，执行常规基于多样性的缩减
		while ((int)subpop.size() > data->popMin)
			removeWorstBiasedFitness(subpop);
	}
	// =========================================================

	// 【修改】：全新的动态均值准入法则
	// 无论当前解是进入哪个种群，只要它的成本低于路径池的历史平均水平，
	// 它就有资格注入新基因，并进一步拉低均值门槛！
	if (pool != nullptr && indiv->isFeasible) {
		if (indiv->fit < pool->getAverageFitness()) {
			pool->extractRoutes(indiv);              // 拆解并提取路径
			pool->recordSolutionFitness(indiv->fit); // 更新动态平均值
		}
	}

	if (isObtainNewPop)
		return index;
	else
		return -1;
}

void popManager::addInElitePop(Individual *indiv){
/*	//update the e-lite population
	SubPopulation & epop = eliteSubpopulation;
	for (Individual * myIndividual : epop)// no clone individuals
		if (myIndividual->fit + Min > indiv->fit && myIndividual->fit - Min < indiv->fit)
			return;
	if ((int)epop.size() < data->popMax){
		Individual * myIndiv = new Individual();
		myIndiv->define(data);
		copyAllData(myIndiv, indiv);
		int place = (int)epop.size();
		while (place > 0 && epop[place - 1]->fit > indiv->fit - Min)
			place--;
		epop.emplace(epop.begin() + place, myIndiv);
	}
	else{
		int place = (int)epop.size();
		while (place > 0 && epop[place - 1]->fit > indiv->fit - Min)
			place--;
		if (place < (int)epop.size()){
			Individual * myIndiv = new Individual();
			myIndiv->define(data);
			copyAllData(myIndiv, indiv);
			epop.emplace(epop.begin() + place, myIndiv);
			int worst = (int)epop.size()-1;
			Individual * worstIndividual = epop[worst];
			epop.erase(epop.begin() + worst); // Removing the individual from the population
			delete worstIndividual;
		}
	}
	*/
}
Individual * popManager::selectionParentSolution(Individual *exist, int rankPop){
	SubPopulation & subpop = popula[rankPop];
	updateBiasedFitnesses(subpop);

	int rand1,rand2;
	Individual *target;
	if (subpop.size() == 1){
		target = subpop[0];
		return target;
	}
	int max_idx = (int)subpop.size() - 1;
	while(1){
		rand1 = RandomInt(0, max_idx); // 使用 basic.h 中的随机函数
		rand2 = RandomInt(0, max_idx);
		while (rand1 == rand2) rand2 = RandomInt(0, max_idx);
		//if (subpop[rand1]->biasedFit < subpop[rand2]->biasedFit)
		if (subpop[rand1]->biasedFit < subpop[rand2]->biasedFit)
			target = subpop[rand1];
		else
			target = subpop[rand2];
			
		if (target != exist) break;
	}
	return target;
}

void popManager::reStart(){
	std::cout<<"----Restart and create a new population----"<<std::endl;
	for (int i=0;i<data->numLocConfiguration+1;i++){
		for (Individual * indiv : popula[i]) delete indiv ;
		popula[i].clear();
	}
	data->nbIterNonImpro=0;
	bestIndivRestart->fit = Max;
}
double popManager::averagePop(){
	double fit=0;
	for (Individual * indiv : feasibleSubpopulation)
		fit += indiv->fit;
	fit = fit/(double)feasibleSubpopulation.size();
	return fit;
}
double popManager::usedDepots(){
	std::set<int>locations;
	for (Individual * indiv : feasibleSubpopulation){
		for (int i=0;i<numL;i++){
			if (indiv->location[i].used)
				locations.insert(i);
		}
	}
	int ss = locations.size();
	return(double(ss) / double(numL));
}
void popManager::removeWorstBiasedFitness(SubPopulation & pop){
	updateBiasedFitnesses(pop);
	if (pop.size() <= 1) throw std::string("Eliminating the best individual: this should not occur in the algorithm");

	Individual * worstIndividual = NULL;
	int worstIndividualPosition = -1;
	bool isWorstIndividualClone = false;
	double worstIndividualBiasedFitness = -1.e30;
	for (int i = 0; i < (int)pop.size(); i++)
	{
		bool isClone = (pop[i]->averageBrokenPairsDistanceClosest(1) < Min); // A distance equal to 0 indicates that a clone exists
		if ((isClone && !isWorstIndividualClone) || (isClone == isWorstIndividualClone && pop[i]->biasedFit > worstIndividualBiasedFitness))
		{
			worstIndividualBiasedFitness = pop[i]->biasedFit;
			isWorstIndividualClone = isClone;
			worstIndividualPosition = i;
			worstIndividual = pop[i];
		}
	}

	pop.erase(pop.begin() + worstIndividualPosition); // Removing the individual from the population
	for (Individual * myIndividual2 : pop) myIndividual2->removeProximity(worstIndividual); // Cleaning its distances from the other individuals in the population
	delete worstIndividual; // Freeing memory
	worstIndividual=NULL;
}

void popManager::updateBiasedFitnesses(SubPopulation & pop){
	// Ranking the individuals based on their diversity contribution (decreasing order of distance)
	std::vector <std::pair <double, int> > ranking;
	for (int i = 0 ; i < (int)pop.size(); i++)
		ranking.push_back({-pop[i]->averageBrokenPairsDistanceClosest(data->nbClost),i});
	std::sort(ranking.begin(), ranking.end());

	// Updating the biased fitness values
	if (pop.size() == 1)
		pop[0]->biasedFit = 0;
	else{
		for (int i = 0; i < (int)pop.size(); i++){
			double divRank = (double)i / (double)(pop.size() - 1); // Ranking from 0 to 1
			double fitRank = (double)ranking[i].second / (double)(pop.size() - 1);
			if ((int)pop.size() <= data->nbElite) // Elite individuals cannot be smaller than population size
				pop[ranking[i].second]->biasedFit = fitRank;
			else
				pop[ranking[i].second]->biasedFit = fitRank + (1.0 - (double)data->nbElite / (double)pop.size()) * divRank;
		}
	}
}
void popManager::removeEmptySubPop(){
	for (int i = popula.size()-2;i >= 0; i--){
		if (popula[i].empty()){
			popula.erase(popula.begin() + i);
			data->numLocConfiguration--;
		}
	}
	data->dimTabu1.clear();
	data->dimTabu2.clear();
	long long int var, var1, var2;;int seed;
	for (int i=0;i<data->numLocConfiguration;i++){
		if ((int)popula[i].size() > 0){
			var = 0;var1 = 0;
			auto it = data->compactLocConfigure[i].begin();
			for (;it != data->compactLocConfigure[i].end(); it++){
				seed = *it - data->numV;
				var2 = 1ULL << seed;
				var += var2;
				var1 += var2 + 9527 * seed;
			}
			data->dimTabu1[var % data->dimension] = i;
			data->dimTabu2[var1 % data->dimension] = i;
		}
	}
}
void popManager::reduceHalfSubPop(){
	if ((int)popula.size() < 3)return;
	std::vector<std::pair<double, int>>averageCost;
	double avgCost;
	for (int i=0;i< data->numLocConfiguration;i++){
		avgCost = 0;
		for (int j=0;j<(int)popula[i].size();j++){
			avgCost += popula[i][j]->fit;
			if (std::abs(bestIndivRestart->fit - popula[i][j]->fit) < 0.000001)
				avgCost = -Max;
		}
		if ((int)popula[i].size() == 0)
			avgCost = Max;
		else
			avgCost = avgCost / (double)popula[i].size();
		averageCost.push_back({avgCost, i});
	}
	std::sort(averageCost.begin(), averageCost.end());
	int halfSize = (int)(data->numLocConfiguration/2.);
	std::vector<int> remove;
	for (int i=(int)averageCost.size()-1; i > halfSize; i--)
		remove.push_back(averageCost[i].second);
	std::sort(remove.begin(),remove.end());
	std::reverse(remove.begin(),remove.end());
	for (int i=0;i<(int)remove.size();i++){
		SubPopulation & sub = popula[remove[i]];
		for (Individual * indiv : sub) delete indiv;
		popula.erase(popula.begin()+remove[i]);
		data->compactLocConfigure.erase(data->compactLocConfigure.begin() + remove[i]);
	}
	data->numLocConfiguration = data->numLocConfiguration - remove.size();

	data->dimTabu1.clear();
	data->dimTabu2.clear();
	long long int var, var1, var2;;int seed;
	for (int i=0;i<data->numLocConfiguration;i++){
		if ((int)popula[i].size() > 0){
			var = 0;var1 = 0;
			auto it = data->compactLocConfigure[i].begin();
			for (;it != data->compactLocConfigure[i].end(); it++){
				seed = *it - data->numV;
				var2 = 1ULL << seed;
				var += var2;
				var1 += var2 + 9527 * seed;
			}
			data->dimTabu1[var % data->dimension] = i;
			data->dimTabu2[var1 % data->dimension] = i;
		}
	}


}

int popManager::getWorstResidentIndex() {
    int worstIdx = 0;
    double worstAvgFit = -1.0;
    
    // 客观事实防线 1：只遍历常驻种群 (0 到 data->numLocConfiguration - 1)
    // 绝对不能触碰索引为 data->numLocConfiguration 的游离种群
    for (int i = 0; i < data->numLocConfiguration; ++i) {
        if (popula[i].empty()) {
            // 如果存在空种群，优先覆盖它，避免覆盖已有数据的种群
            return i; 
        }
        
        double sumFit = 0.0;
        for (Individual* ind : popula[i]) {
            sumFit += ind->fit; 
        }
        double avgFit = sumFit / popula[i].size();
        
        if (avgFit > worstAvgFit) {
            worstAvgFit = avgFit;
            worstIdx = i;
        }
    }
    return worstIdx;
}

int popManager::injectSPSolution(Individual* spIndiv, RoutePool* pool) { // <--- 关键修正：这里必须是 int
    if (!spIndiv->isFeasible) {
        delete spIndiv; 
        return -1; 
    }
    
    if (data->no_capacity_location || data->numLocConfiguration == 0) {
        addIndividual(spIndiv, 0, false); 
        delete spIndiv; 
        return -1; 
    }

    // 1. 提取 Gurobi 产生解的场站配置集合与哈希特征
    std::set<int> spDepots;
    long long int var1 = 0, var2 = 0;
    for (int i = 0; i < data->numL; i++) {
        if (spIndiv->location[i].used) {
            spDepots.insert(i + data->numV);
            long long int var = 1ULL << i;
            var1 += var;
            var2 += var + 9527 * i;
        }
    }

    // 2. 匹配现有的常驻种群配置
    int matchIndex = -1;
    for (int i = 0; i < data->numLocConfiguration; ++i) {
        if (data->compactLocConfigure[i] == spDepots) {
            matchIndex = i;
            break;
        }
    }

    // 3. 同步更新全局最优解
    if (spIndiv->fit < bestIndivRestart->fit) {
        bestIndivRestart->initilization();
        copyAllData(bestIndivRestart, spIndiv);
    }
    if (spIndiv->fit < global->fit) {
        global->initilization();
        copyAllData(global, spIndiv);
        data->meetBestTime = float((double)clock()/(double)CLOCKS_PER_SEC);
    }

    // 4. 执行多分支注入逻辑
    if (matchIndex != -1) {
        // 【新增】：显式判重操作 (Explicit Duplicate Check)
        // 在进行昂贵的距离计算前，如果发现适应度完全相同的解，视为克隆体直接抛弃
        for (Individual* existingIndiv : popula[matchIndex]) {
            if (existingIndiv->fit + Min > spIndiv->fit && existingIndiv->fit - Min < spIndiv->fit) {
                std::cout << "[SP Reject] SP解与常驻种群 [" << matchIndex << "] 中的现有解重复，拦截注入。" << std::endl;
                delete spIndiv; // 释放刚才为SP解分配的内存，防止内存泄漏
                return -1;
            }
        }

        std::cout << "[SP Inject] 配置匹配 (Match found)! 注入常驻种群 [" << matchIndex << "] (Exploitation)" << std::endl;

        for (Individual* existingIndiv : popula[matchIndex]) {
            double myDistance = spIndiv->brokenPairsDistance(existingIndiv);
            existingIndiv->indivsPerProximity.insert({ myDistance, spIndiv });
            spIndiv->indivsPerProximity.insert({ myDistance, existingIndiv });
        }

        popula[matchIndex].push_back(spIndiv);

		// 【新增】：SP生成的精英解贡献统计，进一步拉低门槛
        if (pool != nullptr) {
            pool->recordSolutionFitness(spIndiv->fit);
        }
        
        if ((int)popula[matchIndex].size() > data->popMin) {
            updateBiasedFitnesses(popula[matchIndex]);
            removeWorstBiasedFitness(popula[matchIndex]);
        }
        return -1;
    } else {
        // 计算常驻种群劣势指标，以评估 SP 解的全局接管资质 (Evaluate incumbent vulnerabilities)
        int worstIdx = -1;
        double worstAvgFit = -1.0;
        
        for (int i = 0; i < data->numLocConfiguration; ++i) {
            if (popula[i].empty()) {
                worstIdx = i;
                worstAvgFit = 1e30; // 识别到空余槽位，赋予极高优先级
                break;
            }
            double sumFit = 0.0;
            for (Individual* ind : popula[i]) sumFit += ind->fit;
            double avgFit = sumFit / (double)popula[i].size();
            
            if (avgFit > worstAvgFit) {
                worstAvgFit = avgFit;
                worstIdx = i;
            }
        }

        // 【修改2：提升SP接管门槛】找出最差种群（平均成本最高）里的【最好解】
        double bestFitInWorstPop = 1e30;
        if (worstIdx != -1 && !popula[worstIdx].empty()) {
            for (Individual* ind : popula[worstIdx]) {
                if (ind->fit < bestFitInWorstPop) bestFitInWorstPop = ind->fit;
            }
        } else {
            bestFitInWorstPop = 1e30; // 如果是空槽位，无条件放行
        }

        // 核心审查：SP解必须严格优于劣势种群里的【最好解】
        if (worstIdx != -1 && spIndiv->fit < bestFitInWorstPop) {
            std::cout << "[SP Inject] 发生范式转移! SP解成本 (" << std::fixed << spIndiv->fit 
                      << ") 击败劣势种群的最优解 (" << bestFitInWorstPop << "). 强势接管槽位 [" << worstIdx << "]" << std::endl;
            // 确立接管时间戳，触发禁忌保护 (Establish timestamp for Tabu period)
            popCreationIteration[worstIdx] = data->iteration;

            long long int old_var1 = 0, old_var2 = 0;
            for (auto it = data->compactLocConfigure[worstIdx].begin(); it != data->compactLocConfigure[worstIdx].end(); ++it) {
                int old_loc = *it - data->numV;
                long long int old_var = 1ULL << old_loc;
                old_var1 += old_var;
                old_var2 += old_var + 9527 * old_loc;
            }
            auto it1 = data->dimTabu1.find(old_var1 % data->dimension);
            if (it1 != data->dimTabu1.end() && it1->second == worstIdx) data->dimTabu1.erase(it1);
            auto it2 = data->dimTabu2.find(old_var2 % data->dimension);
            if (it2 != data->dimTabu2.end() && it2->second == worstIdx) data->dimTabu2.erase(it2);

            for (Individual* ind : popula[worstIdx]) { delete ind; }
            popula[worstIdx].clear();

            data->compactLocConfigure[worstIdx] = spDepots;
            data->dimTabu1[var1 % data->dimension] = worstIdx;
            data->dimTabu2[var2 % data->dimension] = worstIdx;

            popula[worstIdx].push_back(spIndiv);

			// 【新增】：SP生成的精英解贡献统计，进一步拉低门槛
            if (pool != nullptr) {
                pool->recordSolutionFitness(spIndiv->fit);
            }

            return worstIdx;
        } else {
            std::cout << "[SP Reject] 准入失败 (Admittance Rejected). SP解 (" << std::fixed << spIndiv->fit 
                      << ") 未能超越劣势种群最优解门槛 (" << bestFitInWorstPop << "). 抛弃该配置." << std::endl;
            delete spIndiv; // 销毁无竞争力个体，避免内存泄漏 (Prevent memory leakage)
            return -1;
        }
    }
}