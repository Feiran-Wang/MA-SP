/*
 * solution.cpp
 *
 * Author: Feiran Wang
 */
#include "basic.h"
#include "solution.h"
#include "initialsol.h"
#include "LS.h"
#include "crossover.h"
using namespace std;
solution::solution(read_data * data, crossover *cross, initial_sol *initialSol,LS *ls,  popManager *popman, RepairProcedure *repair,mutation *mutationOperator) {
	this->data=data;
	this->cross=cross;
	this->initialSol=initialSol;
	this->ls=ls;
	this->popman=popman;
	this->repair=repair;
	this->mutationOperator=mutationOperator;
	//
	temIndiv=new Individual();
	temIndiv->define(data);
	global=new Individual ();
	global->define(data);
	global->initilization();
	loc_best = new Individual();
	loc_best->define(data);
	loc_best->initilization();
	bestIndivRestart=new Individual();
	bestIndivRestart->define(data);
	bestIndivRestart->initilization();

	popman->global=global;
	popman->bestIndivRestart = bestIndivRestart;
	int sizeSubPop=100;//
	subPop=new Individual[sizeSubPop];//
	for (int i=0;i<sizeSubPop;i++){
		subPop[i].define(data);
		subPop[i].initilization();
	}
	global->fit=Max;
	bestIndivRestart->fit=Max;

	// === 实例化 Matheuristic 组件 ===
    this->routePool = new RoutePool(data);
    this->spSolver = new SPSolver(data);
}
solution::~solution(){
	delete temIndiv;
	delete global;
	delete bestIndivRestart;
	delete [] subPop;
	delete loc_best;
	// === 释放 Matheuristic 组件 ===
    if (this->routePool != nullptr) {
        delete this->routePool;
    }
    if (this->spSolver != nullptr) {
        delete this->spSolver;
    }
}
void solution::preprocessingLocationConfiguration(){
	data->compactLocConfigure.clear();
	//int initalNumDepots = (int)data->LocConfigure.size();
	int initalNumDepots = std::min(500, (int)data->LocConfigure.size());      // the key parameters// 100
	if ((int)data->LocConfigure.size() < initalNumDepots)
		initalNumDepots = (int)data->LocConfigure.size();
	for (int i=0;i<initalNumDepots; i++)
		data->compactLocConfigure.push_back(data->LocConfigure[i].second);

	if (data->no_capacity_location){
		data->numLocConfiguration = 0;
		return;// if no depot configurations, jump
	}

	std::vector<std::pair<double, int>>averageCost;
	double avgCost,bestCost = Max;
	double global_cost = Max;
	for (int i = 0; i<initalNumDepots;i++){// an extra subPop
		avgCost = 0;bestCost = Max;
		for (int j = 0; j < numSubPopPara; j++){
			initialSol->initial_solution(temIndiv, i);
			ls->local_search_run(temIndiv,data->penaltyDepot, data->penaltyVeh);
			avgCost += temIndiv->fit;
			if (bestCost > temIndiv->fit)
				bestCost = temIndiv->fit;
			if (temIndiv->fit < global_cost){
				// to record the best solution given each depot configuration
				global_cost = temIndiv->fit;
				loc_best->initilization();
				popman->copyAllData(loc_best, temIndiv);// record the best solution given the current depot configuration.
			}
		}
		avgCost = avgCost / (double)(data->popMin);// in this place, we use the best solution
	//	averageCost.push_back({avgCost, i});
		averageCost.push_back({bestCost, i});
		std::cout<<i<<"   tries   "<< bestCost<<std::endl;

//		loc_best->evaluationDis();
	}
	std::sort(averageCost.begin(), averageCost.end());
	loc_best->evaluationDis();

	//
	data->numLocConfiguration = numSubPopPara;//5///////////////////////////////////////////////////
	data->compactLocConfigure.clear();
	for (int i=0;i < std::min(data->numLocConfiguration,initalNumDepots); i++)
		data->compactLocConfigure.push_back(data->LocConfigure[averageCost[i].second].second);
	data->numLocConfiguration = std::min(data->numLocConfiguration,initalNumDepots);
	// add the best solution of each depot configuration in the elite subpop.
//	loc_best->outputBestSolution();
	popman->addIndividual(loc_best, data->numLocConfiguration, true);// insert the local best solution to elite pop
	std::cout<<"Finishing initial filter"<<std::endl;
}
void solution::generation_population(int rankPop, bool isInitial){
	double bestFit=Max;
	/*generate a feasible population set*/
	for (int i = 0; i < data->popMin*4; i++){
		initialSol->initial_solution(temIndiv, rankPop);
		ls->local_search_run(temIndiv,data->penaltyDepot, data->penaltyVeh);
		popman->addIndividual(temIndiv, rankPop, isInitial, this->routePool);
		if (bestFit>temIndiv->fit)
			bestFit=temIndiv->fit;
//		if (i % 100 == 0)
//			std::cout<<i<<"  "<<bestFit<<std::endl;
	}
	std::cout<<"The best fit is    "<<bestFit<<"  The time consuming  "<<(double)clock()/(double)CLOCKS_PER_SEC<<std::endl;
}
void solution::outputprocess(Individual *indiv){
	std::cout<<"It: "<<data->iteration<<"--G-Fit: "<<std::setiosflags(ios::fixed)<<std::setprecision(2)<<float(global->fit);
	std::cout<<"  --R-Fit: "<<bestIndivRestart->fit;
//	std::cout<<"    Avg F "<<float(popman->averagePop())<<std::setprecision(2);
	std::cout<<"  Curr Sol--"<<float(indiv->fit)<<std::setprecision(2);
	std::cout<<"  IT NoImpro--"<<data->nbIterNonImpro;
	std::cout<<"  Time comsuming--"<<float((double)clock()/(double)CLOCKS_PER_SEC)<<std::setprecision(2);
//	std::cout<<"  used depots--"<<popman->usedDepots();
//	double ratio = data->loopp / 1000.;
//	std::cout<<"  overSat. Loc "<< ratio;
//	std::cout<<"  CostProb "<<data->probFixedCost<<std::endl;
	std::cout<<std::endl;
	data->loopp = 0;
}
void solution::getAverageValue(int rank){
	if (rank < 0){
		avgCostPop.clear();
		for (int i=0;i<data->numLocConfiguration + 1; i++){
			double cost = 0;
			for (int j=0;j<(int)popman->popula[i].size();j++){
				cost += popman->popula[i][j]->fit;
			}
			int ss = popman->popula[i].size();
		//	avgCostPop.push_back(cost);
			avgCostPop.push_back(cost / (double)ss);
		}
		//cut some pop whose obj is too worse.
/*		double min_avg = Max;
		for (int i=0;i <data->numLocConfiguration + 1; i++){
			if (min_avg > avgCostPop[i])
				min_avg = avgCostPop[i];
		}
		for (int i=0;i <data->numLocConfiguration + 1; i++){
			if (100*(avgCostPop[i] - min_avg) / min_avg > 5){
				for (Individual * indiv : popman->popula[i])
					delete indiv ;
				popman->popula[i].clear();
			}
		}
		*/
		//removing half subpop
		std::vector<std::pair<double,int>>rankOrder;
		for (int i=0;i<(int)avgCostPop.size();i++)
			rankOrder.push_back({-avgCostPop[i],i});
		sort(rankOrder.begin(),rankOrder.end());
		for (int i=0;i<data->numLocConfiguration / 2; i++){
			int subpop = rankOrder[i].second;
			for (Individual * indiv : popman->popula[subpop]) delete indiv ;
			popman->popula[subpop].clear();
		}
		//
		long long int var, var1, var2;;int seed;
		for (int i=0;i<data->numLocConfiguration;i++){
			if ((int)popman->popula[i].size() > 0){
				var = 0;var1 = 0;
				auto it = data->compactLocConfigure[i].begin();
				for (;it != data->compactLocConfigure[i].end(); it++){
					seed = *it - data->numV;
					var2 = pow(2, seed);
					var += var2;
					var1 += var2 + 9527 * seed;
				}
				data->dimTabu1[var % data->dimension] = i;
				data->dimTabu2[var1 % data->dimension] = i;
			}
		}
	}
	else{
		double cost = 0;
		for (int j=0;j<(int)popman->popula[rank].size();j++)
			cost += popman->popula[rank][j]->fit;
		int ss = popman->popula[rank].size();
		avgCostPop[rank] = cost / (double)ss;
	//	avgCostPop[rank] = cost;
	}
	// removing half population

}
int solution::selectsubPop(int subpop1){
	int rand1,rand2;
	int target;
	std::vector<int>locConf;
	
	for (int i = 0; i < data->numLocConfiguration + 1; i++) {
        if ((int)popman->popula[i].size() > 0)
            locConf.push_back(i);
    }

	if (locConf.empty()) return -1;

	if ((int)locConf.size() == 1) return locConf[0];
    if ((int)locConf.size() == 2) {
        return (locConf[0] == subpop1) ? locConf[1] : locConf[0]; // 修复下标越界
    }

	while(1){
        rand1 = RandomInt(0, locConf.size() - 1);
		rand2 = RandomInt(0, locConf.size() - 1);

        if ((int)locConf.size() <= 3) return locConf[rand1];

        while(rand1 == rand2)
            rand2 = RandomInt(0, locConf.size() - 1);

        if (avgCostPop[locConf[rand1]] < avgCostPop[locConf[rand2]])
            target = locConf[rand1];
        else
            target = locConf[rand2];

        // 检查选择的子种群是否符合演化要求
        if (target == subpop1 && (int)popman->popula[target].size() > 2) break;
        if (target != subpop1) break;
        
        // 兜底逻辑
        if ((int)locConf.size() == 2) {
            target = (locConf[0] == subpop1) ? locConf[1] : locConf[0];
            break;
        }
    }
    return target;
}
void solution::evolvePop(int rankPop, int &count){
	if ((int)popman->popula[rankPop].size() <= 2){
		count = data->maxIterationPerLoc;
		return;
	}
	int sizeSubPop;
	ma = NULL;
	fa = popman->selectionParentSolution(ma, rankPop);
	ma = popman->selectionParentSolution(fa, rankPop);
	cross->gEAX(fa,ma,subPop,sizeSubPop);
	for (int i =0; i<sizeSubPop; i++){
		repair->repairCapacityConstraint(subPop+i, data->penaltyDepot, data->penaltyVeh);
		if (subPop[i].isFeasible){
		//	if (rand()%100 > data->mutationProb)
		//		mutationOperator->mutationRun(subPop+i);
			ls->local_search_run(subPop+i, data->penaltyDepot, data->penaltyVeh);
			popman->addIndividual(subPop+i, rankPop, true);
		}
	//	if (data->iteration % 500 == 0)	outputprocess(subPop+i);
	//	data->iteration++;
		count++;
	}
}
void solution::initialPop(){
	//phase 1: to construct a number of populations
//	data->compactLocConfigure.clear();
//	if ((int)data->LocConfigure.size() < data->numLocConfiguration)
//		data->numLocConfiguration = (int)data->LocConfigure.size();
//	for (int i=0;i<data->numLocConfiguration; i++)
//		data->compactLocConfigure.push_back(data->LocConfigure[i].second);
	popman->popula.erase(popman->popula.begin()+data->numLocConfiguration +1, popman->popula.end());
	int i;
	for ( i = 0; i<data->numLocConfiguration +1 ;i++){// an extra subPop
		std::cout<<i<<" th ";
		std::cout<<(double)clock()/(double)CLOCKS_PER_SEC<<std::endl;
		generation_population(i, true);
		int count = 0;
		if ((int)popman->popula[i].size() <= 1)
			continue;
		while (count < data->maxIterationPerLoc)
			evolvePop(i, count);
	}
	std::cout<<"Finishing constructing number of initial population"<<std::endl;
}
void solution::main_loop(){
	std::cout<<"---- Start the search----"<<std::endl;
	preprocessingLocationConfiguration();
	int sizeSubPop;
	data->loopp = 0;
	data->nbIterNonImpro=1;
	int subPop1, subPop2;
	initialPop();

	// 【新增】：初始化之后，迭代开始之前，全员注入路径池建立均值基线
	std::cout << "[RoutePool] 正在建立初始精英池基线..." << std::endl;
	for (int i = 0; i < data->numLocConfiguration + 1; i++) {
		for (Individual* indiv : popman->popula[i]) {
			if (indiv->isFeasible) {
				this->routePool->extractRoutes(indiv);              // 提取路径入池
				this->routePool->recordSolutionFitness(indiv->fit); // 记录成本以计算均值
			}
		}
	}
	std::cout << "[RoutePool] 初始路径池建立完成! 均值门槛: " << std::fixed << this->routePool->getAverageFitness() << std::endl;

	getAverageValue(-1);// for using to select some locations
	//	popman->removeEmptySubPop();

	data->iteration=0;
	int isGenerateNewSubPop;
	std::vector<int>depot;
	long long int var1, var2, var;
	int whichSubPop;
	while(1){
		// phase 2: to crossover individuals from different subpopulation;
		subPop1 = selectsubPop(-1);
		subPop2 = selectsubPop(subPop1);
		ma = NULL;
		fa = popman->selectionParentSolution(ma, subPop1);
		ma = popman->selectionParentSolution(fa, subPop2);
		cross->gEAX(fa,ma,subPop,sizeSubPop);
		for (int i =0; i<sizeSubPop; i++){
			repair->repairCapacityConstraint(subPop+i, data->penaltyDepot, data->penaltyVeh);
			if (subPop[i].isFeasible){
				if (RandomInt(0, 99) > data->mutationProb)
					mutationOperator->mutationRun(subPop+i);
				ls->local_search_run(subPop+i, data->penaltyDepot, data->penaltyVeh);

				// 1. 无差别收割高质量可行解的路径（作为数学规划的列）
				// 【删除无差别收割逻辑，路径提取已后置到 addIndividual 中严格把关】

				// 2. 然后再处理子种群的归属和哈希映射
				if (!data->no_capacity_location){
					depot.clear();
					for (int j=0; j<data->numL;j++){
						if (subPop[i].location[j].used)
							depot.push_back(j);
					}
					var1 = 0; var2 = 0;
					for (int j=0;j<(int)depot.size();j++){
						var = pow(2, depot[j]);
						var1 += var;
						var2 += var + 9527 * depot[j];
					}
					if (data->dimTabu1[var1 % data->dimension] == data->dimTabu2[var2 % data->dimension] && data->dimTabu2[var2 % data->dimension] > -1)
						whichSubPop = data->dimTabu2[var2 % data->dimension];
					else
						whichSubPop = data->numLocConfiguration;
					
					isGenerateNewSubPop = popman->addIndividual(subPop+i, whichSubPop, false, this->routePool);
					getAverageValue(whichSubPop);
					
					if (isGenerateNewSubPop > -1){
						// 【修改3A：游离种群成功接管后，快速精准扩充到 popMin】
						while ((int)popman->popula[isGenerateNewSubPop].size() < data->popMin) {
							initialSol->initial_solution(temIndiv, isGenerateNewSubPop);
							ls->local_search_run(temIndiv, data->penaltyDepot, data->penaltyVeh);
							popman->addIndividual(temIndiv, isGenerateNewSubPop, false, this->routePool);
						}
						getAverageValue(isGenerateNewSubPop);
						
						// [修补机制] 基于刚写入的真实车场配置重新计算哈希
						long long int correct_var = 0, correct_var1 = 0, correct_var2 = 0;
						for (auto it = data->compactLocConfigure[isGenerateNewSubPop].begin(); it != data->compactLocConfigure[isGenerateNewSubPop].end(); it++) {
							int seed_val = *it - data->numV;
							correct_var = pow(2, seed_val);
							correct_var1 += correct_var;
							correct_var2 += correct_var + 9527 * seed_val;
						}
						data->dimTabu1[correct_var1 % data->dimension] = isGenerateNewSubPop;
						data->dimTabu2[correct_var2 % data->dimension] = isGenerateNewSubPop;
					}
				}
				else{
					popman->addIndividual(subPop+i, 0, false);
				}

				/////////////////////////////////////
				if (data->iteration % 1000 == 0)outputprocess(subPop+i);
				
				// 终端打印逆袭机制的生效次数及游离种群规模
				if (data->iteration > 0 && data->iteration % 50000 == 0) {
					std::cout << ">>> [机制监控] \"gamma+1\" 游离种群动态逆袭替换机制累计生效: " 
							<< popman->reverseReplacementCount << " 次 | "
							<< "当前游离种群个体数量: " 
							<< popman->popula[data->numLocConfiguration].size() << std::endl;
				}

				//	if (data->iteration > 480000)
				//		outputprocess(subPop+i);
				data->iteration++;
				
				// 激活混合元启发式 (Matheuristic) 机制，基于预定周期调用集合划分求解器
				if (data->iteration > 0 && data->iteration % data->spTriggerFrequency == 0) {
					this->sp_total_triggers++;
					int current_pool_size = this->routePool->getPool().size();

					std::cout << "\n[SP MONITOR] Iteration " << data->iteration 
					          << ": Triggered #" << this->sp_total_triggers 
					          << " | RoutePool Size: " << current_pool_size << std::endl;
						
					std::vector<StandaloneRoute> optimalCombination = this->spSolver->solve(this->routePool);

					if (current_pool_size < 10) {
						std::cout << "[SP Warning] RoutePool is too small (" << current_pool_size << "). Gurobi might not find any improvement." << std::endl;
					}
				
					if (!optimalCombination.empty()) {
						this->sp_found_solutions++;
						std::cout << "[SP Success] Gurobi found a valid configuration! (Total found: " << this->sp_found_solutions << ")" << std::endl;
						
						Individual* spIndiv = new Individual();
						spIndiv->define(this->data);
						spIndiv->rebuildFromSP(optimalCombination);
						
						// 【新增】：对 Gurobi 拼装出的解进行一次 LS 打磨，避免虚高
						this->ls->local_search_run(spIndiv, data->penaltyDepot, data->penaltyVeh);
						
						this->sp_injection_attempts++;
						
						// 【修改】：接收返回值，判断是否开辟了新种群，并传入 pool 用于均值记录
						int spNewPopIdx = this->popman->injectSPSolution(spIndiv, this->routePool);
						
						// 【新增】：核心联动孵化逻辑
						if (spNewPopIdx > -1) {
							std::cout << "[SP Incubation] Rapidly expanding population to popMin for new SP configuration [" << spNewPopIdx << "]..." << std::endl;
							
							// 【修改3B：SP种群成功接管后，快速精准扩充到 popMin】
							while ((int)popman->popula[spNewPopIdx].size() < data->popMin) {
								initialSol->initial_solution(temIndiv, spNewPopIdx);
								ls->local_search_run(temIndiv, data->penaltyDepot, data->penaltyVeh);
								popman->addIndividual(temIndiv, spNewPopIdx, false, this->routePool);
							}
							getAverageValue(spNewPopIdx);
							
							// 2. 刷新全局哈希表（照搬你的原版游离种群防护逻辑）
							long long int correct_var = 0, correct_var1 = 0, correct_var2 = 0;
							for (auto it = data->compactLocConfigure[spNewPopIdx].begin(); it != data->compactLocConfigure[spNewPopIdx].end(); it++) {
								int seed_val = *it - data->numV;
								correct_var = pow(2, seed_val);
								correct_var1 += correct_var;
								correct_var2 += correct_var + 9527 * seed_val;
							}
							data->dimTabu1[correct_var1 % data->dimension] = spNewPopIdx;
							data->dimTabu2[correct_var2 % data->dimension] = spNewPopIdx;
						}
					} else {
						std::cout << "[SP Info] Gurobi finished but no better set partitioning was found.\n" << std::endl;
					}
				}

				/*re-start procedure*/
				if (data->nbIterNonImpro > data->maxIterNonProd){
					// 1. 计算剩余时间（秒）
					double elapsedTime = (double)(clock() - data->startTime) / CLOCKS_PER_SEC;
					double remainingTime = data->timeLimit - elapsedTime;
					
					// 2. 计算剩余迭代次数
					int remainingIters = data->iterLimit - data->iteration;

					// 3. 拦截判定：如果剩余迭代不足 50000 或 剩余时间不足 10 分钟（600秒），则拒绝重启
					if (remainingIters < 50000 || remainingTime < 600.0) {
						std::cout << "  [Intercepted] Restart aborted. Remaining Iters: " << remainingIters 
								  << ", Remaining Time: " << remainingTime << "s. Focused search continued." << std::endl;
						
						// 重置无改进计数器，避免在接下来的每一步都重复进入此分支并频繁打印
						data->nbIterNonImpro = 0; 
					} else {
						// 资源充足，执行正常的大重启
						std::cout << "  [Triggered] Executing Global Restart..." << std::endl;
						popman->reStart();
						initialPop();
					}
				}
				data->nbIterNonImpro++;
			}
			///////////////////
			if (data->iteration == data->iterLimit/2)
				popman->reduceHalfSubPop();

			if (data->iteration == data->iterLimit * 0.75)
				popman->reduceHalfSubPop();

			/////////
			if (data->stopCondition(global->fit)){
				global->isRight();
				global->outputBestSolution();
				std::cout<<global->fit<<std::endl;
				std::cout<<"----The memetic algorithm is finished, time spends: "<<(double)clock()/(double)CLOCKS_PER_SEC<<std::endl;
				return;
			}
		}
	}
}
