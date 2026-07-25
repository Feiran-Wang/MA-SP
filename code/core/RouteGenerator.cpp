/******************************************************************************
 * File: RouteGenerator.cpp
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description: 
 *   Core engine for generating routes and managing heuristic operators.
 ******************************************************************************/

#include "RouteGenerator.h"
#include "basic.h"
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <limits>

using namespace std;

RouteGenerator::RouteGenerator(read_data * data) {
    this->data=data;
    
    // Internalize creation of all operators
    this->popman = std::unique_ptr<popManager>(new popManager(data));
    this->cross = std::unique_ptr<crossover>(new crossover(data, popman.get()));
    this->repair = std::unique_ptr<RepairProcedure>(new RepairProcedure(data, popman.get()));
    this->ls = std::unique_ptr<LS>(new LS(data, repair.get()));
    this->initialSol = std::unique_ptr<initial_sol>(new initial_sol(data, repair.get()));
    this->mutationOperator = std::unique_ptr<mutation>(new mutation(data));
    
    temIndiv = std::unique_ptr<Individual>(new Individual());
    temIndiv->define(data);
    global = std::unique_ptr<Individual>(new Individual());
    global->define(data);
    global->initilization();
    loc_best = std::unique_ptr<Individual>(new Individual());
    loc_best->define(data);
    loc_best->initilization();
    bestIndivRestart = std::unique_ptr<Individual>(new Individual());
    bestIndivRestart->define(data);
    bestIndivRestart->initilization();

    popman->global = global.get();
    popman->bestIndivRestart = bestIndivRestart.get();
    int sizeSubPop = 100;
    subPop = std::unique_ptr<Individual[]>(new Individual[sizeSubPop]);
    for (int i = 0; i < sizeSubPop; i++){
        subPop[i].define(data);
        subPop[i].initilization();
    }
    global->fit = Max;
    bestIndivRestart->fit = Max;
    fa = NULL;
    ma = NULL;
}

RouteGenerator::~RouteGenerator(){
}

void RouteGenerator::preprocessingLocationConfiguration(){
    data->compactLocConfigure.clear();
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
            initialSol->initial_solution(temIndiv.get(), i);
            ls->local_search_run(temIndiv.get(),data->penaltyDepot, data->penaltyVeh);
            avgCost += temIndiv->fit;
            if (bestCost > temIndiv->fit)
                bestCost = temIndiv->fit;
            if (temIndiv->fit < global_cost){
                // to record the best solution given each depot configuration
                global_cost = temIndiv->fit;
                loc_best->initilization();
                popman->copyAllData(loc_best.get(), temIndiv.get());// record the best solution given the current depot configuration.
            }
        }
        avgCost = avgCost / (double)(data->popMin);// in this place, we use the best solution
        averageCost.push_back({bestCost, i});
        std::cout<<i<<"   tries   "<< bestCost<<std::endl;
    }
    std::sort(averageCost.begin(), averageCost.end());
    loc_best->evaluationDis();

    data->numLocConfiguration = numSubPopPara;//5
    data->compactLocConfigure.clear();
    for (int i=0;i < std::min(data->numLocConfiguration,initalNumDepots); i++)
        data->compactLocConfigure.push_back(data->LocConfigure[averageCost[i].second].second);
    data->numLocConfiguration = std::min(data->numLocConfiguration,initalNumDepots);
    
    popman->addIndividual(loc_best.get(), data->numLocConfiguration, true);// insert the local best solution to elite pop
    std::cout<<"Finishing initial filter"<<std::endl;
}

void RouteGenerator::getAverageValue(int rank){
    if (rank < 0){
        avgCostPop.clear();
        for (int i=0;i<data->numLocConfiguration + 1; i++){
            double cost = 0;
            for (int j=0;j<(int)popman->popula[i].size();j++){
                cost += popman->popula[i][j]->fit;
            }
            int ss = popman->popula[i].size();
            avgCostPop.push_back(cost / (double)ss);
        }
        
        // removing half population
        std::vector<std::pair<double,int>>rankOrder;
        for (int i=0;i<(int)avgCostPop.size();i++)
            rankOrder.push_back({-avgCostPop[i],i});
        sort(rankOrder.begin(),rankOrder.end());
        for (int i=0;i<data->numLocConfiguration / 2; i++){
            int subpop = rankOrder[i].second;
            for (Individual * indiv : popman->popula[subpop]) delete indiv ;
            popman->popula[subpop].clear();
        }
        
        long long int var, var1, var2; int seed;
        for (int i=0;i<data->numLocConfiguration;i++){
            if ((int)popman->popula[i].size() > 0){
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
    else{
        double cost = 0;
        for (int j=0;j<(int)popman->popula[rank].size();j++)
            cost += popman->popula[rank][j]->fit;
        int ss = popman->popula[rank].size();
        avgCostPop[rank] = cost / (double)ss;
    }
}

int RouteGenerator::selectsubPop(int subpop1){
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
        return (locConf[0] == subpop1) ? locConf[1] : locConf[0]; 
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

        if (target == subpop1 && (int)popman->popula[target].size() > 2) break;
        if (target != subpop1) break;
        
        if ((int)locConf.size() == 2) {
            target = (locConf[0] == subpop1) ? locConf[1] : locConf[0];
            break;
        }
    }
    return target;
}

void RouteGenerator::evolvePop(int rankPop, int &count, RoutePool* pool){
    if ((int)popman->popula[rankPop].size() <= 2){
        count = data->maxIterationPerLoc;
        return;
    }
    int sizeSubPop;
    ma = NULL;
    fa = popman->selectionParentSolution(ma, rankPop);
    ma = popman->selectionParentSolution(fa, rankPop);
    cross->gEAX(fa,ma,subPop.get(),sizeSubPop);
    for (int i =0; i<sizeSubPop; i++){
        repair->repairCapacityConstraint(&subPop[i], data->penaltyDepot, data->penaltyVeh);
        if (subPop[i].isFeasible){
            ls->local_search_run(&subPop[i], data->penaltyDepot, data->penaltyVeh);
            popman->addIndividual(&subPop[i], rankPop, true, pool);
        }
        count++;
    }
}

void RouteGenerator::initialPop(RoutePool* pool){
    popman->popula.erase(popman->popula.begin()+data->numLocConfiguration +1, popman->popula.end());
    int i;
    for ( i = 0; i<data->numLocConfiguration +1 ;i++){// an extra subPop
        std::cout<<i<<" th ";
        std::cout<<(double)clock()/(double)CLOCKS_PER_SEC<<std::endl;
        
        double bestFit=Max;
        for (int j = 0; j < data->popMin*4; j++){
            initialSol->initial_solution(temIndiv.get(), i);
            ls->local_search_run(temIndiv.get(),data->penaltyDepot, data->penaltyVeh);
            popman->addIndividual(temIndiv.get(), i, true, pool);
            if (bestFit>temIndiv->fit)
                bestFit=temIndiv->fit;
        }
        std::cout<<"The best fit is    "<<bestFit<<"  The time consuming  "<<(double)clock()/(double)CLOCKS_PER_SEC<<std::endl;
        
        int count = 0;
        if ((int)popman->popula[i].size() <= 1)
            continue;
        while (count < data->maxIterationPerLoc)
            evolvePop(i, count, pool);
    }
    std::cout<<"Finishing constructing number of initial population"<<std::endl;
}

int RouteGenerator::generateEAXChildren(int subPop1, int subPop2) {
    int sizeSubPop;
    ma = NULL;
    fa = popman->selectionParentSolution(ma, subPop1);
    ma = popman->selectionParentSolution(fa, subPop2);
    cross->gEAX(fa,ma,subPop.get(),sizeSubPop);
    return sizeSubPop;
}

bool RouteGenerator::refineChild(int childIndex) {
    repair->repairCapacityConstraint(&subPop[childIndex], data->penaltyDepot, data->penaltyVeh);
    if (subPop[childIndex].isFeasible){
        if (RandomInt(0, 99) < data->mutationProbabilityPercent)
            mutationOperator->mutationRun(&subPop[childIndex]);
        ls->local_search_run(&subPop[childIndex], data->penaltyDepot, data->penaltyVeh);
        return true;
    }
    return false;
}

void RouteGenerator::integrateToPopulation(int childIndex, RoutePool* pool) {
    std::vector<int> depot;
    long long int var1, var2, var;
    int whichSubPop;
    int isGenerateNewSubPop;

    if (!data->no_capacity_location){
        depot.clear();
        for (int j=0; j<data->numL;j++){
            if (subPop[childIndex].location[j].used)
                depot.push_back(j);
        }
        var1 = 0; var2 = 0;
        for (int j=0;j<(int)depot.size();j++){
            var = 1ULL << depot[j];
            var1 += var;
            var2 += var + 9527 * depot[j];
        }
        auto it1 = data->dimTabu1.find(var1 % data->dimension);
        auto it2 = data->dimTabu2.find(var2 % data->dimension);
        if (it1 != data->dimTabu1.end() && it2 != data->dimTabu2.end() && it1->second == it2->second && it2->second > -1)
            whichSubPop = it2->second;
        else
            whichSubPop = data->numLocConfiguration;
        
        isGenerateNewSubPop = popman->addIndividual(&subPop[childIndex], whichSubPop, false, pool);
        getAverageValue(whichSubPop);
        
        if (isGenerateNewSubPop > -1){
            while ((int)popman->popula[isGenerateNewSubPop].size() < data->popMin) {
                initialSol->initial_solution(temIndiv.get(), isGenerateNewSubPop);
                ls->local_search_run(temIndiv.get(), data->penaltyDepot, data->penaltyVeh);
                popman->addIndividual(temIndiv.get(), isGenerateNewSubPop, false, pool);
            }
            getAverageValue(isGenerateNewSubPop);
            
            long long int correct_var = 0, correct_var1 = 0, correct_var2 = 0;
            for (auto it = data->compactLocConfigure[isGenerateNewSubPop].begin(); it != data->compactLocConfigure[isGenerateNewSubPop].end(); it++) {
                int seed_val = *it - data->numV;
                correct_var = 1ULL << seed_val;
                correct_var1 += correct_var;
                correct_var2 += correct_var + 9527 * seed_val;
            }
            data->dimTabu1[correct_var1 % data->dimension] = isGenerateNewSubPop;
            data->dimTabu2[correct_var2 % data->dimension] = isGenerateNewSubPop;
        }
    }
    else {
        popman->addIndividual(&subPop[childIndex], 0, false, pool);
    }
}

void RouteGenerator::injectIncumbent(
        std::unique_ptr<Individual> spIndiv,
        RoutePool* pool) {
    int spNewPopIdx =
        popman->injectSPSolution(std::move(spIndiv), pool);

    // SP insertion can change an existing resident population without
    // creating a new configuration. Refresh every non-empty cached mean so
    // subsequent subpopulation tournaments observe the injected solution.
    for (int rank = 0; rank < data->numLocConfiguration + 1; ++rank) {
        if (!popman->popula[rank].empty()) {
            getAverageValue(rank);
        }
    }

    if (spNewPopIdx > -1) {
        std::cout << "[SP Incubation] Rapidly expanding population to popMin for new SP configuration [" << spNewPopIdx << "]..." << std::endl;
        
        while ((int)popman->popula[spNewPopIdx].size() < data->popMin) {
            initialSol->initial_solution(temIndiv.get(), spNewPopIdx);
            ls->local_search_run(temIndiv.get(), data->penaltyDepot, data->penaltyVeh);
            popman->addIndividual(temIndiv.get(), spNewPopIdx, false, pool);
        }
        getAverageValue(spNewPopIdx);
        
        long long int correct_var = 0, correct_var1 = 0, correct_var2 = 0;
        for (auto it = data->compactLocConfigure[spNewPopIdx].begin(); it != data->compactLocConfigure[spNewPopIdx].end(); it++) {
            int seed_val = *it - data->numV;
            correct_var = 1ULL << seed_val;
            correct_var1 += correct_var;
            correct_var2 += correct_var + 9527 * seed_val;
        }
        data->dimTabu1[correct_var1 % data->dimension] = spNewPopIdx;
        data->dimTabu2[correct_var2 % data->dimension] = spNewPopIdx;
    }
}

void RouteGenerator::checkStagnationAndRestart(RoutePool* pool) {
    if (data->nbIterNonImpro > data->maxIterNonProd){
        double elapsedTime = (double)(clock() - data->startTime) / CLOCKS_PER_SEC;
        const double remainingTime =
            data->timeLimit > 0
                ? data->timeLimit - elapsedTime
                : std::numeric_limits<double>::infinity();
        int remainingIters = data->iterLimit - data->iteration;

        if (remainingIters < 50000 || remainingTime < 600.0) {
            std::cout << "  [Intercepted] Restart aborted. Remaining Iters: " << remainingIters 
                        << ", Remaining Time: " << remainingTime << "s. Focused search continued." << std::endl;
            data->nbIterNonImpro = 0; 
        } else {
            std::cout << "  [Triggered] Executing Global Restart..." << std::endl;
            popman->reStart();
            initialPop(pool);
            
            std::cout << "[RoutePool] 正在建立初始精英池基线..." << std::endl;
            for (int i = 0; i < data->numLocConfiguration + 1; i++) {
                for (Individual* indiv : popman->popula[i]) {
                    if (indiv->isFeasible) {
                        pool->extractRoutes(indiv);
                        pool->recordSolutionFitness(indiv->fit);
                    }
                }
            }
            getAverageValue(-1);
        }
    }
}

void RouteGenerator::checkSubPopReduction() {
    if (data->iteration == data->iterLimit/2)
        popman->reduceHalfSubPop();

    if (data->iteration == data->iterLimit * 0.75)
        popman->reduceHalfSubPop();
}

bool RouteGenerator::checkStopCondition() {
    if (data->stopCondition(global->fit)){
        global->isRight();
        global->outputBestSolution();
        std::cout<<global->fit<<std::endl;
        std::cout<<"----The memetic algorithm is finished, time spends: "<<(double)clock()/(double)CLOCKS_PER_SEC<<std::endl;
        return true;
    }
    return false;
}

void RouteGenerator::outputprocess(Individual *indiv){
    std::cout<<"It: "<<data->iteration<<"--G-Fit: "<<std::setiosflags(ios::fixed)<<std::setprecision(2)<<float(global->fit);
    std::cout<<"  --R-Fit: "<<bestIndivRestart->fit;
    std::cout<<"  Curr Sol--"<<float(indiv->fit)<<std::setprecision(2);
    std::cout<<"  IT NoImpro--"<<data->nbIterNonImpro;
    std::cout<<"  Time comsuming--"<<float((double)clock()/(double)CLOCKS_PER_SEC)<<std::setprecision(2);
    std::cout<<std::endl;
    data->loopp = 0;
}

void RouteGenerator::printProgress(int childIndex){
    if (data->iteration % 1000 == 0) outputprocess(&subPop[childIndex]);
    
    if (data->iteration > 0 && data->iteration % 50000 == 0) {
        std::cout << ">>> [机制监控] \"gamma+1\" 游离种群动态逆袭替换机制累计生效: " 
                    << popman->reverseReplacementCount << " 次 | "
                    << "当前游离种群个体数量: " 
                    << popman->popula[data->numLocConfiguration].size() << std::endl;
    }
}
