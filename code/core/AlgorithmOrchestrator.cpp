/******************************************************************************
 * File: AlgorithmOrchestrator.cpp
 * Author: Wang
 * Date: 2026-06-24
 * Description: 
 *   High-level controller managing the algorithm lifecycle and orchestrating modules.
 ******************************************************************************/

#include "AlgorithmOrchestrator.h"
#include <iostream>

AlgorithmOrchestrator::AlgorithmOrchestrator(read_data * data) {
    this->data = data;
    this->routeGenerator = std::unique_ptr<RouteGenerator>(new RouteGenerator(data));
    this->routePool = std::unique_ptr<RoutePool>(new RoutePool(data));
    this->spSolver = std::unique_ptr<SPSolver>(new SPSolver(data));
}

AlgorithmOrchestrator::~AlgorithmOrchestrator() {
    // std::unique_ptr will automatically delete the resources
}

void AlgorithmOrchestrator::run() {
    std::cout << "---- Start the search----" << std::endl;
    
    // 1. Initialization
    routeGenerator->preprocessingLocationConfiguration();
    
    data->loopp = 0;
    data->nbIterNonImpro = 1;
    
    routeGenerator->initialPop(routePool.get());

    std::cout << "[RoutePool] 正在建立初始精英池基线..." << std::endl;
    for (int i = 0; i < data->numLocConfiguration + 1; i++) {
        for (Individual* indiv : routeGenerator->popman->popula[i]) {
            if (indiv->isFeasible) {
                routePool->extractRoutes(indiv);              
                routePool->recordSolutionFitness(indiv->fit); 
            }
        }
    }
    std::cout << "[RoutePool] 初始路径池建立完成! 均值门槛: " << std::fixed << routePool->getAverageFitness() << std::endl;

    routeGenerator->getAverageValue(-1);

    data->iteration = 0;
    
    while (true) {
        // 2. Pricing / Column Generation
        int subPop1 = routeGenerator->selectsubPop(-1);
        int subPop2 = routeGenerator->selectsubPop(subPop1);
        
        int sizeSubPop = routeGenerator->generateEAXChildren(subPop1, subPop2);
        
        for (int i = 0; i < sizeSubPop; i++) {
            bool isFeasible = routeGenerator->refineChild(i);
            if (isFeasible) {
                // 3. Pool Insertion & Population Integration
                routeGenerator->integrateToPopulation(i, routePool.get());
                
                routeGenerator->printProgress(i);
                
                data->iteration++;
                
                // 4. Matheuristic Trigger: Set Partitioning Master Problem
                if (data->iteration > 0 && data->iteration % data->spTriggerFrequency == 0) {
                    
                    int current_pool_size = routePool->getPool().size();
                    std::cout << "\n[SP MONITOR] Iteration " << data->iteration 
                              << " | RoutePool Size: " << current_pool_size << std::endl;
                              
                    std::vector<StandaloneRoute> optimalCombination = spSolver->solve(routePool.get());

                    if (current_pool_size < 10) {
                        std::cout << "[SP Warning] RoutePool is too small (" << current_pool_size << "). Gurobi might not find any improvement." << std::endl;
                    }
                
                    if (!optimalCombination.empty()) {
                        std::cout << "[SP Success] Gurobi found a valid configuration!" << std::endl;
                        
                        Individual* spIndiv = new Individual();
                        spIndiv->define(data);
                        spIndiv->rebuildFromSP(optimalCombination);
                        
                        routeGenerator->ls->local_search_run(spIndiv, data->penaltyDepot, data->penaltyVeh);
                        
                        routeGenerator->injectIncumbent(spIndiv, routePool.get());
                    } else {
                        std::cout << "[SP Info] Gurobi finished but no better set partitioning was found.\n" << std::endl;
                    }
                }

                routeGenerator->checkStagnationAndRestart(routePool.get());
                data->nbIterNonImpro++;

                routeGenerator->checkSubPopReduction();

                if (routeGenerator->checkStopCondition()) {
                    return;
                }
            }
        }
    }
}
