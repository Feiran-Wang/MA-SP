/******************************************************************************
 * File: RouteGenerator.h
 * Author: Wang
 * Date: 2026-06-24
 * Description: 
 *   Core engine for generating routes and managing heuristic operators.
 ******************************************************************************/

#ifndef ROUTE_GENERATOR_H_
#define ROUTE_GENERATOR_H_

#include "initialsol.h"
#include "read_data.h"
#include "Individual.h"
#include "LS.h"
#include "crossover.h"
#include "popManager.h"
#include "RepairProcedure.h"
#include "mutation.h"
#include "RoutePool.h"
#include <vector>
#include <memory>

class RouteGenerator {
public:
    RouteGenerator(read_data * data);
    virtual ~RouteGenerator();

    // 1. Initialization
    void preprocessingLocationConfiguration();
    void initialPop(RoutePool* pool);
    
    // 2. Generation & Heuristics
    int selectsubPop(int subpop1);
    int generateEAXChildren(int subPop1, int subPop2);
    bool refineChild(int childIndex); 
    
    // 3. Integration & Incumbent
    void integrateToPopulation(int childIndex, RoutePool* pool);
    void injectIncumbent(Individual* spIndiv, RoutePool* pool);
    
    // 4. System Status & Restarts
    void getAverageValue(int rank);
    void checkStagnationAndRestart(RoutePool* pool);
    void checkSubPopReduction();
    bool checkStopCondition();
    void printProgress(int childIndex);

    // Public members strictly needed for state access
    std::unique_ptr<popManager> popman;
    std::unique_ptr<LS> ls;
    read_data * data;

private:
    std::unique_ptr<initial_sol> initialSol;
    std::unique_ptr<crossover> cross;
    std::unique_ptr<RepairProcedure> repair;
    std::unique_ptr<mutation> mutationOperator;

    std::unique_ptr<Individual> global;
    std::unique_ptr<Individual> loc_best;
    std::unique_ptr<Individual> temIndiv;
    std::unique_ptr<Individual> bestIndivRestart;
    std::unique_ptr<Individual[]> subPop; // used to save offspring solutions
    
    Individual * fa;
    Individual * ma;

    std::vector<double> avgCostPop;
    
    // Helper function moved from solution
    void evolvePop(int rankPop, int &count, RoutePool* pool);
    void outputprocess(Individual *indiv);
};

#endif /* ROUTE_GENERATOR_H_ */
