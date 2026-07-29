/******************************************************************************
 * File: RouteGenerator.h
 * Author: Feiran Wang
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
#include "AdaptiveSearchAllocator.h"
#include "PopulationIncubationGuard.h"
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
    static int deterministicSubpopulationFallback(
        const std::vector<int>& eligible,
        int excludedPopulation,
        const std::vector<std::size_t>& populationSizes);
    int generateEAXChildren(int subPop1, int subPop2);
    bool refineChild(int childIndex); 
    
    // 3. Integration & Incumbent
    void integrateToPopulation(int childIndex, RoutePool* pool);
    SpInjectionResult injectIncumbent(
        std::unique_ptr<Individual> spIndiv,
        RoutePool* pool);
    
    // 4. System Status & Restarts
    void getAverageValue(int rank);
    void checkStagnationAndRestart(RoutePool* pool);
    void checkSubPopReduction();
    bool checkStopCondition();
    bool hasFeasibleIncumbent() const noexcept;
    const Individual* globalIncumbent() const noexcept;
    void updateDualGuidance(
        const std::vector<double>& customerDuals);
    void reportSearchOutcome(
        int firstPopulationIndex,
        int secondPopulationIndex,
        double objectiveImprovement,
        double cpuSeconds,
        std::size_t newColumns);
    void reportSpImprovement(bool improved);
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
    hgasp::AdaptiveSearchAllocator searchAllocator;
    int currentMutationProbabilityPercent = 15;
    bool spJustImproved = false;
    
    // Helper function moved from solution
    void evolvePop(int rankPop, int &count, RoutePool* pool);
    bool timeLimitReached() const noexcept;
    bool initializationTimeLimitReached() const noexcept;
    void incubatePopulation(
        int populationIndex,
        RoutePool* pool,
        const char* source);
    void registerPopulationConfiguration(int populationIndex);
    double populationDiversity(int populationIndex) const;
    void outputprocess(Individual *indiv);
};

#endif /* ROUTE_GENERATOR_H_ */
