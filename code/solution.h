// Author: Feiran
// Declare search coordination and owned run state.

#ifndef SOLUTION_H_
#define SOLUTION_H_
#include "initialsol.h"
#include "read_data.h"
#include "Individual.h"
#include "LS.h"
#include "crossover.h"
#include "popManager.h"
#include "RepairProcedure.h"
#include "mutation.h"
#include "RoutePool.h"
#include "SPSolver.h"
#include "StudyEvents.h"
class solution {
  public:
    solution(read_data *data, crossover *cross, initial_sol *initialSol, LS *ls, popManager *popman,
             RepairProcedure *repair, mutation *mutationOperator);
    virtual ~solution();
    void main_loop();
    RoutePool *routePool = nullptr;
    SPSolver *spSolver = nullptr;

    int sp_total_triggers = 0;
    int sp_found_solutions = 0;
    int sp_injection_attempts = 0;

  private:
    StudyEvents *study = nullptr;
    Individual *reportedBest = nullptr;
    LS *sideLS = nullptr;
    std::mt19937 sideRng;
    bool noFeasibleEnd = false;
    double initializationCPU = 0, postSPCPU = 0, newPopulationCPU = 0;
    std::set<int> reachedSPSchedule;
    void recordHGA(Individual *candidate);
    void recordSP(Individual *candidate, const std::string &source);
    bool stopRequested();
    void finishRun();
    void storeConfigurationSignature(int population);
    void runSP();
    void generation_population(int rankPop, bool isInitial);
    void evolvePop(int rankPop, int &count);
    void getAverageValue(int rank);
    int selectsubPop(int subpop1);
    void outputprocess(Individual *indiv);
    void initialPop();
    void preprocessingLocationConfiguration();
    void preprocessingTimeWindows();

    // Operator pointers are borrowed; the outer run owner outlives this coordinator.
    read_data *data;
    initial_sol *initialSol;
    LS *ls;
    crossover *cross;
    popManager *popman;
    RepairProcedure *repair;
    mutation *mutationOperator;

    Individual *global = nullptr;
    Individual *loc_best = nullptr;
    Individual *temIndiv = nullptr;
    Individual *bestIndivRestart = nullptr;
    Individual *subPop = nullptr;

    Individual *fa = NULL;
    Individual *ma = NULL;

    std::vector<double> avgCostPop;
};

#endif
