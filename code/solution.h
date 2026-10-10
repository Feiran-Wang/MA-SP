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
	solution(read_data * data, crossover *cross, initial_sol *initialSol,LS *ls,  popManager *popman, RepairProcedure *repair,mutation *mutationOperator);
	virtual ~solution();
	void main_loop();
	RoutePool* routePool;
    SPSolver* spSolver;

	int sp_total_triggers = 0;    // SP 条件触发次数
    int sp_found_solutions = 0;   // Gurobi 成功找到解的次数
    int sp_injection_attempts = 0; // 尝试注入种群的次数

private:
	StudyEvents* study = nullptr;
	Individual* reportedBest = nullptr;
	RepairProcedure* sideRepair = nullptr;
	LS* sideLS = nullptr;
	std::mt19937 sideRng;
	bool noFeasibleEnd = false;
	double initializationCPU = 0, postSPCPU = 0, newPopulationCPU = 0;
	std::set<int> reachedSPSchedule;
	void recordHGA(Individual* candidate);
	void recordSP(Individual* candidate, const std::string& source);
	bool stopRequested();
	void finishRun();
	void runSP();
	void generation_population(int rankPop, bool isInitial);
	void evolvePop(int rankPop, int &count);
	void getAverageValue(int rank);
	int selectsubPop(int subpop1);
	void outputprocess(Individual *indiv);
	void initialPop();
	void preprocessingLocationConfiguration();
	void preprocessingTimeWindows();
	//variables
	read_data * data;
	initial_sol * initialSol;
	LS * ls;
	crossover* cross;
	popManager * popman;
	RepairProcedure *repair;
	mutation *mutationOperator;
	///////////////////////////////////////////////////////////////////////////
	Individual * global;
	Individual * loc_best;
	Individual * temIndiv;// needs to be
	Individual * bestIndivRestart;
	Individual * subPop;//used to save offspring solutions                                              the size cannot exceed 100.
	//
	Individual * fa=NULL;
	Individual * ma=NULL;

	std::vector<double>avgCostPop;
};

#endif /* SOLUTION_H_ */
