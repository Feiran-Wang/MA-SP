/******************************************************************************
 * File: initialsol.h
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description: 
 *   Heuristics for generating the initial population and starting routes.
 ******************************************************************************/

#ifndef INITIALSOL_H_
#define INITIALSOL_H_
#include "read_data.h"
#include "Individual.h"
#include "RepairProcedure.h"
#include <vector>
class initial_sol {
public:
	initial_sol(read_data * data,RepairProcedure *repair);
	virtual ~initial_sol();
	// Existing CLRP callers may ignore the return value.  CLRP-TW callers
	// should reject a false result; a failed build leaves an empty, explicitly
	// infeasible Individual.
	bool initial_solution(Individual *s, int rankPop);

	// Rebuilds a CLRP-TW solution from a complete customer permutation using
	// cheapest feasible insertion.  This is the safe repair boundary used
	// after crossover: no linked-list mutation occurs until a complete,
	// independently validated route plan has been found.
	bool rebuildTimeWindowFeasible(
		Individual *s,
		const std::vector<int>& customerOrder,
		int rankPop = -1);
private:
	Individual * s;
	read_data * data;
	RepairProcedure *repair;
	std::vector<int >sque;
	int *customer;
	int numV;
	int numL;
	int numC;
	int rankPop;

	//methods
	void greedyInsert();
	void insertCustomerToLocation(int &customer, int &loc, int newCus);
	double get_delta(int &in, Node *tem);
	void insertClient(Node *U, Node *V);
	// new initial solutions
	void newGreedyInsert();
	void insertCusToLoc(int loc);
	void remedyTheSolution(std::vector<int> location);
	void clearFailedTimeWindowSolution(Individual *target);


	//parameters
	int probDeterUsingLarLoc = 50;
	int remainDem = 0;
	bool *locUsed;
	bool *cusUsed;
	double penaltyDepot;
	double penaltyVeh;

};
#endif /* INITIALSOL_H_ */
