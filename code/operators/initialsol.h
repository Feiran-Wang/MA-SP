/******************************************************************************
 * File: initialsol.h
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description: 
 *   Heuristics for generating the initial population and starting routes.
 ******************************************************************************/

#ifndef INITIALSOL_H_
#define INITIALSOL_H_
#include "SolverContext.h"
#include "Individual.h"
#include "RepairProcedure.h"
#include "RouteKey.h"
#include <cstddef>
#include <vector>

namespace hgasp {

// Shared, testable bounds contract for the exact-constructor insertion
// commit.  The route index and insertion position are validated before any
// vector subscript or iterator arithmetic occurs.
void requireValidExactInsertion(
	std::size_t routeCount,
	std::size_t routeIndex,
	std::size_t routeSize,
	std::size_t insertionPosition);

}  // namespace hgasp

class initial_sol {
public:
	initial_sol(SolverContext * data,RepairProcedure *repair);
	~initial_sol() = default;
	// CLRP follows the HGAMP capacity constructor.  Hard-TW profiles always use
	// the active temporal evaluator and never fall back to the legacy builder.
	// A failed build leaves an empty, explicitly infeasible Individual.
	bool initial_solution(Individual *s, int rankPop);

	// Builds a capacity-only seed whose used depots exactly match the fixed
	// configuration published at populationRank.  Construction is
	// transactional: false leaves target empty and infeasible, while true
	// guarantees complete coverage and all active capacity constraints.
	bool initialCapacityOnlyExactConfiguration(
		Individual *target,
		int populationRank);
	bool initialCapacityOnlyExactConfiguration(
		Individual *target,
		const hgasp::DepotConfigurationKey& configuration);

	// Builds a hard-TW seed using only the explicitly supplied depots.  A
	// successful return additionally guarantees that every supplied depot is
	// active, so the actual configuration equals the requested key.
	bool initialConstraintAwareExactConfiguration(
		Individual *target,
		const hgasp::DepotConfigurationKey& configuration);

	// Reconstructs a CLRP solution from a complete customer permutation using
	// cheapest insertion filtered by the active constraints. This is the safe
	// repair boundary used
	// after crossover: no linked-list mutation occurs until a complete,
	// independently validated route plan has been found.
	bool reconstructConstraintFeasiblePlan(
		Individual *s,
		const std::vector<int>& customerOrder,
		int rankPop = -1);
	bool reconstructConstraintFeasiblePlan(
		Individual *s,
		const std::vector<int>& customerOrder,
		const std::vector<bool>& allowedDepots,
		int rankPop = -1);
	bool reconstructConstraintFeasiblePlan(
		Individual *s,
		const std::vector<int>& customerOrder,
		const std::vector<std::vector<int>>& trustedBlocks,
		int rankPop = -1);
	bool reconstructConstraintFeasiblePlan(
		Individual *s,
		const std::vector<int>& customerOrder,
		const std::vector<std::vector<int>>& trustedBlocks,
		const std::vector<bool>& allowedDepots,
		int rankPop = -1);

private:
	Individual * s;
	SolverContext * data;
	RepairProcedure *repair;
	std::vector<int >sque;
	std::vector<int> customer;
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
	bool initialCapacityOnly(Individual *target, int populationRank);
	bool initialConstraintAware(Individual *target, int populationRank);


	//parameters
	int remainDem = 0;
	std::vector<bool> locUsed;
	std::vector<bool> cusUsed;

};
#endif /* INITIALSOL_H_ */
