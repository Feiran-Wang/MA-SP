/******************************************************************************
 * File: RepairProcedure.h
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description: 
 *   Operator for repairing infeasible solutions and handling constraints.
 ******************************************************************************/

#ifndef REPAIRPROCEDURE_H_
#define REPAIRPROCEDURE_H_
#include <cstddef>
#include <cstdint>
#include <vector>

#include "Individual.h"
#include "popManager.h"

struct CapacityRepairResult {
	bool feasible = false;
	bool searchEntered = false;
	bool changed = false;
};

class RepairProcedure {
public:
	RepairProcedure(SolverContext * data, popManager * popman);
	virtual ~RepairProcedure() = default;
	bool repairForInitialSolution(Individual *s);
	CapacityRepairResult repairCapacityConstraint(
		Individual *subPop,
		double penaltyDepot,
		double penaltyVeh);
	double penaltyLocationFunction(Node *p, int loc);
	double penaltyExcessLocation(int loc, int inventory);
	double penaltyExcessRoute(int load);
	void updateThePenalty(int iden, Route *rou);
	const std::vector<int>& orderedActiveRouteIndices(
		std::uint64_t legacySlotCount);
	//
	void setLocalVariableRouteU();
	void setLocalVariableRouteV();
	double compute(bool istrue, double value);
	void Relocate_one();//  0-1 insert
	void Relocate_four();
	void Relocate_nine();
	void Relocate_ten();
	void updateTheMove();
	void swapNode(Node *U, Node *V);
	void insertNode(Node *U, Node *V);
	//

	void localRepair();
	bool isCapacityFeasible();
	void loadInventory();
	bool loadSolution();

	void finalCostCalibration();
	static bool repairBudgetExceeded(
		int attempts,
		int noProgressAttempts,
		int numCustomers,
		bool cpuStopRequested) noexcept;

	//
	void newRepair();


private:
	int numV = 0;
	int numL = 0;
	int numC = 0;
	SolverContext *data = nullptr;
	Individual *s = nullptr;
	int sizeSubPop = 0;
	double penaltyDepot = 0.0;
	double penaltyVeh = 0.0;
	//
	std::vector<int> tabuTable;
	std::vector<std::size_t> touchedTabuCells;
	int iteration = 0;
	Node *nodeU = nullptr;
	Node *nodeX = nullptr;
	Node *nodeUb = nullptr;
	Node *nodeVb = nullptr;
	Node *nodeV = nullptr;
	Node *nodeY = nullptr;
	Route *routeU = nullptr;
	Route *routeV = nullptr;
	int nodeUPrevIndex = 0, nodeUIndex = 0;
	int nodeXIndex = 0, nodeXNextIndex = 0;
	int nodeVPrevIndex = 0, nodeVIndex = 0;
	int nodeYIndex = 0, nodeYNextIndex = 0;
	int loadU = 0, loadX = 0, loadV = 0, loadY = 0;
	double min_delta = Max, delta = 0.0;
	int whichMove = 0;
	long long totalCapacityViolation() const noexcept;
	int tabuValue(int route, int customer) const noexcept;
	void setTabuValue(int route, int customer, int value);
	void clearTouchedTabu() noexcept;
	//
	int tabu_length = 30;


	popManager * popman = nullptr;
};

#endif /* REPAIRPROCEDURE_H_ */
