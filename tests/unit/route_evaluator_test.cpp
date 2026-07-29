#include "RouteEvaluator.h"
#include "read_data.h"

#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <vector>

std::mt19937 global_rng;

namespace {

int fail(const std::string& message) {
	std::cerr << "route_evaluator_test: " << message << std::endl;
	return 1;
}

bool close(const double lhs, const double rhs) {
	return std::fabs(lhs - rhs) <= 1.0e-9;
}

void initializeBaseInstance(read_data& data) {
	data.numV = 2;
	data.numL = 1;
	data.numC = 3;
	data.define();

	data.vehCap = 10;
	data.demCus[0] = 4;
	data.demCus[1] = 5;

	for (int from = 0; from < data.numC; ++from) {
		for (int to = 0; to < data.numC; ++to) {
			data.D[from][to] = 0.0;
			data.T[from][to] = 0.0;
		}
	}

	// Depot node is 2. The forward sequence [0, 1] costs 12 and takes 12
	// travel-time units before waiting and service.
	data.D[2][0] = 2.0;
	data.D[0][1] = 4.0;
	data.D[1][2] = 6.0;
	data.T[2][0] = 2.0;
	data.T[0][1] = 4.0;
	data.T[1][2] = 6.0;

	// The reverse direction is deliberately distinct.
	data.D[2][1] = 8.0;
	data.D[1][0] = 8.0;
	data.D[0][2] = 8.0;
	data.T[2][1] = 8.0;
	data.T[1][0] = 8.0;
	data.T[0][2] = 8.0;

	data.readyTime = {0.0, 0.0};
	data.dueTime = {100.0, 100.0};
	data.serviceTime = {0.0, 0.0};
	data.depotOpenTime = {0.0};
	data.depotCloseTime = {100.0};
}

}  // namespace

int main() {
	read_data data;
	initializeBaseInstance(data);

	NoTemporalConstraint noTimeWindows;
	HardTimeWindowConstraint hardTimeWindows;
	RouteEvaluator clrpEvaluator(data, noTimeWindows);
	RouteEvaluator twEvaluator(data, hardTimeWindows);

	const std::vector<int> forward = {0, 1};
	const RouteEvaluation clrp =
		clrpEvaluator.evaluate(0, forward);
	const RouteEvaluation neutralTw =
		twEvaluator.evaluate(0, forward, EvaluationDetail::FullSchedule);

	// A neutral hard-time-window policy must preserve the common CLRP
	// aggregates and feasibility result.
	if (!clrp.feasible() || !neutralTw.feasible() ||
		clrp.load != neutralTw.load ||
		clrp.customerCount != neutralTw.customerCount ||
		!close(clrp.travelCost, neutralTw.travelCost) ||
		!close(clrp.travelCost, 12.0)) {
		return fail("neutral time windows changed CLRP route aggregates");
	}

	data.vehCap = 8;
	const RouteEvaluation overCapacity =
		clrpEvaluator.evaluate(0, forward);
	if (overCapacity.capacityFeasible || !overCapacity.temporalFeasible ||
		overCapacity.feasible()) {
		return fail("capacity infeasibility was not represented correctly");
	}
	data.vehCap = 10;

	// Early arrival at customer 0 waits until time 5. Service then propagates
	// into customer 1's arrival.
	data.readyTime = {5.0, 0.0};
	data.dueTime = {5.0, 11.0};
	data.serviceTime = {2.0, 1.0};
	data.depotCloseTime = {18.0};
	const RouteEvaluation waiting =
		twEvaluator.evaluate(0, forward, EvaluationDetail::FullSchedule);
	if (!waiting.feasible() || waiting.schedule.size() != 2 ||
		!close(waiting.schedule[0].arrivalTime, 2.0) ||
		!close(waiting.schedule[0].waitingTime, 3.0) ||
		!close(waiting.schedule[0].serviceStartTime, 5.0) ||
		!close(waiting.schedule[0].departureTime, 7.0) ||
		!close(waiting.schedule[1].serviceStartTime, 11.0) ||
		!close(waiting.returnTime, 18.0) ||
		!close(waiting.totalWaitingTime, 3.0)) {
		return fail("waiting or service-time propagation is incorrect");
	}

	// Equality at both a customer due time and depot closing time is feasible.
	if (waiting.temporalViolation != TemporalViolationKind::None) {
		return fail("exact upper-bound equality was reported as a violation");
	}

	// The explicit epsilon accepts a numerically negligible customer overrun.
	data.dueTime[1] = 11.0 - 0.5e-9;
	const RouteEvaluation withinTolerance =
		twEvaluator.evaluate(0, forward);
	if (!withinTolerance.temporalFeasible) {
		return fail("customer due-time epsilon was not applied");
	}

	// A genuine customer lateness identifies the first violating customer.
	data.dueTime[1] = 11.0 - 2.0e-9;
	const RouteEvaluation lateCustomer =
		twEvaluator.evaluate(0, forward);
	if (lateCustomer.temporalFeasible ||
		lateCustomer.violatingCustomer != 1 ||
		lateCustomer.temporalViolation !=
			TemporalViolationKind::CustomerDueTime) {
		return fail("customer lateness was not classified correctly");
	}

	// Restore customer feasibility and make only the depot return late.
	data.dueTime[1] = 11.0;
	data.depotCloseTime[0] = 18.0 - 2.0e-9;
	const RouteEvaluation lateReturn =
		twEvaluator.evaluate(0, forward);
	if (lateReturn.temporalFeasible ||
		lateReturn.violatingCustomer != -1 ||
		lateReturn.temporalViolation !=
			TemporalViolationKind::DepotCloseTime) {
		return fail("late depot return was not classified correctly");
	}

	// Time feasibility is directional: the forward sequence remains feasible,
	// but its reverse cannot meet customer 0's due time.
	data.readyTime = {0.0, 0.0};
	data.dueTime = {12.0, 100.0};
	data.serviceTime = {0.0, 0.0};
	data.depotCloseTime = {100.0};
	const RouteEvaluation directedForward =
		twEvaluator.evaluate(0, forward);
	const RouteEvaluation directedReverse =
		twEvaluator.evaluate(0, std::vector<int>{1, 0});
	if (!directedForward.feasible() || directedReverse.temporalFeasible ||
		directedReverse.violatingCustomer != 0 ||
		!close(directedForward.travelCost, 12.0) ||
		!close(directedReverse.travelCost, 24.0)) {
		return fail("directed route semantics were not preserved");
	}

	std::cout << "route_evaluator_test: PASS" << std::endl;
	return 0;
}
