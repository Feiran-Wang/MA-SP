#include "FeasibleInsertionFinder.h"
#include "Individual.h"
#include "ProblemMode.h"
#include "RouteEvaluator.h"
#include "initialsol.h"
#include "read_data.h"

#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <vector>

std::mt19937 global_rng(17U);

namespace {

int fail(const std::string& message) {
	std::cerr << "feasible_insertion_finder_test: "
			  << message << std::endl;
	return 1;
}

void initializeInstance(read_data& data) {
	data.numV = 3;
	data.numL = 2;
	data.numC = 5;
	data.define();

	data.problemMode = hgasp::ProblemMode::ClrpTw;
	data.vehCap = 8;
	data.totalDemand = 12;
	data.usingCostVeh = 10.0;
	data.demCus[0] = 4;
	data.demCus[1] = 4;
	data.demCus[2] = 4;
	for (int depot = 0; depot < data.numL; ++depot) {
		data.invLoc[data.numV + depot] = 8;
		data.openCostLoc[data.numV + depot] = 100.0;
	}

	for (int from = 0; from < data.numC; ++from) {
		for (int to = 0; to < data.numC; ++to) {
			const double value = from == to ? 0.0 : 50.0;
			data.D[from][to] = value;
			data.T[from][to] = value;
		}
	}

	const int depot0 = 3;
	const int depot1 = 4;
	data.D[depot0][0] = data.T[depot0][0] = 2.0;
	data.D[0][depot0] = data.T[0][depot0] = 2.0;
	data.D[depot0][2] = data.T[depot0][2] = 3.0;
	data.D[2][depot0] = data.T[2][depot0] = 3.0;
	data.D[0][2] = data.T[0][2] = 3.0;
	data.D[2][0] = data.T[2][0] = 3.0;

	data.D[depot1][1] = data.T[depot1][1] = 2.0;
	data.D[1][depot1] = data.T[1][depot1] = 2.0;
	data.D[depot1][2] = data.T[depot1][2] = 3.0;
	data.D[2][depot1] = data.T[2][depot1] = 3.0;
	data.D[1][2] = data.T[1][2] = 3.0;
	data.D[2][1] = data.T[2][1] = 3.0;

	data.readyTime = {0.0, 0.0, 0.0};
	data.dueTime = {5.0, 5.0, 20.0};
	data.serviceTime = {0.0, 0.0, 0.0};
	data.depotOpenTime = {0.0, 0.0};
	data.depotCloseTime = {30.0, 30.0};
}

}  // namespace

int main() {
	read_data data;
	initializeInstance(data);

	hgasp::FeasibleInsertionFinder finder(data);
	hgasp::FeasibleInsertionResult result;
	std::string diagnostic;
	if (!finder.build(
			std::vector<int>{2, 0, 1}, -1, result, &diagnostic)) {
		return fail("known feasible construction failed: " + diagnostic);
	}
	if (result.routes.size() != 2U ||
		std::fabs(result.objective - 232.0) > 1.0e-9) {
		return fail("cheapest insertion returned an unexpected plan");
	}

	HardTimeWindowConstraint hardTimeWindows;
	RouteEvaluator evaluator(data, hardTimeWindows);
	std::vector<int> coverage(3, 0);
	for (const StandaloneRoute& route : result.routes) {
		const RouteEvaluation evaluation =
			evaluator.evaluate(route.locId, route.sequence);
		if (!evaluation.feasible()) {
			return fail("returned route was not independently TW-feasible");
		}
		for (const int customer : route.sequence) {
			++coverage[static_cast<std::size_t>(customer)];
		}
	}
	if (coverage != std::vector<int>({1, 1, 1})) {
		return fail("returned route plan has invalid customer coverage");
	}

	Individual individual;
	individual.define(&data);
	initial_sol constructor(&data, nullptr);
	if (!constructor.rebuildTimeWindowFeasible(
			&individual, std::vector<int>{2, 0, 1}, -1) ||
		!individual.isFeasible) {
		return fail("value plan could not be committed to an Individual");
	}

	// Customer 1 cannot be reached from either depot by its new due time.
	// Failure must not leak the formerly feasible linked-list topology.
	data.dueTime[1] = 1.0;
	if (constructor.rebuildTimeWindowFeasible(
			&individual, std::vector<int>{2, 0, 1}, -1)) {
		return fail("infeasible customer was incorrectly accepted");
	}
	if (individual.isFeasible) {
		return fail("failed repair did not mark the Individual infeasible");
	}
	for (int depot = 0; depot < data.numL; ++depot) {
		if (individual.location[depot].used ||
			individual.location[depot].splitTime != 0) {
			return fail("failed repair left a partially routed solution");
		}
	}

	result.routes.push_back(StandaloneRoute{});
	if (finder.build(
			std::vector<int>{2, 0, 1}, -1, result, &diagnostic) ||
		!result.routes.empty() ||
		diagnostic.find("customer 1") == std::string::npos) {
		return fail("failure result or diagnostic contract was violated");
	}

	std::cout << "feasible_insertion_finder_test: PASS" << std::endl;
	return 0;
}
