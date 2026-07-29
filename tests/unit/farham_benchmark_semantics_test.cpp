#include "ProblemMode.h"
#include "RouteEvaluator.h"
#include "read_data.h"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

std::mt19937 global_rng;

namespace {

constexpr double kTolerance = 1.0e-9;

void require(const bool condition, const std::string& message) {
	if (!condition) {
		throw std::runtime_error(message);
	}
}

void requireClose(
	const double actual,
	const double expected,
	const std::string& label) {
	if (std::fabs(actual - expected) > kTolerance) {
		throw std::runtime_error(
			label + ": expected " + std::to_string(expected) +
			", got " + std::to_string(actual));
	}
}

int findOriginalId(
	const std::vector<int>& originalIds,
	const int requestedId,
	const std::string& kind) {
	for (std::size_t index = 0; index < originalIds.size(); ++index) {
		if (originalIds[index] == requestedId) {
			return static_cast<int>(index);
		}
	}
	throw std::runtime_error(
		"missing original " + kind + " id " +
		std::to_string(requestedId));
}

std::vector<int> internalCustomerSequence(
	const read_data& data,
	const std::vector<int>& originalIds) {
	std::vector<int> sequence;
	sequence.reserve(originalIds.size());
	for (const int originalId : originalIds) {
		sequence.push_back(findOriginalId(
			data.originalCustomerIds, originalId, "customer"));
	}
	return sequence;
}

struct GoldenRoute {
	std::vector<int> originalCustomers;
	double travelCost;
	double returnTime;
};

void testFarhamGoldenSemantics(const std::filesystem::path& root) {
	const std::filesystem::path instancePath =
		root / "instances/set_F/R101_s1_3_10.txt";

	read_data data;
	data.read_fun(
		instancePath.string(),
		"NULL",
		(root / "farham_benchmark_semantics_solution.txt").string(),
		2687.0,
		1,
		0,
		0,
		hgasp::ProblemMode::ClrpTw);

	require(
		data.problemMode == hgasp::ProblemMode::ClrpTw,
		"Farham instance was not loaded in CLRP-TW mode");
	require(
		data.numV == 10 && data.numL == 3,
		"Farham declared-prefix dimensions changed");

	const int depot = findOriginalId(
		data.originalDepotIds, 1, "depot");
	const int depotNode = data.numV + depot;
	const int customerOne = findOriginalId(
		data.originalCustomerIds, 1, "customer");

	// Depot 1 is at (16,29), while customer 1 is at (41,49).
	// Their exact Euclidean distance is sqrt(1025), so this arc distinguishes
	// the benchmark's floor-Euclidean semantics from an exact-distance matrix.
	const double exactDistance = std::sqrt(1025.0);
	require(
		exactDistance > 32.0 + kTolerance &&
			exactDistance < 33.0 - kTolerance,
		"chosen golden arc no longer distinguishes floor from exact distance");
	requireClose(
		data.D[depotNode][customerOne],
		std::floor(exactDistance),
		"Farham travel cost floor rounding");
	requireClose(
		data.T[depotNode][customerOne],
		std::floor(exactDistance),
		"Farham travel time floor rounding");

	const std::vector<GoldenRoute> goldenRoutes = {
		{{7, 10, 1}, 82.0, 203.0},
		{{2, 6}, 47.0, 118.0},
		{{9, 3, 4}, 129.0, 207.0},
		{{5, 8}, 29.0, 120.0},
	};

	HardTimeWindowConstraint hardTimeWindows;
	RouteEvaluator evaluator(data, hardTimeWindows);
	double totalTravelCost = 0.0;
	int totalDepotLoad = 0;
	for (std::size_t routeIndex = 0;
		 routeIndex < goldenRoutes.size();
		 ++routeIndex) {
		const GoldenRoute& golden = goldenRoutes[routeIndex];
		const RouteEvaluation evaluation = evaluator.evaluate(
			depot,
			internalCustomerSequence(data, golden.originalCustomers),
			EvaluationDetail::FullSchedule);

		const std::string label =
			"golden route " + std::to_string(routeIndex + 1);
		require(
			evaluation.capacityFeasible,
			label + " violates vehicle capacity");
		require(
			evaluation.temporalFeasible,
			label + " violates a hard time window");
		require(
			evaluation.feasible(),
			label + " is not route-feasible");
		require(
			evaluation.schedule.size() ==
				golden.originalCustomers.size(),
			label + " schedule length changed");
		requireClose(
			evaluation.travelCost,
			golden.travelCost,
			label + " travel cost");
		requireClose(
			evaluation.returnTime,
			golden.returnTime,
			label + " depot return time");

		totalTravelCost += evaluation.travelCost;
		totalDepotLoad += evaluation.load;
	}

	require(
		totalDepotLoad <= data.invLoc[depotNode],
		"golden solution violates depot capacity");
	const double objective =
		data.openCostLoc[depotNode] +
		static_cast<double>(goldenRoutes.size()) * data.usingCostVeh +
		totalTravelCost;
	requireClose(totalTravelCost, 287.0, "golden total travel cost");
	requireClose(objective, 2687.0, "golden solution objective");

	// Swapping customers 10 and 1 in [7,10,1] yields [7,1,10].
	// Customer 1 cannot start before 161, so customer 10 is then reached
	// strictly after its due time 134.
	const RouteEvaluation swapped = evaluator.evaluate(
		depot,
		internalCustomerSequence(data, {7, 1, 10}),
		EvaluationDetail::FullSchedule);
	require(
		swapped.capacityFeasible,
		"swapped route unexpectedly violates capacity");
	require(
		!swapped.temporalFeasible && !swapped.feasible(),
		"known-infeasible swapped route passed hard time windows");
	require(
		swapped.temporalViolation ==
			TemporalViolationKind::CustomerDueTime,
		"swapped route violation was not classified as customer lateness");
	require(
		swapped.violatingCustomer >= 0 &&
			data.originalCustomerIds[
				static_cast<std::size_t>(swapped.violatingCustomer)] == 10,
		"swapped route did not identify original customer 10");
}

}  // namespace

int main(int argc, char* argv[]) {
	try {
		const std::filesystem::path root =
			argc > 1 ? std::filesystem::path(argv[1])
					 : std::filesystem::current_path();
		testFarhamGoldenSemantics(root);
		std::cout << "farham benchmark semantics test passed\n";
		return 0;
	} catch (const std::exception& error) {
		std::cerr << "farham benchmark semantics test failed: "
				  << error.what() << '\n';
		return 1;
	}
}
