#include "Individual.h"
#include "ProblemMode.h"
#include "RouteEvaluator.h"
#include "mutation.h"
#include "read_data.h"

#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

std::mt19937 global_rng(1729U);

namespace {

class ThrowingCommitMutation final : public mutation {
public:
	explicit ThrowingCommitMutation(read_data* data)
		: mutation(data) {}

	bool injected() const noexcept {
		return injected_;
	}

protected:
	void validateTimeWindowCommit(
			Individual* target) const override {
		if (!injected_) {
			injected_ = true;
			throw std::runtime_error(
				"injected TW mutation commit validation failure");
		}
		mutation::validateTimeWindowCommit(target);
	}

private:
	mutable bool injected_ = false;
};

void require(const bool condition, const std::string& message) {
	if (!condition)
		throw std::runtime_error(message);
}

void initializeTimeWindowData(read_data& data, const int customers) {
	data.problemMode = hgasp::ProblemMode::ClrpTw;
	data.numV = customers;
	data.numL = 1;
	data.numC = customers + 1;
	data.define();

	data.vehCap = std::max(1, customers / 2);
	data.totalDemand = customers;
	data.usingCostVeh = 10.0;
	for (int customer = 0; customer < customers; ++customer)
		data.demCus[customer] = 1;
	data.invLoc[customers] = customers;
	data.openCostLoc[customers] = 100.0;

	for (int from = 0; from < data.numC; ++from) {
		for (int to = 0; to < data.numC; ++to) {
			const double travel =
				from == to
					? 0.0
					: 1.0 +
						static_cast<double>(
							std::abs(from - to) % 3);
			data.D[from][to] = travel;
			data.T[from][to] = travel;
		}
	}

	data.readyTime.assign(
		static_cast<std::size_t>(customers), 0.0);
	data.dueTime.assign(
		static_cast<std::size_t>(customers), 100.0);
	data.serviceTime.assign(
		static_cast<std::size_t>(customers), 0.0);
	data.depotOpenTime = {0.0};
	data.depotCloseTime = {100.0};

	// Two early customers make an arbitrary distance-only swap unsafe while
	// still allowing singleton routes and several feasible mutations.
	if (customers >= 4) {
		data.dueTime[0] = 4.0;
		data.dueTime[3] = 5.0;
	}
}

StandaloneRoute evaluatedRoute(
		const read_data& data,
		std::vector<int> sequence) {
	HardTimeWindowConstraint timeWindows;
	const RouteEvaluation evaluation =
		RouteEvaluator(data, timeWindows).evaluate(0, sequence);
	require(evaluation.feasible(), "test route is not TW-feasible");

	StandaloneRoute route;
	route.locId = 0;
	route.sequence = std::move(sequence);
	route.totalLoad = evaluation.load;
	route.routeCost = evaluation.travelCost;
	route.returnTime = evaluation.returnTime;
	route.totalWaitingTime = evaluation.totalWaitingTime;
	route.temporalFeasible = evaluation.temporalFeasible;
	route.temporalViolationCustomer =
		evaluation.violatingCustomer;
	return route;
}

std::vector<std::pair<int, std::vector<int>>> topology(
		const Individual& individual,
		const read_data& data) {
	std::vector<std::pair<int, std::vector<int>>> result;
	for (int depot = 0; depot < data.numL; ++depot) {
		for (int slot = 0;
			 slot < individual.location[depot].maxVisit;
			 ++slot) {
			const Route* route =
				individual.location[depot].depots[slot].route;
			if (!route->used)
				continue;
			std::vector<int> sequence;
			const Node* node =
				individual.location[depot].depots[slot].next;
			while (!node->isDepot) {
				sequence.push_back(node->cour);
				node = node->next;
			}
			result.push_back({depot, std::move(sequence)});
		}
	}
	return result;
}

void independentlyValidate(
		Individual& individual,
		const read_data& data) {
	individual.evaluationDis();
	individual.decideFeasible();
	require(
		individual.isFeasible,
		"TW mutation left an infeasible Individual");
	individual.isRight();

	HardTimeWindowConstraint timeWindows;
	RouteEvaluator evaluator(data, timeWindows);
	std::vector<int> coverage(
		static_cast<std::size_t>(data.numV), 0);
	for (const auto& [depot, sequence] :
		 topology(individual, data)) {
		const RouteEvaluation evaluation =
			evaluator.evaluate(depot, sequence);
		require(
			evaluation.feasible(),
			"mutated route failed independent TW validation");
		for (const int customer : sequence)
			++coverage[static_cast<std::size_t>(customer)];
	}
	require(
		coverage ==
			std::vector<int>(
				static_cast<std::size_t>(data.numV), 1),
		"mutated plan changed customer coverage");
}

void testTwMutationCommitsOnlyFeasibleChanges() {
	read_data data;
	initializeTimeWindowData(data, 6);

	Individual individual;
	individual.define(&data);
	individual.rebuildFromSP({
		evaluatedRoute(data, {0, 1, 2}),
		evaluatedRoute(data, {3, 4, 5}),
	});
	independentlyValidate(individual, data);

	mutation operatorUnderTest(&data);
	bool observedMutation = false;
	for (int attempt = 0; attempt < 20; ++attempt) {
		const auto before = topology(individual, data);
		const bool applied =
			operatorUnderTest.mutationRun(&individual);
		independentlyValidate(individual, data);
		if (applied) {
			require(
				topology(individual, data) != before,
				"TW mutation reported success without a structural change");
			observedMutation = true;
			break;
		}
		require(
			topology(individual, data) == before,
			"failed TW mutation did not roll back its topology");
	}
	require(
		observedMutation,
		"TW mutation never found a feasible structural change");
}

void testTwMutationFailureLeavesIncumbentUntouched() {
	read_data data;
	initializeTimeWindowData(data, 1);

	Individual individual;
	individual.define(&data);
	individual.rebuildFromSP({
		evaluatedRoute(data, {0}),
	});
	const auto before = topology(individual, data);
	const double beforeFit = individual.fit;

	mutation operatorUnderTest(&data);
	require(
		!operatorUnderTest.mutationRun(&individual),
		"single-customer mutation reported a structural change");
	require(
		topology(individual, data) == before,
		"failed mutation changed the incumbent topology");
	require(
		std::fabs(individual.fit - beforeFit) <= 1.0e-9,
		"failed mutation changed the incumbent objective");
	independentlyValidate(individual, data);
}

void testTwMutationCommitFailureRollsBackAndPropagates() {
	read_data data;
	initializeTimeWindowData(data, 6);

	Individual individual;
	individual.define(&data);
	individual.rebuildFromSP({
		evaluatedRoute(data, {0, 1, 2}),
		evaluatedRoute(data, {3, 4, 5}),
	});
	const auto before = topology(individual, data);
	const double beforeFit = individual.fit;

	global_rng.seed(9173U);
	ThrowingCommitMutation operatorUnderTest(&data);
	bool observedInjectedFailure = false;
	for (int attempt = 0; attempt < 30; ++attempt) {
		try {
			const bool applied =
				operatorUnderTest.mutationRun(&individual);
			require(
				!applied,
				"fault-injected mutation committed without validation");
			require(
				topology(individual, data) == before,
				"unsuccessful pre-injection mutation changed topology");
		} catch (const std::runtime_error& error) {
			require(
				std::string(error.what()).find("injected") !=
					std::string::npos,
				"mutation propagated the wrong commit exception");
			observedInjectedFailure = true;
			break;
		}
	}
	require(
		operatorUnderTest.injected() && observedInjectedFailure,
		"test never reached the transactional commit fault");
	require(
		topology(individual, data) == before,
		"commit exception did not restore incumbent topology");
	require(
		std::fabs(individual.fit - beforeFit) <= 1.0e-9,
		"commit exception did not restore incumbent objective");
	independentlyValidate(individual, data);
}

void testTwMutationInputValidationPropagates() {
	read_data data;
	initializeTimeWindowData(data, 4);

	Individual individual;
	individual.define(&data);
	individual.rebuildFromSP({
		evaluatedRoute(data, {0, 1}),
		evaluatedRoute(data, {2, 3}),
	});
	const auto before = topology(individual, data);
	individual.fit += 1.0;
	const double malformedFit = individual.fit;

	mutation operatorUnderTest(&data);
	bool threw = false;
	try {
		(void)operatorUnderTest.mutationRun(&individual);
	} catch (const std::logic_error&) {
		threw = true;
	}
	require(
		threw,
		"malformed incumbent was silently treated as no mutation");
	require(
		topology(individual, data) == before,
		"input-validation exception changed incumbent topology");
	require(
		std::fabs(individual.fit - malformedFit) <= 1.0e-9,
		"input-validation exception changed incumbent objective");

	threw = false;
	try {
		(void)operatorUnderTest.mutationRun(nullptr);
	} catch (const std::invalid_argument&) {
		threw = true;
	}
	require(
		threw,
		"null mutation target did not propagate invalid_argument");
}

}  // namespace

int main() {
	try {
		testTwMutationCommitsOnlyFeasibleChanges();
		testTwMutationFailureLeavesIncumbentUntouched();
		testTwMutationCommitFailureRollsBackAndPropagates();
		testTwMutationInputValidationPropagates();
		std::cout << "tw_mutation_test: PASS\n";
		return 0;
	} catch (const std::exception& error) {
		std::cerr << "tw_mutation_test: "
				  << error.what() << '\n';
		return 1;
	}
}
