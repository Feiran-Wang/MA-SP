#include "Individual.h"
#include "RoutePool.h"
#include "read_data.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

std::mt19937 global_rng;

namespace {

int fail(const std::string& message) {
	std::cerr << "tw_model_integrity_test: " << message << std::endl;
	return 1;
}

StandaloneRoute route(
	std::vector<int> sequence,
	const double cost) {
	StandaloneRoute result;
	result.locId = 0;
	result.routeCost = cost;
	result.totalLoad = 5;
	result.sequence = std::move(sequence);
	return result;
}

void initialize(read_data& data) {
	data.problemMode = hgasp::ProblemMode::ClrpTw;
	data.numV = 2;
	data.numL = 1;
	data.numC = 3;
	data.define();
	data.vehCap = 10;
	data.demCus[0] = 2;
	data.demCus[1] = 3;
	data.totalDemand = 5;
	data.invLoc[2] = 10;
	data.openCostLoc[2] = 7.0;
	data.usingCostVeh = 4.0;
	data.spMaxPoolSize = 100;
	data.readyTime = {0.0, 0.0};
	data.dueTime = {12.0, 100.0};
	data.serviceTime = {0.0, 0.0};
	data.depotOpenTime = {0.0};
	data.depotCloseTime = {100.0};
	data.originalCustomerIds = {101, 205};
	data.originalDepotIds = {17};

	for (int from = 0; from < data.numC; ++from) {
		for (int to = 0; to < data.numC; ++to) {
			data.D[from][to] = 0.0;
			data.T[from][to] = 0.0;
		}
	}
	// depot -> 0 -> 1 -> depot is feasible with cost/return time 12.
	data.D[2][0] = data.T[2][0] = 2.0;
	data.D[0][1] = data.T[0][1] = 4.0;
	data.D[1][2] = data.T[1][2] = 6.0;
	// The reverse reaches customer 0 at time 16, after its due time 12.
	data.D[2][1] = data.T[2][1] = 8.0;
	data.D[1][0] = data.T[1][0] = 8.0;
	data.D[0][2] = data.T[0][2] = 8.0;
}

}  // namespace

int main() {
	read_data data;
	initialize(data);

	if (Individual::representationRouteLimit(data, 0) != data.numV) {
		return fail("CLRP-TW route-slot bound is not numV");
	}

	Individual empty;
	empty.define(&data);
	empty.decideFeasible();
	if (empty.isFeasible) {
		return fail("empty solution passed customer-coverage validation");
	}

	Individual valid;
	valid.define(&data);
	valid.rebuildFromSP({route({0, 1}, 12.0)});
	if (!valid.isFeasible ||
		!valid.route[0].temporalFeasible ||
		valid.route[0].returnTime != 12.0) {
		return fail("valid CLRP-TW route was not represented correctly");
	}

	RoutePool pool(&data);
	pool.extractRoutes(&valid);
	if (pool.getPool().size() != 1 ||
		!pool.getPool().front().temporalFeasible ||
		pool.getPool().front().returnTime != 12.0) {
		return fail("route pool did not retain verified temporal metadata");
	}
	data.dueTime[1] = 5.0;
	try {
		RoutePool rejectingPool(&data);
		rejectingPool.extractRoutes(&valid);
		return fail("route pool trusted stale individual feasibility");
	} catch (const std::runtime_error&) {
		// Expected: route-pool admission independently re-evaluates the route.
	}
	data.dueTime[1] = 100.0;

	Individual invalid;
	invalid.define(&data);
	try {
		invalid.rebuildFromSP({route({1, 0}, 24.0)});
		return fail("SP importer accepted a time-window-infeasible column");
	} catch (const std::invalid_argument&) {
		// Expected before any linked-list topology is changed.
	}

	const std::filesystem::path outputDirectory =
		std::filesystem::temp_directory_path() /
		"hgasp_tw_model_integrity_test_output";
	std::filesystem::create_directories(outputDirectory);
	data.pathInstance = "toy_set_f.txt";
	data.pathSolution = (outputDirectory / "solution.txt").string();
	valid.outputBestSolution();

	std::ifstream textFile(outputDirectory / "solution.txt");
	std::ostringstream textBuffer;
	textBuffer << textFile.rdbuf();
	const std::string text = textBuffer.str();
	if (!textFile ||
		text.find("The time basis is: \nprocess_cpu") ==
			std::string::npos ||
		text.find(
			"The time scope is: \nsearch_after_instance_loading") ==
			std::string::npos) {
		return fail("solution.txt does not declare its search CPU scope");
	}

	std::ifstream jsonFile(outputDirectory / "solution.json");
	std::ostringstream buffer;
	buffer << jsonFile.rdbuf();
	const std::string json = buffer.str();
	if (!jsonFile ||
		json.find("\"problem_mode\": \"clrp-tw\"") ==
			std::string::npos ||
			json.find("\"distance_policy\": \"euclidean-floor\"") ==
				std::string::npos ||
			json.find("\"basis\": \"process_cpu\"") ==
				std::string::npos ||
			json.find(
				"\"scope\": \"search_after_instance_loading\"") ==
				std::string::npos ||
			json.find("\"depot_id\": 17") == std::string::npos ||
		json.find("\"customers\": [101, 205]") ==
			std::string::npos ||
		json.find("\"schedule\":") == std::string::npos) {
		return fail("solution.json does not follow the CLRP-TW contract");
	}

	std::filesystem::remove_all(outputDirectory);
	std::cout << "tw_model_integrity_test: PASS" << std::endl;
	return 0;
}
