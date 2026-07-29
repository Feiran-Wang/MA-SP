#include "Individual.h"
#include "RouteGenerator.h"
#include "crossover.h"
#include "popManager.h"
#include "read_data.h"
#include "RepairProcedure.h"

#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

std::mt19937 global_rng;

struct ReadDataLoopSafetyTestAccess {
    static void spanningTree(read_data& data) {
        data.spanningTreeForLoc();
    }
    static std::size_t includedCustomers(
            const read_data& data,
            const int depot) {
        return data.locIncCus[static_cast<std::size_t>(depot)].size();
    }
    static void tabuConfiguration(read_data& data) {
        data.tabuSearchFindCompactConfiguration();
    }
    static void constructedConfigurations(read_data& data) {
        data.constructedDepotConfigurations();
    }
    static void readDepotConfigurations(read_data& data) {
        data.readDepotConfigurations();
    }
};

namespace {

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

StandaloneRoute makeRoute() {
    StandaloneRoute route;
    route.locId = 0;
    route.sequence = {0, 1, 2, 3};
    route.totalLoad = 4;
    route.routeCost = 5.0;
    return route;
}

void initializeToyData(read_data& data) {
    data.problemMode = hgasp::ProblemMode::Clrp;
    data.numV = 4;
    data.numL = 1;
    data.numC = 5;
    data.numLocConfiguration = 0;
    data.define();
    data.vehCap = 4;
    data.totalDemand = 4;
    data.usingCostVeh = 0.0;
    for (int customer = 0; customer < data.numV; ++customer) {
        data.demCus[customer] = 1;
    }
    data.invLoc[4] = 4;
    data.openCostLoc[4] = 0.0;
    for (int from = 0; from < data.numC; ++from) {
        for (int to = 0; to < data.numC; ++to) {
            data.D[from][to] = from == to ? 0.0 : 1.0;
            data.T[from][to] = data.D[from][to];
        }
    }
}

void testExactIterationBoundary() {
    read_data data;
    data.timeLimit = 0;
    data.iterLimit = 3;
    data.startTime = clock();
    data.meetOptimal = false;
    data.isOptimal = 0;

    data.iteration = 2;
    require(
        !data.stopCondition(100.0),
        "search stopped before reaching its iteration budget");
    data.iteration = 3;
    require(
        data.stopCondition(100.0),
        "search did not stop exactly at its iteration budget");
}

void testCpuTimeBoundary() {
    read_data data;
    data.timeLimit = 1;
    data.iterLimit = 100;
    data.iteration = 0;
    data.meetOptimal = false;
    data.isOptimal = 0;
    data.startTime =
        clock() - static_cast<clock_t>(2 * CLOCKS_PER_SEC);

    require(
        data.elapsedCpuSeconds() >= 1.0,
        "elapsed CPU time did not use the configured search origin");
    require(
        data.stopCondition(100.0),
        "search time limit did not use process CPU time");
}

void testGlobalIncumbentClock() {
    read_data data;
    data.resetSearchCpuClock();
    data.startTime =
        clock() - static_cast<clock_t>(2 * CLOCKS_PER_SEC);
    data.recordGlobalIncumbent();

    require(
        data.meetBestTime >= 1.0,
        "global-incumbent time did not use the search CPU origin");
}

void testZeroOffspringContract() {
    read_data data;
    initializeToyData(data);

    Individual father;
    father.define(&data);
    father.rebuildFromSP({makeRoute()});
    Individual mother;
    mother.define(&data);
    mother.rebuildFromSP({makeRoute()});

    popManager manager(&data);
    crossover eax(&data, &manager);
    std::vector<Individual> offspring(4);
    for (Individual& child : offspring) {
        child.define(&data);
        child.initilization();
    }

    int offspringCount = 12345;
    const bool generated = eax.gEAX(
        &father, &mother, offspring.data(), offspringCount);
    require(!generated, "identical parents unexpectedly produced offspring");
    require(
        offspringCount == 0,
        "EAX did not initialize the output count on a zero-child path");
}

void testPopulationRejectsInfeasibleIndividual() {
    read_data data;
    initializeToyData(data);
    data.problemMode = hgasp::ProblemMode::ClrpTw;

    popManager manager(&data);
    Individual empty;
    empty.define(&data);
    empty.initilization();
    empty.isFeasible = false;

    require(
        manager.addIndividual(&empty, 0, true) == -1,
        "population accepted an infeasible individual");
    require(
        manager.popula[0].empty(),
        "population cloned an infeasible individual before rejection");
}

void testTwoMemberParentSelectionTerminates() {
    read_data data;
    initializeToyData(data);

    popManager manager(&data);
    Individual* first = new Individual();
    first->define(&data);
    first->rebuildFromSP({makeRoute()});
    Individual* second = new Individual();
    second->define(&data);
    second->rebuildFromSP({makeRoute()});
    manager.popula[0].push_back(first);
    manager.popula[0].push_back(second);

    require(
        manager.selectionParentSolution(first, 0) == second,
        "two-member selection did not return the only distinct parent");
    require(
        manager.selectionParentSolution(second, 0) == first,
        "two-member selection depended on tournament winner ordering");
}

void testSubpopulationTournamentHasDeterministicFallback() {
    require(
        RouteGenerator::deterministicSubpopulationFallback(
            {2, 4, 6}, 2, {0U, 0U, 3U, 0U, 2U, 0U, 1U}) ==
            4,
        "fallback did not exclude the first parent population");
    require(
        RouteGenerator::deterministicSubpopulationFallback(
            {2}, 2, {0U, 0U, 3U}) == 2,
        "fallback rejected the only sufficiently diverse population");
}

void testSpanningTreeStopsWhenAllCustomersAreSelected() {
    read_data data;
    initializeToyData(data);
    data.invLoc[data.numV] = data.totalDemand + 100;
    ReadDataLoopSafetyTestAccess::spanningTree(data);
    require(
        data.locRatio.size() == 1U,
        "spanning-tree preprocessing did not finish the depot");
    require(
        ReadDataLoopSafetyTestAccess::includedCustomers(data, 0) ==
            static_cast<std::size_t>(data.numV),
        "spanning-tree preprocessing lost a customer above total-demand "
        "depot capacity");
}

void testTabuConstructionAcceptsExactAggregateCapacity() {
    read_data data;
    data.numV = 2;
    data.numL = 2;
    data.numC = 4;
    data.numLocConfiguration = 1;
    data.define();
    data.totalDemand = 2;
    data.invLoc[2] = 1;
    data.invLoc[3] = 1;
    data.locRatio = {{1.0, 2}, {1.0, 3}};
    ReadDataLoopSafetyTestAccess::tabuConfiguration(data);
    require(
        !data.LocConfigure.empty(),
        "exact aggregate depot capacity was rejected by tabu construction");
}

void testConstructedConfigurationAcceptsSufficientSeedDepot() {
    read_data data;
    data.numV = 2;
    data.numL = 10;
    data.numC = 12;
    data.numLocConfiguration = 1;
    data.dimension = 100;
    data.define();
    data.totalDemand = 2;
    for (int depot = 0; depot < data.numL; ++depot) {
        data.invLoc[data.numV + depot] = data.totalDemand;
        data.locRatio.push_back(
            {static_cast<double>(depot + 1), data.numV + depot});
    }
    ReadDataLoopSafetyTestAccess::constructedConfigurations(data);
    require(
        data.LocConfigure.size() == 1U,
        "single sufficient seed depot did not terminate configuration "
        "construction");
    require(
        data.LocConfigure.front().second.size() == 1U,
        "sufficient seed depot was unnecessarily expanded");
}

void testDepotConfigurationMissingSentinelFailsFast() {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() /
        "hgasp_missing_depot_configuration_sentinel.txt";
    {
        std::ofstream output(path);
        output << "1\n0 1\n";
    }
    read_data data;
    data.numV = 4;
    data.pathDepotCon = path.string();
    bool rejected = false;
    try {
        ReadDataLoopSafetyTestAccess::readDepotConfigurations(data);
    } catch (const std::runtime_error&) {
        rejected = true;
    }
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
    require(
        rejected,
        "depot configuration EOF without 'e' did not fail fast");
}

void testCrossoverEndpointSelectionIsTotal() {
    require(
        crossover::selectOpenEndpoint(true, false, 0) == 1,
        "EAX did not select an open predecessor endpoint");
    require(
        crossover::selectOpenEndpoint(true, true, -1) == -1,
        "EAX did not honor a valid successor preference");
    bool rejected = false;
    try {
        static_cast<void>(
            crossover::selectOpenEndpoint(false, false, 0));
    } catch (const std::logic_error&) {
        rejected = true;
    }
    require(
        rejected,
        "fully linked EAX endpoint did not fail fast");
}

void testRepairBudgetHasDeterministicBounds() {
    require(
        !RepairProcedure::repairBudgetExceeded(
            199, 99, 1, false),
        "repair budget stopped before its deterministic boundary");
    require(
        RepairProcedure::repairBudgetExceeded(
            200, 0, 1, false),
        "repair attempt boundary was not enforced");
    require(
        RepairProcedure::repairBudgetExceeded(
            0, 100, 1, false),
        "repair no-progress boundary was not enforced");
    require(
        RepairProcedure::repairBudgetExceeded(
            0, 0, 100, true),
        "repair ignored a CPU stop request");
}

}  // namespace

int main() {
    try {
        testExactIterationBoundary();
        testCpuTimeBoundary();
        testGlobalIncumbentClock();
        testZeroOffspringContract();
        testPopulationRejectsInfeasibleIndividual();
        testTwoMemberParentSelectionTerminates();
        testSubpopulationTournamentHasDeterministicFallback();
        testSpanningTreeStopsWhenAllCustomersAreSelected();
        testTabuConstructionAcceptsExactAggregateCapacity();
        testConstructedConfigurationAcceptsSufficientSeedDepot();
        testDepotConfigurationMissingSentinelFailsFast();
        testCrossoverEndpointSelectionIsTotal();
        testRepairBudgetHasDeterministicBounds();
        std::cout << "search_control_safety_test: PASS" << std::endl;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "search_control_safety_test: " << error.what()
                  << std::endl;
        return 1;
    }
}
