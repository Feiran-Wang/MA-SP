#include "crossover.h"
#include "StudyEvents.h"
#include <memory>
#include <stdexcept>

std::mt19937 global_rng(17);

namespace {
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

struct Fixture {
    read_data data;
    Fixture() {
        data.numV = 8;
        data.numL = 4;
        data.numC = data.numV + data.numL;
        data.vehCap = 8;
        data.define();
        delete[] data.dimTabu1;
        delete[] data.dimTabu2;
        data.dimension = 257;
        data.dimTabu1 = new int[data.dimension];
        data.dimTabu2 = new int[data.dimension];
        std::fill(data.dimTabu1, data.dimTabu1 + data.dimension, -1);
        std::fill(data.dimTabu2, data.dimTabu2 + data.dimension, -1);
        data.totalDemand = data.numV;
        data.usingCostVeh = 10;
        data.iteration = 100;
        data.nbIterNonImpro = 0;
        data.meetBestTime = 0;
        data.numLocConfiguration = 1;
        data.compactLocConfigure = {{data.numV}};
        for (int i = 0; i < data.numC; ++i) {
            data.x[i] = i;
            data.y[i] = 0;
            data.invLoc[i] = 8;
            data.openCostLoc[i] = 100;
            for (int j = 0; j < data.numC; ++j)
                data.D[i][j] = i == j ? 0 : 1 + std::abs(i - j);
        }
        std::fill(data.demCus, data.demCus + data.numV, 1);
    }
};

const std::vector<int> ordered{0, 1, 2, 3, 4, 5, 6, 7};
const std::vector<int> medium{0, 2, 4, 6, 1, 3, 5, 7};
const std::vector<int> expensive{0, 7, 1, 6, 2, 5, 3, 4};

void makeRoutes(Individual& indiv, read_data& data,
                const std::vector<std::pair<int, std::vector<int>>>& routes) {
    indiv.initilization();
    for (const auto& plan : routes) {
        Route* route = indiv.location[plan.first].depots[0].route;
        Node* previous = route->depot;
        for (int customer : plan.second) {
            Node* node = &indiv.client[customer];
            previous->next = node;
            node->pre = previous;
            node->route = route;
            previous = node;
        }
        previous->next = route->endDepot;
        route->endDepot->pre = previous;
        indiv.updateRouteInfor(route->cour, -1);
    }
    for (int depotIndex = 0; depotIndex < data.numL; ++depotIndex) {
        int id = data.numV + depotIndex;
        indiv.updateLocationInfor(id);
    }
    indiv.evaluationDis();
    indiv.decideFeasible();
    require(indiv.isFeasible, "fixture must be feasible");
}

void makeSolution(Individual& indiv, read_data& data, int depot,
                  const std::vector<int>& sequence) {
    makeRoutes(indiv, data, {{depot, sequence}});
}

std::unique_ptr<Individual> solution(read_data& data, int depot,
                                     const std::vector<int>& sequence) {
    std::unique_ptr<Individual> result(new Individual());
    result->define(&data);
    makeSolution(*result, data, depot, sequence);
    return result;
}

void checkCopiedDepotState(read_data& data) {
    popManager populations(&data);
    auto source = solution(data, 0, ordered);
    auto destination = solution(data, 2, expensive);
    populations.copyAllData(destination.get(), source.get());
    for (int depot = 0; depot < data.numL; ++depot) {
        const NodeDepot& location = destination->location[depot];
        require(location.used == (depot == 0), "copied depot occupancy differs");
        require(location.remainInven == (depot == 0 ? data.totalDemand : 0),
                "closed depot retained its previous load");
        require(location.splitTime == (depot == 0 ? 1 : 0), "copied split count differs");
        int empty = 0;
        for (int slot = 0; slot < location.maxVisit; ++slot) {
            const Route& route = *location.depots[slot].route;
            const bool occupied = depot == 0 && slot == 0;
            require(route.used == occupied, "copied route retained stale occupancy");
            require(route.nbClients == (occupied ? data.numV : 0), "route customer count differs");
            if (!occupied) ++empty;
            require((location.emptyRoutes.count(route.cour) != 0) == !occupied,
                    "empty route is missing from the copied depot index");
        }
        require(static_cast<int>(location.emptyRoutes.size()) == empty, "copied empty-slot count differs");
    }
    populations.copyAllData(destination.get(), destination.get());
    require(destination->client[0].route != nullptr && destination->fit == source->fit,
            "self-copy destroyed a valid solution");
}

void checkSPInsertionAndTwoMemberSelection(read_data& data) {
    auto global = solution(data, 0, expensive);
    auto restart = solution(data, 0, expensive);
    global->fit = restart->fit = Max;
    popManager populations(&data);
    populations.global = global.get();
    populations.bestIndivRestart = restart.get();
    auto first = solution(data, 0, medium);
    auto second = solution(data, 0, expensive);
    populations.addIndividual(first.get(), 0, true);
    populations.addIndividual(second.get(), 0, true);
    require(populations.popula[0].size() == 2, "fixture did not retain two members");
    Individual* excluded = populations.popula[0].front();
    require(populations.selectionParentSolution(excluded, 0) == populations.popula[0].back(),
            "two-member selection did not return the admissible parent");
    auto sp = solution(data, 0, ordered);
    const double objective = sp->fit;
    populations.injectSPSolution(sp.release());
    require(populations.popula[0].size() == 3 && populations.popula[0].front()->fit == objective,
            "better SP solution was not inserted at its fitness rank");
    for (std::size_t i = 1; i < populations.popula[0].size(); ++i)
        require(populations.popula[0][i - 1]->fit <= populations.popula[0][i]->fit,
                "SP feedback broke population fitness order");
}

void checkInitialConfigurationAssignment(read_data& data) {
    const auto check = [&](const std::set<int>& resident, bool twoDepots, bool matches) {
        data.numLocConfiguration = 1;
        data.compactLocConfigure = {resident};
        auto global = solution(data, 0, expensive);
        auto restart = solution(data, 0, expensive);
        global->fit = restart->fit = Max;
        popManager populations(&data);
        populations.global = global.get();
        populations.bestIndivRestart = restart.get();
        auto candidate = solution(data, 0, ordered);
        if (twoDepots)
            makeRoutes(*candidate, data, {{0, {0, 1, 2, 3}}, {1, {4, 5, 6, 7}}});
        populations.addIndividual(candidate.get(), 0, true);
        require(populations.popula[0].size() == (matches ? 1u : 0u),
                "initial solution entered a resident with a different depot set");
        require(populations.popula[1].size() == (matches ? 0u : 1u),
                "mismatched initial solution was not routed to the existing free population");
    };
    check({data.numV}, false, true);
    check({data.numV + 1}, false, false);
    check({data.numV, data.numV + 1}, false, false);
    check({data.numV}, true, false);
    data.compactLocConfigure = {{data.numV}};
}

void checkCloneTopology(read_data& data) {
    const auto check = [&](bool directed, bool reverseSecond, bool keepUnique) {
        // Build feasible native routes first; only the adjacency equivalence
        // rule changes below. This does not exercise a TW search or evaluator.
        data.hardTimeWindows = false;
        popManager populations(&data);
        auto first = solution(data, 0, ordered);
        std::vector<int> secondOrder = ordered;
        if (reverseSecond) std::reverse(secondOrder.begin(), secondOrder.end());
        auto second = solution(data, 0, secondOrder);
        auto unique = solution(data, 0, expensive);
        const double uniqueObjective = unique->fit;
        SubPopulation& members = populations.popula[0];
        members.push_back(first.release());
        members.push_back(second.release());
        members.push_back(unique.release());
        for (std::size_t i = 0; i < members.size(); ++i) {
            for (std::size_t j = i + 1; j < members.size(); ++j) {
                const double score = members[i]->brokenPairsDistance(members[j]);
                members[i]->indivsPerProximity.insert({score, members[j]});
                members[j]->indivsPerProximity.insert({score, members[i]});
            }
        }
        require(members[2]->averageBrokenPairsDistanceClosest(1) < 0,
                "fixture must expose the retained negative score for a distinct topology");
        data.hardTimeWindows = directed;
        populations.removeWorstBiasedFitness(members);
        data.hardTimeWindows = false;
        const bool retained = std::any_of(members.begin(), members.end(),
            [&](const Individual* member) { return member->fit == uniqueObjective; });
        require(retained == keepUnique,
                "clone removal did not respect actual topology and family direction");
    };
    check(false, true, true);  // A whole-route reversal is the same Set S topology.
    check(true, true, false);  // Opposite directions are distinct TW topologies.
    check(true, false, true);  // Identical directed routes remain clones.
}

void checkReductionMetadata(read_data& data) {
    data.numLocConfiguration = 4;
    data.compactLocConfigure.clear();
    data.openCostLoc[data.numV + 1] = 1000;
    auto best = solution(data, 0, ordered);
    StudyEvents events(&data); // Ordinary mode: no event file or solver is created.
    popManager populations(&data);
    populations.bestIndivRestart = best.get();
    populations.study = &events;
    for (int depot = 0; depot < data.numL; ++depot) {
        data.compactLocConfigure.push_back({data.numV + depot});
        populations.popula[depot].push_back(solution(data, depot, ordered).release());
    }
    for (int slot = 0; slot <= data.numL; ++slot) {
        populations.popCreationIteration[slot] = 10 * (slot + 1);
        populations.replaceStudyPopulation(slot, "fixture");
    }
    populations.reduceHalfSubPop();
    require(data.numLocConfiguration == 3, "reduction changed its established survivor count");
    require(data.compactLocConfigure[1] == std::set<int>{data.numV + 2},
            "fixture did not remove the middle population");
    const int survivingIds[] = {0, 2, 3, 4};
    const int creation[] = {10, 30, 40, 50};
    for (int slot = 0; slot <= data.numLocConfiguration; ++slot) {
        require(populations.studyPopulationAtSlot[slot] == survivingIds[slot],
                "reduction detached a survivor from its study identity");
        require(populations.popCreationIteration[slot] == creation[slot],
                "reduction detached a survivor from its protection timestamp");
    }
    require(populations.studyPopulations[1].exitedAt == data.iteration,
            "removed population was not closed in study metadata");
    data.openCostLoc[data.numV + 1] = 100;
}
} // namespace

struct CrossoverBoundaryChecks {
    static void run(read_data& data) {
        data.numLocConfiguration = 1;
        data.compactLocConfigure = {{data.numV}};
        popManager populations(&data);
        crossover cross(&data, &populations);
        for (auto& cycles : cross.whichCycles) cycles.clear();
        cross.nbABcycles = 4;
        cross.whichCycles[0] = {0, 1};
        cross.whichCycles[1] = {0};
        cross.whichCycles[2] = {1};
        cross.whichCycles[3] = {2};
        cross.whichCycles[4] = {3};
        cross.generationEsets();
        require(cross.nbeSets == 3, "shared-cycle grouping lost the three-way partition");
        require(cross.eSets[0].cycles == std::set<int>({0, 1})
                    && cross.eSets[1].cycles == std::set<int>({2})
                    && cross.eSets[2].cycles == std::set<int>({3}),
                "shared-cycle grouping lost a nonempty group after deduplication");
        cross.generationEsets();
        require(cross.nbeSets == 3 && cross.eSets[2].cycles == std::set<int>({3}),
                "shared-cycle grouping depends on stale previous-call state");
        auto parent = solution(data, 0, ordered);
        int count = 71;
        require(cross.gEAX(parent.get(), parent.get(), nullptr, count) && count == 0,
                "an empty EAX batch retained an earlier offspring count");
        cross.sizeSubPop = 0;
        count = 71;
        require(!cross.gEAX(parent.get(), parent.get(), nullptr, count) && count == 0,
                "a rejected EAX batch retained an earlier offspring count");
    }
};

int main() {
    try {
        Fixture fixture;
        // Startup can fail before screening reduces the configuration count.
        // Cleanup must follow owned storage, not that still-unresolved count.
        const int screenedCount = fixture.data.numLocConfiguration;
        fixture.data.numLocConfiguration = 500;
        {
            popManager pending(&fixture.data);
            pending.popula.front().push_back(solution(fixture.data, 0, ordered).release());
            pending.popula.back().push_back(solution(fixture.data, 1, medium).release());
        }
        fixture.data.numLocConfiguration = screenedCount;
        checkCopiedDepotState(fixture.data);
        checkSPInsertionAndTwoMemberSelection(fixture.data);
        checkInitialConfigurationAssignment(fixture.data);
        checkCloneTopology(fixture.data);
        checkReductionMetadata(fixture.data);
        CrossoverBoundaryChecks::run(fixture.data);
        std::cout << "population and crossover boundary checks passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    } catch (const std::string& error) {
        std::cerr << error << '\n';
        return 1;
    }
}
