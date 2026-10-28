#include "Individual.h"
#include "LS.h"
#include "RepairProcedure.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <vector>

std::mt19937 global_rng(7);

namespace {
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

// Tiny in-memory states exercise route maintenance and repair directly. No
// instance reader, experiment coordinator, MIP environment or solver is run.
struct Fixture {
    read_data data;
    std::unique_ptr<Individual> individual;

    Fixture(const std::vector<int>& demands, const std::vector<int>& capacities,
            int vehicleCapacity, bool hardWindows = false) {
        data.numV = static_cast<int>(demands.size());
        data.numL = static_cast<int>(capacities.size());
        data.numC = data.numV + data.numL;
        data.hardTimeWindows = hardWindows;
        data.define();
        data.vehCap = vehicleCapacity;
        data.usingCostVeh = 10;
        data.totalDemand = std::accumulate(demands.begin(), demands.end(), 0);
        data.normalizaPara = double(data.numV) / data.totalDemand;
        data.isConstraintLoc = true;
        data.timeLimit = 60000;
        data.iterLimit = 300000;
        data.iteration = 0;
        data.loopp = 0;
        data.alpha = 1;
        data.readyTime.assign(data.numV, 0);
        data.dueTime.assign(data.numV, 1000000);
        data.serviceTime.assign(data.numV, 0);
        data.depotCloseTime.assign(data.numL, 1000000);
        for (int i = 0; i < data.numC; ++i) {
            data.invLoc[i] = 0;
            data.openCostLoc[i] = 0;
            for (int j = 0; j < data.numC; ++j) data.D[i][j] = i == j ? 0 : 1;
        }
        std::copy(demands.begin(), demands.end(), data.demCus);
        data.maxVnum = 0;
        for (int depot = 0; depot < data.numL; ++depot) {
            data.invLoc[data.numV + depot] = capacities[depot];
            data.maxVnum += hardWindows ? data.numV : data.safeRouteLimit(depot);
        }
        individual.reset(new Individual());
        individual->define(&data);
        individual->initilization();
    }

    void distances(const std::vector<int>& coordinates) {
        require(coordinates.size() == static_cast<size_t>(data.numC), "distance fixture size");
        for (int i = 0; i < data.numC; ++i)
            for (int j = 0; j < data.numC; ++j)
                data.D[i][j] = std::abs(coordinates[i] - coordinates[j]);
    }

    void install(const std::vector<std::pair<int, std::vector<int>>>& routes) {
        individual->initilization();
        std::vector<int> slots(data.numL, 0);
        for (const auto& entry : routes) {
            NodeDepot& depot = individual->location[entry.first];
            const int slot = slots[entry.first]++;
            require(slot < depot.maxVisit && !entry.second.empty(), "route fixture exceeds storage");
            Node* start = &depot.depots[slot];
            Node* end = &depot.endDepots[slot];
            Node* previous = start;
            for (int customer : entry.second) {
                Node* node = &individual->client[customer];
                previous->next = node;
                node->pre = previous;
                node->route = start->route;
                previous = node;
            }
            previous->next = end;
            end->pre = previous;
        }
        for (int r = 0; r < individual->maxVnum; ++r)
            individual->updateRouteInfor(r, -1);
        for (int l = 0; l < data.numL; ++l) {
            int index = l + data.numV;
            individual->updateLocationInfor(index);
        }
        individual->evaluationDis();
        individual->decideFeasible();
    }

    void requireCoverage() const {
        std::vector<int> visits(data.numV, 0);
        for (int r = 0; r < individual->maxVnum; ++r) {
            const Route& route = individual->route[r];
            int count = 0;
            const Node* previous = route.depot;
            const Node* node = previous->next;
            while (node != route.endDepot) {
                require(node && !node->isDepot && node->cour >= 0 && node->cour < data.numV,
                        "route has an invalid customer or terminator");
                require(node->pre == previous && node->route == &route, "route links/owner differ");
                require(++visits[node->cour] == 1, "customer duplicated after repair");
                require(++count <= data.numV, "cyclic route after repair");
                previous = node;
                node = node->next;
            }
            require(node->pre == previous, "broken depot return");
            require(route.nbClients == count, "customer count includes a depot");
        }
        for (int count : visits) require(count == 1, "customer lost during repair");
    }
};

void customerCounts(bool hardWindows) {
    Fixture fixture({1, 1, 1}, {3}, 3, hardWindows);
    fixture.install({{0, {0}}, {0, {1, 2}}});
    require(fixture.individual->route[0].nbClients == 1, "singleton customer count");
    require(fixture.individual->route[1].nbClients == 2, "two-customer count");
    require(fixture.individual->route[2].nbClients == 0, "empty route customer count");
    fixture.requireCoverage();
    fixture.individual->isRight();
}

void splitCommit(bool prefix, bool blockedFirst = false) {
    Fixture fixture({1, 1, 1}, blockedFirst ? std::vector<int>{3, 0, 3} : std::vector<int>{3, 3}, 3);
    // Empty correlated neighborhoods isolate the existing split operators.
    std::vector<int> coordinates = prefix ? std::vector<int>{100, 100, 0, 0, 100}
                                          : std::vector<int>{0, 100, 100, 0, 100};
    if (blockedFirst) coordinates.push_back(100);
    fixture.distances(coordinates);
    for (auto& locations : fixture.data.customerToLoc) {
        locations = {fixture.data.numV + 1};
        if (blockedFirst) locations.push_back(fixture.data.numV + 2);
    }
    fixture.install({{0, {0, 1, 2}}});
    const double before = fixture.individual->fit;
    LS search(&fixture.data);
    search.local_search_run(fixture.individual.get(), 1, 1);
    const Route* source = fixture.individual->location[0].depots[0].route;
    const Route* destination = fixture.individual->location[blockedFirst ? 2 : 1].depots[0].route;
    require(fixture.individual->fit < before, "split did not improve the fixture");
    require(source->nbClients == 1 && destination->nbClients == 2, "split segment was not committed");
    require(source->whenLastModified > 0 && destination->whenLastModified > 0,
            "split left route cache timestamps unchanged");
    // A continued pass must revisit every customer after the committed split.
    for (int c = 0; c < fixture.data.numV; ++c)
        require(fixture.individual->client[c].whenLastTestedRI >= source->whenLastModified,
                "search ended before revisiting a split-modified route");
    fixture.requireCoverage();
    fixture.individual->isRight();
}

void fullTargetIsSkipped() {
    Fixture fixture({2, 1, 1}, {2, 1, 5}, 2);
    fixture.install({{0, {0}}, {0, {1}}, {1, {2}}});
    require(fixture.individual->location[1].emptyRoutes.empty(), "target must be full");
    for (auto& locations : fixture.data.customerToLoc)
        locations = {fixture.data.numV + 1, fixture.data.numV + 2};
    RepairProcedure repair(&fixture.data, nullptr);
    repair.repairCapacityConstraint(fixture.individual.get(), 1, 1);
    require(fixture.individual->isFeasible, "repair failed to use a later available target");
    fixture.requireCoverage();
    fixture.individual->isRight();
}

void noSlotsInInitialRepair() {
    Fixture fixture({2, 1, 1, 1}, {2, 1, 0}, 2);
    fixture.install({{0, {0, 1}}, {0, {2}}, {1, {3}}});
    for (const auto& route : std::vector<int>{0, 1})
        require(fixture.individual->location[route].emptyRoutes.empty(), "occupied slot fixture");
    require(fixture.individual->location[2].maxVisit == 0, "zero-slot fixture");
    for (auto& neighbors : fixture.data.edgeNode)
        neighbors = {fixture.data.numV + 1, fixture.data.numV + 2};
    RepairProcedure repair(&fixture.data, nullptr);
    repair.repairForInitialSolution(fixture.individual.get());
    fixture.individual->decideFeasible();
    require(!fixture.individual->isFeasible, "unrepairable state was reported feasible");
    fixture.requireCoverage();
}

void singletonShortageTerminates() {
    Fixture fixture({2, 1}, {1, 1}, 2);
    fixture.install({{0, {0}}, {1, {1}}});
    RepairProcedure repair(&fixture.data, nullptr);
    repair.repairCapacityConstraint(fixture.individual.get(), 1, 1);
    require(!fixture.individual->isFeasible, "capacity shortage was reported feasible");
    fixture.requireCoverage();
}

void depotChangesInvalidateUntouchedRoutes() {
    Fixture fixture({1, 1, 1}, {3, 3}, 3);
    fixture.install({{0, {0}}, {0, {1}}, {1, {2}}});
    Individual& individual = *fixture.individual;
    for (int r = 0; r < individual.maxVnum; ++r) individual.route[r].whenLastModified = 0;
    Route* untouched = individual.client[0].route;
    Node* moved = &individual.client[1];
    Route* source = moved->route;
    Route* target = individual.client[2].route;
    moved->pre->next = moved->next;
    moved->next->pre = moved->pre;
    Node* previous = target->endDepot->pre;
    previous->next = moved;
    moved->pre = previous;
    moved->next = target->endDepot;
    target->endDepot->pre = moved;
    moved->route = target;
    individual.updateRouteInfor(source->cour, 7);
    individual.updateRouteInfor(target->cour, 7);
    for (int depot = 0; depot < fixture.data.numL; ++depot) {
        int index = fixture.data.numV + depot;
        individual.updateLocationInfor(index);
    }
    require(untouched->whenLastModified == 7,
            "depot capacity/opening changes left another route's move cache valid");
    individual.evaluationDis();
    individual.decideFeasible();
    fixture.requireCoverage();
    individual.isRight();
}

} // namespace

int main() {
    try {
        customerCounts(false);
        customerCounts(true);
        splitCommit(false);
        splitCommit(true);
        splitCommit(false, true);
        splitCommit(true, true);
        fullTargetIsSkipped();
        noSlotsInInitialRepair();
        singletonShortageTerminates();
        depotChangesInvalidateUntouchedRoutes();
        std::cout << "route state checks passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
