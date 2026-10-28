// Author: Feiran
// Apply Shaw removal and ordered customer reinsertion.

#include "TimeWindowOperators.h"

#include "mutation.h"

mutation::mutation(read_data *data) {
    this->data = data;
    this->numV = data->numV;
    this->numL = data->numL;
    auxArray1 = new int[numV];
    this->perturbLength = data->ratioPerturbLength * numV;
    cityRank = new int[numV];
}
mutation::~mutation() {
    delete[] auxArray1;
    delete[] cityRank;
}
double mutation::compute(bool istrue, double value) {
    if (istrue)
        return 0.;
    else
        return value;
}
double mutation::newRouteInsertionCost(const Node *position, const Node *customer) const {
    // Keep opening, vehicle, and travel terms in this exact addition order.
    return compute(position->route->whichLocation->used,
                   data->openCostLoc[position->route->whichLocation->cour]) +
           compute(!(position->isDepot && position->next->isDepot), data->usingCostVeh) +
           data->D[position->cour][customer->cour] * 2;
}
void mutation::shawRemoval() {

    for (int i = 0; i < numV; i++)
        auxArray1[i] = 0;
    int seed = RandomInt(0, numV - 1);
    auxArray1[seed] = 1;
    listRemove.push_back(seed);
    int iter;
    double y;
    int rank;
    while ((int)listRemove.size() < perturbLength) {
        seed = RandomInt(0, listRemove.size() - 1);
        iter = 0;
        for (int i = 0; i < numV - 1; i++) {
            if (data->nearCity[listRemove[seed]][i] < numV &&
                auxArray1[data->nearCity[listRemove[seed]][i]] == 0) {
                cityRank[iter] = data->nearCity[listRemove[seed]][i];
                iter++;
            }
        }
        y = RandomDouble(0.0, 1.0);
        rank = int(pow(y, data->determine_para) * iter);
        listRemove.push_back(cityRank[rank]);
        auxArray1[cityRank[rank]] = 1;
    }
    if (data->experimentMode) {
        ++data->setSMutationCalls;
        data->setSMutationRemovedCustomers += listRemove.size();
    }

    Node *p = NULL;
    Node *pre = NULL, *next = NULL;
    for (int i = 0; i < (int)listRemove.size(); i++) {
        p = &s->client[listRemove[i]];
        pre = p->pre;
        next = p->next;
        pre->next = next;
        next->pre = pre;
        p->pre = NULL;
        p->next = NULL;
    }
    for (int i = 0; i < numL; i++) {
        for (int j = 0; j < s->location[i].maxVisit; j++)
            s->updateRouteInfor(s->location[i].depots[j].route->cour, -1);
        int ll = i + numV;
        s->updateLocationInfor(ll);
    }

    RandomShuffle(listRemove);
    double delta, min_delta;
    Node *node, *nodeU, *pos;
    while ((int)listRemove.size() != 0) {
        min_delta = Max;
        pos = NULL;
        node = &s->client[listRemove[0]];
        for (int i = 0; i < (int)data->edgeNode[node->cour].size(); i++) {
            if (data->edgeNode[node->cour][i] >= numV)
                continue;
            if (auxArray1[data->edgeNode[node->cour][i]] == 0) {
                nodeU = &s->client[data->edgeNode[node->cour][i]];
                if (nodeU->route->load + node->dem > data->vehCap ||
                    nodeU->route->whichLocation->remainInven + node->dem >
                        nodeU->route->whichLocation->inventory)
                    continue;
                if (nodeU->route->cour == node->route->cour)
                    continue;
                delta = data->D[nodeU->cour][node->cour] + data->D[node->cour][nodeU->next->cour] -
                        data->D[nodeU->cour][nodeU->next->cour];
                if (delta < min_delta) {
                    pos = nodeU;
                    min_delta = delta;
                }
            }
        }
        for (int i = 0; i < (int)data->customerToLoc[node->cour].size(); i++) {
            if (s->location[data->customerToLoc[node->cour][i] - numV].remainInven + node->dem >
                s->location[data->customerToLoc[node->cour][i] - numV].inventory)
                continue;
            if (s->location[data->customerToLoc[node->cour][i] - numV].emptyRoutes.empty())
                continue;
            nodeU = s->route[*s->location[data->customerToLoc[node->cour][i] - numV]
                                  .emptyRoutes.begin()]
                        .depot;
            delta = newRouteInsertionCost(nodeU, node);
            if (delta < min_delta) {
                pos = nodeU;
                min_delta = delta;
            }
        }

        if (min_delta == Max) {
            for (int i = 0; i < numL; i++) {
                if (s->location[i].used) {
                    if (s->location[i].remainInven + node->dem > s->location[i].inventory)
                        continue;
                    nodeU = s->route[*s->location[i].emptyRoutes.begin()].depot;
                    delta = newRouteInsertionCost(nodeU, node);
                    if (delta < min_delta) {
                        pos = nodeU;
                        min_delta = delta;
                    }
                }
            }
            if (min_delta == Max) {
                for (int i = 0; i < numL; i++) {
                    if (!s->location[i].used) {
                        if (s->location[i].remainInven + node->dem > s->location[i].inventory)
                            continue;
                        nodeU = s->route[*s->location[i].emptyRoutes.begin()].depot;
                        delta = newRouteInsertionCost(nodeU, node);
                        if (delta < min_delta) {
                            pos = nodeU;
                            min_delta = delta;
                        }
                    }
                }
            }
        }

        insertNode(node, pos);
        auxArray1[node->cour] = 0;
        listRemove.erase(listRemove.begin());
        s->updateRouteInfor(pos->route->cour, -1);
        s->updateLocationInfor(pos->route->whichLocation->cour);
    }
}
void mutation::insertNode(Node *U, Node *V) {
    V->next->pre = U;
    U->next = V->next;
    V->next = U;
    U->pre = V;
    U->route = V->route;
}
void mutation::mutationRun(Individual *s) {
    if (data->hardTimeWindows) {
        masptw::mutate(s, data);
        return;
    }
    this->s = s;
    std::uniform_int_distribution<int> dist_mut(1, 1);
    // Retain the constant-range draw as part of the shared RNG call sequence.
    (void)dist_mut(global_rng);
    shawRemoval();

    s->evaluationDis();
    s->isRight();
}
