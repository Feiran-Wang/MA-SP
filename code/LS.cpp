// Author: Feiran
// Apply relocation, exchange, and route-splitting neighborhoods.

#include "TimeWindowOperators.h"
#include "LS.h"
#include "basic.h"
#include "Individual.h"
LS::LS(read_data *data) {
    this->data = data;
    this->numV = data->numV;
    this->numC = data->numC;
    for (int i = 0; i < numV; i++)
        orderNodes.push_back(i);
}
LS::~LS() {}
double LS::compute(bool istrue, double value) {
    if (istrue)
        return 0.;
    else
        return value;
}
void LS::srun() {
    RandomShuffle(orderNodes);
    for (int i = 0; i < numV; i++)
        if (RandomInt(0, data->alpha - 1) == 0)
            RandomShuffle(data->edgeNode[i]);

    searchComplete = false;
    for (loopID = 0; !searchComplete; loopID++) {
        searchComplete = true;
        for (int posU = 0; posU < numV; posU++) {
            nodeU = &s->client[orderNodes[posU]];
            int lastTestRINodeU = nodeU->whenLastTestedRI;
            nodeU->whenLastTestedRI = nbMoves;
            for (int posV = 0; posV < (int)data->edgeNode[nodeU->cour].size(); posV++) {
                if (data->edgeNode[nodeU->cour][posV] >= numV)
                    continue;
                nodeV = &s->client[data->edgeNode[nodeU->cour][posV]];
                if (loopID == 0 ||
                    std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) >
                        lastTestRINodeU) {
                    setLocalVariableRouteU();
                    setLocalVariableRouteV();

                    if (Relocate_one())
                        continue;
                    if (Relocate_two())
                        continue;
                    if (Relocate_three())
                        continue;
                    if (nodeUIndex < nodeVIndex && Relocate_four())
                        continue;
                    if (Relocate_five())
                        continue;
                    if (nodeUIndex <= nodeVIndex && Relocate_six())
                        continue;
                    if (nodeUIndex <= nodeVIndex && Relocate_seven())
                        continue;
                    ;
                    if (routeU == routeV && Relocate_eight())
                        continue;
                    if (routeU != routeV && Relocate_nine())
                        continue;
                    if (routeU != routeV && Relocate_ten())
                        continue;

                    if (nodeV->pre->isDepot) {
                        nodeV = nodeV->pre;
                        setLocalVariableRouteV();
                        if (Relocate_one())
                            continue;
                        if (Relocate_two())
                            continue;
                        if (Relocate_three())
                            continue;
                        if (routeU != routeV && Relocate_nine())
                            continue;
                        if (routeU != routeV && Relocate_ten())
                            continue;
                    }
                }
            }
            if (loopID > 0) {
                for (int posV = 0; posV < (int)data->customerToLoc[nodeU->cour].size(); posV++) {
                    if (!s->location[data->customerToLoc[nodeU->cour][posV] - numV].used)
                        continue;
                    if (!s->location[data->customerToLoc[nodeU->cour][posV] - numV]
                             .emptyRoutes.empty()) {
                        nodeV = s->route[*s->location[data->customerToLoc[nodeU->cour][posV] - numV]
                                              .emptyRoutes.begin()]
                                    .depot;
                        setLocalVariableRouteU();
                        setLocalVariableRouteV();
                        if (Relocate_one())
                            continue;
                        if (Relocate_two())
                            continue;
                        if (Relocate_nine())
                            continue;
                        if (Relocate_ten())
                            continue;
                    }
                }
            }

            setLocalVariableRouteU();
            if (Splitb())
                continue;
            if (Splitf())
                continue;
        }
    }
}
bool LS::Splitf() {
    for (int i = 0; i < (int)data->customerToLoc[nodeUIndex].size(); i++) {
        int loc = data->customerToLoc[nodeUIndex][i];
        if (loc != nodeU->route->whichLocation->cour) {
            Node *beginU = nodeU->route->depot->next;

            if (s->location[loc - numV].remainInven + nodeU->cumulatedLoad >
                s->location[loc - numV].inventory)
                continue;
            if (s->location[loc - numV].emptyRoutes.empty())
                continue;
            moveGain =
                compute(s->location[loc - numV].used, data->openCostLoc[loc]) -
                compute(!(nodeU->route->whichLocation->splitTime == 1 && nodeU->next->isDepot),
                        data->openCostLoc[nodeU->route->whichLocation->cour]) -
                compute(!nodeU->next->isDepot, data->usingCostVeh) + data->usingCostVeh +
                data->D[nodeUIndex][loc] + data->D[loc][beginU->cour] +
                data->D[nodeXIndex][beginU->pre->cour] - data->D[nodeXIndex][nodeUIndex] -
                data->D[beginU->cour][beginU->pre->cour];

            if (moveGain > -Min)
                return false;

            Node *xx = nodeU;
            Route *rr = &s->route[*s->location[loc - numV].emptyRoutes.begin()];

            while (!xx->isDepot) {
                xx->route = rr;
                xx = xx->pre;
            }
            Node *pre, *next;
            pre = beginU->pre;
            next = nodeX;
            pre->next = next;
            next->pre = pre;

            beginU->pre = rr->depot;
            rr->depot->next = beginU;
            nodeU->next = rr->endDepot;
            rr->endDepot->pre = nodeU;
            ++nbMoves;
            searchComplete = false;
            s->updateRouteInfor(routeU->cour, nbMoves);
            s->updateRouteInfor(rr->cour, nbMoves);

            s->updateLocationInfor(routeU->whichLocation->cour);
            s->updateLocationInfor(nodeU->route->whichLocation->cour);

            return true;
        }
    }
    return false;
}
bool LS::Splitb() {
    for (int i = 0; i < (int)data->customerToLoc[nodeUIndex].size(); i++) {
        int loc = data->customerToLoc[nodeUIndex][i];
        if (loc != nodeU->route->whichLocation->cour) {
            Node *lastU = nodeU->route->endDepot->pre;

            if (s->location[loc - numV].remainInven +
                    (nodeU->route->load - nodeU->pre->cumulatedLoad) >
                s->location[loc - numV].inventory)
                continue;
            if (s->location[loc - numV].emptyRoutes.empty())
                continue;
            moveGain =
                compute(s->location[loc - numV].used, data->openCostLoc[loc]) -
                compute(!(nodeU->route->whichLocation->splitTime == 1 && nodeU->pre->isDepot),
                        data->openCostLoc[nodeU->route->whichLocation->cour]) -
                compute(!nodeU->pre->isDepot, data->usingCostVeh) + data->usingCostVeh +
                data->D[nodeUIndex][loc] + data->D[loc][lastU->cour] +
                data->D[nodeUPrevIndex][lastU->next->cour] - data->D[nodeUPrevIndex][nodeUIndex] -
                data->D[lastU->cour][lastU->next->cour];

            if (moveGain > -Min)
                return false;
            Node *xx = nodeU;
            Route *rr = &s->route[*s->location[loc - numV].emptyRoutes.begin()];

            while (!xx->isDepot) {
                xx->route = rr;
                xx = xx->next;
            }
            Node *pre, *next;
            pre = nodeU->pre;
            next = lastU->next;
            pre->next = next;
            next->pre = pre;

            nodeU->pre = rr->depot;
            rr->depot->next = nodeU;
            lastU->next = rr->endDepot;
            rr->endDepot->pre = lastU;
            ++nbMoves;
            searchComplete = false;
            s->updateRouteInfor(routeU->cour, nbMoves);
            s->updateRouteInfor(rr->cour, nbMoves);

            s->updateLocationInfor(routeU->whichLocation->cour);
            s->updateLocationInfor(nodeU->route->whichLocation->cour);

            return true;
        }
    }
    return false;
}
bool LS::Relocate_one() {

    if (nodeUIndex == nodeYIndex || nodeUIndex == nodeVIndex)
        return false;

    if (nodeU->route->whichLocation != nodeV->route->whichLocation) {
        if (!(nodeV->route->whichLocation->remainInven + nodeU->dem <=
              nodeV->route->whichLocation->inventory))
            return false;
    }
    if (nodeV->route != nodeU->route && nodeV->route->load + nodeU->dem > data->vehCap)
        return false;

    moveGain =
        compute(nodeV->route->whichLocation->used,
                data->openCostLoc[nodeV->route->whichLocation->cour]) +
        compute(!(nodeV->isDepot && nodeY->isDepot), data->usingCostVeh) -
        compute(!(nodeU->route->whichLocation->splitTime == 1 && nodeU->route->nbClients == 1 &&
                  nodeU->route->whichLocation != nodeV->route->whichLocation),
                data->openCostLoc[nodeU->route->whichLocation->cour]) -
        compute(nodeU->route->nbClients != 1, data->usingCostVeh) +
        data->D[nodeUPrevIndex][nodeXIndex] - data->D[nodeUPrevIndex][nodeUIndex] -
        data->D[nodeUIndex][nodeXIndex] + data->D[nodeVIndex][nodeUIndex] +
        data->D[nodeUIndex][nodeYIndex] - data->D[nodeVIndex][nodeYIndex];

    if (moveGain > -Min)
        return false;
    insertNode(nodeU, nodeV);

    commitRoutePairMove();

    return true;
}

bool LS::Relocate_two() {

    if (nodeUIndex == nodeYIndex || nodeXIndex == nodeVIndex || nodeUIndex == nodeVIndex ||
        nodeX->isDepot)
        return false;

    if (nodeU->route->whichLocation != nodeV->route->whichLocation) {
        if (!(nodeV->route->whichLocation->remainInven + nodeU->dem + nodeX->dem <=
              nodeV->route->whichLocation->inventory))
            return false;
    }
    if (nodeV->route != nodeU->route && nodeV->route->load + nodeU->dem + nodeX->dem > data->vehCap)
        return false;

    moveGain =
        compute(nodeV->route->whichLocation->used,
                data->openCostLoc[nodeV->route->whichLocation->cour]) +
        compute(!(nodeV->isDepot && nodeY->isDepot), data->usingCostVeh) -
        compute(!(nodeU->route->whichLocation->splitTime == 1 && nodeU->route->nbClients == 2 &&
                  nodeU->route->whichLocation != nodeV->route->whichLocation),
                data->openCostLoc[nodeU->route->whichLocation->cour]) -
        compute(nodeU->route->nbClients != 2, data->usingCostVeh) +
        data->D[nodeUPrevIndex][nodeXNextIndex] - data->D[nodeUPrevIndex][nodeUIndex] -
        data->D[nodeXIndex][nodeXNextIndex] + data->D[nodeVIndex][nodeUIndex] +
        data->D[nodeXIndex][nodeYIndex] - data->D[nodeVIndex][nodeYIndex];

    if (moveGain > -Min)
        return false;

    insertNode(nodeU, nodeV);
    insertNode(nodeX, nodeU);

    commitRoutePairMove();

    return true;
}

bool LS::Relocate_three() {

    if (nodeUIndex == nodeYIndex || nodeXIndex == nodeVIndex || nodeUIndex == nodeVIndex ||
        nodeX->isDepot)
        return false;

    if (nodeU->route->whichLocation != nodeV->route->whichLocation) {
        if (!(nodeV->route->whichLocation->remainInven + nodeU->dem + nodeX->dem <=
              nodeV->route->whichLocation->inventory))
            return false;
    }
    if (nodeV->route != nodeU->route && nodeV->route->load + nodeU->dem + nodeX->dem > data->vehCap)
        return false;

    moveGain =
        compute(nodeV->route->whichLocation->used,
                data->openCostLoc[nodeV->route->whichLocation->cour]) +
        compute(!(nodeV->isDepot && nodeY->isDepot), data->usingCostVeh) -
        compute(!(nodeU->route->whichLocation->splitTime == 1 && nodeU->route->nbClients == 2 &&
                  nodeU->route->whichLocation != nodeV->route->whichLocation),
                data->openCostLoc[nodeU->route->whichLocation->cour]) -
        compute(nodeU->route->nbClients != 2, data->usingCostVeh) +
        data->D[nodeUPrevIndex][nodeXNextIndex] - data->D[nodeUPrevIndex][nodeUIndex] -
        data->D[nodeXIndex][nodeXNextIndex] + data->D[nodeVIndex][nodeXIndex] +
        data->D[nodeUIndex][nodeYIndex] - data->D[nodeVIndex][nodeYIndex] +
        data->D[nodeXIndex][nodeUIndex] - data->D[nodeUIndex][nodeXIndex];

    if (moveGain > -Min)
        return false;

    insertNode(nodeX, nodeV);
    insertNode(nodeU, nodeX);

    commitRoutePairMove();

    return true;
}

bool LS::Relocate_four() {
    if (nodeUIndex == nodeVPrevIndex || nodeUIndex == nodeYIndex)
        return false;
    if (nodeU->route->whichLocation != nodeV->route->whichLocation)
        if ((nodeV->route->whichLocation->remainInven + nodeU->dem - nodeV->dem >
             nodeV->route->whichLocation->inventory) ||
            (nodeU->route->whichLocation->remainInven + nodeV->dem - nodeU->dem >
             nodeU->route->whichLocation->inventory))
            return false;
    if (nodeU->route->load - nodeU->dem + nodeV->dem > data->vehCap ||
        nodeV->route->load - nodeV->dem + nodeU->dem > data->vehCap)
        return false;

    moveGain = data->D[nodeUPrevIndex][nodeVIndex] + data->D[nodeVIndex][nodeXIndex] -
               data->D[nodeUPrevIndex][nodeUIndex] - data->D[nodeUIndex][nodeXIndex];
    moveGain += data->D[nodeVPrevIndex][nodeUIndex] + data->D[nodeUIndex][nodeYIndex] -
                data->D[nodeVPrevIndex][nodeVIndex] - data->D[nodeVIndex][nodeYIndex];

    if (moveGain > -Min)
        return false;
    swapNode(nodeU, nodeV);

    commitRoutePairMove();

    return true;
}
bool LS::Relocate_five() {
    if (nodeU == nodeV->pre || nodeX == nodeV->pre || nodeU == nodeY || nodeX->isDepot)
        return false;
    if (nodeU->route->whichLocation != nodeV->route->whichLocation) {
        if (nodeU->route->whichLocation->remainInven + nodeV->dem - nodeU->dem - nodeX->dem >
            nodeU->route->whichLocation->inventory)
            return false;
        if (nodeV->route->whichLocation->remainInven + nodeU->dem + nodeX->dem - nodeV->dem >
            nodeV->route->whichLocation->inventory)
            return false;
    }
    if (nodeU->route->load - nodeU->dem - nodeX->dem + nodeV->dem > data->vehCap ||
        nodeV->route->load + nodeU->dem + nodeX->dem - nodeV->dem > data->vehCap)
        return false;

    moveGain = data->D[nodeUPrevIndex][nodeVIndex] + data->D[nodeVIndex][nodeXNextIndex] -
               data->D[nodeUPrevIndex][nodeUIndex] - data->D[nodeXIndex][nodeXNextIndex];
    moveGain += data->D[nodeVPrevIndex][nodeUIndex] + data->D[nodeXIndex][nodeYIndex] -
                data->D[nodeVPrevIndex][nodeVIndex] - data->D[nodeVIndex][nodeYIndex];

    if (moveGain > -Min)
        return false;

    swapNode(nodeU, nodeV);
    insertNode(nodeX, nodeU);

    commitRoutePairMove();

    return true;
}
bool LS::Relocate_six() {
    if (nodeX->isDepot || nodeY->isDepot || nodeY == nodeU->pre || nodeU == nodeY ||
        nodeX == nodeV || nodeV == nodeX->next)
        return false;
    if (nodeU->route->whichLocation != nodeV->route->whichLocation) {
        if (nodeU->route->whichLocation->remainInven + nodeV->dem + nodeY->dem - nodeU->dem -
                nodeX->dem >
            nodeU->route->whichLocation->inventory)
            return false;
        if (nodeV->route->whichLocation->remainInven + nodeU->dem + nodeX->dem - nodeV->dem -
                nodeY->dem >
            nodeV->route->whichLocation->inventory)
            return false;
    }
    if (nodeU->route->load - nodeU->dem - nodeX->dem + nodeV->dem + nodeY->dem > data->vehCap ||
        nodeV->route->load - nodeV->dem - nodeY->dem + nodeU->dem + nodeX->dem > data->vehCap)
        return false;

    moveGain = data->D[nodeUPrevIndex][nodeVIndex] + data->D[nodeYIndex][nodeXNextIndex] -
               data->D[nodeUPrevIndex][nodeUIndex] - data->D[nodeXIndex][nodeXNextIndex];
    moveGain += data->D[nodeVPrevIndex][nodeUIndex] + data->D[nodeXIndex][nodeYNextIndex] -
                data->D[nodeVPrevIndex][nodeVIndex] - data->D[nodeYIndex][nodeYNextIndex];

    if (moveGain > -Min)
        return false;
    swapNode(nodeU, nodeV);
    swapNode(nodeX, nodeY);

    commitRoutePairMove();

    return true;
}

bool LS::Relocate_seven() {

    if (nodeX->isDepot || nodeY->isDepot || nodeY == nodeU->pre || nodeU == nodeY ||
        nodeX == nodeV || nodeV == nodeX->next)
        return false;

    if (nodeU->route->whichLocation != nodeV->route->whichLocation) {

        if (nodeU->route->whichLocation->remainInven + nodeV->dem + nodeY->dem - nodeU->dem -
                nodeX->dem >
            nodeU->route->whichLocation->inventory)
            return false;
        if (nodeV->route->whichLocation->remainInven + nodeU->dem + nodeX->dem - nodeV->dem -
                nodeY->dem >
            nodeV->route->whichLocation->inventory)
            return false;
    }

    if (nodeU->route->load - nodeU->dem - nodeX->dem + nodeV->dem + nodeY->dem > data->vehCap ||
        nodeV->route->load - nodeV->dem - nodeY->dem + nodeU->dem + nodeX->dem > data->vehCap)
        return false;

    moveGain = data->D[nodeUPrevIndex][nodeYIndex] + data->D[nodeVIndex][nodeXNextIndex] -
               data->D[nodeUPrevIndex][nodeUIndex] - data->D[nodeXIndex][nodeXNextIndex];
    moveGain += data->D[nodeVPrevIndex][nodeXIndex] + data->D[nodeUIndex][nodeYNextIndex] -
                data->D[nodeVPrevIndex][nodeVIndex] - data->D[nodeYIndex][nodeYNextIndex];

    moveGain += data->D[nodeYIndex][nodeVIndex] + data->D[nodeXIndex][nodeUIndex];
    moveGain -= data->D[nodeUIndex][nodeXIndex] + data->D[nodeVIndex][nodeYIndex];

    if (moveGain > -Min)
        return false;

    swapNode(nodeX, nodeV);
    swapNode(nodeU, nodeY);

    nbMoves++;
    searchComplete = false;
    s->updateRouteInfor(routeU->cour, nbMoves);
    if (routeU != routeV)
        s->updateRouteInfor(routeV->cour, nbMoves);
    s->updateLocationInfor(routeU->whichLocation->cour);
    if (routeU->whichLocation != routeV->whichLocation)
        s->updateLocationInfor(routeV->whichLocation->cour);

    return true;
}

bool LS::Relocate_eight() {

    if (nodeU->route != nodeV->route)
        return false;
    if (nodeX->isDepot || nodeV->isDepot || nodeY->isDepot || nodeU->isDepot)
        return false;
    if (nodeU == nodeV || nodeX == nodeV || nodeU == nodeY || nodeX == nodeY)
        return false;

    bool isUbeforeV = false;
    Node *checkNode = nodeX;
    while (!checkNode->isDepot) {
        if (checkNode == nodeV) {
            isUbeforeV = true;
            break;
        }
        checkNode = checkNode->next;
    }
    if (!isUbeforeV)
        return false;

    moveGain = data->D[nodeUIndex][nodeVIndex] + data->D[nodeXIndex][nodeYIndex] -
               data->D[nodeUIndex][nodeXIndex] - data->D[nodeVIndex][nodeYIndex];

    Node *curr = nodeX;
    while (curr != nodeV) {
        Node *nxt = curr->next;

        moveGain += data->D[nxt->cour][curr->cour] - data->D[curr->cour][nxt->cour];
        curr = nxt;
    }

    if (moveGain > -Min)
        return false;

    curr = nodeX;
    Node *stopNode = nodeY;
    while (curr != stopNode) {
        Node *nxt = curr->next;

        Node *temp = curr->next;
        curr->next = curr->pre;
        curr->pre = temp;

        curr = nxt;
    }

    nodeU->next = nodeV;
    nodeV->pre = nodeU;

    nodeX->next = nodeY;
    nodeY->pre = nodeX;

    nbMoves++;
    searchComplete = false;

    s->updateRouteInfor(nodeU->route->cour, nbMoves);

    return true;
}

bool LS::Relocate_nine() {
    if (nodeU->route == nodeV->route)
        return false;
    if (nodeX->isDepot && nodeY->isDepot)
        return false;

    int safety_limit = data->numC + 2;
    Node *curr = nodeX;
    Node *lastA = nullptr;
    int countA = 0;
    bool sameRoute = false;
    while (!curr->isDepot) {
        if (curr == nodeV || curr == nodeY)
            sameRoute = true;
        lastA = curr;
        countA++;
        if (countA > safety_limit)
            return false;
        curr = curr->next;
    }
    if (sameRoute)
        return false;
    Node *endDepotU = curr;

    curr = nodeY;
    Node *lastB = nullptr;
    int countB = 0;
    while (!curr->isDepot) {
        if (curr == nodeU || curr == nodeX)
            sameRoute = true;
        lastB = curr;
        countB++;
        if (countB > safety_limit)
            return false;
        curr = curr->next;
    }
    if (sameRoute)
        return false;
    Node *endDepotV = curr;

    double weightTailA = nodeU->route->load - nodeU->cumulatedLoad;
    double weightTailB = nodeV->route->load - nodeV->cumulatedLoad;

    if (nodeU->cumulatedLoad + weightTailB > data->vehCap)
        return false;
    if (nodeV->cumulatedLoad + weightTailA > data->vehCap)
        return false;

    if (nodeU->route->whichLocation != nodeV->route->whichLocation) {
        if (nodeU->route->whichLocation->remainInven - weightTailA + weightTailB >
            nodeU->route->whichLocation->inventory)
            return false;
        if (nodeV->route->whichLocation->remainInven - weightTailB + weightTailA >
            nodeV->route->whichLocation->inventory)
            return false;
    }

    moveGain = -data->D[nodeUIndex][nodeXIndex] - data->D[nodeVIndex][nodeYIndex];

    if (countB > 0) {
        moveGain += data->D[nodeUIndex][lastB->cour];
        moveGain += data->D[nodeYIndex][endDepotU->cour];
        moveGain -= data->D[lastB->cour][endDepotV->cour];
        Node *c = nodeY;
        while (c != endDepotV) {
            Node *nxt = c->next;
            if (nxt != endDepotV)
                moveGain += data->D[nxt->cour][c->cour] - data->D[c->cour][nxt->cour];
            c = nxt;
        }
    } else
        moveGain += data->D[nodeUIndex][endDepotU->cour];

    if (countA > 0) {
        moveGain += data->D[nodeVIndex][lastA->cour];
        moveGain += data->D[nodeXIndex][endDepotV->cour];
        moveGain -= data->D[lastA->cour][endDepotU->cour];
        Node *c = nodeX;
        while (c != endDepotU) {
            Node *nxt = c->next;
            if (nxt != endDepotU)
                moveGain += data->D[nxt->cour][c->cour] - data->D[c->cour][nxt->cour];
            c = nxt;
        }
    } else
        moveGain += data->D[nodeVIndex][endDepotV->cour];

    int old_nbU = nodeU->route->nbClients, old_nbV = nodeV->route->nbClients;
    int new_nbU = old_nbU - countA + countB, new_nbV = old_nbV - countB + countA;

    if (old_nbU > 0 && new_nbU == 0) {
        moveGain -= data->usingCostVeh;
        if (nodeU->route->whichLocation->splitTime == 1 &&
            nodeU->route->whichLocation != nodeV->route->whichLocation)
            moveGain -= data->openCostLoc[nodeU->route->whichLocation->cour];
    } else if (old_nbU == 0 && new_nbU > 0) {
        moveGain += data->usingCostVeh;
        if (!nodeU->route->whichLocation->used &&
            nodeU->route->whichLocation != nodeV->route->whichLocation)
            moveGain += data->openCostLoc[nodeU->route->whichLocation->cour];
    }

    if (old_nbV > 0 && new_nbV == 0) {
        moveGain -= data->usingCostVeh;
        if (nodeV->route->whichLocation->splitTime == 1 &&
            nodeU->route->whichLocation != nodeV->route->whichLocation)
            moveGain -= data->openCostLoc[nodeV->route->whichLocation->cour];
    } else if (old_nbV == 0 && new_nbV > 0) {
        moveGain += data->usingCostVeh;
        if (!nodeV->route->whichLocation->used &&
            nodeU->route->whichLocation != nodeV->route->whichLocation)
            moveGain += data->openCostLoc[nodeV->route->whichLocation->cour];
    }

    if (std::abs(moveGain) < 1e-5)
        moveGain = 0.0;
    if (moveGain > -Min)
        return false;

    curr = nodeX;
    while (!curr->isDepot) {
        curr->route = nodeV->route;
        curr = curr->next;
    }
    curr = nodeY;
    while (!curr->isDepot) {
        curr->route = nodeU->route;
        curr = curr->next;
    }

    if (countA > 0) {
        curr = nodeX;
        Node *stopNode = endDepotU;
        while (curr != stopNode) {
            Node *nxt = curr->next;
            Node *temp = curr->next;
            curr->next = curr->pre;
            curr->pre = temp;
            curr = nxt;
        }
        nodeV->next = lastA;
        lastA->pre = nodeV;
        nodeX->next = endDepotV;
        endDepotV->pre = nodeX;
    } else {
        nodeV->next = endDepotV;
        endDepotV->pre = nodeV;
    }

    if (countB > 0) {
        curr = nodeY;
        Node *stopNode = endDepotV;
        while (curr != stopNode) {
            Node *nxt = curr->next;
            Node *temp = curr->next;
            curr->next = curr->pre;
            curr->pre = temp;
            curr = nxt;
        }
        nodeU->next = lastB;
        lastB->pre = nodeU;
        nodeY->next = endDepotU;
        endDepotU->pre = nodeY;
    } else {
        nodeU->next = endDepotU;
        endDepotU->pre = nodeU;
    }

    nodeU->route->nbClients = new_nbU;
    nodeV->route->nbClients = new_nbV;
    nbMoves++;
    searchComplete = false;

    s->updateRouteInfor(nodeU->route->cour, nbMoves);
    s->updateRouteInfor(nodeV->route->cour, nbMoves);
    s->updateLocationInfor(nodeU->route->whichLocation->cour);
    if (nodeU->route->whichLocation != nodeV->route->whichLocation)
        s->updateLocationInfor(nodeV->route->whichLocation->cour);

    return true;
}

bool LS::Relocate_ten() {
    if (nodeU->route == nodeV->route)
        return false;
    if (nodeX->isDepot && nodeY->isDepot)
        return false;

    int safety_limit = data->numC + 2;
    Node *curr = nodeX;
    Node *lastA = nullptr;
    int countA = 0;
    bool sameRoute = false;
    while (!curr->isDepot) {
        if (curr == nodeV || curr == nodeY)
            sameRoute = true;
        lastA = curr;
        countA++;
        if (countA > safety_limit)
            return false;
        curr = curr->next;
    }
    if (sameRoute)
        return false;
    Node *endDepotU = curr;

    curr = nodeY;
    Node *lastB = nullptr;
    int countB = 0;
    while (!curr->isDepot) {
        if (curr == nodeU || curr == nodeX)
            sameRoute = true;
        lastB = curr;
        countB++;
        if (countB > safety_limit)
            return false;
        curr = curr->next;
    }
    if (sameRoute)
        return false;
    Node *endDepotV = curr;

    double weightTailA = nodeU->route->load - nodeU->cumulatedLoad;
    double weightTailB = nodeV->route->load - nodeV->cumulatedLoad;

    if (nodeU->cumulatedLoad + weightTailB > data->vehCap)
        return false;
    if (nodeV->cumulatedLoad + weightTailA > data->vehCap)
        return false;

    if (nodeU->route->whichLocation != nodeV->route->whichLocation) {
        if (nodeU->route->whichLocation->remainInven - weightTailA + weightTailB >
            nodeU->route->whichLocation->inventory)
            return false;
        if (nodeV->route->whichLocation->remainInven - weightTailB + weightTailA >
            nodeV->route->whichLocation->inventory)
            return false;
    }

    moveGain = -data->D[nodeUIndex][nodeXIndex] - data->D[nodeVIndex][nodeYIndex];

    if (countB > 0) {
        moveGain += data->D[nodeUIndex][nodeYIndex];
        moveGain -= data->D[lastB->cour][endDepotV->cour];
        moveGain += data->D[lastB->cour][endDepotU->cour];
    } else
        moveGain += data->D[nodeUIndex][endDepotU->cour];

    if (countA > 0) {
        moveGain += data->D[nodeVIndex][nodeXIndex];
        moveGain -= data->D[lastA->cour][endDepotU->cour];
        moveGain += data->D[lastA->cour][endDepotV->cour];
    } else
        moveGain += data->D[nodeVIndex][endDepotV->cour];

    int old_nbU = nodeU->route->nbClients, old_nbV = nodeV->route->nbClients;
    int new_nbU = old_nbU - countA + countB, new_nbV = old_nbV - countB + countA;

    if (old_nbU > 0 && new_nbU == 0) {
        moveGain -= data->usingCostVeh;
        if (nodeU->route->whichLocation->splitTime == 1 &&
            nodeU->route->whichLocation != nodeV->route->whichLocation)
            moveGain -= data->openCostLoc[nodeU->route->whichLocation->cour];
    } else if (old_nbU == 0 && new_nbU > 0) {
        moveGain += data->usingCostVeh;
        if (!nodeU->route->whichLocation->used &&
            nodeU->route->whichLocation != nodeV->route->whichLocation)
            moveGain += data->openCostLoc[nodeU->route->whichLocation->cour];
    }

    if (old_nbV > 0 && new_nbV == 0) {
        moveGain -= data->usingCostVeh;
        if (nodeV->route->whichLocation->splitTime == 1 &&
            nodeU->route->whichLocation != nodeV->route->whichLocation)
            moveGain -= data->openCostLoc[nodeV->route->whichLocation->cour];
    } else if (old_nbV == 0 && new_nbV > 0) {
        moveGain += data->usingCostVeh;
        if (!nodeV->route->whichLocation->used &&
            nodeU->route->whichLocation != nodeV->route->whichLocation)
            moveGain += data->openCostLoc[nodeV->route->whichLocation->cour];
    }

    if (std::abs(moveGain) < 1e-5)
        moveGain = 0.0;
    if (moveGain > -Min)
        return false;

    curr = nodeX;
    while (!curr->isDepot) {
        curr->route = nodeV->route;
        curr = curr->next;
    }
    curr = nodeY;
    while (!curr->isDepot) {
        curr->route = nodeU->route;
        curr = curr->next;
    }

    if (countB > 0) {
        nodeU->next = nodeY;
        nodeY->pre = nodeU;
        lastB->next = endDepotU;
        endDepotU->pre = lastB;
    } else {
        nodeU->next = endDepotU;
        endDepotU->pre = nodeU;
    }

    if (countA > 0) {
        nodeV->next = nodeX;
        nodeX->pre = nodeV;
        lastA->next = endDepotV;
        endDepotV->pre = lastA;
    } else {
        nodeV->next = endDepotV;
        endDepotV->pre = nodeV;
    }

    nodeU->route->nbClients = new_nbU;
    nodeV->route->nbClients = new_nbV;
    nbMoves++;
    searchComplete = false;

    s->updateRouteInfor(nodeU->route->cour, nbMoves);
    s->updateRouteInfor(nodeV->route->cour, nbMoves);
    s->updateLocationInfor(nodeU->route->whichLocation->cour);
    if (nodeU->route->whichLocation != nodeV->route->whichLocation)
        s->updateLocationInfor(nodeV->route->whichLocation->cour);

    return true;
}

void LS::commitRoutePairMove() {
    // Refresh routes before depots so depot-dependent move caches see the new epoch.
    nbMoves++;
    searchComplete = false;
    s->updateRouteInfor(routeU->cour, nbMoves);
    if (routeU != routeV)
        s->updateRouteInfor(routeV->cour, nbMoves);
    s->updateLocationInfor(routeU->whichLocation->cour);
    if (routeU->whichLocation != routeV->whichLocation)
        s->updateLocationInfor(routeV->whichLocation->cour);
}
void LS::setLocalVariableRouteU() {
    routeU = nodeU->route;
    nodeX = nodeU->next;
    nodeXNextIndex = nodeX->next->cour;
    nodeUIndex = nodeU->cour;
    nodeUPrevIndex = nodeU->pre->cour;
    nodeXIndex = nodeX->cour;
}
void LS::setLocalVariableRouteV() {
    routeV = nodeV->route;
    nodeY = nodeV->next;
    nodeYNextIndex = nodeY->next->cour;
    nodeVIndex = nodeV->cour;
    nodeVPrevIndex = nodeV->pre->cour;
    nodeYIndex = nodeY->cour;
}
void LS::insertNode(Node *U, Node *V) {
    U->pre->next = U->next;
    U->next->pre = U->pre;
    V->next->pre = U;
    U->pre = V;
    U->next = V->next;
    V->next = U;
    U->route = V->route;
}
void LS::swapNode(Node *U, Node *V) {
    Node *myVPred = V->pre;
    Node *myVSuiv = V->next;
    Node *myUPred = U->pre;
    Node *myUSuiv = U->next;
    Route *myRouteU = U->route;
    Route *myRouteV = V->route;

    myUPred->next = V;
    myUSuiv->pre = V;
    myVPred->next = U;
    myVSuiv->pre = U;

    U->pre = myVPred;
    U->next = myVSuiv;
    V->pre = myUPred;
    V->next = myUSuiv;

    U->route = myRouteV;
    V->route = myRouteU;
}
void LS::loadSolution() {
    nbMoves = 0;
    for (int i = 0; i < numV; i++)
        s->client[i].whenLastTestedRI = -1;
    for (int i = numV; i < numC; i++) {
        for (int j = 0; j < s->location[i - numV].maxVisit; j++) {
            s->location[i - numV].depots[j].route->whenLastModified = -1;
        }
    }
}
void LS::local_search_run(Individual *s, double, double) {
    if (data->hardTimeWindows) {
        if (s->isFeasible)
            masptw::localSearch(s, data);
        return;
    }
    this->s = s;

    loadSolution();
    srun();
    s->evaluationDis();
    s->isRight();

    long long totalInv = 0;
    std::set<std::pair<int, int>> order;

    for (int i = 0; i < data->numL; i++) {
        if (s->location[i].used) {
            order.insert({s->location[i].inventory, i});
            totalInv += s->location[i].inventory;
        }
    }
    if (totalInv > data->totalDemand) {
        auto it = order.begin();
        if (totalInv - it->first > data->totalDemand)
            data->loopp++;
    }
}
