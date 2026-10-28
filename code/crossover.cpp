// Author: Feiran
// Build AB cycles and offspring with the mdEAX crossover.

#include "crossover.h"

crossover::crossover(read_data *data, popManager *popman) {
    this->data = data;
    this->popman = popman;
    this->numV = data->numV;
    this->numL = data->numL;
    this->numC = data->numC;
    sizeSubPop = numC / 2;

    AcapB = new Node1[numC];
    for (int i = 0; i < numV; i++) {
        AcapB[i].edgeF = new Node0[2];
        AcapB[i].edgeM = new Node0[2];
    }
    for (int i = numV; i < numC; i++) {
        AcapB[i].edgeF = new Node0[data->numC * 2];
        AcapB[i].edgeM = new Node0[data->numC * 2];
    }
    ABcycles = new Node2[numC / 2];
    for (int i = 0; i < (int)numC / 2; i++)
        ABcycles[i].sque = new Node0[numC * 4];
    auxArray1 = new int[numC];
    auxArray2 = new int[numC];

    whichCycles = std::vector<std::set<int>>(numC);
    commconCycles = std::vector<std::set<int>>(numC);

    eSets = new Node5[numC / 2];

    gStart = new Node[numC];
    gEnd = new Node[numC];
    for (int i = 0; i < numV; i++) {
        gStart[i].dem = 0;
        gStart[i].cour = -1;
        gStart[i].isDepot = false;
        gEnd[i].dem = 0;
        gEnd[i].cour = -1;
        gEnd[i].isDepot = false;
    }
    for (int i = numV; i < numC; i++) {
        gStart[i].dem = 0;
        gStart[i].cour = i;
        gStart[i].isDepot = true;
        gStart[i].pre = NULL;
        gStart[i].next = NULL;
        gEnd[i].dem = 0;
        gEnd[i].cour = i;
        gEnd[i].isDepot = true;
        gEnd[i].pre = NULL;
        gEnd[i].next = NULL;
    }
    auxA1 = new bool[numV];
    auxA2 = new bool[numV];
}
crossover::~crossover() {
    for (int i = 0; i < numC; i++) {
        delete[] AcapB[i].edgeF;
        delete[] AcapB[i].edgeM;
    }
    delete[] AcapB;
    for (int i = 0; i < (int)numC / 2; i++)
        delete[] ABcycles[i].sque;
    delete[] ABcycles;
    delete[] auxArray1;
    delete[] auxArray2;
    delete[] eSets;

    delete[] gStart;
    delete[] gEnd;
    delete[] auxA1;
    delete[] auxA2;
}

void crossover::appendParentEdge(int vertex, int neighbor, bool father) {
    Node1 &edges = AcapB[vertex];
    Node0 *selected = father ? edges.edgeF : edges.edgeM;
    int &count = father ? edges.nbeF : edges.nbeM;
    selected[count].cour = neighbor;
    ++count;
}

void crossover::collectParentEdges(int depot, const Individual *parent, const Individual *other,
                                   bool father) {
    if (!parent->location[depot].used)
        return;
    for (int slot = 0; slot < parent->location[depot].maxVisit; ++slot) {
        if (!parent->location[depot].depots[slot].route->used)
            continue;
        Node *node = &parent->location[depot].depots[slot];
        if (other->client[node->next->cour].pre->cour != node->cour &&
            other->client[node->next->cour].next->cour != node->cour)
            appendParentEdge(node->cour, node->next->cour, father);
        node = node->next;
        while (!node->isDepot) {
            if (other->client[node->cour].pre->cour != node->next->cour &&
                other->client[node->cour].next->cour != node->next->cour)
                appendParentEdge(node->cour, node->next->cour, father);
            if (other->client[node->cour].pre->cour != node->pre->cour &&
                other->client[node->cour].next->cour != node->pre->cour)
                appendParentEdge(node->cour, node->pre->cour, father);
            node = node->next;
        }
        if (other->client[node->pre->cour].pre->cour != node->cour &&
            other->client[node->pre->cour].next->cour != node->cour)
            appendParentEdge(node->cour, node->pre->cour, father);
    }
}

void crossover::loadFaCapMa() {
    for (int i = 0; i < numC; i++) {
        AcapB[i].nbeF = 0;
        AcapB[i].nbeM = 0;
    }
    // Preserve father-then-mother collection at each depot before singleton remedies.
    for (int depot = 0; depot < numL; ++depot) {
        collectParentEdges(depot, fa, ma, true);
        collectParentEdges(depot, ma, fa, false);
    }
    Node *p = NULL;

    for (int i = 0; i < numV; i++) {
        if (fa->client[i].pre->cour == fa->client[i].next->cour) {
            if ((ma->client[i].pre->cour == fa->client[i].pre->cour &&
                 ma->client[i].next->cour != fa->client[i].pre->cour) ||
                (ma->client[i].next->cour == fa->client[i].pre->cour &&
                 ma->client[i].pre->cour != fa->client[i].pre->cour)) {
                p = &fa->client[i];
                AcapB[p->cour].edgeF[AcapB[p->cour].nbeF].cour = p->pre->cour;
                AcapB[p->cour].nbeF++;
                AcapB[p->pre->cour].edgeF[AcapB[p->pre->cour].nbeF].cour = p->cour;
                AcapB[p->pre->cour].nbeF++;
            }
        }
        if (ma->client[i].pre->cour == ma->client[i].next->cour) {
            if ((fa->client[i].pre->cour == ma->client[i].pre->cour &&
                 fa->client[i].next->cour != ma->client[i].pre->cour) ||
                (fa->client[i].next->cour == ma->client[i].pre->cour &&
                 fa->client[i].pre->cour != ma->client[i].pre->cour)) {
                p = &ma->client[i];
                AcapB[p->cour].edgeM[AcapB[p->cour].nbeM].cour = p->pre->cour;
                AcapB[p->cour].nbeM++;
                AcapB[p->pre->cour].edgeM[AcapB[p->pre->cour].nbeM].cour = p->cour;
                AcapB[p->pre->cour].nbeM++;
            }
        }
    }

    int nbDummy;
    for (int i = numV; i < numC; i++) {
        if (AcapB[i].nbeF == AcapB[i].nbeM)
            continue;
        if (AcapB[i].nbeF > AcapB[i].nbeM) {
            nbDummy = (AcapB[i].nbeF - AcapB[i].nbeM) / 2;
            for (int j = 0; j < nbDummy; j++) {
                AcapB[i].edgeM[AcapB[i].nbeM].cour = -1;
                AcapB[i].nbeM++;
            }
        } else {
            nbDummy = (AcapB[i].nbeM - AcapB[i].nbeF) / 2;
            for (int j = 0; j < nbDummy; j++) {
                AcapB[i].edgeF[AcapB[i].nbeF].cour = -1;
                AcapB[i].nbeF++;
            }
        }
    }
}
void crossover::addNodeToABcycles(int &nb, Node0 &tem) {
    ABcycles[nb].sque[ABcycles[nb].nbCity].cour = tem.cour;
    ABcycles[nb].sque[ABcycles[nb].nbCity].isFather = tem.isFather;
    ABcycles[nb].nbCity++;
}
void crossover::emptyACity(int &city, int &numAvaCity) {
    auxArray2[auxArray1[city]] = auxArray2[numAvaCity - 1];
    auxArray1[auxArray2[numAvaCity - 1]] = auxArray1[city];
    numAvaCity--;
}
void crossover::extractABcycles() {
    for (int i = 0; i < sizeSubPop; i++)
        ABcycles[i].nbCity = 0;
    int numAvaCity = 0;
    for (int i = 0; i < numC; i++) {
        whichCycles[i].clear();
        if (!(AcapB[i].nbeF == 0 && AcapB[i].nbeM == 0)) {
            auxArray2[numAvaCity] = i;
            auxArray1[i] = numAvaCity;
            numAvaCity++;
        }
    }

    Node0 startCity, ori, rCity;
    bool father = false;
    int pos, count;
    nbABcycles = 0;
    int isF, isM;
    while (numAvaCity) {
        ABcycles[nbABcycles].nbCity = 0;
        ori.cour = RandomInt(0, numAvaCity - 1);
        ori.cour = auxArray2[ori.cour];
        isF = 0;
        isM = 0;
        for (int i = 0; i < AcapB[ori.cour].nbeF; i++)
            if (AcapB[ori.cour].edgeF[i].cour > -1) {
                isF = 1;
                break;
            }
        for (int i = 0; i < AcapB[ori.cour].nbeM; i++)
            if (AcapB[ori.cour].edgeM[i].cour > -1) {
                isM = 1;
                break;
            }
        if (isF == 1 && isM == 0)
            father = true;
        if (isF == 0 && isM == 1)
            father = false;
        if (isF == 1 && isM == 1) {
            // Retain this RNG draw even though both outcomes select father edges.
            if (RandomInt(0, 1) == 0)
                father = true;
            else
                father = true;
        }

        ori.isFather = father;
        count = 1;
        startCity.cour = ori.cour;
        addNodeToABcycles(nbABcycles, ori);
        rCity.cour = -2;
        while (1) {
            if (father) {
                pos = RandomInt(0, AcapB[ori.cour].nbeF - 1);
                while (ABcycles[nbABcycles].nbCity == 1 && AcapB[ori.cour].edgeF[pos].cour == -1)
                    pos = RandomInt(0, AcapB[ori.cour].nbeF - 1);

                rCity.cour = AcapB[ori.cour].edgeF[pos].cour;
                father = false;
                rCity.isFather = father;
            } else {
                pos = RandomInt(0, AcapB[ori.cour].nbeM - 1);
                while (ABcycles[nbABcycles].nbCity == 1 && AcapB[ori.cour].edgeM[pos].cour == -1)
                    pos = RandomInt(0, AcapB[ori.cour].nbeM - 1);

                rCity.cour = AcapB[ori.cour].edgeM[pos].cour;
                father = true;
                rCity.isFather = father;
            }
            if (rCity.cour > -1) {
                addNodeToABcycles(nbABcycles, rCity);

                if (!father) {
                    for (int i = pos; i < AcapB[ori.cour].nbeF - 1; i++)
                        AcapB[ori.cour].edgeF[i].cour = AcapB[ori.cour].edgeF[i + 1].cour;
                    AcapB[ori.cour].nbeF--;

                    for (int i = 0; i < AcapB[rCity.cour].nbeF; i++)
                        if (AcapB[rCity.cour].edgeF[i].cour == ori.cour) {
                            pos = i;
                            break;
                        }

                    for (int i = pos; i < AcapB[rCity.cour].nbeF - 1; i++)
                        AcapB[rCity.cour].edgeF[i].cour = AcapB[rCity.cour].edgeF[i + 1].cour;
                    AcapB[rCity.cour].nbeF--;
                } else {
                    for (int i = pos; i < AcapB[ori.cour].nbeM - 1; i++)
                        AcapB[ori.cour].edgeM[i].cour = AcapB[ori.cour].edgeM[i + 1].cour;
                    AcapB[ori.cour].nbeM--;
                    for (int i = 0; i < AcapB[rCity.cour].nbeM; i++)
                        if (AcapB[rCity.cour].edgeM[i].cour == ori.cour) {
                            pos = i;
                            break;
                        }

                    for (int i = pos; i < AcapB[rCity.cour].nbeM - 1; i++)
                        AcapB[rCity.cour].edgeM[i].cour = AcapB[rCity.cour].edgeM[i + 1].cour;
                    AcapB[rCity.cour].nbeM--;
                }
                if (AcapB[ori.cour].nbeF == 0 && AcapB[ori.cour].nbeM == 0)
                    emptyACity(ori.cour, numAvaCity);
                if (AcapB[rCity.cour].nbeF == 0 && AcapB[rCity.cour].nbeM == 0)
                    emptyACity(rCity.cour, numAvaCity);
                ori.cour = rCity.cour;
                ori.isFather = rCity.isFather;
            } else {
                if (!father) {
                    for (int i = pos; i < AcapB[ori.cour].nbeF - 1; i++)
                        AcapB[ori.cour].edgeF[i].cour = AcapB[ori.cour].edgeF[i + 1].cour;
                    AcapB[ori.cour].nbeF--;
                } else {
                    for (int i = pos; i < AcapB[ori.cour].nbeM - 1; i++)
                        AcapB[ori.cour].edgeM[i].cour = AcapB[ori.cour].edgeM[i + 1].cour;
                    AcapB[ori.cour].nbeM--;
                }

                if (ABcycles[nbABcycles].sque[ABcycles[nbABcycles].nbCity - 1].isFather)
                    ABcycles[nbABcycles].sque[ABcycles[nbABcycles].nbCity - 1].isFather = false;
                else
                    ABcycles[nbABcycles].sque[ABcycles[nbABcycles].nbCity - 1].isFather = true;

                if (AcapB[ori.cour].nbeF == 0 && AcapB[ori.cour].nbeM == 0)
                    emptyACity(ori.cour, numAvaCity);
            }

            whichCycles[ori.cour].insert(nbABcycles);
            if (ori.cour == startCity.cour && count > 3 && count % 2 == 0) {
                whichCycles[startCity.cour].insert(nbABcycles);
                nbABcycles++;
                if (nbABcycles == sizeSubPop)
                    return;
                break;
            }

            count++;
        }
    }
}
void crossover::addCurrentESetCycles(const std::set<int> &cycles) {
    for (int cycle : cycles)
        eSets[nbeSets].cycles.insert(cycle);
}

void crossover::generationEsets() {
    for (int i = 0; i < numC / 2; i++) {
        eSets[i].cycles.clear();
        commconCycles[i].clear();
    }

    for (int i = 0; i < numC / 2; i++) {
        if (whichCycles[i].empty())
            continue;
        std::set<int>::iterator k1 = whichCycles[i].begin();
        if (whichCycles[i].size() != 1) {
            while (k1 != whichCycles[i].end()) {
                std::set<int>::iterator k2 = k1;
                k2++;
                if (k2 == whichCycles[i].end())
                    break;
                commconCycles[*k1].insert(*k2);
                commconCycles[*k2].insert(*k1);
                k1++;
            }
        } else
            commconCycles[*k1].insert(*k1);
    }

    for (int i = 0; i < nbABcycles; i++) {
        for (int j = 0; j < i; j++) {
            if (commconCycles[j].empty())
                continue;
            if (commconCycles[i].size() != commconCycles[j].size())
                continue;
            bool theSame = true;
            std::set<int>::iterator k1 = commconCycles[i].begin();
            std::set<int>::iterator k2 = commconCycles[j].begin();
            while (k1 != commconCycles[i].end() && k2 != commconCycles[j].end()) {
                int r1 = *k1;
                int r2 = *k2;
                if (r1 != r2) {
                    theSame = false;
                    break;
                }
                k1++;
                k2++;
            }
            if (theSame)
                commconCycles[j].clear();
        }
    }
    nbeSets = 0;

    if (data->eSets_strategy == 1) {
        for (int i = 0; i < nbABcycles; i++) {
            if (commconCycles[i].empty())
                continue;
            addCurrentESetCycles(commconCycles[i]);
            nbeSets++;
        }
        return;
    }

    int actualNbCycles = nbABcycles;
    int reccord = 0;
    int iiter = 1;

    int bEta = 3;

    if (data->eSets_strategy == 2) {
        if (actualNbCycles < bEta) {
            for (int i = 0; i < actualNbCycles; i++) {
                eSets[nbeSets].cycles.insert(i);
                nbeSets++;
            }
            return;
        }
        for (int ii = 0; ii < bEta; ii++) {
            for (int i = reccord; i < iiter * actualNbCycles / bEta; i++) {
                eSets[nbeSets].cycles.insert(i);
                reccord++;
            }
            nbeSets++;
            iiter++;
        }
        return;
    }

    int nbCommonCycles = 0;
    std::vector<int> activeGroups;
    for (int i = 0; i < nbABcycles; i++)
        if (!commconCycles[i].empty()) {
            activeGroups.push_back(i);
            nbCommonCycles++;
        }
    actualNbCycles = nbCommonCycles;
    reccord = 0;
    iiter = 1;
    if (data->eSets_strategy == 3) {
        if (actualNbCycles < bEta) {
            for (int i = 0; i < nbABcycles; i++) {
                if (commconCycles[i].empty())
                    continue;
                addCurrentESetCycles(commconCycles[i]);
                nbeSets++;
            }
            return;
        }

        for (int ii = 0; ii < bEta; ii++) {
            for (int i = reccord; i < iiter * actualNbCycles / bEta;) {
                addCurrentESetCycles(commconCycles[activeGroups[i]]);
                reccord++;
                i++;
            }
            nbeSets++;
            iiter++;
        }
        return;
    }
}
void crossover::flipSubString(Node *nodeU, Node *nodeV, bool reverse) {
    Node *tem = NULL;
    if (!reverse) {
        while (nodeU != nodeV) {
            tem = nodeU->next;
            nodeU->next = nodeU->pre;
            nodeU->pre = tem;
            nodeU = tem;
        }
        tem = nodeU->next;
        nodeU->next = nodeU->pre;
        nodeU->pre = tem;
    } else {
        while (nodeU != nodeV) {
            tem = nodeU->pre;
            nodeU->pre = nodeU->next;
            nodeU->next = tem;
            nodeU = tem;
        }
        tem = nodeU->pre;
        nodeU->pre = nodeU->next;
        nodeU->next = tem;
    }
}
void crossover::generationIntermediateSolutions() {
    int p1, p2;
    Node *tem = NULL, *tem2 = NULL;
    for (int i = 0; i < nbeSets; i++) {
        popman->copyAllData(subPop + i, fa);
        off = subPop + i;

        std::set<int>::iterator k = eSets[i].cycles.begin();
        for (; k != eSets[i].cycles.end(); k++) {
            for (int j = 0; j < ABcycles[*k].nbCity - 1; j++) {
                if (ABcycles[*k].sque[j].isFather) {
                    p1 = ABcycles[*k].sque[j].cour;
                    p2 = ABcycles[*k].sque[j + 1].cour;

                    if (p1 == p2) {
                        std::cout
                            << "the ABcycles is wrong with that an edge starts and ends at the same city"
                            << std::endl;
                        exit(EXIT_FAILURE);
                    }

                    Node *pp1 = NULL, *pp2 = NULL;
                    if (p1 >= numV) {
                        if (off->client[p2].pre != NULL && off->client[p2].pre->cour == p1)
                            pp1 = off->client[p2].pre;
                        else if (off->client[p2].next != NULL && off->client[p2].next->cour == p1)
                            pp1 = off->client[p2].next;
                        pp2 = &off->client[p2];
                    } else if (p2 >= numV) {
                        if (off->client[p1].pre != NULL && off->client[p1].pre->cour == p2)
                            pp2 = off->client[p1].pre;
                        else if (off->client[p1].next != NULL && off->client[p1].next->cour == p2)
                            pp2 = off->client[p1].next;
                        pp1 = &off->client[p1];
                    } else {
                        pp1 = &off->client[p1];
                        pp2 = &off->client[p2];
                    }

                    if (pp1->pre != NULL && pp1->pre == pp2) {
                        pp1->pre = NULL;
                        pp2->next = NULL;
                        continue;
                    }
                    if (pp1->next != NULL && pp1->next == pp2) {
                        pp1->next = NULL;
                        pp2->pre = NULL;
                    }
                }
            }
        }

        int p1Before, p2Before;
        for (k = eSets[i].cycles.begin(); k != eSets[i].cycles.end(); k++) {
            for (int j = 0; j < ABcycles[*k].nbCity - 1; j++) {
                if (!ABcycles[*k].sque[j].isFather) {
                    p1 = ABcycles[*k].sque[j].cour;
                    p2 = ABcycles[*k].sque[j + 1].cour;
                    if (p1 == p2) {
                        std::cout
                            << "the ABcycles is wrong with that an edge starts and ends at the same city"
                            << std::endl;
                        exit(EXIT_FAILURE);
                    }

                    if (p1 >= numV) {
                        if (off->client[p2].pre == NULL)
                            off->client[p2].pre = &gStart[p1];
                        else if (off->client[p2].next == NULL)
                            off->client[p2].next = &gEnd[p1];
                    } else {
                        if (p2 >= numV) {
                            if (off->client[p1].pre == NULL)
                                off->client[p1].pre = &gStart[p2];
                            else if (off->client[p1].next == NULL)
                                off->client[p1].next = &gEnd[p2];
                        } else {
                            p1Before = 0;
                            while (1) {
                                if (off->client[p1].pre == NULL) {
                                    p1Before = 1;
                                    break;
                                }
                                if (off->client[p1].next == NULL) {
                                    p1Before = -1;
                                    break;
                                }
                            }
                            p2Before = 0;
                            while (1) {
                                if (p1Before == 1 && off->client[p2].next == NULL) {
                                    p2Before = -1;
                                    break;
                                }
                                if (p1Before == -1 && off->client[p2].pre == NULL) {
                                    p2Before = 1;
                                    break;
                                }
                                if (off->client[p2].pre == NULL) {
                                    p2Before = 1;
                                    break;
                                }
                                if (off->client[p2].next == NULL) {
                                    p2Before = -1;
                                    break;
                                }
                            }

                            if (p1Before == 1 && p2Before == -1) {
                                off->client[p1].pre = &off->client[p2];
                                off->client[p2].next = &off->client[p1];
                                continue;
                            }
                            if (p1Before == -1 && p2Before == 1) {
                                off->client[p1].next = &off->client[p2];
                                off->client[p2].pre = &off->client[p1];
                                continue;
                            }
                            if (p1Before == 1 && p2Before == 1) {
                                tem = &off->client[p2];
                                tem2 = tem;
                                while (tem2->next != NULL)
                                    tem2 = tem2->next;
                                flipSubString(tem, tem2, false);
                                off->client[p1].pre = &off->client[p2];
                                off->client[p2].next = &off->client[p1];
                                continue;
                            }
                            if (p1Before == -1 && p2Before == -1) {
                                tem = &off->client[p2];
                                tem2 = tem;
                                while (tem2->pre != NULL)
                                    tem2 = tem2->pre;
                                flipSubString(tem, tem2, true);
                                off->client[p1].next = &off->client[p2];
                                off->client[p2].pre = &off->client[p1];
                                continue;
                            }
                        }
                    }
                }
            }
        }

        for (int k = 0; k < numV; k++) {
            if (off->client[k].pre == NULL && off->client[k].next != NULL)
                std::cout << "report a wrong left   " << k << std::endl;
            if (off->client[k].pre != NULL && off->client[k].next == NULL)
                std::cout << "report a wrong right   " << k << std::endl;
        }

        updateTheLocation();
        eliminateSubtours();
        checkTheSolution();
    }
}
void crossover::checkTheSolution() {
    for (int i = 0; i < numV; i++)
        auxA2[i] = false;
    Node *p;
    for (int i = 0; i < numL; i++) {
        if (off->location[i].used) {
            for (int j = 0; j < off->location[i].maxVisit; j++) {
                if (off->location[i].depots[j].route->used) {
                    p = off->location[i].depots[j].next;
                    while (!p->isDepot) {
                        auxA2[p->cour] = true;
                        p = p->next;
                    }
                }
            }
        }
    }
    for (int i = 0; i < numV; i++) {
        if (!auxA2[i]) {
            std::cout << "The intermediate solution is wrong and the customer " << i << " is wrong"
                      << std::endl;
        }
    }
}
void crossover::updateTheLocation() {

    headSubstring.clear();
    for (int i = 0; i < numL; i++) {
        for (int j = 0; j < off->location[i].maxVisit; j++) {
            off->location[i].depots[j].next = &off->location[i].endDepots[j];
            off->location[i].depots[j].pre = &off->location[i].endDepots[j];
            off->location[i].depots[j].route->load = 0;
            off->location[i].endDepots[j].pre = &off->location[i].depots[j];
            off->location[i].endDepots[j].next = &off->location[i].depots[j];
        }
        int ll = i + numV;
        off->updateLocationInfor(ll);
    }
    for (int i = 0; i < numV; i++)
        auxA1[i] = false;

    Node *p, *start;
    bool isFirst, subtour;
    Node *nodeU, *nodeV;
    for (int i = 0; i < numV; i++) {
        if (!auxA1[i]) {
            nodeU = NULL;
            nodeV = NULL;
            p = &off->client[i];
            start = p;
            auxA1[p->cour] = true;
            isFirst = true;
            subtour = true;
            while (p != start || isFirst) {
                if (p->next->cour >= numV) {
                    subtour = false;
                    p->route = NULL;
                    nodeU = p;
                    break;
                }
                p->route = NULL;
                p = p->next;
                auxA1[p->cour] = true;
                isFirst = false;
            }
            if (subtour) {
                headSubstring.push_back(start);
                continue;
            }
            p = start;
            isFirst = true;
            while (p != start || isFirst) {
                p->route = NULL;
                if (p->pre->cour >= numV) {
                    p->route = NULL;
                    nodeV = p;
                    break;
                }
                p = p->pre;
                auxA1[p->cour] = true;
                isFirst = false;
            }

            if (nodeU != NULL && nodeV != NULL && nodeU->next->cour == nodeV->pre->cour) {
                int loc = nodeU->next->cour - numV;
                off->location[loc].depots[off->location[loc].splitTime].next = nodeV;
                nodeV->pre = &off->location[loc].depots[off->location[loc].splitTime];
                off->location[loc].endDepots[off->location[loc].splitTime].pre = nodeU;
                nodeU->next = &off->location[loc].endDepots[off->location[loc].splitTime];
                Route *routeU = off->location[loc].depots[off->location[loc].splitTime].route;
                while (nodeU != nodeV) {
                    nodeV->route = routeU;
                    nodeV = nodeV->next;
                }
                nodeU->route = routeU;
                off->updateRouteInfor(
                    off->location[loc].depots[off->location[loc].splitTime].route->cour, -1);
                off->location[loc].splitTime++;
                loc = loc + numV;
                off->updateLocationInfor(loc);
            } else if (nodeU != NULL && nodeV != NULL && nodeU->next->cour != nodeV->pre->cour) {
                p = nodeV;
                p->cumulatedLoad = p->dem;
                while (p != nodeU) {
                    p = p->next;
                    p->cumulatedLoad = p->pre->cumulatedLoad + p->dem;
                }

                Node *depot = NULL;
                if (off->location[nodeU->next->cour - numV].iscapCon) {
                    int loc1 = nodeV->pre->cour - numV;
                    int loc2 = nodeU->next->cour - numV;
                    if (off->location[loc1].remainInven + p->cumulatedLoad <=
                            off->location[loc1].inventory &&
                        off->location[loc2].remainInven + p->cumulatedLoad <=
                            off->location[loc2].inventory) {
                        if ((off->location[loc1].inventory - off->location[loc1].remainInven -
                             p->cumulatedLoad) <=
                            (off->location[loc2].inventory - off->location[loc2].remainInven -
                             p->cumulatedLoad))
                            depot = &off->location[loc1].depots[off->location[loc1].splitTime];
                        else
                            depot = &off->location[loc2].depots[off->location[loc2].splitTime];
                    } else if (off->location[loc1].remainInven + p->cumulatedLoad >
                                   off->location[loc1].inventory &&
                               off->location[loc2].remainInven + p->cumulatedLoad <=
                                   off->location[loc2].inventory)
                        depot = &off->location[loc2].depots[off->location[loc2].splitTime];
                    else if (off->location[loc1].remainInven + p->cumulatedLoad <=
                                 off->location[loc1].inventory &&
                             off->location[loc2].remainInven + p->cumulatedLoad >
                                 off->location[loc2].inventory)
                        depot = &off->location[loc1].depots[off->location[loc1].splitTime];
                    else if (off->location[loc1].remainInven + p->cumulatedLoad >
                                 off->location[loc1].inventory &&
                             off->location[loc2].remainInven + p->cumulatedLoad >
                                 off->location[loc2].inventory) {
                        if ((off->location[loc1].remainInven + p->cumulatedLoad -
                             off->location[loc1].inventory) <=
                            (off->location[loc2].remainInven + p->cumulatedLoad -
                             off->location[loc2].inventory))
                            depot = &off->location[loc1].depots[off->location[loc1].splitTime];
                        else
                            depot = &off->location[loc2].depots[off->location[loc2].splitTime];
                    }
                    if (depot == NULL)
                        std::cout << "report a bug in assign locations" << std::endl;
                    else {
                        nodeU->next = &off->location[depot->cour - numV]
                                           .endDepots[off->location[depot->cour - numV].splitTime];
                        off->location[depot->cour - numV]
                            .endDepots[off->location[depot->cour - numV].splitTime]
                            .pre = nodeU;
                        nodeV->pre = &off->location[depot->cour - numV]
                                          .depots[off->location[depot->cour - numV].splitTime];
                        off->location[depot->cour - numV]
                            .depots[off->location[depot->cour - numV].splitTime]
                            .next = nodeV;
                        int loc = depot->cour - numV;
                        Route *routeU =
                            off->location[loc].depots[off->location[loc].splitTime].route;
                        while (nodeU != nodeV) {
                            nodeV->route = routeU;
                            nodeV = nodeV->next;
                        }
                        nodeU->route = routeU;
                        off->updateRouteInfor(
                            off->location[loc].depots[off->location[loc].splitTime].route->cour,
                            -1);
                        off->location[loc].splitTime++;
                        loc = loc + numV;
                        off->updateLocationInfor(loc);
                    }
                }

                else {
                    int loc;
                    if (data->D[nodeV->cour][nodeV->pre->cour] >
                        data->D[nodeU->cour][nodeU->next->cour])
                        loc = nodeV->pre->cour - numV;
                    else
                        loc = nodeU->next->cour - numV;
                    off->location[loc].depots[off->location[loc].splitTime].next = nodeV;
                    nodeV->pre = &off->location[loc].depots[off->location[loc].splitTime];
                    off->location[loc].endDepots[off->location[loc].splitTime].pre = nodeU;
                    nodeU->next = &off->location[loc].endDepots[off->location[loc].splitTime];
                    nodeV->route = off->location[loc].depots[off->location[loc].splitTime].route;
                    while (nodeV != nodeU) {
                        nodeV = nodeV->next;
                        nodeV->route = nodeV->pre->route;
                    }
                    nodeU->route = nodeV->pre->route;
                    off->updateRouteInfor(
                        off->location[loc].depots[off->location[loc].splitTime].route->cour, -1);
                    off->location[loc].splitTime++;
                    loc = loc + numV;
                    off->updateLocationInfor(loc);
                }
            }
        }
    }
}
void crossover::eliminateSubtours() {
    Node *nodeU, *nodeV, *begin, *p, *q;
    bool isFirst;
    int load;
    while ((int)headSubstring.size() > 0) {

        double delta, min_delta;
        p = headSubstring[0];
        headSubstring.erase(headSubstring.begin());
        begin = p;
        load = 0;
        isFirst = true;
        while (p != begin || isFirst) {
            load += p->dem;
            p = p->next;
            isFirst = false;
        }
        p = begin;
        isFirst = true;
        min_delta = Max;
        nodeU = NULL;
        nodeV = NULL;
        while (p != begin || isFirst) {
            for (int i = 0; i < (int)data->edgeNode[p->cour].size(); i++) {
                if (data->edgeNode[p->cour][i] < numV) {
                    q = &off->client[data->edgeNode[p->cour][i]];
                    if (q->route != NULL) {
                        if (q->route->whichLocation->remainInven + load <=
                            q->route->whichLocation->inventory) {
                            delta =
                                data->D[p->cour][q->cour] + data->D[p->pre->cour][q->next->cour] -
                                data->D[p->cour][p->pre->cour] - data->D[q->cour][q->next->cour];
                            if (delta < min_delta) {
                                nodeU = p;
                                nodeV = q;
                                min_delta = delta;
                            }
                        }
                    }
                } else {
                    q = &off->location[data->edgeNode[p->cour][i] - numV]
                             .depots[off->location[data->edgeNode[p->cour][i] - numV].splitTime];
                    if (q->route->whichLocation->remainInven + load <
                        q->route->whichLocation->inventory) {
                        delta = data->D[p->cour][q->cour] + data->D[p->pre->cour][q->next->cour] -
                                data->D[p->cour][p->pre->cour] - data->D[q->cour][q->next->cour];
                        if (delta < min_delta) {
                            nodeU = p;
                            nodeV = q;
                            min_delta = delta;
                        }
                    }
                }
            }
            p = p->next;
            isFirst = false;
        }
        if (nodeU == NULL || nodeV == NULL) {
            isFirst = true;
            p = begin;
            while (p != begin || isFirst) {
                for (int i = 0; i < (int)data->edgeNode[p->cour].size(); i++) {
                    if (data->edgeNode[p->cour][i] < numV) {
                        q = &off->client[data->edgeNode[p->cour][i]];
                        if (q->route != NULL) {
                            delta =
                                data->D[p->cour][q->cour] + data->D[p->pre->cour][q->next->cour] -
                                data->D[p->cour][p->pre->cour] - data->D[q->cour][q->next->cour];
                            if (delta < min_delta) {
                                nodeU = p;
                                nodeV = q;
                                min_delta = delta;
                            }
                        }
                    } else {
                        q = &off->location[data->edgeNode[p->cour][i] - numV].depots
                                 [off->location[data->edgeNode[p->cour][i] - numV].splitTime];
                        delta = data->D[p->cour][q->cour] + data->D[p->pre->cour][q->next->cour] -
                                data->D[p->cour][p->pre->cour] - data->D[q->cour][q->next->cour];
                        if (delta < min_delta) {
                            nodeU = p;
                            nodeV = q;
                            min_delta = delta;
                        }
                    }
                }
                p = p->next;
                isFirst = false;
            }
        }
        if (nodeU == NULL || nodeV == NULL) {
            isFirst = true;
            p = begin;
            while (p != begin || isFirst) {
                for (int i = 0; i < numC; i++) {
                    if (i < numV) {
                        q = &off->client[i];
                        if (q->route != NULL) {
                            delta =
                                data->D[p->cour][q->cour] + data->D[p->pre->cour][q->next->cour] -
                                data->D[p->cour][p->pre->cour] - data->D[q->cour][q->next->cour];
                            if (delta < min_delta) {
                                nodeU = p;
                                nodeV = q;
                                min_delta = delta;
                            }
                        }
                    } else {
                        q = &off->location[i - numV].depots[off->location[i - numV].splitTime];
                        delta = data->D[p->cour][q->cour] + data->D[p->pre->cour][q->next->cour] -
                                data->D[p->cour][p->pre->cour] - data->D[q->cour][q->next->cour];
                        if (delta < min_delta) {
                            nodeU = p;
                            nodeV = q;
                            min_delta = delta;
                        }
                    }
                }
                p = p->next;
                isFirst = false;
            }
        }
        if (nodeU == NULL || nodeV == NULL)
            std::cout << "there are errors in eliminating subtours" << std::endl;
        else {
            Node *nodeX, *nodeY;
            nodeU->route = nodeV->route;
            p = nodeU->next;
            while (p != nodeU) {
                p->route = nodeV->route;
                p = p->next;
            }
            nodeX = nodeU->pre;
            nodeY = nodeV->next;

            nodeV->next = nodeU;
            nodeU->pre = nodeV;
            nodeX->next = nodeY;
            nodeY->pre = nodeX;
        }
        off->updateRouteInfor(nodeV->route->cour, -1);
        off->updateLocationInfor(nodeV->route->whichLocation->cour);
    }
}
bool crossover::gEAX(Individual *fa, Individual *ma, Individual *subPop, int &numSubPop) {
    numSubPop = 0;
    this->fa = fa;
    this->ma = ma;
    this->subPop = subPop;

    loadFaCapMa();
    extractABcycles();
    if (nbABcycles == sizeSubPop)
        return false;
    generationEsets();
    generationIntermediateSolutions();

    numSubPop = nbeSets;
    return true;
}
