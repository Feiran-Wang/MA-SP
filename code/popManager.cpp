// Author: Feiran
// Manage population admission, ranking, replacement, and SP feedback.

#include "TimeWindowOperators.h"

#include "popManager.h"
#include "RoutePool.h"
#include "StudyEvents.h"
#include <cmath>

namespace {
bool sameCustomerAdjacency(const Individual &left, const Individual &right, int customers,
                           bool directed) {
    for (int customer = 0; customer < customers; ++customer) {
        const Node &a = left.client[customer];
        const Node &b = right.client[customer];
        if (!a.pre || !a.next || !b.pre || !b.next)
            return false;
        const bool sameDirection = a.pre->cour == b.pre->cour && a.next->cour == b.next->cour;
        const bool reversed = a.pre->cour == b.next->cour && a.next->cour == b.pre->cour;
        if (!sameDirection && (directed || !reversed))
            return false;
    }
    return true;
}
}

std::string popManager::studyPopulationId(int slot) {
    if (!study || slot < 0)
        return "";
    if (studyPopulationAtSlot[slot] < 0)
        replaceStudyPopulation(slot, "initialization_or_refill");
    return studyPopulations[studyPopulationAtSlot[slot]].id;
}
void popManager::closeStudyPopulation(int slot) {
    if (!study || slot < 0 || studyPopulationAtSlot[slot] < 0)
        return;
    StudyPopulation &old = studyPopulations[studyPopulationAtSlot[slot]];
    old.exitedAt = data->iteration;
    study->event("population_exit", "\"population_id\":" + StudyEvents::quote(old.id) +
                                        ",\"exit_reason\":\"replacement_or_restart_or_reduction\"");
    studyPopulationAtSlot[slot] = -1;
}
void popManager::replaceStudyPopulation(int slot, const std::string &origin) {
    if (!study)
        return;
    closeStudyPopulation(slot);
    StudyPopulation next;
    next.id = "population_" + std::to_string(studyPopulations.size() + 1);
    next.slot = slot;
    next.installedAt = data->iteration;
    studyPopulationAtSlot[slot] = static_cast<int>(studyPopulations.size());
    studyPopulations.push_back(next);
    study->event(
        "population_created",
        "\"population_id\":" + StudyEvents::quote(next.id) + ",\"slot\":" + std::to_string(slot) +
            ",\"origin\":" + StudyEvents::quote(origin) + ",\"origin_candidate_id\":" +
            (origin == "SP_direct_install" ? StudyEvents::quote(study->candidateId) : "null"));
}
void popManager::finishStudyPopulations() {
    if (!study)
        return;
    for (const auto &population : studyPopulations) {
        study->event(
            "population_summary",
            "\"population_id\":" + StudyEvents::quote(population.id) +
                ",\"slot\":" + std::to_string(population.slot) + ",\"install_counter\":" +
                std::to_string(population.installedAt) + ",\"exit_counter\":" +
                (population.exitedAt < 0 ? "null" : std::to_string(population.exitedAt)) +
                ",\"right_censored\":" + (population.exitedAt < 0 ? "true" : "false") +
                ",\"later_parent_selections\":" + std::to_string(population.parentSelections) +
                ",\"later_incumbent_improvements_observed\":" +
                std::to_string(population.incumbentImprovements));
    }
}

popManager::popManager(read_data *data) {
    this->data = data;
    this->numV = data->numV;
    this->numL = data->numL;
    this->numC = data->numC;
    order = new int[numV];
    popula = std::vector<std::vector<Individual *>>(100);
    reverseReplacementCount = 0;
    popCreationIteration = std::vector<long long int>(100, 0);
}

void popManager::deleteMembers(SubPopulation &population) {
    for (Individual *individual : population)
        delete individual;
}

void popManager::linkProximities(Individual *individual, const SubPopulation &population) {
    // Evaluate each score once and insert the resident's proximity link first.
    for (Individual *member : population) {
        double distance = individual->brokenPairsDistance(member);
        member->indivsPerProximity.insert({distance, individual});
        individual->indivsPerProximity.insert({distance, member});
    }
}

popManager::~popManager() {
    delete[] order;
    for (auto &population : popula) {
        deleteMembers(population);
        population.clear();
    }
}
void popManager::copyAllData(Individual *s, Individual *ss) {
    if (s == ss)
        return;
    // Closed depots retain their empty-slot index after routes are rebuilt.
    s->initilization();
    Node *nodeU;

    for (int i = 0; i < numV; i++) {
        s->client[i].pre = NULL;
        s->client[i].next = NULL;
        s->client[i].route = NULL;
    }
    for (int i = 0; i < numL; i++) {
        s->location[i].emptyRoutes.clear();
        for (int j = 0; j < s->location[i].maxVisit; j++) {
            s->location[i].depots[j].next = &s->location[i].endDepots[j];
            s->location[i].endDepots[j].pre = &s->location[i].depots[j];
            s->updateRouteInfor(s->location[i].depots[j].route->cour, -1);
        }
        s->location[i].used = false;
    }

    int count;
    Route *routeU;
    for (int i = numV; i < numC; i++) {
        if (!ss->location[i - numV].used)
            continue;
        for (int j = 0; j < ss->location[i - numV].maxVisit; j++) {
            if (ss->location[i - numV].depots[j].route->used) {
                count = 0;
                nodeU = ss->location[i - numV].depots[j].next;
                while (!nodeU->isDepot) {
                    order[count] = nodeU->cour;
                    count++;
                    nodeU = nodeU->next;
                }
                s->location[i - numV].depots[j].route->used = true;
                routeU = s->location[i - numV].depots[j].route;
                nodeU = &s->client[order[0]];
                nodeU->pre = routeU->depot;
                nodeU->route = routeU;
                routeU->depot->next = nodeU;
                for (int k = 1; k < count; k++) {
                    s->client[order[k]].pre = nodeU;
                    nodeU->next = &s->client[order[k]];
                    s->client[order[k]].route = routeU;
                    nodeU = &s->client[order[k]];
                }
                nodeU->next = routeU->endDepot;
                routeU->endDepot->pre = nodeU;
                s->updateRouteInfor(routeU->cour, -1);
            } else {
                s->location[i - numV].depots[j].next = &s->location[i - numV].endDepots[j];
                s->location[i - numV].endDepots[j].pre = &s->location[i - numV].depots[j];
                s->updateRouteInfor(s->location[i - numV].depots[j].route->cour, -1);
            }
        }
        s->updateLocationInfor(i);
    }
    s->dis = ss->dis;
    s->fit = ss->fit;
    s->biasedFit = ss->biasedFit;
    s->isFeasible = ss->isFeasible;
}
int popManager::addIndividual(Individual *indiv, int rankPop, bool isInitial, RoutePool *pool) {
    lastAddAccepted = false;
    if (data->hardTimeWindows) {
        if (!indiv->isFeasible || !masptw::validate(indiv, data))
            return -1;
        std::set<int> actual;
        for (int l = 0; l < numL; ++l)
            if (indiv->location[l].used)
                actual.insert(l + numV);
        rankPop = data->numLocConfiguration;
        for (int p = 0; p < data->numLocConfiguration; ++p)
            if (actual == data->compactLocConfigure[p]) {
                rankPop = p;
                break;
            }
    }
    if (isInitial && rankPop >= 0 && rankPop < data->numLocConfiguration) {
        std::set<int> actual;
        for (int depot = 0; depot < numL; ++depot)
            if (indiv->location[depot].used)
                actual.insert(depot + numV);
        if (actual != data->compactLocConfigure[rankPop])
            rankPop = data->numLocConfiguration;
    }

    SubPopulation &subpop = popula[rankPop];

    for (Individual *myIndividual : subpop)
        if (myIndividual->fit + Min > indiv->fit && myIndividual->fit - Min < indiv->fit)
            return -1;

    Individual *myIndividual = new Individual();
    myIndividual->define(data);
    myIndividual->initilization();
    copyAllData(myIndividual, indiv);
    linkProximities(myIndividual, subpop);

    int place = (int)subpop.size();
    while (place > 0 && subpop[place - 1]->fit > indiv->fit - Min)
        place--;
    subpop.emplace(subpop.begin() + place, myIndividual);
    lastAddAccepted = true;
    if (study)
        studyPopulationId(rankPop);

    std::vector<int> depot;
    bool isObtainNewPop = false;
    int index = -1;

    if (indiv->fit + Min < bestIndivRestart->fit) {

        indiv->isRight();
        bestIndivRestart->initilization();
        copyAllData(bestIndivRestart, indiv);
        data->meetBestTime = (double)clock() / (double)CLOCKS_PER_SEC;
        data->nbIterNonImpro = 0;
        if (indiv->fit + Min < global->fit) {
            global->initilization();
            copyAllData(global, indiv);
            data->meetBestTime = float((double)clock() / (double)CLOCKS_PER_SEC);
            if (study)
                ++studyPopulations[studyPopulationAtSlot[rankPop]].incumbentImprovements;
        }
        if (study)
            study->sourcePopulationId = studyPopulationId(rankPop);
        if (studyIncumbent)
            studyIncumbent(global);
        if (study)
            study->sourcePopulationId.clear();

        if (!data->no_capacity_location) {
            depot.clear();
            for (int i = 0; i < numL; i++) {

                if ((data->experimentMode ? myIndividual : global)->location[i].used)
                    depot.push_back(i + numV);
            }
            if (rankPop == data->numLocConfiguration && !isInitial) {

                std::vector<std::pair<double, int>> avgCostPop;
                for (int i = 0; i < data->numLocConfiguration; i++) {
                    if ((int)popula[i].size() == 0)
                        continue;
                    double cost = 0;
                    int count = 0;
                    SubPopulation &sub = popula[i];
                    for (Individual *myIndiv : sub) {
                        cost += myIndiv->fit;
                        count++;
                    }
                    if (count > 0)
                        avgCostPop.push_back({cost / (double)count, i});
                }
                if ((int)avgCostPop.size() == 0)
                    index = 0;
                else {
                    double max_avg = 0;
                    for (int i = 0; i < (int)avgCostPop.size(); i++) {
                        if (max_avg < avgCostPop[i].first) {
                            max_avg = avgCostPop[i].first;
                            index = avgCostPop[i].second;
                        }
                    }
                }
                SubPopulation &sub = popula[index];
                if (study)
                    replaceStudyPopulation(index, "free_global_improvement");
                deleteMembers(sub);
                sub.clear();
                data->compactLocConfigure[index].clear();
                for (int i = 0; i < (int)depot.size(); i++)
                    data->compactLocConfigure[index].insert(depot[i]);
                Individual *myIndivi = new Individual();
                myIndivi->define(data);
                myIndivi->initilization();
                copyAllData(myIndivi, myIndividual);

                sub.emplace(sub.begin(), myIndivi);

                if (pool != nullptr && myIndivi->isFeasible) {
                    pool->extractRoutes(myIndivi);
                }

                isObtainNewPop = true;
                reverseReplacementCount++;

                popCreationIteration[index] = data->iteration;
            }
        }
    }

    if ((int)subpop.size() == data->popMax) {

        if (rankPop == data->numLocConfiguration && !isInitial) {

            Individual *best_free = subpop[0];
            for (Individual *ind : subpop) {
                if (ind->fit < best_free->fit)
                    best_free = ind;
            }

            double max_avg_cost = -1.0;
            int worst_res_idx = -1;
            bool found_empty_slot = false;

            for (int i = 0; i < data->numLocConfiguration; i++) {

                if (popula[i].empty()) {
                    worst_res_idx = i;
                    found_empty_slot = true;
                    break;
                }

                if ((data->iteration - popCreationIteration[i]) < data->popProtectionPeriod) {
                    continue;
                }

                double current_avg = 0.0;
                for (Individual *ind : popula[i])
                    current_avg += ind->fit;
                current_avg /= (double)popula[i].size();

                if (current_avg > max_avg_cost) {
                    max_avg_cost = current_avg;
                    worst_res_idx = i;
                }
            }

            if (worst_res_idx != -1) {
                bool is_authorized_to_replace = false;
                double target_fit_for_log = 0.0;

                if (found_empty_slot) {
                    is_authorized_to_replace = true;
                }

                else {
                    Individual *best_worst_res = popula[worst_res_idx][0];
                    for (Individual *ind : popula[worst_res_idx]) {
                        if (ind->fit < best_worst_res->fit)
                            best_worst_res = ind;
                    }

                    if (best_free->fit < best_worst_res->fit) {
                        is_authorized_to_replace = true;
                        target_fit_for_log = best_worst_res->fit;
                    }
                }

                if (is_authorized_to_replace) {

                    if (found_empty_slot) {
                        std::cout
                            << "\n>>> [Mechanism Monitor] 游离解发现系统空余槽位 (Empty Slot Detected)! (Free Best: "
                            << std::fixed << best_free->fit << ")，直接进驻并激活槽位 ["
                            << worst_res_idx << "]\n"
                            << std::endl;
                    } else {
                        std::cout << "\n>>> [Mechanism Monitor] 游离解触发种群更迭 (Free Best: "
                                  << std::fixed << best_free->fit
                                  << " < Incumbent Worst: " << target_fit_for_log
                                  << ")，成功接管槽位 [" << worst_res_idx << "]\n"
                                  << std::endl;
                    }

                    reverseReplacementCount++;

                    popCreationIteration[worst_res_idx] = data->iteration;

                    SubPopulation &target_sub = popula[worst_res_idx];
                    if (study)
                        replaceStudyPopulation(worst_res_idx, "free_tournament");

                    deleteMembers(target_sub);
                    target_sub.clear();

                    data->compactLocConfigure[worst_res_idx].clear();
                    for (int j = 0; j < numL; j++) {
                        if (best_free->location[j].used) {
                            data->compactLocConfigure[worst_res_idx].insert(j + numV);
                        }
                    }

                    Individual *seed_indiv = new Individual();
                    seed_indiv->define(data);
                    seed_indiv->initilization();
                    copyAllData(seed_indiv, best_free);
                    target_sub.emplace(target_sub.begin(), seed_indiv);

                    if (pool != nullptr && seed_indiv->isFeasible) {
                        pool->extractRoutes(seed_indiv);
                    }

                    deleteMembers(subpop);
                    subpop.clear();
                    if (study)
                        closeStudyPopulation(rankPop);

                    return worst_res_idx;
                }
            }
        }

        while ((int)subpop.size() > data->popMin)
            removeWorstBiasedFitness(subpop);
    }

    if (pool != nullptr && indiv->isFeasible) {
        if (indiv->fit < pool->getAverageFitness()) {
            pool->extractRoutes(indiv);
            pool->recordSolutionFitness(indiv->fit);
        }
    }

    if (isObtainNewPop)
        return index;
    else
        return -1;
}

Individual *popManager::selectionParentSolution(Individual *exist, int rankPop) {
    SubPopulation &subpop = popula[rankPop];
    if (study && !subpop.empty()) {
        studyPopulationId(rankPop);
        ++studyPopulations[studyPopulationAtSlot[rankPop]].parentSelections;
    }
    if (data->hardTimeWindows) {
        std::vector<Individual *> eligible;
        for (Individual *item : subpop)
            if (item != exist)
                eligible.push_back(item);
        if (eligible.empty())
            return subpop.empty() ? nullptr : subpop.front();
        updateBiasedFitnesses(subpop);
        Individual *a = eligible[RandomInt(0, static_cast<int>(eligible.size()) - 1)];
        Individual *b = eligible[RandomInt(0, static_cast<int>(eligible.size()) - 1)];
        return a->biasedFit < b->biasedFit ? a : b;
    }
    updateBiasedFitnesses(subpop);

    if (subpop.size() == 2) {
        if (exist == subpop[0] && subpop[0]->biasedFit < subpop[1]->biasedFit)
            return subpop[1];
        if (exist == subpop[1] && subpop[1]->biasedFit < subpop[0]->biasedFit)
            return subpop[0];
    }

    int rand1, rand2;
    Individual *target;
    if (subpop.size() == 1) {
        target = subpop[0];
        return target;
    }
    int max_idx = (int)subpop.size() - 1;
    while (1) {
        rand1 = RandomInt(0, max_idx);
        rand2 = RandomInt(0, max_idx);
        while (rand1 == rand2)
            rand2 = RandomInt(0, max_idx);
        if (subpop[rand1]->biasedFit < subpop[rand2]->biasedFit)
            target = subpop[rand1];
        else
            target = subpop[rand2];

        if (target != exist)
            break;
    }
    return target;
}

void popManager::reStart() {
    std::cout << "----Restart and create a new population----" << std::endl;
    for (int i = 0; i < data->numLocConfiguration + 1; i++) {
        if (study)
            closeStudyPopulation(i);
        deleteMembers(popula[i]);
        popula[i].clear();
    }
    data->nbIterNonImpro = 0;
    bestIndivRestart->fit = Max;
}
void popManager::removeWorstBiasedFitness(SubPopulation &pop) {
    updateBiasedFitnesses(pop);
    if (pop.size() <= 1)
        throw std::string(
            "Eliminating the best individual: this should not occur in the algorithm");

    Individual *worstIndividual = NULL;
    int worstIndividualPosition = -1;
    bool isWorstIndividualClone = false;
    double worstIndividualBiasedFitness = -1.e30;
    for (int i = 0; i < (int)pop.size(); i++) {
        // Topology identifies clones independently of the retained negative ranking score.
        bool isClone = false;
        for (const Individual *other : pop) {
            if (other != pop[i] &&
                sameCustomerAdjacency(*pop[i], *other, numV, data->hardTimeWindows)) {
                isClone = true;
                break;
            }
        }
        if ((isClone && !isWorstIndividualClone) ||
            (isClone == isWorstIndividualClone &&
             pop[i]->biasedFit > worstIndividualBiasedFitness)) {
            worstIndividualBiasedFitness = pop[i]->biasedFit;
            isWorstIndividualClone = isClone;
            worstIndividualPosition = i;
            worstIndividual = pop[i];
        }
    }

    pop.erase(pop.begin() + worstIndividualPosition);
    for (Individual *myIndividual2 : pop)
        myIndividual2->removeProximity(worstIndividual);
    delete worstIndividual;
    worstIndividual = NULL;
}

void popManager::updateBiasedFitnesses(SubPopulation &pop) {

    std::vector<std::pair<double, int>> ranking;
    for (int i = 0; i < (int)pop.size(); i++)
        ranking.push_back({-pop[i]->averageBrokenPairsDistanceClosest(data->nbClost), i});
    std::sort(ranking.begin(), ranking.end());

    if (pop.size() == 1)
        pop[0]->biasedFit = 0;
    else {
        for (int i = 0; i < (int)pop.size(); i++) {
            double divRank = (double)i / (double)(pop.size() - 1);
            double fitRank = (double)ranking[i].second / (double)(pop.size() - 1);
            if ((int)pop.size() <= data->nbElite)
                pop[ranking[i].second]->biasedFit = fitRank;
            else
                pop[ranking[i].second]->biasedFit =
                    fitRank + (1.0 - (double)data->nbElite / (double)pop.size()) * divRank;
        }
    }
}
void popManager::reduceHalfSubPop() {
    if ((int)popula.size() < 3)
        return;
    std::vector<std::pair<double, int>> averageCost;
    double avgCost;
    for (int i = 0; i < data->numLocConfiguration; i++) {
        avgCost = 0;
        for (int j = 0; j < (int)popula[i].size(); j++) {
            avgCost += popula[i][j]->fit;
            if (std::abs(bestIndivRestart->fit - popula[i][j]->fit) < 0.000001)
                avgCost = -Max;
        }
        if ((int)popula[i].size() == 0)
            avgCost = Max;
        else
            avgCost = avgCost / (double)popula[i].size();
        averageCost.push_back({avgCost, i});
    }
    std::sort(averageCost.begin(), averageCost.end());
    int halfSize = (int)(data->numLocConfiguration / 2.);
    std::vector<int> remove;
    for (int i = (int)averageCost.size() - 1; i > halfSize; i--)
        remove.push_back(averageCost[i].second);
    std::sort(remove.begin(), remove.end());
    std::reverse(remove.begin(), remove.end());
    for (int i = 0; i < (int)remove.size(); i++) {
        if (study)
            closeStudyPopulation(remove[i]);
        SubPopulation &sub = popula[remove[i]];
        deleteMembers(sub);
        popula.erase(popula.begin() + remove[i]);
        data->compactLocConfigure.erase(data->compactLocConfigure.begin() + remove[i]);
        // Remove per-slot metadata at the same index as its population.
        popCreationIteration.erase(popCreationIteration.begin() + remove[i]);
        studyPopulationAtSlot.erase(studyPopulationAtSlot.begin() + remove[i]);
    }
    data->numLocConfiguration = data->numLocConfiguration - remove.size();

    for (int i = 0; i < data->dimension; i++) {
        data->dimTabu1[i] = -1;
        data->dimTabu2[i] = -1;
    }
    long long int var, var1, var2;
    ;
    int seed;
    for (int i = 0; i < data->numLocConfiguration; i++) {
        if ((int)popula[i].size() > 0) {
            var = 0;
            var1 = 0;
            auto it = data->compactLocConfigure[i].begin();
            for (; it != data->compactLocConfigure[i].end(); it++) {
                seed = *it - data->numV;
                var2 = pow(2, seed);
                var += var2;
                var1 += var2 + 9527 * seed;
            }
            data->dimTabu1[var % data->dimension] = i;
            data->dimTabu2[var1 % data->dimension] = i;
        }
    }
}

int popManager::injectSPSolution(Individual *spIndiv, RoutePool *pool) {
    const double globalBefore = global->fit;
    lastSPDecision = "invalid_candidate";
    lastSPTargetId.clear();
    lastSPTarget = -1;
    lastSPTargetMean = lastSPTargetBest = std::numeric_limits<double>::quiet_NaN();
    if (!spIndiv->isFeasible) {
        delete spIndiv;
        return -1;
    }

    if (data->no_capacity_location || data->numLocConfiguration == 0) {
        addIndividual(spIndiv, 0, false);
        lastSPDecision = lastAddAccepted ? "matched_inserted" : "duplicate_rejected";
        lastSPTarget = 0;
        if (lastAddAccepted)
            ++spMatchingInjections;
        delete spIndiv;
        return -1;
    }

    std::set<int> spDepots;
    long long int var1 = 0, var2 = 0;
    for (int i = 0; i < data->numL; i++) {
        if (spIndiv->location[i].used) {
            spDepots.insert(i + data->numV);
            long long int var = std::pow(2, i);
            var1 += var;
            var2 += var + 9527 * i;
        }
    }

    int matchIndex = -1;
    for (int i = 0; i < data->numLocConfiguration; ++i) {
        if (data->compactLocConfigure[i] == spDepots) {
            matchIndex = i;
            break;
        }
    }

    if (spIndiv->fit < bestIndivRestart->fit) {
        bestIndivRestart->initilization();
        copyAllData(bestIndivRestart, spIndiv);
    }
    if (spIndiv->fit < global->fit) {
        global->initilization();
        copyAllData(global, spIndiv);
        data->meetBestTime = float((double)clock() / (double)CLOCKS_PER_SEC);
    }
    if (studyIncumbent)
        studyIncumbent(global);

    if (matchIndex != -1) {
        lastSPTarget = matchIndex;
        if (study)
            lastSPTargetId = studyPopulationId(matchIndex);
        if (!popula[matchIndex].empty()) {
            lastSPTargetMean = 0;
            lastSPTargetBest = std::numeric_limits<double>::infinity();
            for (Individual *member : popula[matchIndex]) {
                lastSPTargetMean += member->fit;
                lastSPTargetBest = std::min(lastSPTargetBest, member->fit);
            }
            lastSPTargetMean /= popula[matchIndex].size();
        }

        for (Individual *existingIndiv : popula[matchIndex]) {
            if (existingIndiv->fit + Min > spIndiv->fit &&
                existingIndiv->fit - Min < spIndiv->fit) {
                std::cout << "[SP Reject] SP解与常驻种群 [" << matchIndex
                          << "] 中的现有解重复，拦截注入。" << std::endl;
                delete spIndiv;
                lastSPDecision = "duplicate_rejected";
                return -1;
            }
        }

        std::cout << "[SP Inject] 配置匹配 (Match found)! 注入常驻种群 [" << matchIndex
                  << "] (Exploitation)" << std::endl;

        linkProximities(spIndiv, popula[matchIndex]);

        auto place = popula[matchIndex].begin();
        while (place != popula[matchIndex].end() && (*place)->fit <= spIndiv->fit - Min)
            ++place;
        popula[matchIndex].insert(place, spIndiv);
        lastSPDecision = "matched_inserted";
        ++spMatchingInjections;

        if (pool != nullptr) {
            pool->recordSolutionFitness(spIndiv->fit);
        }

        if ((int)popula[matchIndex].size() > data->popMin) {
            updateBiasedFitnesses(popula[matchIndex]);
            removeWorstBiasedFitness(popula[matchIndex]);
        }
        return -1;
    } else {
        if (data->experimentMode && data->experimentArm == "NoSPInstall") {

            lastSPDecision = "disabled_install";
            delete spIndiv;
            return -1;
        }

        int worstIdx = -1;
        double worstAvgFit = -1.0;

        for (int i = 0; i < data->numLocConfiguration; ++i) {
            if (popula[i].empty()) {
                worstIdx = i;
                worstAvgFit = 1e30;
                break;
            }
            double sumFit = 0.0;
            for (Individual *ind : popula[i])
                sumFit += ind->fit;
            double avgFit = sumFit / (double)popula[i].size();

            if (avgFit > worstAvgFit) {
                worstAvgFit = avgFit;
                worstIdx = i;
            }
        }

        double bestFitInWorstPop = 1e30;
        if (worstIdx != -1 && !popula[worstIdx].empty()) {
            for (Individual *ind : popula[worstIdx]) {
                if (ind->fit < bestFitInWorstPop)
                    bestFitInWorstPop = ind->fit;
            }
        } else {
            bestFitInWorstPop = 1e30;
        }
        lastSPTarget = worstIdx;
        if (worstIdx >= 0) {
            if (study)
                lastSPTargetId = studyPopulationId(worstIdx);
            if (!popula[worstIdx].empty()) {
                lastSPTargetMean = worstAvgFit;
                lastSPTargetBest = bestFitInWorstPop;
            }
        }

        const bool fullPass = worstIdx != -1 && spIndiv->fit < bestFitInWorstPop;
        const bool globalPass = spIndiv->fit < globalBefore;
        if (study)
            study->event("admission_gate",
                         "\"candidate_id\":" + StudyEvents::quote(study->candidateId) +
                             ",\"global_before\":" + StudyEvents::number(globalBefore) +
                             ",\"full_gate_pass\":" + (fullPass ? "true" : "false") +
                             ",\"global_gate_pass\":" + (globalPass ? "true" : "false") +
                             ",\"empty_slot\":" +
                             (worstIdx >= 0 && popula[worstIdx].empty() ? "true" : "false"));
        if (fullPass && (data->experimentArm != "GlobalOnly" || globalPass)) {
            lastSPDecision = "new_installed";
            ++spDirectInstalls;
            if (study)
                replaceStudyPopulation(worstIdx, "SP_direct_install");
            std::cout << "[SP Inject] 发生范式转移! SP解成本 (" << std::fixed << spIndiv->fit
                      << ") 击败劣势种群的最优解 (" << bestFitInWorstPop << "). 强势接管槽位 ["
                      << worstIdx << "]" << std::endl;

            popCreationIteration[worstIdx] = data->iteration;

            long long int old_var1 = 0, old_var2 = 0;
            for (auto it = data->compactLocConfigure[worstIdx].begin();
                 it != data->compactLocConfigure[worstIdx].end(); ++it) {
                int old_loc = *it - data->numV;
                long long int old_var = std::pow(2, old_loc);
                old_var1 += old_var;
                old_var2 += old_var + 9527 * old_loc;
            }
            if (data->dimTabu1[old_var1 % data->dimension] == worstIdx)
                data->dimTabu1[old_var1 % data->dimension] = -1;
            if (data->dimTabu2[old_var2 % data->dimension] == worstIdx)
                data->dimTabu2[old_var2 % data->dimension] = -1;

            deleteMembers(popula[worstIdx]);
            popula[worstIdx].clear();

            data->compactLocConfigure[worstIdx] = spDepots;
            data->dimTabu1[var1 % data->dimension] = worstIdx;
            data->dimTabu2[var2 % data->dimension] = worstIdx;

            popula[worstIdx].push_back(spIndiv);

            if (pool != nullptr) {
                pool->recordSolutionFitness(spIndiv->fit);
            }

            return worstIdx;
        } else {
            lastSPDecision = fullPass       ? "global_gate_rejected"
                             : worstIdx < 0 ? "no_viable_population"
                                            : "competition_rejected";
            std::cout << "[SP Reject] 准入失败 (Admittance Rejected). SP解 (" << std::fixed
                      << spIndiv->fit << ") 未能超越劣势种群最优解门槛 (" << bestFitInWorstPop
                      << "). 抛弃该配置." << std::endl;
            delete spIndiv;
            return -1;
        }
    }
}
