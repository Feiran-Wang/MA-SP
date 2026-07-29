/******************************************************************************
 * File: RouteGenerator.cpp
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description: 
 *   Core engine for generating routes and managing heuristic operators.
 ******************************************************************************/

#include "RouteGenerator.h"
#include "basic.h"
#include "RunLogger.h"
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <limits>
#include <stdexcept>
#include <vector>

using namespace std;

namespace {

std::vector<int> extractCompleteCustomerOrder(
        const Individual& individual,
        const read_data& data) {
    std::vector<int> order;
    order.reserve(static_cast<std::size_t>(data.numV));
    std::vector<unsigned char> seen(
        static_cast<std::size_t>(data.numV), 0U);

    for (int depot = 0; depot < data.numL; ++depot) {
        for (int slot = 0;
             slot < individual.location[depot].maxVisit;
             ++slot) {
            const Node* node =
                individual.location[depot].depots[slot].next;
            int guard = 0;
            while (node != nullptr && !node->isDepot &&
                   guard++ <= data.numV) {
                if (node->cour >= 0 && node->cour < data.numV &&
                    seen[static_cast<std::size_t>(node->cour)] == 0U) {
                    seen[static_cast<std::size_t>(node->cour)] = 1U;
                    order.push_back(node->cour);
                }
                node = node->next;
            }
        }
    }

    // Crossover is expected to preserve coverage.  Appending any omitted
    // customer makes the repair boundary total and avoids propagating a
    // partially linked offspring into the TW constructor.
    std::vector<int> missing;
    for (int customer = 0; customer < data.numV; ++customer) {
        if (seen[static_cast<std::size_t>(customer)] == 0U)
            missing.push_back(customer);
    }
    RandomShuffle(missing);
    order.insert(order.end(), missing.begin(), missing.end());
    return order;
}

}  // namespace

RouteGenerator::RouteGenerator(read_data * data) {
    this->data=data;
    
    // Internalize creation of all operators
    this->popman = std::unique_ptr<popManager>(new popManager(data));
    this->cross = std::unique_ptr<crossover>(new crossover(data, popman.get()));
    this->repair = std::unique_ptr<RepairProcedure>(new RepairProcedure(data, popman.get()));
    this->ls = std::unique_ptr<LS>(new LS(data, repair.get()));
    this->initialSol = std::unique_ptr<initial_sol>(new initial_sol(data, repair.get()));
    this->mutationOperator = std::unique_ptr<mutation>(new mutation(data));
    hgasp::AdaptiveSearchPolicy allocationPolicy;
    allocationPolicy.exploration =
        data->adaptivePopulationExploration;
    allocationPolicy.diversityWeight =
        data->adaptivePopulationDiversityWeight;
    allocationPolicy.columnWeight =
        data->adaptivePopulationColumnWeight;
    allocationPolicy.emaWeight =
        data->adaptiveSearchEmaWeight;
    allocationPolicy.mutationMinimumPercent =
        data->adaptiveMutationMinimumPercent;
    allocationPolicy.mutationMaximumPercent =
        data->adaptiveMutationMaximumPercent;
    searchAllocator.configure(allocationPolicy);
    currentMutationProbabilityPercent =
        data->mutationProbabilityPercent;
    
    temIndiv = std::unique_ptr<Individual>(new Individual());
    temIndiv->define(data);
    global = std::unique_ptr<Individual>(new Individual());
    global->define(data);
    global->initilization();
    loc_best = std::unique_ptr<Individual>(new Individual());
    loc_best->define(data);
    loc_best->initilization();
    bestIndivRestart = std::unique_ptr<Individual>(new Individual());
    bestIndivRestart->define(data);
    bestIndivRestart->initilization();

    popman->global = global.get();
    popman->bestIndivRestart = bestIndivRestart.get();
    int sizeSubPop = 100;
    subPop = std::unique_ptr<Individual[]>(new Individual[sizeSubPop]);
    for (int i = 0; i < sizeSubPop; i++){
        subPop[i].define(data);
        subPop[i].initilization();
    }
    global->fit = Max;
    bestIndivRestart->fit = Max;
    fa = NULL;
    ma = NULL;
}

RouteGenerator::~RouteGenerator(){
}

void RouteGenerator::preprocessingLocationConfiguration(){
    data->compactLocConfigure.clear();
    int initalNumDepots = std::min(500, (int)data->LocConfigure.size());      // the key parameters// 100
    if ((int)data->LocConfigure.size() < initalNumDepots)
        initalNumDepots = (int)data->LocConfigure.size();
    for (int i=0;i<initalNumDepots; i++)
        data->compactLocConfigure.push_back(data->LocConfigure[i].second);

    if (data->no_capacity_location){
        data->numLocConfiguration = 0;
        return;// if no depot configurations, jump
    }

    std::vector<std::pair<double, int>>averageCost;
    double avgCost,bestCost = Max;
    double global_cost = Max;
    bool deadlineReached = false;
    for (int i = 0; i<initalNumDepots;i++){// an extra subPop
        avgCost = 0;bestCost = Max;
        int successfulSamples = 0;
        for (int j = 0; j < numSubPopPara; j++){
            if (!initialSol->initial_solution(temIndiv.get(), i)) {
                if (global_cost < Max && timeLimitReached())
                    deadlineReached = true;
                if (deadlineReached)
                    break;
                continue;
            }
            temIndiv->decideFeasible();
            if (!temIndiv->isFeasible)
                continue;
            // Once the construction itself has consumed the budget, retain
            // that feasible solution instead of spending additional time in
            // local search before the first incumbent can be published.
            if (!timeLimitReached()) {
                ls->local_search_run(
                    temIndiv.get(),
                    data->penaltyDepot,
                    data->penaltyVeh);
                temIndiv->decideFeasible();
                if (!temIndiv->isFeasible)
                    continue;
            }
            ++successfulSamples;
            avgCost += temIndiv->fit;
            if (bestCost > temIndiv->fit)
                bestCost = temIndiv->fit;
            if (temIndiv->fit < global_cost){
                // to record the best solution given each depot configuration
                global_cost = temIndiv->fit;
                loc_best->initilization();
                popman->copyAllData(loc_best.get(), temIndiv.get());// record the best solution given the current depot configuration.
            }
            if (timeLimitReached()) {
                deadlineReached = true;
                break;
            }
        }
        if (successfulSamples > 0) {
            avgCost /= static_cast<double>(successfulSamples);
            averageCost.push_back({bestCost, i});
        }
        std::cout<<i<<"   tries   "<< bestCost<<std::endl;
        if (deadlineReached)
            break;
    }
    if (averageCost.empty()) {
        throw std::runtime_error(
            "failed to construct a feasible solution while filtering depot "
            "configurations");
    }
    std::sort(averageCost.begin(), averageCost.end());
    loc_best->evaluationDis();

    data->numLocConfiguration =
        std::min(numSubPopPara, static_cast<int>(averageCost.size()));//5
    data->compactLocConfigure.clear();
    for (int i=0;i < data->numLocConfiguration; i++)
        data->compactLocConfigure.push_back(data->LocConfigure[averageCost[i].second].second);
    
    popman->addIndividual(loc_best.get(), data->numLocConfiguration, true);// insert the local best solution to elite pop
    std::cout<<"Finishing initial filter"<<std::endl;
}

void RouteGenerator::getAverageValue(int rank){
    if (rank < 0){
        avgCostPop.clear();
        for (int i=0;i<data->numLocConfiguration + 1; i++){
            double cost = 0;
            for (int j=0;j<(int)popman->popula[i].size();j++){
                cost += popman->popula[i][j]->fit;
            }
            int ss = popman->popula[i].size();
            avgCostPop.push_back(cost / (double)ss);
        }
        
        // removing half population
        std::vector<std::pair<double,int>>rankOrder;
        for (int i=0;i<(int)avgCostPop.size();i++)
            rankOrder.push_back({-avgCostPop[i],i});
        sort(rankOrder.begin(),rankOrder.end());
        for (int i=0;i<data->numLocConfiguration / 2; i++){
            int subpop = rankOrder[i].second;
            for (Individual * indiv : popman->popula[subpop]) delete indiv ;
            popman->popula[subpop].clear();
        }
        
        long long int var, var1, var2; int seed;
        for (int i=0;i<data->numLocConfiguration;i++){
            if ((int)popman->popula[i].size() > 0){
                var = 0;var1 = 0;
                auto it = data->compactLocConfigure[i].begin();
                for (;it != data->compactLocConfigure[i].end(); it++){
                    seed = *it - data->numV;
                    var2 = 1ULL << seed;
                    var += var2;
                    var1 += var2 + 9527 * seed;
                }
                data->dimTabu1[var % data->dimension] = i;
                data->dimTabu2[var1 % data->dimension] = i;
            }
        }
    }
    else{
        double cost = 0;
        for (int j=0;j<(int)popman->popula[rank].size();j++)
            cost += popman->popula[rank][j]->fit;
        int ss = popman->popula[rank].size();
        avgCostPop[rank] = cost / (double)ss;
    }
}

int RouteGenerator::selectsubPop(int subpop1){
    int rand1,rand2;
    int target;
    std::vector<int>locConf;
    
    for (int i = 0; i < data->numLocConfiguration + 1; i++) {
        if ((int)popman->popula[i].size() > 0)
            locConf.push_back(i);
    }

    if (locConf.empty()) return -1;
    if ((int)locConf.size() == 1) {
        if (subpop1 < 0) {
            const double diversity =
                populationDiversity(locConf[0]);
            const double stagnationFraction =
                data->maxIterNonProd > 0
                    ? static_cast<double>(
                          data->nbIterNonImpro) /
                          static_cast<double>(
                              data->maxIterNonProd)
                    : 0.0;
            currentMutationProbabilityPercent =
                data->adaptiveSearchAllocationEnabled
                    ? searchAllocator.mutationProbability(
                          data->mutationProbabilityPercent,
                          diversity,
                          stagnationFraction,
                          spJustImproved)
                    : data->mutationProbabilityPercent;
        }
        return locConf[0];
    }

    if (data->adaptiveSearchAllocationEnabled) {
        std::vector<double> diversity(
            popman->popula.size(), 0.0);
        for (const int index : locConf) {
            diversity[static_cast<std::size_t>(index)] =
                populationDiversity(index);
        }
        std::vector<int> adaptiveEligible = locConf;
        if (subpop1 >= 0 && adaptiveEligible.size() > 1U) {
            adaptiveEligible.erase(
                std::remove(
                    adaptiveEligible.begin(),
                    adaptiveEligible.end(),
                    subpop1),
                adaptiveEligible.end());
        }
        int selected =
            searchAllocator.selectPopulation(
                adaptiveEligible, diversity);
        if (subpop1 < 0) {
            const double stagnationFraction =
                data->maxIterNonProd > 0
                    ? static_cast<double>(
                          data->nbIterNonImpro) /
                          static_cast<double>(
                              data->maxIterNonProd)
                    : 0.0;
            currentMutationProbabilityPercent =
                searchAllocator.mutationProbability(
                    data->mutationProbabilityPercent,
                    diversity[
                        static_cast<std::size_t>(selected)],
                    stagnationFraction,
                    spJustImproved);
            if (data->iteration % 1000 == 0) {
                std::cout
                    << "[HGA Adaptive] population="
                    << selected
                    << " diversity="
                    << diversity[
                           static_cast<std::size_t>(
                               selected)]
                    << " improvement_cpu_ema="
                    << searchAllocator
                           .populationImprovementEma(
                               selected)
                    << " mutation_probability="
                    << currentMutationProbabilityPercent
                    << std::endl;
            }
        }
        return selected;
    }

    if ((int)locConf.size() == 2) {
        return (locConf[0] == subpop1) ? locConf[1] : locConf[0]; 
    }

    constexpr int maximumTournamentAttempts = 64;
    for (int attempt = 0;
         attempt < maximumTournamentAttempts;
         ++attempt) {
        rand1 = RandomInt(0, locConf.size() - 1);
        rand2 = RandomInt(0, locConf.size() - 1);

        if ((int)locConf.size() <= 3) return locConf[rand1];

        while(rand1 == rand2)
            rand2 = RandomInt(0, locConf.size() - 1);

        if (avgCostPop[locConf[rand1]] < avgCostPop[locConf[rand2]])
            target = locConf[rand1];
        else
            target = locConf[rand2];

        if (target == subpop1 && (int)popman->popula[target].size() > 2) break;
        if (target != subpop1) break;
        if (attempt + 1 == maximumTournamentAttempts) {
            std::vector<std::size_t> populationSizes(
                popman->popula.size(), 0U);
            for (std::size_t index = 0;
                 index < popman->popula.size();
                 ++index) {
                populationSizes[index] =
                    popman->popula[index].size();
            }
            target = deterministicSubpopulationFallback(
                locConf, subpop1, populationSizes);
        }
    }
    return target;
}

int RouteGenerator::deterministicSubpopulationFallback(
        const std::vector<int>& eligible,
        const int excludedPopulation,
        const std::vector<std::size_t>& populationSizes) {
    if (eligible.empty()) {
        return -1;
    }
    for (const int candidate : eligible) {
        if (candidate != excludedPopulation) {
            return candidate;
        }
    }
    if (excludedPopulation >= 0 &&
        static_cast<std::size_t>(excludedPopulation) <
            populationSizes.size() &&
        populationSizes[
            static_cast<std::size_t>(excludedPopulation)] > 2U) {
        return excludedPopulation;
    }
    return eligible.front();
}

void RouteGenerator::evolvePop(int rankPop, int &count, RoutePool* pool){
    if ((int)popman->popula[rankPop].size() <= 2){
        count = data->maxIterationPerLoc;
        return;
    }
    int sizeSubPop = 0;
    ma = NULL;
    fa = popman->selectionParentSolution(ma, rankPop);
    ma = popman->selectionParentSolution(fa, rankPop);
    if (!cross->gEAX(fa,ma,subPop.get(),sizeSubPop) ||
        sizeSubPop <= 0) {
        // A degenerate parent pair is still one diversification attempt.
        // Advancing the counter prevents the initialization loop from
        // spinning forever when EAX has no admissible cycle set.
        ++count;
        return;
    }
    for (int i =0; i<sizeSubPop; i++){
        if (refineChild(i)){
            popman->addIndividual(&subPop[i], rankPop, true, pool);
        }
        count++;
        if (initializationTimeLimitReached())
            return;
    }
}

void RouteGenerator::initialPop(RoutePool* pool){
    popman->popula.erase(popman->popula.begin()+data->numLocConfiguration +1, popman->popula.end());
    int i;
	    for ( i = 0; i<data->numLocConfiguration +1 ;i++){// an extra subPop
	        std::cout<<i<<" th ";
	        std::cout<<data->elapsedCpuSeconds()<<std::endl;
        
        double bestFit=Max;
        for (int j = 0; j < data->popMin*4; j++){
            if (!initialSol->initial_solution(temIndiv.get(), i)) {
                if (initializationTimeLimitReached())
                    return;
                continue;
            }
            temIndiv->decideFeasible();
            if (!temIndiv->isFeasible)
                continue;
            if (!timeLimitReached()) {
                ls->local_search_run(
                    temIndiv.get(),
                    data->penaltyDepot,
                    data->penaltyVeh);
            }
            popman->addIndividual(temIndiv.get(), i, true, pool);
            if (bestFit>temIndiv->fit)
                bestFit=temIndiv->fit;
            if (initializationTimeLimitReached())
                return;
        }
        if (popman->popula[i].empty()) {
            throw std::runtime_error(
                "failed to construct any feasible initial solution for "
                "population " + std::to_string(i));
        }
	        std::cout<<"The best fit is    "<<bestFit
	                 <<"  The CPU time consuming  "
	                 <<data->elapsedCpuSeconds()<<std::endl;
        
        int count = 0;
        if ((int)popman->popula[i].size() <= 1)
            continue;
        while (count < data->maxIterationPerLoc) {
            evolvePop(i, count, pool);
            if (initializationTimeLimitReached())
                return;
        }
    }
    std::cout<<"Finishing constructing number of initial population"<<std::endl;
}

int RouteGenerator::generateEAXChildren(int subPop1, int subPop2) {
    int sizeSubPop = 0;
    ma = NULL;
    fa = popman->selectionParentSolution(ma, subPop1);
    ma = popman->selectionParentSolution(fa, subPop2);
    if (!cross->gEAX(fa,ma,subPop.get(),sizeSubPop))
        return 0;
    return sizeSubPop;
}

bool RouteGenerator::refineChild(int childIndex) {
    const int mutationProbability =
        data->adaptiveSearchAllocationEnabled
            ? currentMutationProbabilityPercent
            : data->mutationProbabilityPercent;
    if (data->problemMode == hgasp::ProblemMode::ClrpTw) {
        const std::vector<int> order =
            extractCompleteCustomerOrder(subPop[childIndex], *data);
        if (!initialSol->rebuildTimeWindowFeasible(
                &subPop[childIndex], order, childIndex)) {
            // An order for which the deterministic TW constructor cannot
            // build a complete solution is an expected rejected offspring.
            return false;
        }
        // CLRP-TW uses a detached, Shaw-style ruin/recreate mutation.
        // A false result means only that no distinct feasible reconstruction
        // was found. Validation, commit, and other programming errors are
        // deliberately allowed to propagate and abort the experiment.
        if (RandomInt(0, 99) < mutationProbability) {
            const double beforeMutation =
                subPop[childIndex].fit;
            const double mutationCpuStarted =
                data->elapsedCpuSeconds();
            const bool committed =
                mutationOperator->mutationRun(
                    &subPop[childIndex]);
            searchAllocator.reportMutation(
                committed,
                committed
                    ? std::max(
                          0.0,
                          beforeMutation -
                              subPop[childIndex].fit)
                    : 0.0,
                data->elapsedCpuSeconds() -
                    mutationCpuStarted);
        }
        ls->local_search_run(
            &subPop[childIndex],
            data->penaltyDepot,
            data->penaltyVeh);
        subPop[childIndex].decideFeasible();
        if (!subPop[childIndex].isFeasible) {
            throw std::logic_error(
                "CLRP-TW offspring became infeasible after mutation/local "
                "search");
        }
        return true;
    }

    repair->repairCapacityConstraint(&subPop[childIndex], data->penaltyDepot, data->penaltyVeh);
    if (subPop[childIndex].isFeasible){
        if (RandomInt(0, 99) < mutationProbability) {
            const double beforeMutation =
                subPop[childIndex].fit;
            const double mutationCpuStarted =
                data->elapsedCpuSeconds();
            const bool committed =
                mutationOperator->mutationRun(
                    &subPop[childIndex]);
            searchAllocator.reportMutation(
                committed,
                committed
                    ? std::max(
                          0.0,
                          beforeMutation -
                              subPop[childIndex].fit)
                    : 0.0,
                data->elapsedCpuSeconds() -
                    mutationCpuStarted);
        }
        ls->local_search_run(&subPop[childIndex], data->penaltyDepot, data->penaltyVeh);
        return true;
    }
    return false;
}

void RouteGenerator::integrateToPopulation(int childIndex, RoutePool* pool) {
    std::vector<int> depot;
    long long int var1, var2, var;
    int whichSubPop;
    int isGenerateNewSubPop;

    if (!data->no_capacity_location){
        depot.clear();
        for (int j=0; j<data->numL;j++){
            if (subPop[childIndex].location[j].used)
                depot.push_back(j);
        }
        var1 = 0; var2 = 0;
        for (int j=0;j<(int)depot.size();j++){
            var = 1ULL << depot[j];
            var1 += var;
            var2 += var + 9527 * depot[j];
        }
        auto it1 = data->dimTabu1.find(var1 % data->dimension);
        auto it2 = data->dimTabu2.find(var2 % data->dimension);
        if (it1 != data->dimTabu1.end() && it2 != data->dimTabu2.end() && it1->second == it2->second && it2->second > -1)
            whichSubPop = it2->second;
        else
            whichSubPop = data->numLocConfiguration;
        
        isGenerateNewSubPop = popman->addIndividual(&subPop[childIndex], whichSubPop, false, pool);
        getAverageValue(whichSubPop);
        
        if (isGenerateNewSubPop > -1){
            searchAllocator.resetPopulation(
                isGenerateNewSubPop);
            incubatePopulation(
                isGenerateNewSubPop, pool, "hga");
            registerPopulationConfiguration(
                isGenerateNewSubPop);
        }
    }
    else {
        popman->addIndividual(&subPop[childIndex], 0, false, pool);
    }
}

SpInjectionResult RouteGenerator::injectIncumbent(
        std::unique_ptr<Individual> spIndiv,
        RoutePool* pool) {
    const SpInjectionResult injection =
        popman->injectSPSolution(std::move(spIndiv), pool);

    // SP insertion can change an existing resident population without
    // creating a new configuration. Refresh every non-empty cached mean so
    // subsequent subpopulation tournaments observe the injected solution.
    for (int rank = 0; rank < data->numLocConfiguration + 1; ++rank) {
        if (!popman->popula[rank].empty()) {
            getAverageValue(rank);
        }
    }

    if (injection.createdPopulation()) {
        const int spNewPopIdx = injection.populationIndex;
        searchAllocator.resetPopulation(spNewPopIdx);
        incubatePopulation(spNewPopIdx, pool, "sp");
        registerPopulationConfiguration(spNewPopIdx);
    }
    return injection;
}

void RouteGenerator::incubatePopulation(
        const int populationIndex,
        RoutePool* pool,
        const char* source) {
    if (populationIndex < 0 ||
        static_cast<std::size_t>(populationIndex) >=
            popman->popula.size()) {
        throw std::out_of_range(
            "population incubation index is out of range");
    }
    SubPopulation& population =
        popman->popula[static_cast<std::size_t>(populationIndex)];
    if (population.empty()) {
        throw std::runtime_error(
            "population incubation has no feasible seed elite");
    }
    const std::size_t target =
        static_cast<std::size_t>(std::max(1, data->popMin));
    hgasp::PopulationIncubationGuard guard(
        target,
        population.size(),
        std::max<std::size_t>(100U, target * 50U),
        std::max<std::size_t>(50U, target * 10U));

    while (guard.beginAttempt(
            population.size(),
            timeLimitReached() ||
                (data->iterLimit > 0 &&
                 data->iteration >= data->iterLimit))) {
        if (initialSol->initial_solution(
                temIndiv.get(), populationIndex)) {
            ls->local_search_run(
                temIndiv.get(),
                data->penaltyDepot,
                data->penaltyVeh);
            temIndiv->decideFeasible();
            if (temIndiv->isFeasible) {
                popman->addIndividual(
                    temIndiv.get(),
                    populationIndex,
                    false,
                    pool);
            }
        }
        guard.observe(population.size());
    }

    getAverageValue(populationIndex);
    if (population.size() < target) {
        std::cout
            << "[Population Incubation] source=" << source
            << " population=" << populationIndex
            << " status=partial"
            << " size=" << population.size()
            << " target=" << target
            << " attempts=" << guard.attempts()
            << " no_progress=" << guard.noProgressAttempts()
            << " external_stop="
            << (guard.stoppedByExternalRequest()
                    ? "true"
                    : "false")
            << std::endl;
        hgasp::RunLogger::event(
            hgasp::LogLevel::warning,
            "population",
            "incubation_partial",
            {},
            {
                {"iteration", data->iteration},
                {"source", source},
                {"population", populationIndex},
                {"size", static_cast<unsigned long long>(
                    population.size())},
                {"target", static_cast<unsigned long long>(target)},
                {"attempts", static_cast<unsigned long long>(
                    guard.attempts())},
                {"no_progress", static_cast<unsigned long long>(
                    guard.noProgressAttempts())},
            });
    }
}

void RouteGenerator::registerPopulationConfiguration(
        const int populationIndex) {
    long long int key1 = 0;
    long long int key2 = 0;
    for (const int encodedDepot :
         data->compactLocConfigure[populationIndex]) {
        const int depot = encodedDepot - data->numV;
        const long long int bit = 1ULL << depot;
        key1 += bit;
        key2 += bit + 9527 * depot;
    }
    data->dimTabu1[key1 % data->dimension] =
        populationIndex;
    data->dimTabu2[key2 % data->dimension] =
        populationIndex;
}

void RouteGenerator::checkStagnationAndRestart(RoutePool* pool) {
    if (data->nbIterNonImpro > data->maxIterNonProd){
        const double elapsedTime = data->elapsedCpuSeconds();
        const double remainingTime =
            data->timeLimit > 0
                ? data->timeLimit - elapsedTime
                : std::numeric_limits<double>::infinity();
        int remainingIters = data->iterLimit - data->iteration;

        if (remainingIters < 50000 || remainingTime < 600.0) {
            std::cout << "  [Intercepted] Restart aborted. Remaining Iters: " << remainingIters 
                        << ", Remaining Time: " << remainingTime << "s. Focused search continued." << std::endl;
            data->nbIterNonImpro = 0; 
        } else {
            std::cout << "  [Triggered] Executing Global Restart..." << std::endl;
            searchAllocator.resetAllPopulations();
            popman->reStart();
            initialPop(pool);
            
            std::cout << "[RoutePool] 正在建立初始精英池基线..." << std::endl;
            for (int i = 0; i < data->numLocConfiguration + 1; i++) {
                for (Individual* indiv : popman->popula[i]) {
                    if (indiv->isFeasible) {
                        pool->extractRoutes(indiv);
                    }
                }
            }
            getAverageValue(-1);
        }
    }
}

void RouteGenerator::checkSubPopReduction() {
    if (data->iteration == data->iterLimit/2)
        popman->reduceHalfSubPop();

    if (data->iteration == data->iterLimit * 0.75)
        popman->reduceHalfSubPop();
}

bool RouteGenerator::checkStopCondition() {
    if (data->stopCondition(global->fit)){
        global->isRight();
        global->outputBestSolution();
        std::cout<<global->fit<<std::endl;
        std::cout<<"----The memetic algorithm is finished, CPU time spends: "
                 <<data->elapsedCpuSeconds()<<std::endl;
        return true;
    }
    return false;
}

bool RouteGenerator::hasFeasibleIncumbent() const noexcept {
    return global != nullptr &&
           global->isFeasible &&
           global->fit < Max;
}

const Individual* RouteGenerator::globalIncumbent() const noexcept {
    return hasFeasibleIncumbent() ? global.get() : nullptr;
}

void RouteGenerator::updateDualGuidance(
        const std::vector<double>& customerDuals) {
    mutationOperator->setCustomerDualGuidance(customerDuals);
}

void RouteGenerator::reportSearchOutcome(
        const int firstPopulationIndex,
        const int secondPopulationIndex,
        const double objectiveImprovement,
        const double cpuSeconds,
        const std::size_t newColumns) {
    if (!data->adaptiveSearchAllocationEnabled) {
        return;
    }
    hgasp::SearchArmObservation observation;
    observation.objectiveImprovement =
        std::max(0.0, objectiveImprovement);
    observation.cpuSeconds = std::max(0.0, cpuSeconds);
    observation.newColumns =
        static_cast<double>(newColumns);
    searchAllocator.reportSharedPopulations(
        firstPopulationIndex,
        secondPopulationIndex,
        observation);
    spJustImproved = false;
}

void RouteGenerator::reportSpImprovement(
        const bool improved) {
    if (improved) {
        spJustImproved = true;
    }
}

double RouteGenerator::populationDiversity(
        const int populationIndex) const {
    if (populationIndex < 0 ||
        static_cast<std::size_t>(populationIndex) >=
            popman->popula.size()) {
        return 0.0;
    }
    const SubPopulation& population =
        popman->popula[
            static_cast<std::size_t>(populationIndex)];
    if (population.size() <= 1U) {
        return 0.0;
    }
    double total = 0.0;
    for (Individual* individual : population) {
        total += individual->averageBrokenPairsDistanceClosest(
            std::min(
                data->nbClost,
                static_cast<int>(population.size()) - 1));
    }
    // The inherited HGAMP implementation stores broken-pairs proximity as a
    // non-positive score: structurally different solutions have a larger
    // negative magnitude. Convert that convention to a [0,1] diversity
    // signal before feeding the allocation controller.
    const double normalizer =
        static_cast<double>(
            std::max(1, 20 * data->numV));
    return std::clamp(
        -total /
            static_cast<double>(population.size()) /
            normalizer,
        0.0,
        1.0);
}

bool RouteGenerator::timeLimitReached() const noexcept {
    if (data->timeLimit <= 0)
        return false;
    const double elapsedTime = data->elapsedCpuSeconds();
    return elapsedTime >= data->timeLimit;
}

bool RouteGenerator::initializationTimeLimitReached() const noexcept {
    return hasFeasibleIncumbent() && timeLimitReached();
}

void RouteGenerator::outputprocess(Individual *indiv){
    std::cout<<"It: "<<data->iteration<<"--G-Fit: "<<std::setiosflags(ios::fixed)<<std::setprecision(2)<<float(global->fit);
    std::cout<<"  --R-Fit: "<<bestIndivRestart->fit;
    std::cout<<"  Curr Sol--"<<float(indiv->fit)<<std::setprecision(2);
    std::cout<<"  IT NoImpro--"<<data->nbIterNonImpro;
    std::cout<<"  Time comsuming--"
             <<float(data->elapsedCpuSeconds())<<std::setprecision(2);
    std::cout<<std::endl;
    data->loopp = 0;
}

void RouteGenerator::printProgress(int childIndex){
    if (data->iteration % 1000 == 0) {
        outputprocess(&subPop[childIndex]);
        hgasp::RunLogger::event(
            hgasp::LogLevel::info,
            "search",
            "progress",
            {},
            {
                {"iteration", data->iteration},
                {"global_objective", global->fit},
                {"restart_objective",
                    bestIndivRestart->fit},
                {"non_improving", data->nbIterNonImpro},
                {"cpu_seconds", data->elapsedCpuSeconds()},
            });
        hgasp::RunLogger::status(
            "search",
            "running",
            {
                {"iteration", data->iteration},
                {"global_objective", global->fit},
                {"cpu_seconds", data->elapsedCpuSeconds()},
            });
    }
    
    if (data->iteration > 0 && data->iteration % 50000 == 0) {
        std::cout << ">>> [机制监控] \"gamma+1\" 游离种群动态逆袭替换机制累计生效: " 
                    << popman->reverseReplacementCount << " 次 | "
                    << "当前游离种群个体数量: " 
                    << popman->popula[data->numLocConfiguration].size() << std::endl;
    }
}
