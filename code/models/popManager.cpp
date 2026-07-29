/******************************************************************************
 * File: popManager.cpp
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description: 
 *   Manages the population of individuals, sub-populations, and diversity logic.
 ******************************************************************************/

#include "popManager.h"
#include "RoutePool.h"
#include <cmath>
#include <stdexcept>

popManager::popManager(read_data *data) {
	this->data=data;
	this->numV = data->numV;
	this->numL = data->numL;
	this->numC = data->numC;
	order = new int [numV];
	popula = std::vector <std::vector<Individual*>>(100);
	reverseReplacementCount = 0; // 初始化计数器
    popCreationIteration = std::vector<long long int>(100, 0); // 初始化种群代数记录器，默认容量为 100 
}

popManager::~popManager() {
	for (int i = 0; i < (int)feasibleSubpopulation.size(); i++)
		delete feasibleSubpopulation[i];
	delete [] order;
	for (int i=0;i<data->numLocConfiguration+1;i++){
		for (Individual * indiv : popula[i]) delete indiv;
		popula[i].clear();
	}
}
void popManager:: copyAllData(Individual * s,Individual *ss){
	Node *nodeU;
	s->saturatLoc.clear();
	s->saturatRoute.clear();
	/*initialize solution s*/
	for (int i=0;i<numV; i++){
		s->client[i].pre = NULL;
		s->client[i].next = NULL;
		s->client[i].route = NULL;
	}
	for (int i=0;i<numL;i++){
		s->location[i].emptyRoutes.clear();
		s->location[i].used = false;
		s->location[i].splitTime = 0;
		s->location[i].remainInven = 0;
		for (int j=0;j<s->location[i].maxVisit;j++){
			s->location[i].depots[j].next = &s->location[i].endDepots[j];
			s->location[i].endDepots[j].pre = &s->location[i].depots[j];
			s->location[i].depots[j].route->used = false;
			s->updateRouteInfor(s->location[i].depots[j].route->cour, -1);
		}
		s->location[i].emptyRoutes.clear();
	}
	//
	int count;
	Route * routeU;
	for (int i=numV; i<numC;i++){
		if (!ss->location[i-numV].used)continue;
		for (int j=0;j<ss->location[i-numV].maxVisit;j++){
			if (ss->location[i-numV].depots[j].route->used){
				count = 0;
				nodeU = ss->location[i-numV].depots[j].next;
				while(!nodeU->isDepot){
					order[count] = nodeU->cour;
					count++;
					nodeU=nodeU->next;
				}
				s->location[i-numV].depots[j].route->used = true;
				routeU = s->location[i-numV].depots[j].route;
				nodeU = &s->client[order[0]];
				nodeU->pre = routeU->depot;
				nodeU->route = routeU;
				routeU->depot->next = nodeU;
				for (int k=1; k<count; k++){
					s->client[order[k]].pre = nodeU;
					nodeU->next = &s->client[order[k]];
					s->client[order[k]].route = routeU;
					nodeU = &s->client[order[k]];
				}
				nodeU->next = routeU->endDepot;
				routeU->endDepot->pre = nodeU;
				s->updateRouteInfor(routeU->cour,-1);
			}
			else{
				s->location[i-numV].depots[j].next = &s->location[i-numV].endDepots[j];
				s->location[i-numV].endDepots[j].pre = &s->location[i-numV].depots[j];
				s->updateRouteInfor(s->location[i-numV].depots[j].route->cour,-1);
			}
		}
		s->updateLocationInfor(i);
	}
	s->dis=ss->dis;
	s->fit=ss->fit;
	s->biasedFit=ss->biasedFit;
	// Feasibility is a property of the rebuilt topology, not a transferable
	// flag.  In CLRP-TW this independently rechecks every hard time window.
	s->decideFeasible();
}
int popManager::addIndividual(Individual *indiv, int rankPop, bool isInitial, RoutePool* pool){
	if (indiv == nullptr)
		return -1;
	indiv->decideFeasible();
	// Population membership is a trust boundary.  Never clone or export an
	// individual that fails the current problem mode's feasibility policy.
	if (!indiv->isFeasible)
		return -1;
	// Route contribution is deliberately independent from population
	// admission. In particular, an equal-cost/structural duplicate or a
	// candidate that later loses a replacement decision can still contribute
	// new master columns.
	if (pool != nullptr)
		pool->submitFeasibleIndividual(indiv);
	if (isInitial){
		std::vector<int>depot;
		if (rankPop > -1 && rankPop < data->numLocConfiguration){
			for (int i=0;i<numL;i++){
				if (indiv->location[i].used)
					depot.push_back(i+numV);
			}
			auto it = data->compactLocConfigure[rankPop].begin();
			if ((int)depot.size() == (int)data->compactLocConfigure[rankPop].size()){
				for (int i=0;i<(int)depot.size();i++){
					if (*it != depot[i]){
						rankPop = data->numLocConfiguration;
					//	addInElitePop(indiv);
						break;
					}
					it ++;
				}
			}
		}
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	SubPopulation & subpop = popula[rankPop];
	//
	for (Individual * myIndividual : subpop)// no structural clones
		if (data->equalTwoVar(myIndividual->fit, indiv->fit) &&
			myIndividual->hasSameRouteStructure(*indiv))
			return -1;
	//
	Individual * myIndividual = new Individual();
	myIndividual->define(data);
	myIndividual->initilization();
	copyAllData(myIndividual, indiv);
	for (Individual * myIndividual2 : subpop){
		double myDistance = myIndividual->brokenPairsDistance(myIndividual2);
		myIndividual2->indivsPerProximity.insert({ myDistance, myIndividual });
		myIndividual->indivsPerProximity.insert({ myDistance, myIndividual2 });
	}

	int place = (int)subpop.size();
	while (place > 0 && subpop[place - 1]->fit > indiv->fit - Min) place--;
	subpop.emplace(subpop.begin() + place, myIndividual);
	//
	std::vector<int>depot;
	bool isObtainNewPop = false;int index = -1;

	if (indiv->fit + Min < bestIndivRestart->fit){

		indiv->isRight();
		bestIndivRestart->initilization();
		copyAllData(bestIndivRestart, indiv);
		data->nbIterNonImpro=0;
		if (indiv->fit + Min < global->fit){
			global->initilization();
			copyAllData(global, indiv);
				data->recordGlobalIncumbent();
		}
		//to replace a subPop if the new global solution's not belong to any subpop.
		if (!data->no_capacity_location){
			depot.clear();
			for (int i=0;i<numL;i++){
				if (global->location[i].used)
					depot.push_back(i+numV);
			}
			if (rankPop == data->numLocConfiguration && !isInitial){

				std::vector<std::pair<double, int>>avgCostPop;
				for (int i=0;i<data->numLocConfiguration; i++){
					if((int)popula[i].size() == 0)continue;
					double cost = 0;int count = 0;
					SubPopulation & sub = popula[i];
					for (Individual * myIndiv : sub){
						cost += myIndiv->fit;
						count ++;
					}
					if (count > 0)
						avgCostPop.push_back({cost / (double)count, i});
				}
				if ((int)avgCostPop.size() == 0)
					index = 0;
				else{
					double max_avg = 0;
					for (int i=0;i <(int)avgCostPop.size(); i++){
						if (max_avg < avgCostPop[i].first){
							max_avg = avgCostPop[i].first;
							index = avgCostPop[i].second;
						}
					}
				}
				SubPopulation & sub = popula[index];
				for (Individual * indiv : sub) delete indiv;
				sub.clear();
				data->compactLocConfigure[index].clear();
				for (int i=0;i<(int)depot.size();i++)
					data->compactLocConfigure[index].insert(depot[i]);
				Individual * myIndivi = new Individual();
				myIndivi->define(data);
				myIndivi->initilization();
				copyAllData(myIndivi, myIndividual);

				sub.emplace(sub.begin(),myIndivi);
                // Export the seed routes of the newly established population.
                if (pool != nullptr && myIndivi->isFeasible) {
                    pool->extractRoutes(myIndivi);
                }
				// generate the initial pop for the promising depot configuration
				isObtainNewPop = true;
				reverseReplacementCount++;

				// Start the protection period for the new resident population.
				popCreationIteration[index] = data->iteration;
			}
		}
		////////////////////////////////////////////
	}
	// A full free population may occupy an empty slot or challenge the
	// weakest resident population.
	if ((int)subpop.size() == data->popMax) {
		
		// Replacement applies only to the free population after initialization.
		if (rankPop == data->numLocConfiguration && !isInitial) {
			
			// Find the best free-population individual.
			Individual* best_free = subpop[0];
			for (Individual* ind : subpop) {
				if (ind->fit < best_free->fit) best_free = ind;
			}

			// Prefer an empty slot; otherwise consider the worst unprotected
			// resident population.
			double max_avg_cost = -1.0;
			int worst_res_idx = -1;
			bool found_empty_slot = false;

			for (int i = 0; i < data->numLocConfiguration; i++) {
				if (popula[i].empty()) {
					worst_res_idx = i;
					found_empty_slot = true;
					break;
				}
				
                // Do not replace a population during its protection period.
                if ((data->iteration - popCreationIteration[i]) < data->popProtectionPeriod) {
                    continue;
                }

				double current_avg = 0.0;
				for (Individual* ind : popula[i]) current_avg += ind->fit;
				current_avg /= (double)popula[i].size();
				
				if (current_avg > max_avg_cost) {
					max_avg_cost = current_avg;
					worst_res_idx = i;
				}
			}

			// An empty slot is accepted directly. Replacing a resident requires
			// the free best to beat that resident's best individual.
			if (worst_res_idx != -1) {
				bool is_authorized_to_replace = false;
				double target_fit_for_log = 0.0;

				if (found_empty_slot) {
					is_authorized_to_replace = true;
				} 
				else {
					Individual* best_worst_res = popula[worst_res_idx][0];
					for (Individual* ind : popula[worst_res_idx]) {
						if (ind->fit < best_worst_res->fit) best_worst_res = ind;
					}
					if (best_free->fit < best_worst_res->fit) {
						is_authorized_to_replace = true;
						target_fit_for_log = best_worst_res->fit;
					}
				}

				if (is_authorized_to_replace) {
					if (found_empty_slot) {
						std::cout << "\n[Population] Free population occupies empty slot "
						          << worst_res_idx << " (objective="
						          << std::fixed << best_free->fit 
						          << ")\n" << std::endl;
					} else {
						std::cout << "\n[Population] Free population replaces slot "
						          << worst_res_idx << " (candidate="
						          << std::fixed << best_free->fit
						          << ", resident_best=" << target_fit_for_log
						          << ")\n" << std::endl;
					}

					reverseReplacementCount++;

					// Start a new protection period.
					popCreationIteration[worst_res_idx] = data->iteration;

					SubPopulation& target_sub = popula[worst_res_idx];

					for (Individual* ind : target_sub) delete ind;
					target_sub.clear();

					data->compactLocConfigure[worst_res_idx].clear();
					for (int j = 0; j < numL; j++) {
						if (best_free->location[j].used) {
							data->compactLocConfigure[worst_res_idx].insert(j + numV);
						}
					}

					// Seed the resident population with the free best.
					Individual* seed_indiv = new Individual();
					seed_indiv->define(data);
					seed_indiv->initilization();
					copyAllData(seed_indiv, best_free);
					target_sub.emplace(target_sub.begin(), seed_indiv);
                    
                    // Export the seed routes.
                    if (pool != nullptr && seed_indiv->isFeasible) {
                        pool->extractRoutes(seed_indiv);
                    }

					for (Individual* ind : subpop) delete ind;
					subpop.clear();

					// Return the slot so RouteGenerator can refill it.
					return worst_res_idx; 
				}
			}
		}

		// Otherwise apply the normal diversity-based population reduction.
		while ((int)subpop.size() > data->popMin)
			removeWorstBiasedFitness(subpop);
	}
	if (isObtainNewPop)
		return index;
	else
		return -1;
}

void popManager::addInElitePop(Individual * /*indiv*/){
}
Individual * popManager::selectionParentSolution(Individual *exist, int rankPop){
	if (rankPop < 0 || rankPop >= static_cast<int>(popula.size())) {
		throw std::out_of_range(
			"parent selection population index is out of range");
	}
	SubPopulation & subpop = popula[rankPop];
	if (subpop.empty()) {
		throw std::runtime_error(
			"cannot select a parent from an empty population");
	}
	updateBiasedFitnesses(subpop);

	int rand1,rand2;
	Individual *target;
	if (subpop.size() == 1){
		target = subpop[0];
		return target;
	}
	if (subpop.size() == 2 && exist != nullptr) {
		if (subpop[0] == exist)
			return subpop[1];
		if (subpop[1] == exist)
			return subpop[0];
	}
	int max_idx = (int)subpop.size() - 1;
	const int maximumAttempts =
		std::max(16, static_cast<int>(subpop.size()) * 4);
	for (int attempt = 0; attempt < maximumAttempts; ++attempt) {
		rand1 = RandomInt(0, max_idx); // 使用 basic.h 中的随机函数
		rand2 = RandomInt(0, max_idx);
		while (rand1 == rand2) rand2 = RandomInt(0, max_idx);
		//if (subpop[rand1]->biasedFit < subpop[rand2]->biasedFit)
		if (subpop[rand1]->biasedFit < subpop[rand2]->biasedFit)
			target = subpop[rand1];
		else
			target = subpop[rand2];

		if (target != exist)
			return target;
	}

	// Rejection sampling above preserves the historical tournament behavior
	// in normal runs. A deterministic fallback guarantees progress even when
	// the excluded parent wins every sampled tournament.
	target = nullptr;
	for (Individual* candidate : subpop) {
		if (candidate == exist)
			continue;
		if (target == nullptr ||
			candidate->biasedFit < target->biasedFit) {
			target = candidate;
		}
	}
	if (target == nullptr) {
		throw std::logic_error(
			"parent selection found no candidate distinct from the "
			"excluded parent");
	}
	return target;
}

void popManager::reStart(){
	std::cout<<"----Restart and create a new population----"<<std::endl;
	for (int i=0;i<data->numLocConfiguration+1;i++){
		for (Individual * indiv : popula[i]) delete indiv ;
		popula[i].clear();
	}
	data->nbIterNonImpro=0;
	bestIndivRestart->fit = Max;
}
double popManager::averagePop(){
	double fit=0;
	for (Individual * indiv : feasibleSubpopulation)
		fit += indiv->fit;
	fit = fit/(double)feasibleSubpopulation.size();
	return fit;
}
double popManager::usedDepots(){
	std::set<int>locations;
	for (Individual * indiv : feasibleSubpopulation){
		for (int i=0;i<numL;i++){
			if (indiv->location[i].used)
				locations.insert(i);
		}
	}
	int ss = locations.size();
	return(double(ss) / double(numL));
}
void popManager::removeWorstBiasedFitness(SubPopulation & pop){
	updateBiasedFitnesses(pop);
	if (pop.size() <= 1) throw std::string("Eliminating the best individual: this should not occur in the algorithm");

	Individual * worstIndividual = NULL;
	int worstIndividualPosition = -1;
	bool isWorstIndividualClone = false;
	double worstIndividualBiasedFitness = -1.e30;
	for (int i = 0; i < (int)pop.size(); i++)
	{
		bool isClone = (pop[i]->averageBrokenPairsDistanceClosest(1) < Min); // A distance equal to 0 indicates that a clone exists
		if ((isClone && !isWorstIndividualClone) || (isClone == isWorstIndividualClone && pop[i]->biasedFit > worstIndividualBiasedFitness))
		{
			worstIndividualBiasedFitness = pop[i]->biasedFit;
			isWorstIndividualClone = isClone;
			worstIndividualPosition = i;
			worstIndividual = pop[i];
		}
	}

	pop.erase(pop.begin() + worstIndividualPosition); // Removing the individual from the population
	for (Individual * myIndividual2 : pop) myIndividual2->removeProximity(worstIndividual); // Cleaning its distances from the other individuals in the population
	delete worstIndividual; // Freeing memory
	worstIndividual=NULL;
}

void popManager::updateBiasedFitnesses(SubPopulation & pop){
	// Ranking the individuals based on their diversity contribution (decreasing order of distance)
	std::vector <std::pair <double, int> > ranking;
	for (int i = 0 ; i < (int)pop.size(); i++)
		ranking.push_back({-pop[i]->averageBrokenPairsDistanceClosest(data->nbClost),i});
	std::sort(ranking.begin(), ranking.end());

	// Updating the biased fitness values
	if (pop.size() == 1)
		pop[0]->biasedFit = 0;
	else{
		for (int i = 0; i < (int)pop.size(); i++){
			double divRank = (double)i / (double)(pop.size() - 1); // Ranking from 0 to 1
			double fitRank = (double)ranking[i].second / (double)(pop.size() - 1);
			if ((int)pop.size() <= data->nbElite) // Elite individuals cannot be smaller than population size
				pop[ranking[i].second]->biasedFit = fitRank;
			else
				pop[ranking[i].second]->biasedFit = fitRank + (1.0 - (double)data->nbElite / (double)pop.size()) * divRank;
		}
	}
}
void popManager::removeEmptySubPop(){
	for (int i = popula.size()-2;i >= 0; i--){
		if (popula[i].empty()){
			popula.erase(popula.begin() + i);
			data->numLocConfiguration--;
		}
	}
	data->dimTabu1.clear();
	data->dimTabu2.clear();
	long long int var, var1, var2;;int seed;
	for (int i=0;i<data->numLocConfiguration;i++){
		if ((int)popula[i].size() > 0){
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
void popManager::reduceHalfSubPop(){
	if ((int)popula.size() < 3)return;
	std::vector<std::pair<double, int>>averageCost;
	double avgCost;
	for (int i=0;i< data->numLocConfiguration;i++){
		avgCost = 0;
		for (int j=0;j<(int)popula[i].size();j++){
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
	int halfSize = (int)(data->numLocConfiguration/2.);
	std::vector<int> remove;
	for (int i=(int)averageCost.size()-1; i > halfSize; i--)
		remove.push_back(averageCost[i].second);
	std::sort(remove.begin(),remove.end());
	std::reverse(remove.begin(),remove.end());
	for (int i=0;i<(int)remove.size();i++){
		SubPopulation & sub = popula[remove[i]];
		for (Individual * indiv : sub) delete indiv;
		popula.erase(popula.begin()+remove[i]);
		data->compactLocConfigure.erase(data->compactLocConfigure.begin() + remove[i]);
	}
	data->numLocConfiguration = data->numLocConfiguration - remove.size();

	data->dimTabu1.clear();
	data->dimTabu2.clear();
	long long int var, var1, var2;;int seed;
	for (int i=0;i<data->numLocConfiguration;i++){
		if ((int)popula[i].size() > 0){
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

int popManager::getWorstResidentIndex() {
    int worstIdx = 0;
    double worstAvgFit = -1.0;
    
    // 客观事实防线 1：只遍历常驻种群 (0 到 data->numLocConfiguration - 1)
    // 绝对不能触碰索引为 data->numLocConfiguration 的游离种群
    for (int i = 0; i < data->numLocConfiguration; ++i) {
        if (popula[i].empty()) {
            // 如果存在空种群，优先覆盖它，避免覆盖已有数据的种群
            return i; 
        }
        
        double sumFit = 0.0;
        for (Individual* ind : popula[i]) {
            sumFit += ind->fit; 
        }
        double avgFit = sumFit / popula[i].size();
        
        if (avgFit > worstAvgFit) {
            worstAvgFit = avgFit;
            worstIdx = i;
        }
    }
    return worstIdx;
}

SpInjectionResult popManager::injectSPSolution(
        std::unique_ptr<Individual> ownedSpIndiv,
        RoutePool* pool) {
    if (!ownedSpIndiv) {
        return {};
    }
    Individual* spIndiv = ownedSpIndiv.get();
    spIndiv->decideFeasible();
    if (!spIndiv->isFeasible) {
        return {};
    }
    spIndiv->isRight();

    // Route-pool admission is independent from population admission. Even a
    // population duplicate or a rejected depot configuration may contain new
    // columns created by the post-SP local search.
    if (pool != nullptr)
        pool->submitFeasibleIndividual(spIndiv);
    
    if (data->no_capacity_location || data->numLocConfiguration == 0) {
        // This SP candidate has already crossed the route-submission
        // boundary above.
        for (Individual* existing : popula[0]) {
            if (data->equalTwoVar(existing->fit, spIndiv->fit) &&
                existing->hasSameRouteStructure(*spIndiv)) {
                return {};
            }
        }
        addIndividual(spIndiv, 0, false, nullptr);
        return {
            SpInjectionDisposition::AdmittedExisting, 0};
    }

    // Derive the depot configuration and its lookup keys.
    std::set<int> spDepots;
    long long int var1 = 0, var2 = 0;
    for (int i = 0; i < data->numL; i++) {
        if (spIndiv->location[i].used) {
            spDepots.insert(i + data->numV);
            long long int var = 1ULL << i;
            var1 += var;
            var2 += var + 9527 * i;
        }
    }

    // Match an existing resident depot configuration.
    int matchIndex = -1;
    for (int i = 0; i < data->numLocConfiguration; ++i) {
        if (data->compactLocConfigure[i] == spDepots) {
            matchIndex = i;
            break;
        }
    }

    // Update restart and global incumbents.
    if (spIndiv->fit + Min < bestIndivRestart->fit) {
        bestIndivRestart->initilization();
        copyAllData(bestIndivRestart, spIndiv);
        data->nbIterNonImpro = 0;
    }
    if (spIndiv->fit + Min < global->fit) {
        global->initilization();
        copyAllData(global, spIndiv);
        data->recordGlobalIncumbent();
    }

    if (matchIndex != -1) {
        // Equal objective values are common on rounded instances. Reject only
        // an exact route/depot structural clone before computing diversity.
        for (Individual* existingIndiv : popula[matchIndex]) {
            if (data->equalTwoVar(existingIndiv->fit, spIndiv->fit) &&
                existingIndiv->hasSameRouteStructure(*spIndiv)) {
                std::cout << "[SP Inject] rejected duplicate in resident "
                          << matchIndex << std::endl;
                return {};
            }
        }

        std::cout << "[SP Inject] matched resident " << matchIndex
                  << std::endl;

        for (Individual* existingIndiv : popula[matchIndex]) {
            double myDistance = spIndiv->brokenPairsDistance(existingIndiv);
            existingIndiv->indivsPerProximity.insert({ myDistance, spIndiv });
            spIndiv->indivsPerProximity.insert({ myDistance, existingIndiv });
        }

        SubPopulation& matchedPopulation = popula[matchIndex];
        int insertionIndex = static_cast<int>(matchedPopulation.size());
        while (insertionIndex > 0 &&
               matchedPopulation[insertionIndex - 1]->fit >
                   spIndiv->fit - Min) {
            --insertionIndex;
        }
        matchedPopulation.emplace(
            matchedPopulation.begin() + insertionIndex,
            ownedSpIndiv.release());

        if ((int)matchedPopulation.size() > data->popMin) {
            updateBiasedFitnesses(matchedPopulation);
            removeWorstBiasedFitness(matchedPopulation);
        }
        return {
            SpInjectionDisposition::AdmittedExisting,
            matchIndex};
    } else {
        // Prefer an empty slot; otherwise identify the resident with the
        // highest mean objective.
        int worstIdx = -1;
        double worstAvgFit = -1.0;
        
        for (int i = 0; i < data->numLocConfiguration; ++i) {
            if (popula[i].empty()) {
                worstIdx = i;
                worstAvgFit = 1e30;
                break;
            }
            double sumFit = 0.0;
            for (Individual* ind : popula[i]) sumFit += ind->fit;
            double avgFit = sumFit / (double)popula[i].size();
            
            if (avgFit > worstAvgFit) {
                worstAvgFit = avgFit;
                worstIdx = i;
            }
        }

        // Require the SP solution to beat the best individual in the target
        // resident population.
        double bestFitInWorstPop = 1e30;
        if (worstIdx != -1 && !popula[worstIdx].empty()) {
            for (Individual* ind : popula[worstIdx]) {
                if (ind->fit < bestFitInWorstPop) bestFitInWorstPop = ind->fit;
            }
        } else {
            bestFitInWorstPop = 1e30;
        }

        if (worstIdx != -1 && spIndiv->fit < bestFitInWorstPop) {
            std::cout << "[SP Inject] replaces resident " << worstIdx
                      << " (candidate=" << std::fixed << spIndiv->fit
                      << ", resident_best=" << bestFitInWorstPop << ")"
                      << std::endl;
            // Start the protection period for the replacement population.
            popCreationIteration[worstIdx] = data->iteration;

            long long int old_var1 = 0, old_var2 = 0;
            for (auto it = data->compactLocConfigure[worstIdx].begin(); it != data->compactLocConfigure[worstIdx].end(); ++it) {
                int old_loc = *it - data->numV;
                long long int old_var = 1ULL << old_loc;
                old_var1 += old_var;
                old_var2 += old_var + 9527 * old_loc;
            }
            auto it1 = data->dimTabu1.find(old_var1 % data->dimension);
            if (it1 != data->dimTabu1.end() && it1->second == worstIdx) data->dimTabu1.erase(it1);
            auto it2 = data->dimTabu2.find(old_var2 % data->dimension);
            if (it2 != data->dimTabu2.end() && it2->second == worstIdx) data->dimTabu2.erase(it2);

            for (Individual* ind : popula[worstIdx]) { delete ind; }
            popula[worstIdx].clear();

            data->compactLocConfigure[worstIdx] = spDepots;
            data->dimTabu1[var1 % data->dimension] = worstIdx;
            data->dimTabu2[var2 % data->dimension] = worstIdx;

            popula[worstIdx].push_back(ownedSpIndiv.release());

            return {
                SpInjectionDisposition::AdmittedNewConfiguration,
                worstIdx};
        } else {
            std::cout << "[SP Inject] rejected (candidate=" << std::fixed
                      << spIndiv->fit
                      << ", required_below=" << bestFitInWorstPop << ")"
                      << std::endl;
            return {};
        }
    }
}
