/******************************************************************************
 * File: popManager.cpp
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description: 
 *   Manages the population of individuals, sub-populations, and diversity logic.
 ******************************************************************************/

#include "popManager.h"
#include "ComponentProfiler.h"
#include "RunLogger.h"
#include "RoutePool.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <utility>

namespace {

const char* pipelineName(
        const hgasp::RouteSubmissionContext* context) noexcept {
    if (context == nullptr) {
        return "initial_construction";
    }
    switch (context->origin) {
        case hgasp::RouteOriginKind::InitialConstruction:
            return "initial_construction";
        case hgasp::RouteOriginKind::EaxOffspring:
            return "eax+local_search";
        case hgasp::RouteOriginKind::Mutation:
            return "eax+mutation+local_search";
        case hgasp::RouteOriginKind::LocalSearch:
            return "local_search";
        case hgasp::RouteOriginKind::SpInjection:
            return "sp_injection";
    }
    return "unknown";
}

hgasp::DepotConfigurationKey individualConfiguration(
		const Individual& individual,
		const int depotCount) {
	std::vector<int> depots;
	for (int depot = 0; depot < depotCount; ++depot) {
		if (individual.location[depot].used) {
			depots.push_back(depot);
		}
	}
	return hgasp::makeDepotConfigurationKey(std::move(depots));
}

void logIncumbentChanged(
        const SolverContext& data,
        const Individual& individual,
        const hgasp::RouteSubmissionContext* context) {
    hgasp::RunLogger::event(
        hgasp::LogLevel::info,
        "search",
        "incumbent_changed",
        {},
        {
            {"iteration", data.iteration},
            {"cpu_seconds", data.elapsedCpuSeconds()},
            {"objective", individual.fit},
            {"source_trace_id",
                static_cast<unsigned long long>(
                    context == nullptr
                        ? 0U
                        : context->traceId)},
            {"first_population",
                context == nullptr
                    ? -1
                    : context->firstPopulation},
            {"second_population",
                context == nullptr
                    ? -1
                    : context->secondPopulation},
            {"first_parent_individual_id",
                static_cast<unsigned long long>(
                    context == nullptr
                        ? 0U
                        : context->firstParentIndividualId)},
            {"second_parent_individual_id",
                static_cast<unsigned long long>(
                    context == nullptr
                        ? 0U
                        : context->secondParentIndividualId)},
            {"operator_pipeline",
                pipelineName(context)},
            {"sp_ancestry_call_id",
                context == nullptr
                    ? std::string()
                    : context->spAncestryCallId},
        });
}

}  // namespace

popManager::popManager(SolverContext *data) {
	this->data=data;
	this->numV = data->numV;
	this->numL = data->numL;
	this->numC = data->numC;
	popula = std::vector <std::vector<Individual*>>(100);
	reverseReplacementCount = 0; // 初始化计数器
    popCreationIteration = std::vector<long long int>(100, 0); // 初始化种群代数记录器，默认容量为 100 
}

popManager::~popManager() {
	for (int i = 0; i < (int)feasibleSubpopulation.size(); i++)
		delete feasibleSubpopulation[i];
	// Population storage has spare physical slots. V1.16.1 assigns one slot
	// after the permanent free population to the challenger and one additional
	// slot to transactional promotion staging.
	// Ownership cleanup must therefore cover every allocated subpopulation,
	// not only the historical fixed-plus-free prefix.
	for (SubPopulation& population : popula) {
		for (Individual* individual : population) delete individual;
		population.clear();
	}
	for (Individual* individual : recycledIndividuals) delete individual;
	recycledIndividuals.clear();
}

Individual* popManager::acquirePopulationIndividual() {
	if (!recycledIndividuals.empty()) {
		Individual* const individual = recycledIndividuals.back();
		recycledIndividuals.pop_back();
		hgasp::recordHotPathCounter(
			hgasp::HotPathCounter::PopulationObjectReuses);
		return individual;
	}
	auto* const individual = new Individual();
	individual->define(data);
	hgasp::recordHotPathCounter(
		hgasp::HotPathCounter::PopulationObjectAllocations);
	return individual;
}

void popManager::recyclePopulationIndividual(Individual* const individual) {
	if (individual == nullptr) {
		return;
	}
	individual->indivsPerProximity.clear();
	const std::size_t capacity = static_cast<std::size_t>(
		std::max(1, data->parameters.population.maximumSize()));
	if (recycledIndividuals.size() < capacity) {
		recycledIndividuals.push_back(individual);
	} else {
		delete individual;
	}
}

bool popManager::acceptValidatedSpIncumbent(
		Individual& individual,
		const hgasp::RouteSubmissionContext* const submission,
		const Individual::PopulationValidationReceipt*
			const validation) {
	if (!validateForPopulationAdmission(individual, validation)) {
		throw std::logic_error(
			"validated SP incumbent failed independent feasibility");
	}
	if (individual.fit + Min < bestIndivRestart->fit) {
		copyAllData(bestIndivRestart, &individual);
		data->nbIterNonImpro = 0;
	}
	if (individual.fit + Min >= global->fit) {
		return false;
	}
	copyAllData(global, &individual);
	data->recordGlobalIncumbent();
	logIncumbentChanged(*data, individual, submission);
	return true;
}

bool popManager::validateForPopulationAdmission(
		Individual& individual,
		const Individual::PopulationValidationReceipt*
			const validation) {
	if (validation != nullptr &&
		individual.matchesPopulationValidationReceipt(*validation)) {
		hgasp::recordHotPathCounter(
			hgasp::HotPathCounter::PopulationValidationCacheHits);
		return individual.isFeasible;
	}
	individual.decideFeasibleAtTrustBoundary();
	return individual.isFeasible;
}

void popManager::rebuildProximity(
		SubPopulation& population) {
	for (Individual* individual : population) {
		individual->indivsPerProximity.clear();
	}
	for (std::size_t left = 0U;
		 left < population.size();
		 ++left) {
		for (std::size_t right = left + 1U;
			 right < population.size();
			 ++right) {
			const double distance =
				population[left]->brokenPairsDistance(
					population[right]);
			population[left]->indivsPerProximity.insert(
				{distance, population[right]});
			population[right]->indivsPerProximity.insert(
				{distance, population[left]});
		}
	}
}

void popManager::assignSearchMetadata(
		Individual* const individual,
		const hgasp::RouteSubmissionContext* const submission) {
	if (individual == nullptr) {
		return;
	}
	if (nextIndividualId ==
		std::numeric_limits<std::uint64_t>::max()) {
		throw std::overflow_error(
			"population individual identifier exhausted");
	}
	individual->searchIndividualId = nextIndividualId++;
	individual->sourceTraceId =
		submission == nullptr ? 0U : submission->traceId;
	individual->spAncestryCallId =
		submission == nullptr
			? std::string()
			: submission->spAncestryCallId;
	hgasp::RunLogger::event(
		submission != nullptr &&
				submission->origin ==
					hgasp::RouteOriginKind::SpInjection
			? hgasp::LogLevel::info
			: hgasp::LogLevel::debug,
		"population",
		"individual_admitted",
		{},
		{
			{"iteration", data->iteration},
			{"individual_id",
				static_cast<unsigned long long>(
					individual->searchIndividualId)},
			{"source_trace_id",
				static_cast<unsigned long long>(
					individual->sourceTraceId)},
			{"sp_ancestry_call_id",
				individual->spAncestryCallId},
		});
}

void popManager::copyAllData(Individual* s, const Individual* ss) {
	hgasp::recordHotPathCounter(
		hgasp::HotPathCounter::PopulationCopyCalls);
	if (s == nullptr || ss == nullptr) {
		throw std::invalid_argument(
			"population copy requires non-null individuals");
	}
	if (s == ss) {
		return;
	}
	const std::vector<int>& activeRoutes = ss->activeRouteIndices();

	// The fixed representation is initialized once.  A copy only needs to
	// clear route slots that were occupied by the target's previous solution;
	// every other slot is already a canonical empty route.
	s->resetForPopulationCopy();
	for (const int routeIndex : activeRoutes) {
		const Route& sourceRoute = ss->route[routeIndex];
		if (!sourceRoute.used || !routeIsOccupied(sourceRoute)) {
			throw std::logic_error(
				"population copy source route state is inconsistent");
		}
		hgasp::recordHotPathCounter(
			hgasp::HotPathCounter::PopulationActiveRoutesCopied);
		Route& targetRoute = s->route[routeIndex];
		targetRoute.used = true;
		Node* targetPrevious = targetRoute.depot;
		const Node* sourceNode = sourceRoute.depot->next;
		while (!sourceNode->isDepot) {
			Node* const targetNode = &s->client[sourceNode->cour];
			targetPrevious->next = targetNode;
			targetNode->pre = targetPrevious;
			targetNode->route = &targetRoute;
			targetPrevious = targetNode;
			sourceNode = sourceNode->next;
		}
		targetPrevious->next = targetRoute.endDepot;
		targetRoute.endDepot->pre = targetPrevious;
		int mutableRouteIndex = routeIndex;
		s->updateRouteInfor(mutableRouteIndex, -1);
		NodeDepot& targetLocation = *targetRoute.whichLocation;
		targetLocation.used = true;
		targetLocation.remainInven += targetRoute.load;
		++targetLocation.splitTime;
	}
	s->dis=ss->dis;
	s->fit=ss->fit;
	s->biasedFit=ss->biasedFit;
	// Search identity belongs to the new population object and is assigned
	// explicitly at admission. It is never copied as solution semantics.
	s->searchIndividualId = 0U;
	s->sourceTraceId = 0U;
	s->spAncestryCallId.clear();
	// The target view was invalid while routes were reconstructed. The final
	// ownership is exactly the source's route-index ordered active view.
	s->activeRouteScratch_ = activeRoutes;
	s->activeRouteScratchOccupancyGeneration_ =
		s->routeOccupancyGeneration_;
	// A current certificate is transferable only because this function has
	// rebuilt the exact directed routes on the same finalized problem revision.
	// Unfinalized fixtures and stale sources retain the authoritative fallback.
	if (!s->copyFeasibilityCertificateFrom(*ss)) {
		s->decideFeasible();
	}
}
int popManager::addIndividual(
		Individual *indiv,
		int rankPop,
		bool isInitial,
		RoutePool* pool,
		const hgasp::RouteSubmissionContext* submission,
		bool* admitted,
		std::uint64_t* survivingIndividualId,
		const bool allowIncumbentUpdate,
		const Individual::PopulationValidationReceipt*
			const validation) {
	hgasp::ScopedComponentTimer timer(
		hgasp::TimedComponent::PopulationAdmission);
	hgasp::recordHotPathCounter(
		hgasp::HotPathCounter::PopulationAdmissionCalls);
	if (admitted != nullptr) {
		*admitted = false;
	}
	if (survivingIndividualId != nullptr) {
		*survivingIndividualId = 0U;
	}
	if (indiv == nullptr)
		return -1;
	validateForPopulationAdmission(*indiv, validation);
	// Population membership is a trust boundary. Never rank, clone, or export
	// an individual until feasibility and objective consistency both pass.
	if (!indiv->isFeasible)
		return -1;
	return addValidatedIndividual(
		indiv,
		rankPop,
		isInitial,
		pool,
		submission,
		admitted,
		survivingIndividualId,
		allowIncumbentUpdate);
}

int popManager::addValidatedIndividual(
		Individual* indiv,
		int rankPop,
		bool isInitial,
		RoutePool* pool,
		const hgasp::RouteSubmissionContext* submission,
		bool* admitted,
		std::uint64_t* survivingIndividualId,
		const bool allowIncumbentUpdate) {
	if (admitted != nullptr) {
		*admitted = false;
	}
	if (survivingIndividualId != nullptr) {
		*survivingIndividualId = 0U;
	}
	// Route contribution is deliberately independent from population
	// admission. In particular, an equal-cost/structural duplicate or a
	// candidate that later loses a replacement decision can still contribute
	// new master columns.
	if (pool != nullptr) {
		if (submission != nullptr) {
			pool->submitFeasibleIndividual(
				indiv, *submission);
		} else {
			pool->submitFeasibleIndividual(indiv);
		}
	}
	if (isInitial){
			std::set<int> actualDepots;
			if (rankPop > -1 && rankPop < data->numLocConfiguration){
				for (int i=0;i<numL;i++){
					if (indiv->location[i].used)
						actualDepots.insert(i+numV);
				}
				if (actualDepots !=
					data->compactLocConfigure[
						static_cast<std::size_t>(rankPop)]) {
					// A configuration-changing initialization operator may not
					// contaminate the requested fixed population. The permanent
					// unrestricted population is the only safe legacy fallback;
					// V1.16.1 static runtime normally classifies before this call.
					rankPop = data->numLocConfiguration;
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
	Individual* myIndividual = acquirePopulationIndividual();
	copyAllData(myIndividual, indiv);
	assignSearchMetadata(myIndividual, submission);
	for (Individual * myIndividual2 : subpop){
		double myDistance = myIndividual->brokenPairsDistance(myIndividual2);
		myIndividual2->indivsPerProximity.insert({ myDistance, myIndividual });
		myIndividual->indivsPerProximity.insert({ myDistance, myIndividual2 });
	}

	int place = (int)subpop.size();
	while (place > 0 && subpop[place - 1]->fit > indiv->fit - Min) place--;
	subpop.emplace(subpop.begin() + place, myIndividual);
	if (admitted != nullptr) {
		*admitted = true;
	}
	//
	std::vector<int>depot;
	bool isObtainNewPop = false;int index = -1;

	if (allowIncumbentUpdate &&
		indiv->fit + Min < bestIndivRestart->fit){
		copyAllData(bestIndivRestart, indiv);
		data->nbIterNonImpro=0;
		if (indiv->fit + Min < global->fit){
			copyAllData(global, indiv);
			data->recordGlobalIncumbent();
			logIncumbentChanged(
				*data, *indiv, submission);
		}
		//to replace a subPop if the new global solution's not belong to any subpop.
			if (!data->no_capacity_location &&
				!data->staticHgampPopulationsEnabled()){
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
				copyAllData(myIndivi, myIndividual);
					hgasp::RouteSubmissionContext replacementLineage;
					replacementLineage.traceId =
						myIndividual->sourceTraceId;
					replacementLineage.firstParentIndividualId =
						myIndividual->searchIndividualId;
					replacementLineage.spAncestryCallId =
						myIndividual->spAncestryCallId;
					assignSearchMetadata(
						myIndivi, &replacementLineage);

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
	if ((int)subpop.size() ==
		data->parameters.population.maximumSize()) {
		
		// Replacement applies only to the free population after initialization.
			if (rankPop == data->numLocConfiguration &&
				!isInitial &&
				!data->staticHgampPopulationsEnabled()) {
			
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
					copyAllData(seed_indiv, best_free);
						hgasp::RouteSubmissionContext replacementLineage;
						replacementLineage.traceId =
							best_free->sourceTraceId;
						replacementLineage.firstParentIndividualId =
							best_free->searchIndividualId;
						replacementLineage.spAncestryCallId =
							best_free->spAncestryCallId;
						assignSearchMetadata(
							seed_indiv, &replacementLineage);
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
	if (survivingIndividualId != nullptr) {
		const auto survivor = std::find(
			subpop.begin(), subpop.end(), myIndividual);
		if (survivor != subpop.end()) {
			*survivingIndividualId =
				(*survivor)->searchIndividualId;
		}
	}
	if (isObtainNewPop)
		return index;
	else {
		return -1;
	}
}

int popManager::addIndividualToTarget(
		Individual* const indiv,
		const hgasp::PopulationAdmissionTarget& target,
		const bool isInitial,
		RoutePool* const pool,
		const hgasp::RouteSubmissionContext* const submission,
		bool* const admitted,
		std::uint64_t* const survivingIndividualId,
		const bool allowIncumbentUpdate,
		const Individual::PopulationValidationReceipt*
			const validation) {
	hgasp::ScopedComponentTimer timer(
		hgasp::TimedComponent::PopulationAdmission);
	hgasp::recordHotPathCounter(
		hgasp::HotPathCounter::PopulationAdmissionCalls);
	if (admitted != nullptr) {
		*admitted = false;
	}
	if (survivingIndividualId != nullptr) {
		*survivingIndividualId = 0U;
	}
	if (target.populationIndex < 0 ||
		static_cast<std::size_t>(target.populationIndex) >= popula.size()) {
		throw std::out_of_range(
			"typed population admission index is out of range");
	}
	if (target.generation == 0U) {
		throw std::invalid_argument(
			"typed population admission requires a generation");
	}
	if (indiv == nullptr) {
		if (admitted != nullptr) {
			*admitted = false;
		}
		return -1;
	}
	if (!validateForPopulationAdmission(*indiv, validation)) {
		if (admitted != nullptr) {
			*admitted = false;
		}
		return -1;
	}
	const bool exactRole =
		target.role == hgasp::PopulationAdmissionRole::FixedResident ||
		target.role == hgasp::PopulationAdmissionRole::DynamicChallenger ||
		target.role == hgasp::PopulationAdmissionRole::TransactionStaging;
	if (exactRole && !target.expectedConfiguration.has_value()) {
		throw std::invalid_argument(
			"exact typed population admission has no expected configuration");
	}
	if (target.expectedConfiguration.has_value()) {
		const hgasp::DepotConfigurationKey canonical =
			hgasp::makeDepotConfigurationKey(
				target.expectedConfiguration->openDepots);
		if (canonical != *target.expectedConfiguration ||
			canonical.openDepots.empty()) {
			throw std::invalid_argument(
				"typed population admission configuration is not canonical");
		}
		if (individualConfiguration(*indiv, numL) != canonical) {
			throw std::logic_error(
				"typed population admission configuration mismatch");
		}
	}
	return addValidatedIndividual(
		indiv,
		target.populationIndex,
		isInitial,
		pool,
		submission,
		admitted,
		survivingIndividualId,
		allowIncumbentUpdate);
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

void popManager::reStart(const int preservedPopulationIndex){
	if (preservedPopulationIndex < -1 ||
		(preservedPopulationIndex >= 0 &&
		 static_cast<std::size_t>(preservedPopulationIndex) >= popula.size())) {
		throw std::out_of_range(
			"restart preservation index is outside population storage");
	}
	std::cout<<"----Restart and create a new population----"<<std::endl;
	for (std::size_t index = 0U; index < popula.size(); ++index) {
		if (static_cast<int>(index) == preservedPopulationIndex) {
			continue;
		}
		SubPopulation& population = popula[index];
		for (Individual* individual : population) delete individual;
		population.clear();
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
	recyclePopulationIndividual(worstIndividual);
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
        RoutePool* pool,
        const hgasp::RouteSubmissionContext* const submission) {
	hgasp::ScopedComponentTimer timer(
		hgasp::TimedComponent::PopulationAdmission);
	hgasp::recordHotPathCounter(
		hgasp::HotPathCounter::PopulationAdmissionCalls);
    if (!ownedSpIndiv) {
        return {};
    }
    Individual* spIndiv = ownedSpIndiv.get();
    spIndiv->decideFeasibleAtTrustBoundary();
    if (!spIndiv->isFeasible) {
        return {};
    }

    // Route-pool admission is independent from population admission. Even a
    // population duplicate or a rejected depot configuration may contain new
    // columns created by the post-SP local search.
    if (pool != nullptr) {
        if (submission != nullptr) {
            pool->submitFeasibleIndividual(
                spIndiv, *submission);
        } else {
            pool->submitFeasibleIndividual(spIndiv);
        }
    }
    
    if (data->no_capacity_location || data->numLocConfiguration == 0) {
        // This SP candidate has already crossed the route-submission
        // boundary above.
        for (Individual* existing : popula[0]) {
            if (data->equalTwoVar(existing->fit, spIndiv->fit) &&
                existing->hasSameRouteStructure(*spIndiv)) {
                return {};
            }
        }
        addValidatedIndividual(
            spIndiv,
            0,
            false,
            nullptr,
            submission,
			nullptr,
			nullptr,
			true);
        return {
            SpInjectionDisposition::AdmittedExisting, 0};
    }

    // Derive the depot configuration and its lookup keys.
    std::set<int> spDepots;
    for (int i = 0; i < data->numL; i++) {
        if (spIndiv->location[i].used) {
            spDepots.insert(i + data->numV);
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
        copyAllData(bestIndivRestart, spIndiv);
        data->nbIterNonImpro = 0;
    }
    if (spIndiv->fit + Min < global->fit) {
        copyAllData(global, spIndiv);
        data->recordGlobalIncumbent();
        logIncumbentChanged(
            *data, *spIndiv, submission);
    }

    if (matchIndex == -1 &&
        data->staticHgampPopulationsEnabled()) {
        bool admitted = false;
        addValidatedIndividual(
            spIndiv,
            data->numLocConfiguration,
            false,
            nullptr,
            submission,
			&admitted,
			nullptr,
			true);
        return admitted
            ? SpInjectionResult{
                  SpInjectionDisposition::AdmittedExisting,
                  data->numLocConfiguration}
            : SpInjectionResult{};
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
        assignSearchMetadata(
            matchedPopulation[
                static_cast<std::size_t>(insertionIndex)],
            submission);

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

            for (Individual* ind : popula[worstIdx]) { delete ind; }
            popula[worstIdx].clear();

            data->compactLocConfigure[worstIdx] = spDepots;

            popula[worstIdx].push_back(ownedSpIndiv.release());
            assignSearchMetadata(
                popula[worstIdx].back(), submission);

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
