/*
 * main.cpp
 *
 *  Created on: 27 Apr 2021
 *      Author: Pengfei He
 */
#include"basic.h"
#include"commandline.h"
#include"solution.h"

std::mt19937 global_rng;

int main(int argc, char*argv[]){
	try {
	CommandLine comLin(argc,argv);
	global_rng.seed(comLin.seed);
	read_data *data=new read_data();
	data->read_fun(comLin.pathInstance, comLin.pathDepotCon, comLin.pathSolution, comLin.bestDis, comLin.isOptimal, comLin.timeLimit,comLin.iterLimit);
	data->startTime=clock();
	if (data->hardTimeWindows || data->experimentMode) {
		if (comLin.timeLimit <= 0 || comLin.iterLimit < 0)
			throw std::invalid_argument("This run requires a positive CPU limit and nonnegative iteration limit");
		data->iteration = 0; data->nbIterNonImpro = 0; data->meetBestTime = 0;
		if (!data->hardTimeWindows) data->loopp = 0;
	}
	if (data->experimentMode) std::cout << std::setprecision(17);
	std::cout << "[PROFILE] algorithm="
	          << (data->experimentMode ? (data->hardTimeWindows ? "hga-sp2-setf-study-v1" : "hga-sp2-sets-study-v3") : "hga-sp2-family-profiles-v1")
	          << " parameters=" << data->parameterProfile
	          << (data->hardTimeWindows ? " metric=farham-floor-euclidean" : "")
	          << " hard_time_windows=" << data->hardTimeWindows
	          << " customers=" << data->numV << " depots=" << data->numL
	          << " alpha=" << data->alpha << " population_min=" << data->popMin
	          << " population_max=" << data->popMax << " pool=" << data->spMaxPoolSize
	          << " mutation=" << (100 - data->mutationProb) / 100.0
	          << " threads=" << data->spThreads << " sp_time_limit=" << data->spTimeLimit
	          << " sp_gap=" << data->spMIPGap << " sp_frequency=" << data->spTriggerFrequency
	          << " config_candidates=" << data->configurationCandidateLimit
	          << " config_samples=" << data->configurationSamples
	          << " config_limit=" << data->maxActiveConfigurations
	          << " initial_multiplier=" << data->initialPopulationMultiplier
	          << " warmup_iterations=" << data->maxIterationPerLoc
	          << " restart_non_improving=" << data->maxIterNonProd
	          << " restart_min_iterations=" << data->restartMinRemainingIterations
	          << " restart_min_seconds=" << data->restartMinRemainingSeconds
	          << " protection_iterations=" << data->popProtectionPeriod
	          << " tabu_dimension=" << data->dimension << std::endl;
	if (data->experimentMode) {
		std::cout << "[EXPERIMENT_REQUESTED] r_cfg=" << data->requestedScreeningRepetitions
		          << " mu=" << data->requestedPopulationMinimum
		          << (data->hardTimeWindows ? " lambda=" : " alpha=")
		          << (data->hardTimeWindows ? data->requestedNeighborhoodFraction : double(data->alpha))
		          << " p_mut_percent=" << data->requestedMutationPercent
		          << " rho=" << data->requestedShawRemovalFraction
		          << " pool=" << data->requestedRoutePoolCapacity
		          << " arm=" << data->experimentArm << " sp_policy=" << data->spPolicyId
		          << " beta=" << data->spBudgetMultiplier
		          << " effective_alpha=" << data->alpha
		          << " effective_shaw_removal_count=" << data->requestedShawRemovalCount
		          << " mutation_comparison=draw_ge_threshold mutation_threshold=" << data->mutationProb
		          << (data->hardTimeWindows ? " opt_match=exact_integer" : " opt_match=exact_native_objective")
		          << " opt_ack_timeout_wall_seconds=300 opt_ack_poll_ms=10"
		          << " main_counter_limit=" << data->iterLimit << " main_counter_comparison=gt"
		          << " process_cpu_safety_limit=" << data->timeLimit << " sp_schedule=";
		for (std::size_t i = 0; i < data->spSchedule.size(); ++i)
			std::cout << (i ? "," : "") << data->spSchedule[i];
		std::cout << std::endl;
	}
	/***********************/

	popManager *popman=new popManager(data);
	crossover *cross=new crossover (data,popman);
	RepairProcedure *repair=new RepairProcedure(data, popman);
	LS * ls=new LS(data, repair);// local search phase
	initial_sol *initialSol=new initial_sol (data, repair);
	mutation *mutationOperator=new mutation(data);
	solution *solver=new solution(data,cross,initialSol,ls,popman,repair,mutationOperator);
	//
	solver->main_loop();
	if (data->experimentMode && data->hardTimeWindows) {
		std::cout << "[EXPERIMENT_EFFECTIVE] neighborhood_lists=" << data->twNeighborhoodLists
		          << " neighborhood_entries=" << data->twNeighborhoodEntries
		          << " neighborhood_mean=";
		if (data->twNeighborhoodLists)
			std::cout << double(data->twNeighborhoodEntries) / data->twNeighborhoodLists;
		else std::cout << "null";
		std::cout << " neighborhood_max=" << data->twNeighborhoodMaximum
		          << " neighborhood_filtered=" << data->twNeighborhoodFiltered
		          << " ls_candidates=" << data->twLocalSearchCandidates
		          << " ls_infeasible_candidates=" << data->twLocalSearchRejected
		          << " mutation_calls=" << data->twMutationCalls
		          << " mutation_removed_customers=" << data->twMutationRemovedCustomers
		          << " tw_repair_failures=" << data->twRepairFailures
		          << " opt_independently_accepted=" << data->experimentOptimumAccepted
		          << " opt_ack_wall_seconds=" << data->experimentOptAckWallSeconds
		          << " termination_reason=" << data->experimentTerminationReason << std::endl;
	}
    if (data->experimentMode && !data->hardTimeWindows) {
        std::size_t entries = 0, maximum = 0;
        for (const auto& neighbors : data->edgeNode) {
            entries += neighbors.size();
            maximum = std::max(maximum, neighbors.size());
        }
        std::cout << "[EXPERIMENT_EFFECTIVE] neighborhood_semantics=symmetric_customer_depot_graph"
                  << " neighborhood_lists=" << data->edgeNode.size()
                  << " neighborhood_entries=" << entries
                  << " neighborhood_mean=" << double(entries) / data->edgeNode.size()
                  << " neighborhood_max=" << maximum
                  << " mutation_calls=" << data->setSMutationCalls
                  << " mutation_removed_customers=" << data->setSMutationRemovedCustomers
                  << " opt_independently_accepted=" << data->experimentOptimumAccepted
                  << " opt_ack_wall_seconds=" << data->experimentOptAckWallSeconds
                  << " termination_reason=" << data->experimentTerminationReason << std::endl;
    }
	/*************************************************/
	delete solver;
	delete repair;
	delete cross;
	delete popman;
	delete mutationOperator;
	delete ls;
	delete initialSol;
	delete data;
	return 0;
	} catch (const std::exception& error) {
		std::cerr << "Error: " << error.what() << std::endl;
		return 1;
	} catch (const std::string& error) {
		std::cerr << "Error: " << error << std::endl;
		return 1;
	} catch (...) {
		std::cerr << "Error: unknown solver exception" << std::endl;
		return 1;
	}
}
