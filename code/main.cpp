/*
 * main.cpp
 *
 *  Created on: 27 Apr 2021
 *      Author: Pengfei He
 */
#include"basic.h"
#include"commandline.h"
#include"solution.h"
#include <memory>

std::mt19937 global_rng;

int main(int argc, char*argv[]){
	try {
	if (argc == 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
		std::cout << CommandLine::usage() << std::endl;
		return 0;
	}
	CommandLine comLin(argc,argv);
	global_rng.seed(comLin.seed);
	std::unique_ptr<read_data> data(new read_data());
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
	          << (data->experimentMode ? (data->hardTimeWindows ? "masp-v22-setf-study" : "masp-v22-sets-study") : "masp-v22-family-profiles")
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
		          << " main_counter_limit=" << data->iterLimit << " main_counter_comparison=gt"
		          << " process_cpu_safety_limit=" << data->timeLimit << " sp_schedule=";
		for (std::size_t i = 0; i < data->spSchedule.size(); ++i)
			std::cout << (i ? "," : "") << data->spSchedule[i];
		std::cout << std::endl;
	}
	/***********************/

	std::unique_ptr<popManager> popman(new popManager(data.get()));
	std::unique_ptr<crossover> cross(new crossover(data.get(), popman.get()));
	std::unique_ptr<RepairProcedure> repair(new RepairProcedure(data.get(), popman.get()));
	std::unique_ptr<LS> ls(new LS(data.get()));
	std::unique_ptr<initial_sol> initialSol(new initial_sol(data.get(), repair.get()));
	std::unique_ptr<mutation> mutationOperator(new mutation(data.get()));
	std::unique_ptr<solution> solver(new solution(data.get(), cross.get(), initialSol.get(), ls.get(),
	                                            popman.get(), repair.get(), mutationOperator.get()));
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
                  << " termination_reason=" << data->experimentTerminationReason << std::endl;
    }
	/*************************************************/
	return 0;
	} catch (GRBException& error) {
		std::cerr << "Gurobi error: " << error.getMessage() << std::endl;
		return 1;
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
