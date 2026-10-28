/*
 * solution.cpp
 *
 *  Created on: 17 Apr 2020
 *      Author: Peng
 */
#include "basic.h"
#include "solution.h"
#include "initialsol.h"
#include "LS.h"
#include "crossover.h"
#include <chrono>
#include <map>
#include <stdexcept>
#include <memory>
#include <limits>
using namespace std;

void solution::recordHGA(Individual* candidate) {
    if (!study || !candidate->isFeasible || !std::isfinite(candidate->fit)) return;
    study->observe("hga_state_best", candidate, study->source);
    if (study->observe("reported_best", candidate, study->source)) popman->copyAllData(reportedBest, candidate);
}
void solution::recordSP(Individual* candidate, const std::string& origin) {
    if (!study) return;
    study->observe("sp_side_best", candidate, origin);
    if (study->observe("reported_best", candidate, origin)) popman->copyAllData(reportedBest, candidate);
}
bool solution::stopRequested() {
    return data->stopCondition(study ? reportedBest->fit : global->fit);
}
void solution::finishRun() {
    if (!study) { global->isRight(); global->outputBestSolution(); return; }
    const bool feasible = !noFeasibleEnd && study->best.count("reported_best");
    if (feasible) {
        reportedBest->isRight();
        reportedBest->outputBestSolution();
    }
    const bool target = data->experimentTerminationReason == "target_reached";
    for (int counter : data->spSchedule) if (!reachedSPSchedule.count(counter)) {
        study->event("sp_opportunity", "\"call_id\":\"sp_" + std::to_string(counter)
            + "\",\"scheduled_counter\":" + std::to_string(counter)
            + ",\"actual_counter\":null,\"event_status\":" + StudyEvents::quote(target ? "not_reached_target" : "not_reached_other")
            + ",\"skip_reason\":" + StudyEvents::quote(target ? "target_before_trigger" : "termination_before_trigger")
            + ",\"nominal_time_limit_wall_seconds\":" + StudyEvents::number(data->spTimeLimit));
    }
    popman->finishStudyPopulations();
    const long long starts = spSolver ? spSolver->optimizerStarts : 0;
    const std::string zeroReason = starts ? "null" : StudyEvents::quote(data->experimentArm == "NoSP" ? "disabled"
        : target ? "target_before_trigger" : sp_total_triggers ? "no_feasible_offspring_branch" : "no_trigger_reached");
    study->event("summary", "\"termination_reason\":" + StudyEvents::quote(data->experimentTerminationReason)
        + ",\"has_feasible_solution\":" + (feasible ? "true" : "false")
        + ",\"objective_exact\":" + (feasible ? StudyEvents::number(reportedBest->fit) : "null")
        + ",\"main_counter_final\":" + std::to_string(data->iteration)
        + ",\"sp_opportunities_reached\":" + std::to_string(sp_total_triggers)
        + ",\"sp_optimizer_starts\":" + std::to_string(starts)
        + ",\"sp_valid_candidates\":" + std::to_string(sp_found_solutions)
        + ",\"sp_matching_injections\":" + std::to_string(popman->spMatchingInjections)
        + ",\"sp_direct_installs\":" + std::to_string(popman->spDirectInstalls)
        + ",\"free_population_replacements\":" + std::to_string(popman->reverseReplacementCount)
        + ",\"mutation_decisions\":" + std::to_string(study->mutationDecisions)
        + ",\"mutations_executed\":" + std::to_string(study->mutationsExecuted)
        + (data->hardTimeWindows ? ",\"tw_repair_failures\":" + std::to_string(data->twRepairFailures) : "")
        + ",\"invalid_sp_outputs\":" + std::to_string(study->invalidSPOutputs)
        + ",\"final_solution_id\":" + (feasible ? StudyEvents::quote("reported_best_" + std::to_string(study->indices["reported_best"])) : "null")
        + ",\"zero_sp_reason\":" + zeroReason
        + ",\"initialization_cpu_seconds\":" + StudyEvents::number(initializationCPU)
        + ",\"ordinary_hga_cpu_seconds\":null"
        + ",\"sp_build_cpu_seconds\":" + StudyEvents::number(spSolver ? spSolver->modelBuildCPU : 0)
        + ",\"sp_optimization_cpu_seconds\":" + StudyEvents::number(spSolver ? spSolver->optimizationCPU : 0)
        + ",\"post_sp_ls_cpu_seconds\":" + StudyEvents::number(postSPCPU)
        + ",\"new_population_cpu_seconds\":" + StudyEvents::number(newPopulationCPU));
}

void solution::runSP() {
    // Study-only dispatch; the original baseline path below remains unchanged.
    reachedSPSchedule.insert(data->iteration);
    ++sp_total_triggers;
    const std::string call = "sp_" + std::to_string(data->iteration);
    const std::string common = "\"call_id\":" + StudyEvents::quote(call)
        + ",\"scheduled_counter\":" + std::to_string(data->iteration)
        + ",\"actual_counter\":" + std::to_string(data->iteration)
        + ",\"nominal_time_limit_wall_seconds\":" + StudyEvents::number(data->spTimeLimit);
    if (data->experimentArm == "NoSP") {
        study->event("sp_opportunity", common + ",\"event_status\":\"disabled\",\"skip_reason\":\"disabled\"");
        return;
    }
    const double before = reportedBest->fit;
    const double isolatedStarted = StudyEvents::cpu();
    data->isolatedSPStarted = isolatedStarted;
    study->candidateId = "feedback_" + std::to_string(data->iteration);
    study->event("sp_opportunity", common + ",\"event_status\":\"invoked\",\"skip_reason\":null"
        + ",\"cpu_before\":" + StudyEvents::number(StudyEvents::cpu())
        + ",\"wall_before\":" + StudyEvents::number(study->wall())
        + ",\"objective_before_call\":" + StudyEvents::number(before)
        + ",\"pool_ordered_routes\":" + std::to_string(routePool->getPool().size()));
    try {
        study->source = "raw_SP";
        const auto combination = spSolver->solve(routePool, global);
        if (combination.empty() && (spSolver->lastTimeoutNoSolution || spSolver->lastBudgetExhausted)) {
            const std::string reason = spSolver->lastBudgetExhausted ? "budget_exhausted_no_solve" : "timeout_no_solution";
            study->event("sp_completed", common + ",\"event_status\":" + StudyEvents::quote(reason)
                + ",\"raw_sp_objective\":null,\"raw_sp_validation_pass\":null"
                + ",\"post_ls_objective\":null,\"post_ls_validation_pass\":null"
                + ",\"post_ls_skipped_reason\":" + StudyEvents::quote(reason) + ",\"feedback_candidate_id\":null");
            study->source = "HGA";
            data->isolatedSPCPU += StudyEvents::cpu() - isolatedStarted;
            data->isolatedSPStarted = -1;
            study->candidateId.clear();
            return;
        }
        if (combination.empty()) throw std::runtime_error("SP returned no complete incumbent");
        std::unique_ptr<Individual> candidate(new Individual());
        candidate->define(data);
        try {
            candidate->rebuildFromSP(combination);
            candidate->isRight();
        } catch (...) { ++study->invalidSPOutputs; throw; }
        ++sp_found_solutions;
        const double raw = candidate->fit;
        const std::string beforeConfig = study->configuration(candidate.get());
        recordSP(candidate.get(), "raw_SP");
        if (stopRequested()) {
            study->event("sp_completed", common + ",\"event_status\":\"completed\",\"raw_sp_objective\":" + StudyEvents::number(raw)
                + ",\"raw_sp_validation_pass\":true,\"post_ls_objective\":null,\"post_ls_validation_pass\":null"
                + ",\"post_ls_skipped_reason\":" + StudyEvents::quote(data->experimentTerminationReason)
                + ",\"candidate_config_before_ls\":" + beforeConfig
                + ",\"raw_improved_reported_best\":" + (raw < before ? "true" : "false")
                + ",\"post_ls_improved_reported_best\":null,\"feedback_candidate_id\":null");
            study->source = "HGA";
            data->isolatedSPCPU += StudyEvents::cpu() - isolatedStarted;
            data->isolatedSPStarted = -1;
            study->candidateId.clear();
            return;
        }
        study->source = "post_SP_LS";
        const double postCPUStarted = StudyEvents::cpu(), postWallStarted = StudyEvents::monotonic();
        bool isolated = false;
        if (data->experimentArm == "OneWaySP") {
            // Distinct LS/repair objects isolate object-local move state. TW LS
            // uses automatic candidate lists; Set S LS additionally shuffles
            // shared edges, restored below along with the main RNG.
            const auto savedRng = global_rng;
            const auto savedEdges = data->edgeNode;
            const int savedDiagnosticLoops = data->hardTimeWindows ? 0 : data->loopp;
            const auto savedConfigurations = data->compactLocConfigure;
            const int savedNoImprovement = data->nbIterNonImpro;
            const int savedIteration = data->iteration;
            const double savedGlobal = global->fit, savedRestart = bestIndivRestart->fit;
            const double savedPenaltyDepot = data->penaltyDepot, savedPenaltyVeh = data->penaltyVeh;
            const double savedPoolMean = routePool->getAverageFitness();
            const auto savedPoolSize = routePool->getPool().size();
            const auto savedCreation = popman->popCreationIteration;
            global_rng = sideRng;
            try {
                sideLS->local_search_run(candidate.get(), data->penaltyDepot, data->penaltyVeh);
                sideRng = global_rng;
            } catch (...) {
                global_rng = savedRng;
                if (!data->hardTimeWindows) {
                    data->edgeNode = savedEdges;
                    data->loopp = savedDiagnosticLoops;
                }
                throw;
            }
            global_rng = savedRng;
            // Set S LS shuffles the shared adjacency lists; its side search
            // must not change the order subsequently seen by ordinary HGA.
            if (!data->hardTimeWindows) {
                data->edgeNode = savedEdges;
                data->loopp = savedDiagnosticLoops;
            }
            isolated = global_rng == savedRng && data->edgeNode == savedEdges
                && data->compactLocConfigure == savedConfigurations && data->nbIterNonImpro == savedNoImprovement
                && data->iteration == savedIteration && global->fit == savedGlobal && bestIndivRestart->fit == savedRestart
                && data->penaltyDepot == savedPenaltyDepot && data->penaltyVeh == savedPenaltyVeh
                && routePool->getAverageFitness() == savedPoolMean && routePool->getPool().size() == savedPoolSize
                && popman->popCreationIteration == savedCreation;
            if (!isolated) throw std::runtime_error("OneWaySP post-LS changed shared HGA decision state");
        } else ls->local_search_run(candidate.get(), data->penaltyDepot, data->penaltyVeh);
        const double postCPU = StudyEvents::cpu() - postCPUStarted;
        const double postWall = StudyEvents::monotonic() - postWallStarted;
        postSPCPU += postCPU;
        try { candidate->isRight(); }
        catch (...) { ++study->invalidSPOutputs; throw; }
        candidate->writeStudyWitness("sp_post_ls_" + std::to_string(data->iteration) + ".txt");
        const double post = candidate->fit;
        const std::string afterConfig = study->configuration(candidate.get());
        const double beforePost = reportedBest->fit;
        recordSP(candidate.get(), "post_SP_LS");
        const std::string feedbackId = "feedback_" + std::to_string(data->iteration);
        study->event("sp_completed", common + ",\"event_status\":\"completed\",\"raw_sp_objective\":" + StudyEvents::number(raw)
            + ",\"raw_sp_validation_pass\":true,\"post_ls_objective\":" + StudyEvents::number(post)
            + ",\"post_ls_validation_pass\":true,\"candidate_config_before_ls\":" + beforeConfig
            + ",\"candidate_config_after_ls\":" + afterConfig
            + ",\"raw_improved_reported_best\":" + (raw < before ? "true" : "false")
            + ",\"post_ls_improved_reported_best\":" + (post < beforePost ? "true" : "false")
            + ",\"post_ls_cpu_seconds\":" + StudyEvents::number(postCPU)
            + ",\"post_ls_wall_seconds\":" + StudyEvents::number(postWall)
            + ",\"feedback_candidate_id\":" + StudyEvents::quote(feedbackId));

        bool matched = false;
        for (const auto& configuration : data->compactLocConfigure) {
            std::set<int> actual;
            for (int l = 0; l < data->numL; ++l) if (candidate->location[l].used) actual.insert(l + data->numV);
            if (configuration == actual) { matched = true; break; }
        }
        int newSlot = -1;
        const double globalBeforeAdmission = global->fit;
        const double candidateWall = study->wall();
        const double candidateCPU = StudyEvents::cpu();
        data->isolatedSPCPU += candidateCPU - isolatedStarted;
        data->isolatedSPStarted = -1;
        study->candidateId = feedbackId;
        std::string targetId, installedId;
        if (data->experimentArm == "OneWaySP") {
            popman->lastSPDecision = "disabled_oneway";
            popman->lastSPTarget = -1;
            popman->lastSPTargetMean = popman->lastSPTargetBest = std::numeric_limits<double>::quiet_NaN();
        } else {
            ++sp_injection_attempts;
            newSlot = popman->injectSPSolution(candidate.release(), routePool);
            targetId = popman->lastSPTargetId;
            if (newSlot >= 0) installedId = popman->studyPopulationId(newSlot);
        }
        int attempts = 0, feasibleConstructed = 0, admitted = 0;
        double constructionCPU = 0;
        if (newSlot >= 0) {
            study->source = "new_population";
            const double constructionStarted = StudyEvents::cpu();
            int twFillAttempts = 0;
            while ((int)popman->popula[newSlot].size() < data->popMin &&
                   (!data->hardTimeWindows || ++twFillAttempts <= data->popMin) && !stopRequested()) {
                ++attempts;
                ++study->physicalAttempts;
                initialSol->initial_solution(temIndiv, newSlot);
                if (!temIndiv->isFeasible) continue;
                ls->local_search_run(temIndiv, data->penaltyDepot, data->penaltyVeh);
                if (temIndiv->isFeasible) ++feasibleConstructed;
                popman->addIndividual(temIndiv, newSlot, false, routePool);
                if (popman->lastAddAccepted) ++admitted;
            }
            getAverageValue(newSlot);
            long long hash1 = 0, hash2 = 0;
            for (int depot : data->compactLocConfigure[newSlot]) {
                const int local = depot - data->numV;
                const long long bit = std::pow(2, local);
                hash1 += bit;
                hash2 += bit + 9527 * local;
            }
            data->dimTabu1[hash1 % data->dimension] = newSlot;
            data->dimTabu2[hash2 % data->dimension] = newSlot;
            constructionCPU = StudyEvents::cpu() - constructionStarted;
            newPopulationCPU += constructionCPU;
        }
        study->event("feedback", "\"feedback_candidate_id\":" + StudyEvents::quote(feedbackId)
            + ",\"call_id\":" + StudyEvents::quote(call)
            + ",\"candidate_objective\":" + StudyEvents::number(post)
            + ",\"global_before\":" + StudyEvents::number(globalBeforeAdmission)
            + ",\"candidate_wall_s\":" + StudyEvents::number(candidateWall)
            + ",\"candidate_cpu_s\":" + StudyEvents::number(candidateCPU)
            + ",\"candidate_configuration\":" + afterConfig
            + (data->hardTimeWindows ? ",\"tw_valid\":true" : ",\"capacity_valid\":true")
            + ",\"decision\":" + StudyEvents::quote(popman->lastSPDecision)
            + ",\"matched_existing_configuration\":" + (matched ? "true" : "false")
            + ",\"target_population_id\":" + (targetId.empty() ? "null" : StudyEvents::quote(targetId))
            + ",\"target_mean_objective\":" + StudyEvents::number(popman->lastSPTargetMean)
            + ",\"target_best_objective\":" + StudyEvents::number(popman->lastSPTargetBest)
            + ",\"installed_population_id\":" + (installedId.empty() ? "null" : StudyEvents::quote(installedId))
            + ",\"construction_attempts\":" + std::to_string(attempts)
            + (data->hardTimeWindows ? ",\"tw_feasible_constructed\":" : ",\"feasible_constructed\":") + std::to_string(feasibleConstructed)
            + ",\"admitted_members\":" + std::to_string(admitted)
            + ",\"construction_cpu_seconds\":" + StudyEvents::number(constructionCPU)
            + ",\"install_counter\":" + (newSlot < 0 ? "null" : std::to_string(data->iteration))
            + ",\"isolation_check_pass\":" + (data->experimentArm == "OneWaySP" ? (isolated ? "true" : "false") : "null"));
        study->source = "HGA";
        study->candidateId.clear();
    } catch (...) {
        if (data->isolatedSPStarted >= 0) {
            data->isolatedSPCPU += StudyEvents::cpu() - data->isolatedSPStarted;
            data->isolatedSPStarted = -1;
        }
        study->candidateId.clear();
        study->event("sp_error", common + ",\"event_status\":\"error\"");
        throw;
    }
}

solution::solution(read_data* data, crossover* cross, initial_sol* initialSol, LS* ls,
                   popManager* popman, RepairProcedure* repair, mutation* mutationOperator)
    : data(data), initialSol(initialSol), ls(ls), cross(cross), popman(popman),
      repair(repair), mutationOperator(mutationOperator) {
    auto individual = [&]() {
        std::unique_ptr<Individual> value(new Individual());
        value->define(data);
        value->initilization();
        return value;
    };
    auto temporary = individual();
    auto globalBest = individual();
    auto localBest = individual();
    auto restartBest = individual();
    globalBest->fit = restartBest->fit = std::numeric_limits<double>::infinity();
    std::unique_ptr<Individual[]> offspring(new Individual[100]);
    for (int i = 0; i < 100; ++i) { offspring[i].define(data); offspring[i].initilization(); }
    std::unique_ptr<RoutePool> pool(new RoutePool(data));
    std::unique_ptr<SPSolver> solver(data->experimentMode && data->experimentArm == "NoSP"
        ? nullptr : new SPSolver(data));
    std::unique_ptr<StudyEvents> events;
    std::unique_ptr<Individual> reported;
    std::unique_ptr<LS> isolatedLS;
    std::function<void(Individual*)> callback;
    if (data->experimentMode) {
        events.reset(new StudyEvents(data));
        reported = individual();
        reported->fit = std::numeric_limits<double>::infinity();
        callback = [this](Individual* candidate) { recordHGA(candidate); };
        if (data->experimentArm == "OneWaySP") {
            isolatedLS.reset(new LS(data));
            sideRng = global_rng;
        }
    }
    temIndiv = temporary.release(); global = globalBest.release(); loc_best = localBest.release();
    bestIndivRestart = restartBest.release(); subPop = offspring.release();
    routePool = pool.release(); spSolver = solver.release();
    study = events.release(); reportedBest = reported.release(); sideLS = isolatedLS.release();
    popman->global = global; popman->bestIndivRestart = bestIndivRestart;
    popman->study = study;
    popman->studyIncumbent.swap(callback);
    if (spSolver) spSolver->study = study;
}
solution::~solution(){
	popman->study = nullptr;
	popman->studyIncumbent = nullptr;
	delete sideLS;
	delete reportedBest;
	delete study;
	delete temIndiv;
	delete global;
	delete bestIndivRestart;
	delete [] subPop;
	delete loc_best;
	// === 释放 Matheuristic 组件 ===
    if (this->routePool != nullptr) {
        delete this->routePool;
    }
    if (this->spSolver != nullptr) {
        delete this->spSolver;
    }
}
void solution::preprocessingTimeWindows() {
    // Set F: screen only complete TW-feasible constructions. Capacity alone
    // neither certifies a configuration nor justifies an unbounded fill loop.
    data->compactLocConfigure.clear();
    for (const auto& config : data->LocConfigure)
        data->compactLocConfigure.push_back(config.second);
    std::map<std::set<int>, double> discovered;
    const int candidates = std::min(data->configurationCandidateLimit, static_cast<int>(data->compactLocConfigure.size()));
    for (int candidate = -1; candidate < candidates; ++candidate) {
        // Start with unrestricted construction so a tight budget can retain a witness.
        const int rank = candidate < 0 ? static_cast<int>(data->compactLocConfigure.size()) : candidate;
        for (int sample = 0; sample < data->configurationSamples; ++sample) {
            if (stopRequested()) break;
            if (study) ++study->physicalAttempts;
            initialSol->initial_solution(temIndiv, rank);
            if (!temIndiv->isFeasible) continue;
            ls->local_search_run(temIndiv, data->penaltyDepot, data->penaltyVeh);
            std::set<int> actual;
            for (int l = 0; l < data->numL; ++l)
                if (temIndiv->location[l].used) actual.insert(l + data->numV);
            auto entry = discovered.emplace(actual, temIndiv->fit);
            entry.first->second = std::min(entry.first->second, temIndiv->fit);
            routePool->extractRoutes(temIndiv);
            if (temIndiv->fit < global->fit) {
                popman->copyAllData(global, temIndiv);
                popman->copyAllData(bestIndivRestart, temIndiv);
                data->meetBestTime = static_cast<double>(clock()) / CLOCKS_PER_SEC;
                recordHGA(global);
            }
        }
        if (stopRequested()) break;
    }
    if (discovered.empty() && study) {
        noFeasibleEnd = true;
        data->experimentTerminationReason = "no_feasible_end";
        return;
    }
    if (discovered.empty())
        throw std::runtime_error("Set F construction found no feasible solution within the attempt/time budget; infeasibility is not proved");
    std::vector<std::pair<double, std::set<int>>> ranked;
    for (const auto& item : discovered) ranked.push_back({item.second, item.first});
    std::sort(ranked.begin(), ranked.end());
    data->compactLocConfigure.clear();
    for (int i = 0; i < std::min(data->maxActiveConfigurations, static_cast<int>(ranked.size())); ++i)
        data->compactLocConfigure.push_back(ranked[i].second);
    data->numLocConfiguration = static_cast<int>(data->compactLocConfigure.size());
    popman->addIndividual(global, data->numLocConfiguration, true, routePool);
    std::cout << "[Set F] Feasible configurations: " << data->numLocConfiguration << std::endl;
}

void solution::preprocessingLocationConfiguration(){
    if (data->hardTimeWindows) { preprocessingTimeWindows(); return; }
	data->compactLocConfigure.clear();
	//int initalNumDepots = (int)data->LocConfigure.size();
	int initalNumDepots = std::min(data->configurationCandidateLimit, (int)data->LocConfigure.size());
	if ((int)data->LocConfigure.size() < initalNumDepots)
		initalNumDepots = (int)data->LocConfigure.size();
	for (int i=0;i<initalNumDepots; i++)
		data->compactLocConfigure.push_back(data->LocConfigure[i].second);

	if (data->no_capacity_location){
		data->numLocConfiguration = 0;
		return;// if no depot configurations, jump
	}

	std::vector<std::pair<double, int>>averageCost;
	double bestCost = std::numeric_limits<double>::infinity();
	double global_cost = std::numeric_limits<double>::infinity();
	for (int i = 0; i<initalNumDepots;i++){// an extra subPop
        bestCost = std::numeric_limits<double>::infinity();
		for (int j = 0; j < data->configurationSamples; j++){
            if (data->stopCondition(global_cost)) break;
            if (study) ++study->physicalAttempts;
			initialSol->initial_solution(temIndiv, i);
            if (!temIndiv->isFeasible) continue;
			ls->local_search_run(temIndiv,data->penaltyDepot, data->penaltyVeh);
			if (bestCost > temIndiv->fit)
				bestCost = temIndiv->fit;
			if (temIndiv->fit < global_cost){
				// to record the best solution given each depot configuration
				global_cost = temIndiv->fit;
				loc_best->initilization();
				popman->copyAllData(loc_best, temIndiv);// record the best solution given the current depot configuration.
			}
		}
        if (std::isfinite(bestCost)) averageCost.push_back({bestCost, i});
		std::cout<<i<<"   tries   "<< bestCost<<std::endl;
        if (data->stopCondition(global_cost)) break;

//		loc_best->evaluationDis();
	}
    if (averageCost.empty()) throw std::runtime_error("initial screening found no feasible solution within its budget");
	std::sort(averageCost.begin(), averageCost.end());
	loc_best->evaluationDis();

	//
	data->numLocConfiguration = data->maxActiveConfigurations;
	data->compactLocConfigure.clear();
	for (int i=0;i < std::min(data->numLocConfiguration, static_cast<int>(averageCost.size())); i++)
		data->compactLocConfigure.push_back(data->LocConfigure[averageCost[i].second].second);
	data->numLocConfiguration = std::min(data->numLocConfiguration, static_cast<int>(averageCost.size()));
	// add the best solution of each depot configuration in the elite subpop.
//	loc_best->outputBestSolution();
	popman->addIndividual(loc_best, data->numLocConfiguration, true);// insert the local best solution to elite pop
	std::cout<<"Finishing initial filter"<<std::endl;
}
void solution::generation_population(int rankPop, bool isInitial){
	double bestFit=std::numeric_limits<double>::infinity();
	/*generate a feasible population set*/
	for (int i = 0; i < data->popMin * data->initialPopulationMultiplier; i++){
		if (stopRequested()) break;
		if (study) ++study->physicalAttempts;
		initialSol->initial_solution(temIndiv, rankPop);
		if (!temIndiv->isFeasible) continue;
		ls->local_search_run(temIndiv,data->penaltyDepot, data->penaltyVeh);
		popman->addIndividual(temIndiv, rankPop, isInitial, this->routePool);
		if (bestFit>temIndiv->fit)
			bestFit=temIndiv->fit;
//		if (i % 100 == 0)
//			std::cout<<i<<"  "<<bestFit<<std::endl;
	}
	std::cout<<"The best fit is    "<<bestFit<<"  The time consuming  "<<(double)clock()/(double)CLOCKS_PER_SEC<<std::endl;
}
void solution::outputprocess(Individual *indiv){
	std::cout<<"It: "<<data->iteration<<"--G-Fit: "<<std::setiosflags(ios::fixed)<<std::setprecision(2)<<float(global->fit);
	std::cout<<"  --R-Fit: "<<bestIndivRestart->fit;
//	std::cout<<"    Avg F "<<float(popman->averagePop())<<std::setprecision(2);
	std::cout<<"  Curr Sol--"<<float(indiv->fit)<<std::setprecision(2);
	std::cout<<"  IT NoImpro--"<<data->nbIterNonImpro;
	std::cout<<"  Time comsuming--"<<float((double)clock()/(double)CLOCKS_PER_SEC)<<std::setprecision(2);
//	std::cout<<"  used depots--"<<popman->usedDepots();
//	double ratio = data->loopp / 1000.;
//	std::cout<<"  overSat. Loc "<< ratio;
//	std::cout<<"  CostProb "<<data->probFixedCost<<std::endl;
	std::cout<<std::endl;
	data->loopp = 0;
}
void solution::getAverageValue(int rank){
    if (data->hardTimeWindows) {
        // Admission follows the actual depot set, so a caller's requested rank
        // may differ from the destination. Refresh the small F cohort together.
        avgCostPop.assign(data->numLocConfiguration + 1, Max);
        for (int p = 0; p <= data->numLocConfiguration; ++p) {
            if (popman->popula[p].empty()) continue;
            long double sum = 0;
            for (const Individual* item : popman->popula[p]) sum += item->fit;
            avgCostPop[p] = sum / popman->popula[p].size();
        }
        return;
    }
	if (rank < 0){
		avgCostPop.clear();
		for (int i=0;i<data->numLocConfiguration + 1; i++){
			long double cost = 0;
			for (int j=0;j<(int)popman->popula[i].size();j++){
				cost += popman->popula[i][j]->fit;
			}
			int ss = popman->popula[i].size();
		//	avgCostPop.push_back(cost);
			avgCostPop.push_back(ss ? cost / (double)ss : std::numeric_limits<double>::infinity());
		}
		//cut some pop whose obj is too worse.
/*		double min_avg = Max;
		for (int i=0;i <data->numLocConfiguration + 1; i++){
			if (min_avg > avgCostPop[i])
				min_avg = avgCostPop[i];
		}
		for (int i=0;i <data->numLocConfiguration + 1; i++){
			if (100*(avgCostPop[i] - min_avg) / min_avg > 5){
				for (Individual * indiv : popman->popula[i])
					delete indiv ;
				popman->popula[i].clear();
			}
		}
		*/
		//removing half subpop
		std::vector<std::pair<double,int>>rankOrder;
		for (int i=0;i<(int)avgCostPop.size();i++)
			rankOrder.push_back({-avgCostPop[i],i});
		sort(rankOrder.begin(),rankOrder.end());
		for (int i=0;i<data->numLocConfiguration / 2; i++){
			int subpop = rankOrder[i].second;
			for (Individual * indiv : popman->popula[subpop]) delete indiv ;
			popman->popula[subpop].clear();
		}
		//
		long long int var, var1, var2;;int seed;
		for (int i=0;i<data->numLocConfiguration;i++){
			if ((int)popman->popula[i].size() > 0){
				var = 0;var1 = 0;
				auto it = data->compactLocConfigure[i].begin();
				for (;it != data->compactLocConfigure[i].end(); it++){
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
	else{
		long double cost = 0;
		for (int j=0;j<(int)popman->popula[rank].size();j++)
			cost += popman->popula[rank][j]->fit;
		int ss = popman->popula[rank].size();
		avgCostPop[rank] = ss ? cost / (double)ss : std::numeric_limits<double>::infinity();
	//	avgCostPop[rank] = cost;
	}
	// removing half population

}
int solution::selectsubPop(int subpop1){
    if (data->hardTimeWindows) {
        std::vector<int> eligible;
        for (int p = 0; p <= data->numLocConfiguration; ++p)
            if (!popman->popula[p].empty() && p != subpop1) eligible.push_back(p);
        if (eligible.empty()) return subpop1;
        int a = eligible[RandomInt(0, static_cast<int>(eligible.size()) - 1)];
        int b = eligible[RandomInt(0, static_cast<int>(eligible.size()) - 1)];
        return avgCostPop[a] < avgCostPop[b] ? a : b;
    }
	int rand1,rand2;
	int target;
	std::vector<int>locConf;

	for (int i = 0; i < data->numLocConfiguration + 1; i++) {
        if ((int)popman->popula[i].size() > 0)
            locConf.push_back(i);
    }

	if (locConf.empty()) return -1;

	if ((int)locConf.size() == 1) return locConf[0];
    if ((int)locConf.size() == 2) {
        return (locConf[0] == subpop1) ? locConf[1] : locConf[0]; // 修复下标越界
    }

	while(1){
        rand1 = RandomInt(0, locConf.size() - 1);
		rand2 = RandomInt(0, locConf.size() - 1);

        if ((int)locConf.size() <= 3) return locConf[rand1];

        while(rand1 == rand2)
            rand2 = RandomInt(0, locConf.size() - 1);

        if (avgCostPop[locConf[rand1]] < avgCostPop[locConf[rand2]])
            target = locConf[rand1];
        else
            target = locConf[rand2];

        // 检查选择的子种群是否符合演化要求
        if (target == subpop1 && (int)popman->popula[target].size() > 2) break;
        if (target != subpop1) break;

        // 兜底逻辑
        if ((int)locConf.size() == 2) {
            target = (locConf[0] == subpop1) ? locConf[1] : locConf[0];
            break;
        }
    }
    return target;
}
void solution::evolvePop(int rankPop, int &count){
    if (stopRequested()) {
        count = data->maxIterationPerLoc; return;
    }
	if ((int)popman->popula[rankPop].size() <= 2){
		count = data->maxIterationPerLoc;
		return;
	}
	int sizeSubPop = 0;
	ma = NULL;
	fa = popman->selectionParentSolution(ma, rankPop);
	ma = popman->selectionParentSolution(fa, rankPop);
	cross->gEAX(fa,ma,subPop,sizeSubPop);
    if (sizeSubPop == 0) { ++count; if (study) ++study->physicalAttempts; }
	for (int i =0; i<sizeSubPop; i++){
		if (stopRequested()) break;
		if (study) ++study->physicalAttempts;
		repair->repairCapacityConstraint(subPop+i, data->penaltyDepot, data->penaltyVeh);
		if (subPop[i].isFeasible){
		//	if (rand()%100 > data->mutationProb)
		//		mutationOperator->mutationRun(subPop+i);
			ls->local_search_run(subPop+i, data->penaltyDepot, data->penaltyVeh);
			popman->addIndividual(subPop+i, rankPop, true);
		}
	//	if (data->iteration % 500 == 0)	outputprocess(subPop+i);
	//	data->iteration++;
		count++;
	}
}
void solution::initialPop(){
	//phase 1: to construct a number of populations
//	data->compactLocConfigure.clear();
//	if ((int)data->LocConfigure.size() < data->numLocConfiguration)
//		data->numLocConfiguration = (int)data->LocConfigure.size();
//	for (int i=0;i<data->numLocConfiguration; i++)
//		data->compactLocConfigure.push_back(data->LocConfigure[i].second);
	popman->popula.erase(popman->popula.begin()+data->numLocConfiguration +1, popman->popula.end());
	int i;
	for ( i = 0; i<data->numLocConfiguration +1 ;i++){// an extra subPop
        if (stopRequested()) break;
		std::cout<<i<<" th ";
		std::cout<<(double)clock()/(double)CLOCKS_PER_SEC<<std::endl;
		generation_population(i, true);
		int count = 0;
		if ((int)popman->popula[i].size() <= 1)
			continue;
		while (count < data->maxIterationPerLoc && !stopRequested())
			evolvePop(i, count);
	}
	std::cout<<"Finishing constructing number of initial population"<<std::endl;
}
void solution::main_loop(){
    if (data->hardTimeWindows || data->experimentMode) {
        data->iteration = 0;
        data->nbIterNonImpro = 0;
    }
	std::cout<<"---- Start the search----"<<std::endl;
	const double initializeStarted = StudyEvents::cpu();
	preprocessingLocationConfiguration();
	if (study && (noFeasibleEnd || stopRequested())) {
		initializationCPU = StudyEvents::cpu() - initializeStarted;
		finishRun(); return;
	}
	int sizeSubPop = 0;
	data->loopp = 0;
	data->nbIterNonImpro=1;
	int subPop1, subPop2;
	initialPop();
	initializationCPU = StudyEvents::cpu() - initializeStarted;
	if (stopRequested()) { finishRun(); return; }

	// 【新增】：初始化之后，迭代开始之前，全员注入路径池建立均值基线
	std::cout << "[RoutePool] 正在建立初始精英池基线..." << std::endl;
	for (int i = 0; i < data->numLocConfiguration + 1; i++) {
		for (Individual* indiv : popman->popula[i]) {
			if (indiv->isFeasible) {
				this->routePool->extractRoutes(indiv);              // 提取路径入池
				this->routePool->recordSolutionFitness(indiv->fit); // 记录成本以计算均值
			}
		}
	}
	std::cout << "[RoutePool] 初始路径池建立完成! 均值门槛: " << std::fixed << this->routePool->getAverageFitness() << std::endl;

	getAverageValue(-1);// for using to select some locations
	//	popman->removeEmptySubPop();

	data->iteration=0;
	int isGenerateNewSubPop;
	std::vector<int>depot;
	long long int var1, var2, var;
	int whichSubPop;
	if (study) study->source = "HGA";
    bool reducedHalf = false, reducedThreeQuarters = false;
	while(1){
        if (stopRequested()) {
            finishRun(); return;
        }
		// phase 2: to crossover individuals from different subpopulation;
		subPop1 = selectsubPop(-1);
		subPop2 = selectsubPop(subPop1);
        if (subPop1 < 0 || subPop2 < 0) throw std::runtime_error("search has no nonempty population");
		ma = NULL;
		fa = popman->selectionParentSolution(ma, subPop1);
		ma = popman->selectionParentSolution(fa, subPop2);
        sizeSubPop = 0;
		cross->gEAX(fa,ma,subPop,sizeSubPop);
        if (data->hardTimeWindows && sizeSubPop == 0) {
            // A failed/empty EAX proposal must not reuse stale offspring. The
            // retained parent passes through the ordinary repair/mutation path.
            popman->copyAllData(subPop, fa);
            sizeSubPop = 1;
        }
		for (int i =0; i<sizeSubPop; i++){
			if (stopRequested()) { finishRun(); return; }
			if (study) ++study->physicalAttempts;
			repair->repairCapacityConstraint(subPop+i, data->penaltyDepot, data->penaltyVeh);
			if (subPop[i].isFeasible){
				if (study) ++study->mutationDecisions;
				if (RandomInt(0, 99) >= data->mutationProb) {
					if (study) ++study->mutationsExecuted;
					mutationOperator->mutationRun(subPop+i);
				}
				ls->local_search_run(subPop+i, data->penaltyDepot, data->penaltyVeh);

				// 1. 无差别收割高质量可行解的路径（作为数学规划的列）
				// 【删除无差别收割逻辑，路径提取已后置到 addIndividual 中严格把关】

				// 2. 然后再处理子种群的归属和哈希映射
				if (!data->no_capacity_location){
					depot.clear();
					for (int j=0; j<data->numL;j++){
						if (subPop[i].location[j].used)
							depot.push_back(j);
					}
					var1 = 0; var2 = 0;
					for (int j=0;j<(int)depot.size();j++){
						var = pow(2, depot[j]);
						var1 += var;
						var2 += var + 9527 * depot[j];
					}
					if (data->dimTabu1[var1 % data->dimension] == data->dimTabu2[var2 % data->dimension] && data->dimTabu2[var2 % data->dimension] > -1)
						whichSubPop = data->dimTabu2[var2 % data->dimension];
					else
						whichSubPop = data->numLocConfiguration;

					isGenerateNewSubPop = popman->addIndividual(subPop+i, whichSubPop, false, this->routePool);
					getAverageValue(whichSubPop);

					if (isGenerateNewSubPop > -1){
						const double fillStarted = StudyEvents::cpu();
						if (study) study->source = "new_population";
						// 【修改3A：游离种群成功接管后，快速精准扩充到 popMin】
						int twFillAttempts = 0;
                            while ((int)popman->popula[isGenerateNewSubPop].size() < data->popMin &&
                                   (!data->hardTimeWindows || (++twFillAttempts <= data->popMin && !stopRequested())) &&
                                   !stopRequested()) {
							if (study) ++study->physicalAttempts;
							initialSol->initial_solution(temIndiv, isGenerateNewSubPop);
                                if (!temIndiv->isFeasible) continue;
							ls->local_search_run(temIndiv, data->penaltyDepot, data->penaltyVeh);
							popman->addIndividual(temIndiv, isGenerateNewSubPop, false, this->routePool);
						}
						getAverageValue(isGenerateNewSubPop);
						if (study) { newPopulationCPU += StudyEvents::cpu() - fillStarted; study->source = "HGA"; }

						// [修补机制] 基于刚写入的真实车场配置重新计算哈希
						long long int correct_var = 0, correct_var1 = 0, correct_var2 = 0;
						for (auto it = data->compactLocConfigure[isGenerateNewSubPop].begin(); it != data->compactLocConfigure[isGenerateNewSubPop].end(); it++) {
							int seed_val = *it - data->numV;
							correct_var = pow(2, seed_val);
							correct_var1 += correct_var;
							correct_var2 += correct_var + 9527 * seed_val;
						}
						data->dimTabu1[correct_var1 % data->dimension] = isGenerateNewSubPop;
						data->dimTabu2[correct_var2 % data->dimension] = isGenerateNewSubPop;
					}
				}
				else{
					popman->addIndividual(subPop+i, 0, false);
				}

				/////////////////////////////////////
				if (data->iteration % 1000 == 0)outputprocess(subPop+i);

				// 终端打印逆袭机制的生效次数及游离种群规模
				if (data->iteration > 0 && data->iteration % 50000 == 0) {
					std::cout << ">>> [机制监控] \"gamma+1\" 游离种群动态逆袭替换机制累计生效: "
							<< popman->reverseReplacementCount << " 次 | "
							<< "当前游离种群个体数量: "
							<< popman->popula[data->numLocConfiguration].size() << std::endl;
				}

				//	if (data->iteration > 480000)
				//		outputprocess(subPop+i);
				data->iteration++;

				// 激活混合元启发式 (Matheuristic) 机制，基于预定周期调用集合划分求解器
				if (study) {
					// A validated optimal HGA endpoint is allowed to finish before
					// a scheduled SP call; do not manufacture SP exposure.
					if (stopRequested()) { finishRun(); return; }
					if (std::binary_search(data->spSchedule.begin(), data->spSchedule.end(), data->iteration)) runSP();
					if (stopRequested()) { finishRun(); return; }
				} else if (data->iteration > 0 && data->iteration % data->spTriggerFrequency == 0) {
					this->sp_total_triggers++;
					int current_pool_size = this->routePool->getPool().size();

					std::cout << "\n[SP MONITOR] Iteration " << data->iteration
					          << ": Triggered #" << this->sp_total_triggers
					          << " | RoutePool Size: " << current_pool_size << std::endl;

					const double globalBeforeSP = global->fit;
					std::vector<StandaloneRoute> optimalCombination = this->spSolver->solve(this->routePool, global);

					if (current_pool_size < 10) {
						std::cout << "[SP Warning] RoutePool is too small (" << current_pool_size << "). Gurobi might not find any improvement." << std::endl;
					}

					if (!optimalCombination.empty()) {
						this->sp_found_solutions++;
						std::cout << "[SP Success] Gurobi found a valid configuration! (Total found: " << this->sp_found_solutions << ")" << std::endl;

						std::unique_ptr<Individual> spIndiv(new Individual());
						spIndiv->define(this->data);
						spIndiv->rebuildFromSP(optimalCombination);
						const double rawSPObjective = spIndiv->fit;
						const auto spLSStarted = std::chrono::steady_clock::now();

						// 【新增】：对 Gurobi 拼装出的解进行一次 LS 打磨，避免虚高
						this->ls->local_search_run(spIndiv.get(), data->penaltyDepot, data->penaltyVeh);
						const double postLSObjective = spIndiv->fit;
						const double spLSWall = std::chrono::duration<double>(std::chrono::steady_clock::now() - spLSStarted).count();

						this->sp_injection_attempts++;

						// 【修改】：接收返回值，判断是否开辟了新种群，并传入 pool 用于均值记录
						int spNewPopIdx = this->popman->injectSPSolution(spIndiv.release(), this->routePool);
						std::ostringstream injectionEvent;
						injectionEvent << std::setprecision(17) << "[SP INJECTION] iteration=" << data->iteration
						               << " global_before=" << globalBeforeSP << " raw_objective=" << rawSPObjective
						               << " post_ls_objective=" << postLSObjective << " global_after=" << global->fit
						               << " ls_wall_seconds=" << spLSWall << " new_population_index=" << spNewPopIdx;
						std::cout << injectionEvent.str() << std::endl;

						// 【新增】：核心联动孵化逻辑
						if (spNewPopIdx > -1) {
							std::cout << "[SP Incubation] Rapidly expanding population to popMin for new SP configuration [" << spNewPopIdx << "]..." << std::endl;

							// 【修改3B：SP种群成功接管后，快速精准扩充到 popMin】
							int twFillAttempts = 0;
                            while ((int)popman->popula[spNewPopIdx].size() < data->popMin &&
                                   (!data->hardTimeWindows || ++twFillAttempts <= data->popMin) && !stopRequested()) {
								initialSol->initial_solution(temIndiv, spNewPopIdx);
                                if (!temIndiv->isFeasible) continue;
								ls->local_search_run(temIndiv, data->penaltyDepot, data->penaltyVeh);
								popman->addIndividual(temIndiv, spNewPopIdx, false, this->routePool);
							}
							getAverageValue(spNewPopIdx);

							// 2. 刷新全局哈希表（照搬你的原版游离种群防护逻辑）
							long long int correct_var = 0, correct_var1 = 0, correct_var2 = 0;
							for (auto it = data->compactLocConfigure[spNewPopIdx].begin(); it != data->compactLocConfigure[spNewPopIdx].end(); it++) {
								int seed_val = *it - data->numV;
								correct_var = pow(2, seed_val);
								correct_var1 += correct_var;
								correct_var2 += correct_var + 9527 * seed_val;
							}
							data->dimTabu1[correct_var1 % data->dimension] = spNewPopIdx;
							data->dimTabu2[correct_var2 % data->dimension] = spNewPopIdx;
						}
					} else {
						std::cout << "[SP Info] Gurobi finished but no better set partitioning was found.\n" << std::endl;
					}
				}

				/*re-start procedure*/
				if (data->nbIterNonImpro > data->maxIterNonProd){
					// 1. 计算剩余时间（秒）
					double elapsedTime = (double)(clock() - data->startTime) / CLOCKS_PER_SEC - data->isolatedSPCPU;
					double remainingTime = data->timeLimit - elapsedTime;

					// 2. 计算剩余迭代次数
					int remainingIters = data->iterLimit - data->iteration;

					// Family-specific minimum budget for a useful restart.
					if (remainingIters < data->restartMinRemainingIterations ||
					    remainingTime < data->restartMinRemainingSeconds) {
						std::cout << "  [Intercepted] Restart aborted. Remaining Iters: " << remainingIters
								  << ", Remaining Time: " << remainingTime << "s. Focused search continued." << std::endl;

						// 重置无改进计数器，避免在接下来的每一步都重复进入此分支并频繁打印
						data->nbIterNonImpro = 0;
					} else {
						// 资源充足，执行正常的大重启
						std::cout << "  [Triggered] Executing Global Restart..." << std::endl;
						popman->reStart();
						const double restartStarted = StudyEvents::cpu();
						if (study) study->source = "new_population";
						initialPop();
						if (study) { newPopulationCPU += StudyEvents::cpu() - restartStarted; study->source = "HGA"; }
					}
				}
				data->nbIterNonImpro++;
			}
            else if (data->hardTimeWindows || data->experimentMode) {
                ++data->iteration;
                if (study && std::binary_search(data->spSchedule.begin(), data->spSchedule.end(), data->iteration)) {
                    reachedSPSchedule.insert(data->iteration);
                    ++sp_total_triggers;
                    study->event("sp_opportunity", "\"call_id\":\"sp_" + std::to_string(data->iteration)
                        + "\",\"scheduled_counter\":" + std::to_string(data->iteration)
                        + ",\"actual_counter\":" + std::to_string(data->iteration)
                        + ",\"event_status\":\"skipped\",\"skip_reason\":\"no_feasible_offspring_branch\""
                        + ",\"nominal_time_limit_wall_seconds\":" + StudyEvents::number(data->spTimeLimit));
                }
            }
			///////////////////
			if (!data->hardTimeWindows && !reducedHalf && data->iteration == data->iterLimit/2) {
                reducedHalf = true;
				popman->reduceHalfSubPop();
            }

			if (!data->hardTimeWindows && !reducedThreeQuarters && data->iteration == data->iterLimit * 0.75) {
                reducedThreeQuarters = true;
				popman->reduceHalfSubPop();
            }

			/////////
			if (stopRequested()){
				finishRun();
				std::cout<<global->fit<<std::endl;
				std::cout<<"----The memetic algorithm is finished, time spends: "<<(double)clock()/(double)CLOCKS_PER_SEC<<std::endl;
				return;
			}
		}
	}
}
