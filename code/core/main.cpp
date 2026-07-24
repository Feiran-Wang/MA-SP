/******************************************************************************
 * File: main.cpp
 * Author: Wang
 * Date: 2026-06-24
 * Description: 
 *   Entry point of the program. Initializes the orchestrator and starts the algorithm.
 ******************************************************************************/

#include "basic.h"
#include "commandline.h"
#include "AlgorithmOrchestrator.h"
#include <memory>

std::mt19937 global_rng;

int main(int argc, char*argv[]){
	CommandLine comLin(argc,argv);
	global_rng.seed(comLin.seed);
	
	// Use unique_ptr for data
	std::unique_ptr<read_data> data(new read_data());
	data->read_fun(comLin.pathInstance, comLin.pathDepotCon, comLin.pathSolution, comLin.bestDis, comLin.isOptimal, comLin.timeLimit,comLin.iterLimit);
	data->startTime=clock();
	
	/***********************/
	// The AlgorithmOrchestrator takes complete ownership of instantiating components
	AlgorithmOrchestrator orchestrator(data.get());
	orchestrator.run();
	/*************************************************/

	return 0;
}
