/******************************************************************************
 * File: main.cpp
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description: 
 *   Entry point of the program. Initializes the orchestrator and starts the algorithm.
 ******************************************************************************/

#include "basic.h"
#include "commandline.h"
#include "AlgorithmOrchestrator.h"
#include "RunLogger.h"
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <random>
#include <string>

std::mt19937 global_rng;

int main(int argc, char*argv[]){
    if (argc == 2 &&
        (std::string(argv[1]) == "--help" ||
         std::string(argv[1]) == "-h")) {
        CommandLine::printUsage(std::cout, argv[0]);
        return EXIT_SUCCESS;
    }

    std::unique_ptr<hgasp::RunLogger> runLogger;
    try {
        CommandLine commandLine(argc, argv);
        runLogger = std::make_unique<hgasp::RunLogger>(
            commandLine.pathInstance,
            commandLine.seed);
        commandLine.displayInfo(std::cout);
        global_rng.seed(commandLine.seed);

        std::unique_ptr<read_data> data(new read_data());
        data->applyEnvironmentOverrides();
        if (data->spLogFile.empty()) {
            data->spLogFile =
                (runLogger->directory() / "gurobi.log").string();
        }
        data->read_fun(
            commandLine.pathInstance,
            commandLine.pathDepotCon,
            commandLine.pathSolution,
            commandLine.bestDis,
            commandLine.isOptimal,
            commandLine.timeLimit,
            commandLine.iterLimit);
        data->startTime = clock();

        AlgorithmOrchestrator orchestrator(data.get());
        orchestrator.run();
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "HGA-SP error: " << error.what() << "\n\n";
        CommandLine::printUsage(std::cerr, argv[0]);
        return EXIT_FAILURE;
    }
}
