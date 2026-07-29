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
        hgasp::RunLogger::event(
            hgasp::LogLevel::info,
            "lifecycle",
            "run.started",
            {},
            {
                {"instance", commandLine.pathInstance},
                {"seed", commandLine.seed},
                {"problem_mode",
                 hgasp::toString(commandLine.problemMode)},
                {"time_limit", commandLine.timeLimit},
                {"iteration_limit", commandLine.iterLimit}
            });
        hgasp::RunLogger::status(
            "lifecycle",
            "initializing",
            {
                {"instance", commandLine.pathInstance},
                {"seed", commandLine.seed}
            });
        commandLine.displayInfo(std::cout);
        global_rng.seed(commandLine.seed);

        std::unique_ptr<read_data> data(new read_data());
        data->applyEnvironmentOverrides();
        if (commandLine.eaxParentMode.has_value()) {
            data->eaxParentMode = *commandLine.eaxParentMode;
        }
        std::cout << "EAX parent mode: "
                  << hgasp::toString(data->eaxParentMode) << '\n';
        if (data->spLogFile.empty() &&
            runLogger->fileLoggingAvailable()) {
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
            commandLine.iterLimit,
            commandLine.problemMode);
        // Publication timing starts after instance parsing and includes HGA-SP
        // setup/search plus final solution validation through the timing
        // sample written to solution.txt.
        data->resetSearchCpuClock();

        AlgorithmOrchestrator orchestrator(data.get());
        hgasp::RunLogger::event(
            hgasp::LogLevel::info,
            "lifecycle",
            "search.started");
        hgasp::RunLogger::status(
            "lifecycle",
            "searching",
            {
                {"iteration", 0},
                {"problem_mode",
                 hgasp::toString(commandLine.problemMode)}
            });
        orchestrator.run();
        hgasp::RunLogger::event(
            hgasp::LogLevel::info,
            "lifecycle",
            "run.completed");
        hgasp::RunLogger::status(
            "lifecycle",
            "completed");
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        hgasp::RunLogger::event(
            hgasp::LogLevel::error,
            "lifecycle",
            "run.failed",
            error.what());
        hgasp::RunLogger::status(
            "lifecycle",
            "failed",
            {{"error", error.what()}});
        std::cerr << "HGA-SP error: " << error.what() << "\n\n";
        CommandLine::printUsage(std::cerr, argv[0]);
        return EXIT_FAILURE;
    }
}
