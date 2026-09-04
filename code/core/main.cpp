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
#include "ComponentProfiler.h"
#include "RunLogger.h"
#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <string>

std::mt19937 global_rng;

namespace {

std::string joinedConstraints(const hgasp::ProblemProfile& profile) {
    const std::vector<std::string> constraints =
        hgasp::activeConstraintIds(profile);
    std::ostringstream stream;
    for (std::size_t index = 0; index < constraints.size(); ++index) {
        if (index != 0U) {
            stream << ',';
        }
        stream << constraints[index];
    }
    return stream.str();
}

}  // namespace

int main(int argc, char*argv[]){
    if (argc == 2 &&
        (std::string(argv[1]) == "--help" ||
         std::string(argv[1]) == "-h")) {
        CommandLine::printUsage(std::cout, argv[0]);
        return EXIT_SUCCESS;
    }

    std::unique_ptr<hgasp::RunLogger> runLogger;
    std::unique_ptr<SolverContext> data;
    bool profilerConfigured = false;
    bool searchClockStarted = false;
    bool timingFinalEmitted = false;
    try {
        CommandLine commandLine(argc, argv);
        runLogger = std::make_unique<hgasp::RunLogger>(
            commandLine.pathInstance,
            commandLine.seed);
        hgasp::ComponentProfiler::instance()
            .configureFromEnvironment();
        profilerConfigured = true;
        hgasp::RunLogger::event(
            hgasp::LogLevel::info,
            "lifecycle",
            "run.started",
            {},
            {
                {"instance", commandLine.pathInstance},
                {"seed", commandLine.seed},
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

        data = std::make_unique<SolverContext>();
        {
            hgasp::ScopedComponentTimer timer(
                hgasp::TimedComponent::InstanceLoading);
            data->initializeRun(
                commandLine.pathInstance,
                commandLine.pathDepotCon,
                commandLine.pathSolution,
                commandLine.bestDis,
                commandLine.isOptimal,
                commandLine.timeLimit,
                commandLine.iterLimit);
        }
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
        const hgasp::ProblemProfile& profile = data->problemProfile();
        std::cout << "Problem family: " << profile.family << '\n'
                  << "Benchmark family: "
                  << hgasp::toString(profile.benchmarkFamily) << '\n'
                  << "Active constraints: "
                  << joinedConstraints(profile) << '\n'
                  << "Metric policy: "
                  << profile.reporting.metricPolicyId << '\n';
        hgasp::RunLogger::event(
            hgasp::LogLevel::info,
            "lifecycle",
            "instance.profile",
            {},
            {
                {"problem_family", profile.family},
                {"benchmark_family",
                 hgasp::toString(profile.benchmarkFamily)},
                {"active_constraints", joinedConstraints(profile)},
                {"metric_policy_id",
                 profile.reporting.metricPolicyId},
                {"reporting_policy_id",
                 profile.reporting.reportingPolicyId},
                {"instance_format", hgasp::toString(profile.format)}
            });
        // Publication timing starts after instance parsing and includes HGA-SP
        // setup/search plus final solution validation through the timing
        // sample written to solution.txt.
        data->resetSearchCpuClock();
        searchClockStarted = true;

        std::unique_ptr<AlgorithmOrchestrator> orchestrator;
        {
            hgasp::ScopedComponentTimer timer(
                hgasp::TimedComponent::AlgorithmSetup);
            orchestrator =
                std::make_unique<AlgorithmOrchestrator>(
                    data.get());
        }
        hgasp::RunLogger::event(
            hgasp::LogLevel::info,
            "lifecycle",
            "search.started");
        hgasp::RunLogger::status(
            "lifecycle",
            "searching",
            {
                {"iteration", 0},
                {"problem_family", profile.family},
                {"benchmark_family",
                 hgasp::toString(profile.benchmarkFamily)},
                {"active_constraints", joinedConstraints(profile)}
        });
        orchestrator->run();
        hgasp::emitHotPathCountersFinal(true);
        hgasp::emitComponentTimingFinal(
            static_cast<std::uint64_t>(
                std::max(0, data->iteration)),
            data->elapsedCpuSeconds(),
            true);
        timingFinalEmitted = true;
        hgasp::RunLogger::status(
            "lifecycle",
            "finalizing");
        orchestrator->shutdown();
        orchestrator.reset();
        hgasp::RunLogger::event(
            hgasp::LogLevel::info,
            "lifecycle",
            "solver_context.shutdown_started");
        data.reset();
        hgasp::RunLogger::event(
            hgasp::LogLevel::info,
            "lifecycle",
            "solver_context.shutdown_completed");
        hgasp::RunLogger::event(
            hgasp::LogLevel::info,
            "lifecycle",
            "run.completed");
        hgasp::RunLogger::status(
            "lifecycle",
            "completed");
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        if (profilerConfigured && !timingFinalEmitted) {
            hgasp::emitHotPathCountersFinal(false);
            hgasp::emitComponentTimingFinal(
                data == nullptr
                    ? 0U
                    : static_cast<std::uint64_t>(
                          std::max(0, data->iteration)),
                data != nullptr && searchClockStarted
                    ? data->elapsedCpuSeconds()
                    : 0.0,
                false);
            timingFinalEmitted = true;
        }
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
