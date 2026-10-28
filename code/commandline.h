// Author: Feiran
// Parse and validate the solver command line.

#ifndef COMMANDLINE_H_
#define COMMANDLINE_H_

#include "basic.h"
#include "SearchParameters.h"
#include <stdexcept>
#include <limits>

class CommandLine {
  public:
    std::string pathInstance, pathDepotCon, pathSolution;
    int timeLimit = Max, iterLimit = 0, seed = 0;
    double bestDis = 0;
    int isOptimal = 0;

    static const char *usage() {
        return "Usage: masp INSTANCE [DEPOT_CONFIG] OUTPUT BEST_VALUE IS_OPTIMAL CPU_LIMIT MAIN_COUNTER_LIMIT SEED";
    }

    CommandLine(int argc, char *argv[]) {
        if (argc != 8 && argc != 9)
            throw std::invalid_argument(usage());
        const int offset = argc == 8 ? 3 : 4;
        pathInstance = argv[1];
        pathDepotCon = argc == 8 ? "NULL" : argv[2];
        pathSolution = argv[offset - 1];
        bestDis = SearchParameters::parseExperimentDecimal(argv[offset], "BEST_VALUE");
        isOptimal = SearchParameters::parseExperimentInteger(argv[offset + 1], "IS_OPTIMAL");
        timeLimit = SearchParameters::parseExperimentInteger(argv[offset + 2], "CPU_LIMIT");
        iterLimit =
            SearchParameters::parseExperimentInteger(argv[offset + 3], "MAIN_COUNTER_LIMIT");
        seed = SearchParameters::parseExperimentInteger(argv[offset + 4], "SEED");
        validateArguments();
        std::cout << "Starting The Algorithm\nThe Instance is " << pathInstance << std::endl;
    }

  private:
    void validateArguments() const {
        if (pathInstance.empty() || pathSolution.empty())
            throw std::invalid_argument("Instance and output paths must not be empty");
        if (isOptimal != 0 && isOptimal != 1)
            throw std::invalid_argument("IS_OPTIMAL must be 0 or 1");
        if (timeLimit <= 0)
            throw std::invalid_argument("CPU_LIMIT must be positive");
        if (iterLimit == std::numeric_limits<int>::max())
            throw std::invalid_argument("MAIN_COUNTER_LIMIT must be smaller than INT_MAX");
        if (bestDis < 0 || (isOptimal && bestDis <= 0))
            throw std::invalid_argument(
                "BEST_VALUE must be nonnegative and positive when IS_OPTIMAL is 1");
    }
};

#endif
