#ifndef HGASP_UTILS_COMMANDLINE_H_
#define HGASP_UTILS_COMMANDLINE_H_

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

class CommandLine {
public:
    std::string pathInstance;
    std::string pathDepotCon;
    std::string pathSolution;
    int timeLimit = 0;
    int iterLimit = 0;
    int seed = 0;
    double bestDis = 0.0;
    int isOptimal = 0;

private:
    static int parseInt(const char* text, const char* label) {
        try {
            std::size_t consumed = 0;
            const long long value = std::stoll(text, &consumed, 10);
            if (consumed != std::string(text).size() ||
                value < std::numeric_limits<int>::min() ||
                value > std::numeric_limits<int>::max()) {
                throw std::out_of_range(label);
            }
            return static_cast<int>(value);
        } catch (const std::exception&) {
            throw std::invalid_argument(std::string(label) + " must be an integer");
        }
    }

    static double parseDouble(const char* text, const char* label) {
        try {
            std::size_t consumed = 0;
            const double value = std::stod(text, &consumed);
            if (consumed != std::string(text).size() || !std::isfinite(value)) {
                throw std::out_of_range(label);
            }
            return value;
        } catch (const std::exception&) {
            throw std::invalid_argument(
                std::string(label) + " must be a finite number");
        }
    }

public:

    static void printUsage(std::ostream& output, const char* program) {
        output
            << "Usage:\n  " << program
            << " INSTANCE [DEPOT_CONFIG] OUTPUT BEST_VALUE IS_OPTIMAL"
               " TIME_LIMIT ITERATION_LIMIT SEED\n\n"
            << "IS_OPTIMAL must be 0 or 1. TIME_LIMIT must be non-negative"
               " (0 means unlimited), ITERATION_LIMIT must be positive,"
               " and SEED must be non-negative.\n";
    }

    CommandLine(const int argc, char* argv[]) {
        if (argc != 8 && argc != 9) {
            throw std::invalid_argument("expected 7 or 8 positional arguments");
        }

        int offset = 0;
        pathInstance = argv[1];
        if (argc == 9) {
            pathDepotCon = argv[2];
            offset = 1;
        } else {
            pathDepotCon = "NULL";
        }
        pathSolution = argv[2 + offset];

        bestDis = parseDouble(argv[3 + offset], "BEST_VALUE");
        isOptimal = parseInt(argv[4 + offset], "IS_OPTIMAL");
        timeLimit = parseInt(argv[5 + offset], "TIME_LIMIT");
        iterLimit = parseInt(argv[6 + offset], "ITERATION_LIMIT");
        seed = parseInt(argv[7 + offset], "SEED");

        if (isOptimal != 0 && isOptimal != 1) {
            throw std::invalid_argument("IS_OPTIMAL must be 0 or 1");
        }
        if (timeLimit < 0 || iterLimit <= 0) {
            throw std::invalid_argument(
                "TIME_LIMIT must be non-negative and ITERATION_LIMIT must be positive");
        }
        if (seed < 0) {
            throw std::invalid_argument("SEED must be non-negative");
        }
    }

    void displayInfo(std::ostream& output) const {
        output << "Starting HGA-SP\n"
               << "Instance: " << pathInstance << '\n'
               << "Seed: " << seed << std::endl;
    }
};

#endif  // HGASP_UTILS_COMMANDLINE_H_
