#ifndef HGASP_UTILS_COMMANDLINE_H_
#define HGASP_UTILS_COMMANDLINE_H_

#include "EaxParentMode.h"
#include "Version.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

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
    std::optional<hgasp::EaxParentMode> eaxParentMode;

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
               " TIME_LIMIT ITERATION_LIMIT SEED"
               " [--eax-mode paper-compatible|corrected]\n\n"
            << "The instance declares the CLRP constraint and numeric"
               " profiles; no problem-mode option is required.\n"
            << "IS_OPTIMAL must be 0 or 1. TIME_LIMIT must be non-negative"
               " (0 means unlimited), ITERATION_LIMIT must be positive,"
               " and SEED must be non-negative.\n";
    }

    CommandLine(const int argc, char* argv[]) {
        std::vector<std::string> positional;
        bool eaxModeSeen = false;
        for (int index = 1; index < argc; ++index) {
            const std::string argument = argv[index];
            if (argument == "--eax-mode") {
                if (eaxModeSeen) {
                    throw std::invalid_argument(
                        "--eax-mode may be specified only once");
                }
                if (index + 1 >= argc) {
                    throw std::invalid_argument(
                        "--eax-mode requires paper-compatible or corrected");
                }
                eaxParentMode =
                    hgasp::parseEaxParentMode(argv[++index]);
                eaxModeSeen = true;
                continue;
            }
            const std::string eaxPrefix = "--eax-mode=";
            if (argument.compare(0, eaxPrefix.size(), eaxPrefix) == 0) {
                if (eaxModeSeen) {
                    throw std::invalid_argument(
                        "--eax-mode may be specified only once");
                }
                eaxParentMode = hgasp::parseEaxParentMode(
                    argument.substr(eaxPrefix.size()));
                eaxModeSeen = true;
                continue;
            }
            if (argument.compare(0, 2, "--") == 0) {
                throw std::invalid_argument(
                    "unknown option: " + argument);
            }
            positional.push_back(argument);
        }

        if (positional.size() != 7 && positional.size() != 8) {
            throw std::invalid_argument("expected 7 or 8 positional arguments");
        }

        int offset = 0;
        pathInstance = positional[0];
        if (positional.size() == 8) {
            pathDepotCon = positional[1];
            offset = 1;
        } else {
            pathDepotCon = "NULL";
        }
        pathSolution = positional[1 + offset];

        bestDis = parseDouble(
            positional[2 + offset].c_str(), "BEST_VALUE");
        isOptimal = parseInt(
            positional[3 + offset].c_str(), "IS_OPTIMAL");
        timeLimit = parseInt(
            positional[4 + offset].c_str(), "TIME_LIMIT");
        iterLimit = parseInt(
            positional[5 + offset].c_str(), "ITERATION_LIMIT");
        seed = parseInt(
            positional[6 + offset].c_str(), "SEED");

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
        output << "Starting HGA-SP " << hgasp::kSolverVersion << '\n'
               << "Instance: " << pathInstance << '\n'
               << "EAX mode override: "
               << (eaxParentMode.has_value()
                       ? hgasp::toString(*eaxParentMode)
                       : "environment/default")
               << '\n'
               << "Seed: " << seed << std::endl;
    }
};

#endif  // HGASP_UTILS_COMMANDLINE_H_
