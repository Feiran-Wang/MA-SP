// Author: Feiran
// Define family profiles and resolve supported runtime parameters.

#ifndef MASP_SEARCH_PARAMETERS_H
#define MASP_SEARCH_PARAMETERS_H

#include <string>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <initializer_list>
#include <stdexcept>
#include <vector>

struct SearchParameters {
    const char *parameterProfile;
    int nbClost, nbElite, nearest_pop_parameter;
    double determine_para;
    int alpha, popMin, popMax, popElit;
    int mutationProb;
    double ratioPerturbLength;
    int maxIterNonProd, maxIterationPerLoc, eSets_strategy;
    int configurationCandidateLimit, configurationSamples, maxActiveConfigurations;
    int initialPopulationMultiplier;
    int restartMinRemainingIterations;
    double restartMinRemainingSeconds;
    int spTriggerFrequency;
    double spTimeLimit, spMIPGap;
    int spMaxPoolSize, spThreads;
    std::string spLogFile;
    int spOutputFlag, spMIPFocus;
    double spHeuristics;
    int popProtectionPeriod, dimension;

    bool experimentMode = false;
    std::string experimentArm = "Full";
    std::string experimentEventsPath;
    std::string spPolicyId;
    std::vector<int> spSchedule;
    double spBudgetMultiplier = 1.0;
    int requestedScreeningRepetitions = 3, requestedPopulationMinimum = 20;
    double requestedNeighborhoodFraction = 0.30;
    int requestedMutationPercent = 15;
    double requestedShawRemovalFraction = 0.20;
    int requestedRoutePoolCapacity = 10000, requestedShawRemovalCount = 0;

    static bool setSExperimentEnabled() {
        return readExperimentSwitch("MASP_SETS_EXPERIMENT", "MASP_SETS_EXPERIMENT must be 0 or 1");
    }

    static bool experimentEnabled() {
        const bool setFEnabled =
            readExperimentSwitch("MASP_SETF_EXPERIMENT", "MASP_SETF_EXPERIMENT must be 0 or 1");
        const bool setSEnabled = setSExperimentEnabled();
        if (setFEnabled && setSEnabled)
            throw std::invalid_argument("Set S and Set F experiment modes are mutually exclusive");
        const bool enabled = setFEnabled || setSEnabled;
        if (setSEnabled && std::getenv("MASP_LAMBDA"))
            throw std::invalid_argument("Set S uses MASP_ALPHA, not MASP_LAMBDA");
        if (setFEnabled && std::getenv("MASP_ALPHA"))
            throw std::invalid_argument("Set F uses MASP_LAMBDA, not MASP_ALPHA");
        if (!enabled) {
            for (const char *name :
                 {"MASP_R_CFG", "MASP_MU", "MASP_LAMBDA", "MASP_ALPHA", "MASP_P_MUT", "MASP_RHO",
                  "MASP_POOL", "MASP_SP_POLICY", "MASP_SP_FREQUENCY", "MASP_SP_BETA", "MASP_ARM",
                  "MASP_EVENT_PATH"})
                if (std::getenv(name))
                    throw std::invalid_argument(std::string(name) +
                                                " requires an explicit family experiment mode");
        }
        return enabled;
    }

    static int parseExperimentInteger(const std::string &value, const char *name) {
        if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos)
            throw std::invalid_argument(std::string("invalid integer for ") + name);
        std::size_t consumed = 0;
        const int result = std::stoi(value, &consumed);
        if (consumed != value.size())
            throw std::invalid_argument(std::string("invalid integer for ") + name);
        return result;
    }

    static double parseExperimentDecimal(const std::string &value, const char *name) {
        if (value.empty() || value.find_first_not_of("0123456789.eE+-") != std::string::npos)
            throw std::invalid_argument(std::string("invalid number for ") + name);
        std::size_t consumed = 0;
        const double result = std::stod(value, &consumed);
        if (consumed != value.size() || !std::isfinite(result))
            throw std::invalid_argument(std::string("invalid number for ") + name);
        return result;
    }

    static int experimentInteger(const char *name, int fallback,
                                 std::initializer_list<int> values) {
        const char *raw = std::getenv(name);
        const int value = raw ? parseExperimentInteger(raw, name) : fallback;
        if (std::find(values.begin(), values.end(), value) == values.end())
            throw std::invalid_argument(std::string(name) + " is outside the frozen study levels");
        return value;
    }

    static double experimentDecimal(const char *name, double fallback,
                                    std::initializer_list<double> values) {
        const char *raw = std::getenv(name);
        const double value = raw ? parseExperimentDecimal(raw, name) : fallback;
        if (std::find(values.begin(), values.end(), value) == values.end())
            throw std::invalid_argument(std::string(name) + " is outside the frozen study levels");
        return value;
    }

    static SearchParameters setS(int customers) {
        SearchParameters p;
        setSharedDefaults(p, customers);
        p.parameterProfile = "set-s-baseline-v4-p1";
        p.alpha = 20;
        p.maxIterationPerLoc = 500;
        p.configurationCandidateLimit = 500;
        p.configurationSamples = 15;
        p.maxActiveConfigurations = 15;
        p.initialPopulationMultiplier = 4;
        p.spMaxPoolSize = 50000;
        p.dimension = 10000000;
        return p;
    }

    static SearchParameters setF(int customers, int depots) {
        SearchParameters p;
        setSharedDefaults(p, customers);
        p.parameterProfile = "set-f-tw-v1-p1";
        p.alpha = 10;
        p.maxIterationPerLoc = 100;
        p.configurationCandidateLimit = 50;
        p.configurationSamples = 3;
        p.maxActiveConfigurations = 10;
        p.initialPopulationMultiplier = 1;
        p.spMaxPoolSize = 10000;
        // Include the largest exact signature key for the small-depot table.
        p.dimension = depots >= 1 && depots <= 5 ? (1 << depots) + 9527 * depots * (depots - 1) / 2
                                                 : 10000000;
        return p;
    }

    static SearchParameters setFExperiment(int customers, int depots) {
        if (customers < 1 || customers > 50)
            throw std::invalid_argument(
                "Set F study requires 1..50 customers; no SP budget extrapolation is defined");
        return studyParameters(setF(customers, depots), customers, false);
    }

    static SearchParameters setSExperiment(int customers) {
        if (customers < 2)
            throw std::invalid_argument("Set S study requires at least two customers");
        return studyParameters(setS(customers), customers, true);
    }

    static SearchParameters studyParameters(SearchParameters p, int customers, bool setSStudy) {
        const double familySPTimeLimit = p.spTimeLimit;
        p.experimentMode = true;
        p.parameterProfile = setSStudy ? "set-s-study-v1" : "set-f-study-v1";
        p.requestedScreeningRepetitions =
            setSStudy ? experimentInteger("MASP_R_CFG", 15, {5, 10, 15, 20, 25})
                      : experimentInteger("MASP_R_CFG", 3, {1, 2, 3, 5, 8});
        p.requestedPopulationMinimum = setSStudy
                                           ? experimentInteger("MASP_MU", 30, {20, 25, 30, 40, 50})
                                           : experimentInteger("MASP_MU", 20, {10, 15, 20, 25, 30});
        if (!setSStudy)
            p.requestedNeighborhoodFraction =
                experimentDecimal("MASP_LAMBDA", 0.30, {0.10, 0.20, 0.30, 0.40, 0.50});
        p.requestedMutationPercent = experimentInteger("MASP_P_MUT", 15, {5, 10, 15, 20, 25});
        p.requestedShawRemovalFraction =
            setSStudy ? experimentDecimal("MASP_RHO", 0.25, {0.10, 0.15, 0.25, 0.35, 0.45})
                      : experimentDecimal("MASP_RHO", 0.20, {0.10, 0.15, 0.20, 0.25, 0.30});
        p.requestedRoutePoolCapacity =
            setSStudy ? experimentInteger("MASP_POOL", 50000, {10000, 25000, 50000, 75000, 100000})
                      : experimentInteger("MASP_POOL", 10000, {2000, 5000, 10000, 20000, 50000});
        p.configurationSamples = p.requestedScreeningRepetitions;
        p.maxActiveConfigurations = setSStudy ? p.requestedScreeningRepetitions : 15;
        p.popMin = p.requestedPopulationMinimum;
        p.popMax = p.popMin + 30;
        p.alpha = setSStudy ? experimentInteger("MASP_ALPHA", 20, {10, 15, 20, 30, 40})
                            : std::min(customers - 1,
                                       std::max(1, int(std::ceil(p.requestedNeighborhoodFraction *
                                                                 (customers - 1)))));

        // The inclusive integer draw uses >= against this mutation threshold.
        p.mutationProb = 100 - p.requestedMutationPercent;
        p.ratioPerturbLength = p.requestedShawRemovalFraction;
        p.requestedShawRemovalCount =
            std::max(1, int(std::floor(p.ratioPerturbLength * customers)));
        p.spMaxPoolSize = p.requestedRoutePoolCapacity;
        p.spTriggerFrequency =
            experimentInteger("MASP_SP_FREQUENCY", 50001, {20001, 30001, 50001, 75001, 100001});
        p.spBudgetMultiplier = experimentDecimal("MASP_SP_BETA", 1.0, {0.25, 0.50, 1.0, 2.0, 4.0});
        const char *policy = std::getenv("MASP_SP_POLICY");
        if (policy) {
            p.spPolicyId = policy;
            if (p.spPolicyId == "Early5" || p.spPolicyId == "Late5") {
                if (std::getenv("MASP_SP_FREQUENCY") || p.spBudgetMultiplier != 1.0)
                    throw std::invalid_argument(
                        "Early5/Late5 require beta=1 and no periodic frequency override");
                p.spTriggerFrequency = 0;
                p.spSchedule = p.spPolicyId == "Early5"
                                   ? std::vector<int>{6001, 15001, 30001, 45001, 60001}
                                   : std::vector<int>{150001, 180001, 210001, 240001, 270001};
            } else {
                bool found = false;
                for (int frequency : {20001, 30001, 50001, 75001, 100001}) {
                    for (const char *beta : {"0.25", "0.5", "1", "2", "4"}) {
                        if (p.spPolicyId != "F" + std::to_string(frequency) + "_b" + beta)
                            continue;
                        const double multiplier = parseExperimentDecimal(beta, "MASP_SP_POLICY");
                        if ((std::getenv("MASP_SP_FREQUENCY") &&
                             p.spTriggerFrequency != frequency) ||
                            (std::getenv("MASP_SP_BETA") && p.spBudgetMultiplier != multiplier))
                            throw std::invalid_argument(
                                "SP policy conflicts with frequency/beta overrides");
                        p.spTriggerFrequency = frequency;
                        p.spBudgetMultiplier = multiplier;
                        found = true;
                    }
                }
                if (!found)
                    throw std::invalid_argument(
                        "MASP_SP_POLICY is not one of the 27 frozen policies");
            }
        }
        if (p.spSchedule.empty()) {
            if (!policy) {
                const char *beta = p.spBudgetMultiplier == 0.25  ? "0.25"
                                   : p.spBudgetMultiplier == 0.5 ? "0.5"
                                   : p.spBudgetMultiplier == 1.0 ? "1"
                                   : p.spBudgetMultiplier == 2.0 ? "2"
                                                                 : "4";
                p.spPolicyId = "F" + std::to_string(p.spTriggerFrequency) + "_b" + beta;
            }
            for (int event = p.spTriggerFrequency; event <= 300000; event += p.spTriggerFrequency)
                p.spSchedule.push_back(event);
        }
        const double base = setSStudy         ? familySPTimeLimit
                            : customers <= 10 ? 2.0
                            : customers <= 25 ? 5.0
                            : customers <= 40 ? 10.0
                                              : 20.0;
        // Normalize per-call SP time with the reference 300000-counter schedule.
        p.spTimeLimit = 5.0 * base * p.spBudgetMultiplier / p.spSchedule.size();
        if (const char *arm = std::getenv("MASP_ARM"))
            p.experimentArm = arm;
        if (p.experimentArm != "Full" && p.experimentArm != "NoSP" &&
            p.experimentArm != "OneWaySP" && p.experimentArm != "NoSPInstall" &&
            p.experimentArm != "GlobalOnly")
            throw std::invalid_argument(
                "MASP_ARM must be Full, NoSP, OneWaySP, NoSPInstall, or GlobalOnly");
        if (const char *events = std::getenv("MASP_EVENT_PATH")) {
            if (!*events)
                throw std::invalid_argument("MASP_EVENT_PATH cannot be empty");
            p.experimentEventsPath = events;
        }
        return p;
    }

  private:
    static bool readExperimentSwitch(const char *name, const char *error) {
        const char *value = std::getenv(name);
        if (!value)
            return false;
        if ((value[0] != '0' && value[0] != '1') || value[1] != '\0')
            throw std::invalid_argument(error);
        return value[0] == '1';
    }

    static void setSharedDefaults(SearchParameters &p, int customers) {
        p.nbClost = 5;
        p.nbElite = 4;
        p.nearest_pop_parameter = 3;
        p.determine_para = 6;
        p.popMin = 30;
        p.popMax = 60;
        p.popElit = 30;
        p.mutationProb = 85;
        p.ratioPerturbLength = 0.25;
        p.maxIterNonProd = 70000;
        p.eSets_strategy = 3;
        p.restartMinRemainingIterations = 50000;
        p.restartMinRemainingSeconds = 600;
        p.spTriggerFrequency = 50001;
        p.spTimeLimit = customers <= 100   ? 10
                        : customers <= 200 ? 60
                        : customers <= 300 ? 150
                        : customers <= 400 ? 210
                        : customers <= 500 ? 280
                                           : 360;
        p.spMIPGap = 0.001;
        p.spThreads = 1;
        p.spLogFile = "gurobi_sp.log";
        p.spOutputFlag = 0;
        p.spMIPFocus = 1;
        p.spHeuristics = 0.15;
        p.popProtectionPeriod = 10000;
    }
};

#endif
