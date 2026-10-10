#ifndef HGAMP2_SEARCH_PARAMETERS_H
#define HGAMP2_SEARCH_PARAMETERS_H

#include <string>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <initializer_list>
#include <stdexcept>
#include <vector>

// Independent family profiles. Equal values are intentional, not inheritance
// from the other family. CLI CPU/iteration budgets are not profile parameters.
struct SearchParameters {
    const char* parameterProfile;
    int nbClost, nbElite, nearest_pop_parameter;
    double determine_para;
    int alpha, popMin, popMax, popElit;
    int mutationProb; // Inclusive threshold on draws 0..99: 85 means 15%.
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
    // This study is opt-in; the two established family profiles are unchanged.
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
        const char* value = std::getenv("HGAMP2_SETS_EXPERIMENT");
        if (value && std::string(value) != "0" && std::string(value) != "1")
            throw std::invalid_argument("HGAMP2_SETS_EXPERIMENT must be 0 or 1");
        return value && std::string(value) == "1";
    }

    static bool experimentEnabled() {
        const char* value = std::getenv("HGAMP2_SETF_EXPERIMENT");
        if (value && std::string(value) != "0" && std::string(value) != "1")
            throw std::invalid_argument("HGAMP2_SETF_EXPERIMENT must be 0 or 1");
        const bool setFEnabled = value && std::string(value) == "1";
        const bool setSEnabled = setSExperimentEnabled();
        if (setFEnabled && setSEnabled)
            throw std::invalid_argument("Set S and Set F experiment modes are mutually exclusive");
        const bool enabled = setFEnabled || setSEnabled;
        if (setSEnabled && std::getenv("HGAMP2_LAMBDA"))
            throw std::invalid_argument("Set S uses HGAMP2_ALPHA, not HGAMP2_LAMBDA");
        if (setFEnabled && std::getenv("HGAMP2_ALPHA"))
            throw std::invalid_argument("Set F uses HGAMP2_LAMBDA, not HGAMP2_ALPHA");
        if (!enabled) {
            for (const char* name : {"HGAMP2_R_CFG", "HGAMP2_MU", "HGAMP2_LAMBDA", "HGAMP2_ALPHA",
                    "HGAMP2_P_MUT", "HGAMP2_RHO", "HGAMP2_POOL", "HGAMP2_SP_POLICY",
                    "HGAMP2_SP_FREQUENCY", "HGAMP2_SP_BETA", "HGAMP2_ARM", "HGAMP2_EVENT_PATH"})
                if (std::getenv(name))
                    throw std::invalid_argument(std::string(name) + " requires an explicit family experiment mode");
        }
        return enabled;
    }

    static int parseExperimentInteger(const std::string& value, const char* name) {
        if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos)
            throw std::invalid_argument(std::string("invalid integer for ") + name);
        std::size_t consumed = 0;
        const int result = std::stoi(value, &consumed);
        if (consumed != value.size()) throw std::invalid_argument(std::string("invalid integer for ") + name);
        return result;
    }

    static double parseExperimentDecimal(const std::string& value, const char* name) {
        if (value.empty() || value.find_first_not_of("0123456789.eE+-") != std::string::npos)
            throw std::invalid_argument(std::string("invalid number for ") + name);
        std::size_t consumed = 0;
        const double result = std::stod(value, &consumed);
        if (consumed != value.size() || !std::isfinite(result))
            throw std::invalid_argument(std::string("invalid number for ") + name);
        return result;
    }

    static int experimentInteger(const char* name, int fallback, std::initializer_list<int> values) {
        const char* raw = std::getenv(name);
        const int value = raw ? parseExperimentInteger(raw, name) : fallback;
        if (std::find(values.begin(), values.end(), value) == values.end())
            throw std::invalid_argument(std::string(name) + " is outside the frozen study levels");
        return value;
    }

    static double experimentDecimal(const char* name, double fallback, std::initializer_list<double> values) {
        const char* raw = std::getenv(name);
        const double value = raw ? parseExperimentDecimal(raw, name) : fallback;
        if (std::find(values.begin(), values.end(), value) == values.end())
            throw std::invalid_argument(std::string(name) + " is outside the frozen study levels");
        return value;
    }

    static SearchParameters setS(int customers) {
        SearchParameters p;
        p.parameterProfile = "set-s-baseline-v4-p1";
        p.nbClost = 5; p.nbElite = 4; p.nearest_pop_parameter = 3;
        p.determine_para = 6;
        p.alpha = 20; p.popMin = 30; p.popMax = 60; p.popElit = 30;
        p.mutationProb = 85; p.ratioPerturbLength = 0.25;
        p.maxIterNonProd = 70000; p.maxIterationPerLoc = 500; p.eSets_strategy = 3;
        p.configurationCandidateLimit = 500; p.configurationSamples = 15;
        p.maxActiveConfigurations = 15; p.initialPopulationMultiplier = 4;
        p.restartMinRemainingIterations = 50000; p.restartMinRemainingSeconds = 600;
        p.spTriggerFrequency = 50001;
        // Preserve the previously effective Set S customer-count staircase.
        p.spTimeLimit = customers <= 100 ? 10 : customers <= 200 ? 60 :
            customers <= 300 ? 150 : customers <= 400 ? 210 : customers <= 500 ? 280 : 360;
        p.spMIPGap = 0.001; p.spMaxPoolSize = 50000; p.spThreads = 1;
        p.spLogFile = "gurobi_sp.log"; p.spOutputFlag = 0;
        p.spMIPFocus = 1; p.spHeuristics = 0.15;
        p.popProtectionPeriod = 10000; p.dimension = 10000000;
        return p;
    }

    static SearchParameters setF(int customers, int depots) {
        SearchParameters p;
        p.parameterProfile = "set-f-tw-v1-p1";
        p.nbClost = 5; p.nbElite = 4; p.nearest_pop_parameter = 3;
        p.determine_para = 6;
        p.alpha = 10; p.popMin = 30; p.popMax = 60; p.popElit = 30;
        p.mutationProb = 85; p.ratioPerturbLength = 0.25;
        p.maxIterNonProd = 70000; p.maxIterationPerLoc = 100; p.eSets_strategy = 3;
        p.configurationCandidateLimit = 50; p.configurationSamples = 3;
        p.maxActiveConfigurations = 10; p.initialPopulationMultiplier = 1;
        p.restartMinRemainingIterations = 50000; p.restartMinRemainingSeconds = 600;
        p.spTriggerFrequency = 50001;
        // 10 seconds for the existing 10/25/40/50-customer Set F library.
        // Retain the old limits for larger Farham inputs; no extrapolated tuning.
        p.spTimeLimit = customers <= 100 ? 10 : customers <= 200 ? 60 :
            customers <= 300 ? 150 : customers <= 400 ? 210 : customers <= 500 ? 280 : 360;
        p.spMIPGap = 0.001; p.spMaxPoolSize = 10000; p.spThreads = 1;
        p.spLogFile = "gurobi_sp.log"; p.spOutputFlag = 0;
        p.spMIPFocus = 1; p.spHeuristics = 0.15;
        p.popProtectionPeriod = 10000;
        // Keys are sum(2^l) and sum(2^l + 9527*l). For 1..5 depots,
        // max(key)+1 prevents modulo wrap just as the original 10M table did.
        // Keep the original storage outside this proved small-depot domain.
        p.dimension = depots >= 1 && depots <= 5
            ? (1 << depots) + 9527 * depots * (depots - 1) / 2 : 10000000;
        return p;
    }

    static SearchParameters setFExperiment(int customers, int depots) {
        if (customers < 1 || customers > 50)
            throw std::invalid_argument("Set F study requires 1..50 customers; no SP budget extrapolation is defined");
        return studyParameters(setF(customers, depots), customers, false);
    }

    static SearchParameters setSExperiment(int customers) {
        if (customers < 2) throw std::invalid_argument("Set S study requires at least two customers");
        return studyParameters(setS(customers), customers, true);
    }

    static SearchParameters studyParameters(SearchParameters p, int customers, bool setSStudy) {
        const double familySPTimeLimit = p.spTimeLimit;
        p.experimentMode = true;
        p.parameterProfile = setSStudy ? "set-s-study-v1" : "set-f-study-v1";
        p.requestedScreeningRepetitions = setSStudy
            ? experimentInteger("HGAMP2_R_CFG", 15, {5, 10, 15, 20, 25})
            : experimentInteger("HGAMP2_R_CFG", 3, {1, 2, 3, 5, 8});
        p.requestedPopulationMinimum = setSStudy
            ? experimentInteger("HGAMP2_MU", 30, {20, 25, 30, 40, 50})
            : experimentInteger("HGAMP2_MU", 20, {10, 15, 20, 25, 30});
        if (!setSStudy)
            p.requestedNeighborhoodFraction = experimentDecimal("HGAMP2_LAMBDA", 0.30, {0.10, 0.20, 0.30, 0.40, 0.50});
        p.requestedMutationPercent = experimentInteger("HGAMP2_P_MUT", 15, {5, 10, 15, 20, 25});
        p.requestedShawRemovalFraction = setSStudy
            ? experimentDecimal("HGAMP2_RHO", 0.25, {0.10, 0.15, 0.25, 0.35, 0.45})
            : experimentDecimal("HGAMP2_RHO", 0.20, {0.10, 0.15, 0.20, 0.25, 0.30});
        p.requestedRoutePoolCapacity = setSStudy
            ? experimentInteger("HGAMP2_POOL", 50000, {10000, 25000, 50000, 75000, 100000})
            : experimentInteger("HGAMP2_POOL", 10000, {2000, 5000, 10000, 20000, 50000});
        p.configurationSamples = p.requestedScreeningRepetitions;
        p.maxActiveConfigurations = setSStudy ? p.requestedScreeningRepetitions : 15;
        p.popMin = p.requestedPopulationMinimum;
        p.popMax = p.popMin + 30;
        p.alpha = setSStudy ? experimentInteger("HGAMP2_ALPHA", 20, {10, 15, 20, 30, 40})
            : std::min(customers - 1, std::max(1, int(std::ceil(p.requestedNeighborhoodFraction * (customers - 1)))));
        // The active branch uses draw >= threshold, not the historical > branch.
        p.mutationProb = 100 - p.requestedMutationPercent;
        p.ratioPerturbLength = p.requestedShawRemovalFraction;
        p.requestedShawRemovalCount = std::max(1, int(std::floor(p.ratioPerturbLength * customers)));
        p.spMaxPoolSize = p.requestedRoutePoolCapacity;
        p.spTriggerFrequency = experimentInteger("HGAMP2_SP_FREQUENCY", 50001, {20001, 30001, 50001, 75001, 100001});
        p.spBudgetMultiplier = experimentDecimal("HGAMP2_SP_BETA", 1.0, {0.25, 0.50, 1.0, 2.0, 4.0});
        const char* policy = std::getenv("HGAMP2_SP_POLICY");
        if (policy) {
            p.spPolicyId = policy;
            if (p.spPolicyId == "Early5" || p.spPolicyId == "Late5") {
                if (std::getenv("HGAMP2_SP_FREQUENCY") || p.spBudgetMultiplier != 1.0)
                    throw std::invalid_argument("Early5/Late5 require beta=1 and no periodic frequency override");
                p.spTriggerFrequency = 0;
                p.spSchedule = p.spPolicyId == "Early5"
                    ? std::vector<int>{6001, 15001, 30001, 45001, 60001}
                    : std::vector<int>{150001, 180001, 210001, 240001, 270001};
            } else {
                bool found = false;
                for (int frequency : {20001, 30001, 50001, 75001, 100001}) {
                    for (const char* beta : {"0.25", "0.5", "1", "2", "4"}) {
                        if (p.spPolicyId != "F" + std::to_string(frequency) + "_b" + beta) continue;
                        const double multiplier = parseExperimentDecimal(beta, "HGAMP2_SP_POLICY");
                        if ((std::getenv("HGAMP2_SP_FREQUENCY") && p.spTriggerFrequency != frequency) ||
                            (std::getenv("HGAMP2_SP_BETA") && p.spBudgetMultiplier != multiplier))
                            throw std::invalid_argument("SP policy conflicts with frequency/beta overrides");
                        p.spTriggerFrequency = frequency;
                        p.spBudgetMultiplier = multiplier;
                        found = true;
                    }
                }
                if (!found) throw std::invalid_argument("HGAMP2_SP_POLICY is not one of the 27 frozen policies");
            }
        }
        if (p.spSchedule.empty()) {
            if (!policy) {
                const char* beta = p.spBudgetMultiplier == 0.25 ? "0.25" : p.spBudgetMultiplier == 0.5 ? "0.5" :
                    p.spBudgetMultiplier == 1.0 ? "1" : p.spBudgetMultiplier == 2.0 ? "2" : "4";
                p.spPolicyId = "F" + std::to_string(p.spTriggerFrequency) + "_b" + beta;
            }
            for (int event = p.spTriggerFrequency; event <= 300000; event += p.spTriggerFrequency)
                p.spSchedule.push_back(event);
        }
        const double base = setSStudy ? familySPTimeLimit
            : customers <= 10 ? 2.0 : customers <= 25 ? 5.0 : customers <= 40 ? 10.0 : 20.0;
        p.spTimeLimit = 5.0 * base * p.spBudgetMultiplier / p.spSchedule.size();
        if (const char* arm = std::getenv("HGAMP2_ARM")) p.experimentArm = arm;
        if (p.experimentArm != "Full" && p.experimentArm != "NoSP" &&
            p.experimentArm != "OneWaySP" && p.experimentArm != "NoSPInstall" &&
            p.experimentArm != "GlobalOnly")
            throw std::invalid_argument("HGAMP2_ARM must name one of the four frozen study arms");
        if (const char* events = std::getenv("HGAMP2_EVENT_PATH")) {
            if (!*events) throw std::invalid_argument("HGAMP2_EVENT_PATH cannot be empty");
            p.experimentEventsPath = events;
        }
        return p;
    }
};

#endif
