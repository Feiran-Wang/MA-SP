// Author: Feiran
// Declare benchmark storage, resource limits, and configuration services.

#ifndef READ_DATA_H_
#define READ_DATA_H_
#include <time.h>
#include "basic.h"
#include "SearchParameters.h"
#include "ExperimentClock.h"
class read_data : public SearchParameters {
  public:
    double isolatedSPCPU = 0;
    double isolatedSPStarted = -1;
    double evolutionCPU() const {
        const double now = double(clock()) / CLOCKS_PER_SEC;
        return now - isolatedSPCPU - (isolatedSPStarted >= 0 ? now - isolatedSPStarted : 0);
    }
    int safeRouteLimit(int depot) const {
        int minimum = numV ? demCus[0] : 0;
        for (int i = 1; i < numV; ++i)
            minimum = std::min(minimum, demCus[i]);
        return minimum > 0 ? std::min(numV, invLoc[depot + numV] / minimum) : numV;
    }

    int loopp = 0;
    double probFixedCost = 0;
    int iteration = 0;
    int nbIterNonImpro = 0;
    clock_t startTime = 0;
    int timeLimit = 0;
    int iterLimit = 0;
    int totalDemand = 0;
    double avgDemand = 0;
    bool isConstraintLoc = false;

    int numLocConfiguration = 2000;
    double penaltyDepot = 1.;
    double penaltyVeh = 1.;
    double normalizaPara = 0;

    std::vector<std::vector<int>> edgeNode;
    std::vector<std::vector<int>> customerToLoc;
    std::vector<std::vector<int>> nearCity;
    std::vector<std::vector<int>> nearCityLoca;
    std::vector<std::vector<int>> edgeLocVec;
    int numV = 0;
    int numL = 0;
    int numC = 0;
    int vehCap = 0;
    int maxVnum = 0;
    // Null-initialized storage permits cleanup after partially completed input allocation.
    double **D = nullptr;
    double *x = nullptr;
    double *y = nullptr;
    int *demCus = nullptr;
    int *invLoc = nullptr;
    double *openCostLoc = nullptr;
    double usingCostVeh = 0;
    int realOrInteger = 0;
    bool hardTimeWindows = false;
    long long setSMutationCalls = 0, setSMutationRemovedCustomers = 0;
    std::vector<double> readyTime, dueTime, serviceTime;
    std::vector<double> depotCloseTime;
    std::vector<int> originalCustomerIds, originalDepotIds;

    bool routeTimeFeasible(int depotIndex, const std::vector<int> &sequence) const;

    read_data();
    virtual ~read_data();
    void define();
    void read_fun(std::string pathInstance, std::string pathDepotCon, std::string pathSolution,
                  double bestDis, int isOptimal, int timeLimit, int iterLimit);
    std::vector<std::string> splitwithstr(std::string &str, char ch);
    bool equalTwoVar(double A, double B);
    bool stopCondition(double dis);

    std::string pathSolution;
    std::string pathDepotCon;
    std::string pathInstance;
    double bestDis = 0;
    int isOptimal = 0;
    bool meetOptimal = true;
    std::string experimentTerminationReason;
    unsigned long long twNeighborhoodLists = 0, twNeighborhoodEntries = 0;
    unsigned long long twNeighborhoodFiltered = 0;
    int twNeighborhoodMaximum = 0;
    unsigned long long twLocalSearchCandidates = 0, twLocalSearchRejected = 0;
    unsigned long long twMutationCalls = 0, twMutationRemovedCustomers = 0, twRepairFailures = 0;
    double timeToTarget = Max;
    double meetBestTime = 0;

    std::vector<std::pair<double, int>> locRatio;
    std::vector<std::set<int>> compactLocConfigure;
    std::vector<std::pair<double, std::set<int>>> LocConfigure;

    int *dimTabu1 = nullptr;
    int *dimTabu2 = nullptr;
    bool no_capacity_location = false;

  private:
    void validateDimensions();
    void readFarham(const std::string &path);
    double maxDemand = 0;
    double maxDist = 0;

    int **conflictLoc = nullptr;
    void analyzeLocation();
    void tabuSearchFindCompactConfiguration();
    void spanningTreeForLoc();
    void analyzeConflict();

    void constructedDepotConfigurations();
    std::vector<std::set<int>> locIncCus;

    void readDepotConfigurations();
    std::vector<std::vector<int>> depotCon;
    void decideUsingDepotConfigurations();
};

#endif
