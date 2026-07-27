/******************************************************************************
 * File: read_data.h
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description:
 *   Handles data reading, parameter configuration, and instance parsing.
 ******************************************************************************/

#ifndef READ_DATA_H_
#define READ_DATA_H_

#include <time.h>
#include <unordered_map>

#include "basic.h"

class read_data {
public:
	// Search state and stopping information.
	int loopp;
	double probFixedCost;
	int iteration;
	int nbIterNonImpro;
	int isFleetLimited;
	clock_t startTime;
	int timeLimit;
	int iterLimit;
	int totalDemand;
	double avgDemand;
	bool isConstraintLoc;

	// Population parameters.
	int nbClost = 5;
	int nbElite = 4;

	// Shaw removal parameters.
	int nearest_pop_parameter = 3;
	double determine_para = 6;

	// HGAMP parameters.
	int alpha = 20;
	int popMin = 30;
	int popMax = popMin + 30;
	int popElit = 30;
	// HGAMP Table 2: Shaw mutation probability zeta = 0.15.
	int mutationProbabilityPercent = 15;
	double ratioPerturbLength = 0.25;
	int maxIterNonProd = 70000;
	int numLocConfiguration = 2000;
	int maxIterationPerLoc = 500;

	// Crossover and penalty parameters.
	int eSets_strategy = 3;
	double penaltyDepot = 1.;
	double penaltyVeh = 1.;
	double normalizaPara;

	// Route-pool and set-partitioning parameters.
	int spTriggerFrequency = 50001;
	// Delays the first SP call to frequency + offset iterations. Subsequent
	// calls remain frequency iterations apart.
	int spTriggerOffset = 0;
	double spTimeLimit = 360;
	// Optional deterministic Gurobi work budget. Zero leaves WorkLimit unset.
	double spWorkLimit = 0;
	double spMIPGap = 0.001;
	int spMaxPoolSize = 50000;
	int spThreads = 1;
	// Empty means main() assigns an isolated gurobi.log path for the run.
	std::string spLogFile;
	int spOutputFlag = 0;
	int spSeed = 0;
	int spMIPFocus = 1;
	double spHeuristics = 0.15;
	// Gurobi environment and WLS reliability controls. A zero token duration
	// preserves Gurobi's automatic policy instead of extending license
	// sessions in short-lived experiment workers.
	int spWLSTokenDurationMinutes = 0;
	double spWLSTokenRefresh = 0.9;
	int spClientLogLevel = 1;
	int spServerTimeoutSeconds = 30;
	int spEnvironmentMaxAttempts = 3;
	int spEnvironmentRetryBaseMilliseconds = 250;
	// HGAMP assumes an unlimited fleet. This optional compatibility guard
	// prevents SP solutions from exceeding the fixed-slot Individual
	// representation.
	bool spEnforceRepresentationRouteLimit = true;

	// Newly established resident populations are protected for this many
	// iterations before they may be replaced.
	int popProtectionPeriod = 10000;

	// Parsed instance data and neighborhoods.
	std::vector<std::vector<int>> edgeNode;
	std::vector<std::vector<int>> customerToLoc;
	std::vector<std::vector<int>> nearCity;
	std::vector<std::vector<int>> nearCityLoca;
	std::vector<std::vector<int>> edgeLocVec;
	int numV;
	int numL;
	int numC;
	int vehCap;
	int maxVnum;
	// Row pointers retained for the existing D[from][to] API. All rows point
	// into one contiguous allocation to improve spatial locality and reduce
	// allocator/TLB overhead in the route-generator hot loops.
	double** D;
	double* x;
	double* y;
	int* demCus;
	int* invLoc;
	double* openCostLoc;
	double usingCostVeh;
	int realOrInteger = 0;

	read_data();
	virtual ~read_data();
	void applyEnvironmentOverrides();
	void define();
	void read_fun(
		std::string pathInstance,
		std::string pathDepotCon,
		std::string pathSolution,
		double bestDis,
		int isOptimal,
		int timeLimit,
		int iterLimit);
	std::vector<std::string> splitwithstr(std::string& str, char ch);
	bool equalTwoVar(double A, double B);
	bool stopCondition(double dis);

	// Run input and best-known-value tracking.
	std::string pathSolution;
	std::string pathDepotCon;
	std::string pathInstance;
	double bestDis;
	int isOptimal;
	bool meetOptimal = true;
	double timeToTarget = Max;
	double meetBestTime;

	// Candidate depot-configuration analysis.
	std::vector<std::pair<double, int>> locRatio;
	std::vector<std::set<int>> compactLocConfigure;
	std::vector<std::pair<double, std::set<int>>> LocConfigure;
	int locCostSignificant;
	int dimension = 10000000;
	std::unordered_map<long long int, int> dimTabu1;
	std::unordered_map<long long int, int> dimTabu2;
	bool no_capacity_location = false;

private:
	double* distanceStorage;
	int* solutionTabu;
	double maxDemand;
	double maxDist;
	int** conflictLoc;
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

#endif /* READ_DATA_H_ */
