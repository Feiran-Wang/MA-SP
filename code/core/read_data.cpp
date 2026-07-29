/******************************************************************************
 * File: read_data.cpp
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description: 
 *   Handles data reading, parameter configuration, and instance parsing.
 ******************************************************************************/

#include "read_data.h"
#include "basic.h"
#include "Individual.h"
#include "InstanceLoader.h"
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>

namespace {

int environmentInt(
    const char* name,
    const int fallback,
    const int minimum,
    const int maximum) {
    const char* raw = std::getenv(name);
    if (raw == nullptr || *raw == '\0') {
        return fallback;
    }
    errno = 0;
    char* end = nullptr;
    const long value = std::strtol(raw, &end, 10);
    if (errno != 0 || end == raw || *end != '\0' ||
        value < minimum || value > maximum) {
        throw std::invalid_argument(
            std::string(name) + " is outside its accepted integer range");
    }
    return static_cast<int>(value);
}

double environmentDouble(
    const char* name,
    const double fallback,
    const double minimum,
    const double maximum) {
    const char* raw = std::getenv(name);
    if (raw == nullptr || *raw == '\0') {
        return fallback;
    }
    errno = 0;
    char* end = nullptr;
    const double value = std::strtod(raw, &end);
    if (errno != 0 || end == raw || *end != '\0' ||
        !std::isfinite(value) || value < minimum || value > maximum) {
        throw std::invalid_argument(
            std::string(name) + " is outside its accepted numeric range");
    }
    return value;
}

bool environmentBool(const char* name, const bool fallback) {
    return environmentInt(name, fallback ? 1 : 0, 0, 1) == 1;
}

double euclideanDistance(
    const double fromX,
    const double fromY,
    const double toX,
    const double toY) {
    return std::sqrt(
        (fromX - toX) * (fromX - toX) +
        (fromY - toY) * (fromY - toY));
}

double applyRounding(
    const double value,
    const hgasp::NumericRounding rounding) {
    if (rounding == hgasp::NumericRounding::CeilAfterScalingBy100) {
        return std::ceil(value * 100.0);
    }
    if (rounding == hgasp::NumericRounding::Floor) {
        // Coordinates in the published Farham files are integral.  The
        // tiny guard prevents floating-point sqrt error from mapping an
        // exact integer distance k to k-1, without affecting a genuinely
        // non-integral distance on this coordinate scale.
        return std::floor(value + 1.0e-12);
    }
    return value;
}

}  // namespace

read_data::read_data() {
    // Give embedded/test callers a valid process-CPU origin even when they do
    // not go through main().  main() resets this immediately before the
    // algorithm starts, so published search timings exclude instance parsing.
    startTime = clock();
    meetBestTime = 0.0;
    numV = 0;
    numL = 0;
    numC = 0;
    D = nullptr;
    T = nullptr;
    distanceStorage = nullptr;
    timeStorage = nullptr;
    x = nullptr;
    y = nullptr;
    demCus = nullptr;
    invLoc = nullptr;
    openCostLoc = nullptr;
    conflictLoc = nullptr;
    solutionTabu = nullptr;
}

void read_data::applyEnvironmentOverrides() {
    const char* eaxMode = std::getenv("HGASP_EAX_PARENT_MODE");
    if (eaxMode != nullptr && *eaxMode != '\0') {
        eaxParentMode = hgasp::parseEaxParentMode(eaxMode);
    }

    spTriggerFrequency = environmentInt(
        "HGASP_SP_TRIGGER_FREQUENCY",
        spTriggerFrequency,
        1,
        std::numeric_limits<int>::max());
    spTriggerOffset = environmentInt(
        "HGASP_SP_TRIGGER_OFFSET",
        spTriggerOffset,
        0,
        std::numeric_limits<int>::max());
    if (spTriggerOffset < 0 ||
        spTriggerOffset >= spTriggerFrequency) {
        throw std::invalid_argument(
            "HGASP_SP_TRIGGER_OFFSET must be non-negative and smaller than "
            "HGASP_SP_TRIGGER_FREQUENCY");
    }
    spTimeLimit = environmentDouble(
        "HGASP_SP_TIME_LIMIT_SECONDS",
        spTimeLimit,
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::max());
    spLocalEnabled = environmentBool(
        "HGASP_SP_LOCAL_ENABLED",
        spLocalEnabled);
    spLocalTriggerFrequency = environmentInt(
        "HGASP_SP_LOCAL_TRIGGER_FREQUENCY",
        spLocalTriggerFrequency,
        1,
        std::numeric_limits<int>::max());
    spLocalTimeLimit = environmentDouble(
        "HGASP_SP_LOCAL_TIME_LIMIT_SECONDS",
        spLocalTimeLimit,
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::max());
    spAdaptiveTriggerEnabled = environmentBool(
        "HGASP_SP_ADAPTIVE_TRIGGER_ENABLED",
        spAdaptiveTriggerEnabled);
    spAdaptiveMinimumInterval = environmentInt(
        "HGASP_SP_ADAPTIVE_MINIMUM_INTERVAL",
        spAdaptiveMinimumInterval,
        1,
        std::numeric_limits<int>::max());
    spAdaptiveLocalNewColumns = environmentInt(
        "HGASP_SP_ADAPTIVE_LOCAL_NEW_COLUMNS",
        spAdaptiveLocalNewColumns,
        1,
        std::numeric_limits<int>::max());
    spAdaptiveLocalStagnation = environmentInt(
        "HGASP_SP_ADAPTIVE_LOCAL_STAGNATION",
        spAdaptiveLocalStagnation,
        1,
        std::numeric_limits<int>::max());
    spAdaptiveGlobalStagnation = environmentInt(
        "HGASP_SP_ADAPTIVE_GLOBAL_STAGNATION",
        spAdaptiveGlobalStagnation,
        spAdaptiveLocalStagnation,
        std::numeric_limits<int>::max());
    spAdaptiveUtilityEmaWeight = environmentDouble(
        "HGASP_SP_ADAPTIVE_UTILITY_EMA_WEIGHT",
        spAdaptiveUtilityEmaWeight,
        std::numeric_limits<double>::min(),
        1.0);
    spAdaptiveHighUtilityThreshold = environmentDouble(
        "HGASP_SP_ADAPTIVE_HIGH_UTILITY_THRESHOLD",
        spAdaptiveHighUtilityThreshold,
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::max());
    spLpDualFeedbackEnabled = environmentBool(
        "HGASP_SP_LP_DUAL_FEEDBACK_ENABLED",
        spLpDualFeedbackEnabled);
    spLpTimeLimit = environmentDouble(
        "HGASP_SP_LP_TIME_LIMIT_SECONDS",
        spLpTimeLimit,
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::max());
    spPersistentLpEnabled = environmentBool(
        "HGASP_SP_PERSISTENT_LP_ENABLED",
        spPersistentLpEnabled);
    spSolutionPoolSize = environmentInt(
        "HGASP_SP_SOLUTION_POOL_SIZE",
        spSolutionPoolSize,
        1,
        1000);
    spSolutionPoolGap = environmentDouble(
        "HGASP_SP_SOLUTION_POOL_GAP",
        spSolutionPoolGap,
        0.0,
        std::numeric_limits<double>::max());
    spSolutionPoolSearchMode = environmentInt(
        "HGASP_SP_SOLUTION_POOL_SEARCH_MODE",
        spSolutionPoolSearchMode,
        0,
        2);
    spDiverseInjectionCount = environmentInt(
        "HGASP_SP_DIVERSE_INJECTION_COUNT",
        spDiverseInjectionCount,
        1,
        spSolutionPoolSize);
    adaptiveSearchAllocationEnabled = environmentBool(
        "HGASP_ADAPTIVE_SEARCH_ALLOCATION_ENABLED",
        adaptiveSearchAllocationEnabled);
    adaptivePopulationExploration = environmentDouble(
        "HGASP_ADAPTIVE_POPULATION_EXPLORATION",
        adaptivePopulationExploration,
        0.0,
        std::numeric_limits<double>::max());
    adaptivePopulationDiversityWeight = environmentDouble(
        "HGASP_ADAPTIVE_POPULATION_DIVERSITY_WEIGHT",
        adaptivePopulationDiversityWeight,
        0.0,
        std::numeric_limits<double>::max());
    adaptivePopulationColumnWeight = environmentDouble(
        "HGASP_ADAPTIVE_POPULATION_COLUMN_WEIGHT",
        adaptivePopulationColumnWeight,
        0.0,
        std::numeric_limits<double>::max());
    adaptiveSearchEmaWeight = environmentDouble(
        "HGASP_ADAPTIVE_SEARCH_EMA_WEIGHT",
        adaptiveSearchEmaWeight,
        std::numeric_limits<double>::min(),
        1.0);
    adaptiveMutationMinimumPercent = environmentInt(
        "HGASP_ADAPTIVE_MUTATION_MINIMUM_PERCENT",
        adaptiveMutationMinimumPercent,
        0,
        100);
    adaptiveMutationMaximumPercent = environmentInt(
        "HGASP_ADAPTIVE_MUTATION_MAXIMUM_PERCENT",
        adaptiveMutationMaximumPercent,
        adaptiveMutationMinimumPercent,
        100);
    spWorkLimit = environmentDouble(
        "HGASP_SP_WORK_LIMIT",
        spWorkLimit,
        0.0,
        std::numeric_limits<double>::max());
    spMIPGap = environmentDouble(
        "HGASP_SP_MIP_GAP",
        spMIPGap,
        0.0,
        1.0);
    spMaxPoolSize = environmentInt(
        "HGASP_SP_MAX_POOL_SIZE",
        spMaxPoolSize,
        1,
        std::numeric_limits<int>::max());
    spThreads = environmentInt(
        "HGASP_SP_THREADS",
        spThreads,
        1,
        1024);
    spSeed = environmentInt(
        "HGASP_SP_SEED",
        spSeed,
        0,
        2000000000);
    spOutputFlag = environmentInt(
        "HGASP_SP_OUTPUT_FLAG",
        spOutputFlag,
        0,
        1);
    spMIPFocus = environmentInt(
        "HGASP_SP_MIP_FOCUS",
        spMIPFocus,
        0,
        3);
    spHeuristics = environmentDouble(
        "HGASP_SP_HEURISTICS",
        spHeuristics,
        0.0,
        1.0);
    spWLSTokenDurationMinutes = environmentInt(
        "HGASP_SP_WLS_TOKEN_DURATION_MINUTES",
        spWLSTokenDurationMinutes,
        0,
        2000000000);
    spWLSTokenRefresh = environmentDouble(
        "HGASP_SP_WLS_TOKEN_REFRESH",
        spWLSTokenRefresh,
        0.0,
        1.0);
    spClientLogLevel = environmentInt(
        "HGASP_SP_CLIENT_LOG_LEVEL",
        spClientLogLevel,
        0,
        3);
    spServerTimeoutSeconds = environmentInt(
        "HGASP_SP_SERVER_TIMEOUT_SECONDS",
        spServerTimeoutSeconds,
        1,
        2000000000);
    spEnvironmentMaxAttempts = environmentInt(
        "HGASP_SP_ENVIRONMENT_MAX_ATTEMPTS",
        spEnvironmentMaxAttempts,
        1,
        10);
    spEnvironmentRetryBaseMilliseconds = environmentInt(
        "HGASP_SP_ENVIRONMENT_RETRY_BASE_MILLISECONDS",
        spEnvironmentRetryBaseMilliseconds,
        0,
        60000);
    spEnforceRepresentationRouteLimit = environmentBool(
        "HGASP_SP_REPRESENTATION_ROUTE_LIMIT",
        spEnforceRepresentationRouteLimit);

    const char* logFile = std::getenv("HGASP_SP_LOG_FILE");
    if (logFile != nullptr) {
        spLogFile = logFile;
    }
}

read_data::~read_data() {
	dimTabu1.clear();
	dimTabu2.clear();
	delete [] distanceStorage;
	delete [] D;
	delete [] timeStorage;
	delete [] T;
	delete [] x;
	delete [] y;
	delete [] demCus;
	delete [] invLoc;
	delete [] openCostLoc;
    if (conflictLoc != nullptr) {
        for (int i = 0; i < numL; ++i) {
            delete [] conflictLoc[i];
        }
    }
	delete [] conflictLoc;
}
void read_data::define(){
	dimTabu1.clear();
	dimTabu2.clear();

	D = new double *[numC];
	T = new double *[numC];
	distanceStorage = new double[
		static_cast<std::size_t>(numC) *
		static_cast<std::size_t>(numC)]{};
	timeStorage = new double[
		static_cast<std::size_t>(numC) *
		static_cast<std::size_t>(numC)]{};
	for (int i = 0; i < numC; ++i) {
		D[i] = distanceStorage +
			static_cast<std::size_t>(i) *
			static_cast<std::size_t>(numC);
		T[i] = timeStorage +
			static_cast<std::size_t>(i) *
			static_cast<std::size_t>(numC);
	}
	x=new double [numC];
	y=new double [numC];
	demCus=new int [numV];
	invLoc = new int [numC];
	openCostLoc = new double [numC];
	edgeNode = std::vector< std::vector < int > >(numV);
	customerToLoc = std::vector< std::vector < int > >(numV);
	edgeLocVec = std::vector < std::vector < int > > (numL);
	nearCity = std::vector< std::vector < int > >(numV);
	nearCityLoca = std::vector< std::vector < int > >(numL);
	conflictLoc = new int *[numL];
	for(int i=0;i<numL;i++) conflictLoc[i] = new int [numL];
	for (int i=0;i<numL;i++)
		for (int j=0;j<numL;j++)
			conflictLoc[i][j] = 0;
	locIncCus = std::vector<std::set<int>>(numL);
	readyTime.assign(
		static_cast<std::size_t>(numV), 0.0);
	dueTime.assign(
		static_cast<std::size_t>(numV),
		std::numeric_limits<double>::infinity());
	serviceTime.assign(
		static_cast<std::size_t>(numV), 0.0);
	depotOpenTime.assign(
		static_cast<std::size_t>(numL), 0.0);
	depotCloseTime.assign(
		static_cast<std::size_t>(numL),
		std::numeric_limits<double>::infinity());
	originalCustomerIds.resize(static_cast<std::size_t>(numV));
	originalDepotIds.resize(static_cast<std::size_t>(numL));
	for (int customer = 0; customer < numV; ++customer) {
		originalCustomerIds[static_cast<std::size_t>(customer)] = customer;
	}
	for (int depot = 0; depot < numL; ++depot) {
		originalDepotIds[static_cast<std::size_t>(depot)] = depot;
	}
}
void read_data::read_fun(
	std::string pathInstance,
	std::string pathDepotCon,
	std::string pathSolution,
	double bestDis,
	int isOptimal,
	int timeLimit,
	int iterLimit,
	const hgasp::ProblemMode requestedMode) {
	this->timeLimit = timeLimit;
	this->iterLimit = iterLimit;
	this->bestDis = bestDis;
	this->isOptimal = isOptimal;
	this->pathSolution = pathSolution;
	this->pathDepotCon = pathDepotCon;
	this->pathInstance = pathInstance;

	const hgasp::ParsedInstance parsed =
		hgasp::loadInstance(pathInstance);
	hgasp::validateModeCompatibility(
		parsed, requestedMode, pathInstance);

	problemMode = requestedMode;
	instanceFormat = parsed.format;
	numV = static_cast<int>(parsed.customers.size());
	numL = static_cast<int>(parsed.depots.size());
	numC = numV + numL;
	vehCap = parsed.vehicleCapacity;
	usingCostVeh = parsed.vehicleFixedCost;
	realOrInteger = parsed.legacyRealOrInteger;
	define();

	for (int customer = 0; customer < numV; ++customer) {
		const hgasp::ParsedCustomer& source =
			parsed.customers[static_cast<std::size_t>(customer)];
		x[customer] = source.x;
		y[customer] = source.y;
		demCus[customer] = source.demand;
		readyTime[static_cast<std::size_t>(customer)] =
			source.readyTime;
		dueTime[static_cast<std::size_t>(customer)] = source.dueTime;
		serviceTime[static_cast<std::size_t>(customer)] =
			source.serviceTime;
		originalCustomerIds[static_cast<std::size_t>(customer)] =
			source.originalId;
	}
	for (int depot = 0; depot < numL; ++depot) {
		const hgasp::ParsedDepot& source =
			parsed.depots[static_cast<std::size_t>(depot)];
		const int node = numV + depot;
		x[node] = source.x;
		y[node] = source.y;
		invLoc[node] = source.capacity;
		openCostLoc[node] = source.openingCost;
		depotOpenTime[static_cast<std::size_t>(depot)] =
			source.openTime;
		depotCloseTime[static_cast<std::size_t>(depot)] =
			source.closeTime;
		originalDepotIds[static_cast<std::size_t>(depot)] =
			source.originalId;
	}

	readDepotConfigurations();


	totalDemand=0;
	maxDemand=0;
	for (int i=0; i < numV; i++){
		if (demCus[i] > vehCap) {
			throw std::invalid_argument(
				"instance is infeasible: customer " +
				std::to_string(originalCustomerIds[
					static_cast<std::size_t>(i)]) +
				" demand exceeds vehicle capacity");
		}
		if (demCus[i] > maxDemand)
			maxDemand=demCus[i];
		totalDemand+=demCus[i];
	}
	if (totalDemand <= 0) {
		throw std::invalid_argument(
			"instance must contain a positive total customer demand");
	}
	long long aggregateDepotCapacity = 0;
	for (int depot = 0; depot < numL; ++depot) {
		aggregateDepotCapacity +=
			static_cast<long long>(invLoc[numV + depot]);
	}
	if (aggregateDepotCapacity <
		static_cast<long long>(totalDemand)) {
		throw std::invalid_argument(
			"instance is infeasible: aggregate depot capacity is smaller "
			"than total customer demand");
	}
	normalizaPara = double(numV) / double(totalDemand);
	avgDemand = double(totalDemand) / double(numV);
	//
	maxVnum=0;
	for (int i=0;i<numL;i++)
		maxVnum += Individual::representationRouteLimit(*this, i);
	// process the initial data

	isConstraintLoc = false;
	for (int i=numV;i<numC;i++){
		if (invLoc[i] <= totalDemand){
			isConstraintLoc = true;
			break;
		}
	}
	/***********************************************************************************************/
	/* Build distinct objective-cost and travel-time matrices. */
	maxDist=0;
	for (int i=0;i<numV;i++){
		for (int j=0;j<numV;j++){
			const double distance =
				euclideanDistance(x[i], y[i], x[j], y[j]);
			D[i][j] = applyRounding(
				distance, parsed.matrixPolicy.travelCostRounding);
			T[i][j] = applyRounding(
				distance, parsed.matrixPolicy.travelTimeRounding);
			if (distance > maxDist)
				maxDist=distance;
		}
	}
	for (int i=0; i < numV;i++){
		for (int j=numV; j<numC; j++){
			const double distance =
				euclideanDistance(x[i], y[i], x[j], y[j]);
			D[i][j] = applyRounding(
				distance, parsed.matrixPolicy.travelCostRounding);
			D[j][i] = D[i][j];
			T[i][j] = applyRounding(
				distance, parsed.matrixPolicy.travelTimeRounding);
			T[j][i] = T[i][j];
		}
	}
	for (int i=numV; i<numC; i++)
		for (int j=numV; j<numC; j++) {
			D[i][j] = 0;
			T[i][j] = 0;
		}
	// the most key element to decide promising depot configurations.
	/****************************************************************************/

	/****************************************************************************/
	/* construct sparse graph****************/
	std::vector < std::set < int > > setCorrelatedVertices = std::vector < std::set <int> >(numC);
	std::vector < std::pair <double, int> > orderProximity;
	for (int i = 0; i <numV; i++){
		orderProximity.clear();
		for (int j = 0; j <numC; j++)
			if (i != j) orderProximity.push_back(std::pair <double, int>(D[i][j], j));

		sort(orderProximity.begin(), orderProximity.end());
		for (int j = 0; j < std::min<int>(alpha, numV - 2);  j++){
			setCorrelatedVertices[i].insert(orderProximity[j].second);
			setCorrelatedVertices[orderProximity[j].second].insert(i);
		}
		for (int j=0;j < (int)orderProximity.size();j++)
			nearCity[i].push_back(orderProximity[j].second);
	}
	// Filling the vector of correlated vertices
	for (int i = 0; i <numV; i++)
		for (int x : setCorrelatedVertices[i])
			edgeNode[i].push_back(x);
	// to control the number of locations closed to customers.
	for (int i = 0; i< numV; i++){
		for (int j=0; j<(int)nearCity[i].size(); j++){
			if (nearCity[i][j] >= numV){
				customerToLoc[i].push_back(nearCity[i][j]);
				if ((int)customerToLoc[i].size() > numL/2.)
					break;
			}
		}
	}
	if (problemMode == hgasp::ProblemMode::ClrpTw) {
		// The legacy nearest-depot truncation is a search acceleration, not a
		// valid domain reduction for LRPTW.  Keep every depot for which serving
		// the customer alone is feasible.  With non-negative travel/service
		// times, singleton feasibility is necessary for the customer to appear
		// on any route from that depot.
		for (int customer = 0; customer < numV; ++customer) {
			customerToLoc[customer].clear();
			for (int depot = 0; depot < numL; ++depot) {
				const int depotNode = numV + depot;
				const double arrival =
					depotOpenTime[depot] + T[depotNode][customer];
				const double serviceStart =
					std::max(arrival, readyTime[customer]);
				const double returnTime =
					serviceStart + serviceTime[customer] +
					T[customer][depotNode];
				if (demCus[customer] <= vehCap &&
					demCus[customer] <= invLoc[depotNode] &&
					serviceStart <= dueTime[customer] + 1.0e-9 &&
					returnTime <= depotCloseTime[depot] + 1.0e-9) {
					customerToLoc[customer].push_back(depotNode);
				}
			}
			if (customerToLoc[customer].empty()) {
				throw std::runtime_error(
					"CLRP-TW instance is infeasible: customer " +
					std::to_string(originalCustomerIds[customer]) +
					" cannot be served from any candidate depot");
			}
		}
	}
	//
	std::vector < std::set < int > > setCorrelatedLocation = std::vector < std::set <int> >(numL);
	for (int i=0; i < numL; i++){
		orderProximity.clear();
		for (int j = 0; j <numV; j++)
			orderProximity.push_back(std::pair <double, int>(D[j][i+numV], j));
		sort(orderProximity.begin(), orderProximity.end());

		for (int j = 0; j < std::min<int>(alpha, numV - 2);  j++)
				setCorrelatedLocation[i].insert(orderProximity[j].second);

		for (int j=0;j < (int)orderProximity.size();j++)
			nearCityLoca[i].push_back(orderProximity[j].second);
	}
	for (int i = 0; i <numL; i++)
		for (int x : setCorrelatedLocation[i])
			edgeLocVec[i].push_back(x);
	/****************************************************///


	// Depot-configuration preprocessing was designed around spatial/capacity
	// proximity only.  In CLRP-TW mode it can discard the only temporally
	// feasible assignment, so the extension deliberately uses the existing
	// free-population path and lets the common objective choose depots.
	if (problemMode == hgasp::ProblemMode::ClrpTw) {
		no_capacity_location = true;
		numLocConfiguration = 0;
		LocConfigure.clear();
		compactLocConfigure.clear();
	} else {
		decideUsingDepotConfigurations();
	}

}
void read_data::decideUsingDepotConfigurations(){
	if (usingDepotConfiguration == 0)//not using
		analyzeLocation();
	else if (usingDepotConfiguration == 1){// only using
		std::set<int> array;
		LocConfigure.clear();
		for (int i=0;i<(int)depotCon.size();i++){
			array.clear();
			for (int j=0;j<(int)depotCon[i].size();j++)
				array.emplace (depotCon[i][j]);
			LocConfigure.push_back({0,array});
		}
		spanningTreeForLoc();
		std::sort(locRatio.begin(),locRatio.end());

	}
	else if (usingDepotConfiguration == 2){// mixed used
		analyzeLocation();
		std::set<long>binary;
		long var;
		int size;
		for (int i=0;i<(int)LocConfigure.size();i++){
			auto it = LocConfigure[i].second.begin();
			var = 0;
			while(it != LocConfigure[i].second.end()){
				var += std::pow(2,*it-numV);
				it++;
			}
			binary.emplace(var);
		}
		//
		std::set<int> array;
		for (int i=0;i<(int)depotCon.size();i++){
			array.clear();
			var = 0;
			for (int j=0;j<(int)depotCon[i].size();j++){
				array.emplace (depotCon[i][j]);
				var += std::pow(2, depotCon[i][j] - numV);
			}
			size = binary.size();
			binary.insert(var);
			if (size == (int)binary.size())continue;
			else LocConfigure.insert(LocConfigure.begin(),{0,array});
		}
	}
}

std::vector<std::string>read_data:: splitwithstr(std::string &str,char ch){
	  std::string tmpstr = str+ch;
	  std::vector<std::string> res;
	  if (str.size()<=0)
		return res;
	  size_t pos = tmpstr.find(ch);
	  while (pos!=std::string::npos){
			std::string child_str = tmpstr.substr(0, pos);
			if (child_str.size()>0)
			  res.push_back(child_str);
			tmpstr = tmpstr.substr(pos + 1);
			pos = tmpstr.find(ch);
	  }
	  if(res.empty()){
		  res.push_back(str);
		  return res;
	  }
	  return res;
}
//
void read_data::resetSearchCpuClock() noexcept {
	startTime = clock();
	meetBestTime = 0.0;
}

double read_data::elapsedCpuSeconds() const noexcept {
	return static_cast<double>(clock() - startTime) /
		static_cast<double>(CLOCKS_PER_SEC);
}

void read_data::recordGlobalIncumbent() noexcept {
	meetBestTime = elapsedCpuSeconds();
}

bool read_data::stopCondition(double dis){
	const double elapsedTime = elapsedCpuSeconds();
	if (timeLimit > 0 && elapsedTime >= timeLimit)
		return true;
	if (iteration >= iterLimit)
		return true;
	if (meetOptimal && bestDis > dis +Min){
		timeToTarget = elapsedCpuSeconds();
		meetOptimal = false;
	}
	if (std::abs(bestDis - dis) < 0.01){
		if (isOptimal)return true;
	}
	return false;
}
bool read_data::equalTwoVar(double A, double B){
	if (abs(A-B) < Min)return true;
	else return false;
}
//////////////////////////////////////////////////to configure the promising locations.
void read_data::analyzeLocation(){
//	for (int i=numV; i<numC;i++)
//		locRatio.push_back({- double(invLoc[i]) / double(openCostLoc[i]), i});

	spanningTreeForLoc();
	if (numL>=10)analyzeConflict();
	//
//	solutionTabuSearchFindCompactConfiguration();
//	if (locCostSignificant == 1 || locCostSignificant == 0)
//		tabuSearchFindCompactConfiguration();
//	else
	int min_depot = Max;
	int max_depot = 0;
	for (int i=numV;i<numC;i++){
		if (min_depot > invLoc[i])
			min_depot = invLoc[i];
		if (max_depot < invLoc[i])
			max_depot = invLoc[i];
	}
	if (min_depot > totalDemand){
		std::vector<int> array(numL);
		for (int i=0;i<numL;i++){
			array[i] = invLoc[i+numV];
			invLoc [i+numV] = (double)totalDemand / ((double)numL / 2.);
		//	invLoc[i+numV] = 0;
		}
	//	for (int i=0;i<numV;i++){
	//		invLoc[customerToLoc[i][0]] += demCus[i] * 2;
	//	}


		constructedDepotConfigurations();
		std::cout<<"The number of Depot Configurations is: "<<(int)LocConfigure.size()<<std::endl;
	//	if ((int)LocConfigure.size() < 500){
			std::cout<<"triggering the tabu search to find more depot configurations"<<std::endl;
		//	tabuSearchFindCompactConfiguration();

		for (int i=0;i<numL;i++){
			invLoc[i+numV] = array[i];
		}
//		tabuSearchFindCompactConfiguration();
//		no_capacity_location = true;
		std::sort(locRatio.begin(),locRatio.end());
		return;
	}

	if (numL < 10)
		tabuSearchFindCompactConfiguration();
	else{
		constructedDepotConfigurations();
		std::cout<<"The number of Depot Configurations is: "<<(int)LocConfigure.size()<<std::endl;
	//	if ((int)LocConfigure.size() < 500){
			std::cout<<"triggering the tabu search to find more depot configurations"<<std::endl;
			tabuSearchFindCompactConfiguration();
	//	}
	}

	std::sort(LocConfigure.begin(), LocConfigure.end());
	std::cout<<"finishing the depot configurations"<<std::endl;
	std::sort(locRatio.begin(),locRatio.end());
}
void read_data::analyzeConflict(){
	double min_x = Max, min_y = Max, max_x = 0,max_y = 0;
	double range_x, range_y;
	for (int i=0;i<numC;i++){
		if (x[i] < min_x) min_x = x[i];
		if (y[i] < min_y) min_y = y[i];
		if (x[i] > max_x) max_x = x[i];
		if (y[i] > max_y) max_y = y[i];
	}
	double range_loc, min_loc = Max, max_loc = 0;
	for (int i=numV;i<numC;i++){
		if (invLoc[i] < min_loc)min_loc = invLoc[i];
		if (invLoc[i] > max_loc)max_loc = invLoc[i];
	}
	range_loc = max_loc - min_loc;
	range_x = max_x - min_x;
	range_y = max_y - min_y;
	//decide the instance whether fixed location costs significant.
	double avgCost = 0;
	int count = 0;
	for (int i=0;i<numV;i++){
		for (int j=0;j<(int)edgeNode[i].size();j++){
			avgCost += edgeNode[i][j];
			count ++;
		}
	}
	avgCost = avgCost / count;
	//double travelCost = avgCost * numV;
	double locCost = 0;
	for (int i=0;i<numL;i++)
		locCost += openCostLoc[i + numV];
	// there are two cases. the first case is type C and the other cases are all other instances
//	if (travelCost > locCost * 10)// this type is not complete since types a b d are not
//		locCostSignificant = 0;// type c
//	else if (travelCost * 100 < locCost)
//		locCostSignificant = 1;// type e
//	else locCostSignificant = 2;
	//the following is to decide some locations conflict
	double dis_measure = (range_x + range_y) / 2.;
	double ratio = range_loc / min_loc;// if the value is greater than 1, it is type e, otherwise it is other type.
	double xx,yy,xy;
	for (int i=numV;i<numC;i++){
		if (ratio > 1 && ((double)invLoc[i] / (double)totalDemand > 0.2)){// the type e
			if (max_loc / invLoc[i] > 1.8)continue;
			for (int j = i; j < numC; j++){
				if (i == j || ((double)invLoc[i] / (double)invLoc[j] > 1.8))continue;
				xx = std::abs(x[i] - x[j]);
				yy = std::abs(y[i] - y[j]);
				xy = (xx + yy) / 2.;
				if (xy / dis_measure < (double)invLoc[i] / (double)totalDemand){
					conflictLoc[i - numV][j - numV] = 1;
					conflictLoc[j - numV][i - numV] = 1;
				}
			}
		}
		if (ratio < 1 && (double)invLoc[i] / (double)totalDemand > 0.1){// the type a b d
			for (int j = i; j < numC; j++){
				if (i == j)continue;
				xx = std::abs(x[i] - x[j]);
				yy = std::abs(y[i] - y[j]);
				xy = (xx + yy) / 2.;
				if (2.0*xy / dis_measure < (double)invLoc[i] / (double)totalDemand){// the coefficient 2 is used to not limit too much
					conflictLoc[i - numV][j - numV] = 1;
					conflictLoc[j - numV][i - numV] = 1;
				}
			}
		}
	}
/*
	for (int i=0;i<numL;i++){
		for (int j=0;j<numL;j++)
			std::cout<<conflictLoc[i][j]<<" ";
		std::cout<<std::endl;
	}
*/
}
void read_data::spanningTreeForLoc(){
	int * check_cus;
	check_cus = new int [numV];

	double * low_cost;
	low_cost = new double [numV];

	//////////////the following is the spanning tree method
	double min_dis;
	double total_dis_loc;
	int total_dem;
	int v;
	for (int l=0;l<numL;l++){
		for (int i=0;i<numV;i++)check_cus[i] = 0;
		total_dis_loc = 0;
		total_dem = 0;
		for (int j=0;j<numV;j++)
			low_cost[j]=D[l+numV][j];
		int numVehicle = 0;
		int selectedCustomers = 0;
		while(selectedCustomers < numV){
			int totalVeh = 0;
			bool madeProgress = false;
			for (int i=0;i< numV;i++){
				min_dis=Max;
				v=-1;
				for (int j=0;j<numV;j++){
					if (check_cus[j]==0 && min_dis>low_cost[j]){
						v=j;
						min_dis=low_cost[j];
					}
				}
				if (v!=-1){
					if (total_dem + demCus[v] >= invLoc[l+numV] || totalVeh >= vehCap)break;
					check_cus[v] = 1;
					selectedCustomers++;
					madeProgress = true;
					locIncCus[l].insert(v);
					total_dis_loc += min_dis;
					total_dem += demCus[v];
					totalVeh += demCus[v];
					for (int k=0;k<numV;k++){
						if (check_cus[k]==0 && D[k][v]<low_cost[k])
							low_cost[k]=D[k][v];
					}
				}
			}
			numVehicle++;
			// A depot with capacity above the total demand can exhaust every
			// unchecked customer.  The legacy code then dereferenced demCus[-1]
			// and could spin forever.  No progress means the finite Prim-style
			// construction is complete (or the next customer cannot fit).
			if (!madeProgress || v < 0 || selectedCustomers >= numV)
				break;
			if (total_dem + demCus[v] >= invLoc[l+numV])
				break;
			for (int j=0;j<numV;j++)
				if (check_cus[j] == 1)
					low_cost[j]=Max;

		}
		double cost_loc = openCostLoc[l + numV] + total_dis_loc + numVehicle * usingCostVeh;
	//	locRatio.push_back({ cost_loc / double(total_dem), l+numV});
		locRatio.push_back({(double)cost_loc, l+numV});
	}
	delete [] check_cus;
	delete [] low_cost;
	//
/*	double total_dis_loc;
	int total_dem;
	for (int l=0;l<numL;l++){
		for (int i=0;i<numV;i++)check_cus[i] = 0;
		total_dis_loc = 0;
		total_dem = 0;
		for (int i=0;i<numV;i++){
			if (total_dem + demCus[nearCityLoca[l][i]] <= invLoc[l+numV]){
				total_dem += demCus[nearCityLoca[l][i]];
				total_dis_loc += D[l+numV][i];
				locIncCus[l].insert(nearCityLoca[l][i]);
			}
			else break;
		}
		double cost_loc = openCostLoc[l + numV] + total_dis_loc;
	//	locRatio.push_back({ cost_loc / double(total_dem), l+numV});
		locRatio.push_back({(double)cost_loc, l+numV});
	}
*/
}

void read_data::tabuSearchFindCompactConfiguration(){
	std::vector<std::set<int>> listLocConfigure;
	// using tabu search to determine that does this instance has perfect locations configurations
	int *sol,*best_sol, *tabu_table, *dem;
	sol = new int [numL];
	best_sol = new int [numL];
	tabu_table = new int [numL];
	for (int i=0;i<numL;i++)tabu_table[i] = 0;
	dem = new int [numL];
	int cusDem = 0;
	int obj=0,best_obj;
	// construct a initial solution
	int count;
	for (int i=0;i<numL;i++){
		dem[i] = locRatio[i].first;
	}
	const int maximumConstructionAttempts =
		std::max(100, numL * 100);
	int constructionAttempt = 0;
	while(constructionAttempt++ < maximumConstructionAttempts){
		for (int i=0;i<numL;i++)sol[i] = 0;
		cusDem = 0;
		obj = 0;
		int seed = RandomInt(0, numL - 1);
		int cumLoc = 0;
		count = 0;
		while(1){
			cumLoc += invLoc[seed+numV];
			sol[seed] = 1;
			cusDem += invLoc[seed+numV];
			obj += dem[seed];
				if (cumLoc >= totalDemand)break;
			count = 0;
			while(1){
				seed = RandomInt(0, numL - 1);
				if (sol[seed] == 0){
					bool isConfilict = false;
					for (int i=0;i<numL;i++){
						if (sol[i] == 1){
							if(conflictLoc[i][seed] == 1){
								isConfilict = true;
								break;
							}
						}
					}
					if (!isConfilict)break;
				}
				count ++;
				if (count > 1000)
					break;
			}
			if (count > 1000)break;
		}
			if (count < 1000)break;
	}
	if (constructionAttempt > maximumConstructionAttempts) {
		// conflictLoc is only a preprocessing heuristic.  Falling back to all
		// depots preserves the original CLRP domain and deterministically
		// supplies a capacity-feasible starting configuration.
		cusDem = 0;
		obj = 0;
		for (int depot = 0; depot < numL; ++depot) {
			sol[depot] = 1;
			cusDem += invLoc[depot + numV];
			obj += dem[depot];
		}
		if (cusDem < totalDemand) {
			throw std::runtime_error(
				"depot-configuration preprocessing found insufficient "
				"aggregate capacity");
		}
	}
	int seed;
	best_obj = obj;
	// The tabu walk records configurations after its first move.  A
	// capacity-tight construction can already be locally immovable, so retain
	// the feasible seed before entering the neighborhood loop.
	std::set<int> initialDepots;
	for (int depot = 0; depot < numL; ++depot) {
		if (sol[depot] == 1)
			initialDepots.insert(depot + numV);
	}
	if (cusDem >= totalDemand && !initialDepots.empty()) {
		LocConfigure.push_back({static_cast<double>(obj), initialDepots});
	}
	int can_node_1[100],can_node_2[100];
	int can_tabu_node_1[100],can_tabu_node_2[100];
	//
	int delta,min_delta, min_tabu_delta;
	int num_best, num_tabu_best;
	int no_improve = 0;
	double tabuLength = 0.3;
	int length_improve = 2000;
	int tabu_length = ceil(numL * tabuLength);
	int step = 1;
	int loc1, loc2;

	while (1){
		min_delta=Max;min_tabu_delta=Max;num_best=0;num_tabu_best=0;
		for (int i=0;i<numL;i++){
			if (sol[i] == 1){
				if (cusDem - invLoc[i+numV] >= totalDemand){
					delta = -dem[i];
					if (tabu_table[i] < step){
						if (delta < min_delta){
							min_delta = delta;
							can_node_1[0] = i;
							can_node_2[0] = -1;
							num_best = 1;
						}
						else if (delta == min_delta && num_best < 100){
							can_node_1[num_best] = i;
							can_node_2[num_best] = -1;
							num_best ++;
						}
					}
					else if (tabu_table[i] > step){
						if (delta < min_tabu_delta){
							min_tabu_delta = delta;
							can_tabu_node_1[0] = i;
							can_tabu_node_2[0] = -1;
							num_tabu_best = 1;
						}
						else if (delta == min_tabu_delta && num_tabu_best < 100){
							can_tabu_node_1[num_tabu_best] = i;
							can_tabu_node_2[num_tabu_best] = -1;
							num_tabu_best ++;
						}
					}
				}
				//
				for (int j=0; j<numL;j++){
					if (i == j || sol[j] == 1)continue;
					if (cusDem - invLoc[i+numV] + invLoc[j+numV] < totalDemand)continue;
					//decide conflict
					seed = j;
					bool isConfilict = false;
					for (int ii=0;ii<numL;ii++){
						if (ii == i)continue;
						if (sol[ii] == 1){
							if(conflictLoc[ii][seed] == 1){
								isConfilict = true;
								break;
							}
						}
					}
					if (isConfilict)continue;
					//

					delta = dem[j] - dem[i];
					if (tabu_table[i] <= step && tabu_table[j] <= step){
						if (delta < min_delta){
							min_delta = delta;
							can_node_1[0] = i;
							can_node_2[0] = j;
							num_best = 1;
						}
						else if (delta == min_delta && num_best < 100){
							can_node_1[num_best] = i;
							can_node_2[num_best] = j;
							num_best ++;
						}
					}
					else if (tabu_table[i] > step || tabu_table[j]> step ){
						if (delta < min_tabu_delta){
							min_tabu_delta = delta;
							can_tabu_node_1[0] = i;
							can_tabu_node_2[0] = j;
							num_tabu_best = 1;
						}
						else if (delta == min_tabu_delta && num_tabu_best < 100){
							can_tabu_node_1[num_tabu_best] = i;
							can_tabu_node_2[num_tabu_best] = j;
							num_tabu_best ++;
						}
					}
				}
			}
			else{
				seed = i;
				bool isConfilict = false;
				for (int ii=0;ii<numL;ii++){
					if (sol[ii] == 1){
						if(conflictLoc[ii][seed] == 1){
							isConfilict = true;
							break;
						}
					}
				}
				if (isConfilict)continue;
				////////////////////////////////////////////////////
				delta = dem[i];
				if (tabu_table[i] <= step){
					if (delta < min_delta){
						min_delta = delta;
						can_node_1[0] = i;
						can_node_2[0] = -2;
						num_best = 1;
					}
					else{
						can_node_1[num_best] = i;
						can_node_2[num_best] = -2;
						num_best ++;
					}
				}
				else if (tabu_table[i] > step){
					if (delta < min_tabu_delta){
						min_tabu_delta = delta;
						can_tabu_node_1[0] = i;
						can_tabu_node_2[0] = -2;
						num_tabu_best = 1;
					}
					else if (delta == min_tabu_delta && num_tabu_best < 100){
						can_tabu_node_1[num_tabu_best] = i;
						can_tabu_node_2[num_tabu_best] = -2;
						num_tabu_best ++;
					}
				}
			}
		}
		if (min_tabu_delta == Max && min_delta == Max)break;
		int rand_order;
		if ((num_tabu_best>0 && min_tabu_delta < min_delta && (obj + min_tabu_delta< best_obj)) || num_best==0){
			obj += min_tabu_delta;
			rand_order = RandomInt(0, num_tabu_best - 1);
			loc1 =can_tabu_node_1[rand_order];
			loc2 =can_tabu_node_2[rand_order];
			tabu_table[loc1]=step+tabu_length+RandomInt(0, tabu_length - 1);
			if (loc2 >-1)
				tabu_table[loc2]=step+tabu_length+RandomInt(0, tabu_length - 1);
		}
		else{
			obj += min_delta;
			rand_order = RandomInt(0, num_best - 1);
			loc1 = can_node_1[rand_order];
			loc2 = can_node_2[rand_order];
			tabu_table[loc1]=step+tabu_length+RandomInt(0, tabu_length - 1);
			if (loc2 > -1)
				tabu_table[loc2]=step+tabu_length+RandomInt(0, tabu_length - 1);
		}
		if (loc2 > -1){
			sol[loc1] = 0;
			sol[loc2] = 1;
			cusDem += invLoc[loc2 + numV];
			cusDem = cusDem - invLoc[loc1 +numV];
		}
		else if (loc2 == -1){
			sol[loc1] = 0;
			cusDem = cusDem - invLoc[loc1 + numV];
		}
		else if (loc2 == -2){
			sol[loc1] = 1;
			cusDem += invLoc[loc1 + numV];
		}
		//
		if (best_obj > obj){
			best_obj = obj;
			for (int i=0;i<numL;i++)
				best_sol[i] = sol[i];
		}
		else	no_improve ++;
		step++;
		if (no_improve > length_improve){
	//		disturbation(sol, cusDem);
	//		no_improve = 0;

			break;
		}
		//
/*		for (int i=0;i<numL;i++)
			std::cout<<i<<" ";
		std::cout<<std::endl;
		for (int i=0;i<numL;i++)
			std::cout<<sol[i]<<" ";
		std::cout<<std::endl;
*/
		//
		std::set<int>depot;
		double cost = 0;
		for (int i=0;i<numL;i++)
			if (sol[i] == 1){
				depot.insert(i+numV);
				cost += dem[i];
			}
		////////////////////////////
		int sumDem = 0;
		for (int i=0;i<numL;i++){
			if (sol[i] == 1)
				sumDem += invLoc[i+numV];
		}
		bool isContinue = false;
		for (int i=0;i<numL;i++){
			if (sol[i] == 1)
				if (sumDem - invLoc[i+numV] >= totalDemand){
					isContinue = true;
					break;
				}
		}
		if (isContinue)continue;
		/////////////////////////////


		bool same = true;
		for (int i=0;i<(int)LocConfigure.size();i++){
			if ((int)depot.size() == (int)LocConfigure[i].second.size()){
				auto it1 = depot.begin();
				auto it2 = LocConfigure[i].second.begin();
				bool theSame = true;
				while(it1 != depot.end()){
					if (*it1 != *it2){
						theSame = false;break;
					}
					it1 ++;
					it2 ++;
				}
				if (theSame)same = false;
			}
		}
		if (same){
			if ((int)LocConfigure.size() >= numLocConfiguration){
				std::sort(LocConfigure.begin(), LocConfigure.end());
				LocConfigure.pop_back();
			}
			LocConfigure.push_back({cost,depot});
		}
		int sum =0;
		for (int i=0;i<numL;i++)
			sum += sol[i];

//		if (sol[4] == 1 && sol[6] == 1 && sol[14] == 1 && sol[16] == 1)
//			std::cout<<" the best results  "<<std::endl;
	}
	delete [] sol;
	delete [] best_sol;
	delete [] tabu_table;
	delete [] dem;
	//
//	std::sort(LocConfigure.begin(), LocConfigure.end());
	//
/*	std::set<int>depot;
	depot.insert(304);
	depot.insert(306);depot.insert(314);depot.insert(316);
	LocConfigure.push_back({100,depot});
	std::sort(LocConfigure.begin(), LocConfigure.end());
*/
/*  this code repeats the evaluation logic described by Schneider et al.
	std::set<int>depot;
	depot.insert(200);
	depot.insert(203);depot.insert(204);depot.insert(205);depot.insert(206);depot.insert(210);
	LocConfigure.push_back({100,depot});
	std::sort(LocConfigure.begin(), LocConfigure.end());
	return;

	LocConfigure.push_back({100,depot});
	std::vector<int>loc;
	for (int i=0;i<(int)LocConfigure.size();i++){
		loc.clear();
		auto it = LocConfigure[i].second.begin();
		for (;it != LocConfigure[i].second.end();it++)
			loc.push_back(*it);
		//computing the rating function
		double costLoc = 0;
		double part1 = 0,part2 = 0;
		for (int j=0;j<(int)loc.size();j++){
			for (int k=0;k<numV;k++)
				part1 += demCus[k] * pow(0.95, (D[loc[j]][k]* 2)/100);

			for (int k=0;k<(int)loc.size();k++)
				part2 += invLoc[loc[k]] * pow(0.95, (D[loc[j]][loc[k]] * 2)/100);
			costLoc += std::abs(part1 - part2) + openCostLoc[loc[j]];
		}
		LocConfigure[i].first = costLoc;
	}
	std::sort(LocConfigure.begin(), LocConfigure.end());
*/



/*	for (int i=0;i < (int)LocConfigure.size(); i++){
		auto it = LocConfigure[i].second.begin();
		while(it != LocConfigure[i].second.end()){
			std::cout<<*it<<" ";
			it ++;
		}
		std::cout<<std::endl;
	}
	*/
}
void read_data::constructedDepotConfigurations(){
	//the line function to estimate the coverRatio
	double min_coverRatio = 0.1;
	double max_coverRatio = 0.6;
	double kParameter = max_coverRatio - min_coverRatio;
	//
	solutionTabu = new int [dimension];
	for (int i=0;i<dimension;i++)
		solutionTabu[i] = 0;
	int seed;
	double coverRatio = 0.2;// assuming the covering ratio is 50%
	double costDepotConf;
	int countOver;
	int iter;
	int totalDem;
	std::set<int>coverVertex;
	std::vector<std::pair<double, int>>priorityOrder;// the first is the ratio of overlaping vertices and the second is the order of depot
	std::set<int>sol;
	std::vector<int>depot;
	std::vector<std::pair<double, int>>orderDepots;
	std::set<int>array;
	std::set<int>array2;
	int iteration = 0;
	int no_get = 0;
	bool isUsed;
	isUsed = true;
	while(1){
		sol.clear();
		depot.clear();
		coverVertex.clear();
		orderDepots.clear();
		for (int i=0;i<numL;i++)depot.push_back(i);
		seed = RandomInt(0, numL - 1);
		costDepotConf = locRatio[seed].first;
		sol.insert(seed);
		totalDem = invLoc[seed+numV];
		depot.erase(depot.begin()+seed);//
		//
		auto it = locIncCus[seed].begin();
		for (;it != locIncCus[seed].end(); it++) coverVertex.insert(*it);
		while(totalDem < totalDemand){// select other depots
			priorityOrder.clear();
			for (int i=0;i<(int)depot.size();i++){//deicde the overlapping vertices
				countOver = 0;iter = 0;
				auto it1 = locIncCus[depot[i]].begin();
				for (;it1 != locIncCus[depot[i]].end(); it1++){
					array2.insert(*it1);

					auto it2 = coverVertex.find(*it1);
					if (it2 != coverVertex.end()){
						countOver ++;
					}
					iter ++;
				}

				double var = (double)countOver / (double)iter;
				priorityOrder.push_back({var, depot[i]});
			}
			sort(priorityOrder.begin(), priorityOrder.end());
			orderDepots.clear();
			for (int i=0;i<(int)priorityOrder.size();i++){
				int cap = invLoc[priorityOrder[i].second + numV] + totalDem;
				coverRatio = kParameter * ((double)cap / (double)totalDemand) + min_coverRatio;
				if (priorityOrder[i].first > coverRatio)break;// discard these non-promising depots
				else orderDepots.push_back({locRatio[priorityOrder[i].second].first,priorityOrder[i].second});
			}

			if ((int)orderDepots.size() == 0){
				if (isUsed){
					if (iteration > 100 && (int)LocConfigure.size() == 0)
						isUsed = false;
					break;
				}
					else{
						int relaxationSteps = 0;
						while ((int)orderDepots.size() == 0 &&
							   !priorityOrder.empty() &&
							   relaxationSteps++ < 20){
							coverRatio += 0.1;
						for (int i=0;i<(int)priorityOrder.size();i++){
						//	int cap = invLoc[priorityOrder[i].second + numV] + totalDem;
						//	coverRatio = kParameter * ((double)cap / (double)totalDemand) + min_coverRatio;
							if (priorityOrder[i].first > coverRatio)break;// discard these non-promising depots
							else orderDepots.push_back({locRatio[priorityOrder[i].second].first,priorityOrder[i].second});
							}
						}
						if (orderDepots.empty())
							break;
					}
				}

			sort(orderDepots.begin(),orderDepots.end());

			double y = RandomDouble(0.0, 1.0);
			int rank=int(pow(y,determine_para)*(int)orderDepots.size());

			seed = orderDepots[rank].second;

			auto it1 = locIncCus[seed].begin();
			for (;it1 != locIncCus[seed].end(); it1++)
				coverVertex.insert(*it1);
			int index = -1;
			for (int i=0;i<(int)depot.size();i++)
				if (depot[i] == seed){
					index = i;
					break;
				}
			depot.erase(depot.begin() + index);
			sol.insert(seed);
			costDepotConf += locRatio[seed].first;
			totalDem += invLoc[seed+numV];
		}
		if (totalDem < totalDemand){
			no_get ++;
			iteration ++;
			if (no_get > 1000) {
				std::sort(LocConfigure.begin(), LocConfigure.end());
				std::cout
					<<"finishing construction depots configurations"
					<<std::endl;
				break;
			}
			continue;
		}


		it = sol.begin();
		array.clear();
		long long int binaryVal = 0;
		for (;it != sol.end();it++){
			array.insert(*it + numV);
			binaryVal += pow(2,*it);
		}
		// solution based
		if(solutionTabu[binaryVal % dimension] == 0){
			solutionTabu[binaryVal % dimension] = 1;
			//
			LocConfigure.push_back({costDepotConf,array});
			if ((int)LocConfigure.size() >= numLocConfiguration){
				std::sort(LocConfigure.begin(), LocConfigure.end());
				std::cout<<"finishing construction depots configurations"<<std::endl;
				break;
			}
			no_get = 0;
		}
		else no_get ++;
		if (no_get > 1000){
			std::sort(LocConfigure.begin(), LocConfigure.end());
			std::cout<<"finishing construction depots configurations"<<std::endl;
			std::cout
				<< "instance preprocessing process CPU time  "
				<< elapsedCpuSeconds() << std::endl;
			break;
		}
		iteration++;
	//	std::cout<<iteration<<"   "<<(int)LocConfigure.size()<<std::endl;
	}
	delete [] solutionTabu;
}

void read_data::readDepotConfigurations(){
	std::vector<int>array;
	std::string var;
	if (pathDepotCon != "NULL"){
		std::ifstream inputFile(pathDepotCon);
			if (inputFile.is_open()){
				int numDepotCon;
				if (!(inputFile >> numDepotCon) || numDepotCon < 0) {
					throw std::runtime_error(
						"invalid depot configuration header: " +
						pathDepotCon);
				}
				for (int i=0;i<numDepotCon;i++){
					array.clear();
					if (!(inputFile >> var)) {
						throw std::runtime_error(
							"unexpected end of depot configuration file: " +
							pathDepotCon);
					}
					while(var != "e"){
						array.push_back(std::stoi(var) + numV);
						if (!(inputFile >> var)) {
							throw std::runtime_error(
								"missing 'e' terminator in depot "
								"configuration file: " + pathDepotCon);
						}
					}
				depotCon.push_back(array);
			}
		}
        else {
            throw std::runtime_error(
                "failed to open depot configuration file: " +
                pathDepotCon);
        }
    }
    else {
        return;
    }
}
