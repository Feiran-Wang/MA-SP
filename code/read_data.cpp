

#include "read_data.h"
#include "basic.h"
#include "Individual.h"
#include "JsonInput.h"
#include <limits>
#include <iterator>
#include <stdexcept>
read_data::read_data() : SearchParameters(SearchParameters::setS(0)) {

}
read_data::~read_data() {
	delete [] dimTabu1;
	delete [] dimTabu2;
	if (D) for (int i=0;i<numC;i++) delete [] D[i];
	delete [] D;
	delete [] x;
	delete [] y;
	delete [] demCus;
	delete [] invLoc;
	delete [] openCostLoc;
	if (conflictLoc) for (int i=0;i<numL;i++) delete [] conflictLoc[i];
	delete [] conflictLoc;
}
void read_data::define(){
	// Resolve the family and size once, before allocating parameter-sized storage.
	const bool study = SearchParameters::experimentEnabled();
	const bool setSStudy = SearchParameters::setSExperimentEnabled();
	if (study && hardTimeWindows == setSStudy)
		throw std::invalid_argument("Experiment family does not match the instance parser");
	static_cast<SearchParameters&>(*this) = study
		? (setSStudy ? SearchParameters::setSExperiment(numV) : SearchParameters::setFExperiment(numV, numL))
		: hardTimeWindows ? SearchParameters::setF(numV, numL) : SearchParameters::setS(numV);
    if (experimentMode && spTriggerFrequency > 0) {
        spSchedule.clear();
        for (long long at = spTriggerFrequency; at <= iterLimit; at += spTriggerFrequency)
            spSchedule.push_back(static_cast<int>(at));
    }
	dimTabu1 = new int [dimension];
	dimTabu2 = new int [dimension];
	for (int i=0;i<dimension;i++){
		dimTabu1 [i] = -1;
		dimTabu2 [i] = -1;
	}

	D=new double *[numC]();
	for (int i=0;i<numC;i++) D[i]=new double [numC];
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
	conflictLoc = new int *[numL]();
	for(int i=0;i<numL;i++) conflictLoc[i] = new int [numL];
	for (int i=0;i<numL;i++)
		for (int j=0;j<numL;j++)
			conflictLoc[i][j] = 0;
	locIncCus = std::vector<std::set<int>>(numL);
}
void read_data::read_fun(std::string pathInstance,std::string pathDepotCon,std::string pathSolution, double bestDis, int isOptimal, int timeLimit,int iterLimit){

    this->timeLimit = timeLimit;
    this->iterLimit = iterLimit;
    this->bestDis = bestDis;
    this->isOptimal = isOptimal;
    this->pathSolution = pathSolution;
    this->pathDepotCon = pathDepotCon;
    this->pathInstance = pathInstance;
    if (D) throw std::logic_error("read_data can load one instance only");
    if (timeLimit <= 0 || iterLimit < 0 || !std::isfinite(bestDis) || bestDis < 0 ||
        (isOptimal != 0 && isOptimal != 1) || (isOptimal && bestDis <= 0))
        throw std::invalid_argument("invalid run limits or reference value");
    const auto extensionAt = pathInstance.find_last_of('.');
    const std::string extension = extensionAt == std::string::npos ? "" : pathInstance.substr(extensionAt + 1);
    std::ifstream inputFile(pathInstance);
    if (!inputFile) throw std::runtime_error("cannot open instance: " + pathInstance);
    auto integer = [](double value, const std::string& field) {
        if (!std::isfinite(value) || std::floor(value) != value ||
            value < std::numeric_limits<int>::min() || value > std::numeric_limits<int>::max())
            throw std::runtime_error(field + " must be a representable integer");
        return static_cast<int>(value);
    };
    auto dimensions = [&]() {
        if (numV <= 0 || numL <= 0 || numL > 62 || numV > std::numeric_limits<int>::max() - numL ||
            numV > std::numeric_limits<int>::max() / numL)
            throw std::runtime_error("invalid or oversized customer/depot counts");
        numC = numV + numL;
    };
    if (extension == "txt") {
        readFarham(pathInstance);
        if (isOptimal && std::floor(bestDis) != bestDis)
            throw std::runtime_error("Set F requires an integer reference when isOptimal is set");
    } else if (extension == "json" || extension == "dat") {
        std::vector<double> coordinatesX, coordinatesY, openingCosts;
        std::vector<int> demands, inventories;
        if (extension == "json") {
            const std::string source((std::istreambuf_iterator<char>(inputFile)), std::istreambuf_iterator<char>());
            if (inputFile.bad()) throw std::runtime_error("instance read failed");
            const auto root = masp_input::JsonParser(source).parse();
            const auto& customers = root.required("customers");
            const auto& depots = root.required("depots");
            if (customers.kind != masp_input::Json::Array || depots.kind != masp_input::Json::Array ||
                customers.array.size() > std::size_t(std::numeric_limits<int>::max()) ||
                depots.array.size() > std::size_t(std::numeric_limits<int>::max()))
                throw std::runtime_error("customers and depots must be bounded JSON arrays");
            numV = static_cast<int>(customers.array.size());
            numL = static_cast<int>(depots.array.size());
            dimensions();
            for (const auto& customer : customers.array) {
                coordinatesX.push_back(customer.numeric("x"));
                coordinatesY.push_back(customer.numeric("y"));
                demands.push_back(integer(customer.numeric("demand"), "customer demand"));
            }
            for (const auto& depot : depots.array) {
                coordinatesX.push_back(depot.numeric("x"));
                coordinatesY.push_back(depot.numeric("y"));
                inventories.push_back(integer(depot.numeric("capacity"), "depot capacity"));
                openingCosts.push_back(depot.numeric("costs"));
            }
            vehCap = integer(root.numeric("vehicle_capacity"), "vehicle capacity");
            usingCostVeh = root.numeric("vehicle_costs");
            realOrInteger = 0;
        } else {
            auto token = [&](const std::string& field) {
                std::string text;
                if (!(inputFile >> text)) throw std::runtime_error("missing DAT field: " + field);
                std::size_t used = 0;
                double number;
                try { number = std::stod(text, &used); }
                catch (const std::exception&) { throw std::runtime_error("invalid DAT field: " + field); }
                if (used != text.size() || !std::isfinite(number))
                    throw std::runtime_error("invalid DAT field: " + field);
                return number;
            };
            numV = integer(token("customer count"), "customer count");
            numL = integer(token("depot count"), "depot count");
            dimensions();
            coordinatesX.resize(numC); coordinatesY.resize(numC);
            demands.resize(numV); inventories.resize(numL); openingCosts.resize(numL);
            for (int i = numV; i < numC; ++i) {
                coordinatesX[i] = token("depot x"); coordinatesY[i] = token("depot y");
            }
            for (int i = 0; i < numV; ++i) {
                coordinatesX[i] = token("customer x"); coordinatesY[i] = token("customer y");
            }
            vehCap = integer(token("vehicle capacity"), "vehicle capacity");
            for (int& capacity : inventories) capacity = integer(token("depot capacity"), "depot capacity");
            for (int& demand : demands) demand = integer(token("customer demand"), "customer demand");
            for (double& cost : openingCosts) cost = token("depot opening cost");
            usingCostVeh = token("vehicle cost");
            realOrInteger = integer(token("metric flag"), "metric flag");
            std::string extra;
            if (inputFile >> extra) throw std::runtime_error("unexpected data after DAT metric flag");
            if (inputFile.bad()) throw std::runtime_error("instance read failed");
        }
        if (vehCap <= 0 || usingCostVeh < 0 || !std::isfinite(usingCostVeh) ||
            (realOrInteger != 0 && realOrInteger != 1))
            throw std::runtime_error("invalid vehicle capacity, cost or metric flag");
        long long demandSum = 0, inventorySum = 0;
        for (int demand : demands) {
            if (demand <= 0 || demand > vehCap)
                throw std::runtime_error("Set S requires positive demands no greater than vehicle capacity");
            demandSum += demand;
            if (demandSum > std::numeric_limits<int>::max()) throw std::runtime_error("total demand exceeds integer range");
        }
        for (int i = 0; i < numL; ++i) {
            if (inventories[i] <= 0 || openingCosts[i] < 0 || !std::isfinite(openingCosts[i]))
                throw std::runtime_error("invalid depot capacity or opening cost");
            inventorySum += inventories[i];
        }
        if (inventorySum < demandSum) throw std::runtime_error("total depot capacity is below customer demand");
        define();
        for (int i = 0; i < numC; ++i) { x[i] = coordinatesX[i]; y[i] = coordinatesY[i]; }
        for (int i = 0; i < numV; ++i) demCus[i] = demands[i];
        for (int i = 0; i < numL; ++i) {
            invLoc[numV + i] = inventories[i]; openCostLoc[numV + i] = openingCosts[i];
        }
        readDepotConfigurations();
    } else {
        throw std::runtime_error("unsupported instance extension: " + extension);
    }
	totalDemand=0;
	maxDemand=0;
	for (int i=0; i < numV; i++){
		if (demCus[i] > maxDemand)
			maxDemand=demCus[i];
		totalDemand+=demCus[i];
	}
	normalizaPara = (hardTimeWindows && totalDemand == 0) ? 1.0 : double(numV) / double(totalDemand);
	avgDemand = double(totalDemand) / double(numV);
	//
	maxVnum=0;
	for (int i=0; !hardTimeWindows && i<numL;i++){
		maxVnum += safeRouteLimit(i);
	}
	if (hardTimeWindows) maxVnum = numV * numL;
	// process the initial data

	isConstraintLoc = false;
	for (int i=numV;i<numC;i++){
		if (invLoc[i] <= totalDemand){
			isConstraintLoc = true;
			break;
		}
	}

	maxDist=0;
	for (int i=0;i<numV;i++){
		for (int j=0;j<numV;j++){
			D[i][j]=std::sqrt((x[i]-x[j])*(x[i]-x[j])+(y[i]-y[j])*(y[i]-y[j]));
			if (D[i][j]>maxDist)
				maxDist=D[i][j];
		}
	}
	for (int i=0; i < numV;i++){
		for (int j=numV; j<numC; j++){
			D[i][j] = std::sqrt((x[i]-x[j])*(x[i]-x[j])+(y[i]-y[j])*(y[i]-y[j]));
			D[j][i] = D[i][j];
		}
	}
	for (int i=numV; i<numC; i++)
		for (int j=numV; j<numC; j++)
			D[i][j] = 0;
	for (int i = 0; i < numC; ++i) {
		for (int j = 0; j < numC; ++j) {
			if (!std::isfinite(D[i][j])) throw std::runtime_error("coordinate distance exceeds numeric range");
		}
	}
	if (realOrInteger == 0)
		for (int i=0;i<numC;i++)
			for (int j=0;j<numC;j++)
				{
                    const double scaled = std::ceil(D[i][j] * 100);
                    if (!std::isfinite(scaled) || scaled > std::numeric_limits<int>::max())
                        throw std::runtime_error("Set S scaled distance exceeds integer range");
                    D[i][j] = scaled;
                }
	if (hardTimeWindows)
		for (int i=0;i<numC;i++)
			for (int j=0;j<numC;j++)
				D[i][j] = std::floor(D[i][j] + 1e-12);
	// the most key element to decide promising depot configurations.

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
	//

	//decide whether used extra depot configurations
	if (hardTimeWindows) {
		// Capacity is only a necessary condition; construction must check TW.
		std::set<int> allDepots;
		for (int i=numV;i<numC;i++) {
			allDepots.insert(i);
			locRatio.push_back({openCostLoc[i], i});
		}
		std::sort(locRatio.begin(), locRatio.end());
		if (numL <= 5) {
			for (unsigned mask=1; mask < (1U << numL); ++mask) {
				long long capacity = 0;
				double cost = 0;
				std::set<int> depots;
				for (int depot=0; depot<numL; ++depot) if (mask & (1U << depot)) {
					capacity += invLoc[numV + depot];
					cost += openCostLoc[numV + depot];
					depots.insert(numV + depot);
				}
				if (capacity >= totalDemand) LocConfigure.push_back({cost, depots});
			}
			std::sort(LocConfigure.begin(), LocConfigure.end());
		} else {
			// Larger non-benchmark files avoid exponential subset enumeration.
			LocConfigure.push_back({0.0, allDepots});
		}
		if (LocConfigure.empty()) throw std::runtime_error("Set F total depot capacity is below customer demand");
	} else decideUsingDepotConfigurations();

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
bool read_data::stopCondition(double dis) {
    if (experimentRemaining() <= 0) {
        experimentTerminationReason = "elapsed_deadline";
        return true;
    }
    // bestDis is a caller-supplied target, not an independent optimality proof.
    if (bestDis > 0 && std::isfinite(dis) && dis <= bestDis) {
        if (meetOptimal) {
            timeToTarget = double(clock()) / CLOCKS_PER_SEC;
            meetOptimal = false;
        }
        experimentTerminationReason = "target_reached";
        return true;
    }
    if (double(clock()) / CLOCKS_PER_SEC > timeLimit) {
        experimentTerminationReason = "cpu_limit";
        return true;
    }
    if (iteration > iterLimit) {
        experimentTerminationReason = "counter_limit";
        return true;
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
	int min_depot = std::numeric_limits<int>::max();
	int max_depot = 0;
	for (int i=numV;i<numC;i++){
		if (min_depot > invLoc[i])
			min_depot = invLoc[i];
		if (max_depot < invLoc[i])
			max_depot = invLoc[i];
	}
	if (min_depot > totalDemand){
		int array[numL];
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
	double min_x = std::numeric_limits<double>::infinity(), min_y = min_x, max_x = -min_x, max_y = -min_x;
	double range_x, range_y;
	for (int i=0;i<numC;i++){
		if (x[i] < min_x) min_x = x[i];
		if (y[i] < min_y) min_y = y[i];
		if (x[i] > max_x) max_x = x[i];
		if (y[i] > max_y) max_y = y[i];
	}
	double range_loc, min_loc = std::numeric_limits<double>::infinity(), max_loc = 0;
	for (int i=numV;i<numC;i++){
		if (invLoc[i] < min_loc)min_loc = invLoc[i];
		if (invLoc[i] > max_loc)max_loc = invLoc[i];
	}
	range_loc = max_loc - min_loc;
	range_x = max_x - min_x;
	range_y = max_y - min_y;
	//the following is to decide some locations conflict
	double dis_measure = (range_x + range_y) / 2.;
	if (dis_measure == 0) return;
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

}
void read_data::spanningTreeForLoc(){
	std::vector<int> check_cus(numV);

	std::vector<double> low_cost(numV);

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
		while(1){
			int totalVeh = 0;
			for (int i=0;i< numV;i++){
				min_dis=std::numeric_limits<double>::infinity();
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
			if (v < 0 || total_dem >= invLoc[l+numV] || total_dem >= totalDemand) break;
            if (total_dem + demCus[v] >= invLoc[l+numV]) break;
			for (int j=0;j<numV;j++)
				if (check_cus[j] == 1)
					low_cost[j]=std::numeric_limits<double>::infinity();

		}
		double cost_loc = openCostLoc[l + numV] + total_dis_loc + numVehicle * usingCostVeh;
	//	locRatio.push_back({ cost_loc / double(total_dem), l+numV});
		locRatio.push_back({(double)cost_loc, l+numV});
	}
	//

}

void read_data::tabuSearchFindCompactConfiguration(){
	std::vector<std::set<int>> listLocConfigure;
	// using tabu search to determine that does this instance has perfect locations configurations
	std::vector<int> sol(numL), best_sol(numL), tabu_table(numL);
    std::vector<double> dem(numL);
	for (int i=0;i<numL;i++)tabu_table[i] = 0;
	long long cusDem = 0;
	double obj=0,best_obj;
	// construct a initial solution
	int count;
	for (int i=0;i<numL;i++){
		dem[i] = locRatio[i].first;
	}
    int configurationAttempts = 0;
	while(1){
        if (++configurationAttempts > 1000 || stopCondition(std::numeric_limits<double>::infinity()))
            throw std::runtime_error("depot configuration construction exhausted its attempt/time budget");
		for (int i=0;i<numL;i++)sol[i] = 0;
        cusDem = 0; obj = 0;
		int seed = RandomInt(0, numL - 1);
		long long cumLoc = 0;
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
	int seed;
	best_obj = obj;
	int can_node_1[100],can_node_2[100];
	int can_tabu_node_1[100],can_tabu_node_2[100];
	//
	double delta,min_delta, min_tabu_delta;
	int num_best, num_tabu_best;
	int no_improve = 0;
	double tabuLength = 0.3;
	int length_improve = 2000;
	int tabu_length = ceil(numL * tabuLength);
	int step = 1;
	int loc1, loc2;

	while (1){
		min_delta=std::numeric_limits<double>::infinity();min_tabu_delta=std::numeric_limits<double>::infinity();num_best=0;num_tabu_best=0;
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
					else if (delta == min_delta && num_best < 100) {
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
		if (!std::isfinite(min_tabu_delta) && !std::isfinite(min_delta)) {
                if (LocConfigure.empty()) {
                    std::set<int> current;
                    double cost = 0;
                    for (int i = 0; i < numL; ++i) if (sol[i]) { current.insert(i + numV); cost += dem[i]; }
                    if (cusDem >= totalDemand && !current.empty()) LocConfigure.push_back({cost, current});
                }
                break;
            }
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

		//
		std::set<int>depot;
		double cost = 0;
		for (int i=0;i<numL;i++)
			if (sol[i] == 1){
				depot.insert(i+numV);
				cost += dem[i];
			}
		////////////////////////////
		long long sumDem = 0;
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
	//
//	std::sort(LocConfigure.begin(), LocConfigure.end());
	//

}
void read_data::constructedDepotConfigurations(){
	//the line function to estimate the coverRatio
	double min_coverRatio = 0.1;
	double max_coverRatio = 0.6;
	double kParameter = max_coverRatio - min_coverRatio;
	//
	std::vector<int> solutionTabu(dimension, 0);
	for (int i=0;i<dimension;i++)
		solutionTabu[i] = 0;
	int seed;
	double coverRatio = 0.2;// assuming the covering ratio is 50%
	double costDepotConf;
	int countOver;
	int iter;
	long long totalDem;
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
        if (stopCondition(std::numeric_limits<double>::infinity())) {
            if (LocConfigure.empty()) throw std::runtime_error("depot configuration construction exhausted its time budget");
            break;
        }
		sol.clear();
		depot.clear();
		coverVertex.clear();
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
            if (depot.empty()) throw std::runtime_error("cannot construct a capacity-feasible depot configuration");
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

				double var = iter ? (double)countOver / (double)iter : 0.0;
				priorityOrder.push_back({var, depot[i]});
			}
			sort(priorityOrder.begin(), priorityOrder.end());
			orderDepots.clear();
			for (int i=0;i<(int)priorityOrder.size();i++){
				long long cap = invLoc[priorityOrder[i].second + numV] + totalDem;
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
					while ((int)orderDepots.size() == 0){
						coverRatio += 0.1;
						for (int i=0;i<(int)priorityOrder.size();i++){
						//	long long cap = invLoc[priorityOrder[i].second + numV] + totalDem;
						//	coverRatio = kParameter * ((double)cap / (double)totalDemand) + min_coverRatio;
							if (priorityOrder[i].first > coverRatio)break;// discard these non-promising depots
							else orderDepots.push_back({locRatio[priorityOrder[i].second].first,priorityOrder[i].second});
						}
					}
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
        if (totalDem < totalDemand) {
            ++no_get; ++iteration;
            if (no_get > 1000) break;
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
			std::cout<<"time consuming  "<<float((double)clock()/(double)CLOCKS_PER_SEC)<<std::endl;
			break;
		}
		iteration++;
	//	std::cout<<iteration<<"   "<<(int)LocConfigure.size()<<std::endl;
	}
}

void read_data::readDepotConfigurations() {
    if (pathDepotCon == "NULL") return;
    std::ifstream input(pathDepotCon);
    if (!input) throw std::runtime_error("cannot open depot configurations: " + pathDepotCon);
    auto readInteger = [&](const std::string& token) {
        std::size_t used = 0;
        long long value;
        try { value = std::stoll(token, &used); }
        catch (const std::exception&) { throw std::runtime_error("invalid depot configuration integer: " + token); }
        if (used != token.size() || value < 0 || value > std::numeric_limits<int>::max())
            throw std::runtime_error("invalid depot configuration integer: " + token);
        return static_cast<int>(value);
    };
    std::string token;
    if (!(input >> token)) throw std::runtime_error("missing depot configuration count");
    const int count = readInteger(token);
    if (count == 0) throw std::runtime_error("depot configuration list must not be empty");
    for (int i = 0; i < count; ++i) {
        std::vector<int> configuration;
        std::set<int> seen;
        bool ended = false;
        while (input >> token) {
            if (token == "e") { ended = true; break; }
            const int depot = readInteger(token);
            if (depot >= numL || !seen.insert(depot).second)
                throw std::runtime_error("out-of-range or duplicate depot configuration index");
            configuration.push_back(numV + depot);
        }
        if (!ended || configuration.empty()) throw std::runtime_error("incomplete or empty depot configuration");
        long long capacity = 0;
        for (int depot : configuration) capacity += invLoc[depot];
        long long demand = 0;
        for (int c = 0; c < numV; ++c) demand += demCus[c];
        if (capacity < demand) throw std::runtime_error("depot configuration capacity is below total demand");
        depotCon.push_back(std::move(configuration));
    }
    if (input >> token) throw std::runtime_error("unexpected data after depot configurations");
    if (input.bad()) throw std::runtime_error("depot configuration read failed");
}
