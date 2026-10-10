/*
 * commandline.h
 *
 *  Created on: 22 Jun 2021
 *      Author: Peng
 */

#ifndef COMMANDLINE_H_
#define COMMANDLINE_H_
#include "basic.h"
#include "SearchParameters.h"
#include <stdexcept>
class CommandLine{
public:
	std::string pathInstance;
	std::string pathDepotCon;
	std::string pathSolution;
	int timeLimit=Max;
	int iterLimit;
	int seed=0;
	double bestDis;
	int isOptimal;
	CommandLine(int argc, char* argv[]){
		if (argc!=8 && argc != 9){
			throw std::invalid_argument("the input parameters in configuration are wrong: expected 7 or 8 arguments");
		}
		else if (argc == 8){
			pathInstance=std::string(argv[1]);
			pathDepotCon = "NULL";
			pathSolution=std::string(argv[2]);
			if (sscanf(argv[3],"%lf",&bestDis) != 1) throw std::invalid_argument("invalid bestDis parameter");
			if (sscanf(argv[4],"%d",&isOptimal) != 1) throw std::invalid_argument("invalid isOptimal parameter");
			if (sscanf(argv[5],"%d",&timeLimit) != 1) throw std::invalid_argument("invalid timeLimit parameter");
			if (sscanf(argv[6],"%d",&iterLimit) != 1) throw std::invalid_argument("invalid iterLimit parameter");
			if (sscanf(argv[7],"%d",&seed) != 1) throw std::invalid_argument("invalid seed parameter");
			display_infor();
		}
		else if (argc == 9){
			pathInstance=std::string(argv[1]);
			pathDepotCon=std::string(argv[2]);
			pathSolution=std::string(argv[3]);
			if (sscanf(argv[4],"%lf",&bestDis) != 1) throw std::invalid_argument("invalid bestDis parameter");
			if (sscanf(argv[5],"%d",&isOptimal) != 1) throw std::invalid_argument("invalid isOptimal parameter");
			if (sscanf(argv[6],"%d",&timeLimit) != 1) throw std::invalid_argument("invalid timeLimit parameter");
			if (sscanf(argv[7],"%d",&iterLimit) != 1) throw std::invalid_argument("invalid iterLimit parameter");
			if (sscanf(argv[8],"%d",&seed) != 1) throw std::invalid_argument("invalid seed parameter");
			display_infor();
		}
		if (SearchParameters::experimentEnabled()) {
			const int offset = argc == 8 ? 3 : 4;
			bestDis = SearchParameters::parseExperimentDecimal(argv[offset], "bestDis");
			isOptimal = SearchParameters::parseExperimentInteger(argv[offset + 1], "isOptimal");
			timeLimit = SearchParameters::parseExperimentInteger(argv[offset + 2], "timeLimit");
			iterLimit = SearchParameters::parseExperimentInteger(argv[offset + 3], "iterLimit");
			seed = SearchParameters::parseExperimentInteger(argv[offset + 4], "seed");
			if (isOptimal != 0 && isOptimal != 1)
				throw std::invalid_argument("Set F study isOptimal must be 0 or 1");
			if (timeLimit != 60000 || iterLimit != 300000)
				throw std::invalid_argument("Set F study requires CPU limit 60000 and main-counter limit 300000");
			if (bestDis < 0 || (isOptimal && (bestDis <= 0 || std::floor(bestDis) != bestDis)))
				throw std::invalid_argument("Set F study requires a nonnegative reference and an exact positive integer Opt value");
		}
	}
	void display_infor(){
		std::cout<<"Starting The Algorithm"<<std::endl;
		std::cout<<"The Instance is "<<pathInstance<<std::endl;
	}
};
#endif /* COMMANDLINE_H_ */
