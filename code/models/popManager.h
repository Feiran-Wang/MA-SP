/******************************************************************************
 * File: popManager.h
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description: 
 *   Manages the population of individuals, sub-populations, and diversity logic.
 ******************************************************************************/

#ifndef POPMANAGER_H_
#define POPMANAGER_H_
#include "read_data.h"
#include "Individual.h"
#include <memory>
typedef std::vector <Individual*> SubPopulation ;
typedef std::vector <std::vector<Individual*>> Population ;
class popManager {
public:
	popManager(read_data * data);
	virtual ~popManager();
	void reduceHalfSubPop();
	void removeEmptySubPop();

	void copyAllData(Individual * s,Individual *ss);
	// Optionally exports accepted routes to the shared route pool.
	int addIndividual(Individual *indiv, int rankPop, bool isInitial, class RoutePool* pool = nullptr);
	void addInElitePop(Individual *indiv);
	void reStart();
	//
	void removeWorstBiasedFitness(SubPopulation & pop);
	void updateBiasedFitnesses(SubPopulation & pop);
	/////////////
	Individual * selectionParentSolution(Individual *exist, int rankPop);
	Individual * selectionMotherSolution(Individual *exist, int rankPop);
	double averagePop();
	double usedDepots();
	//variables;
	Individual *global;
	Individual *bestIndivRestart;
	std::vector <std::vector<Individual*>> popula;
	int reverseReplacementCount; // Number of free-population takeovers.

	// Returns the resident population with the worst mean objective.
    int getWorstResidentIndex();
    
    // Admits an SP solution to a matching resident population, or lets it
    // replace the worst resident population when it introduces a new layout.
	int injectSPSolution(
		std::unique_ptr<Individual> spIndiv,
		class RoutePool* pool = nullptr);

	// Creation iteration used to protect newly established populations.
    std::vector<long long int> popCreationIteration;


private:
	//variables
	read_data *data;
	SubPopulation feasibleSubpopulation;			// Feasible subpopulation, kept ordered by increasing penalized cost

	int numV;
	int numL;
	int numC;
	int *order;
};
#endif /* POPMANAGER_H_ */
