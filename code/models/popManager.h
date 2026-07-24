/******************************************************************************
 * File: popManager.h
 * Author: Wang
 * Date: 2026-06-24
 * Description: 
 *   Manages the population of individuals, sub-populations, and diversity logic.
 ******************************************************************************/

#ifndef POPMANAGER_H_
#define POPMANAGER_H_
#include "read_data.h"
#include "Individual.h"
typedef std::vector <Individual*> SubPopulation ;
typedef std::vector <std::vector<Individual*>> Population ;
class popManager {
public:
	popManager(read_data * data);
	virtual ~popManager();
	void reduceHalfSubPop();
	void removeEmptySubPop();

	//
	//
	void copyAllData(Individual * s,Individual *ss);
    // 【修改】增加 RoutePool 默认参数
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
	int reverseReplacementCount; // 新增：游离种群替换计数器

	// 评估所有常驻种群，返回当前平均适应度最差的子种群索引
    int getWorstResidentIndex();
    
    // 接收从 Gurobi 重组的个体，执行情景A（开发强化）或情景B（范式转移）
	int injectSPSolution(Individual* spIndiv, class RoutePool* pool = nullptr);

	// 记录各常驻种群配置的确立代数，用于实施生存期禁忌策略
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
