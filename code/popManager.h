/*
 * popManager.h
 */

#ifndef POPMANAGER_H_
#define POPMANAGER_H_
#include "read_data.h"
#include "Individual.h"
#include <functional>
#include <limits>
class StudyEvents;
typedef std::vector <Individual*> SubPopulation ;
typedef std::vector <std::vector<Individual*>> Population ;
class popManager {
public:
	popManager(read_data * data);
	virtual ~popManager();
	void reduceHalfSubPop();

	//
	//
	void copyAllData(Individual * s,Individual *ss);
    // 【修改】增加 RoutePool 默认参数
	int addIndividual(Individual *indiv, int rankPop, bool isInitial, class RoutePool* pool = nullptr);
	void reStart();
	//
	void removeWorstBiasedFitness(SubPopulation & pop);
	void updateBiasedFitnesses(SubPopulation & pop);
	/////////////
	Individual * selectionParentSolution(Individual *exist, int rankPop);
	Individual *global;
	Individual *bestIndivRestart;
	std::vector <std::vector<Individual*>> popula;
	StudyEvents* study = nullptr;
	std::function<void(Individual*)> studyIncumbent;
	bool lastAddAccepted = false;
	long long spMatchingInjections = 0;
	long long spDirectInstalls = 0;
	struct StudyPopulation {
		std::string id;
		int slot, installedAt, exitedAt = -1;
		long long parentSelections = 0, incumbentImprovements = 0;
	};
	std::vector<StudyPopulation> studyPopulations;
	std::vector<int> studyPopulationAtSlot = std::vector<int>(100, -1);
	std::string studyPopulationId(int slot);
	void replaceStudyPopulation(int slot, const std::string& origin);
	void closeStudyPopulation(int slot);
	void finishStudyPopulations();
	std::string lastSPDecision;
	std::string lastSPTargetId;
	int lastSPTarget = -1;
	double lastSPTargetMean = std::numeric_limits<double>::quiet_NaN();
	double lastSPTargetBest = std::numeric_limits<double>::quiet_NaN();
	int reverseReplacementCount; // 新增：游离种群替换计数器

	// 评估所有常驻种群，返回当前平均适应度最差的子种群索引

    // 接收从 Gurobi 重组的个体，执行情景A（开发强化）或情景B（范式转移）
	int injectSPSolution(Individual* spIndiv, class RoutePool* pool = nullptr);

	// 记录各常驻种群配置的确立代数，用于实施生存期禁忌策略
    std::vector<long long int> popCreationIteration;

private:
	//variables
	read_data *data;

	int numV;
	int numL;
	int numC;
	int *order;
};
#endif /* POPMANAGER_H_ */
