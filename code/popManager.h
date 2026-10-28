// Author: Feiran
// Declare populations, proximity bookkeeping, and admission services.

#ifndef POPMANAGER_H_
#define POPMANAGER_H_
#include "read_data.h"
#include "Individual.h"
#include <functional>
#include <limits>
class StudyEvents;
typedef std::vector<Individual *> SubPopulation;
typedef std::vector<std::vector<Individual *>> Population;
class popManager {
  public:
    popManager(read_data *data);
    virtual ~popManager();
    void reduceHalfSubPop();

    void copyAllData(Individual *s, Individual *ss);

    int addIndividual(Individual *indiv, int rankPop, bool isInitial,
                      class RoutePool *pool = nullptr);
    void reStart();

    void removeWorstBiasedFitness(SubPopulation &pop);
    void updateBiasedFitnesses(SubPopulation &pop);

    Individual *selectionParentSolution(Individual *exist, int rankPop);
    Individual *global;
    Individual *bestIndivRestart;
    std::vector<std::vector<Individual *>> popula;
    StudyEvents *study = nullptr;
    std::function<void(Individual *)> studyIncumbent;
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
    void replaceStudyPopulation(int slot, const std::string &origin);
    void closeStudyPopulation(int slot);
    void finishStudyPopulations();
    std::string lastSPDecision;
    std::string lastSPTargetId;
    int lastSPTarget = -1;
    double lastSPTargetMean = std::numeric_limits<double>::quiet_NaN();
    double lastSPTargetBest = std::numeric_limits<double>::quiet_NaN();
    int reverseReplacementCount;

    int injectSPSolution(Individual *spIndiv, class RoutePool *pool = nullptr);

    std::vector<long long int> popCreationIteration;

  private:
    static void deleteMembers(SubPopulation &population);
    static void linkProximities(Individual *individual, const SubPopulation &population);

    read_data *data;

    int numV;
    int numL;
    int numC;
    int *order;
};
#endif
