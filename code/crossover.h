// Author: Feiran
// Declare crossover graph state, cycle assembly, and offspring construction.

#ifndef CROSSOVER_H_
#define CROSSOVER_H_
#include "Individual.h"
#include "read_data.h"
#include "popManager.h"
struct Node0 {
    int cour;
    bool isFather;
};
struct Node1 {
    bool isDepot;
    Node0 *edgeF;
    int nbeF;
    Node0 *edgeM;
    int nbeM;
};
struct Node2 {
    Node0 *sque;
    int nbCity;
};
struct Node5 {
    std::set<int> cycles;
};
class crossover {
  public:
    crossover(read_data *data, popManager *popman);
    virtual ~crossover();
    bool gEAX(Individual *fa, Individual *ma, Individual *subPop, int &numSubPop);
    popManager *popman;

  private:
    void collectParentEdges(int depot, const Individual *parent, const Individual *other,
                            bool father);
    void appendParentEdge(int vertex, int neighbor, bool father);
    void addCurrentESetCycles(const std::set<int> &cycles);

    void loadFaCapMa();

    void extractABcycles();
    void addNodeToABcycles(int &nb, Node0 &tem);
    void emptyACity(int &city, int &numAvaCity);

    void generationEsets();

    void generationIntermediateSolutions();
    void flipSubString(Node *nodeU, Node *nodeV, bool reverse);
    void updateTheLocation();

    void eliminateSubtours();

    void checkTheSolution();

    Individual *off;
    Individual *subPop;
    read_data *data;
    int numV;
    int numL;
    int numC;
    Individual *fa;
    Individual *ma;
    int sizeSubPop;

    Node1 *AcapB;

    Node2 *ABcycles;
    int nbABcycles;
    int *auxArray1;
    int *auxArray2;

    std::vector<std::set<int>> whichCycles;
    std::vector<std::set<int>> commconCycles;

    Node5 *eSets;
    int nbeSets;

    Node *gStart;
    Node *gEnd;
    bool *auxA1;
    bool *auxA2;
    std::vector<Node *> headSubstring;
};

#endif
