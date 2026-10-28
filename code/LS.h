// Author: Feiran
// Declare local-search operators and their working state.

#ifndef LS_H_
#define LS_H_
#include "Individual.h"
#include "read_data.h"
class LS {
  public:
    explicit LS(read_data *data);
    virtual ~LS();
    void local_search_run(Individual *s, double penaltyDepot, double penaltyVeh);

  private:
    void srun();
    void loadSolution();

    double compute(bool istrue, double value);

    bool Relocate_one();
    bool Relocate_two();
    bool Relocate_three();
    bool Relocate_four();
    bool Relocate_five();
    bool Relocate_six();
    bool Relocate_seven();
    bool Relocate_eight();
    bool Relocate_nine();
    bool Relocate_ten();

    void setLocalVariableRouteU();
    void setLocalVariableRouteV();
    void commitRoutePairMove();
    void insertNode(Node *node1, Node *node2);
    void swapNode(Node *node1, Node *node2);

    bool Splitb();
    bool Splitf();

    read_data *data;
    Individual *s;
    int numV;
    int numC;
    std::vector<int> orderNodes;
    Node *nodeU;
    Node *nodeX;
    Node *nodeV;
    Node *nodeY;
    Route *routeU;
    Route *routeV;
    int nodeUPrevIndex, nodeUIndex, nodeXIndex, nodeXNextIndex;
    int nodeVPrevIndex, nodeVIndex, nodeYIndex, nodeYNextIndex;
    int loopID;
    int nbMoves;
    double moveGain;
    bool searchComplete;
};
#endif
