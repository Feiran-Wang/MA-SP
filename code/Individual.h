// Author: Feiran
// Declare customer, route, depot, and complete-solution state.

#ifndef INDIVIDUAL_H_
#define INDIVIDUAL_H_

#include <vector>
#include <set>
#include "basic.h"
#include "read_data.h"
#include "RoutePool.h"
struct Node;
struct NodeDepot;
struct Route {
    bool used;
    int cour;
    int nbClients;
    int whenLastModified = -1;
    Node *depot;
    Node *endDepot;
    NodeDepot *whichLocation;
    double dis;
    int load;
    double penalty;
};
struct Node {
    bool isDepot;
    int cour;
    int position;
    int dem;
    int whenLastTestedRI;
    Node *next;
    Node *pre;
    Route *route;
    int cumulatedLoad;
    double cumulatedDis;

    int start;
    int end;
    bool iseum;
};
struct NodeDepot {
    bool used;
    int cour;
    int inventory;
    Node *depots;
    Node *endDepots;
    int splitTime;
    int maxVisit;
    int remainInven;
    bool iscapCon;
    double penalty;
    std::set<int> emptyRoutes;
};
class Individual {
  public:
    Individual();
    virtual ~Individual();
    void define(read_data *data);
    void evaluationDis();
    void initilization();
    void updateRouteInfor(int &r, int nbMoves);
    void updateLocationInfor(int &ll);

    void outputSolution();
    void isRight();
    void decideFeasible();
    void outputBestSolution();
    void writeStudyWitness(const std::string &path) const;

    void rebuildFromSP(const std::vector<StandaloneRoute> &spRoutes);

    double brokenPairsDistance(Individual *indiv2);
    double averageBrokenPairsDistanceClosest(int nbClosest);
    void removeProximity(Individual *indiv);
    Node *client;
    NodeDepot *location;
    Route *route;

    int inventory;
    int inventoryVeh;

    double dis;
    int capExcess;
    double penalty;
    double fit;
    double biasedFit;
    bool isFeasible;
    std::multiset<std::pair<double, Individual *>> indivsPerProximity;
    int maxVnum;

  private:
    static void resetClientBookkeeping(Node &node);
    static void resetRouteContents(Route &route, int index);
    read_data *data;
    int numV;
    int numL;
    int numC;

    int *checkArray;
};
#endif
