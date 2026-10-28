// Author: Feiran
// Declare initial construction and customer-insertion operations.

#ifndef INITIALSOL_H_
#define INITIALSOL_H_
#include "read_data.h"
#include "Individual.h"
#include "RepairProcedure.h"
class initial_sol {
  public:
    initial_sol(read_data *data, RepairProcedure *repair);
    virtual ~initial_sol();
    void initial_solution(Individual *s, int rankPop);

  private:
    Individual *s;
    read_data *data;
    RepairProcedure *repair;
    int numV;
    int numL;
    int numC;
    int rankPop;

    void insertClient(Node *U, Node *V);
    Node *bestInsertionPosition(const Node *customer, int depot);
    int selectedDepotCapacity() const;

    void newGreedyInsert();
    void insertCusToLoc(int loc);
    void remedyTheSolution(const std::vector<int> &location);

    int remainDem = 0;
    bool *locUsed;
    bool *cusUsed;
};
#endif
