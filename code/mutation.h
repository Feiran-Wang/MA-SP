// Author: Feiran
// Declare mutation state and customer-insertion operations.

#ifndef MUTATION_H_
#define MUTATION_H_
#include "Individual.h"
class mutation {
  public:
    mutation(read_data *data);
    virtual ~mutation();
    void mutationRun(Individual *s);

  private:
    Individual *s;
    void shawRemoval();
    static double compute(bool istrue, double value);
    double newRouteInsertionCost(const Node *position, const Node *customer) const;
    void insertNode(Node *U, Node *V);

    read_data *data;
    int numV;
    int numL;
    int *auxArray1;
    int perturbLength;
    std::vector<int> listRemove;
    int *cityRank;
};

#endif
