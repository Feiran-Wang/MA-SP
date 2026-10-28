// Author: Feiran
// Declare the set-partitioning interface and solve statistics.

#ifndef SPSOLVER_H_
#define SPSOLVER_H_

#include <vector>
#include "gurobi_c++.h"
#include "RoutePool.h"
#include "read_data.h"
class StudyEvents;

class SPSolver {
  public:
    SPSolver(read_data *data);
    ~SPSolver();

    std::vector<StandaloneRoute> solve(const RoutePool *pool, const Individual *incumbent);
    StudyEvents *study = nullptr;
    bool lastTimeoutNoSolution = false;
    bool lastBudgetExhausted = false;
    long long optimizerStarts = 0;
    double modelBuildCPU = 0;
    double optimizationCPU = 0;

  private:
    read_data *data;
    GRBEnv *env;
};

#endif
