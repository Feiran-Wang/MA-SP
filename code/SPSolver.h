// SPSolver.h
#ifndef SPSOLVER_H_
#define SPSOLVER_H_

#include <vector>
#include "gurobi_c++.h"
#include "RoutePool.h"
#include "read_data.h"
class StudyEvents;

class SPSolver {
public:
    SPSolver(read_data* data);
    ~SPSolver();

    // Build a temporary pool snapshot containing the complete incumbent MIP start.
    // Solver/validation failures propagate except the explicit Set S timeout endpoint.
    std::vector<StandaloneRoute> solve(const RoutePool* pool, const Individual* incumbent);
    StudyEvents* study = nullptr;
    bool lastTimeoutNoSolution = false;
    long long optimizerStarts = 0;
    double modelBuildCPU = 0;
    double optimizationCPU = 0;

private:
    read_data* data;
    GRBEnv* env; // Gurobi 全局环境指针
};

#endif /* SPSOLVER_H_ */
