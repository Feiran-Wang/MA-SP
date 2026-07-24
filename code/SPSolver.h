// SPSolver.h
#ifndef SPSOLVER_H_
#define SPSOLVER_H_

#include <vector>
#include "gurobi_c++.h"
#include "RoutePool.h"
#include "read_data.h"

class SPSolver {
public:
    SPSolver(read_data* data);
    ~SPSolver();

    // 核心接口：接收路径池，返回最优路径组合
    // 如果求解失败或无可行解，返回空的 vector
    std::vector<StandaloneRoute> solve(const RoutePool* pool);

private:
    read_data* data;
    GRBEnv* env; // Gurobi 全局环境指针
};

#endif /* SPSOLVER_H_ */