/******************************************************************************
 * File: SPSolver.cpp
 * Author: Feiran Wang
 * Date: 2026-06-25
 * Description: 
 *   Set Partitioning (SP) solver using Gurobi for exact optimization and bounds.
 ******************************************************************************/

// SPSolver.cpp
#include "SPSolver.h"
#include <iostream>
#include <string>

SPSolver::SPSolver(read_data* data) {
    this->data = data;
    try {
        // 初始化 Gurobi 环境，由 read_data 统管参数
        this->env = new GRBEnv(true);
        this->env->set("LogFile", data->spLogFile);
        this->env->set(GRB_IntParam_OutputFlag, data->spOutputFlag); 
        this->env->start();
    } catch(GRBException& e) {
        std::cerr << "[SPSolver] Error initializing Gurobi env: " << e.getMessage() << std::endl;
        this->env = nullptr;
    }
}

SPSolver::~SPSolver() {
    if (this->env != nullptr) {
        delete this->env;
    }
}

std::vector<StandaloneRoute> SPSolver::solve(const RoutePool* pool) {
    std::vector<StandaloneRoute> optimalCombination;
    if (this->env == nullptr || pool->getPool().empty()) {
        return optimalCombination; 
    }

    try {
        GRBModel model = GRBModel(*this->env);
        
        const std::vector<StandaloneRoute>& routes = pool->getPool();
        int numRoutes = routes.size();
        int numCustomers = data->numV; 
        int numDepots = data->numL;

        // 定义决策变量
        std::vector<GRBVar> x(numRoutes);
        for (int r = 0; r < numRoutes; ++r) {
            x[r] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "x_" + std::to_string(r));
        }
        std::vector<GRBVar> y(numDepots);
        for (int l = 0; l < numDepots; ++l) {
            y[l] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "y_" + std::to_string(l));
        }
        model.update();

        GRBLinExpr objective = 0.0;
        
        // 1. 场站开放成本
        // 真实变量对齐: data->openCostLoc
        for (int l = 0; l < numDepots; ++l) {
            objective += data->openCostLoc[l + numCustomers] * y[l]; 
        }

        // 2. 路径成本 = 行驶距离 + 车辆固定成本
        // 真实变量对齐: data->usingCostVeh
        for (int r = 0; r < numRoutes; ++r) {
            double totalRouteCost = routes[r].routeCost + data->usingCostVeh;
            objective += totalRouteCost * x[r];
        }
        
        model.setObjective(objective, GRB_MINIMIZE);

        std::vector<GRBLinExpr> customerCoverage(numCustomers, 0.0);
        std::vector<GRBLinExpr> depotLoad(numDepots, 0.0);
        std::vector<GRBLinExpr> depotRouteCount(numDepots, 0.0); // 新增：记录路线数

        for (int r = 0; r < numRoutes; ++r) {
            depotLoad[routes[r].locId] += routes[r].totalLoad * x[r];
            depotRouteCount[routes[r].locId] += x[r]; // 累加分配给场站的路线数
            for (int custId : routes[r].sequence) {
                if (custId >= 0 && custId < numCustomers) {
                    customerCoverage[custId] += x[r];
                }
            }
        }
        
        // 3. 集合划分约束：客户被严格覆盖一次
        for (int i = 0; i < numCustomers; ++i) {
            model.addConstr(customerCoverage[i] == 1.0, "Cov_" + std::to_string(i));
        }

        // 4. 场站容量约束
        // 真实变量对齐: data->invLoc[l + numV] (结合客户数偏移获取场站容量)
        for (int l = 0; l < numDepots; ++l) {
            int depotCapacity = data->invLoc[l + numCustomers];
            model.addConstr(depotLoad[l] <= depotCapacity * y[l], "Cap_" + std::to_string(l));
            // 重新计算该场站的底层最大槽位 maxVisit，并增加车辆上限约束
            int numOfVeh = std::ceil(1.2 * data->totalDemand / data->vehCap) + 5;
            int maxVisit = std::min((double)numOfVeh, 3.0 * std::ceil((double)depotCapacity / (double)data->vehCap));
            
            model.addConstr(depotRouteCount[l] <= maxVisit * y[l], "MaxVeh_" + std::to_string(l));
        }

        // ==========================================
        // 【新增】：规模自适应时间分配机制 (Scale-Adaptive Time Allocation)
        // 根据客户规模 (numV) 动态映射 Gurobi 的最大求解时间
        double dynamicTimeLimit = 360.0; // 默认基准时间
        
        if (data->numV <= 100) {
            dynamicTimeLimit = 10.0;     // 100 客户规模：极小规模，迅速返回
        } else if (data->numV <= 200) {
            dynamicTimeLimit = 60.0;     // 200 客户规模
        } else if (data->numV <= 300) {
            dynamicTimeLimit = 150.0;    // 300 客户规模
        } else if (data->numV <= 400) {
            dynamicTimeLimit = 210.0;    // 400 客户规模
        } else if (data->numV <= 500) {
            dynamicTimeLimit = 280.0;    // 500 客户规模
        } else {
            dynamicTimeLimit = 360.0;    // 600 及以上客户规模：跑满 360 秒
        }
        
        // 打印当前规模匹配的分配时间，方便后台监控
        std::cout << "[SPSolver] 规模自适应触发: 客户数 " << data->numV 
                  << " -> 分配 Gurobi 求解上限时间: " << dynamicTimeLimit << "s" << std::endl;
        // ==========================================
        // 求解器参数设置
        model.set(GRB_DoubleParam_TimeLimit, dynamicTimeLimit);
        model.set(GRB_DoubleParam_MIPGap, data->spMIPGap);
        model.set(GRB_IntParam_Threads, data->spThreads); // 严格限制 Gurobi 实例占用的物理线程数
        // 【新增】应用解耦后的高级参数
        model.set(GRB_IntParam_MIPFocus, data->spMIPFocus);
        model.set(GRB_DoubleParam_Heuristics, data->spHeuristics);
        
        model.optimize();

        // 提取结果
        if (model.get(GRB_IntAttr_SolCount) > 0) {
            for (int r = 0; r < numRoutes; ++r) {
                if (x[r].get(GRB_DoubleAttr_X) > 0.5) {
                    optimalCombination.push_back(routes[r]);
                }
            }
        } else {
            std::cout << "[SPSolver] Optimization finished, but no feasible solution found." << std::endl;
        }
    } catch(GRBException& e) {
        std::cerr << "[SPSolver] Gurobi Optimization Exception: Error code = " 
                  << e.getErrorCode() << " - " << e.getMessage() << std::endl;
    } catch(...) {
        std::cerr << "[SPSolver] Unknown exception during optimization." << std::endl;
    }

    return optimalCombination;
}