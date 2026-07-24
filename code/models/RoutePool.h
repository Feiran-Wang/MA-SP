/******************************************************************************
 * File: RoutePool.h
 * Author: Wang
 * Date: 2026-06-24
 * Description: 
 *   Central pool for managing and storing elite route fragments.
 ******************************************************************************/

#ifndef ROUTEPOOL_H_
#define ROUTEPOOL_H_

#include <vector>
#include <unordered_set>
#include "read_data.h"

class Individual;

// 纯数据形态的路径结构，解耦了底层的双向链表指针
struct StandaloneRoute {
    int locId;              // 所属场站的索引 (0 到 data->numL - 1)
    double routeCost;       // 该路径的纯行驶距离成本
    int totalLoad;          // 路径上所有客户需求的总负载量
    std::vector<int> sequence; // 依次访问的客户节点集合 (0 到 data->numC - 1)

    // 重载等于运算符，用于 unordered_set 的去重
    bool operator==(const StandaloneRoute& other) const {
        if (locId != other.locId) return false;
        if (sequence.size() != other.sequence.size()) return false;
        for (size_t i = 0; i < sequence.size(); ++i) {
            if (sequence[i] != other.sequence[i]) return false;
        }
        return true;
    }
};

// 自定义哈希函数，用于 StandaloneRoute 在 unordered_set 中的去重
struct StandaloneRouteHash {
    std::size_t operator()(const StandaloneRoute& r) const {
        std::size_t seed = r.sequence.size();
        seed ^= std::hash<int>()(r.locId) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        for(int node : r.sequence) {
            seed ^= std::hash<int>()(node) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

class RoutePool {
public:
    RoutePool(read_data* data);
    ~RoutePool();

    // 核心接口：从一个个体中提取所有有效路径并存入池中
    void extractRoutes(Individual* indiv);

    // 获取当前池中所有的不重复路径
    const std::vector<StandaloneRoute>& getPool() const { return poolList; }

    // 打印池的状态信息
    void printStatus() const;

    // 更新统计信息和获取平均值的接口
    void recordSolutionFitness(double fitness) {
        totalFitnessSum += fitness;
        solutionCount++;
    }

    double getAverageFitness() const {
        if (solutionCount == 0) return 1e30; // 初始返回极大值
        return totalFitnessSum / (double)solutionCount;
    }

private:
    read_data* data;
    
    //动态统计变量
    double totalFitnessSum = 0.0;
    int solutionCount = 0;

    // 使用 unordered_set 实现 O(1) 的去重插入
    std::unordered_set<StandaloneRoute, StandaloneRouteHash> uniqueRoutes;
    
    // 为了方便后续 Gurobi 线性遍历，维护一个 vector
    std::vector<StandaloneRoute> poolList;

    // 容量上限，防止 Gurobi 模型爆炸
    std::size_t maxPoolSize;

    // 池容量管理：淘汰劣质路径
    void prunePool();
};

#endif /* ROUTEPOOL_H_ */