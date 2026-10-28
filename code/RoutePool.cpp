// RoutePool.cpp
#include "RoutePool.h"
#include "Individual.h"
#include <iostream>
#include <algorithm>

RoutePool::RoutePool(read_data* data) {
    this->data = data;
    this->maxPoolSize = data->spMaxPoolSize; // 从全局配置读取上限
}

RoutePool::~RoutePool() {
    uniqueRoutes.clear();
    poolList.clear();
}

void RoutePool::extractRoutes(Individual* indiv) {
    // 保护性检查：仅提取满足车辆容量约束的路径
    // 如果想要容忍轻微违规的路径，可以放宽条件，但通常传入的 indiv 应是局部搜索后的可行解或优质解

    for (int i = 0; i < data->numL; i++) {
        // 检查该场站是否被使用
        if (indiv->location[i].used) {
            // 遍历该场站下所有划分的槽位 (splitTime)
            for (int j = 0; j < indiv->location[i].maxVisit; j++) {
                // 如果该槽位的 route 被使用且负载 > 0
                if (indiv->location[i].depots[j].route->used &&
                    (data->hardTimeWindows ? !indiv->location[i].depots[j].next->isDepot
                                           : indiv->location[i].depots[j].route->load > 0)) {

                    StandaloneRoute sr;
                    sr.locId = i; // 记录场站索引
                    sr.totalLoad = indiv->location[i].depots[j].route->load;

                    // 计算路径纯行驶距离 (根据论文，可能不包含固定成本)
                    // 复用原框架逻辑：起点的 next，直到 isDepot == true
                    sr.routeCost = 0.0;
                    Node* current = indiv->location[i].depots[j].next;
                    Node* previous = &indiv->location[i].depots[j]; // 虚拟的起点 depot

                    while (!current->isDepot) {
                        sr.sequence.push_back(current->cour); // cour 存储了节点的真实 ID (0 到 numC-1)
                        // 累加距离 (需注意 D 矩阵的索引逻辑：0 到 numV-1 是客户，后续是场站)
                        sr.routeCost += data->D[previous->cour][current->cour];

                        previous = current;
                        current = current->next;
                    }
                    // 加上最后一段回到尾 depot 的距离
                    sr.routeCost += data->D[previous->cour][current->cour];

                    // 将抽象路径加入池中，依赖 unordered_set 自动去重
                    if (data->hardTimeWindows &&
                        (sr.totalLoad > data->vehCap || !data->routeTimeFeasible(sr.locId, sr.sequence)))
                        continue;
                    auto result = uniqueRoutes.insert(sr);
                    if (result.second) {
                        // 如果是新插入的路径，同步更新 vector
                        poolList.push_back(sr);
                    }
                }
            }
        }
    }

    // 容量控制
    if (poolList.size() > maxPoolSize) {
        prunePool();
    }
}

void RoutePool::prunePool() {
    std::cout << "[RoutePool] Pruning pool. Current size: " << poolList.size() << std::endl;

    auto compare = [](const StandaloneRoute& a, const StandaloneRoute& b) {
        // 保护极低成本的优质路径，赋予理论极大值 1e9
        double efficiencyA = (a.routeCost > 1e-6) ? (double)a.totalLoad / a.routeCost : 1e9;
        double efficiencyB = (b.routeCost > 1e-6) ? (double)b.totalLoad / b.routeCost : 1e9;
        return efficiencyA > efficiencyB;
    };

    // 按照效率对 vector 进行排序
    std::sort(poolList.begin(), poolList.end(), compare);

    // 截断到上限的 80% (例如：留下 8 万条)，以便留下空间给未来的新列
    int targetSize = maxPoolSize * 0.8;
    poolList.resize(targetSize);

    // 重构 uniqueRoutes (必须清空后重新插入)
    uniqueRoutes.clear();
    for (const auto& sr : poolList) {
        uniqueRoutes.insert(sr);
    }

    std::cout << "[RoutePool] Pruning complete. New size: " << poolList.size() << std::endl;
}
