/******************************************************************************
 * File: AlgorithmOrchestrator.h
 * Author: Wang
 * Date: 2026-06-24
 * Description: 
 *   High-level controller managing the algorithm lifecycle and orchestrating modules.
 ******************************************************************************/

#ifndef ALGORITHM_ORCHESTRATOR_H_
#define ALGORITHM_ORCHESTRATOR_H_

#include "RouteGenerator.h"
#include "RoutePool.h"
#include "SPSolver.h"
#include "read_data.h"
#include <memory>

class AlgorithmOrchestrator {
public:
    AlgorithmOrchestrator(read_data * data);
    virtual ~AlgorithmOrchestrator();

    void run();

private:
    read_data * data;
    std::unique_ptr<RouteGenerator> routeGenerator;
    std::unique_ptr<RoutePool> routePool;
    std::unique_ptr<SPSolver> spSolver;
};

#endif /* ALGORITHM_ORCHESTRATOR_H_ */
