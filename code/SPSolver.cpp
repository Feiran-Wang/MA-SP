// SPSolver.cpp
#include "SPSolver.h"
#include "StudyEvents.h"
#include "Individual.h"
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace {
using WallClock = std::chrono::steady_clock;

void requireStart(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

bool sameCost(double a, double b) {
    return std::isfinite(a) && std::isfinite(b)
        && std::abs(a - b) <= 1e-6 + 1e-10 * std::max(std::abs(a), std::abs(b));
}

int routeLimit(const read_data* data, int depot) {
    if (data->hardTimeWindows) return data->numV;
    return data->safeRouteLimit(depot);
}

// Validate against the same metric and constraints as the restricted master.
double validateRoutes(const std::vector<StandaloneRoute>& routes,
                      const std::vector<int>& opened, const read_data* data) {
    std::vector<int> visits(data->numV, 0), loads(data->numL, 0), counts(data->numL, 0);
    double objective = 0.0;
    for (int l = 0; l < data->numL; ++l) {
        if (opened[l]) objective += data->openCostLoc[l + data->numV];
    }
    for (const auto& route : routes) {
        const int l = route.locId;
        requireStart(l >= 0 && l < data->numL && opened[l], "route uses an invalid or closed depot");
        requireStart(!route.sequence.empty(), "empty route in complete solution");
        int load = 0, previous = l + data->numV;
        double travel = 0.0;
        for (int customer : route.sequence) {
            requireStart(customer >= 0 && customer < data->numV, "invalid customer index");
            requireStart(++visits[customer] == 1, "duplicate customer in complete solution");
            load += data->demCus[customer];
            travel += data->D[previous][customer];
            previous = customer;
        }
        travel += data->D[previous][l + data->numV];
        requireStart(!data->hardTimeWindows || data->routeTimeFeasible(l, route.sequence), "hard time-window violation");
        requireStart(load == route.totalLoad && load <= data->vehCap, "route capacity/load mismatch");
        requireStart(sameCost(travel, route.routeCost), "route travel cost mismatch");
        loads[l] += load;
        ++counts[l];
        objective += travel + data->usingCostVeh;
    }
    for (int count : visits) requireStart(count == 1, "incomplete customer coverage");
    for (int l = 0; l < data->numL; ++l) {
        requireStart(loads[l] <= data->invLoc[l + data->numV], "depot capacity violation");
        requireStart(counts[l] <= routeLimit(data, l), "depot route-slot violation");
    }
    requireStart(std::isfinite(objective), "non-finite objective");
    return objective;
}

std::vector<StandaloneRoute> extractIncumbent(const Individual* incumbent,
                                             const read_data* data,
                                             std::vector<int>& opened) {
    requireStart(incumbent && incumbent->isFeasible, "missing feasible global incumbent");
    std::vector<StandaloneRoute> routes;
    opened.assign(data->numL, 0);
    for (int l = 0; l < data->numL; ++l) {
        const NodeDepot& depot = incumbent->location[l];
        if (!depot.used) continue;
        opened[l] = 1;
        int routeCount = 0;
        for (int slot = 0; slot < depot.maxVisit; ++slot) {
            const Node* first = &depot.depots[slot];
            const Node* last = &depot.endDepots[slot];
            requireStart(first->cour == l + data->numV && last->cour == l + data->numV,
                         "incumbent depot index mismatch");
            requireStart(first->route != nullptr, "missing incumbent route record");
            if (!first->route->used) continue;
            StandaloneRoute route;
            route.locId = l;
            route.totalLoad = 0;
            route.routeCost = 0.0;
            const Node* previous = first;
            const Node* node = first->next;
            while (node != last) {
                requireStart(node && !node->isDepot, "invalid incumbent route terminator");
                requireStart(node->cour >= 0 && node->cour < data->numV, "invalid incumbent customer");
                requireStart(route.sequence.size() < static_cast<size_t>(data->numV), "cyclic incumbent route");
                requireStart(node->pre == previous, "broken incumbent route link");
                route.sequence.push_back(node->cour);
                route.totalLoad += data->demCus[node->cour];
                route.routeCost += data->D[previous->cour][node->cour];
                previous = node;
                node = node->next;
            }
            requireStart(last->pre == previous, "broken incumbent depot return");
            route.routeCost += data->D[previous->cour][l + data->numV];
            requireStart(route.totalLoad == first->route->load, "incumbent route load mismatch");
            routes.push_back(route); // Nonempty zero-demand routes are retained, too.
            ++routeCount;
        }
        requireStart(routeCount == depot.splitTime, "incumbent route count mismatch");
    }
    requireStart(sameCost(validateRoutes(routes, opened, data), incumbent->fit),
                 "incumbent objective does not match complete routes");
    return routes;
}

// Legacy-format witnesses can be reconstructed by the existing independent reader.
// The time field is the process CPU timestamp at capture, not SP solve duration.
void writeWitness(const std::string& kind, int iteration,
                  const std::vector<StandaloneRoute>& routes,
                  const std::vector<int>& opened, double objective) {
    const std::string path = "sp_" + kind + "_" + std::to_string(iteration) + ".txt";
    std::ofstream out(path);
    requireStart(out.good(), "cannot create SP witness: " + path);
    out << "# SP " << kind << " witness; time below is capture process CPU, not solve duration\n";
    if (kind == "candidate") out << "# UNVALIDATED candidate; not a successful SP result\n";
    for (size_t l = 0; l < opened.size(); ++l) {
        if (!opened[l]) continue;
        int load = 0;
        for (const auto& route : routes) if (route.locId == static_cast<int>(l)) load += route.totalLoad;
        out << "Location: " << l << "(load: " << load << ")\n";
        int number = 0;
        for (const auto& route : routes) {
            if (route.locId != static_cast<int>(l)) continue;
            out << "Route " << number++ << "(load: " << route.totalLoad << ") ";
            for (int customer : route.sequence) out << customer << ' ';
            out << '\n';
        }
    }
    out << std::setprecision(17) << "The actual cost is: " << objective << '\n'
        << "The running time is: " << static_cast<double>(clock()) / CLOCKS_PER_SEC << '\n';
    out.close();
    requireStart(out.good(), "cannot finish SP witness: " + path);
}

struct ObjectiveAudit {
    long double raw = 0.0L;
    long double discrete = 0.0L;
    long double roundingDelta = 0.0L;
    double maxBinaryResidual = 0.0;
    bool finite = true;

    void add(double value, double cost) {
        const int selected = value > 0.5;
        finite = finite && std::isfinite(value) && std::isfinite(cost);
        maxBinaryResidual = std::max(maxBinaryResidual, std::abs(value - selected));
        raw += static_cast<long double>(cost) * value;
        discrete += static_cast<long double>(cost) * selected;
        roundingDelta += static_cast<long double>(cost) * (value - selected);
    }
};

// Preserve raw variable values and selected routes BEFORE validation can throw.
// This is diagnostic evidence, not a certified feasible result. Omitted variables
// have exact X=0, hence contribute zero to both objective reconstructions.
ObjectiveAudit writeResultDiagnostics(int iteration,
                                     const std::vector<StandaloneRoute>& routes,
                                     const std::vector<double>& rawX,
                                     const std::vector<double>& rawY,
                                     const read_data* data, double rawObjective,
                                     double intFeasTol) {
    const std::string path = "sp_result_" + std::to_string(iteration) + "_raw.tsv";
    std::ofstream out(path);
    requireStart(out.good(), "cannot create SP diagnostics: " + path);
    out << std::setprecision(21)
        << "# Unvalidated candidate; omitted variables have exact X=0\n"
        << "# gurobi_objective=" << rawObjective << " int_feas_tol=" << intFeasTol
        << " model_routes=" << routes.size() << " depots=" << rawY.size() << '\n'
        << "variable\tvalue\tselected\tobjective_coefficient\tdepot\tload\tcustomers\n";
    ObjectiveAudit audit;
    for (size_t r = 0; r < routes.size(); ++r) {
        const auto& route = routes[r];
        const double cost = route.routeCost + data->usingCostVeh;
        audit.add(rawX[r], cost);
        if (rawX[r] == 0.0) continue;
        out << "x_" << r << '\t' << rawX[r] << '\t' << (rawX[r] > 0.5)
            << '\t' << cost << '\t' << route.locId << '\t' << route.totalLoad << '\t';
        for (int customer : route.sequence) out << customer << ' ';
        out << '\n';
    }
    for (size_t l = 0; l < rawY.size(); ++l) {
        const double cost = data->openCostLoc[l + data->numV];
        audit.add(rawY[l], cost);
        if (rawY[l] == 0.0) continue;
        out << "y_" << l << '\t' << rawY[l] << '\t' << (rawY[l] > 0.5)
            << '\t' << cost << '\t' << l << "\t0\t\n";
    }
    out << "# raw_variable_objective=" << audit.raw
        << " discrete_model_objective=" << audit.discrete
        << " objective_rounding_delta=" << audit.roundingDelta
        << " max_binary_residual=" << audit.maxBinaryResidual
        << " finite=" << audit.finite << '\n';
    out.close();
    requireStart(out.good(), "cannot finish SP diagnostics: " + path);
    return audit;
}

class StartReceipt : public GRBCallback {
public:
    bool accepted = false;
    bool callbackFailed = false;
protected:
    void callback() override {
        try {
            if (where == GRB_CB_MESSAGE &&
                getStringInfo(GRB_CB_MSG_STRING).find("Loaded user MIP start with objective") != std::string::npos)
                accepted = true;
        } catch (...) {
            callbackFailed = true;
        }
    }
};
} // namespace

SPSolver::SPSolver(read_data* data) {
    this->data = data;
    this->env = nullptr;
    try {
        // 初始化 Gurobi 环境，由 read_data 统管参数
        this->env = new GRBEnv(true);
        this->env->set("LogFile", data->spLogFile);
        this->env->set(GRB_IntParam_OutputFlag, data->spOutputFlag);
        this->env->start();
    } catch(GRBException& e) {
        std::cerr << "[SPSolver] Error initializing Gurobi env: " << e.getMessage() << std::endl;
        delete this->env;
        this->env = nullptr;
        throw;
    }
}

SPSolver::~SPSolver() {
    if (this->env != nullptr) {
        delete this->env;
    }
}

std::vector<StandaloneRoute> SPSolver::solve(const RoutePool* pool, const Individual* incumbent) {
    lastTimeoutNoSolution = false;
    std::vector<StandaloneRoute> optimalCombination;
    if (this->env == nullptr || pool == nullptr) {
        throw std::runtime_error("SP requires a valid solver environment and route pool");
    }

    try {
        const auto started = WallClock::now();
        const clock_t cpuStarted = clock();
        std::vector<int> startOpened;
        const auto startRoutes = extractIncumbent(incumbent, data, startOpened);
        // Only this invocation's snapshot is augmented. Never mutate or prune RoutePool.
        std::vector<StandaloneRoute> routes = pool->getPool();
        if (data->hardTimeWindows) {
            for (const auto& route : routes) {
                requireStart(data->routeTimeFeasible(route.locId, route.sequence), "invalid TW column in SP pool");
                int load = 0, previous = route.locId + data->numV;
                double travel = 0.0;
                std::set<int> visited;
                for (int customer : route.sequence) {
                    requireStart(visited.insert(customer).second, "duplicate customer in TW column");
                    load += data->demCus[customer];
                    travel += data->D[previous][customer];
                    previous = customer;
                }
                travel += data->D[previous][route.locId + data->numV];
                requireStart(!route.sequence.empty() && load <= data->vehCap && load == route.totalLoad
                    && load <= data->invLoc[route.locId + data->numV] && sameCost(travel, route.routeCost),
                    "invalid TW column capacity or cost");
            }
        }
        const size_t permanentRoutes = routes.size();
        std::unordered_map<StandaloneRoute, int, StandaloneRouteHash> routeIndex;
        for (size_t r = 0; r < routes.size(); ++r) routeIndex.emplace(routes[r], static_cast<int>(r));
        std::vector<int> startIndices;
        for (const auto& route : startRoutes) {
            const auto inserted = routeIndex.emplace(route, static_cast<int>(routes.size()));
            if (inserted.second) routes.push_back(route);
            const int index = inserted.first->second;
            requireStart(routes[index].totalLoad == route.totalLoad && sameCost(routes[index].routeCost, route.routeCost),
                         "pooled incumbent route metadata mismatch");
            startIndices.push_back(index);
        }
        writeWitness("start", data->iteration, startRoutes, startOpened, incumbent->fit);
        StartReceipt receipt; // Remains alive until after the model is destroyed.
        GRBModel model = GRBModel(*this->env);

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
            int maxVisit = routeLimit(data, l);

            model.addConstr(depotRouteCount[l] <= maxVisit * y[l], "MaxVeh_" + std::to_string(l));
        }

        // The selected family profile resolves the size-dependent limit once.
        double dynamicTimeLimit = data->spTimeLimit;
        dynamicTimeLimit = std::min(dynamicTimeLimit, std::max(0.001, experimentRemaining()));

        if (data->hardTimeWindows) {
            dynamicTimeLimit = std::min(dynamicTimeLimit,
                std::max(0.001, data->timeLimit - static_cast<double>(clock()) / CLOCKS_PER_SEC));
        }

        // Report the effective limit after the Set F remaining-budget clamp.
        std::cout << "[SPSolver] profile=" << data->parameterProfile << " 客户数 " << data->numV
                  << " -> 分配 Gurobi 求解上限时间: " << dynamicTimeLimit << "s" << std::endl;
        // ==========================================
        // 求解器参数设置
        model.set(GRB_DoubleParam_TimeLimit, dynamicTimeLimit);
        model.set(GRB_DoubleParam_MIPGap, data->spMIPGap);
        model.set(GRB_IntParam_Threads, data->spThreads); // 严格限制 Gurobi 实例占用的物理线程数
        if (data->experimentMode) model.set(GRB_IntParam_Seed, 0);
        // 【新增】应用解耦后的高级参数
        model.set(GRB_IntParam_MIPFocus, data->spMIPFocus);
        model.set(GRB_DoubleParam_Heuristics, data->spHeuristics);

        // Complete binary MIP start; these are suggestions, not fixed bounds.
        for (int r = 0; r < numRoutes; ++r) x[r].set(GRB_DoubleAttr_Start, 0.0);
        for (int r : startIndices) x[r].set(GRB_DoubleAttr_Start, 1.0);
        for (int l = 0; l < numDepots; ++l) y[l].set(GRB_DoubleAttr_Start, startOpened[l]);
        model.set(GRB_IntParam_OutputFlag, 1);
        model.set(GRB_IntParam_LogToConsole, 0);
        model.set(GRB_StringParam_LogFile, data->spLogFile);
        model.setCallback(&receipt);
        std::ostringstream startEvent;
        startEvent << std::setprecision(17) << "[SP WARM START] iteration=" << data->iteration
                   << " permanent_routes=" << permanentRoutes << " model_routes=" << numRoutes
                   << " added_routes=" << routes.size() - permanentRoutes
                   << " start_routes=" << startRoutes.size() << " start_objective=" << incumbent->fit
                   << " validated=1 complete=1";
        std::cout << startEvent.str() << std::endl;
        if (data->experimentMode) model.update();
        const auto optimizeStarted = WallClock::now();
        const double optimizeCPUStarted = StudyEvents::cpu();
        const double buildCPU = optimizeCPUStarted - static_cast<double>(cpuStarted) / CLOCKS_PER_SEC;
        modelBuildCPU += buildCPU;
        if (study) study->event("sp_optimizer_started", "\"call_id\":\"sp_" + std::to_string(data->iteration)
            + "\",\"scheduled_counter\":" + std::to_string(data->iteration)
            + ",\"event_status\":\"optimizer_started\",\"model_columns\":" + std::to_string(numRoutes)
            + ",\"model_nonzeros\":" + std::to_string(model.get(GRB_IntAttr_NumNZs))
            + ",\"pool_ordered_routes\":" + std::to_string(permanentRoutes)
            + ",\"tw_route_certificates_checked\":" + std::to_string(permanentRoutes + startRoutes.size())
            + ",\"gurobi_seed\":" + std::to_string(model.get(GRB_IntParam_Seed))
            + ",\"model_build_cpu_seconds\":" + StudyEvents::number(buildCPU)
            + ",\"effective_time_limit_wall_seconds\":" + StudyEvents::number(dynamicTimeLimit));
        ++optimizerStarts;
        model.optimize();
        const auto optimizeFinished = WallClock::now();
        const double optimizeCPU = StudyEvents::cpu() - optimizeCPUStarted;
        optimizationCPU += optimizeCPU;

        // 提取结果
        const int solutionCount = model.get(GRB_IntAttr_SolCount);
        if (solutionCount > 0) {
            std::vector<double> rawX(numRoutes), rawY(numDepots);
            for (int r = 0; r < numRoutes; ++r) {
                rawX[r] = x[r].get(GRB_DoubleAttr_X);
                if (rawX[r] > 0.5) {
                    optimalCombination.push_back(routes[r]);
                }
            }
            std::vector<int> resultOpened(numDepots, 0);
            for (int l = 0; l < numDepots; ++l) {
                rawY[l] = y[l].get(GRB_DoubleAttr_X);
                resultOpened[l] = rawY[l] > 0.5;
            }
            const double rawObjective = model.get(GRB_DoubleAttr_ObjVal);
            const double intFeasTol = model.get(GRB_DoubleParam_IntFeasTol);
            const auto audit = writeResultDiagnostics(data->iteration, routes, rawX, rawY,
                                                      data, rawObjective, intFeasTol);
            writeWitness("candidate", data->iteration, optimalCombination, resultOpened,
                         static_cast<double>(audit.discrete));
            requireStart(!receipt.callbackFailed && receipt.accepted,
                         "Gurobi did not acknowledge the complete MIP start");
            requireStart(audit.finite && audit.maxBinaryResidual <= intFeasTol,
                         "SP result has non-finite or non-binary variable values");
            const double recomputed = validateRoutes(optimalCombination, resultOpened, data);
            // ObjVal uses the raw X values, which need not be exact integers.
            // Compare like with like: raw model objective vs sum(c*X), and route
            // cost vs sum(c*selected). Do not weaken route/feasibility tolerances
            // or round ObjVal and assume that it represents a feasible solution.
            std::ostringstream objectiveDetails;
            objectiveDetails << std::setprecision(17) << ": iteration=" << data->iteration
                             << " gurobi_objective=" << rawObjective
                             << " raw_variable_objective=" << audit.raw
                             << " discrete_model_objective=" << audit.discrete
                             << " route_objective=" << recomputed;
            requireStart(sameCost(rawObjective, static_cast<double>(audit.raw)),
                         "SP raw objective does not match variable values" + objectiveDetails.str());
            requireStart(sameCost(recomputed, static_cast<double>(audit.discrete)),
                         "SP route objective does not match selected model columns" + objectiveDetails.str());
            requireStart(recomputed <= incumbent->fit || sameCost(recomputed, incumbent->fit),
                         "SP result is worse than the accepted incumbent start" + objectiveDetails.str());
            writeWitness("result", data->iteration, optimalCombination, resultOpened, recomputed);
            if (study) study->event("sp_incumbent_returned", "\"call_id\":\"sp_" + std::to_string(data->iteration)
                + "\",\"event_status\":\"incumbent_returned\",\"solver_status\":" + std::to_string(model.get(GRB_IntAttr_Status))
                + ",\"raw_sp_objective\":" + StudyEvents::number(recomputed)
                + ",\"raw_sp_validation_pass\":true,\"mip_runtime_wall_seconds\":" + StudyEvents::number(model.get(GRB_DoubleAttr_Runtime))
                + ",\"sp_process_cpu_seconds\":" + StudyEvents::number(optimizeCPU)
                + ",\"model_build_cpu_seconds\":" + StudyEvents::number(buildCPU));
            std::ostringstream resultEvent;
            resultEvent << std::setprecision(17) << "[SP RESULT] iteration=" << data->iteration
                        << " start_accepted=1 status=" << model.get(GRB_IntAttr_Status)
                        << " solution_count=" << solutionCount << " objective=" << recomputed
                        << " gurobi_objective=" << rawObjective
                        << " raw_variable_objective=" << audit.raw
                        << " discrete_model_objective=" << audit.discrete
                        << " objective_rounding_delta=" << audit.roundingDelta
                        << " max_binary_residual=" << audit.maxBinaryResidual
                        << " int_feas_tol=" << intFeasTol
                        << " bound=" << model.get(GRB_DoubleAttr_ObjBound)
                        << " mip_gap=" << model.get(GRB_DoubleAttr_MIPGap)
                        << " node_count=" << model.get(GRB_DoubleAttr_NodeCount)
                        << " gurobi_runtime_seconds=" << model.get(GRB_DoubleAttr_Runtime)
                        << " gurobi_work=" << model.get(GRB_DoubleAttr_Work)
                        << " prepare_wall_seconds=" << std::chrono::duration<double>(optimizeStarted - started).count()
                        << " optimize_wall_seconds=" << std::chrono::duration<double>(optimizeFinished - optimizeStarted).count()
                        << " sp_wall_seconds=" << std::chrono::duration<double>(WallClock::now() - started).count()
                        << " sp_cpu_seconds=" << static_cast<double>(clock() - cpuStarted) / CLOCKS_PER_SEC;
            std::cout << resultEvent.str() << std::endl;
        } else if (data->experimentMode && !data->hardTimeWindows && study
                   && model.get(GRB_IntAttr_Status) == GRB_TIME_LIMIT && !receipt.callbackFailed) {
            // A short wall limit can expire before Gurobi processes the complete start.
            // This is a no-candidate endpoint, never a returned/accepted SP solution.
            lastTimeoutNoSolution = true;
            study->event("sp_timeout_no_solution", "\"call_id\":\"sp_" + std::to_string(data->iteration)
                + "\",\"event_status\":\"timeout_no_solution\",\"solver_status\":" + std::to_string(GRB_TIME_LIMIT)
                + ",\"solution_count\":0,\"start_accepted\":" + (receipt.accepted ? "true" : "false")
                + ",\"mip_runtime_wall_seconds\":" + StudyEvents::number(model.get(GRB_DoubleAttr_Runtime))
                + ",\"sp_process_cpu_seconds\":" + StudyEvents::number(optimizeCPU)
                + ",\"model_build_cpu_seconds\":" + StudyEvents::number(buildCPU));
            std::cout << "[SP TIMEOUT NO SOLUTION] iteration=" << data->iteration
                      << " status=" << GRB_TIME_LIMIT << " solution_count=0" << std::endl;
        } else {
            throw std::runtime_error("no solution returned despite a complete feasible MIP start");
        }
    } catch(GRBException& e) {
        optimalCombination.clear();
        std::cerr << "[SPSolver] Gurobi Optimization Exception: Error code = "
                  << e.getErrorCode() << " - " << e.getMessage() << std::endl;
        throw;
    } catch(const std::exception& e) {
        optimalCombination.clear();
        std::cerr << "[SPSolver] Warm-start Exception: " << e.what() << std::endl;
        throw;
    } catch(...) {
        optimalCombination.clear();
        std::cerr << "[SPSolver] Unknown exception during optimization." << std::endl;
        throw;
    }

    return optimalCombination;
}
