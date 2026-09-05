#ifndef SPSOLVER_H_
#define SPSOLVER_H_

#include <memory>
#include <vector>

#include "Route.h"
#include "SpTypes.h"

class SPSolver {
public:
    explicit SPSolver(SpConfig config);
    ~SPSolver();

    SPSolver(const SPSolver&) = delete;
    SPSolver& operator=(const SPSolver&) = delete;
    SPSolver(SPSolver&&) noexcept;
    SPSolver& operator=(SPSolver&&) noexcept;

    SpResult solve(
        const SpProblem& problem,
        const std::vector<StandaloneRoute>& routes);
    SpResult solve(
        const SpProblem& problem,
        const std::vector<StandaloneRoute>& routes,
        const SpMipStart& preferredStart);

    SpResult solveLp(
        const SpProblem& problem,
        const std::vector<StandaloneRoute>& routes,
        const SpLpRequest& request = SpLpRequest{});

    SpResult solveMip(
        const SpProblem& problem,
        const std::vector<StandaloneRoute>& routes,
        const SpMipRequest& request = SpMipRequest{});
    SpResult solveMip(
        const SpProblem& problem,
        const std::vector<StandaloneRoute>& routes,
        const SpMipRequest& request,
        const SpMipStart& preferredStart);

    const SpConfig& config() const noexcept;

    // Release the process-wide Gurobi environment after every SPSolver (and
    // therefore every GRBModel) has been destroyed.  Relying on function-
    // static destruction defers the license/session teardown until after
    // main-owned logging and other runtime services have gone away.
    static void shutdownProcessEnvironment() noexcept;

private:
    SpResult solveInternal(
        const SpProblem& problem,
        const std::vector<StandaloneRoute>& routes,
        const SpMipStart& preferredStart,
        const SpConfig& effectiveConfig,
        SpSolveStage stage,
        SpMipMode mipMode,
        std::optional<double> mipTargetObjective);

    class Impl;
    std::unique_ptr<Impl> impl_;
};

#endif  // SPSOLVER_H_
