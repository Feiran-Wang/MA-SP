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

    const SpConfig& config() const noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

#endif  // SPSOLVER_H_
