#ifndef HGASP_PROBLEM_PROBLEMCONTEXT_H_
#define HGASP_PROBLEM_PROBLEMCONTEXT_H_

#include "ProblemProfile.h"
#include "RouteEvaluator.h"

#include <memory>
#include <vector>

class SolverContext;

namespace hgasp {

class ProblemContext {
public:
    explicit ProblemContext(ProblemProfile profile);
    ~ProblemContext();

    ProblemContext(ProblemContext&&) noexcept;
    ProblemContext& operator=(ProblemContext&&) noexcept;
    ProblemContext(const ProblemContext&) = delete;
    ProblemContext& operator=(const ProblemContext&) = delete;

    const ProblemProfile& profile() const noexcept;
    bool hasHardTimeWindows() const noexcept;
    const TemporalConstraint& temporalConstraint() const noexcept;

    RouteEvaluation evaluateRoute(
        const SolverContext& data,
        int depotIndex,
        const std::vector<int>& sequence,
        EvaluationDetail detail = EvaluationDetail::Summary) const;

private:
    ProblemProfile profile_;
    std::unique_ptr<TemporalConstraint> temporalConstraint_;
};

}  // namespace hgasp

#endif  // HGASP_PROBLEM_PROBLEMCONTEXT_H_
