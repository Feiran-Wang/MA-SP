#include "ProblemContext.h"

#include "SolverContext.h"

#include <utility>

namespace hgasp {

ProblemContext::ProblemContext(ProblemProfile profile)
    : profile_(std::move(profile)) {
    validateProblemProfile(profile_);
    if (profile_.constraints.hasHardTimeWindows()) {
        temporalConstraint_ =
            std::make_unique<HardTimeWindowConstraint>();
    } else {
        temporalConstraint_ =
            std::make_unique<NoTemporalConstraint>();
    }
}

ProblemContext::~ProblemContext() = default;
ProblemContext::ProblemContext(ProblemContext&&) noexcept = default;
ProblemContext& ProblemContext::operator=(ProblemContext&&) noexcept =
    default;

const ProblemProfile& ProblemContext::profile() const noexcept {
    return profile_;
}

bool ProblemContext::hasHardTimeWindows() const noexcept {
    return profile_.constraints.hasHardTimeWindows();
}

const TemporalConstraint&
ProblemContext::temporalConstraint() const noexcept {
    return *temporalConstraint_;
}

RouteEvaluation ProblemContext::evaluateRoute(
        const SolverContext& data,
        const int depotIndex,
        const std::vector<int>& sequence,
        const EvaluationDetail detail) const {
    return RouteEvaluator(data, *temporalConstraint_).evaluate(
        depotIndex, sequence, detail);
}

}  // namespace hgasp
