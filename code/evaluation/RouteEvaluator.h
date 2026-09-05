/******************************************************************************
 * File: RouteEvaluator.h
 * Description:
 *   Shared route-level cost, capacity, and temporal feasibility evaluation.
 *
 *   Customer identifiers are zero-based indices in [0, data.numV). The depot
 *   argument is a zero-based depot index in [0, data.numL); its matrix node is
 *   data.numV + depotIndex.
 ******************************************************************************/

#ifndef ROUTE_EVALUATOR_H_
#define ROUTE_EVALUATOR_H_

#include <cstddef>
#include <vector>

class SolverContext;

enum class EvaluationDetail {
	Summary,
	FullSchedule
};

enum class TemporalViolationKind {
	None,
	CustomerDueTime,
	DepotCloseTime
};

struct CustomerSchedule {
	int customer = -1;
	double arrivalTime = 0.0;
	double waitingTime = 0.0;
	double serviceStartTime = 0.0;
	double departureTime = 0.0;
};

struct RouteEvaluation {
	int load = 0;
	std::size_t customerCount = 0;
	double travelCost = 0.0;

	double routeStartTime = 0.0;
	double totalWaitingTime = 0.0;
	double returnTime = 0.0;
	std::vector<CustomerSchedule> schedule;

	bool capacityFeasible = true;
	bool temporalFeasible = true;
	// Valid only for CustomerDueTime. Depot-close violations deliberately use
	// no customer sentinel because they are route-level violations.
	int violatingCustomer = -1;
	TemporalViolationKind temporalViolation =
		TemporalViolationKind::None;

	bool feasible() const noexcept {
		return capacityFeasible && temporalFeasible;
	}
};

class TemporalConstraint {
public:
	virtual ~TemporalConstraint() = default;

	// Enables a zero-overhead temporal fast path after common load and cost
	// aggregates have been calculated.
	virtual bool isNoOp() const noexcept = 0;

	virtual void evaluate(
		const SolverContext& data,
		int depotIndex,
		const std::vector<int>& sequence,
		EvaluationDetail detail,
		bool trustedProblemData,
		RouteEvaluation& result) const = 0;
};

class NoTemporalConstraint final : public TemporalConstraint {
public:
	bool isNoOp() const noexcept override;

	void evaluate(
		const SolverContext& data,
		int depotIndex,
		const std::vector<int>& sequence,
		EvaluationDetail detail,
		bool trustedProblemData,
		RouteEvaluation& result) const override;
};

class HardTimeWindowConstraint final : public TemporalConstraint {
public:
	// A service start or return time at most epsilon beyond its upper bound is
	// accepted. The same tolerance is used for customer due times and depot
	// closing times.
	static constexpr double kDefaultEpsilon = 1.0e-9;

	explicit HardTimeWindowConstraint(
		double epsilon = kDefaultEpsilon);

	bool isNoOp() const noexcept override;

	void evaluate(
		const SolverContext& data,
		int depotIndex,
		const std::vector<int>& sequence,
		EvaluationDetail detail,
		bool trustedProblemData,
		RouteEvaluation& result) const override;

	double epsilon() const noexcept;

private:
	double epsilon_;
};

class RouteEvaluator {
public:
	RouteEvaluator(
		const SolverContext& data,
		const TemporalConstraint& temporalConstraint);

	RouteEvaluation evaluate(
		int depotIndex,
		const std::vector<int>& sequence,
		EvaluationDetail detail = EvaluationDetail::Summary) const;

	// Validate immutable instance data once before production search. This
	// does not certify any route: every evaluate() call still validates route
	// indices and computes exact load/cost/temporal feasibility.
	static void validateProblemDataForTrustedEvaluation(
		const SolverContext& data,
		bool requireTemporalData);

private:
	const SolverContext& data_;
	const TemporalConstraint& temporalConstraint_;
	bool trustedProblemData_ = false;
};

#endif  // ROUTE_EVALUATOR_H_
