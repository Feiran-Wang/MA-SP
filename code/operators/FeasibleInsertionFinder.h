/******************************************************************************
 * File: FeasibleInsertionFinder.h
 * Description:
 *   Value-level cheapest-feasible insertion for CLRP under active constraints.
 *
 *   The finder never mutates an Individual.  A complete route plan is
 *   returned only after every customer has been covered exactly once and
 *   every route has independently passed the shared RouteEvaluator.
 ******************************************************************************/

#ifndef HGASP_OPERATORS_FEASIBLE_INSERTION_FINDER_H_
#define HGASP_OPERATORS_FEASIBLE_INSERTION_FINDER_H_

#include "Route.h"

#include <string>
#include <vector>

class SolverContext;

namespace hgasp {

// ExhaustiveReference exists only for same-binary differential tests and
// manual performance replay. Production callers use the optimized default.
enum class FeasibleInsertionExecution {
	Optimized,
	ExhaustiveReference,
};

struct FeasibleInsertionResult {
	std::vector<StandaloneRoute> routes;
	double objective = 0.0;
};

class FeasibleInsertionFinder {
public:
	explicit FeasibleInsertionFinder(
		const SolverContext& data,
		FeasibleInsertionExecution execution =
			FeasibleInsertionExecution::Optimized);

	// customerOrder must be a permutation of [0, data.numV).  rankPop < 0
	// selects the deterministic cheapest candidate; nonnegative values select
	// from a small restricted candidate list to diversify the population.
	//
		// On failure, result is cleared and diagnostic contains the first customer
		// for which no active-constraint-feasible insertion exists.
	bool build(
		const std::vector<int>& customerOrder,
		int rankPop,
		FeasibleInsertionResult& result,
		std::string* diagnostic = nullptr) const;

	// A nonempty mask restricts only the depots considered by this heuristic
	// construction. RouteEvaluator remains the authority for capacity and
		// active-constraint feasibility, and callers may retry without the mask.
	bool build(
		const std::vector<int>& customerOrder,
		int rankPop,
		const std::vector<bool>& allowedDepots,
		FeasibleInsertionResult& result,
		std::string* diagnostic = nullptr) const;

	// Guided reconstruction first treats each non-overlapping, order-contiguous
	// trusted block as one insertion unit. If a block has no feasible
	// insertion, construction restarts with singleton insertions so guidance
	// can never reject an otherwise constructible TW offspring.
	bool build(
		const std::vector<int>& customerOrder,
		int rankPop,
		const std::vector<std::vector<int>>& trustedBlocks,
		FeasibleInsertionResult& result,
		std::string* diagnostic = nullptr) const;
	bool build(
		const std::vector<int>& customerOrder,
		int rankPop,
		const std::vector<std::vector<int>>& trustedBlocks,
		const std::vector<bool>& allowedDepots,
		FeasibleInsertionResult& result,
		std::string* diagnostic = nullptr) const;

private:
	const SolverContext& data_;
	FeasibleInsertionExecution execution_;
};

}  // namespace hgasp

#endif  // HGASP_OPERATORS_FEASIBLE_INSERTION_FINDER_H_
