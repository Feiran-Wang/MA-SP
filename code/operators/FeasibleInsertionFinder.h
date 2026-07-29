/******************************************************************************
 * File: FeasibleInsertionFinder.h
 * Description:
 *   Value-level cheapest-feasible insertion for CLRP with hard time windows.
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

class read_data;

namespace hgasp {

struct FeasibleInsertionResult {
	std::vector<StandaloneRoute> routes;
	double objective = 0.0;
};

class FeasibleInsertionFinder {
public:
	explicit FeasibleInsertionFinder(const read_data& data);

	// customerOrder must be a permutation of [0, data.numV).  rankPop < 0
	// selects the deterministic cheapest candidate; nonnegative values select
	// from a small restricted candidate list to diversify the population.
	//
	// On failure, result is cleared and diagnostic contains the first customer
	// for which no capacity-and-time-feasible insertion exists.
	bool build(
		const std::vector<int>& customerOrder,
		int rankPop,
		FeasibleInsertionResult& result,
		std::string* diagnostic = nullptr) const;

private:
	const read_data& data_;
};

}  // namespace hgasp

#endif  // HGASP_OPERATORS_FEASIBLE_INSERTION_FINDER_H_
