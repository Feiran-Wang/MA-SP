/******************************************************************************
 * File: mutation.h
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description: 
 *   Mutation operators for genetic algorithm diversification.
 ******************************************************************************/

#ifndef MUTATION_H_
#define MUTATION_H_
#include "Individual.h"

#include <cstddef>
#include <optional>
#include <vector>

namespace hgasp {
// Read-only result exposed as a narrow diagnostic/testing seam.  It is
// produced by the same active-constraint filtering and scoring path used by
// production recreation, but never mutates or commits an Individual.
struct TimeWindowInsertionPreview {
	bool opensRoute = false;
	int routeIndex = -1;
	int depotIndex = -1;
	std::size_t insertionPosition = 0U;
	double objectiveDelta = 0.0;
	double selectionScore = 0.0;
};

}  // namespace hgasp

class mutation {
public:
	mutation(SolverContext *data);
	virtual ~mutation() = default;
	// Returns true when a mutation was committed. The unified policy alternates
	// constraint-aware ruin/recreate and transactional Shaw operators
	// under the same search policy for every constraint profile.
	bool mutationRun(Individual *s);

	// Diagnostic/testing seam: returns the deterministically best insertion
	// accepted by the active constraints and production scorer. The
	// supplied partial routes and the mutation target remain untouched.
	std::optional<hgasp::TimeWindowInsertionPreview>
	previewBestTimeWindowInsertion(
		int customer,
		const std::vector<StandaloneRoute>& partialRoutes);

protected:
	// Kept as a narrow virtual seam so transactional commit/rollback behavior
	// can be fault-injected in unit tests. Production subclasses should not
	// alter the validation contract.
	virtual void validateTimeWindowCommit(Individual *target) const;

private:
	Individual *s;
	bool shawRemoval();
	bool constraintAwareRuinRecreate();
	bool shawMutation();
	std::size_t mutationRunCount_ = 0U;
	void initializeTimeWindowNormalization();
	double compute(bool istrue, double value);
	void insertNode(Node *U, Node *V);
	int selectRemovalSeed() const;



	SolverContext *data;
	int numV;
	int numL;
	std::vector<int> auxArray1;
	int perturbLength;
	std::vector< int >listRemove;
	std::vector<int> cityRank;

	// Instance-static normalization values are computed once on the first
	// constraint-aware mutation. Service starts and route membership remain
	// solution-specific and are intentionally not cached.
	bool timeWindowNormalizationInitialized = false;
	bool temporalHeuristicFeaturesAvailable = false;
	double maximumCustomerDistance = 1.0;
	double timeWindowTimeScale = 1.0;
	double timeWindowDemandScale = 1.0;
	std::vector<double> timeWindowCentres;
};

#endif /* MUTATION_H_ */
