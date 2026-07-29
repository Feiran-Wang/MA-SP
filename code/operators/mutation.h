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
#include <vector>

class mutation {
public:
	mutation(read_data *data);
	virtual ~mutation();
	// Returns true when a mutation was committed.  In CLRP-TW mode the
	// operator works on a detached value-level route plan and commits only a
	// complete solution that satisfies capacity, depot-capacity, and every
	// hard time window.  A failed attempt leaves the incumbent unchanged.
	bool mutationRun(Individual *s);
	void setCustomerDualGuidance(
		const std::vector<double>& customerDuals);

protected:
	// Kept as a narrow virtual seam so transactional commit/rollback behavior
	// can be fault-injected in unit tests. Production subclasses should not
	// alter the validation contract.
	virtual void validateTimeWindowCommit(Individual *target) const;

private:
	Individual *s;
	void shawRemoval();
	bool timeWindowMutation();
	void initializeTimeWindowNormalization();
	void distanceRemove();
	double compute(bool istrue, double value);
	void insertNode(Node *U, Node *V);
	//
	void locationRemove();
	int selectRemovalSeed() const;



	read_data *data;
	int numV;
	int numL;
	int numC;
	int *auxArray1;
	int perturbLength;
	std::vector< int >listRemove;
	int *cityRank;

	// Instance-static normalization values are computed once on the first
	// CLRP-TW mutation.  Service starts and route membership remain
	// solution-specific and are intentionally not cached.
	bool timeWindowNormalizationInitialized = false;
	double maximumCustomerDistance = 1.0;
	double timeWindowTimeScale = 1.0;
	double timeWindowDemandScale = 1.0;
	std::vector<double> timeWindowCentres;
	std::vector<double> customerDualGuidance;
};

#endif /* MUTATION_H_ */
