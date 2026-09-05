/******************************************************************************
 * File: popManager.h
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description: 
 *   Manages the population of individuals, sub-populations, and diversity logic.
 ******************************************************************************/

#ifndef POPMANAGER_H_
#define POPMANAGER_H_
#include "SolverContext.h"
#include "Individual.h"
#include "RouteProvenance.h"
#include <cstdint>
#include <memory>
#include <optional>
typedef std::vector <Individual*> SubPopulation ;
typedef std::vector <std::vector<Individual*>> Population ;

enum class SpInjectionDisposition {
	Rejected,
	AdmittedExisting,
	AdmittedNewConfiguration,
};

struct SpInjectionResult {
	SpInjectionDisposition disposition =
		SpInjectionDisposition::Rejected;
	int populationIndex = -1;
	// A validated strict Global-SP solution may become the global incumbent
	// even when duplicate filtering admits no new population member.
	bool incumbentAccepted = false;

	bool admitted() const noexcept {
		return disposition != SpInjectionDisposition::Rejected;
	}
	bool createdPopulation() const noexcept {
		return disposition ==
			SpInjectionDisposition::AdmittedNewConfiguration;
	}
};

namespace hgasp {

enum class PopulationAdmissionRole {
	FixedResident,
	UnrestrictedFree,
	DynamicChallenger,
	TransactionStaging,
};

struct PopulationAdmissionTarget {
	PopulationAdmissionRole role =
		PopulationAdmissionRole::UnrestrictedFree;
	int populationIndex = -1;
	std::uint64_t generation = 0U;
	std::optional<DepotConfigurationKey> expectedConfiguration;
};

}  // namespace hgasp

class popManager {
public:
	popManager(SolverContext * data);
	virtual ~popManager();
	void reduceHalfSubPop();
	void removeEmptySubPop();

	void copyAllData(Individual* target, const Individual* source);
	// Accepts an independently validated SP solution into incumbent state
	// without requiring any resident/free population transaction to succeed.
	bool acceptValidatedSpIncumbent(
		Individual& individual,
		const hgasp::RouteSubmissionContext* submission = nullptr,
		const Individual::PopulationValidationReceipt*
			validation = nullptr);
	// Optionally exports accepted routes to the shared route pool.
	int addIndividual(
		Individual *indiv,
		int rankPop,
		bool isInitial,
		class RoutePool* pool = nullptr,
		const hgasp::RouteSubmissionContext* submission = nullptr,
		bool* admitted = nullptr,
		std::uint64_t* survivingIndividualId = nullptr,
		bool allowIncumbentUpdate = true,
		const Individual::PopulationValidationReceipt*
			validation = nullptr);
	// Static HGAMP admission is generation-qualified and validates the actual
	// evaluator-produced configuration again at the population boundary.
	int addIndividualToTarget(
		Individual* indiv,
		const hgasp::PopulationAdmissionTarget& target,
		bool isInitial,
		class RoutePool* pool = nullptr,
		const hgasp::RouteSubmissionContext* submission = nullptr,
		bool* admitted = nullptr,
		std::uint64_t* survivingIndividualId = nullptr,
		bool allowIncumbentUpdate = true,
		const Individual::PopulationValidationReceipt*
			validation = nullptr);
	void addInElitePop(Individual *indiv);
	// Clears every restartable population while retaining the optional
	// permanent unrestricted population owned by static HGAMP.
	void reStart(int preservedPopulationIndex = -1);
	//
	void removeWorstBiasedFitness(SubPopulation & pop);
	void updateBiasedFitnesses(SubPopulation & pop);
	/////////////
	Individual * selectionParentSolution(Individual *exist, int rankPop);
	Individual * selectionMotherSolution(Individual *exist, int rankPop);
	double averagePop();
	double usedDepots();
	//variables;
	Individual *global;
	Individual *bestIndivRestart;
	std::vector <std::vector<Individual*>> popula;
	int reverseReplacementCount; // Number of free-population takeovers.

	// Returns the resident population with the worst mean objective.
    int getWorstResidentIndex();
    
    // Admits an SP solution to a matching resident population, or lets it
    // replace the worst resident population when it introduces a new layout.
	SpInjectionResult injectSPSolution(
		std::unique_ptr<Individual> spIndiv,
		class RoutePool* pool = nullptr,
		const hgasp::RouteSubmissionContext* submission = nullptr);

	// Creation iteration used to protect newly established populations.
    std::vector<long long int> popCreationIteration;


private:
	//variables
	SolverContext *data;
	SubPopulation feasibleSubpopulation;			// Feasible subpopulation, kept ordered by increasing penalized cost

	int numV;
	int numL;
	int numC;
	std::uint64_t nextIndividualId = 1U;
	// Normal diversity eviction is followed by another admission into the same
	// manager. Retain a bounded set of already-defined objects so their large,
	// immutable route/node storage can be reused without changing population
	// order, distance calculations, or individual identity assignment.
	std::vector<Individual*> recycledIndividuals;
	Individual* acquirePopulationIndividual();
	void recyclePopulationIndividual(Individual* individual);
	// Internal continuation for candidates that have already crossed the
	// uncached feasibility/objective trust boundary in this call chain.
	// Keeping this private prevents callers from bypassing admission validation.
	int addValidatedIndividual(
		Individual* indiv,
		int rankPop,
		bool isInitial,
		class RoutePool* pool,
		const hgasp::RouteSubmissionContext* submission,
		bool* admitted,
		std::uint64_t* survivingIndividualId,
		bool allowIncumbentUpdate);
	void assignSearchMetadata(
		Individual* individual,
		const hgasp::RouteSubmissionContext* submission);
	void rebuildProximity(SubPopulation& population);
	bool validateForPopulationAdmission(
		Individual& individual,
		const Individual::PopulationValidationReceipt*
			validation);
};
#endif /* POPMANAGER_H_ */
