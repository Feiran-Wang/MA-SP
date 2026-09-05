/******************************************************************************
 * File: Individual.h
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description: 
 *   Data structure representing a single solution (individual) in the population.
 ******************************************************************************/

// Individual.h
#ifndef INDIVIDUAL_H_
#define INDIVIDUAL_H_

#include <vector>
#include <set>
#include <cstdint>
#include <utility>
#include <stdexcept>
#include <string>
#include "basic.h"
#include "SolverContext.h"
#include "Route.h"
struct Node;
struct NodeDepot;
class popManager;
struct Route{
	bool used;
	int cour;
	int nbClients;
	int whenLastModified;				// "When" this route has been last modified
	Node *depot;
	Node *endDepot;
	NodeDepot *whichLocation;
	double dis;					// Total time on the route
	int load;						// Total load on the route
	double returnTime;				// Return time at the route's depot
	double waitingTime;				// Total waiting before customer service
	bool temporalFeasible;			// Hard-TW/depot-horizon feasibility
	int temporalViolationCustomer;	// Internal customer index, or -1
	double penalty;						// Current sum of load and duration penalties
};

// Route occupancy is structural, not capacitary: a route containing only
// zero-demand customers still consumes a vehicle slot and opens its depot.
// nbClients is rebuilt from the linked customer sequence by updateRouteInfor.
inline bool routeIsOccupied(const Route& route) noexcept {
	return route.nbClients > 0;
}

class IndividualObjectiveConsistencyError final
	: public std::logic_error {
public:
	using std::logic_error::logic_error;
};

struct Node{
	bool isDepot;//tells whether this node represent a depot or not
	int cour;//city index
	int position;
	int dem;
	int whenLastTestedRI;//When the MOVES evaluate this node
	Node *next;
	Node *pre;
	Route *route;
	int cumulatedLoad;// Cumulated load on this route until the customer (including itself)
	double cumulatedDis;	// Cumulated distance on this route until the customer (including itself)
	//
	int start;
	int end;
	bool iseum;
};
struct NodeDepot{
	bool used;
	int cour;
	int inventory;
	Node *depots;
	Node *endDepots;
	int splitTime;
	int maxVisit;
	int remainInven;
	bool iscapCon;
	double penalty;
	std::set< int > emptyRoutes;//used to index all empty routes
};

// Compact, read-mostly view used by population diversity calculations.
// Keeping these four integer fields together avoids repeatedly following the
// Node -> Route -> NodeDepot pointer chain for the comparison individual.
struct DiversityNodeCache {
	int predecessor;
	int successor;
	int route;
	int location;
};
static_assert(
	sizeof(DiversityNodeCache) == 4 * sizeof(int),
	"diversity cache must remain a compact four-integer record");

class Individual{
public:
	struct ActiveRouteRefreshSummary {
		bool vehicleCapacityFeasible = true;
		bool depotCapacityFeasible = true;
		bool capacityFeasible() const noexcept {
			return vehicleCapacityFeasible && depotCapacityFeasible;
		}
	};
	// Proof that this exact Individual state crossed the uncached population
	// trust boundary.  The constructor is private so ordinary feasibility
	// certificates, copied individuals, and external/SP candidates cannot
	// manufacture admission authority.  Population admission accepts the
	// receipt only while the problem revision, topology generation, objective,
	// and feasibility state are unchanged.
	class PopulationValidationReceipt final {
	public:
		PopulationValidationReceipt(
			const PopulationValidationReceipt&) = default;
		PopulationValidationReceipt& operator=(
			const PopulationValidationReceipt&) = default;
	private:
		friend class Individual;
		friend class popManager;
		PopulationValidationReceipt(
			const Individual* individual,
			const SolverContext* problem,
			std::uint64_t problemRevision,
			std::uint64_t topologyGeneration,
			double distance,
			double objective,
			double penalty,
			int capacityExcess,
			int carriedInventory,
			int vehicleInventory,
			bool feasible) noexcept
			: individual_(individual),
			  problem_(problem),
			  problemRevision_(problemRevision),
			  topologyGeneration_(topologyGeneration),
			  distance_(distance),
			  objective_(objective),
			  penalty_(penalty),
			  capacityExcess_(capacityExcess),
			  carriedInventory_(carriedInventory),
			  vehicleInventory_(vehicleInventory),
			  feasible_(feasible) {}

		const Individual* individual_;
		const SolverContext* problem_;
		std::uint64_t problemRevision_;
		std::uint64_t topologyGeneration_;
		double distance_;
		double objective_;
		double penalty_;
		int capacityExcess_;
		int carriedInventory_;
		int vehicleInventory_;
		bool feasible_;
	};

	//methods
	Individual();
	virtual ~Individual();
	Individual(const Individual&) = delete;
	Individual& operator=(const Individual&) = delete;

	// Number of route slots available in the fixed-slot linked-list
	// representation. This is a representation bound, not a fleet limit.
	static int representationRouteLimit(
		const SolverContext& data,
		int depotIndex);

	void define(SolverContext *data);
	void evaluationDis();
	void getSaturationDegree();
	void initilization();
	void updateRouteInfor(int &r,int nbMoves);
	void updateLocationInfor(int &ll);
	// Rebuild route and depot aggregates from the authoritative customer-to-
	// route pointers. Empty representation slots remain canonical and are not
	// rescanned.
	void refreshActiveRouteInformation();
	// Performs the same normalization and reports capacity feasibility from
	// the route/depot aggregates already touched by that pass. This avoids a
	// second customer/depot traversal at the Capacity Repair boundary.
	ActiveRouteRefreshSummary
	refreshActiveRouteInformationAndCheckCapacity();
	// Occupancy-generation-stamped, route-index ordered view of routes
	// referenced by customers. Customer ownership remains authoritative;
	// route-internal topology changes do not invalidate this view.
	const std::vector<int>& activeRouteIndices() const;

	void outputSolution();
	void isRight();
	void decideFeasible();
	// Admission deliberately forces one independent evaluator pass and checks
	// dis/fit even when a current feasibility certificate exists.
	void decideFeasibleAtTrustBoundary();
	// Performs the same uncached scan, then returns admission authority bound
	// to the resulting immutable state.  Keep this separate from the ordinary
	// trust-boundary API so callers that do not continue directly into
	// population admission pay no receipt-construction cost.
	PopulationValidationReceipt validateForPopulationAdmission();
	// Role-aware Set S offspring may defer the final objective-only LS traversal. This
	// trust-boundary pass validates the same routes and writes the independently
	// computed objective before issuing the admission receipt.
	PopulationValidationReceipt
	normalizeAndValidateForPopulationAdmission();
	std::uint64_t topologyGeneration() const noexcept {
		return topologyGeneration_;
	}
	bool hasCurrentFeasibilityCertificate() const noexcept;
	void outputBestSolution();
	// Exact population-duplicate predicate. Route-slot order is ignored, while
	// depot assignment and directed customer order inside every route remain
	// part of the structure.
	bool hasSameRouteStructure(const Individual& other) const;
    
    void rebuildFromSP(const std::vector<StandaloneRoute>& spRoutes);// 根据数学规划求解器返回的抽象路径重组底层链表拓扑
	/**********************************/
	double brokenPairsDistance(Individual * indiv2);
	double averageBrokenPairsDistanceClosest(int nbClosest);
	void removeProximity(Individual * indiv);	//variables
	Node * client;
	NodeDepot * location;
	Route * route;
//	std::vector<std::set< int > >emptyRoutes;//used to index all empty routes
	int inventory;// the total inventory carried by selected locations
	int inventoryVeh;// the total vehicles' capacity
	//
	double dis;
	int capExcess;
	double penalty;
	double fit;
	double biasedFit;
	bool isFeasible;
	// Non-semantic search lineage. These fields never participate in
	// feasibility, objective, structural equality, or diversity.
	std::uint64_t searchIndividualId;
	std::uint64_t sourceTraceId;
	std::string spAncestryCallId;
	std::multiset < std::pair < double, Individual* > > indivsPerProximity ;	// The other individuals in the population, ordered by increasing proximity (the set container follows a natural ordering based on the first value of the pair)
	int maxVnum;
	std::set<std::pair<double,int>> saturatLoc;// the first is degree, the second is the order of location.
	std::set<std::pair<double,int>> saturatRoute;// the first is degree, the second is the order of route.
private:
	friend class popManager;
	friend class crossover;
	// Resets a previously initialized individual before an exact population
	// copy.  Empty route slots are already canonical; only routes referenced by
	// customers can carry state that must be cleared.
	void resetForPopulationCopy();
	void validateFeasibility(
		bool allowCachedCertificate,
		bool normalizeObjective = false);
	void stampFeasibilityCertificate(bool feasible) noexcept;
	bool copyFeasibilityCertificateFrom(
		const Individual& source) noexcept;
	bool matchesPopulationValidationReceipt(
		const PopulationValidationReceipt& receipt) const noexcept;
	void invalidateTopologyState() noexcept;
	void invalidateActiveRouteState() noexcept;
	void updateActiveRouteMembership(int routeIndex, bool occupied);
	void invalidateDiversityCache() noexcept;
	void ensureDiversityCache();
	void ensureStructuralRouteCache() const;

	SolverContext *data;
	int numV;
	int numL;
	int numC;

	int *checkArray;
	std::vector<DiversityNodeCache> diversityCache;
	bool diversityCacheValid;
	mutable std::vector<std::pair<int, std::vector<int>>>
		structuralRouteCache_;
	mutable bool structuralRouteCacheValid_ = false;
	void rebuildActiveRouteIndex(bool requireCompleteAssignment) const;
	mutable std::vector<int> activeRouteScratch_;
	mutable std::uint64_t activeRouteScratchOccupancyGeneration_ = 0U;
	std::uint64_t routeOccupancyGeneration_ = 1U;
	std::vector<int> validationCustomerUseScratch_;
	std::vector<int> validationRouteSequenceScratch_;
	std::uint64_t topologyGeneration_ = 0U;
	std::uint64_t validatedTopologyGeneration_ = 0U;
	std::uint64_t validatedProblemRevision_ = 0U;
	bool validatedFeasibility_ = false;
};
#endif /* INDI_H_ */
