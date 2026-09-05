/******************************************************************************
 * File: LS.h
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description: 
 *   Local Search algorithms for route optimization and refinement.
 ******************************************************************************/

#ifndef LS_H_
#define LS_H_
#include <array>
#include <cstdint>
#include <initializer_list>
#include <utility>

#include "Individual.h"
#include "PromisingArcCandidates.h"
#include "SolverContext.h"
#include "RepairProcedure.h"

class LS {
public:
	enum class HgampMoveKind {
		None,
		M1,
		M2,
		M3,
		M4,
		M5,
		M6,
		M7,
		M8,
		M9,
		M10,
		SplitPrefix,
		SplitSuffix,
		// Read-only compatibility names for pre-V1.16.1 diagnostics. New code
		// and artifacts use the HGAMP move identifiers above.
		RelocateOne = M1,
		OrOptTwo = M2,
		TwoOpt = M8,
		TwoOptStar = M9,
		Swap = M4,
		OrOptThree = 255,
	};
	using TwMoveKind = HgampMoveKind;
	struct HgampLocalSearchMetrics {
		std::size_t effectiveAlpha = 0U;
		std::size_t effectiveDepotBeta = 0U;
		std::size_t maximumCandidateVertices = 0U;
		std::size_t maximumDepotCandidates = 0U;
		std::size_t candidateVertices = 0U;
		std::size_t candidateMoves = 0U;
		std::size_t exactDeltaRejectedMoves = 0U;
		std::size_t temporalSummaryRejectedMoves = 0U;
		std::size_t acceptedRouteEvaluations = 0U;
		std::size_t affectedRouteEvaluations = 0U;
		std::size_t sourceRemovalEvaluations = 0U;
		std::size_t sourceRemovalCacheHits = 0U;
		std::size_t fullRouteSetEvaluations = 0U;
		std::size_t acceptedMoves = 0U;
		std::size_t acceptedRelocateOneMoves = 0U;
		std::size_t acceptedOrOptTwoMoves = 0U;
		std::size_t acceptedOrOptThreeMoves = 0U;
		std::size_t acceptedTwoOptMoves = 0U;
		std::size_t acceptedTwoOptStarMoves = 0U;
		std::size_t acceptedSwapMoves = 0U;
		std::array<std::size_t, 12U> acceptedByHgampMove{};
		std::array<std::size_t, 12U> candidateByHgampMove{};
		std::array<std::size_t, 12U> temporalRejectedByHgampMove{};
		std::array<std::size_t, 12U> evaluatorCheckedByHgampMove{};
		std::size_t acceptedSplitPrefixMoves = 0U;
		std::size_t acceptedSplitSuffixMoves = 0U;
		std::size_t canonicalPairDuplicateSkips = 0U;
		std::size_t canonicalPairCacheQueries = 0U;
		std::size_t canonicalPairCacheHits = 0U;
		std::size_t depotCandidateResolutionRequests = 0U;
		std::size_t depotCandidateResolutionBuilds = 0U;
		std::size_t directedPairFailureSkips = 0U;
		HgampMoveKind firstAcceptedMoveKind = HgampMoveKind::None;
		std::vector<HgampMoveKind> acceptedMoveTrace;
		std::size_t summaryRebuilds = 0U;
		std::size_t rebuilds = 0U;
	};
	using TwLocalSearchMetrics = HgampLocalSearchMetrics;
	// methods
	LS(SolverContext * data,RepairProcedure *repair);
	~LS() = default;
	// The two penalty arguments are retained at the current call boundary for
	// source compatibility. Local search is feasible-only and ignores them.
	void local_search_run(Individual *s, double, double);
	// Role-aware Set S offspring variant. It preserves the same move traversal but
	// defers the objective-only full route traversal to the immediately
	// following population trust-boundary validation.
	void local_search_run_deferred_objective(Individual *s, double, double);
	// Runs the same M1--M10/Split engine while rejecting every candidate whose
	// resulting used-depot set differs from the supplied exact configuration.
	// This educates exact seeds and refines ordinary V3 offspring whose current
	// configuration belongs to an active fixed resident. Free/novel offspring
	// and SP post-LS clones retain unrestricted configuration discovery.
	void local_search_fixed_configuration(
		Individual* s,
		const std::vector<int>& openDepots,
		double,
		double);
	void local_search_fixed_configuration_deferred_objective(
		Individual* s,
		const std::vector<int>& openDepots,
		double,
		double);
	const HgampLocalSearchMetrics& lastMetrics() const noexcept {
		return lastTwMetrics_;
	}
	const HgampLocalSearchMetrics& lastTwMetrics() const noexcept {
		return lastMetrics();
	}

private:
	friend struct LSGenerationTestAccess;
	friend struct ClrpLocalSearchOracleAccess;
	friend struct ClrpLocalSearchHotPathTestAccess;

	// A correctness-first neighborhood for hard-TW profiles. Exact
	// travel deltas reject only moves that cannot improve; every remaining
	// affected route is checked by the active evaluator before commit.
	void hardTwLocalSearch();
	void capacityLocalSearch(bool finalizeObjective);
	void runLocalSearch(Individual* s, bool finalizeCapacityObjective);
	void runFixedConfigurationLocalSearch(
		Individual* s,
		const std::vector<int>& openDepots,
		double penaltyDepot,
		double penaltyVehicle,
		bool finalizeCapacityObjective);
	void ensureLocalSearchCandidates();
	struct CapacityRouteBeforeState {
		Route* route = nullptr;
		NodeDepot* depot = nullptr;
		int oldLoad = 0;
		bool oldUsed = false;
	};
	struct DepotCandidateDescriptor {
		int node = -1;
		int depot = -1;
	};
	struct ResolvedDepotCandidate {
		int node = -1;
		int depot = -1;
		NodeDepot* location = nullptr;
		Node* emptyRouteDepot = nullptr;
		bool fixedConfigurationAllowed = false;
	};
	const std::vector<ResolvedDepotCandidate>& resolvedDepotCandidates(
		int customer);
	void advanceDepotOccupancyGeneration() noexcept;
	struct LinkedMoveScratch {
		std::vector<Node*> first;
		std::vector<Node*> second;
		std::vector<Node*> result;
		std::vector<Node*> newFirst;
		std::vector<Node*> newSecond;
		void reserve(const std::size_t size) {
			first.reserve(size);
			second.reserve(size);
			result.reserve(size);
			newFirst.reserve(size);
			newSecond.reserve(size);
		}
	};
	int effectiveLocalSearchAlpha() const noexcept;
	int effectiveDepotCandidateBeta() const noexcept;
	int linkedDependencyVersion(
		const Route* first,
		const Route* second) const noexcept;
	int routeModificationVersion(const Route* route) const noexcept;
	void updateModifiedRoute(Route* route);
	void markLinkedDepotsModified(
		const NodeDepot* first,
		const NodeDepot* second) noexcept;
	void beginInvocationGeneration();
	void advanceCanonicalTopologyEpoch();
	bool canonicalPairAlreadyEvaluated(
		HgampMoveKind kind,
		int firstBlockStart,
		int secondBlockStart) noexcept;
	CapacityRouteBeforeState captureRouteBefore(Route* route) const;
	void refreshChangedRoutesAndDepots(
		const std::array<CapacityRouteBeforeState, 2U>& states,
		std::size_t count);
	void assertIncrementalDepotStateMatchesFullScan(
		const std::array<NodeDepot*, 2U>& depots,
		std::size_t count) const;
	void resetHgampMetrics();
	bool fixedConfigurationAllowsDepotState(
		const NodeDepot* depot,
		bool usedAfterMove) const noexcept;
	bool collectClrpPostSearchSummary(
		double& totalOpenCost) const noexcept;
	void recordHgampCandidate(HgampMoveKind kind);
	void recordHgampAccepted(HgampMoveKind kind);
	//methods
	void srun();
	void loadSolution();
//	void updateRouteInfor(int &r);
	double compute(bool istrue, double value);

	//moves
	bool Relocate_one();//  0-1 insert
	bool Relocate_two();//  2-0 insert
	bool Relocate_three();//0-2 insert
	bool Relocate_four();// 1-1 swap
	bool Relocate_five();// 2-1 swap
	bool Relocate_six();//  2-2 swap
	bool Relocate_seven();//2-2 swap R
	bool Relocate_eight();//2-opt
	bool Relocate_nine();// HGAMP M9 suffix exchange
	bool Relocate_ten();// HGAMP M10 reverse-oriented prefix/suffix exchange

	//methods for Moves
	void setLocalVariableRouteU();
	void setLocalVariableRouteV();
	void linkedRouteCustomers(
		Route* route,
		std::vector<Node*>& output) const;
	std::size_t nodePosition(
		const std::vector<Node*>& customers,
		const Node* target) const;
	void applyLinkedSegmentExchange(
		Route* firstRoute,
		Node* firstStart,
		std::size_t firstLength,
		Route* secondRoute,
		Node* secondStart,
		std::size_t secondLength,
		bool reverseFirstAtSecond = false);
	void applyLinkedReversedPairBefore(Node* first, Node* destination);
	void applyLinkedM9(Node* u, Node* v);
	void applyLinkedM10(Node* u, Node* v);
	void ensureReverseArcPrefix(Route* route);
	bool promisingArcFilterEnabled() const noexcept;
	bool isPromisingArc(int from, int to) const noexcept;
	bool hasPromisingArc(
		std::initializer_list<std::pair<int, int>> arcs) const noexcept;
	void insertNode(Node *node1, Node *node2);
	//methods for split clients

	/*******************************************Split operators*/
	bool Splitb();
	bool Splitf();
	bool Splitb(const std::vector<ResolvedDepotCandidate>& candidates);
	bool Splitf(const std::vector<ResolvedDepotCandidate>& candidates);
	bool SplitbForDepot(const ResolvedDepotCandidate& candidate);
	bool SplitfForDepot(const ResolvedDepotCandidate& candidate);
	/********************************************/
	SolverContext* data;
	Individual* s;
	int numV;
	int numL;
	int numC;
	std::vector< int>orderNodes;
	Node *nodeU;
	Node *nodeX;
	Node *nodeV;
	Node *nodeY;
	Route *routeU;
	Route *routeV;
	int nodeUPrevIndex, nodeUIndex, nodeXIndex, nodeXNextIndex ;
	int nodeVPrevIndex, nodeVIndex, nodeYIndex;
	int loopID;
	int nbMoves;
	double moveGain;
	bool searchComplete;

	HgampLocalSearchMetrics lastTwMetrics_;
	std::vector<std::vector<int>> localSearchCustomerCandidates_;
	std::vector<std::vector<int>> localSearchDepotCandidates_;
	std::vector<std::vector<DepotCandidateDescriptor>>
		localSearchDepotCandidateDescriptors_;
	std::vector<ResolvedDepotCandidate> resolvedDepotCandidateScratch_;
	int resolvedDepotCandidateCustomer_ = -1;
	std::uint64_t resolvedDepotCandidateInvocationGeneration_ = 0U;
	std::uint64_t resolvedDepotCandidateOccupancyGeneration_ = 0U;
	std::uint64_t depotOccupancyGeneration_ = 1U;
	hgasp::PromisingArcCandidateGraph promisingArcGraph_;
	bool localSearchCandidatesInitialized_ = false;
	std::size_t staticMaximumCustomerCandidates_ = 0U;
	std::size_t staticMaximumDepotCandidates_ = 0U;
	std::vector<int> depotModificationVersions_;
	std::vector<std::uint64_t> depotModificationGenerations_;
	std::vector<int> customerLastTestedVersions_;
	std::vector<std::uint64_t> customerLastTestedGenerations_;
	std::vector<int> routeModificationVersions_;
	std::vector<std::uint64_t> routeModificationGenerations_;
	std::vector<double> reverseArcPrefix_;
	std::vector<int> reversePromisingArcPrefix_;
	std::vector<int> reverseArcPrefixGeneration_;
	std::vector<std::uint64_t> reverseArcPrefixInvocationGenerations_;
	std::uint64_t currentInvocationGeneration_ = 0U;
	std::uint32_t canonicalTopologyEpoch_ = 0U;
	// High 28 bits: topology epoch. Low four bits: M4, M6 and the two
	// directed M7 orientations. One table avoids four random-access matrices.
	std::vector<std::uint32_t> canonicalPairStates_;
	LinkedMoveScratch linkedMoveScratch_;
	bool fixedConfigurationMode_ = false;
	std::vector<bool> fixedOpenDepots_;
};
#endif /* LS_H_ */
