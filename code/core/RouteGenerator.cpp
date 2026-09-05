/******************************************************************************
 * File: RouteGenerator.cpp
 * Author: Feiran Wang
 * Date: 2026-06-24
 * Description: 
 *   Core engine for generating routes and managing heuristic operators.
 ******************************************************************************/

#include "RouteGenerator.h"
#include "basic.h"
#include "ComponentProfiler.h"
#include "ConfigurationPreselector.h"
#include "CrhConfigurationRanker.h"
#include "NumericPolicy.h"
#include "PopulationReductionPolicy.h"
#include "RunLogger.h"
#include "SearchAttemptScheduler.h"
#include "SpatioTemporalCrh.h"
#include "StaticHgampPopulationPolicy.h"
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <sstream>
#include <set>
#include <stdexcept>
#include <utility>
#include <unordered_set>
#include <vector>

using namespace std;

namespace {

using ConfigurationEvaluationArc = std::pair<int, int>;
using ConfigurationEvaluationStructure =
    std::vector<ConfigurationEvaluationArc>;

ConfigurationEvaluationStructure configurationEvaluationStructure(
        const Individual& individual,
        const SolverContext& data) {
    ConfigurationEvaluationStructure arcs;
    arcs.reserve(static_cast<std::size_t>(data.numV) * 2U);
    for (int depot = 0; depot < data.numL; ++depot) {
        const int depotNode = data.numV + depot;
        for (int slot = 0;
             slot < individual.location[depot].maxVisit;
             ++slot) {
            const Node& anchor =
                individual.location[depot].depots[slot];
            const Route* route = anchor.route;
            if (route == nullptr || !route->used ||
                route->nbClients <= 0) {
                continue;
            }

            int previous = depotNode;
            int visited = 0;
            const Node* node = route->depot->next;
            while (node != nullptr && !node->isDepot &&
                   visited < data.numV) {
                arcs.emplace_back(previous, node->cour);
                previous = node->cour;
                node = node->next;
                ++visited;
            }
            if (node == nullptr || !node->isDepot ||
                visited != route->nbClients) {
                throw std::logic_error(
                    "configuration evaluation encountered a malformed "
                    "route structure");
            }
            arcs.emplace_back(previous, depotNode);
        }
    }
    std::sort(arcs.begin(), arcs.end());
    arcs.erase(std::unique(arcs.begin(), arcs.end()), arcs.end());
    return arcs;
}

ConfigurationEvaluationStructure configurationEvaluationStructure(
        const std::vector<StandaloneRoute>& routes,
        const SolverContext& data) {
    ConfigurationEvaluationStructure arcs;
    arcs.reserve(static_cast<std::size_t>(data.numV) * 2U);
    for (const StandaloneRoute& route : routes) {
        const int depotNode = data.numV + route.locId;
        int previous = depotNode;
        for (const int customer : route.sequence) {
            arcs.emplace_back(previous, customer);
            previous = customer;
        }
        arcs.emplace_back(previous, depotNode);
    }
    std::sort(arcs.begin(), arcs.end());
    arcs.erase(std::unique(arcs.begin(), arcs.end()), arcs.end());
    return arcs;
}

double configurationStructureDistance(
        const ConfigurationEvaluationStructure& left,
        const ConfigurationEvaluationStructure& right) noexcept {
    std::size_t leftIndex = 0U;
    std::size_t rightIndex = 0U;
    std::size_t intersection = 0U;
    while (leftIndex < left.size() && rightIndex < right.size()) {
        if (left[leftIndex] < right[rightIndex]) {
            ++leftIndex;
        } else if (right[rightIndex] < left[leftIndex]) {
            ++rightIndex;
        } else {
            ++intersection;
            ++leftIndex;
            ++rightIndex;
        }
    }
    const std::size_t unionSize =
        left.size() + right.size() - intersection;
    if (unionSize == 0U) {
        return 0.0;
    }
    return 1.0 -
        static_cast<double>(intersection) /
            static_cast<double>(unionSize);
}

struct ConfigurationEvaluationDiversity {
    std::vector<ConfigurationEvaluationStructure> structures;
    double pairwiseDistanceSum = 0.0;
    std::size_t pairCount = 0U;

    double observe(ConfigurationEvaluationStructure structure) {
        for (const auto& existing : structures) {
            pairwiseDistanceSum +=
                configurationStructureDistance(existing, structure);
            ++pairCount;
        }
        structures.push_back(std::move(structure));
        return pairCount == 0U
            ? 0.0
            : pairwiseDistanceSum /
                static_cast<double>(pairCount);
    }
};

struct ConfigurationScreeningTelemetry {
    std::size_t configurationSize = 0U;
    std::size_t attemptedSamples = 0U;
    std::size_t constructorSuccesses = 0U;
    std::size_t constructorExactContractFailures = 0U;
    std::size_t lsAccepted = 0U;
    std::size_t lsDriftRollbacks = 0U;
    std::size_t finalFeasibleSuccesses = 0U;
    double objectiveSum = 0.0;
    double bestObjective = Max;
};

class PopulationStagingGuard {
public:
    explicit PopulationStagingGuard(SubPopulation& population) noexcept
        : population_(&population) {}

    PopulationStagingGuard(const PopulationStagingGuard&) = delete;
    PopulationStagingGuard& operator=(const PopulationStagingGuard&) = delete;

    ~PopulationStagingGuard() noexcept {
        clear();
    }

    void clear() noexcept {
        if (population_ == nullptr) {
            return;
        }
        for (Individual* individual : *population_) {
            delete individual;
        }
        population_->clear();
        population_ = nullptr;
    }

private:
    SubPopulation* population_ = nullptr;
};

std::vector<int> extractCompleteCustomerOrder(
        const Individual& individual,
        const SolverContext& data) {
    std::vector<int> order;
    order.reserve(static_cast<std::size_t>(data.numV));
    std::vector<unsigned char> seen(
        static_cast<std::size_t>(data.numV), 0U);

    for (int depot = 0; depot < data.numL; ++depot) {
        for (int slot = 0;
             slot < individual.location[depot].maxVisit;
             ++slot) {
            const Node* node =
                individual.location[depot].depots[slot].next;
            int guard = 0;
            while (node != nullptr && !node->isDepot &&
                   guard++ <= data.numV) {
                if (node->cour >= 0 && node->cour < data.numV &&
                    seen[static_cast<std::size_t>(node->cour)] == 0U) {
                    seen[static_cast<std::size_t>(node->cour)] = 1U;
                    order.push_back(node->cour);
                }
                node = node->next;
            }
        }
    }

    // Crossover is expected to preserve coverage.  Appending any omitted
    // customer makes the repair boundary total and avoids propagating a
    // partially linked offspring into the constraint-aware constructor.
    std::vector<int> missing;
    for (int customer = 0; customer < data.numV; ++customer) {
        if (seen[static_cast<std::size_t>(customer)] == 0U)
            missing.push_back(customer);
    }
    RandomShuffle(missing);
    order.insert(order.end(), missing.begin(), missing.end());
    return order;
}


std::string configurationLabel(
        const hgasp::DepotConfigurationKey& configuration) {
    std::ostringstream stream;
    for (std::size_t index = 0U;
         index < configuration.openDepots.size();
         ++index) {
        if (index != 0U) {
            stream << ',';
        }
        stream << configuration.openDepots[index];
    }
    return stream.str();
}

}  // namespace

RouteGenerator::RouteGenerator(SolverContext * data) {
    this->data=data;
		hgasp::ConfigurationCandidateArchivePolicy archivePolicy;
		archivePolicy.maximumRecords =
			data->parameters.challenger.archiveCapacity;
		archivePolicy.topObjectiveLimit =
			data->parameters.challenger.archiveTopObjectiveCount;
		archivePolicy.seedLimit =
			data->parameters.challenger.archiveSeedCount;
		archivePolicy.repeatedDiscoveryThreshold =
			data->parameters.challenger.repeatedDiscoveryThreshold;
		archivePolicy.newObservationsAfterRejection =
			data->parameters.challenger.newObservationsAfterRejection;
		archivePolicy.objectiveTolerance =
			data->challengerObjectiveTolerance;
		hgasp::ConfigurationCandidateArchive configuredArchive(archivePolicy);
		candidateArchive.swap(configuredArchive);
		hgasp::DynamicChallengerParameters challengerParameters;
		challengerParameters.staticMinimumAgeAttempts =
			data->parameters.challenger.selectionMinimumAttempt;
		challengerParameters.selectionIntervalAttempts =
			data->parameters.challenger.selectionIntervalAttempts;
		challengerParameters.clrpTrialAttempts =
			data->parameters.challenger.clrpTrialAttempts;
		challengerParameters.twTrialAttempts =
			data->parameters.challenger.twTrialAttempts;
		challengerParameters.protectedPrimaryPeriod =
			data->parameters.challenger.protectedPrimaryPeriod;
		challengerParameters.clrpLatestLaunchFraction =
			data->parameters.challenger.clrpLatestLaunchFraction;
		challengerParameters.eliteMeanCount =
			data->parameters.challenger.eliteMeanCount;
		challengerParameters.objectiveTolerance =
			data->challengerObjectiveTolerance;
		challengerParameters.targetMinimumSize =
			data->parameters.challenger.targetMinimumSize;
		challengerParameters.targetMaximumSize =
			data->parameters.challenger.targetMaximumSize;
		challengerParameters.targetPopulationDivisor =
			data->parameters.challenger.targetPopulationDivisor;
		challengerParameters.maximumConstructionMultiplier =
			data->parameters.challenger.maximumConstructionMultiplier;
		challengerParameters.minimumViableNumerator =
			data->parameters.challenger.minimumViableNumerator;
		challengerParameters.minimumViableDenominator =
			data->parameters.challenger.minimumViableDenominator;
		challengerPolicy =
			hgasp::DynamicChallengerPolicy(challengerParameters);
    // Internalize creation of all operators
    this->popman = std::make_unique<popManager>(data);
    this->cross = std::make_unique<crossover>(data, popman.get());
    this->repair =
        std::make_unique<RepairProcedure>(data, popman.get());
    this->ls = std::make_unique<LS>(data, repair.get());
    this->initialSol =
        std::make_unique<initial_sol>(data, repair.get());
    this->mutationOperator = std::make_unique<mutation>(data);
    hgasp::AdaptiveSearchPolicy allocationPolicy;
    allocationPolicy.exploration =
        data->adaptivePopulationExploration;
    allocationPolicy.diversityWeight =
        data->adaptivePopulationDiversityWeight;
    allocationPolicy.columnWeight =
        data->adaptivePopulationColumnWeight;
    allocationPolicy.emaWeight =
        data->adaptiveSearchEmaWeight;
    allocationPolicy.mutationMinimumPercent =
        data->adaptiveMutationMinimumPercent;
    allocationPolicy.mutationMaximumPercent =
        data->adaptiveMutationMaximumPercent;
    allocationPolicy.populationResponsibility =
        hgasp::PopulationAllocationResponsibility::LegacyPrimaryAndMate;
    searchAllocator.configure(allocationPolicy);
    currentMutationProbabilityPercent =
        data->mutationProbabilityPercent;
    
    temIndiv = std::unique_ptr<Individual>(new Individual());
    temIndiv->define(data);
    global = std::unique_ptr<Individual>(new Individual());
    global->define(data);
    global->initilization();
    loc_best = std::unique_ptr<Individual>(new Individual());
    loc_best->define(data);
    loc_best->initilization();
    bestIndivRestart = std::unique_ptr<Individual>(new Individual());
    bestIndivRestart->define(data);
    bestIndivRestart->initilization();

    popman->global = global.get();
    popman->bestIndivRestart = bestIndivRestart.get();
    int sizeSubPop = 100;
    subPop = std::unique_ptr<Individual[]>(new Individual[sizeSubPop]);
    for (int i = 0; i < sizeSubPop; i++){
        subPop[i].define(data);
        subPop[i].initilization();
    }
    global->fit = Max;
    bestIndivRestart->fit = Max;
    fa = NULL;
    ma = NULL;
}

void RouteGenerator::setCurrentPhysicalAttempt(
        const std::uint64_t attempt) {
    if (attempt < currentPhysicalAttempt_) {
        throw std::logic_error(
            "physical search attempt cannot move backwards");
    }
    currentPhysicalAttempt_ = attempt;
}

int RouteGenerator::currentPhysicalAttemptForProvenance() const {
    if (currentPhysicalAttempt() >
        static_cast<std::uint64_t>(std::numeric_limits<int>::max())) {
        throw std::overflow_error(
            "physical attempt exceeds the legacy provenance integer range");
    }
    return static_cast<int>(currentPhysicalAttempt());
}

RouteGenerator::~RouteGenerator(){
}

hgasp::ConfigurationScreeningResult
RouteGenerator::preprocessingLocationConfiguration(){
    data->compactLocConfigure.clear();
    if (data->freeOnlyPopulationEnabled()) {
        data->numLocConfiguration = 0;
        return {};
    }
    std::vector<std::vector<int>> fileConfigurations;
    fileConfigurations.reserve(
        data->depotConfigurationSeeds().size());
    for (const std::vector<int>& encoded :
         data->depotConfigurationSeeds()) {
        std::vector<int> depots;
        depots.reserve(encoded.size());
        for (const int depotNode : encoded) {
            depots.push_back(depotNode - data->numV);
        }
        fileConfigurations.push_back(std::move(depots));
    }
    std::vector<hgasp::DepotConfigurationKey> legacyConfigurations;
    legacyConfigurations.reserve(data->LocConfigure.size());
    for (const auto& entry : data->LocConfigure) {
        std::vector<int> depots;
        depots.reserve(entry.second.size());
        for (const int depotNode : entry.second) {
            depots.push_back(depotNode - data->numV);
        }
        legacyConfigurations.push_back(
            hgasp::makeDepotConfigurationKey(std::move(depots)));
    }
    const bool fromConfigurationFile = !fileConfigurations.empty();
    const std::vector<hgasp::DepotConfigurationKey>
        suppliedConfigurations =
            hgasp::selectStaticConfigurationCandidates(
                fileConfigurations,
                legacyConfigurations,
                static_cast<std::size_t>(data->numL));
    if (suppliedConfigurations.empty()) {
        // Instances without a candidate file first build the unrestricted
        // bootstrap.  The post-bootstrap static materializer derives actual
        // configurations and deterministic neighbors from those solutions.
        data->numLocConfiguration = 0;
        hgasp::RunLogger::event(
            hgasp::LogLevel::info,
            "search",
            "configuration_screening_deferred",
            {},
            {{"reason", "awaiting_unrestricted_bootstrap"}});
        return {};
    }
    const std::vector<hgasp::DepotConfigurationKey> ranked =
        profileAwareRankConfigurations(suppliedConfigurations);
    hgasp::ConfigurationScreeningResult screening =
        progressivelyEvaluateConfigurations(ranked);
    if (screening.selectedConfigurations.empty()) {
        data->populationMode =
            hgasp::ConfigurationPopulationMode::FreeOnly;
        data->numLocConfiguration = 0;
        data->compactLocConfigure.clear();
        hgasp::RunLogger::event(
            hgasp::LogLevel::warning,
            "search",
            "configuration_screening_failed",
            {},
            {{"reason", "no_profile_feasible_fixed_configuration"}});
        return {};
    }
    publishStaticConfigurations(screening.selectedConfigurations);
    hgasp::RunLogger::event(
        hgasp::LogLevel::info,
        "search",
        "configuration_screening_finished",
        {},
        {
            {"requested_count", data->staticFixedPopulationCount},
            {"actual_count", data->numLocConfiguration},
            {"source", fromConfigurationFile
                ? "candidate_file"
                : "legacy_preprocessed_candidates"},
        });
    return screening;
}

std::vector<hgasp::DepotConfigurationKey>
RouteGenerator::profileAwareRankConfigurations(
        const std::vector<hgasp::DepotConfigurationKey>& input) const {
    hgasp::CrhSpatialInstance spatial;
    spatial.customerDemand.reserve(static_cast<std::size_t>(data->numV));
    spatial.depotCapacity.reserve(static_cast<std::size_t>(data->numL));
    spatial.depotOpeningCost.reserve(static_cast<std::size_t>(data->numL));
    spatial.depotToCustomerCost.assign(
        static_cast<std::size_t>(data->numL),
        std::vector<double>(static_cast<std::size_t>(data->numV), 0.0));
    spatial.customerToCustomerCost.assign(
        static_cast<std::size_t>(data->numV),
        std::vector<double>(static_cast<std::size_t>(data->numV), 0.0));
    for (int customer = 0; customer < data->numV; ++customer) {
        spatial.customerDemand.push_back(data->demCus[customer]);
        for (int other = 0; other < data->numV; ++other) {
            spatial.customerToCustomerCost[customer][other] =
                data->D[customer][other];
        }
    }
    for (int depot = 0; depot < data->numL; ++depot) {
        const int node = data->numV + depot;
        spatial.depotCapacity.push_back(data->invLoc[node]);
        spatial.depotOpeningCost.push_back(data->openCostLoc[node]);
        for (int customer = 0; customer < data->numV; ++customer) {
            spatial.depotToCustomerCost[depot][customer] =
                data->D[node][customer];
        }
    }
    spatial.vehicleCapacity = data->vehCap;
    spatial.vehicleFixedCost = data->usingCostVeh;
    const hgasp::CrhConfigurationRanker classic(std::move(spatial));

    // Production CRH/ST-CRH must create candidates, not merely rescore a
    // caller-supplied list.  Preserve supplied candidates first, then append
    // the deterministic bounded CRH construction until the validated runtime
    // cheap-stage limit is reached.  This avoids full depot-subset
    // enumeration while preserving manifest control of the experiment.
    const std::size_t cheapLimit = static_cast<std::size_t>(
        std::max(1, data->configurationScreeningCheapLimit));
    const std::size_t firstSamples = static_cast<std::size_t>(
        std::max(1, data->configurationEvaluationFirstSamples));
    std::vector<hgasp::DepotConfigurationKey> candidates;
    candidates.reserve(cheapLimit);
    std::set<hgasp::DepotConfigurationKey> seen;
    const auto appendCandidate = [&](const hgasp::DepotConfigurationKey& raw) {
        if (candidates.size() >= cheapLimit) {
            return;
        }
        const hgasp::DepotConfigurationKey canonical =
            hgasp::makeDepotConfigurationKey(raw.openDepots);
        if (!canonical.openDepots.empty() && seen.insert(canonical).second) {
            candidates.push_back(canonical);
        }
    };
    for (const auto& candidate : input) {
        appendCandidate(candidate);
    }
    const auto constructed = classic.constructCandidates(
        cheapLimit,
        firstSamples,
        0U);
    for (const auto& candidate : constructed) {
        appendCandidate(candidate);
    }

    std::vector<hgasp::DepotConfigurationKey> ranked;
    if (!data->hasHardTimeWindows()) {
        const auto scores = classic.rank(candidates);
        ranked.reserve(scores.size());
        for (const auto& score : scores) {
            if (score.aggregateCapacityFeasible) {
                ranked.push_back(score.configuration);
            }
        }
        return ranked;
    }

    hgasp::SpatioTemporalCrhInput temporal;
    temporal.hardTimeWindows = true;
    temporal.customerWindows.reserve(
        static_cast<std::size_t>(data->numV));
    temporal.depotWindows.reserve(
        static_cast<std::size_t>(data->numL));
    temporal.depotToCustomerTravelTime.assign(
        static_cast<std::size_t>(data->numL),
        std::vector<double>(static_cast<std::size_t>(data->numV), 0.0));
    temporal.customerToDepotTravelTime.assign(
        static_cast<std::size_t>(data->numV),
        std::vector<double>(static_cast<std::size_t>(data->numL), 0.0));
    temporal.customerToCustomerTravelTime.assign(
        static_cast<std::size_t>(data->numV),
        std::vector<double>(static_cast<std::size_t>(data->numV), 0.0));
    for (int customer = 0; customer < data->numV; ++customer) {
        temporal.customerWindows.push_back({
            data->readyTime[static_cast<std::size_t>(customer)],
            data->dueTime[static_cast<std::size_t>(customer)],
            data->serviceTime[static_cast<std::size_t>(customer)],
        });
        for (int other = 0; other < data->numV; ++other) {
            temporal.customerToCustomerTravelTime[customer][other] =
                data->T[customer][other];
        }
    }
    for (int depot = 0; depot < data->numL; ++depot) {
        temporal.depotWindows.push_back({
            data->depotOpenTime[static_cast<std::size_t>(depot)],
            data->depotCloseTime[static_cast<std::size_t>(depot)],
            0.0,
        });
        const int node = data->numV + depot;
        for (int customer = 0; customer < data->numV; ++customer) {
            temporal.depotToCustomerTravelTime[depot][customer] =
                data->T[node][customer];
            temporal.customerToDepotTravelTime[customer][depot] =
                data->T[customer][node];
        }
    }
    const hgasp::SpatioTemporalCrh stCrh(
        classic, std::move(temporal));
    const auto scores = stCrh.rank(candidates);
    ranked.reserve(scores.size());
    for (const auto& score : scores) {
        if (score.classic.aggregateCapacityFeasible) {
            ranked.push_back(score.classic.configuration);
        }
    }
    hgasp::recordV114ComponentActivation(
        hgasp::V114Component::SpatioTemporalCrh);
    return ranked;
}

std::vector<StandaloneRoute>
RouteGenerator::snapshotValidatedIndividual(
        const Individual& individual) const {
    if (!individual.isFeasible) {
        throw std::invalid_argument(
            "configuration screening snapshot requires a feasible individual");
    }
    std::vector<StandaloneRoute> snapshot;
    for (int depot = 0; depot < data->numL; ++depot) {
        for (int slot = 0;
             slot < individual.location[depot].maxVisit;
             ++slot) {
            const Node& anchor =
                individual.location[depot].depots[slot];
            const Route* route = anchor.route;
            if (route == nullptr || !route->used ||
                route->nbClients <= 0) {
                continue;
            }

            StandaloneRoute value;
            value.locId = depot;
            const Node* node = anchor.next;
            int visited = 0;
            while (node != nullptr && !node->isDepot &&
                   visited < data->numV) {
                value.sequence.push_back(node->cour);
                node = node->next;
                ++visited;
            }
            if (node == nullptr || !node->isDepot ||
                visited != route->nbClients ||
                value.sequence.empty()) {
                throw std::logic_error(
                    "configuration screening snapshot encountered a malformed "
                    "route");
            }

            const RouteEvaluation evaluation =
                data->problemContext().evaluateRoute(
                    *data, depot, value.sequence);
            if (!evaluation.feasible() ||
                evaluation.load != route->load ||
                !data->equalTwoVar(
                    evaluation.travelCost, route->dis)) {
                throw std::logic_error(
                    "configuration screening snapshot found stale route "
                    "aggregates");
            }
            if (data->hasHardTimeWindows() &&
                (!route->temporalFeasible ||
                 !data->equalTwoVar(
                     evaluation.returnTime, route->returnTime) ||
                 !data->equalTwoVar(
                     evaluation.totalWaitingTime,
                     route->waitingTime))) {
                throw std::logic_error(
                    "configuration screening snapshot found stale temporal "
                    "aggregates");
            }
            value.totalLoad = evaluation.load;
            value.routeCost = evaluation.travelCost;
            value.returnTime = evaluation.returnTime;
            value.totalWaitingTime = evaluation.totalWaitingTime;
            value.temporalFeasible = evaluation.temporalFeasible;
            value.temporalViolationCustomer =
                evaluation.violatingCustomer;
            snapshot.push_back(std::move(value));
        }
    }
    if (snapshot.empty()) {
        throw std::logic_error(
            "configuration screening snapshot contains no used route");
    }
    return snapshot;
}

hgasp::ConfigurationScreeningResult
RouteGenerator::progressivelyEvaluateConfigurations(
        const std::vector<hgasp::DepotConfigurationKey>& ranked) {
    hgasp::ProgressiveConfigurationScreeningPolicy policy;
    policy.cheapLimit = static_cast<std::size_t>(
        data->configurationScreeningCheapLimit);
    policy.firstStageLimit = static_cast<std::size_t>(
        data->configurationScreeningFirstStage);
    policy.finalistLimit = static_cast<std::size_t>(
        data->configurationScreeningFinalists);
    policy.finalLimit = static_cast<std::size_t>(
        data->staticFixedPopulationCount);
    policy.firstStageSamples = static_cast<std::size_t>(
        data->configurationEvaluationFirstSamples);
    policy.totalSamples = static_cast<std::size_t>(
        data->configurationEvaluationTotalSamples);
    policy.ranking = data->hasHardTimeWindows()
        ? hgasp::EvaluatedConfigurationRanking::StabilityFirst
        : hgasp::EvaluatedConfigurationRanking::BestObjectiveFirst;

    std::vector<hgasp::ProgressiveConfigurationCandidate> candidates;
    candidates.reserve(ranked.size());
    for (std::size_t index = 0U; index < ranked.size(); ++index) {
        hgasp::ProgressiveConfigurationCandidate candidate;
        candidate.sourceIndex = index;
        candidate.configuration = ranked[index];
        candidate.aggregateCapacityFeasible = true;
        // The profile-aware CRH/ST-CRH ordering is hierarchical.  Its stable
        // ordinal is therefore the cheap-stage key; no uncalibrated temporal
        // weight is allowed to override that ordering.
        candidate.cheapScore = static_cast<double>(index);
        candidates.push_back(std::move(candidate));
    }
    auto cheap = hgasp::screenConfigurationCheapStage(
        std::move(candidates), policy);
    const std::size_t cheapStageCount = cheap.cheapPool.size();
    const std::size_t firstStageCount = cheap.firstStage.size();

    // The accumulator is keyed by the immutable pre-screening source index,
    // so diversity observations survive both stage reordering and promotion.
    std::vector<ConfigurationEvaluationDiversity> diversityBySource(
        ranked.size());
    std::vector<ConfigurationScreeningTelemetry> telemetryBySource(
        ranked.size());
    std::vector<std::vector<hgasp::ScreenedConfigurationSeed>>
        seedsBySource(ranked.size());
    for (std::size_t sourceIndex = 0U;
         sourceIndex < ranked.size();
         ++sourceIndex) {
        telemetryBySource[sourceIndex].configurationSize =
            ranked[sourceIndex].openDepots.size();
    }

    const auto evaluateTo = [
            this,
            &diversityBySource,
            &telemetryBySource,
            &seedsBySource](
            std::vector<hgasp::ProgressiveConfigurationCandidate>& stage,
            const std::size_t targetSamples) {
        std::vector<hgasp::DepotConfigurationKey> configurations;
        configurations.reserve(stage.size());
        for (const auto& candidate : stage) {
            configurations.push_back(candidate.configuration);
        }
        publishStaticConfigurations(configurations);
        for (std::size_t index = 0U; index < stage.size(); ++index) {
            auto& candidate = stage[index];
            while (candidate.attemptedSamples < targetSamples) {
                if (initializationStopRequested()) {
                    break;
                }
                ++candidate.attemptedSamples;
                if (candidate.sourceIndex >= telemetryBySource.size() ||
                    candidate.sourceIndex >= diversityBySource.size()) {
                    throw std::logic_error(
                        "configuration evaluation source index is out "
                        "of range");
                }
                ConfigurationScreeningTelemetry& telemetry =
                    telemetryBySource[candidate.sourceIndex];
                ++telemetry.attemptedSamples;
                const bool constructed = data->hasHardTimeWindows()
                    ? initialSol->initial_solution(
                        temIndiv.get(), static_cast<int>(index))
                    : initialSol
                        ->initialCapacityOnlyExactConfiguration(
                            temIndiv.get(), static_cast<int>(index));
                if (!constructed) {
                    continue;
                }
                ++telemetry.constructorSuccesses;
                if (!exactPopulationConfiguration(
                        *temIndiv, static_cast<int>(index))) {
                    ++telemetry.constructorExactContractFailures;
                    if (!data->hasHardTimeWindows()) {
                        throw std::logic_error(
                            "exact fixed-configuration constructor violated "
                            "its contract");
                    }
                    continue;
                }
                const ExactConfigurationEducationResult education =
                    educateExactConfigurationSeed(
                        *temIndiv, static_cast<int>(index));
                if (!education.seedValid) {
                    throw std::logic_error(
                        "fixed-configuration education rejected an "
                        "exact feasible constructor result");
                }
                if (education.lsAccepted) {
                    ++telemetry.lsAccepted;
                }
                if (education.rolledBack) {
                    ++telemetry.lsDriftRollbacks;
                }
                temIndiv->decideFeasible();
                if (!temIndiv->isFeasible ||
                    !exactPopulationConfiguration(
                        *temIndiv, static_cast<int>(index))) {
                    throw std::logic_error(
                        "fixed-configuration education violated its "
                        "contract");
                }
                ++telemetry.finalFeasibleSuccesses;
                telemetry.objectiveSum += temIndiv->fit;
                telemetry.bestObjective = std::min(
                    telemetry.bestObjective, temIndiv->fit);
                candidate.successfulObjectives.push_back(
                    temIndiv->fit);
                candidate.diversity =
                    diversityBySource[candidate.sourceIndex].observe(
                        configurationEvaluationStructure(
                            *temIndiv, *data));
                hgasp::ScreenedConfigurationSeed seed;
                seed.configuration = candidate.configuration;
                seed.routes = snapshotValidatedIndividual(*temIndiv);
                seed.objective = temIndiv->fit;
                seed.diversity = candidate.diversity;
                seed.sourceIndex = candidate.sourceIndex;
                hgasp::ConfigurationDiscoveryTrace discovery;
                discovery.beforeConfiguration =
                    hgasp::makeDepotConfigurationKey(
                        std::vector<int>{});
                discovery.afterConfiguration =
                    candidate.configuration;
                discovery.source = data->hasHardTimeWindows()
                    ? hgasp::ConfigurationDiscoverySource::
                          StCrhInitialization
                    : hgasp::ConfigurationDiscoverySource::
                          CrhInitialization;
                discovery.contributors =
                    hgasp::configurationDiscoverySourceBit(
                        discovery.source);
                discovery.evidenceUnitId =
                    makeConfigurationEvidenceUnitId(
                        data->hasHardTimeWindows()
                            ? "st_crh_seed"
                            : "crh_seed");
                discovery.routeAncestry = "screening_source:" +
                    std::to_string(candidate.sourceIndex);
                discovery.independentlyValidated = true;
                discovery.initialized = true;
                queuePendingConfigurationDiscovery(seed, discovery);
                seedsBySource[candidate.sourceIndex].push_back(
                    std::move(seed));
            }
        }
    };

    evaluateTo(cheap.firstStage, policy.firstStageSamples);
    const std::size_t firstStageSuccesses =
        static_cast<std::size_t>(std::count_if(
            cheap.firstStage.begin(),
            cheap.firstStage.end(),
            [](const auto& candidate) {
                return !candidate.successfulObjectives.empty();
            }));
    cheap.firstStage.erase(
        std::remove_if(
            cheap.firstStage.begin(),
            cheap.firstStage.end(),
            [&policy](const auto& candidate) {
                return candidate.attemptedSamples !=
                    policy.firstStageSamples;
            }),
        cheap.firstStage.end());
    std::vector<hgasp::ProgressiveConfigurationCandidate> finalists =
        hgasp::selectConfigurationFinalists(
            std::move(cheap.firstStage), policy);
    const std::size_t finalistCount = finalists.size();
    evaluateTo(finalists, policy.totalSamples);
    finalists.erase(
        std::remove_if(
            finalists.begin(),
            finalists.end(),
            [&policy](const auto& candidate) {
                return candidate.attemptedSamples !=
                    policy.totalSamples;
            }),
        finalists.end());
    const auto selected = hgasp::selectStaticFixedConfigurations(
        std::move(finalists), policy);
    hgasp::ConfigurationScreeningResult result;
    result.selectedConfigurations.reserve(selected.size());
    std::map<std::size_t, std::size_t> selectedSizeDistribution;
    std::size_t retainedSeedCount = 0U;
    for (const auto& candidate : selected) {
        result.selectedConfigurations.push_back(candidate.configuration);
        ++selectedSizeDistribution[
            candidate.configuration.openDepots.size()];
        if (candidate.sourceIndex >= seedsBySource.size()) {
            throw std::logic_error(
                "selected configuration seed source is out of range");
        }
        std::vector<hgasp::ScreenedConfigurationSeed>& seeds =
            seedsBySource[candidate.sourceIndex];
        retainedSeedCount += seeds.size();
        result.reusableSeeds.emplace(
            candidate.configuration, std::move(seeds));
    }
    const std::size_t createdSeedCount = std::accumulate(
        telemetryBySource.begin(),
        telemetryBySource.end(),
        std::size_t{0U},
        [](const std::size_t total,
           const ConfigurationScreeningTelemetry& telemetry) {
            return total + telemetry.finalFeasibleSuccesses;
        });

    for (std::size_t sourceIndex = 0U;
         sourceIndex < telemetryBySource.size();
         ++sourceIndex) {
        const ConfigurationScreeningTelemetry& telemetry =
            telemetryBySource[sourceIndex];
        if (telemetry.attemptedSamples == 0U) {
            continue;
        }
        const double meanObjective = telemetry.finalFeasibleSuccesses == 0U
            ? std::numeric_limits<double>::quiet_NaN()
            : telemetry.objectiveSum /
                static_cast<double>(telemetry.finalFeasibleSuccesses);
        hgasp::RunLogger::event(
            hgasp::LogLevel::info,
            "search",
            "configuration_screening_candidate",
            {},
            {
                {"source_index", static_cast<unsigned long long>(sourceIndex)},
                {"configuration_size", static_cast<unsigned long long>(
                    telemetry.configurationSize)},
                {"attempted_samples", static_cast<unsigned long long>(
                    telemetry.attemptedSamples)},
                {"constructor_successes", static_cast<unsigned long long>(
                    telemetry.constructorSuccesses)},
                {"constructor_exact_contract_failures",
                    static_cast<unsigned long long>(
                        telemetry.constructorExactContractFailures)},
                {"ls_accepted", static_cast<unsigned long long>(
                    telemetry.lsAccepted)},
                {"ls_drift_rollbacks", static_cast<unsigned long long>(
                    telemetry.lsDriftRollbacks)},
                {"final_feasible_successes",
                    static_cast<unsigned long long>(
                        telemetry.finalFeasibleSuccesses)},
                {"mean_objective", meanObjective},
                {"best_objective",
                    telemetry.finalFeasibleSuccesses == 0U
                        ? std::numeric_limits<double>::quiet_NaN()
                        : telemetry.bestObjective},
            });
    }
    std::ostringstream distribution;
    for (const auto& entry : selectedSizeDistribution) {
        if (distribution.tellp() > 0) {
            distribution << ',';
        }
        distribution << entry.first << ':' << entry.second;
    }
    hgasp::RunLogger::event(
        hgasp::LogLevel::info,
        "search",
        "configuration_screening_summary",
        {},
        {
            {"input_candidates", static_cast<unsigned long long>(
                ranked.size())},
            {"cheap_stage_count", static_cast<unsigned long long>(
                cheapStageCount)},
            {"first_stage_count", static_cast<unsigned long long>(
                firstStageCount)},
            {"first_stage_successes", static_cast<unsigned long long>(
                firstStageSuccesses)},
            {"finalists", static_cast<unsigned long long>(finalistCount)},
            {"selected_fixed_configurations",
                static_cast<unsigned long long>(
                    result.selectedConfigurations.size())},
            {"selected_configuration_size_distribution",
                distribution.str()},
            {"screening_seed_snapshots_created",
                static_cast<unsigned long long>(createdSeedCount)},
            {"screening_seed_snapshots_retained",
                static_cast<unsigned long long>(retainedSeedCount)},
            {"screening_seed_snapshots_discarded",
                static_cast<unsigned long long>(
                    createdSeedCount - retainedSeedCount)},
        });
    return result;
}

void RouteGenerator::publishStaticConfigurations(
        const std::vector<hgasp::DepotConfigurationKey>& configurations) {
    data->compactLocConfigure.clear();
    data->compactLocConfigure.reserve(configurations.size());
    for (const auto& configuration : configurations) {
        std::set<int> encoded;
        for (const int depot : configuration.openDepots) {
            if (depot < 0 || depot >= data->numL) {
                throw std::out_of_range(
                    "static population depot index is out of range");
            }
            encoded.insert(data->numV + depot);
        }
        if (encoded.empty()) {
            throw std::invalid_argument(
                "static population configuration cannot be empty");
        }
        data->compactLocConfigure.push_back(std::move(encoded));
    }
    data->numLocConfiguration =
        static_cast<int>(data->compactLocConfigure.size());
}

bool RouteGenerator::exactPopulationConfiguration(
        const Individual& individual,
        const int populationIndex) const {
    if (populationIndex < 0 ||
        populationIndex >= data->numLocConfiguration) {
        return true;
    }
    const hgasp::DepotConfigurationKey actual =
        configurationKey(individual);
    if (staticConfigurationRegistry != nullptr) {
        const auto& slots = staticConfigurationRegistry->slots();
        if (static_cast<std::size_t>(populationIndex) >= slots.size() ||
            !slots[static_cast<std::size_t>(populationIndex)].active) {
            return false;
        }
        return actual ==
            slots[static_cast<std::size_t>(populationIndex)]
                .configuration;
    }
    std::vector<int> expected;
    for (const int depotNode :
         data->compactLocConfigure[
             static_cast<std::size_t>(populationIndex)]) {
        expected.push_back(depotNode - data->numV);
    }
    return actual ==
        hgasp::makeDepotConfigurationKey(std::move(expected));
}

bool RouteGenerator::exactConfiguration(
        const Individual& individual,
        const hgasp::DepotConfigurationKey& configuration) const {
    return configurationKey(individual) == configuration;
}

RouteGenerator::ExactConfigurationEducationResult
RouteGenerator::educateExactConfigurationSeed(
        Individual& target,
        const int populationIndex,
        const bool retainConfigurationDiscoveries) {
    if (populationIndex < 0 ||
        populationIndex >= data->numLocConfiguration) {
        throw std::out_of_range(
            "fixed-seed local-search population index is out of range");
    }
    hgasp::DepotConfigurationKey configuration;
    if (staticConfigurationRegistry != nullptr) {
        const auto& slots = staticConfigurationRegistry->slots();
        if (static_cast<std::size_t>(populationIndex) >= slots.size() ||
            !slots[static_cast<std::size_t>(populationIndex)].active) {
            throw std::out_of_range(
                "fixed-seed local-search population is inactive");
        }
        configuration =
            slots[static_cast<std::size_t>(populationIndex)].configuration;
    } else {
        std::vector<int> depots;
        for (const int depotNode :
             data->compactLocConfigure[
                 static_cast<std::size_t>(populationIndex)]) {
            depots.push_back(depotNode - data->numV);
        }
        configuration =
            hgasp::makeDepotConfigurationKey(std::move(depots));
    }
    return educateExactConfigurationSeed(
        target, configuration, retainConfigurationDiscoveries);
}

RouteGenerator::ExactConfigurationEducationResult
RouteGenerator::educateExactConfigurationSeed(
        Individual& target,
        const hgasp::DepotConfigurationKey& configuration,
        const bool retainConfigurationDiscoveries) {
    (void)retainConfigurationDiscoveries;
    const hgasp::DepotConfigurationKey canonical =
        hgasp::makeDepotConfigurationKey(configuration.openDepots);
    if (canonical != configuration || configuration.openDepots.empty()) {
        throw std::invalid_argument(
            "fixed-seed local search requires a non-empty canonical "
            "configuration");
    }
    for (const int depot : configuration.openDepots) {
        if (depot < 0 || depot >= data->numL) {
            throw std::out_of_range(
                "fixed-seed local-search configuration depot is out of "
                "range");
        }
    }
    ExactConfigurationEducationResult result;
    target.decideFeasible();
    if (!target.isFeasible ||
        !exactConfiguration(target, configuration)) {
        return result;
    }
    result.seedValid = true;

    ls->local_search_fixed_configuration(
        &target,
        configuration.openDepots,
        data->penaltyDepot,
        data->penaltyVeh);
    target.decideFeasible();
    if (!target.isFeasible ||
        !exactConfiguration(target, configuration)) {
        throw std::logic_error(
            "fixed-configuration local search violated its construction "
            "contract");
    }
    result.lsAccepted = true;
    return result;
}

std::vector<hgasp::DepotConfigurationKey>
RouteGenerator::deriveBootstrapConfigurationCandidates() const {
    std::set<hgasp::DepotConfigurationKey> unique;
    if (!popman->popula.empty()) {
        for (const Individual* individual : popman->popula.front()) {
            if (individual != nullptr && individual->isFeasible) {
                unique.insert(configurationKey(*individual));
            }
        }
    }
    if (global != nullptr && global->isFeasible && global->fit < Max) {
        unique.insert(configurationKey(*global));
    }
    std::vector<hgasp::DepotConfigurationKey> bases(
        unique.begin(), unique.end());
    const auto capacityFeasible = [this](
            const hgasp::DepotConfigurationKey& configuration) {
        long long capacity = 0;
        for (const int depot : configuration.openDepots) {
            if (depot < 0 || depot >= data->numL) {
                return false;
            }
            capacity += data->invLoc[data->numV + depot];
        }
        return !configuration.openDepots.empty() &&
            capacity >= static_cast<long long>(data->totalDemand);
    };
    const std::size_t expansionLimit = std::max<std::size_t>(
        static_cast<std::size_t>(data->configurationScreeningCheapLimit),
        static_cast<std::size_t>(
            data->configurationScreeningCheapLimit) *
            static_cast<std::size_t>(
                data->configurationExpansionMultiplier));
    const auto admit = [&](std::vector<int> depots) {
        const hgasp::DepotConfigurationKey candidate =
            hgasp::makeDepotConfigurationKey(std::move(depots));
        if (capacityFeasible(candidate)) {
            unique.insert(candidate);
        }
    };
    for (const auto& base : bases) {
        if (unique.size() >= expansionLimit) {
            break;
        }
        for (std::size_t removed = 0U;
             removed < base.openDepots.size() &&
                 unique.size() < expansionLimit;
             ++removed) {
            std::vector<int> depots = base.openDepots;
            depots.erase(depots.begin() +
                static_cast<std::ptrdiff_t>(removed));
            admit(std::move(depots));
        }
        for (int opened = 0;
             opened < data->numL && unique.size() < expansionLimit;
             ++opened) {
            if (!std::binary_search(
                    base.openDepots.begin(),
                    base.openDepots.end(), opened)) {
                std::vector<int> depots = base.openDepots;
                depots.push_back(opened);
                admit(std::move(depots));
            }
        }
        for (std::size_t removed = 0U;
             removed < base.openDepots.size() &&
                 unique.size() < expansionLimit;
             ++removed) {
            for (int opened = 0;
                 opened < data->numL && unique.size() < expansionLimit;
                 ++opened) {
                if (std::binary_search(
                        base.openDepots.begin(),
                        base.openDepots.end(), opened)) {
                    continue;
                }
                std::vector<int> depots = base.openDepots;
                depots[removed] = opened;
                admit(std::move(depots));
            }
        }
    }
    std::vector<hgasp::DepotConfigurationKey> result;
    result.reserve(unique.size());
    for (const auto& configuration : unique) {
        if (capacityFeasible(configuration)) {
            result.push_back(configuration);
        }
    }
    return result;
}

void RouteGenerator::initializeStaticConfigurationRegistry() {
    if (staticConfigurationRegistry != nullptr) {
        return;
    }
    std::vector<hgasp::DepotConfigurationKey> configurations;
    configurations.reserve(data->compactLocConfigure.size());
    for (const std::set<int>& encoded : data->compactLocConfigure) {
        std::vector<int> depots;
        depots.reserve(encoded.size());
        for (const int depotNode : encoded) {
            depots.push_back(depotNode - data->numV);
        }
        configurations.push_back(
            hgasp::makeDepotConfigurationKey(std::move(depots)));
    }
    staticConfigurationRegistry =
        std::make_unique<hgasp::StaticConfigurationRegistry>(
            static_cast<std::size_t>(
                data->staticFixedPopulationCount),
            static_cast<std::size_t>(data->numL));
    staticConfigurationRegistry->initialize(
        configurations,
        currentPhysicalAttempt(),
        0U);
    if (staticConfigurationRegistry->fixedSlotCount() !=
        configurations.size()) {
        throw std::logic_error(
            "static registry changed the screened population layout");
    }
    staticPopulationRuntime.assign(
        configurations.size() + 1U,
        StaticPopulationRuntimeState{});
    // Population Local-SP is scheduled on the productive-search clock
    // (data->iteration).  Registry protection/cooldown state deliberately
    // uses the physical-attempt clock, so do not copy the registry timestamp
    // into this controller-facing runtime state.
    const std::uint64_t createdAttempt = static_cast<std::uint64_t>(
        std::max(0, data->iteration));
    for (std::size_t index = 0U;
         index < configurations.size();
         ++index) {
        staticPopulationRuntime[index].generation =
            staticConfigurationRegistry->slots()[index].generation;
        staticPopulationRuntime[index].createdAttempt = createdAttempt;
        staticPopulationRuntime[index].lastBestImprovementAttempt =
            createdAttempt;
    }
    StaticPopulationRuntimeState& unrestrictedRuntime =
        staticPopulationRuntime[configurations.size()];
    unrestrictedRuntime.generation = 1U;
    unrestrictedRuntime.createdAttempt = createdAttempt;
    unrestrictedRuntime.lastBestImprovementAttempt = createdAttempt;
    for (int index = 0; index <= data->numLocConfiguration; ++index) {
        refreshStaticRuntime(index);
    }
    hgasp::recordV114ComponentActivation(
        hgasp::V114Component::StaticHgampPopulations);
}

void RouteGenerator::materializeStaticHgampPopulations(RoutePool* pool) {
    if (!data->staticHgampPopulationsEnabled()) {
        return;
    }
    if (staticConfigurationRegistry != nullptr) {
        const std::size_t requiredPopulationSlots =
            hgasp::staticHgampRequiredPopulationSlotCount(
                staticConfigurationRegistry
                    ->unrestrictedPopulationIndex());
        if (popman->popula.size() < requiredPopulationSlots) {
            popman->popula.resize(requiredPopulationSlots);
        }
        for (const auto& token :
             staticConfigurationRegistry->activeTokens()) {
            registerPopulationConfiguration(
                static_cast<int>(token.slot));
            refreshStaticRuntime(static_cast<int>(token.slot));
        }
        refreshStaticRuntime(
            static_cast<int>(
                staticConfigurationRegistry
                    ->unrestrictedPopulationIndex()));
        (void)refreshPermanentFreePopulationFloor();
        observeInitializedFreePopulation(true);
        drainPendingConfigurationDiscoveries(pool);
        return;
    }

    if (data->numLocConfiguration == 0) {
        if (popman->popula.empty() || popman->popula.front().empty()) {
            throw std::logic_error(
                "static HGAMP bootstrap has no unrestricted population");
        }
        const auto derived = deriveBootstrapConfigurationCandidates();
        const auto ranked = profileAwareRankConfigurations(derived);
        hgasp::ConfigurationScreeningResult screening =
            progressivelyEvaluateConfigurations(ranked);
        if (screening.selectedConfigurations.empty()) {
            data->populationMode =
                hgasp::ConfigurationPopulationMode::FreeOnly;
            data->numLocConfiguration = 0;
            data->compactLocConfigure.clear();
            hgasp::RunLogger::event(
                hgasp::LogLevel::warning,
                "search",
                "configuration_screening_failed",
                {},
                {{"reason", "bootstrap_candidates_failed_evaluation"}});
            return;
        }

        SubPopulation unrestricted =
            std::move(popman->popula.front());
        publishStaticConfigurations(
            screening.selectedConfigurations);
        popman->popula.clear();
        popman->popula.resize(
            hgasp::staticHgampRequiredPopulationSlotCount(
                screening.selectedConfigurations.size()));
        popman->popula[
            screening.selectedConfigurations.size()] =
                std::move(unrestricted);
        initialPop(pool, &screening);
        hgasp::RunLogger::event(
            hgasp::LogLevel::info,
            "search",
            "configuration_screening_finished",
            {},
            {
                {"requested_count", data->staticFixedPopulationCount},
                {"actual_count", data->numLocConfiguration},
                {"source", "unrestricted_bootstrap"},
            });
    }

    const std::size_t requiredPopulationSlots =
        hgasp::staticHgampRequiredPopulationSlotCount(
            static_cast<std::size_t>(data->numLocConfiguration));
    if (popman->popula.size() < requiredPopulationSlots) {
        popman->popula.resize(requiredPopulationSlots);
    }
    initializeStaticConfigurationRegistry();
    for (const auto& token :
         staticConfigurationRegistry->activeTokens()) {
        registerPopulationConfiguration(static_cast<int>(token.slot));
    }
    const std::size_t freeIndex =
        staticConfigurationRegistry->unrestrictedPopulationIndex();
    if (freeIndex >= popman->popula.size() ||
        popman->popula[freeIndex].empty()) {
        throw std::logic_error(
            "static HGAMP unrestricted population is absent or empty");
    }
    (void)refreshPermanentFreePopulationFloor();
    if (data->singleDynamicChallengerEffective()) {
        const std::size_t challengerIndex = freeIndex + 1U;
        if (challengerIndex >= popman->popula.size()) {
            throw std::logic_error(
                "single challenger slot is outside population storage");
        }
        if (!popman->popula[challengerIndex].empty() &&
            !challengerRuntime.active()) {
            throw std::logic_error(
                "inactive challenger slot retained population members");
        }
    }
    // The permanent free population already owns these individuals. Observe
    // it before importing independently validated CRH/ST-CRH or fixed-seed-LS
    // snapshots so one seed is not falsely attributed to two initialization
    // sources.
    observeInitializedFreePopulation(false);
    drainPendingConfigurationDiscoveries(pool);
    if (!staticPopulationStartEmitted) {
        hgasp::RunLogger::event(
            hgasp::LogLevel::info,
            "population",
            "static_hgamp_population_started",
            {},
            {
                {"iteration", data->iteration},
                {"population_mode", hgasp::toString(data->populationMode)},
                {"requested_fixed_populations",
                    data->staticFixedPopulationCount},
                {"actual_fixed_populations",
                    static_cast<unsigned long long>(
                        staticConfigurationRegistry
                            ->actualFixedPopulationCount())},
                {"free_population_effective", true},
                {"configuration_ranker",
                    hgasp::toString(data->configurationRanker)},
                {"classic_crh_effective", true},
                {"st_crh_effective", data->hasHardTimeWindows()},
                {"strict_cross_configuration_mating_effective",
                    data->strictCrossConfigurationMating},
                {"population_local_sp_configured",
                    data->populationLocalSpEnabled},
                {"pre_reduction_global_sp_configured",
                    data->preReductionGlobalSpEnabled},
                {"adaptive_population_allocation_effective",
                    data->adaptivePopulationAllocationEffective()},
                {"local_search_mode",
                    hgasp::routeLocalSearchComponentName(
                        data->hasHardTimeWindows())},
            });
        staticPopulationStartEmitted = true;
    }
}

bool RouteGenerator::staticPopulationsMaterialized() const noexcept {
    return staticConfigurationRegistry != nullptr;
}

int RouteGenerator::challengerPopulationIndex() const noexcept {
    if (staticConfigurationRegistry == nullptr) {
        return -1;
    }
    const std::size_t index =
        staticConfigurationRegistry->unrestrictedPopulationIndex() + 1U;
    return index < popman->popula.size()
        ? static_cast<int>(index)
        : -1;
}

bool RouteGenerator::challengerActive() const noexcept {
    const int index = challengerPopulationIndex();
    return data->singleDynamicChallengerEffective() &&
        challengerRuntime.active() && index >= 0 &&
        !popman->popula[static_cast<std::size_t>(index)].empty();
}

PopulationRole RouteGenerator::populationRole(
        const int populationIndex) const noexcept {
    if (populationIndex < 0 ||
        staticConfigurationRegistry == nullptr) {
        return PopulationRole::InactiveOrSpare;
    }
    const std::size_t index =
        static_cast<std::size_t>(populationIndex);
    if (index < staticConfigurationRegistry->slots().size()) {
        return staticConfigurationRegistry->slots()[index].active
            ? PopulationRole::FixedResident
            : PopulationRole::InactiveOrSpare;
    }
    if (index ==
        staticConfigurationRegistry->unrestrictedPopulationIndex()) {
        return PopulationRole::UnrestrictedFree;
    }
    if (populationIndex == challengerPopulationIndex() &&
        challengerRuntime.active()) {
        return PopulationRole::DynamicChallenger;
    }
    return PopulationRole::InactiveOrSpare;
}

hgasp::PopulationAdmissionTarget
RouteGenerator::admissionTargetForConfiguration(
        const hgasp::DepotConfigurationKey& configuration) const {
    if (staticConfigurationRegistry == nullptr) {
        throw std::logic_error(
            "static population admission requires a registry");
    }
    const hgasp::DepotConfigurationKey canonical =
        hgasp::makeDepotConfigurationKey(configuration.openDepots);
    if (canonical != configuration || canonical.openDepots.empty()) {
        throw std::invalid_argument(
            "static population admission configuration is not canonical");
    }
    const hgasp::StaticPopulationTarget classified =
        staticConfigurationRegistry->classify(canonical);
    if (classified.fixedToken.has_value()) {
        return {
            hgasp::PopulationAdmissionRole::FixedResident,
            static_cast<int>(classified.populationIndex),
            classified.fixedToken->generation,
            canonical,
        };
    }
    if (challengerRuntime.active() &&
        challengerRuntime.configuration == canonical) {
        const int populationIndex = challengerPopulationIndex();
        if (populationIndex < 0 ||
            challengerRuntime.token.generation == 0U) {
            throw std::logic_error(
                "active challenger has no admission identity");
        }
        return {
            hgasp::PopulationAdmissionRole::DynamicChallenger,
            populationIndex,
            challengerRuntime.token.generation,
            canonical,
        };
    }
    const std::size_t freeIndex =
        staticConfigurationRegistry->unrestrictedPopulationIndex();
    if (freeIndex >= staticPopulationRuntime.size() ||
        staticPopulationRuntime[freeIndex].generation == 0U) {
        throw std::logic_error(
            "permanent free population has no admission generation");
    }
    return {
        hgasp::PopulationAdmissionRole::UnrestrictedFree,
        static_cast<int>(freeIndex),
        staticPopulationRuntime[freeIndex].generation,
        std::nullopt,
    };
}

int RouteGenerator::addIndividualToStaticTarget(
        Individual& candidate,
        const hgasp::PopulationAdmissionTarget& target,
        const bool isInitial,
        RoutePool* const pool,
        const hgasp::RouteSubmissionContext* const submission,
        bool* const admitted,
        std::uint64_t* const survivingIndividualId,
        const bool allowIncumbentUpdate,
        const Individual::PopulationValidationReceipt*
            const validation) {
    if (staticConfigurationRegistry == nullptr) {
        throw std::logic_error(
            "typed static admission requires a registry");
    }
    candidate.decideFeasible();
    if (!candidate.isFeasible) {
        if (admitted != nullptr) {
            *admitted = false;
        }
        return -1;
    }
    const hgasp::DepotConfigurationKey actual =
        configurationKey(candidate);
    switch (target.role) {
    case hgasp::PopulationAdmissionRole::FixedResident: {
        if (target.populationIndex < 0) {
            throw std::logic_error(
                "fixed admission has a negative population index");
        }
        const hgasp::StaticPopulationToken token{
            static_cast<std::size_t>(target.populationIndex),
            target.generation,
        };
        if (!target.expectedConfiguration.has_value() ||
            !staticConfigurationRegistry->isCurrent(token)) {
            throw std::logic_error(
                "fixed admission token is stale or incomplete");
        }
        const auto& state = staticConfigurationRegistry->state(token);
        if (*target.expectedConfiguration != state.configuration) {
            throw std::logic_error(
                "fixed admission expected configuration is stale");
        }
        staticConfigurationRegistry->requireMatchingConfiguration(
            token, actual);
        break;
    }
    case hgasp::PopulationAdmissionRole::UnrestrictedFree: {
        const std::size_t freeIndex =
            staticConfigurationRegistry->unrestrictedPopulationIndex();
        if (target.populationIndex != static_cast<int>(freeIndex) ||
            freeIndex >= staticPopulationRuntime.size() ||
            target.generation !=
                staticPopulationRuntime[freeIndex].generation ||
            target.expectedConfiguration.has_value()) {
            throw std::logic_error(
                "permanent free admission identity is stale");
        }
        break;
    }
    case hgasp::PopulationAdmissionRole::DynamicChallenger:
        if (!challengerRuntime.active() ||
            target.populationIndex != challengerPopulationIndex() ||
            target.generation != challengerRuntime.token.generation ||
            !target.expectedConfiguration.has_value() ||
            *target.expectedConfiguration !=
                challengerRuntime.configuration ||
            actual != challengerRuntime.configuration) {
            throw std::logic_error(
                "challenger admission identity is stale or mismatched");
        }
        break;
    case hgasp::PopulationAdmissionRole::TransactionStaging: {
        const std::size_t stagingIndex =
            hgasp::staticHgampPromotionStagingPopulationIndex(
                staticConfigurationRegistry
                    ->unrestrictedPopulationIndex());
        if (target.populationIndex != static_cast<int>(stagingIndex) ||
            target.generation == 0U ||
            !target.expectedConfiguration.has_value() ||
            actual != *target.expectedConfiguration) {
            throw std::logic_error(
                "promotion staging admission identity is mismatched");
        }
        break;
    }
    }
    return popman->addIndividualToTarget(
        &candidate,
        target,
        isInitial,
        pool,
        submission,
        admitted,
        survivingIndividualId,
        allowIncumbentUpdate,
        validation);
}

hgasp::ParentLineageSnapshot RouteGenerator::captureParentLineage(
        const Individual& parent,
        const int populationIndex) const {
    if (populationIndex < 0 ||
        static_cast<std::size_t>(populationIndex) >=
            popman->popula.size()) {
        throw std::out_of_range(
            "parent lineage population index is out of range");
    }
    hgasp::ParentLineageSnapshot snapshot;
    snapshot.individualId = parent.searchIndividualId;
    snapshot.populationIndex = populationIndex;
    snapshot.actualConfiguration = configurationKey(parent);
    snapshot.spAncestryCallId = parent.spAncestryCallId;
    PopulationRole role = populationRole(populationIndex);
    if (staticConfigurationRegistry == nullptr &&
        data->staticHgampPopulationsEnabled()) {
        if (populationIndex < data->numLocConfiguration) {
            role = PopulationRole::FixedResident;
        } else if (populationIndex == data->numLocConfiguration) {
            role = PopulationRole::UnrestrictedFree;
        }
    }
    switch (role) {
    case PopulationRole::FixedResident:
        snapshot.populationRole =
            hgasp::RoutePopulationRole::FixedResident;
        if (staticConfigurationRegistry != nullptr) {
            snapshot.populationGeneration =
                staticConfigurationRegistry->slots()[
                    static_cast<std::size_t>(populationIndex)]
                    .generation;
        } else {
            snapshot.populationGeneration = 1U;
        }
        break;
    case PopulationRole::UnrestrictedFree:
        snapshot.populationRole =
            hgasp::RoutePopulationRole::UnrestrictedFree;
        if (static_cast<std::size_t>(populationIndex) <
            staticPopulationRuntime.size()) {
            snapshot.populationGeneration =
                staticPopulationRuntime[
                    static_cast<std::size_t>(populationIndex)]
                    .generation;
        } else if (staticConfigurationRegistry == nullptr) {
            snapshot.populationGeneration = 1U;
        }
        break;
    case PopulationRole::DynamicChallenger:
        snapshot.populationRole =
            hgasp::RoutePopulationRole::DynamicChallenger;
        snapshot.populationGeneration =
            challengerRuntime.token.generation;
        snapshot.challengerSlot = populationIndex;
        snapshot.challengerGeneration =
            challengerRuntime.token.generation;
        break;
    case PopulationRole::InactiveOrSpare:
        snapshot.populationRole =
            hgasp::RoutePopulationRole::InactiveOrSpare;
        break;
    }
    return snapshot;
}

bool RouteGenerator::isCurrentParentLineage(
        const hgasp::ParentLineageSnapshot& parent) const noexcept {
    if (!parent.valid() ||
        static_cast<std::size_t>(parent.populationIndex) >=
            popman->popula.size()) {
        return false;
    }
    if (staticConfigurationRegistry == nullptr) {
        return true;
    }
    const std::size_t index =
        static_cast<std::size_t>(parent.populationIndex);
    switch (parent.populationRole) {
    case hgasp::RoutePopulationRole::FixedResident:
        return index < staticConfigurationRegistry->slots().size() &&
            staticConfigurationRegistry->slots()[index].active &&
            staticConfigurationRegistry->slots()[index].generation ==
                parent.populationGeneration &&
            staticConfigurationRegistry->slots()[index].configuration ==
                parent.actualConfiguration;
    case hgasp::RoutePopulationRole::UnrestrictedFree:
        return index == staticConfigurationRegistry
                ->unrestrictedPopulationIndex() &&
            index < staticPopulationRuntime.size() &&
            staticPopulationRuntime[index].generation ==
                parent.populationGeneration;
    case hgasp::RoutePopulationRole::DynamicChallenger:
        return challengerRuntime.active() &&
            parent.populationIndex == challengerPopulationIndex() &&
            challengerRuntime.token.generation ==
                parent.populationGeneration &&
            challengerRuntime.configuration ==
                parent.actualConfiguration;
    case hgasp::RoutePopulationRole::InactiveOrSpare:
    case hgasp::RoutePopulationRole::Unspecified:
        return false;
    }
    return false;
}

hgasp::ConfigurationCandidateEligibilityContext
RouteGenerator::challengerEligibilityContext() const {
    hgasp::ConfigurationCandidateEligibilityContext context;
    context.attempt = currentPhysicalAttempt();
    if (staticConfigurationRegistry == nullptr) {
        return context;
    }
    std::vector<hgasp::ChallengerResidentCandidate> residents;
    for (const auto& slot : staticConfigurationRegistry->slots()) {
        if (!slot.active || slot.slot >= popman->popula.size() ||
            popman->popula[slot.slot].empty()) {
            continue;
        }
        context.activeResidents.push_back(slot.configuration);
        hgasp::ChallengerResidentCandidate resident;
        resident.populationIndex = slot.slot;
        resident.active = true;
        resident.protectedResident =
            context.attempt < slot.protectedUntilAttempt;
        resident.minimumAgeSatisfied =
            context.attempt >= slot.createdAttempt &&
            context.attempt - slot.createdAttempt >=
                static_cast<std::uint64_t>(
                    data->staticPopulationMinimumAgeAttempts);
        resident.objectives = populationObjectives(
            static_cast<int>(slot.slot));
        residents.push_back(std::move(resident));
    }
    context.weakestReplaceableResident =
        challengerPolicy.weakestReplaceableResident(residents);
    if (hgasp::isSetSProductionSearchProfile(data->parameters.profile) ||
        hgasp::isSetFProductionSearchProfile(data->parameters.profile)) {
        std::vector<hgasp::ChallengerResidentCandidate> qualityResidents =
            residents;
        for (auto& resident : qualityResidents) {
            resident.protectedResident = false;
            resident.minimumAgeSatisfied = true;
        }
        context.qualityReferenceResident =
            challengerPolicy.weakestReplaceableResident(qualityResidents);
    }
    if (challengerRuntime.active()) {
        context.currentChallenger =
            challengerRuntime.configuration;
    }
    return context;
}

std::string RouteGenerator::makeConfigurationEvidenceUnitId(
        const char* const kind) {
    if (kind == nullptr || *kind == '\0') {
        throw std::invalid_argument(
            "configuration evidence kind is empty");
    }
    if (nextConfigurationEvidenceUnitSequence ==
        std::numeric_limits<std::uint64_t>::max()) {
        throw std::overflow_error(
            "configuration evidence identifier exhausted");
    }
    const std::uint64_t sequence =
        nextConfigurationEvidenceUnitSequence++;
    return std::string(kind) + ":attempt:" +
        std::to_string(currentPhysicalAttempt()) +
        ":unit:" + std::to_string(sequence);
}

void RouteGenerator::initializeChildConfigurationDiscovery(
        const int childIndex,
        const hgasp::ConfigurationDiscoverySource source,
        const hgasp::DepotConfigurationKey& before,
        const hgasp::DepotConfigurationKey& after) {
    if (childIndex < 0 ||
        static_cast<std::size_t>(childIndex) >=
            childConfigurationDiscovery.size()) {
        throw std::out_of_range(
            "configuration discovery child index is out of range");
    }
    hgasp::ConfigurationDiscoveryTrace trace;
    trace.beforeConfiguration =
        hgasp::makeDepotConfigurationKey(before.openDepots);
    trace.afterConfiguration =
        hgasp::makeDepotConfigurationKey(after.openDepots);
    trace.source = trace.beforeConfiguration != trace.afterConfiguration
        ? source
        : hgasp::ConfigurationDiscoverySource::Count;
    if (lastOffspringBatchId == 0U) {
        throw std::logic_error(
            "offspring discovery has no frozen batch identity");
    }
    trace.evidenceUnitId = "hga_batch:" +
        std::to_string(lastOffspringBatchId) + ":child:" +
        std::to_string(childIndex);
    if (!lastBatchParentLineage.first.spAncestryCallId.empty()) {
        trace.routeAncestry =
            lastBatchParentLineage.first.spAncestryCallId;
    } else {
        trace.routeAncestry =
            lastBatchParentLineage.second.spAncestryCallId;
    }
    trace.initialized = true;
    if (trace.beforeConfiguration != trace.afterConfiguration) {
        trace.contributors |=
            hgasp::configurationDiscoverySourceBit(source);
    }
    childConfigurationDiscovery[static_cast<std::size_t>(childIndex)] =
        std::move(trace);
}

void RouteGenerator::recordChildConfigurationStage(
        const int childIndex,
        const hgasp::ConfigurationDiscoverySource source,
        const hgasp::DepotConfigurationKey& before,
        const hgasp::DepotConfigurationKey& after) {
    if (childIndex < 0 ||
        static_cast<std::size_t>(childIndex) >=
            childConfigurationDiscovery.size()) {
        throw std::out_of_range(
            "configuration discovery child index is out of range");
    }
    hgasp::ConfigurationDiscoveryTrace& trace =
        childConfigurationDiscovery[static_cast<std::size_t>(childIndex)];
    if (!trace.initialized) {
        initializeChildConfigurationDiscovery(
            childIndex, source, before, after);
        return;
    }
    const hgasp::DepotConfigurationKey canonicalBefore =
        hgasp::makeDepotConfigurationKey(before.openDepots);
    const hgasp::DepotConfigurationKey canonicalAfter =
        hgasp::makeDepotConfigurationKey(after.openDepots);
    if (canonicalBefore != canonicalAfter) {
        trace.source = source;
        trace.contributors |=
            hgasp::configurationDiscoverySourceBit(source);
    }
    trace.afterConfiguration = canonicalAfter;
}

void RouteGenerator::observeConfigurationDiscovery(
        const Individual& candidate,
        const bool admittedToFree,
        const hgasp::ConfigurationDiscoveryTrace& suppliedTrace,
        const PopulationRole admissionRole) {
    if (!candidate.isFeasible) {
        return;
    }
    const hgasp::DepotConfigurationKey configuration =
        configurationKey(candidate);
    hgasp::ConfigurationDiscoveryTrace trace = suppliedTrace;
    if (!trace.initialized) {
        trace.beforeConfiguration = configuration;
        trace.afterConfiguration = configuration;
        trace.source = hgasp::ConfigurationDiscoverySource::Count;
        trace.initialized = true;
    }
    if (trace.evidenceUnitId.empty()) {
        trace.evidenceUnitId =
            makeConfigurationEvidenceUnitId("legacy_discovery");
    }
    trace.afterConfiguration = configuration;
    trace.independentlyValidated =
        trace.independentlyValidated && candidate.isFeasible;
    const hgasp::ConfigurationChangeKind changeKind =
        hgasp::classifyConfigurationChange(
            trace.beforeConfiguration, configuration);
    ++configurationChangesByKind[
        static_cast<std::size_t>(changeKind)];
    const bool configurationChanged =
        changeKind != hgasp::ConfigurationChangeKind::Unchanged;
    if (!configurationChanged) {
        trace.source = hgasp::ConfigurationDiscoverySource::Count;
        trace.contributors = 0U;
    } else {
        const std::size_t sourceIndex =
            static_cast<std::size_t>(trace.source);
        if (sourceIndex >= configurationDiscoveriesBySource.size()) {
            throw std::logic_error(
                "configuration discovery source is invalid");
        }
        ++configurationDiscoveriesBySource[sourceIndex];
        if (admittedToFree) {
            ++configurationFreeAdmissionsBySource[sourceIndex];
        }
    }

    const bool archiveEligible =
        configurationChanged &&
        data->singleDynamicChallengerEffective() &&
        staticConfigurationRegistry != nullptr &&
        !staticConfigurationRegistry
             ->tokenForConfiguration(configuration)
             .has_value() &&
        !(challengerRuntime.active() &&
          configuration == challengerRuntime.configuration);
    const auto context = challengerEligibilityContext();
    const auto* beforeRecord = archiveEligible
        ? candidateArchive.find(configuration)
        : nullptr;
    const auto beforeDecision = beforeRecord == nullptr
        ? hgasp::ConfigurationCandidateEligibilityDecision{}
        : candidateArchive.evaluateEligibility(*beforeRecord, context);
    hgasp::ConfigurationCandidateObservationResult result;
    if (archiveEligible) {
        hgasp::ConfigurationCandidateObservation observation;
        observation.configuration = configuration;
        observation.attempt = context.attempt;
        observation.objective = candidate.fit;
        observation.admittedToFree = admittedToFree;
        observation.source = trace.source;
        observation.contributorMask = trace.contributors;
        observation.evidenceUnitId = trace.evidenceUnitId;
        observation.routeAncestry = trace.routeAncestry;
        observation.beforeConfiguration = trace.beforeConfiguration;
        observation.independentlyValidated =
            trace.independentlyValidated;
        observation.strictGlobalIncumbent =
            trace.strictGlobalIncumbent;
        result = candidateArchive.observe(observation, context);
        // The archive first applies evidence de-duplication, bounded-record
        // eviction and metadata updates. Only an observation that has a live
        // working record pays for the route-heavy immutable seed snapshot.
        if (trace.independentlyValidated &&
            !result.duplicateEvidenceUnit &&
            !result.capacityRejected) {
            const auto* retainedRecord =
                candidateArchive.find(configuration);
            if (retainedRecord == nullptr) {
                throw std::logic_error(
                    "accepted candidate metadata has no working record");
            }
            hgasp::ScreenedConfigurationSeed seed;
            seed.configuration = configuration;
            seed.routes = snapshotValidatedIndividual(candidate);
            seed.objective = candidate.fit;
            const ConfigurationEvaluationStructure candidateStructure =
                configurationEvaluationStructure(seed.routes, *data);
            double diversitySum = 0.0;
            for (const auto& retained : retainedRecord->diverseSeeds) {
                diversitySum += configurationStructureDistance(
                    candidateStructure,
                    configurationEvaluationStructure(
                        retained.routes, *data));
            }
            seed.diversity = retainedRecord->diverseSeeds.empty()
                ? 0.0
                : diversitySum / static_cast<double>(
                      retainedRecord->diverseSeeds.size());
            seed.sourceIndex = static_cast<std::size_t>(context.attempt);
            result.seedRetained = candidateArchive.retainValidatedSeed(
                std::move(seed));
        }
    }
    if (result.recordCreated) {
        ++candidateRecordsCreated;
    }
    if (result.recordRecreated) {
        ++candidateRecordsRecreated;
    }
    if (result.recordEvicted) {
        ++candidateRecordsEvicted;
    }
    if (result.capacityRejected) {
        ++candidateRecordsCapacityRejected;
    }
    if (result.observationHistoryCreated) {
        ++uniqueNonresidentConfigurationsSeen;
    }
    const auto* afterRecord = archiveEligible
        ? candidateArchive.find(configuration)
        : nullptr;
    const auto afterDecision = afterRecord == nullptr
        ? hgasp::ConfigurationCandidateEligibilityDecision{}
        : candidateArchive.evaluateEligibility(*afterRecord, context);
    if (!beforeDecision.eligible && afterDecision.eligible) {
        ++candidateRecordsEligible;
        hgasp::RunLogger::event(
            hgasp::LogLevel::info,
            "population",
            "challenger_candidate_became_eligible",
            {},
            {
                {"iteration", data->iteration},
                {"configuration", configurationLabel(configuration)},
                {"observations", static_cast<unsigned long long>(
                    afterRecord->totalObservationCount)},
                {"best_objective", candidateArchive
                    .findObservationHistory(configuration)
                    ->bestObjective},
                {"reason", hgasp::toString(afterDecision.reason)},
            });
    }

    const auto roleName = [](const PopulationRole role) {
        switch (role) {
        case PopulationRole::FixedResident:
            return "fixed_resident";
        case PopulationRole::UnrestrictedFree:
            return "permanent_free";
        case PopulationRole::DynamicChallenger:
            return "dynamic_challenger";
        case PopulationRole::InactiveOrSpare:
            return "inactive_or_spare";
        }
        return "unknown";
    };
    std::ostringstream contributors;
    for (std::size_t index = 0U;
         index < hgasp::configurationDiscoverySourceCount();
         ++index) {
        const auto source = static_cast<
            hgasp::ConfigurationDiscoverySource>(index);
        if ((trace.contributors &
             hgasp::configurationDiscoverySourceBit(source)) == 0U) {
            continue;
        }
        if (contributors.tellp() > 0) {
            contributors << ',';
        }
        contributors << hgasp::toString(source);
    }
    hgasp::RunLogger::event(
        result.observationHistoryCreated || trace.strictGlobalIncumbent
            ? hgasp::LogLevel::info
            : hgasp::LogLevel::debug,
        "configuration_discovery",
        "configuration_discovery_observed",
        {},
        {
            {"iteration", data->iteration},
            {"source", hgasp::toString(trace.source)},
            {"evidence_unit_id", trace.evidenceUnitId},
            {"route_ancestry", trace.routeAncestry},
            {"contributors", contributors.str()},
            {"before_configuration",
                configurationLabel(trace.beforeConfiguration)},
            {"after_configuration", configurationLabel(configuration)},
            {"change_kind", hgasp::toString(changeKind)},
            {"objective", candidate.fit},
            {"independently_validated",
                trace.independentlyValidated},
            {"feasible", candidate.isFeasible},
            {"strict_global", trace.strictGlobalIncumbent},
            {"admission_targeted", admissionRole !=
                PopulationRole::InactiveOrSpare},
            {"admitted_to_free", admittedToFree},
            {"admission_role", roleName(admissionRole)},
        });
}

RouteGenerator::ConfigurationAdmissionResult
RouteGenerator::admitConfigurationDiscovery(
        Individual& candidate,
        RoutePool* const pool,
        const hgasp::RouteSubmissionContext* const submission,
        const hgasp::ConfigurationDiscoveryTrace& trace,
        const bool isInitial,
        const Individual::PopulationValidationReceipt*
            const validation) {
    ConfigurationAdmissionResult result;
    if (staticConfigurationRegistry == nullptr) {
        return result;
    }
    candidate.decideFeasible();
    if (!candidate.isFeasible) {
        return result;
    }
    const hgasp::DepotConfigurationKey actual =
        configurationKey(candidate);
    const hgasp::PopulationAdmissionTarget target =
        admissionTargetForConfiguration(actual);
    result.populationIndex = target.populationIndex;
    switch (target.role) {
    case hgasp::PopulationAdmissionRole::FixedResident:
        result.role = PopulationRole::FixedResident;
        break;
    case hgasp::PopulationAdmissionRole::UnrestrictedFree:
        result.role = PopulationRole::UnrestrictedFree;
        break;
    case hgasp::PopulationAdmissionRole::DynamicChallenger:
        result.role = PopulationRole::DynamicChallenger;
        break;
    case hgasp::PopulationAdmissionRole::TransactionStaging:
        throw std::logic_error(
            "configuration discovery cannot target transaction staging");
    }

    bool admitted = false;
    const int replacement = addIndividualToStaticTarget(
        candidate,
        target,
        isInitial,
        pool,
        submission,
        &admitted,
        nullptr,
        true,
        validation);
    if (replacement >= 0) {
        throw std::logic_error(
            "configuration discovery attempted an implicit resident "
            "replacement");
    }
    result.admitted = admitted;
    if (result.role == PopulationRole::DynamicChallenger) {
        SubPopulation& population = popman->popula[
            static_cast<std::size_t>(result.populationIndex)];
        while (population.size() >
               static_cast<std::size_t>(std::max(1, data->popMin))) {
            popman->removeWorstBiasedFitness(population);
        }
    }
    if (result.populationIndex >= 0 &&
        static_cast<std::size_t>(result.populationIndex) <
            popman->popula.size() &&
        !popman->popula[static_cast<std::size_t>(
            result.populationIndex)].empty()) {
        getAverageValue(result.populationIndex);
    }
    hgasp::ConfigurationDiscoveryTrace observedTrace = trace;
    if (submission != nullptr) {
        if (observedTrace.evidenceUnitId.empty()) {
            observedTrace.evidenceUnitId =
                submission->configurationEvidenceUnitId;
        }
        if (observedTrace.routeAncestry.empty()) {
            observedTrace.routeAncestry = submission->spAncestryCallId;
        }
    }
    observeConfigurationDiscovery(
        candidate,
        admitted && result.role == PopulationRole::UnrestrictedFree,
        observedTrace,
        result.role);
    return result;
}

bool RouteGenerator::queuePendingConfigurationDiscovery(
        hgasp::ScreenedConfigurationSeed seed,
        const hgasp::ConfigurationDiscoveryTrace& suppliedTrace) {
    seed.configuration = hgasp::makeDepotConfigurationKey(
        seed.configuration.openDepots);
    if (seed.configuration.openDepots.empty() || seed.routes.empty() ||
        !std::isfinite(seed.objective) ||
        !std::isfinite(seed.diversity) || seed.diversity < 0.0) {
        throw std::invalid_argument(
            "pending configuration discovery seed is invalid");
    }
    hgasp::ConfigurationDiscoveryTrace trace = suppliedTrace;
    trace.afterConfiguration = seed.configuration;
    trace.independentlyValidated = true;
    trace.initialized = true;
    auto found = pendingConfigurationDiscoveries.find(
        seed.configuration);
    if (found == pendingConfigurationDiscoveries.end()) {
        if (pendingConfigurationDiscoveries.size() >=
            candidateArchive.policy().maximumRecords) {
            auto eviction = pendingConfigurationDiscoveries.end();
            for (auto candidate =
                     pendingConfigurationDiscoveries.begin();
                 candidate != pendingConfigurationDiscoveries.end();
                 ++candidate) {
                if (eviction == pendingConfigurationDiscoveries.end() ||
                    hgasp::screenedConfigurationSeedHasHigherPriority(
                        eviction->second.front().seed,
                        candidate->second.front().seed)) {
                    eviction = candidate;
                }
            }
            if (eviction == pendingConfigurationDiscoveries.end() ||
                !hgasp::screenedConfigurationSeedHasHigherPriority(
                    seed, eviction->second.front().seed)) {
                return false;
            }
            pendingConfigurationDiscoveries.erase(eviction);
        }
        found = pendingConfigurationDiscoveries.emplace(
            seed.configuration,
            std::vector<PendingConfigurationDiscovery>{}).first;
    }
    auto& candidates = found->second;
    candidates.push_back({std::move(seed), std::move(trace)});
    std::stable_sort(
        candidates.begin(), candidates.end(),
        [](const PendingConfigurationDiscovery& left,
           const PendingConfigurationDiscovery& right) {
            return hgasp::screenedConfigurationSeedHasHigherPriority(
                left.seed, right.seed);
        });
    if (candidates.size() > candidateArchive.policy().seedLimit) {
        candidates.resize(candidateArchive.policy().seedLimit);
    }
    return true;
}

void RouteGenerator::observeInitializedFreePopulation(
        const bool registryAlreadyMaterialized) {
    if (staticConfigurationRegistry == nullptr) {
        return;
    }
    if (!initialFreePopulationObservationGate.claimBulkObservation(
            registryAlreadyMaterialized)) {
        return;
    }
    const std::size_t freeIndex =
        staticConfigurationRegistry->unrestrictedPopulationIndex();
    if (freeIndex >= popman->popula.size()) {
        throw std::logic_error(
            "permanent free population index is unavailable");
    }
    const hgasp::DepotConfigurationKey emptyConfiguration =
        hgasp::makeDepotConfigurationKey(std::vector<int>{});
    for (Individual* candidate : popman->popula[freeIndex]) {
        if (candidate == nullptr) {
            continue;
        }
        candidate->decideFeasible();
        if (!candidate->isFeasible) {
            throw std::logic_error(
                "free initialization contains an infeasible resident");
        }
        hgasp::ConfigurationDiscoveryTrace trace;
        trace.beforeConfiguration = emptyConfiguration;
        trace.afterConfiguration = configurationKey(*candidate);
        trace.source =
            hgasp::ConfigurationDiscoverySource::FreeInitialization;
        trace.contributors = hgasp::configurationDiscoverySourceBit(
            trace.source);
        const auto pendingDiscovery =
            pendingBootstrapFreeDiscoveries.find(
                candidate->searchIndividualId);
        if (pendingDiscovery !=
            pendingBootstrapFreeDiscoveries.end()) {
            trace = pendingDiscovery->second;
            trace.afterConfiguration = configurationKey(*candidate);
        }
        trace.independentlyValidated = true;
        if (trace.evidenceUnitId.empty()) {
            trace.evidenceUnitId =
                "free_initialization:individual:" +
                std::to_string(candidate->searchIndividualId);
        }
        trace.initialized = true;
        observeConfigurationDiscovery(
            *candidate, true, trace,
            PopulationRole::UnrestrictedFree);
    }
    pendingBootstrapFreeDiscoveries.clear();
}

void RouteGenerator::drainPendingConfigurationDiscoveries(
        RoutePool* const pool) {
    if (staticConfigurationRegistry == nullptr ||
        pendingConfigurationDiscoveries.empty()) {
        return;
    }
    auto pending = std::move(pendingConfigurationDiscoveries);
    pendingConfigurationDiscoveries.clear();
    for (auto& entry : pending) {
        for (PendingConfigurationDiscovery& discovery : entry.second) {
            temIndiv->rebuildFromSP(discovery.seed.routes);
            temIndiv->decideFeasible();
            if (!temIndiv->isFeasible ||
                configurationKey(*temIndiv) !=
                    discovery.seed.configuration ||
                !data->equalTwoVar(
                    temIndiv->fit, discovery.seed.objective)) {
                throw std::logic_error(
                    "pending configuration discovery failed independent "
                    "reconstruction");
            }
            discovery.trace.independentlyValidated = true;
            discovery.trace.afterConfiguration =
                discovery.seed.configuration;
            admitConfigurationDiscovery(
                *temIndiv, pool, nullptr, discovery.trace, true);
        }
    }
}

void RouteGenerator::refreshStaticRuntime(
        const int populationIndex) {
    if (populationIndex < 0 ||
        static_cast<std::size_t>(populationIndex) >=
            staticPopulationRuntime.size() ||
        static_cast<std::size_t>(populationIndex) >=
            popman->popula.size() ||
        popman->popula[static_cast<std::size_t>(populationIndex)].empty()) {
        return;
    }
    StaticPopulationRuntimeState& state =
        staticPopulationRuntime[static_cast<std::size_t>(populationIndex)];
    const double best =
        popman->popula[static_cast<std::size_t>(populationIndex)]
            .front()->fit;
    if (state.bestObjective == Max) {
        state.bestObjective = best;
        state.lastBestImprovementAttempt = state.createdAttempt;
    } else if (best + Min < state.bestObjective) {
        state.bestObjective = best;
        state.lastBestImprovementAttempt =
            static_cast<std::uint64_t>(std::max(0, data->iteration));
    }
}

void RouteGenerator::getAverageValue(int rank){
    if (rank < 0){
        avgCostPop.clear();
        for (int i=0;i<data->numLocConfiguration + 1; i++){
            double cost = 0;
            for (int j=0;j<(int)popman->popula[i].size();j++){
                cost += popman->popula[i][j]->fit;
            }
            int ss = popman->popula[i].size();
            avgCostPop.push_back(
                ss == 0 ? Max : cost / static_cast<double>(ss));
        }

        // Dynamic TW islands own their fixed slots and lifecycle.  The
        // legacy initialization-time "remove half" step would silently
        // destroy manager state and invalidate generation tokens.
        if (data->staticHgampPopulationsEnabled()) {
            return;
        }

        // removing half population
        std::vector<std::pair<double,int>>rankOrder;
        for (int i=0;i<(int)avgCostPop.size();i++)
            rankOrder.push_back({-avgCostPop[i],i});
        sort(rankOrder.begin(),rankOrder.end());
        for (int i=0;i<data->numLocConfiguration / 2; i++){
            int subpop = rankOrder[i].second;
            for (Individual * indiv : popman->popula[subpop]) delete indiv ;
            popman->popula[subpop].clear();
        }

        // Population identity is the canonical depot set.  The historical
        // shift/modular cache was collision-prone and undefined for depot
        // indices outside the integer width, so it is no longer populated.
        data->dimTabu1.clear();
        data->dimTabu2.clear();
    }
    else{
        if (rank < 0 ||
            static_cast<std::size_t>(rank) >=
                popman->popula.size()) {
            throw std::out_of_range(
                "population mean index is out of range");
        }
        if (avgCostPop.size() < popman->popula.size()) {
            avgCostPop.resize(popman->popula.size(), Max);
        }
        double cost = 0;
        for (int j=0;j<(int)popman->popula[rank].size();j++)
            cost += popman->popula[rank][j]->fit;
        int ss = popman->popula[rank].size();
        avgCostPop[rank] =
            ss == 0 ? Max : cost / static_cast<double>(ss);
        if (data->staticHgampPopulationsEnabled() &&
            staticConfigurationRegistry != nullptr) {
            refreshStaticRuntime(rank);
        }
    }
}

int RouteGenerator::selectsubPop(const int subpop1) {
    std::vector<int> eligible;
    bool dedicatedConfigurationDiscoveryMate = false;
    const bool roleAwareProfile =
        hgasp::isSetSProductionSearchProfile(data->parameters.profile) ||
        hgasp::isSetFProductionSearchProfile(data->parameters.profile);
    bool roleAwarePrimaryRoleScheduled = false;

    if (roleAwareProfile && subpop1 < 0 &&
        staticConfigurationRegistry != nullptr) {
        roleAwarePrimaryRoleScheduled = true;
        ++roleAwarePrimaryMatingOpportunity;
        const std::uint64_t roleTurn =
            (roleAwarePrimaryMatingOpportunity - 1U) % 10U;
        const int freeIndex = static_cast<int>(
            staticConfigurationRegistry->unrestrictedPopulationIndex());
        const int challengerIndex = challengerPopulationIndex();
        if (roleTurn == 8U && freeIndex >= 0 &&
            static_cast<std::size_t>(freeIndex) < popman->popula.size() &&
            !popman->popula[static_cast<std::size_t>(freeIndex)].empty()) {
            currentMutationProbabilityPercent =
                data->mutationProbabilityPercent;
            return freeIndex;
        }
        if (roleTurn == 9U && challengerIndex >= 0 &&
            challengerRuntime.active() &&
            !popman->popula[static_cast<std::size_t>(challengerIndex)]
                 .empty()) {
            challengerScheduling.recordOpportunity();
            challengerScheduling.recordProtectedPrimarySelection();
            ++challengerRuntime.primarySelections;
            currentMutationProbabilityPercent =
                data->mutationProbabilityPercent;
            return challengerIndex;
        }
        if (challengerRuntime.active()) {
            challengerScheduling.recordOpportunity();
        }
        const auto& slots = staticConfigurationRegistry->slots();
        if (roleAwareResidentLastPrimaryOpportunity.size() < slots.size()) {
            roleAwareResidentLastPrimaryOpportunity.resize(slots.size(), 0U);
        }
        for (const auto& slot : slots) {
            if (slot.active && slot.slot < popman->popula.size() &&
                !popman->popula[slot.slot].empty()) {
                eligible.push_back(static_cast<int>(slot.slot));
            }
        }
        if (!eligible.empty()) {
            const std::uint64_t starvationBound =
                2U * static_cast<std::uint64_t>(eligible.size());
            const auto starving = std::min_element(
                eligible.begin(), eligible.end(),
                [&](const int left, const int right) {
                    return roleAwareResidentLastPrimaryOpportunity[
                               static_cast<std::size_t>(left)] <
                        roleAwareResidentLastPrimaryOpportunity[
                            static_cast<std::size_t>(right)];
                });
            if (starving != eligible.end() &&
                roleAwarePrimaryMatingOpportunity -
                        roleAwareResidentLastPrimaryOpportunity[
                            static_cast<std::size_t>(*starving)] >=
                    starvationBound) {
                roleAwareResidentLastPrimaryOpportunity[
                    static_cast<std::size_t>(*starving)] =
                    roleAwarePrimaryMatingOpportunity;
                currentMutationProbabilityPercent =
                    data->mutationProbabilityPercent;
                return *starving;
            }
        }
    }

    if (!(roleAwarePrimaryRoleScheduled && subpop1 < 0) &&
        data->singleDynamicChallengerEffective() &&
        challengerRuntime.state ==
            hgasp::ChallengerState::ProtectedTrial) {
        const int challengerIndex = challengerPopulationIndex();
        if (subpop1 < 0) {
            const std::uint64_t opportunity =
                challengerScheduling.recordOpportunity();
            if (challengerIndex >= 0 &&
                challengerPolicy.isProtectedPrimaryTurn(opportunity)) {
                challengerScheduling.recordProtectedPrimarySelection();
                ++challengerRuntime.primarySelections;
                currentMutationProbabilityPercent =
                    data->mutationProbabilityPercent;
                return challengerIndex;
            }
        } else if (subpop1 == challengerIndex) {
            dedicatedConfigurationDiscoveryMate = true;
            for (const auto& slot :
                 staticConfigurationRegistry->slots()) {
                if (slot.active &&
                    slot.configuration !=
                        challengerRuntime.configuration &&
                    slot.slot < popman->popula.size() &&
                    !popman->popula[slot.slot].empty()) {
                    eligible.push_back(static_cast<int>(slot.slot));
                }
            }
            const int freeIndex = static_cast<int>(
                staticConfigurationRegistry
                    ->unrestrictedPopulationIndex());
            if (static_cast<std::size_t>(freeIndex) <
                    popman->popula.size() &&
                !popman->popula[static_cast<std::size_t>(freeIndex)]
                     .empty()) {
                eligible.push_back(freeIndex);
            }
            if (eligible.empty()) {
                throw std::logic_error(
                    "protected challenger turn has no distinct resident "
                    "secondary population");
            }
        }
    }

    if (eligible.empty()) {
        const std::size_t limit =
            data->staticHgampPopulationsEnabled() &&
                    staticConfigurationRegistry != nullptr
                ? popman->popula.size()
                : std::min(
                      popman->popula.size(),
                      static_cast<std::size_t>(
                          data->numLocConfiguration + 1));
        for (std::size_t index = 0U; index < limit; ++index) {
            if (popman->popula[index].empty()) {
                continue;
            }
            if (data->staticHgampPopulationsEnabled() &&
                staticConfigurationRegistry != nullptr) {
                const bool unrestricted =
                    index == staticConfigurationRegistry
                        ->unrestrictedPopulationIndex();
                const bool challenger =
                    challengerActive() &&
                    static_cast<int>(index) ==
                        challengerPopulationIndex();
                const bool resident =
                    index <
                        staticConfigurationRegistry->slots().size() &&
                    staticConfigurationRegistry->slots()[index].active;
                if (!unrestricted && !challenger && !resident) {
                    continue;
                }
            }
            eligible.push_back(static_cast<int>(index));
        }
    }

    if (eligible.empty()) {
        return -1;
    }
    if (eligible.size() == 1U) {
        currentMutationProbabilityPercent =
            data->adaptivePopulationAllocationEffective()
                ? searchAllocator.mutationProbability(
                      data->mutationProbabilityPercent,
                      populationDiversity(eligible.front()),
                      data->maxIterNonProd > 0
                          ? static_cast<double>(
                                data->nbIterNonImpro) /
                                static_cast<double>(
                                    data->maxIterNonProd)
                          : 0.0,
                      spJustImproved)
                : data->mutationProbabilityPercent;
        if (roleAwarePrimaryRoleScheduled && subpop1 < 0) {
            roleAwareResidentLastPrimaryOpportunity[
                static_cast<std::size_t>(eligible.front())] =
                roleAwarePrimaryMatingOpportunity;
        }
        return eligible.front();
    }

    if (data->staticHgampPopulationsEnabled() &&
        staticConfigurationRegistry != nullptr) {
        std::vector<hgasp::StaticPopulationTournamentCandidate>
            candidates;
        candidates.reserve(eligible.size());
        for (const int populationIndex : eligible) {
            candidates.push_back({
                static_cast<std::size_t>(populationIndex),
                true,
                true,
                static_cast<std::size_t>(populationIndex) <
                        avgCostPop.size()
                    ? avgCostPop[
                          static_cast<std::size_t>(populationIndex)]
                    : Max,
            });
        }
        const std::optional<std::size_t> excluded =
            subpop1 >= 0 && dedicatedConfigurationDiscoveryMate
                ? std::optional<std::size_t>(
                      static_cast<std::size_t>(subpop1))
                : std::nullopt;
        const std::size_t tournamentCount =
            static_cast<std::size_t>(std::count_if(
                candidates.begin(),
                candidates.end(),
                [&](const auto& candidate) {
                    return !excluded.has_value() ||
                        candidate.populationIndex != *excluded;
                }));
        const auto selected =
            hgasp::selectStaticPopulationTournament(
                candidates,
                tournamentCount == 0U
                    ? 0U
                    : static_cast<std::uint64_t>(
                          RandomInt(
                              0,
                              static_cast<int>(
                                  tournamentCount - 1U))),
                tournamentCount <= 1U
                    ? 0U
                    : static_cast<std::uint64_t>(
                          RandomInt(
                              0,
                              static_cast<int>(
                                  tournamentCount - 2U))),
                excluded);
        if (selected.selected &&
            !(subpop1 >= 0 &&
              selected.populationIndex ==
                  static_cast<std::size_t>(subpop1) &&
              popman->popula[selected.populationIndex].size() <= 2U)) {
            currentMutationProbabilityPercent =
                data->mutationProbabilityPercent;
            if (roleAwarePrimaryRoleScheduled && subpop1 < 0) {
                roleAwareResidentLastPrimaryOpportunity[
                    selected.populationIndex] =
                    roleAwarePrimaryMatingOpportunity;
            }
            return static_cast<int>(selected.populationIndex);
        }
    }

    if (data->adaptivePopulationAllocationEffective()) {
        std::vector<double> diversity(
            popman->popula.size(), 0.0);
        for (const int index : eligible) {
            diversity[static_cast<std::size_t>(index)] =
                populationDiversity(index);
        }
        std::vector<int> candidates = eligible;
        if (subpop1 >= 0 && candidates.size() > 1U) {
            candidates.erase(
                std::remove(
                    candidates.begin(),
                    candidates.end(),
                    subpop1),
                candidates.end());
        }
        const int selected =
            subpop1 < 0
                ? searchAllocator.selectPrimaryPopulation(
                      candidates, diversity)
                : searchAllocator.selectMatePopulation(
                      candidates, diversity);
        if (subpop1 < 0) {
            currentMutationProbabilityPercent =
                searchAllocator.mutationProbability(
                    data->mutationProbabilityPercent,
                    diversity[static_cast<std::size_t>(selected)],
                    data->maxIterNonProd > 0
                        ? static_cast<double>(
                              data->nbIterNonImpro) /
                              static_cast<double>(
                                  data->maxIterNonProd)
                        : 0.0,
                    spJustImproved);
        }
        return selected;
    }

    if (eligible.size() == 2U) {
        return eligible.front() == subpop1
            ? eligible.back()
            : eligible.front();
    }

    constexpr int maximumTournamentAttempts = 64;
    for (int attempt = 0; attempt < maximumTournamentAttempts; ++attempt) {
        int left = RandomInt(
            0, static_cast<int>(eligible.size()) - 1);
        int right = RandomInt(
            0, static_cast<int>(eligible.size()) - 1);
        while (left == right) {
            right = RandomInt(
                0, static_cast<int>(eligible.size()) - 1);
        }
        const int target =
            avgCostPop[static_cast<std::size_t>(eligible[left])] <
                    avgCostPop[static_cast<std::size_t>(eligible[right])]
                ? eligible[left]
                : eligible[right];
        if (target != subpop1 ||
            popman->popula[static_cast<std::size_t>(target)].size() > 2U) {
            return target;
        }
    }

    std::vector<std::size_t> sizes;
    sizes.reserve(popman->popula.size());
    for (const auto& population : popman->popula) {
        sizes.push_back(population.size());
    }
    return deterministicSubpopulationFallback(
        eligible, subpop1, sizes);
}


int RouteGenerator::deterministicSubpopulationFallback(
        const std::vector<int>& eligible,
        const int excludedPopulation,
        const std::vector<std::size_t>& populationSizes) {
    if (eligible.empty()) {
        return -1;
    }
    for (const int candidate : eligible) {
        if (candidate != excludedPopulation) {
            return candidate;
        }
    }
    if (excludedPopulation >= 0 &&
        static_cast<std::size_t>(excludedPopulation) <
            populationSizes.size() &&
        populationSizes[
            static_cast<std::size_t>(excludedPopulation)] > 2U) {
        return excludedPopulation;
    }
    return eligible.front();
}

hgasp::DepotConfigurationKey
RouteGenerator::configurationKey(
        const Individual& individual) const {
    std::vector<int> depots;
    for (int depot = 0; depot < data->numL; ++depot) {
        if (individual.location[depot].used) {
            depots.push_back(depot);
        }
    }
    return hgasp::makeDepotConfigurationKey(
        std::move(depots));
}


void RouteGenerator::evolvePop(int rankPop, int &count, RoutePool* pool){
    if ((int)popman->popula[rankPop].size() <= 2){
        count = data->maxIterationPerLoc;
        return;
    }
    int sizeSubPop = 0;
    ma = NULL;
    lastBatchParentLineage = {};
    lastOffspringBatchId = 0U;
    lastFirstParentConfiguration.reset();
    fa = popman->selectionParentSolution(ma, rankPop);
    ma = popman->selectionParentSolution(fa, rankPop);
    if (fa != nullptr && ma != nullptr) {
        lastBatchParentLineage.first =
            captureParentLineage(*fa, rankPop);
        lastBatchParentLineage.second =
            captureParentLineage(*ma, rankPop);
        lastBatchParentLineage.initialized = true;
        if (nextOffspringBatchId ==
            std::numeric_limits<std::uint64_t>::max()) {
            throw std::overflow_error(
                "offspring batch identifier exhausted");
        }
        lastOffspringBatchId = nextOffspringBatchId++;
        lastFirstParentConfiguration =
            lastBatchParentLineage.first.actualConfiguration;
    }
    const bool generated = cross->gEAX(
            fa,
            ma,
            subPop.get(),
            sizeSubPop);
    fa = nullptr;
    ma = nullptr;
    if (!generated || sizeSubPop <= 0) {
        // A degenerate parent pair is still one diversification attempt.
        // Advancing the counter prevents the initialization loop from
        // spinning forever when EAX has no admissible cycle set.
        ++count;
        return;
    }
    for (int i =0; i<sizeSubPop; i++){
        if (refineChild(i)){
            if (data->staticHgampPopulationsEnabled()) {
                const hgasp::RouteSubmissionContext submission =
                    submissionContext(0U, rankPop, rankPop, i);
                integrateToPopulation(i, pool, &submission);
            } else {
                popman->addIndividual(&subPop[i], rankPop, true, pool);
            }
        }
        count++;
        if (initializationStopRequested())
            return;
    }
}

void RouteGenerator::initialPop(
        RoutePool* pool,
        const hgasp::ConfigurationScreeningResult* screening){
    hgasp::ScopedComponentTimer timer(
        hgasp::TimedComponent::InitialPopulation);
    popman->popula.erase(popman->popula.begin()+data->numLocConfiguration +1, popman->popula.end());
    const auto admitInitialToTypedPopulation =
        [this, pool](Individual& candidate,
                     const int populationIndex,
                     bool* const admitted,
                     std::uint64_t* const survivingIndividualId = nullptr) {
            if (!data->staticHgampPopulationsEnabled()) {
                return popman->addIndividual(
                    &candidate,
                    populationIndex,
                    true,
                    pool,
                    nullptr,
                    admitted,
                    survivingIndividualId);
            }
            hgasp::PopulationAdmissionTarget target;
            target.populationIndex = populationIndex;
            if (populationIndex < data->numLocConfiguration) {
                target.role =
                    hgasp::PopulationAdmissionRole::FixedResident;
                if (staticConfigurationRegistry != nullptr) {
                    const auto& slot = staticConfigurationRegistry->slots()[
                        static_cast<std::size_t>(populationIndex)];
                    target.generation = slot.generation;
                    target.expectedConfiguration = slot.configuration;
                } else {
                    std::vector<int> depots;
                    for (const int depotNode :
                         data->compactLocConfigure[
                             static_cast<std::size_t>(populationIndex)]) {
                        depots.push_back(depotNode - data->numV);
                    }
                    target.generation = 1U;
                    target.expectedConfiguration =
                        hgasp::makeDepotConfigurationKey(
                            std::move(depots));
                }
            } else {
                target.role =
                    hgasp::PopulationAdmissionRole::UnrestrictedFree;
                target.generation = staticConfigurationRegistry == nullptr
                    ? 1U
                    : staticPopulationRuntime[
                          static_cast<std::size_t>(populationIndex)]
                          .generation;
            }
            if (staticConfigurationRegistry != nullptr) {
                return addIndividualToStaticTarget(
                    candidate,
                    target,
                    true,
                    pool,
                    nullptr,
                    admitted,
                    survivingIndividualId);
            }
            return popman->addIndividualToTarget(
                &candidate,
                target,
                true,
                pool,
                nullptr,
                admitted,
                survivingIndividualId);
        };
    int i;
	    for ( i = 0; i<data->numLocConfiguration +1 ;i++){// an extra subPop
			if (staticConfigurationRegistry != nullptr &&
				i < data->numLocConfiguration &&
				!staticConfigurationRegistry->slots()[
					static_cast<std::size_t>(i)].active) {
				continue;
			}
	        std::cout<<i<<" th ";
	        std::cout<<data->elapsedCpuSeconds()<<std::endl;
        
        double bestFit=Max;
        const bool fixedPopulation =
            i < data->numLocConfiguration;
        const std::size_t configuredCandidateBudget =
            static_cast<std::size_t>(
                data->initialPopulationCandidateMultiplier) *
            static_cast<std::size_t>(std::max(0, data->popMin));
        // Role-aware Set S already paid for independently validated exact-config
        // seeds during configuration screening.  Start each fixed population
        // from that retained set and let ordinary offspring grow it toward
        // popMin; repeating exact education for every configuration dominated
        // short-run CPU without creating a distinct information source.
        const std::size_t candidateBudget =
            fixedPopulation && screening != nullptr &&
                    hgasp::isSetSProductionSearchProfile(
                        data->parameters.profile)
                ? std::min(
                    configuredCandidateBudget,
                    static_cast<std::size_t>(std::max(
                        1, data->configurationEvaluationTotalSamples)))
                : configuredCandidateBudget;
        std::size_t reusableSeedCount = 0U;
        std::size_t validatedReusableSeedCount = 0U;
        std::size_t admittedReusableSeedCount = 0U;
        if (fixedPopulation && screening != nullptr) {
            std::vector<int> depots;
            depots.reserve(
                data->compactLocConfigure[
                    static_cast<std::size_t>(i)].size());
            for (const int depotNode :
                 data->compactLocConfigure[
                     static_cast<std::size_t>(i)]) {
                depots.push_back(depotNode - data->numV);
            }
            const hgasp::DepotConfigurationKey expected =
                hgasp::makeDepotConfigurationKey(
                    std::move(depots));
            const auto reusable =
                screening->reusableSeeds.find(expected);
            if (reusable != screening->reusableSeeds.end()) {
                reusableSeedCount = reusable->second.size();
                for (const hgasp::ScreenedConfigurationSeed& seed :
                     reusable->second) {
                    if (initializationStopRequested()) {
                        return;
                    }
                    if (seed.configuration != expected) {
                        throw std::logic_error(
                            "screening seed configuration does not match "
                            "its fixed population");
                    }
                    temIndiv->rebuildFromSP(seed.routes);
                    temIndiv->decideFeasible();
                    if (!temIndiv->isFeasible ||
                        !exactPopulationConfiguration(*temIndiv, i) ||
                        !data->equalTwoVar(
                            temIndiv->fit, seed.objective)) {
                        throw std::logic_error(
                            "screening seed failed independent fixed-"
                            "population reconstruction");
                    }

                    // The screening evaluation counts toward the original
                    // candidate budget even if population duplicate admission
                    // rejects the independently validated reconstruction.
                    ++validatedReusableSeedCount;
                    bool admitted = false;
                    admitInitialToTypedPopulation(
                        *temIndiv,
                        i,
                        &admitted);
                    if (admitted) {
                        ++admittedReusableSeedCount;
                    }
                    if (bestFit > temIndiv->fit) {
                        bestFit = temIndiv->fit;
                    }
                }
            }
        }
        const std::size_t remainingConstructionAttempts =
            fixedPopulation
                ? (candidateBudget > validatedReusableSeedCount
                       ? candidateBudget - validatedReusableSeedCount
                       : 0U)
                : candidateBudget;
        for (std::size_t j = 0U;
             j < remainingConstructionAttempts;
             ++j){
            const bool constructed =
                fixedPopulation && !data->hasHardTimeWindows()
                    ? initialSol
                        ->initialCapacityOnlyExactConfiguration(
                            temIndiv.get(), i)
                    : initialSol->initial_solution(temIndiv.get(), i);
            if (!constructed) {
                if (initializationStopRequested())
                    return;
                continue;
            }
            temIndiv->decideFeasible();
            if (!temIndiv->isFeasible)
                continue;
            if (i < data->numLocConfiguration &&
                !exactPopulationConfiguration(*temIndiv, i)) {
                continue;
            }
            const hgasp::DepotConfigurationKey
                beforeInitializationLocalSearch =
                    configurationKey(*temIndiv);
            bool initializationLocalSearchRan = false;
            if (fixedPopulation && !timeLimitReached()) {
                const ExactConfigurationEducationResult education =
                    educateExactConfigurationSeed(*temIndiv, i);
                if (!education.seedValid) {
                    throw std::logic_error(
                        "fixed population construction reached local "
                        "search without an exact feasible seed");
                }
            } else if (!timeLimitReached()) {
                ls->local_search_run(
                    temIndiv.get(),
                    data->penaltyDepot,
                    data->penaltyVeh);
                initializationLocalSearchRan = true;
            }
            temIndiv->decideFeasible();
            if (!temIndiv->isFeasible ||
                (i < data->numLocConfiguration &&
                 !exactPopulationConfiguration(*temIndiv, i))) {
                continue;
            }
            hgasp::ConfigurationDiscoveryTrace trace;
            const bool initializationConfigurationChanged =
                !fixedPopulation && initializationLocalSearchRan &&
                beforeInitializationLocalSearch !=
                    configurationKey(*temIndiv);
            if (!fixedPopulation) {
                trace = hgasp::initialFreeConfigurationDiscoveryTrace(
                    beforeInitializationLocalSearch,
                    configurationKey(*temIndiv),
                    makeConfigurationEvidenceUnitId(
                        "free_initialization"));
            }
            bool admitted = false;
            std::uint64_t survivingIndividualId = 0U;
            if (initializationConfigurationChanged &&
                staticConfigurationRegistry != nullptr) {
                const ConfigurationAdmissionResult admission =
                    admitConfigurationDiscovery(
                        *temIndiv, pool, nullptr, trace, true);
                admitted = admission.admitted;
            } else {
                admitInitialToTypedPopulation(
                    *temIndiv,
                    i,
                    &admitted,
                    &survivingIndividualId);
            }
            if (!fixedPopulation &&
                staticConfigurationRegistry != nullptr &&
                !initializationConfigurationChanged) {
                observeConfigurationDiscovery(
                    *temIndiv,
                    admitted,
                    trace,
                    PopulationRole::UnrestrictedFree);
            } else if (!fixedPopulation &&
                       staticConfigurationRegistry == nullptr &&
                       survivingIndividualId != 0U &&
                       initializationConfigurationChanged) {
                pendingBootstrapFreeDiscoveries[
                    survivingIndividualId] = trace;
            }
            if (bestFit>temIndiv->fit)
                bestFit=temIndiv->fit;
            if (initializationStopRequested())
                return;
        }
        if (fixedPopulation) {
            hgasp::RunLogger::event(
                hgasp::LogLevel::info,
                "search",
                "fixed_population_seed_reuse",
                {},
                {
                    {"population_index", i},
                    {"candidate_budget",
                        static_cast<unsigned long long>(candidateBudget)},
                    {"reusable_seeds_available",
                        static_cast<unsigned long long>(reusableSeedCount)},
                    {"reusable_seeds_rebuilt_validated",
                        static_cast<unsigned long long>(
                            validatedReusableSeedCount)},
                    {"reusable_seeds_admitted",
                        static_cast<unsigned long long>(
                            admittedReusableSeedCount)},
                    {"new_construction_attempts",
                        static_cast<unsigned long long>(
                            remainingConstructionAttempts)},
                });
        } else {
            hgasp::RunLogger::event(
                hgasp::LogLevel::info,
                "search",
                "free_population_initialization_budget",
                {},
                {
                    {"population_index", i},
                    {"candidate_budget",
                        static_cast<unsigned long long>(candidateBudget)},
                    {"new_construction_attempts",
                        static_cast<unsigned long long>(
                            remainingConstructionAttempts)},
                });
        }
        if (popman->popula[i].empty()) {
            throw std::runtime_error(
                "failed to construct any feasible initial solution for "
                "population " + std::to_string(i));
        }
	        std::cout<<"The best fit is    "<<bestFit
	                 <<"  The CPU time consuming  "
	                 <<data->elapsedCpuSeconds()<<std::endl;
        
        int count = 0;
        if ((int)popman->popula[i].size() <= 1)
            continue;
        while (count < data->maxIterationPerLoc) {
            evolvePop(i, count, pool);
            if (initializationStopRequested())
                return;
        }
		if (i < data->numLocConfiguration) {
			registerPopulationConfiguration(i);
		}
		if (staticConfigurationRegistry != nullptr) {
			refreshStaticRuntime(i);
		}
    }
    std::cout<<"Finishing constructing number of initial population"<<std::endl;
}


int RouteGenerator::generateEAXChildren(
        int& subPop1,
        int& subPop2,
        RoutePool* const /*pool*/) {
    int sizeSubPop = 0;
    lastBatchParentLineage = {};
    lastOffspringBatchId = 0U;
    lastFirstParentConfiguration.reset();

    std::set<hgasp::DepotConfigurationKey> availableConfigurations;
    std::size_t eligiblePopulationCount = 0U;
    if (data->staticHgampPopulationsEnabled() &&
        staticConfigurationRegistry != nullptr) {
        for (std::size_t populationIndex = 0U;
             populationIndex < popman->popula.size();
             ++populationIndex) {
            const bool unrestricted = populationIndex ==
                staticConfigurationRegistry
                    ->unrestrictedPopulationIndex();
            const bool challenger =
                challengerActive() &&
                static_cast<int>(populationIndex) ==
                    challengerPopulationIndex();
            const bool resident =
                populationIndex <
                    staticConfigurationRegistry->slots().size() &&
                staticConfigurationRegistry->slots()[populationIndex]
                    .active;
            if ((!unrestricted && !challenger && !resident) ||
                popman->popula[populationIndex].empty()) {
                continue;
            }
            ++eligiblePopulationCount;
            for (const Individual* individual :
                 popman->popula[populationIndex]) {
                availableConfigurations.insert(
                    configurationKey(*individual));
            }
        }
    }

    hgasp::CrossConfigurationSelectionDecision crossDecision;
    const hgasp::StaticMatingScope matingScope =
        challengerActive() && subPop1 == challengerPopulationIndex()
            ? hgasp::StaticMatingScope::ConfigurationDiscovery
            : hgasp::StaticMatingScope::ResidentReproduction;
    const bool requireDistinctConfiguration =
        hgasp::requiresDistinctConfigurationMating(
            matingScope,
            data->strictCrossConfigurationMating);
    const std::size_t maximumAttempts = static_cast<std::size_t>(
        std::max(1, data->crossConfigurationSelectionMaxAttempts));
    for (std::size_t attempt = 1U; attempt <= maximumAttempts; ++attempt) {
        ma = nullptr;
        {
            hgasp::ScopedComponentTimer timer(
                hgasp::TimedComponent::ParentAndPopulationSelection);
            fa = popman->selectionParentSolution(ma, subPop1);
            ma = popman->selectionParentSolution(fa, subPop2);
        }
        if (fa == nullptr || ma == nullptr) {
            throw std::logic_error(
                "parent selection returned a null individual");
        }
        if (!data->staticHgampPopulationsEnabled() ||
            staticConfigurationRegistry == nullptr ||
            !requireDistinctConfiguration) {
            break;
        }
        crossDecision = hgasp::evaluateCrossConfigurationPair(
            {
                static_cast<std::size_t>(subPop1),
                static_cast<std::size_t>(subPop2),
                configurationKey(*fa),
                configurationKey(*ma),
            },
            availableConfigurations.size(),
            eligiblePopulationCount,
            attempt,
            maximumAttempts);
        if (crossDecision.action !=
            hgasp::CrossConfigurationSelectionAction::Retry) {
            break;
        }
        subPop2 = selectsubPop(subPop1);
        if (subPop2 < 0) {
            throw std::logic_error(
                "cross-configuration reselection found no mate population");
        }
    }

    if (data->staticHgampPopulationsEnabled() &&
        staticConfigurationRegistry != nullptr) {
        ++staticMatingAttempts;
    }
    if (subPop1 != subPop2) {
        ++crossPopulationMatings;
    }
    const hgasp::DepotConfigurationKey firstConfiguration =
        configurationKey(*fa);
    const hgasp::DepotConfigurationKey secondConfiguration =
        configurationKey(*ma);
    lastBatchParentLineage.first =
        captureParentLineage(*fa, subPop1);
    lastBatchParentLineage.second =
        captureParentLineage(*ma, subPop2);
    lastBatchParentLineage.initialized = true;
    if (nextOffspringBatchId ==
        std::numeric_limits<std::uint64_t>::max()) {
        throw std::overflow_error(
            "offspring batch identifier exhausted");
    }
    lastOffspringBatchId = nextOffspringBatchId++;
    lastFirstParentConfiguration = firstConfiguration;
    if (firstConfiguration != secondConfiguration) {
        ++crossConfigurationMatings;
        if (data->staticHgampPopulationsEnabled() &&
            requireDistinctConfiguration) {
            hgasp::recordV114ComponentActivation(
                hgasp::V114Component::StrictCrossConfigurationMating);
        }
    } else if (data->staticHgampPopulationsEnabled() &&
               requireDistinctConfiguration) {
        ++sameConfigurationFallbackMatings;
        const std::string reason =
            hgasp::crossConfigurationFallbackReasonName(
                crossDecision.fallbackReason);
        ++sameConfigurationFallbackReasons[reason];
        hgasp::RunLogger::event(
            hgasp::LogLevel::warning,
            "crossover",
            "same_configuration_parent_fallback",
            {},
            {
                {"iteration", data->iteration},
                {"first_population", subPop1},
                {"second_population", subPop2},
                {"attempts", static_cast<unsigned long long>(
                    crossDecision.attemptsUsed)},
                {"reason", reason},
            });
    }

    lastFirstParentIndividualId =
        lastBatchParentLineage.first.individualId;
    lastSecondParentIndividualId =
        lastBatchParentLineage.second.individualId;
    lastFirstParentSpAncestry =
        lastBatchParentLineage.first.spAncestryCallId;
    lastSecondParentSpAncestry =
        lastBatchParentLineage.second.spAncestryCallId;

    const bool generated = cross->gEAX(
        fa, ma, subPop.get(), sizeSubPop);
    fa = nullptr;
    ma = nullptr;
    return generated ? sizeSubPop : 0;
}


bool RouteGenerator::refineChild(const int childIndex) {
    lastMutationCommitted = false;
    lastChildAdmitted = false;
    const int mutationProbability =
        data->adaptivePopulationAllocationEffective()
            ? currentMutationProbabilityPercent
            : data->mutationProbabilityPercent;
    return refineUnifiedChild(childIndex, mutationProbability);
}

bool RouteGenerator::refineUnifiedChild(
        const int childIndex,
        const int mutationProbability) {
    childPopulationValidationReceipts[
        static_cast<std::size_t>(childIndex)].reset();
    const hgasp::DepotConfigurationKey rawConfiguration =
        configurationKey(subPop[childIndex]);
    const hgasp::DepotConfigurationKey parentConfiguration =
        hgasp::offspringConfigurationDiscoveryBaseline(
            lastBatchParentLineage, rawConfiguration);
    initializeChildConfigurationDiscovery(
        childIndex,
        hgasp::ConfigurationDiscoverySource::MdEaxRecombination,
        parentConfiguration,
        rawConfiguration);

    // Run the HGAMP repair boundary for every mdEAX child. For an already
    // feasible child this is the inexpensive normalization path that refreshes
    // route aggregates and the objective; skipping it leaves the objective
    // copied from the parent even though EAX has changed the route plan.
    const hgasp::DepotConfigurationKey beforeRepairConfiguration =
        configurationKey(subPop[childIndex]);
    const CapacityRepairResult repairResult =
        repair->repairCapacityConstraint(
        &subPop[childIndex],
        data->penaltyDepot,
        data->penaltyVeh);
    recordChildConfigurationStage(
        childIndex,
        hgasp::ConfigurationDiscoverySource::CapacityRepair,
        beforeRepairConfiguration,
        configurationKey(subPop[childIndex]));
    bool rebuildSucceeded = repairResult.feasible;
    const bool setSRepairOnly =
        hgasp::isSetSProductionSearchProfile(data->parameters.profile);
    if (!rebuildSucceeded && !setSRepairOnly) {
        const hgasp::DepotConfigurationKey beforeReconstructionConfiguration =
            configurationKey(subPop[childIndex]);
        const std::vector<int> order =
            extractCompleteCustomerOrder(subPop[childIndex], *data);
        rebuildSucceeded =
            initialSol->reconstructConstraintFeasiblePlan(
                &subPop[childIndex],
                order,
                std::vector<std::vector<int>>{},
                childIndex);
        recordChildConfigurationStage(
            childIndex,
            hgasp::ConfigurationDiscoverySource::
                ConstraintFeasibleReconstruction,
            beforeReconstructionConfiguration,
            configurationKey(subPop[childIndex]));
    }
    if (!rebuildSucceeded) {
        return false;
    }

    if (RandomInt(0, 99) < mutationProbability) {
        const hgasp::DepotConfigurationKey beforeMutationConfiguration =
            configurationKey(subPop[childIndex]);
        const double beforeMutation = subPop[childIndex].fit;
        const double mutationCpuStarted =
            data->elapsedCpuSeconds();
        const bool committed =
            mutationOperator->mutationRun(&subPop[childIndex]);
        lastMutationCommitted = committed;
        recordChildConfigurationStage(
            childIndex,
            hgasp::ConfigurationDiscoverySource::Mutation,
            beforeMutationConfiguration,
            configurationKey(subPop[childIndex]));
        searchAllocator.reportMutation(
            committed,
            committed
                ? std::max(
                      0.0,
                      beforeMutation - subPop[childIndex].fit)
                : 0.0,
            data->elapsedCpuSeconds() - mutationCpuStarted);
    }

    const hgasp::DepotConfigurationKey beforeLocalSearchConfiguration =
        configurationKey(subPop[childIndex]);
    const bool activeResidentConfiguration =
        staticConfigurationRegistry != nullptr &&
        staticConfigurationRegistry
            ->tokenForConfiguration(beforeLocalSearchConfiguration)
            .has_value();
    const bool primaryParentIsFixedResident =
        lastBatchParentLineage.initialized &&
        lastBatchParentLineage.first.populationRole ==
            hgasp::RoutePopulationRole::FixedResident;
	const bool deferLsObjectiveToAdmission =
		hgasp::isSetSProductionSearchProfile(
			data->parameters.profile) &&
		!data->hasHardTimeWindows();
    if (hgasp::preserveActiveResidentConfigurationDuringOffspringLs(
            data->parameters.profile,
            primaryParentIsFixedResident,
            activeResidentConfiguration)) {
		if (deferLsObjectiveToAdmission) {
			ls->local_search_fixed_configuration_deferred_objective(
				&subPop[childIndex],
				beforeLocalSearchConfiguration.openDepots,
				data->penaltyDepot,
				data->penaltyVeh);
		} else {
			ls->local_search_fixed_configuration(
				&subPop[childIndex],
				beforeLocalSearchConfiguration.openDepots,
				data->penaltyDepot,
				data->penaltyVeh);
		}
    } else {
		if (deferLsObjectiveToAdmission) {
			ls->local_search_run_deferred_objective(
				&subPop[childIndex],
				data->penaltyDepot,
				data->penaltyVeh);
		} else {
			ls->local_search_run(
				&subPop[childIndex],
				data->penaltyDepot,
				data->penaltyVeh);
		}
    }
    const Individual::PopulationValidationReceipt validation =
		deferLsObjectiveToAdmission
			? subPop[childIndex]
				.normalizeAndValidateForPopulationAdmission()
			: subPop[childIndex].validateForPopulationAdmission();
    if (!subPop[childIndex].isFeasible) {
        throw std::logic_error(
            "offspring became infeasible after mutation/local search");
    }
    recordChildConfigurationStage(
        childIndex,
        hgasp::ConfigurationDiscoverySource::LocalSearch,
        beforeLocalSearchConfiguration,
        configurationKey(subPop[childIndex]));
    childConfigurationDiscovery[
        static_cast<std::size_t>(childIndex)]
            .independentlyValidated = true;
    childPopulationValidationReceipts[
        static_cast<std::size_t>(childIndex)] = validation;
    return true;
}


bool RouteGenerator::integrateToPopulation(
        const int childIndex,
        RoutePool* pool,
        const hgasp::RouteSubmissionContext* const submissionInput) {
    const hgasp::RouteSubmissionContext* submission = submissionInput;
    std::optional<hgasp::RouteSubmissionContext>
        enrichedSubmission;
    if (submission != nullptr) {
        enrichedSubmission = *submission;
        submission = &*enrichedSubmission;
    }
    int whichSubPop;
    int isGenerateNewSubPop;
    bool admitted = false;
    const auto& validation = childPopulationValidationReceipts[
        static_cast<std::size_t>(childIndex)];
    if (!validation.has_value()) {
        throw std::logic_error(
            "refined offspring has no population-validation receipt");
    }

    if (data->staticHgampPopulationsEnabled() &&
        data->numLocConfiguration > 0){
        const hgasp::DepotConfigurationKey actual =
            configurationKey(subPop[childIndex]);
        if (staticConfigurationRegistry != nullptr) {
            hgasp::ConfigurationDiscoveryTrace discovery =
                childConfigurationDiscovery[
                    static_cast<std::size_t>(childIndex)];
            if (lastBatchParentLineage.initialized &&
                (actual == lastBatchParentLineage.first.actualConfiguration ||
                 actual == lastBatchParentLineage.second.actualConfiguration)) {
                discovery.beforeConfiguration = actual;
                discovery.afterConfiguration = actual;
                discovery.source =
                    hgasp::ConfigurationDiscoverySource::Count;
                discovery.contributors = 0U;
            }
            if (enrichedSubmission.has_value()) {
                hgasp::enrichConfigurationDiscoverySubmission(
                    *enrichedSubmission,
                    discovery.beforeConfiguration,
                    actual,
                    discovery.source,
                    discovery.contributors,
                    discovery.evidenceUnitId);
                submission = &*enrichedSubmission;
            }
            const int activeChallengerIndex =
                challengerPopulationIndex();
            const bool challengerParent = challengerActive() &&
                submission != nullptr &&
                (submission->firstPopulation == activeChallengerIndex ||
                 submission->secondPopulation == activeChallengerIndex);
            if (challengerParent) {
                ++challengerRuntime.offspringProduced;
                if (actual == challengerRuntime.configuration) {
                    ++challengerRuntime.matchingConfigurationOffspring;
                }
            }
            const ConfigurationAdmissionResult admission =
                admitConfigurationDiscovery(
                    subPop[childIndex],
                    pool,
                    submission,
                    discovery,
                    false,
                    &*validation);
            admitted = admission.admitted;
            lastChildAdmitted = admitted;
            return admitted;
        }
        whichSubPop = data->numLocConfiguration;
            for (int populationIndex = 0;
                 populationIndex < data->numLocConfiguration;
                 ++populationIndex) {
                std::vector<int> expectedDepots;
                for (const int depotNode :
                     data->compactLocConfigure[
                         static_cast<std::size_t>(populationIndex)]) {
                    expectedDepots.push_back(depotNode - data->numV);
                }
                if (hgasp::makeDepotConfigurationKey(
                        std::move(expectedDepots)) == actual) {
                    whichSubPop = populationIndex;
                    break;
                }
            }
        
        hgasp::PopulationAdmissionTarget bootstrapTarget;
        bootstrapTarget.populationIndex = whichSubPop;
        bootstrapTarget.generation = 1U;
        if (whichSubPop < data->numLocConfiguration) {
            bootstrapTarget.role =
                hgasp::PopulationAdmissionRole::FixedResident;
            std::vector<int> expectedDepots;
            for (const int depotNode :
                 data->compactLocConfigure[
                     static_cast<std::size_t>(whichSubPop)]) {
                expectedDepots.push_back(depotNode - data->numV);
            }
            bootstrapTarget.expectedConfiguration =
                hgasp::makeDepotConfigurationKey(
                    std::move(expectedDepots));
        } else {
            bootstrapTarget.role =
                hgasp::PopulationAdmissionRole::UnrestrictedFree;
        }
        std::uint64_t survivingIndividualId = 0U;
        isGenerateNewSubPop = popman->addIndividualToTarget(
            &subPop[childIndex],
            bootstrapTarget,
            false,
            pool,
            submission,
            &admitted,
            &survivingIndividualId,
            true,
            &*validation);
        if (whichSubPop == data->numLocConfiguration &&
            survivingIndividualId != 0U) {
            hgasp::ConfigurationDiscoveryTrace trace =
                childConfigurationDiscovery[
                    static_cast<std::size_t>(childIndex)];
            trace.afterConfiguration = actual;
            trace.independentlyValidated = true;
            pendingBootstrapFreeDiscoveries[
                survivingIndividualId] = std::move(trace);
        }
        getAverageValue(whichSubPop);
        
        if (isGenerateNewSubPop > -1){
            throw std::logic_error(
                "static HGAMP admission attempted an unplanned population "
                "replacement");
        }
    }
    else {
        popman->addIndividual(
            &subPop[childIndex],
            0,
            false,
            pool,
            submission,
            &admitted,
            nullptr,
            true,
            &*validation);
    }
    lastChildAdmitted = admitted;
    return admitted;
}

bool RouteGenerator::retainPostSpLocalSearchConfigurationDiscovery(
        Individual& candidate,
        const hgasp::DepotConfigurationKey& beforeConfiguration,
        RoutePool* const pool,
        const std::string& spCallId,
        const bool /*globalSpScope*/) {
    candidate.decideFeasible();
    if (!candidate.isFeasible) {
        throw std::logic_error(
            "post-SP LS configuration discovery is infeasible");
    }
    const hgasp::DepotConfigurationKey afterConfiguration =
        configurationKey(candidate);
    if (afterConfiguration == beforeConfiguration) {
        return false;
    }
    (void)injectPostSpLocalSearchCandidate(
        candidate,
        beforeConfiguration,
        pool,
        spCallId,
        std::nullopt);
    return true;
}

SpInjectionResult RouteGenerator::injectPostSpLocalSearchCandidate(
        Individual& candidate,
        const hgasp::DepotConfigurationKey& rawSpConfiguration,
        RoutePool* const pool,
        const std::string& spCallId,
        const std::optional<int> unchangedConfigurationTarget,
        const std::string& evidenceUnitId) {
    candidate.decideFeasible();
    if (!candidate.isFeasible) {
        throw std::logic_error(
            "post-SP local-search candidate is infeasible");
    }
    const hgasp::DepotConfigurationKey afterConfiguration =
        configurationKey(candidate);
    hgasp::ConfigurationDiscoveryTrace trace;
    trace.beforeConfiguration = rawSpConfiguration;
    trace.afterConfiguration = afterConfiguration;
    trace.source = afterConfiguration != rawSpConfiguration
        ? hgasp::ConfigurationDiscoverySource::LocalSearch
        : hgasp::ConfigurationDiscoverySource::Count;
    trace.contributors = afterConfiguration != rawSpConfiguration
        ? hgasp::configurationDiscoverySourceBit(trace.source)
        : 0U;
    trace.evidenceUnitId = evidenceUnitId.empty()
        ? makeConfigurationEvidenceUnitId("post_sp_ls")
        : evidenceUnitId;
    trace.routeAncestry = spCallId;
    trace.independentlyValidated = true;
    trace.initialized = true;

    hgasp::RouteSubmissionContext submission;
    submission.origin = hgasp::RouteOriginKind::LocalSearch;
    submission.iteration = currentPhysicalAttemptForProvenance();
    submission.spAncestryCallId = spCallId;
    hgasp::enrichConfigurationDiscoverySubmission(
        submission,
        rawSpConfiguration,
        afterConfiguration,
        trace.source,
        trace.contributors,
        trace.evidenceUnitId);
    const double priorGlobalObjective = global->fit;
    SpInjectionResult injection;
    if (staticConfigurationRegistry != nullptr) {
        if (submission.configurationChanged) {
            const ConfigurationAdmissionResult admission =
                admitConfigurationDiscovery(
                    candidate, pool, &submission, trace, false);
            if (admission.admitted) {
                injection.disposition =
                    SpInjectionDisposition::AdmittedExisting;
                injection.populationIndex = admission.populationIndex;
            }
        } else {
            hgasp::PopulationAdmissionTarget target =
                admissionTargetForConfiguration(afterConfiguration);
            if (unchangedConfigurationTarget.has_value()) {
                const int requested = *unchangedConfigurationTarget;
                const std::size_t freeIndex =
                    staticConfigurationRegistry
                        ->unrestrictedPopulationIndex();
                if (requested < 0 ||
                    static_cast<std::size_t>(requested) > freeIndex) {
                    throw std::out_of_range(
                        "post-SP LS admission target is out of range");
                }
                if (static_cast<std::size_t>(requested) == freeIndex) {
                    target = {
                        hgasp::PopulationAdmissionRole::UnrestrictedFree,
                        requested,
                        staticPopulationRuntime[freeIndex].generation,
                        std::nullopt,
                    };
                } else {
                    const auto& slot =
                        staticConfigurationRegistry->slots()[
                            static_cast<std::size_t>(requested)];
                    if (!slot.active ||
                        slot.configuration != afterConfiguration) {
                        throw std::logic_error(
                            "post-SP LS result does not match its typed "
                            "fixed target");
                    }
                    target = {
                        hgasp::PopulationAdmissionRole::FixedResident,
                        requested,
                        slot.generation,
                        slot.configuration,
                    };
                }
            }
            bool admitted = false;
            const int replacement = addIndividualToStaticTarget(
                candidate,
                target,
                false,
                pool,
                &submission,
                &admitted);
            if (replacement >= 0) {
                throw std::logic_error(
                    "post-SP LS admission attempted implicit replacement");
            }
            if (admitted) {
                injection.disposition =
                    SpInjectionDisposition::AdmittedExisting;
                injection.populationIndex = target.populationIndex;
            }
        }
        injection.incumbentAccepted =
            global->fit + Min < priorGlobalObjective;
        return injection;
    }
    if (popman->popula.empty()) {
        return injection;
    }
    const int targetPopulation = 0;
    bool admitted = false;
    popman->addIndividual(
        &candidate,
        targetPopulation,
        false,
        pool,
        &submission,
        &admitted);
    if (submission.configurationChanged) {
        observeConfigurationDiscovery(
            candidate,
            admitted,
            trace,
            PopulationRole::UnrestrictedFree);
    }
    if (targetPopulation >= 0 &&
        static_cast<std::size_t>(targetPopulation) <
            popman->popula.size() &&
        !popman->popula[static_cast<std::size_t>(
            targetPopulation)].empty()) {
        getAverageValue(targetPopulation);
    }
    if (admitted) {
        injection.disposition = SpInjectionDisposition::AdmittedExisting;
        injection.populationIndex = targetPopulation;
    }
    injection.incumbentAccepted =
        global->fit + Min < priorGlobalObjective;
    return injection;
}

SpInjectionResult RouteGenerator::injectIncumbent(
        std::unique_ptr<Individual> spIndiv,
        RoutePool* pool,
        const std::string& spCallId,
        const std::optional<int> populationAdmissionTarget,
        const hgasp::ConfigurationDiscoveryContributorMask
            additionalDiscoveryContributors,
        const std::string& evidenceUnitId) {
    hgasp::ScopedComponentTimer injectionTimer(
        hgasp::TimedComponent::SpInjection);
    hgasp::RouteSubmissionContext submission;
    submission.origin =
        hgasp::RouteOriginKind::SpInjection;
    submission.iteration = currentPhysicalAttemptForProvenance();
    submission.spAncestryCallId = spCallId;
    const double priorGlobalObjective = global->fit;
    SpInjectionResult injection;
    if (data->staticHgampPopulationsEnabled() &&
        staticConfigurationRegistry != nullptr) {
        spIndiv->decideFeasible();
        if (!spIndiv->isFeasible) {
            return injection;
        }
        const hgasp::DepotConfigurationKey actual =
            configurationKey(*spIndiv);
        const bool globalConfigurationDiscovery =
            !populationAdmissionTarget.has_value() &&
            hgasp::isGlobalSpConfigurationDiscoveryCall(spCallId);
        hgasp::ConfigurationDiscoveryTrace globalSpTrace;
        globalSpTrace.beforeConfiguration =
            globalConfigurationDiscovery && global != nullptr &&
                    global->isFeasible && global->fit < Max
                ? configurationKey(*global)
                : globalConfigurationDiscovery
                ? hgasp::makeDepotConfigurationKey(std::vector<int>{})
                : actual;
        globalSpTrace.afterConfiguration = actual;
        globalSpTrace.source =
            hgasp::ConfigurationDiscoverySource::Count;
        globalSpTrace.contributors = 0U;
        globalSpTrace.evidenceUnitId = evidenceUnitId.empty()
            ? makeConfigurationEvidenceUnitId("raw_sp")
            : evidenceUnitId;
        globalSpTrace.routeAncestry = spCallId;
        globalSpTrace.independentlyValidated = true;
        globalSpTrace.initialized = true;
        if (globalConfigurationDiscovery &&
            globalSpTrace.beforeConfiguration != actual) {
            globalSpTrace.source =
                hgasp::ConfigurationDiscoverySource::GlobalSp;
            globalSpTrace.contributors =
                hgasp::configurationDiscoverySourceBit(
                    globalSpTrace.source) |
                additionalDiscoveryContributors;
        }
        hgasp::enrichConfigurationDiscoverySubmission(
            submission,
            globalSpTrace.beforeConfiguration,
            actual,
            globalSpTrace.source,
            globalSpTrace.contributors,
            globalSpTrace.evidenceUnitId);
        globalSpTrace.strictGlobalIncumbent =
            globalConfigurationDiscovery &&
            spIndiv->fit + Min < priorGlobalObjective;
        hgasp::StaticPopulationTarget target =
            staticConfigurationRegistry->classify(actual);
        if (populationAdmissionTarget.has_value()) {
            const int requested = *populationAdmissionTarget;
            const std::size_t freeIndex =
                staticConfigurationRegistry
                    ->unrestrictedPopulationIndex();
            if (requested < 0 ||
                static_cast<std::size_t>(requested) > freeIndex) {
                throw std::out_of_range(
                    "SP population admission target is out of range");
            }
            if (static_cast<std::size_t>(requested) == freeIndex) {
                target = {
                    hgasp::StaticPopulationTargetKind::Unrestricted,
                    freeIndex,
                    std::nullopt,
                };
            } else {
                const auto& slot =
                    staticConfigurationRegistry->slots()[
                        static_cast<std::size_t>(requested)];
                if (!slot.active || slot.configuration != actual) {
                    throw std::logic_error(
                        "Local SP result does not match its fixed "
                        "population target");
                }
                target = {
                    hgasp::StaticPopulationTargetKind::Fixed,
                    static_cast<std::size_t>(requested),
                    slot.token(),
                };
            }
        }
        if (!populationAdmissionTarget.has_value() &&
            !target.fixedToken.has_value() &&
            globalSpTrace.strictGlobalIncumbent) {
            const bool matchesActiveChallenger = challengerActive() &&
                challengerRuntime.configuration == actual;
            const std::string promotionSource = spCallId.empty()
                ? "global_sp"
                : spCallId;
            const hgasp::GlobalSpDirectPromotionResult result =
                hgasp::executeGlobalSpDirectPromotion(
                    matchesActiveChallenger,
                    [&]() {
                        if (!popman->acceptValidatedSpIncumbent(
                                *spIndiv, &submission)) {
                            throw std::logic_error(
                                "strict Global-SP improvement was not "
                                "accepted as incumbent");
                        }
                    },
                    [&]() {
                        if (pool != nullptr) {
                            pool->submitFeasibleIndividual(
                                spIndiv.get(), submission);
                        }
                    },
                    [&]() {
                        hgasp::RunLogger::event(
                            hgasp::LogLevel::info,
                            "population",
                            "global_sp_direct_promotion_attempted",
                            {},
                            {
                                {"iteration", data->iteration},
                                {"source", promotionSource},
                                {"configuration",
                                    configurationLabel(actual)},
                                {"objective", spIndiv->fit},
                            });
                        hgasp::PromotionFailureKind failure =
                            hgasp::PromotionFailureKind::None;
                        std::string reason;
                        const auto promoted = promoteConfiguration(
                            actual,
                            hgasp::StaticReplacementAuthority::
                                GlobalSpStrictIncumbent,
                            spIndiv.get(),
                            pool,
                            promotionSource,
                            false,
                            false,
                            &failure,
                            &reason);
                        if (promoted.has_value()) {
                            hgasp::RunLogger::event(
                                hgasp::LogLevel::info,
                                "population",
                                "global_sp_direct_promotion_succeeded",
                                {},
                                {
                                    {"iteration", data->iteration},
                                    {"source", promotionSource},
                                    {"configuration",
                                        configurationLabel(actual)},
                                    {"population",
                                        promoted->populationIndex},
                                });
                            return hgasp::DirectPromotionAttempt{
                                true,
                                promoted->populationIndex,
                                hgasp::PromotionFailureKind::None,
                                std::string(),
                            };
                        }
                        hgasp::RunLogger::event(
                            hgasp::LogLevel::warning,
                            "population",
                            "global_sp_direct_promotion_failed",
                            {},
                            {
                                {"iteration", data->iteration},
                                {"source", promotionSource},
                                {"configuration",
                                    configurationLabel(actual)},
                                {"failure_kind", hgasp::toString(failure)},
                                {"reason", reason},
                            });
                        return hgasp::DirectPromotionAttempt{
                            false, -1, failure, reason};
                    },
                    [&](const hgasp::GlobalSpPromotionFallbackTarget
                            fallbackTarget) {
                        const int fallbackPopulation = fallbackTarget ==
                                hgasp::GlobalSpPromotionFallbackTarget::
                                    ActiveChallenger
                            ? challengerPopulationIndex()
                            : static_cast<int>(
                                  staticConfigurationRegistry
                                      ->unrestrictedPopulationIndex());
                        const hgasp::PopulationAdmissionTarget
                            fallbackAdmission = fallbackTarget ==
                                hgasp::GlobalSpPromotionFallbackTarget::
                                    ActiveChallenger
                            ? hgasp::PopulationAdmissionTarget{
                                  hgasp::PopulationAdmissionRole::
                                      DynamicChallenger,
                                  fallbackPopulation,
                                  challengerRuntime.token.generation,
                                  challengerRuntime.configuration,
                              }
                            : hgasp::PopulationAdmissionTarget{
                                  hgasp::PopulationAdmissionRole::
                                      UnrestrictedFree,
                                  fallbackPopulation,
                                  staticPopulationRuntime[
                                      static_cast<std::size_t>(
                                          fallbackPopulation)]
                                      .generation,
                                  std::nullopt,
                              };
                        bool admitted = false;
                        const int replacement = addIndividualToStaticTarget(
                            *spIndiv,
                            fallbackAdmission,
                            false,
                            nullptr,
                            &submission,
                            &admitted);
                        if (replacement >= 0) {
                            throw std::logic_error(
                                "Global-SP promotion fallback attempted "
                                "an implicit resident replacement");
                        }
                        if (fallbackTarget ==
                                hgasp::GlobalSpPromotionFallbackTarget::
                                    ActiveChallenger) {
                            SubPopulation& challengerPopulation =
                                popman->popula[static_cast<std::size_t>(
                                    fallbackPopulation)];
                            while (challengerPopulation.size() >
                                   static_cast<std::size_t>(
                                       std::max(1, data->popMin))) {
                                popman->removeWorstBiasedFitness(
                                    challengerPopulation);
                            }
                            challengerRuntime.bestObjective = std::min(
                                challengerRuntime.bestObjective,
                                spIndiv->fit);
                        }
                        if (!popman->popula[
                                 static_cast<std::size_t>(
                                     fallbackPopulation)].empty()) {
                            getAverageValue(fallbackPopulation);
                        }
                        hgasp::RunLogger::event(
                            hgasp::LogLevel::warning,
                            "population",
                            "global_sp_direct_promotion_fallback",
                            {},
                            {
                                {"iteration", data->iteration},
                                {"source", promotionSource},
                                {"configuration",
                                    configurationLabel(actual)},
                                {"fallback_target",
                                    hgasp::toString(fallbackTarget)},
                                {"fallback_population",
                                    fallbackPopulation},
                                {"admitted", admitted},
                            });
                        return hgasp::PromotionFallbackAdmission{
                            admitted, fallbackPopulation};
                    });
            injection = {
                result.populationAdmitted()
                    ? SpInjectionDisposition::AdmittedExisting
                    : SpInjectionDisposition::Rejected,
                result.populationIndex,
                true};
            const PopulationRole admissionRole = result.promoted
                ? PopulationRole::FixedResident
                : result.fallbackTarget == hgasp::
                      GlobalSpPromotionFallbackTarget::ActiveChallenger
                ? PopulationRole::DynamicChallenger
                : result.fallbackAdmitted
                ? PopulationRole::UnrestrictedFree
                : PopulationRole::InactiveOrSpare;
            observeConfigurationDiscovery(
                *spIndiv,
                result.fallbackAdmitted &&
                    result.fallbackTarget == hgasp::
                        GlobalSpPromotionFallbackTarget::UnrestrictedFree,
                globalSpTrace,
                admissionRole);
            if (result.populationIndex >= 0 &&
                static_cast<std::size_t>(result.populationIndex) <
                    popman->popula.size() &&
                !popman->popula[static_cast<std::size_t>(
                    result.populationIndex)].empty()) {
                getAverageValue(result.populationIndex);
            }
            return injection;
        }
        if (globalConfigurationDiscovery) {
            const bool genuinelyNewConfiguration =
                globalSpTrace.source ==
                    hgasp::ConfigurationDiscoverySource::GlobalSp &&
                !target.fixedToken.has_value() &&
                !(challengerRuntime.active() &&
                  challengerRuntime.configuration == actual);
            double weakestResidentBest = -std::numeric_limits<double>::infinity();
            if (genuinelyNewConfiguration) {
                for (const auto& slot :
                     staticConfigurationRegistry->slots()) {
                    if (!slot.active || slot.slot >= popman->popula.size() ||
                        popman->popula[slot.slot].empty()) {
                        continue;
                    }
                    weakestResidentBest = std::max(
                        weakestResidentBest,
                        popman->popula[slot.slot].front()->fit);
                }
            }
            // A new Global-SP configuration that is promising but does not
            // beat the global incumbent is observed in the permanent free
            // population. It must not consume direct resident-replacement
            // authority; a later challenger trial supplies that evidence.
            if (genuinelyNewConfiguration &&
                std::isfinite(weakestResidentBest) &&
                spIndiv->fit + Min < weakestResidentBest) {
                const std::size_t freeIndex =
                    staticConfigurationRegistry
                        ->unrestrictedPopulationIndex();
                const hgasp::PopulationAdmissionTarget freeTarget{
                    hgasp::PopulationAdmissionRole::UnrestrictedFree,
                    static_cast<int>(freeIndex),
                    staticPopulationRuntime[freeIndex].generation,
                    std::nullopt,
                };
                bool admitted = false;
                const int replacement = addIndividualToStaticTarget(
                    *spIndiv,
                    freeTarget,
                    false,
                    pool,
                    &submission,
                    &admitted);
                if (replacement >= 0) {
                    throw std::logic_error(
                        "Global-SP free observation attempted resident "
                        "replacement");
                }
                observeConfigurationDiscovery(
                    *spIndiv,
                    admitted,
                    globalSpTrace,
                    PopulationRole::UnrestrictedFree);
                if (!popman->popula[freeIndex].empty()) {
                    getAverageValue(static_cast<int>(freeIndex));
                }
                injection = {
                    admitted
                        ? SpInjectionDisposition::AdmittedExisting
                        : SpInjectionDisposition::Rejected,
                    admitted ? static_cast<int>(freeIndex) : -1,
                    false};
                return injection;
            }
            const ConfigurationAdmissionResult admission =
                admitConfigurationDiscovery(
                    *spIndiv,
                    pool,
                    &submission,
                    globalSpTrace,
                    false);
            if (admission.admitted) {
                injection = {
                    SpInjectionDisposition::AdmittedExisting,
                    admission.populationIndex};
            }
            injection.incumbentAccepted =
                global->fit + Min < priorGlobalObjective;
            return injection;
        }
        const hgasp::PopulationAdmissionTarget admissionTarget =
            target.fixedToken.has_value()
            ? hgasp::PopulationAdmissionTarget{
                  hgasp::PopulationAdmissionRole::FixedResident,
                  static_cast<int>(target.populationIndex),
                  target.fixedToken->generation,
                  actual,
              }
            : hgasp::PopulationAdmissionTarget{
                  hgasp::PopulationAdmissionRole::UnrestrictedFree,
                  static_cast<int>(target.populationIndex),
                  staticPopulationRuntime[target.populationIndex]
                      .generation,
                  std::nullopt,
              };
        bool admitted = false;
        const int replacement = addIndividualToStaticTarget(
            *spIndiv,
            admissionTarget,
            false,
            pool,
            &submission,
            &admitted);
        if (replacement >= 0) {
            throw std::logic_error(
                "static SP admission attempted an immediate population "
                "replacement");
        }
        if (admitted) {
            injection = {
                SpInjectionDisposition::AdmittedExisting,
                static_cast<int>(target.populationIndex)};
        }
    } else {
        injection = popman->injectSPSolution(
            std::move(spIndiv),
            pool,
            &submission);
    }

    injection.incumbentAccepted =
        injection.incumbentAccepted ||
        global->fit + Min < priorGlobalObjective;

    // SP insertion can change an existing resident population without
    // creating a new configuration. Refresh every non-empty cached mean so
    // subsequent subpopulation tournaments observe the injected solution.
    for (int rank = 0; rank < data->numLocConfiguration + 1; ++rank) {
        if (!popman->popula[rank].empty()) {
            getAverageValue(rank);
        }
    }

    if (injection.createdPopulation()) {
        const int spNewPopIdx = injection.populationIndex;
        searchAllocator.resetPopulation(spNewPopIdx);
        incubatePopulation(spNewPopIdx, pool, "sp");
        registerPopulationConfiguration(spNewPopIdx);
    }
    return injection;
}

hgasp::ConfigurationPopulationBuildResult
RouteGenerator::buildConfigurationPopulation(
        const int populationIndex,
        const hgasp::ConfigurationPopulationBuildOptions& options,
        RoutePool* pool) {
    if (populationIndex < 0 ||
        static_cast<std::size_t>(populationIndex) >=
            popman->popula.size()) {
        throw std::out_of_range(
            "configuration population build index is out of range");
    }
    hgasp::ConfigurationPopulationBuildResult result;
    const double startCpu = data->elapsedCpuSeconds();
    SubPopulation& population =
        popman->popula[static_cast<std::size_t>(populationIndex)];
    if (options.admissionTarget.has_value() &&
        options.admissionTarget->populationIndex != populationIndex) {
        throw std::invalid_argument(
            "configuration population build target index mismatch");
    }
    if (staticConfigurationRegistry != nullptr &&
        !options.admissionTarget.has_value()) {
        throw std::invalid_argument(
            "static configuration population build requires a typed "
            "admission target");
    }
    if (options.speculative &&
        (staticConfigurationRegistry == nullptr ||
         !options.admissionTarget.has_value())) {
        throw std::invalid_argument(
            "speculative population build requires a typed static target");
    }
    RoutePool* const effectivePool =
        options.speculative ? nullptr : pool;
    const auto admitBuiltIndividual = [&](Individual& individual,
                                          bool* const admitted) {
        if (staticConfigurationRegistry != nullptr) {
            return addIndividualToStaticTarget(
                individual,
                *options.admissionTarget,
                false,
                effectivePool,
                nullptr,
                admitted,
                nullptr,
                !options.speculative);
        }
        return popman->addIndividual(
            &individual,
            populationIndex,
            false,
            effectivePool,
            nullptr,
            admitted,
            nullptr,
            !options.speculative);
    };

    hgasp::DepotConfigurationKey configuration;
    if (options.requireExactConfiguration) {
        configuration = hgasp::makeDepotConfigurationKey(
            options.configuration.openDepots);
        if (configuration != options.configuration ||
            configuration.openDepots.empty()) {
            throw std::invalid_argument(
                "exact population build requires a non-empty canonical "
                "configuration");
        }
        for (const int depot : configuration.openDepots) {
            if (depot < 0 || depot >= data->numL) {
                throw std::out_of_range(
                    "configuration population build depot is out of range");
            }
        }
    }

    const std::size_t target = options.targetSize;
    if (target == 0U || options.maximumConstructionAttempts == 0U) {
        throw std::invalid_argument(
            "configuration population build bounds must be positive");
    }
    for (const hgasp::ScreenedConfigurationSeed& seed : options.seeds) {
        if (population.size() >= target ||
            initializationStopRequested()) {
            break;
        }
        ++result.seedSnapshotsAttempted;
        if (options.requireExactConfiguration &&
            seed.configuration != configuration) {
            throw std::logic_error(
                "population founder snapshot configuration mismatch");
        }
        temIndiv->rebuildFromSP(seed.routes);
        temIndiv->decideFeasible();
        if (!temIndiv->isFeasible ||
            (options.requireExactConfiguration &&
             !exactConfiguration(*temIndiv, configuration)) ||
            (seed.objective != Max &&
             !data->equalTwoVar(temIndiv->fit, seed.objective))) {
            throw std::logic_error(
                "population founder snapshot failed independent rebuild");
        }
        bool admitted = false;
        (void)admitBuiltIndividual(*temIndiv, &admitted);
        if (admitted) {
            ++result.seedSnapshotsAdmitted;
            ++result.admittedMembers;
        }
    }

    const std::size_t noProgressLimit = std::min(
        options.maximumConstructionAttempts,
        std::max<std::size_t>(
            static_cast<std::size_t>(
                data->populationBuildNoProgressMinimumAttempts),
            target * static_cast<std::size_t>(
                data->populationBuildNoProgressTargetMultiplier)));
    hgasp::PopulationIncubationGuard guard(
        target,
        population.size(),
        options.maximumConstructionAttempts,
        noProgressLimit);

    while (guard.beginAttempt(
            population.size(),
            initializationStopRequested())) {
        ++result.constructionAttempts;
        const bool constructed = options.requireExactConfiguration
            ? (data->hasHardTimeWindows()
                ? initialSol->initialConstraintAwareExactConfiguration(
                    temIndiv.get(), configuration)
                : initialSol->initialCapacityOnlyExactConfiguration(
                    temIndiv.get(), configuration))
            : initialSol->initial_solution(
                temIndiv.get(), populationIndex);
        if (constructed) {
            ++result.constructionSuccesses;
            bool seedEligible = true;
            std::optional<hgasp::ConfigurationDiscoveryTrace>
                nonExactLocalSearchDiscovery;
            if (options.requireExactConfiguration) {
                temIndiv->decideFeasible();
                seedEligible = temIndiv->isFeasible &&
                    exactConfiguration(*temIndiv, configuration);
                if (!seedEligible) {
                    throw std::logic_error(
                        "exact fixed-configuration constructor violated "
                        "its contract during population incubation");
                }
                if (seedEligible) {
                    const ExactConfigurationEducationResult education =
                        educateExactConfigurationSeed(
                            *temIndiv,
                            configuration,
                            options.retainConfigurationDiscoveries);
                    if (!education.seedValid) {
                        throw std::logic_error(
                            "population incubation reached local search "
                            "without an exact feasible seed");
                    }
                }
            } else {
                const hgasp::DepotConfigurationKey beforeLocalSearch =
                    configurationKey(*temIndiv);
                ls->local_search_run(
                    temIndiv.get(),
                    data->penaltyDepot,
                    data->penaltyVeh);
                temIndiv->decideFeasible();
                if (temIndiv->isFeasible &&
                    beforeLocalSearch != configurationKey(*temIndiv)) {
                    hgasp::ConfigurationDiscoveryTrace trace;
                    trace.beforeConfiguration = beforeLocalSearch;
                    trace.afterConfiguration =
                        configurationKey(*temIndiv);
                    trace.source = hgasp::
                        ConfigurationDiscoverySource::LocalSearch;
                    trace.contributors =
                        hgasp::configurationDiscoverySourceBit(
                            trace.source);
                    trace.evidenceUnitId =
                        makeConfigurationEvidenceUnitId(
                            "population_build_ls");
                    trace.independentlyValidated = true;
                    trace.initialized = true;
                    nonExactLocalSearchDiscovery = trace;
                    if (options.retainConfigurationDiscoveries &&
                        staticConfigurationRegistry != nullptr) {
                        const ConfigurationAdmissionResult admission =
                            admitConfigurationDiscovery(
                                *temIndiv,
                                pool,
                                nullptr,
                                trace,
                                false);
                        if (admission.admitted &&
                            admission.populationIndex == populationIndex) {
                            ++result.admittedMembers;
                        }
                        seedEligible = false;
                    }
                }
            }
            temIndiv->decideFeasible();
            if (seedEligible && temIndiv->isFeasible &&
                (!options.requireExactConfiguration ||
                 configurationKey(*temIndiv) ==
                     configuration)) {
                bool admitted = false;
                (void)admitBuiltIndividual(*temIndiv, &admitted);
                if (admitted) {
                    ++result.admittedMembers;
                }
                if (options.retainConfigurationDiscoveries &&
                    nonExactLocalSearchDiscovery.has_value()) {
                    observeConfigurationDiscovery(
                        *temIndiv,
                        admitted,
                        *nonExactLocalSearchDiscovery,
                        PopulationRole::UnrestrictedFree);
                }
            }
        }
        guard.observe(population.size());
    }

    result.finalPopulationSize = population.size();
    result.reachedTarget = population.size() >= target;
    result.cpuSeconds = std::max(
        0.0, data->elapsedCpuSeconds() - startCpu);
    if (!options.speculative && !population.empty()) {
        getAverageValue(populationIndex);
    }
    if (population.size() < target) {
        std::cout
            << "[Population Incubation] source=" << options.source
            << " population=" << populationIndex
            << " status=partial"
            << " size=" << population.size()
            << " target=" << target
            << " attempts=" << guard.attempts()
            << " no_progress=" << guard.noProgressAttempts()
            << " external_stop="
            << (guard.stoppedByExternalRequest()
                    ? "true"
                    : "false")
            << std::endl;
        hgasp::RunLogger::event(
            hgasp::LogLevel::warning,
            "population",
            "incubation_partial",
            {},
            {
                {"iteration", data->iteration},
                {"source", options.source},
                {"population", populationIndex},
                {"size", static_cast<unsigned long long>(
                    population.size())},
                {"target", static_cast<unsigned long long>(target)},
                {"attempts", static_cast<unsigned long long>(
                    guard.attempts())},
                {"no_progress", static_cast<unsigned long long>(
                    guard.noProgressAttempts())},
            });
    }
    return result;
}

void RouteGenerator::incubatePopulation(
        const int populationIndex,
        RoutePool* pool,
        const char* source) {
    if (populationIndex < 0 ||
        static_cast<std::size_t>(populationIndex) >=
            popman->popula.size()) {
        throw std::out_of_range(
            "population incubation index is out of range");
    }
    if (popman->popula[static_cast<std::size_t>(populationIndex)].empty()) {
        throw std::runtime_error(
            "population incubation has no feasible seed elite");
    }

    hgasp::ConfigurationPopulationBuildOptions options;
    options.targetSize =
        static_cast<std::size_t>(std::max(1, data->popMin));
    options.maximumConstructionAttempts = std::max<std::size_t>(
        static_cast<std::size_t>(
            data->populationIncubationMinimumConstructionAttempts),
        options.targetSize * static_cast<std::size_t>(
            data->populationIncubationTargetConstructionMultiplier));
    options.source = source == nullptr ? std::string() : source;

    if (staticConfigurationRegistry != nullptr &&
        static_cast<std::size_t>(populationIndex) <
            staticConfigurationRegistry->slots().size() &&
        staticConfigurationRegistry->slots()[
            static_cast<std::size_t>(populationIndex)].active) {
        options.configuration =
            staticConfigurationRegistry->slots()[
                static_cast<std::size_t>(populationIndex)].configuration;
        const auto& slot = staticConfigurationRegistry->slots()[
            static_cast<std::size_t>(populationIndex)];
        options.admissionTarget = hgasp::PopulationAdmissionTarget{
            hgasp::PopulationAdmissionRole::FixedResident,
            populationIndex,
            slot.generation,
            slot.configuration,
        };
    } else {
        options.requireExactConfiguration = false;
    }
    if (staticConfigurationRegistry != nullptr &&
        static_cast<std::size_t>(populationIndex) ==
            staticConfigurationRegistry->unrestrictedPopulationIndex()) {
        const std::size_t freeIndex =
            staticConfigurationRegistry->unrestrictedPopulationIndex();
        options.admissionTarget = hgasp::PopulationAdmissionTarget{
            hgasp::PopulationAdmissionRole::UnrestrictedFree,
            populationIndex,
            staticPopulationRuntime[freeIndex].generation,
            std::nullopt,
        };
    }
    (void)buildConfigurationPopulation(populationIndex, options, pool);
}

void RouteGenerator::registerPopulationConfiguration(
        const int populationIndex) {
    if (populationIndex < 0 ||
        populationIndex >= data->numLocConfiguration) {
        throw std::out_of_range(
            "fixed population registration index is out of range");
    }
    std::optional<hgasp::StaticPopulationToken> token;
    hgasp::DepotConfigurationKey expected;
    if (staticConfigurationRegistry != nullptr) {
        const auto& slot = staticConfigurationRegistry->slots()[
            static_cast<std::size_t>(populationIndex)];
        if (!slot.active) {
            if (!popman->popula[
                     static_cast<std::size_t>(populationIndex)].empty()) {
                throw std::logic_error(
                    "inactive static population retained residents");
            }
            return;
        }
        token = slot.token();
        expected = slot.configuration;
    } else {
        std::vector<int> expectedDepots;
        for (const int depotNode :
             data->compactLocConfigure[
                 static_cast<std::size_t>(populationIndex)]) {
            expectedDepots.push_back(depotNode - data->numV);
        }
        expected = hgasp::makeDepotConfigurationKey(
            std::move(expectedDepots));
    }
    for (const Individual* individual :
         popman->popula[
             static_cast<std::size_t>(populationIndex)]) {
        if (individual == nullptr) {
            throw std::logic_error(
                "fixed population contains a resident with a mismatched "
                "canonical depot configuration");
        }
        const hgasp::DepotConfigurationKey actual =
            configurationKey(*individual);
        if (token.has_value()) {
            staticConfigurationRegistry->requireMatchingConfiguration(
                *token, actual);
        } else if (actual != expected) {
            throw std::logic_error(
                "fixed population contains a resident with a mismatched "
                "canonical depot configuration");
        }
    }
}

std::vector<double> RouteGenerator::populationObjectives(
        const int populationIndex) const {
    std::vector<double> objectives;
    if (populationIndex < 0 ||
        static_cast<std::size_t>(populationIndex) >=
            popman->popula.size()) {
        return objectives;
    }
    objectives.reserve(
        popman->popula[static_cast<std::size_t>(populationIndex)].size());
    for (const Individual* individual :
         popman->popula[static_cast<std::size_t>(populationIndex)]) {
        if (individual != nullptr && individual->isFeasible) {
            objectives.push_back(individual->fit);
        }
    }
    return objectives;
}

void RouteGenerator::clearChallengerPopulation() {
    const int index = challengerPopulationIndex();
    if (index >= 0) {
        SubPopulation& population =
            popman->popula[static_cast<std::size_t>(index)];
        for (Individual* individual : population) {
            delete individual;
        }
        population.clear();
        if (static_cast<std::size_t>(index) < avgCostPop.size()) {
            avgCostPop[static_cast<std::size_t>(index)] = Max;
        }
    }
    challengerRuntime = {};
}

std::size_t RouteGenerator::refreshPermanentFreePopulationFloor() {
    if (staticConfigurationRegistry == nullptr) {
        throw std::logic_error(
            "permanent free population requires a static registry");
    }
    const std::size_t freeIndex =
        staticConfigurationRegistry->unrestrictedPopulationIndex();
    if (freeIndex >= popman->popula.size() ||
        popman->popula[freeIndex].empty()) {
        throw std::logic_error(
            "permanent free population is absent or empty");
    }
    const std::size_t currentSize = popman->popula[freeIndex].size();
    const std::size_t targetFloor =
        static_cast<std::size_t>(std::max(1, data->popMin));
    permanentFreePopulationFloor = std::max(
        permanentFreePopulationFloor,
        std::min(currentSize, targetFloor));
    if (!hgasp::permanentFreePopulationMeetsEstablishedFloor(
            currentSize, permanentFreePopulationFloor)) {
        throw std::logic_error(
            "permanent free population fell below its established floor");
    }
    return currentSize;
}

void RouteGenerator::requirePermanentFreePopulationPreserved(
        const std::size_t sizeBefore) const {
    if (staticConfigurationRegistry == nullptr) {
        throw std::logic_error(
            "permanent free population requires a static registry");
    }
    const std::size_t freeIndex =
        staticConfigurationRegistry->unrestrictedPopulationIndex();
    if (freeIndex >= popman->popula.size() ||
        popman->popula[freeIndex].size() != sizeBefore ||
        !hgasp::permanentFreePopulationMeetsEstablishedFloor(
            popman->popula[freeIndex].size(),
            permanentFreePopulationFloor)) {
        throw std::logic_error(
            "static portfolio transaction changed the permanent free "
            "population");
    }
}

void RouteGenerator::refreshPopulationEliteProtection(
        RoutePool* const pool) const {
    if (pool == nullptr) {
        return;
    }
    std::vector<StandaloneRoute> eliteRoutes;
    for (const auto& population : popman->popula) {
        if (population.empty()) {
            continue;
        }
        const auto routes = pool->exportRoutes(population.front());
        eliteRoutes.insert(
            eliteRoutes.end(), routes.begin(), routes.end());
    }
    pool->setProtectedRoutes(
        RouteProtection::PopulationElite, eliteRoutes);
}

void RouteGenerator::rejectActiveChallenger(
        const char* const reason,
        RoutePool* const pool) {
    if (!challengerRuntime.active()) {
        return;
    }
    const std::uint64_t attempt = static_cast<std::uint64_t>(
        currentPhysicalAttempt());
    const hgasp::DepotConfigurationKey configuration =
        challengerRuntime.configuration;
    const std::uint64_t generation =
        challengerRuntime.token.generation;
    const std::uint64_t cooldown = attempt +
        data->parameters.challenger.rejectionCooldownAttempts;
    candidateArchive.recordRejection(
        configuration, attempt, cooldown);
    ++challengerRejections;
    hgasp::RunLogger::event(
        hgasp::LogLevel::info,
        "population",
        "challenger_rejected",
        {},
        {
            {"iteration", data->iteration},
            {"generation", static_cast<unsigned long long>(generation)},
            {"configuration", configurationLabel(configuration)},
            {"reason", reason == nullptr ? "unspecified" : reason},
            {"population_size", static_cast<unsigned long long>(
                populationObjectives(challengerPopulationIndex()).size())},
            {"primary_selections", static_cast<unsigned long long>(
                challengerRuntime.primarySelections)},
            {"eligible_mating_opportunities", static_cast<unsigned long long>(
                challengerScheduling.opportunitiesSinceTrialStart)},
            {"expected_protected_turns", static_cast<unsigned long long>(
                challengerScheduling.opportunitiesSinceTrialStart /
                challengerPolicy.parameters().protectedPrimaryPeriod)},
            {"actual_protected_turns", static_cast<unsigned long long>(
                challengerScheduling.protectedPrimarySelections)},
            {"cooldown_until", static_cast<unsigned long long>(cooldown)},
        });
    clearChallengerPopulation();
    refreshPopulationEliteProtection(pool);
}

std::optional<RouteGenerator::StaticPopulationReplacementOutcome>
RouteGenerator::promoteConfiguration(
        const hgasp::DepotConfigurationKey& configuration,
        const hgasp::StaticReplacementAuthority authority,
        Individual* const founder,
        RoutePool* const pool,
        const std::string& source,
        const bool challengerTrialCompleted,
        const bool beatsWeakestResident,
        hgasp::PromotionFailureKind* const failureKind,
        std::string* const failureReason) {
    if (failureKind != nullptr) {
        *failureKind = hgasp::PromotionFailureKind::None;
    }
    if (failureReason != nullptr) {
        failureReason->clear();
    }
    const auto recoverableFailure = [&](const hgasp::PromotionFailureKind kind,
                                        const char* const reason) {
        if (failureKind != nullptr) {
            *failureKind = kind;
        }
        if (failureReason != nullptr) {
            *failureReason = reason == nullptr ? "unspecified" : reason;
        }
        return std::optional<StaticPopulationReplacementOutcome>();
    };
    if (staticConfigurationRegistry == nullptr) {
        return recoverableFailure(
            hgasp::PromotionFailureKind::RecoverableAdmissionFailure,
            "registry_unavailable");
    }
    const hgasp::DepotConfigurationKey canonical =
        hgasp::makeDepotConfigurationKey(configuration.openDepots);
    if (canonical != configuration || configuration.openDepots.empty() ||
        staticConfigurationRegistry
            ->tokenForConfiguration(configuration)
            .has_value()) {
        return recoverableFailure(
            hgasp::PromotionFailureKind::RecoverableAdmissionFailure,
            "registry_rejected");
    }
    if (founder != nullptr) {
        founder->decideFeasible();
        if (!founder->isFeasible ||
            configurationKey(*founder) != configuration) {
            throw std::logic_error(
                "configuration promotion founder failed validation");
        }
    }

    const std::uint64_t attempt = static_cast<std::uint64_t>(
        currentPhysicalAttempt());
    std::optional<std::size_t> targetIndex;
    double targetEliteMean = -std::numeric_limits<double>::infinity();
    if (authority ==
        hgasp::StaticReplacementAuthority::HgaChallengerValidated) {
        std::vector<hgasp::ChallengerResidentCandidate> residents;
        for (const auto& slot : staticConfigurationRegistry->slots()) {
            hgasp::ChallengerResidentCandidate resident;
            resident.populationIndex = slot.slot;
            resident.active = slot.active;
            resident.protectedResident =
                slot.active && attempt < slot.protectedUntilAttempt;
            resident.minimumAgeSatisfied = slot.active &&
                attempt >= slot.createdAttempt &&
                attempt - slot.createdAttempt >=
                    static_cast<std::uint64_t>(
                        data->staticPopulationMinimumAgeAttempts);
            resident.objectives = populationObjectives(
                static_cast<int>(slot.slot));
            residents.push_back(std::move(resident));
        }
        const auto decision = challengerPolicy.evaluatePromotion(
            populationObjectives(challengerPopulationIndex()), residents);
        if (!decision.hasReplaceableResident || !decision.promote ||
            !challengerTrialCompleted || !beatsWeakestResident) {
            return std::nullopt;
        }
        targetIndex = decision.targetPopulationIndex;
        targetEliteMean = decision.residentEliteMean;
    } else if (authority ==
               hgasp::StaticReplacementAuthority::
                   ScheduledExplorationLease) {
        const std::optional<hgasp::DepotConfigurationKey> globalConfig =
            global != nullptr && global->isFeasible && global->fit < Max
            ? std::optional<hgasp::DepotConfigurationKey>(
                  configurationKey(*global))
            : std::nullopt;
        std::vector<hgasp::ScheduledRotationCandidate> candidates;
        candidates.reserve(staticConfigurationRegistry->slots().size());
        for (const auto& slot : staticConfigurationRegistry->slots()) {
            const bool populated = slot.slot < popman->popula.size() &&
                !popman->popula[slot.slot].empty();
            candidates.push_back({
                slot.slot,
                slot.active && populated,
                globalConfig.has_value() &&
                    slot.configuration == *globalConfig,
                populated
                    ? popman->popula[slot.slot].front()->fit
                    : 0.0,
            });
        }
        targetIndex = hgasp::selectScheduledRotationTarget(
            candidates, Min);
        if (targetIndex.has_value()) {
            targetEliteMean = challengerPolicy.eliteMean(
                populationObjectives(static_cast<int>(*targetIndex)));
        }
    } else {
        for (const auto& slot : staticConfigurationRegistry->slots()) {
            if (!slot.active || slot.slot >= popman->popula.size() ||
                popman->popula[slot.slot].empty()) {
                continue;
            }
            const double eliteMean = challengerPolicy.eliteMean(
                populationObjectives(static_cast<int>(slot.slot)));
            if (!targetIndex.has_value() ||
                eliteMean > targetEliteMean + Min ||
                (std::abs(eliteMean - targetEliteMean) <= Min &&
                 slot.slot < *targetIndex)) {
                targetIndex = slot.slot;
                targetEliteMean = eliteMean;
            }
        }
    }
    if (!targetIndex.has_value()) {
        return recoverableFailure(
            hgasp::PromotionFailureKind::RecoverableAdmissionFailure,
            "no_active_resident_target");
    }
    const bool targetWasProtected =
        staticConfigurationRegistry->slots()[*targetIndex].active &&
        attempt < staticConfigurationRegistry
                      ->slots()[*targetIndex]
                      .protectedUntilAttempt;

    std::vector<hgasp::ScreenedConfigurationSeed> seeds;
    const auto appendSeed = [&](const Individual& individual,
                                const std::size_t sourceIndex) {
        hgasp::ScreenedConfigurationSeed seed;
        seed.configuration = configuration;
        seed.routes = snapshotValidatedIndividual(individual);
        seed.objective = individual.fit;
        seed.diversity = 0.0;
        seed.sourceIndex = sourceIndex;
        seeds.push_back(std::move(seed));
    };
    if (founder != nullptr) {
        appendSeed(*founder, 0U);
    }
    const bool promotingActiveChallenger = challengerActive() &&
        challengerRuntime.configuration == configuration;
    if (promotingActiveChallenger) {
        std::size_t sourceIndex = seeds.size();
        for (const Individual* individual :
             popman->popula[static_cast<std::size_t>(
                 challengerPopulationIndex())]) {
            appendSeed(*individual, sourceIndex++);
        }
    }
    const std::size_t freeIndex =
        staticConfigurationRegistry->unrestrictedPopulationIndex();
    if (freeIndex >= popman->popula.size()) {
        throw std::logic_error(
            "permanent free population slot is unavailable");
    }
    SubPopulation& freePopulation = popman->popula[freeIndex];
    const std::size_t freePopulationSizeBefore =
        refreshPermanentFreePopulationFloor();
    for (Individual* individual : freePopulation) {
        if (individual != nullptr && individual->isFeasible &&
            configurationKey(*individual) == configuration) {
            appendSeed(*individual, seeds.size());
        }
    }

    const auto& retiredSlot = staticConfigurationRegistry->slots()[
        *targetIndex];
    const auto oldToken = retiredSlot.token();
    const hgasp::DepotConfigurationKey retiredConfiguration =
        retiredSlot.configuration;
    const std::vector<double> retiredObjectives =
        populationObjectives(static_cast<int>(*targetIndex));
    if (retiredObjectives.empty()) {
        throw std::logic_error(
            "configuration promotion target has no resident objective");
    }
    const double retiredBestObjective = *std::min_element(
        retiredObjectives.begin(), retiredObjectives.end());
    hgasp::StaticConfigurationReplacementRequest request;
    request.configuration = configuration;
    request.authority = authority;
    request.independentlyValidated = true;
    request.challengerTrialCompleted = challengerTrialCompleted;
    request.beatsWeakestResident = beatsWeakestResident;
    request.strictlyImprovesPriorGlobal = authority ==
        hgasp::StaticReplacementAuthority::GlobalSpStrictIncumbent;
    request.attempt = attempt;
    request.protectionAttempts = static_cast<std::uint64_t>(
        data->residentPostPromotionProtectionAttempts);

    // Every operation below may allocate or reject a stale token, so it is
    // performed on value copies before the physical successor is built.
    hgasp::StaticConfigurationRegistry preparedRegistry =
        *staticConfigurationRegistry;
    const auto newToken = preparedRegistry.replaceConfiguration(
        oldToken, request);
    std::set<int> preparedEncodedConfiguration;
    for (const int depot : configuration.openDepots) {
        preparedEncodedConfiguration.insert(data->numV + depot);
    }
    // Scheduled rotation and the legacy dynamic challenger are mutually
    // exclusive. Do not copy and grow the challenger archive when no active
    // search decision can consume it; this is a semantics-neutral V4 work
    // elimination for both production profiles.
    std::optional<hgasp::ConfigurationCandidateArchive> preparedArchive;
    const std::uint64_t protectionAttempts =
        static_cast<std::uint64_t>(
            data->retiredConfigurationCooldownAttempts);
    const std::uint64_t retirementCooldownUntil =
        protectionAttempts >
                std::numeric_limits<std::uint64_t>::max() - attempt
            ? std::numeric_limits<std::uint64_t>::max()
            : attempt + protectionAttempts;
    if (data->singleDynamicChallengerEffective()) {
        preparedArchive.emplace(candidateArchive);
        preparedArchive->recordResidentRetirement(
            retiredConfiguration,
            attempt,
            retirementCooldownUntil,
            retiredBestObjective);
        preparedArchive->erasePromoted(configuration);
    }
    StaticPopulationRuntimeState preparedRuntime;
    preparedRuntime.generation = newToken.generation;
    const std::uint64_t productiveAttempt =
        static_cast<std::uint64_t>(std::max(0, data->iteration));
    preparedRuntime.createdAttempt = productiveAttempt;
    preparedRuntime.lastBestImprovementAttempt = productiveAttempt;

    // Build the successor in a reserved internal staging slot.  Registry,
    // resident, free-slot, and challenger state remain untouched unless a
    // full popMin population has been independently reconstructed.
    const std::size_t stagingIndex =
        hgasp::staticHgampPromotionStagingPopulationIndex(freeIndex);
    if (stagingIndex >= popman->popula.size()) {
        throw std::logic_error(
            "configuration promotion staging slot is unavailable");
    }
    SubPopulation& stagingPopulation = popman->popula[stagingIndex];
    if (!stagingPopulation.empty()) {
        throw std::logic_error(
            "configuration promotion staging slot is not empty");
    }
    PopulationStagingGuard stagingGuard(stagingPopulation);
    hgasp::ConfigurationPopulationBuildOptions options;
    options.configuration = configuration;
    options.seeds = std::move(seeds);
    options.targetSize = static_cast<std::size_t>(
        std::max(1, data->popMin));
    options.maximumConstructionAttempts =
        std::max<std::size_t>(
            options.targetSize * static_cast<std::size_t>(
                data->populationPromotionConstructionMultiplier),
            options.targetSize);
    options.requireExactConfiguration = true;
    options.admissionTarget = hgasp::PopulationAdmissionTarget{
        hgasp::PopulationAdmissionRole::TransactionStaging,
        static_cast<int>(stagingIndex),
        newToken.generation,
        configuration,
    };
    options.speculative = true;
    options.retainConfigurationDiscoveries = false;
    options.source = source;
    const auto build = buildConfigurationPopulation(
        static_cast<int>(stagingIndex), options, nullptr);
    if (!build.reachedTarget ||
        build.finalPopulationSize < options.targetSize) {
        hgasp::RunLogger::event(
            hgasp::LogLevel::warning,
            "population",
            "configuration_promotion_top_up_failed",
            {},
            {
                {"iteration", data->iteration},
                {"source", source},
                {"configuration", configurationLabel(configuration)},
                {"population_size", static_cast<unsigned long long>(
                    build.finalPopulationSize)},
                {"target_size", static_cast<unsigned long long>(
                    options.targetSize)},
            });
        return recoverableFailure(
            hgasp::PromotionFailureKind::RecoverableBuildFailure,
            "population_topup_failed");
    }

    // Commit contains only no-throw swaps, scalar assignment and erasure.
    // The staging guard now owns the retired resident population and destroys
    // it on scope exit. The permanent free population is never transferred.
    SubPopulation& targetPopulation = popman->popula[*targetIndex];
    targetPopulation.swap(stagingPopulation);
    staticConfigurationRegistry->swap(preparedRegistry);
    data->compactLocConfigure[*targetIndex].swap(
        preparedEncodedConfiguration);
    if (preparedArchive.has_value()) {
        candidateArchive.swap(*preparedArchive);
    }
    staticPopulationRuntime[*targetIndex] = preparedRuntime;
    selectedMasterColumnsByPopulation.erase(
        static_cast<int>(*targetIndex));
    localSelectedMasterColumnsByPopulation.erase(
        static_cast<int>(*targetIndex));
    // The guard owns the retired resident after the swap. Destroy it before
    // route protection is recomputed so retired routes cannot be re-protected
    // as if they belonged to the successor generation.
    stagingGuard.clear();

    if (promotingActiveChallenger) {
        if (authority ==
            hgasp::StaticReplacementAuthority::GlobalSpStrictIncumbent) {
            ++globalSpPromotedActiveChallenger;
        }
        clearChallengerPopulation();
    }
    requirePermanentFreePopulationPreserved(
        freePopulationSizeBefore);
    hgasp::RunLogger::event(
        hgasp::LogLevel::info,
        "population",
        "resident_configuration_retired",
        {},
        {
            {"iteration", data->iteration},
            {"configuration",
                configurationLabel(retiredConfiguration)},
            {"replacement_configuration",
                configurationLabel(configuration)},
            {"resident_best", retiredBestObjective},
            {"cooldown_until", static_cast<unsigned long long>(
                retirementCooldownUntil)},
        });
    registerPopulationConfiguration(static_cast<int>(*targetIndex));
    getAverageValue(static_cast<int>(*targetIndex));
    refreshPopulationEliteProtection(pool);
    ++staticReplacementCount;
    if (authority ==
        hgasp::StaticReplacementAuthority::GlobalSpStrictIncumbent) {
        ++globalSpDirectPromotions;
    } else if (authority ==
               hgasp::StaticReplacementAuthority::
                   ScheduledExplorationLease) {
        ++scheduledConfigurationRotations;
    } else {
        ++challengerPromotions;
    }

    StaticPopulationReplacementOutcome outcome;
    outcome.retiredToken = {oldToken.slot, oldToken.generation};
    outcome.replacementToken = {newToken.slot, newToken.generation};
    outcome.populationIndex = static_cast<int>(*targetIndex);
    outcome.configuration = configuration;
    hgasp::RunLogger::event(
        hgasp::LogLevel::info,
        "population",
        authority ==
                hgasp::StaticReplacementAuthority::GlobalSpStrictIncumbent
            ? "global_sp_direct_configuration_promotion"
            : authority == hgasp::StaticReplacementAuthority::
                    ScheduledExplorationLease
            ? "scheduled_configuration_rotated"
            : "challenger_promoted",
        {},
        {
            {"iteration", data->iteration},
            {"source", source},
            {"configuration", configurationLabel(configuration)},
            {"target_population", static_cast<unsigned long long>(
                *targetIndex)},
            {"old_generation", static_cast<unsigned long long>(
                oldToken.generation)},
            {"new_generation", static_cast<unsigned long long>(
                newToken.generation)},
            {"target_elite_mean10", targetEliteMean},
            {"protection_bypassed_by_global_sp", authority ==
                hgasp::StaticReplacementAuthority::
                    GlobalSpStrictIncumbent && targetWasProtected},
            {"checkpoint_lease", authority ==
                hgasp::StaticReplacementAuthority::
                    ScheduledExplorationLease},
            {"active_fixed_count", static_cast<unsigned long long>(
                staticConfigurationRegistry
                    ->actualFixedPopulationCount())},
        });
    return outcome;
}

bool RouteGenerator::evaluateActiveChallenger(RoutePool* const pool) {
    if (!challengerActive()) {
        return false;
    }
    std::vector<hgasp::ChallengerResidentCandidate> residents;
    const std::uint64_t attempt = static_cast<std::uint64_t>(
        currentPhysicalAttempt());
    for (const auto& slot : staticConfigurationRegistry->slots()) {
        hgasp::ChallengerResidentCandidate resident;
        resident.populationIndex = slot.slot;
        resident.active = slot.active;
        resident.protectedResident =
            slot.active && attempt < slot.protectedUntilAttempt;
        resident.minimumAgeSatisfied = slot.active &&
            attempt >= slot.createdAttempt &&
            attempt - slot.createdAttempt >=
                static_cast<std::uint64_t>(
                    data->staticPopulationMinimumAgeAttempts);
        resident.objectives = populationObjectives(
            static_cast<int>(slot.slot));
        residents.push_back(std::move(resident));
    }
    const std::vector<double> challengerObjectives =
        populationObjectives(challengerPopulationIndex());
    const auto decision = challengerPolicy.evaluatePromotion(
        challengerObjectives, residents);
    if (!decision.hasReplaceableResident) {
        challengerRuntime.state =
            hgasp::ChallengerState::WaitingForReplaceableResident;
        return false;
    }
    ++challengerTrialsCompleted;
    const double fullMean = challengerObjectives.empty()
        ? std::numeric_limits<double>::infinity()
        : std::accumulate(
              challengerObjectives.begin(),
              challengerObjectives.end(), 0.0) /
              static_cast<double>(challengerObjectives.size());
    const auto percentageMargin = [](const double target,
                                     const double challenger) {
        return std::isfinite(target) && std::isfinite(challenger) &&
                std::abs(target) > Min
            ? 100.0 * (target - challenger) / std::abs(target)
            : 0.0;
    };
    hgasp::RunLogger::event(
        hgasp::LogLevel::info,
        "population",
        "challenger_trial_finished",
        {},
        {
            {"iteration", data->iteration},
            {"generation", static_cast<unsigned long long>(
                challengerRuntime.token.generation)},
            {"configuration", configurationLabel(
                challengerRuntime.configuration)},
            {"selection_reason", hgasp::toString(
                challengerRuntime.selectionReason)},
            {"observations_before_trial", static_cast<unsigned long long>(
                challengerRuntime.observationsBeforeTrial)},
            {"founder_best", challengerRuntime.founderObjective},
            {"top3_mean_before_trial",
                challengerRuntime.top3MeanBeforeTrial},
            {"fill_seed_count", static_cast<unsigned long long>(
                challengerRuntime.fillSeedCount)},
            {"fill_construction_attempts", static_cast<unsigned long long>(
                challengerRuntime.fillConstructionAttempts)},
            {"fill_population_size", static_cast<unsigned long long>(
                challengerRuntime.fillPopulationSize)},
            {"fill_best", challengerRuntime.fillBestObjective},
            {"selection_target_population", static_cast<unsigned long long>(
                challengerRuntime.selectionTargetPopulationIndex)},
            {"selection_target_best",
                challengerRuntime.selectionTargetBest},
            {"selection_target_elite_mean10",
                challengerRuntime.selectionTargetEliteMean},
            {"promote", decision.promote},
            {"challenger_best", decision.challengerBest},
            {"challenger_elite_mean10", decision.challengerEliteMean},
            {"challenger_full_mean", fullMean},
            {"challenger_diversity",
                populationDiversity(challengerPopulationIndex())},
            {"resident_best", decision.residentBest},
            {"resident_elite_mean10", decision.residentEliteMean},
            {"best_margin_pct", percentageMargin(
                decision.residentBest, decision.challengerBest)},
            {"elite_mean10_margin_pct", percentageMargin(
                decision.residentEliteMean,
                decision.challengerEliteMean)},
            {"eligible_mating_opportunities", static_cast<unsigned long long>(
                challengerScheduling.opportunitiesSinceTrialStart)},
            {"expected_protected_turns", static_cast<unsigned long long>(
                challengerScheduling.opportunitiesSinceTrialStart /
                challengerPolicy.parameters().protectedPrimaryPeriod)},
            {"actual_protected_turns", static_cast<unsigned long long>(
                challengerScheduling.protectedPrimarySelections)},
            {"offspring_produced", static_cast<unsigned long long>(
                challengerRuntime.offspringProduced)},
            {"matching_configuration_offspring",
                static_cast<unsigned long long>(
                    challengerRuntime.matchingConfigurationOffspring)},
            {"matching_configuration_offspring_rate",
                challengerRuntime.offspringProduced == 0U
                    ? 0.0
                    : static_cast<double>(
                          challengerRuntime
                              .matchingConfigurationOffspring) /
                      static_cast<double>(
                          challengerRuntime.offspringProduced)},
        });
    if (!decision.promote) {
        rejectActiveChallenger("promotion_quality_gate", pool);
        return false;
    }
    const auto configuration = challengerRuntime.configuration;
    const auto promoted = promoteConfiguration(
        configuration,
        hgasp::StaticReplacementAuthority::HgaChallengerValidated,
        nullptr,
        pool,
        "hga_challenger",
        true,
        true);
    if (!promoted.has_value()) {
        rejectActiveChallenger("promotion_top_up_failed", pool);
        return false;
    }
    return true;
}

void RouteGenerator::advanceSingleDynamicChallenger(
        RoutePool* const pool,
        const bool reduction2Completed) {
    if (!data->singleDynamicChallengerEffective() ||
        staticConfigurationRegistry == nullptr) {
        return;
    }
    const std::uint64_t attempt = static_cast<std::uint64_t>(
        currentPhysicalAttempt());
    if (challengerRuntime.state ==
            hgasp::ChallengerState::ProtectedTrial &&
        attempt >= challengerRuntime.trialEndAttempt) {
        challengerRuntime.state =
            hgasp::ChallengerState::ReadyForEvaluation;
        evaluateActiveChallenger(pool);
        return;
    }
    if (challengerRuntime.state ==
            hgasp::ChallengerState::WaitingForReplaceableResident &&
        attempt % challengerPolicy.parameters()
                .selectionIntervalAttempts == 0U) {
        challengerRuntime.state =
            hgasp::ChallengerState::ReadyForEvaluation;
        evaluateActiveChallenger(pool);
        return;
    }
    if (challengerRuntime.active()) {
        return;
    }
    const bool roleAwareProfile =
        hgasp::isSetSProductionSearchProfile(data->parameters.profile) ||
        hgasp::isSetFProductionSearchProfile(data->parameters.profile);
    if (roleAwareProfile && challengerTrialsCompleted >=
        data->parameters.challenger.maximumCompletedTrials) {
        return;
    }

    hgasp::ChallengerLaunchContext launch;
    launch.profile = data->hasHardTimeWindows()
        ? hgasp::ChallengerProblemProfile::TimeWindows
        : hgasp::ChallengerProblemProfile::Clrp;
    launch.attempt = attempt;
    launch.phaseHorizonAttempts =
        data->populationPhaseHorizonAttempts > 0
            ? static_cast<std::uint64_t>(
                data->populationPhaseHorizonAttempts)
            : static_cast<std::uint64_t>(
                std::max(0, data->iterLimit));
    launch.reduction2Completed = reduction2Completed;
    launch.slotEmpty = true;
    if (!challengerPolicy.shouldAttemptSelection(launch)) {
        return;
    }
    const auto context = challengerEligibilityContext();
    std::vector<const hgasp::ConfigurationCandidateRecord*> eligible;
    for (const auto& entry : candidateArchive.records()) {
        const auto decision = candidateArchive.evaluateEligibility(
            entry.second, context);
        if (decision.eligible) {
            eligible.push_back(&entry.second);
            if (decision.reason == hgasp::
                    ConfigurationCandidateEligibilityReason::
                        ExceptionalBest) {
                ++candidateEligibilityExceptional;
            } else if (decision.reason == hgasp::
                    ConfigurationCandidateEligibilityReason::
                        RepeatedWithQualityFloor) {
                ++candidateEligibilityRepeatedQuality;
            }
            continue;
        }
        if (decision.reason == hgasp::
                ConfigurationCandidateEligibilityReason::QualityFloor) {
            ++candidateRejectedByQualityFloor;
        } else if (decision.reason == hgasp::
                ConfigurationCandidateEligibilityReason::Cooldown) {
            ++candidateBlockedByCooldown;
        } else if (decision.reason == hgasp::
                ConfigurationCandidateEligibilityReason::
                    NoReplaceableResident) {
            ++candidateBlockedNoReplaceableResident;
        }
    }
    if (eligible.empty()) {
        return;
    }
    std::size_t firstIndex = 0U;
    std::size_t secondIndex = 0U;
    if (eligible.size() >= 2U) {
        const auto pair = challengerPolicy.drawDistinctPair(
            eligible.size(),
            [](const std::size_t bound) {
                return static_cast<std::size_t>(RandomInt(
                    0, static_cast<int>(bound) - 1));
            });
        firstIndex = pair.first;
        secondIndex = pair.second;
    }
    const auto* selected = challengerPolicy.selectTwoWayTournament(
        eligible, firstIndex, secondIndex);
    if (selected == nullptr) {
        return;
    }
    if (challengerGeneration ==
        std::numeric_limits<std::uint64_t>::max()) {
        throw std::overflow_error(
            "challenger generation identifier overflow");
    }
    ++challengerGeneration;
    challengerScheduling.beginGeneration(challengerGeneration);
    challengerRuntime = {};
    challengerRuntime.state = hgasp::ChallengerState::Filling;
    challengerRuntime.token.generation = challengerGeneration;
    challengerRuntime.configuration = selected->configuration;
    challengerRuntime.selectedAttempt = attempt;
    challengerRuntime.founderObjective = selected->bestObjective;
    const auto selectedEligibility =
        candidateArchive.evaluateEligibility(*selected, context);
    if (!selectedEligibility.eligible) {
        throw std::logic_error(
            "challenger tournament selected an ineligible record");
    }
    challengerRuntime.selectionReason = selectedEligibility.reason;
    challengerRuntime.observationsBeforeTrial =
        selected->totalObservationCount;
    challengerRuntime.top3MeanBeforeTrial = selected->top3Mean();
    challengerRuntime.hadSelectionTarget =
        context.weakestReplaceableResident.has_value();
    if (context.weakestReplaceableResident.has_value()) {
        challengerRuntime.selectionTargetPopulationIndex =
            context.weakestReplaceableResident->populationIndex;
        challengerRuntime.selectionTargetBest =
            context.weakestReplaceableResident->bestObjective;
        challengerRuntime.selectionTargetEliteMean =
            context.weakestReplaceableResident->eliteMean10;
    }
    const auto* priorTrial = candidateArchive.findTrialHistory(
        selected->configuration);
    if (priorTrial != nullptr && priorTrial->totalTrials > 0U) {
        ++challengerRetriesAfterCooldown;
    }
    candidateArchive.recordTrialStarted(
        selected->configuration, challengerGeneration);
    hgasp::RunLogger::event(
        hgasp::LogLevel::info,
        "population",
        "challenger_selected",
        {},
        {
            {"iteration", data->iteration},
            {"generation", static_cast<unsigned long long>(
                challengerGeneration)},
            {"configuration", configurationLabel(
                selected->configuration)},
            {"observation_count", static_cast<unsigned long long>(
                selected->totalObservationCount)},
            {"founder_objective", selected->bestObjective},
            {"eligibility_reason", hgasp::toString(
                selectedEligibility.reason)},
            {"selection_had_replaceable_resident",
                challengerRuntime.hadSelectionTarget},
            {"replaceable_resident_best",
                challengerRuntime.selectionTargetBest},
            {"replaceable_resident_elite_mean10",
                challengerRuntime.selectionTargetEliteMean},
        });
    hgasp::ConfigurationPopulationBuildOptions options;
    options.configuration = selected->configuration;
    options.seeds = selected->diverseSeeds;
    options.targetSize = challengerPolicy.targetPopulationSize(
        static_cast<std::size_t>(std::max(1, data->popMin)));
    options.maximumConstructionAttempts =
        challengerPolicy.maximumConstructionAttempts(
            static_cast<std::size_t>(std::max(1, data->popMin)));
    options.requireExactConfiguration = true;
    options.admissionTarget = hgasp::PopulationAdmissionTarget{
        hgasp::PopulationAdmissionRole::DynamicChallenger,
        challengerPopulationIndex(),
        challengerRuntime.token.generation,
        challengerRuntime.configuration,
    };
    options.source = "dynamic_challenger_fill";
    const auto build = buildConfigurationPopulation(
        challengerPopulationIndex(), options, pool);
    challengerRuntime.fillSeedCount = selected->diverseSeeds.size();
    challengerRuntime.fillConstructionAttempts =
        build.constructionAttempts;
    challengerRuntime.fillPopulationSize = build.finalPopulationSize;
    challengerRuntime.fillCpuSeconds = build.cpuSeconds;
    challengerFillCpuSeconds += build.cpuSeconds;
    const std::size_t minimumViable =
        challengerPolicy.minimumViablePopulationSize(
            static_cast<std::size_t>(std::max(1, data->popMin)));
    hgasp::RunLogger::event(
        hgasp::LogLevel::info,
        "population",
        "challenger_fill_finished",
        {},
        {
            {"iteration", data->iteration},
            {"generation", static_cast<unsigned long long>(
                challengerGeneration)},
            {"configuration", configurationLabel(
                challengerRuntime.configuration)},
            {"population_size", static_cast<unsigned long long>(
                build.finalPopulationSize)},
            {"minimum_viable", static_cast<unsigned long long>(
                minimumViable)},
            {"construction_attempts", static_cast<unsigned long long>(
                build.constructionAttempts)},
            {"fill_cpu_seconds", build.cpuSeconds},
        });
    if (build.finalPopulationSize < minimumViable) {
        ++challengerFillFailures;
        rejectActiveChallenger("minimum_viable_fill_failed", pool);
        return;
    }
    challengerRuntime.state =
        hgasp::ChallengerState::ProtectedTrial;
    challengerRuntime.trialStartAttempt = attempt;
    challengerRuntime.trialEndAttempt = attempt +
        challengerPolicy.trialAttempts(launch.profile);
    challengerRuntime.bestObjective =
        popman->popula[static_cast<std::size_t>(
            challengerPopulationIndex())].front()->fit;
    challengerRuntime.fillBestObjective =
        challengerRuntime.bestObjective;
    ++challengerTrialsStarted;
    hgasp::RunLogger::event(
        hgasp::LogLevel::info,
        "population",
        "challenger_trial_started",
        {},
        {
            {"iteration", data->iteration},
            {"generation", static_cast<unsigned long long>(
                challengerGeneration)},
            {"configuration", configurationLabel(
                challengerRuntime.configuration)},
            {"trial_end_attempt", static_cast<unsigned long long>(
                challengerRuntime.trialEndAttempt)},
        });
    refreshPopulationEliteProtection(pool);
}

bool RouteGenerator::checkStagnationAndRestart(RoutePool* pool) {
    const double elapsedTime = data->elapsedCpuSeconds();
    const double remainingTime =
        data->timeLimit > 0
            ? static_cast<double>(data->timeLimit) - elapsedTime
            : std::numeric_limits<double>::infinity();
    const int remainingIterations =
        data->iterLimit - data->iteration;
    const RestartControlDecision decision =
        SearchAttemptScheduler::decideRestart(
            {
                data->nbIterNonImpro,
                data->maxIterNonProd,
                remainingIterations,
                remainingTime,
                data->restartMinimumRemainingIterations,
                data->restartMinimumRemainingCpuSeconds,
            });

    if (decision.action == RestartControlAction::None) {
        restartSuppressionLatched = false;
        return false;
    }
    if (decision.action ==
        RestartControlAction::SuppressedByRemainingBudget) {
        // A budget gate is not an optimization improvement. Preserve the
        // true counter so mutation, SP triggering, and diagnostics continue
        // to observe the same long-stagnation state.
        if (!restartSuppressionLatched) {
            restartSuppressionLatched = true;
            ++data->restartSuppressedCount;
            data->lastRestartSuppressedIteration =
                data->iteration;
            data->lastRestartSuppressedNonImproving =
                data->nbIterNonImpro;
            std::cout
                << "  [Intercepted] Restart suppressed by remaining "
                   "budget. Remaining Iters: "
                << remainingIterations
                << ", Remaining Time: " << remainingTime
                << "s. Stagnation counter preserved."
                << std::endl;
            hgasp::RunLogger::event(
                hgasp::LogLevel::info,
                "search",
                "restart_suppressed",
                {},
                {
                    {"iteration", data->iteration},
                    {"non_improving",
                        data->nbIterNonImpro},
                    {"remaining_iterations",
                        remainingIterations},
                    {"remaining_cpu_seconds",
                        std::isfinite(remainingTime)
                            ? remainingTime
                            : -1.0},
                    {"cpu_limit_enabled",
                        data->timeLimit > 0},
                    {"minimum_remaining_iterations",
                        data->restartMinimumRemainingIterations},
                    {"minimum_remaining_cpu_seconds",
                        data->restartMinimumRemainingCpuSeconds},
                    {"iteration_budget_insufficient",
                        decision.iterationBudgetInsufficient},
                    {"cpu_budget_insufficient",
                        decision.cpuBudgetInsufficient},
                    {"suppression_count",
                        static_cast<unsigned long long>(
                            data->restartSuppressedCount)},
                });
        }
        return false;
    }

    restartSuppressionLatched = false;
    std::cout << "  [Triggered] Executing Global Restart..."
              << std::endl;
    searchAllocator.resetAllPopulations();
    traceObservations.clear();
    if (staticConfigurationRegistry != nullptr) {
        // These maps feed generation-local reduction and Local/Global SP
        // telemetry. A restart increments every live population token, so
        // retaining the old slot-indexed credit would assign evidence from a
        // retired generation to its successor.
        selectedMasterColumnsByPopulation.clear();
        localSelectedMasterColumnsByPopulation.clear();
        staticConfigurationRegistry->restartActive(
            currentPhysicalAttempt(),
            0U);
        const std::uint64_t productiveAttempt =
            static_cast<std::uint64_t>(std::max(0, data->iteration));
        for (const auto& slot :
             staticConfigurationRegistry->slots()) {
            if (!slot.active) {
                continue;
            }
            StaticPopulationRuntimeState& runtime =
                staticPopulationRuntime[slot.slot];
            runtime = {};
            runtime.generation = slot.generation;
            runtime.createdAttempt = productiveAttempt;
            runtime.lastBestImprovementAttempt = productiveAttempt;
        }
        const std::size_t freeIndex =
            staticConfigurationRegistry
                ->unrestrictedPopulationIndex();
        StaticPopulationRuntimeState& freeRuntime =
            staticPopulationRuntime[freeIndex];
        if (freeRuntime.generation ==
            std::numeric_limits<std::uint64_t>::max()) {
            throw std::overflow_error(
                "unrestricted population generation overflow");
        }
        const std::uint64_t nextGeneration =
            freeRuntime.generation + 1U;
        freeRuntime = {};
        freeRuntime.generation = nextGeneration;
        freeRuntime.createdAttempt = productiveAttempt;
        freeRuntime.lastBestImprovementAttempt =
            freeRuntime.createdAttempt;
        staticRestartNotice = true;
    }
    if (challengerRuntime.active()) {
        rejectActiveChallenger("search_restart", pool);
    }
    const int preservedFreePopulationIndex =
        staticConfigurationRegistry == nullptr
            ? -1
            : static_cast<int>(
                  staticConfigurationRegistry
                      ->unrestrictedPopulationIndex());
    popman->reStart(preservedFreePopulationIndex);
    initialPop(pool);
    materializeStaticHgampPopulations(pool);
    if (hasFeasibleIncumbent() && checkStopCondition()) {
        return true;
    }

    std::cout << "[RoutePool] 正在建立初始精英池基线..."
              << std::endl;
    for (int i = 0;
         i < data->numLocConfiguration + 1;
         ++i) {
        for (Individual* indiv : popman->popula[i]) {
            if (indiv->isFeasible) {
                pool->extractRoutes(indiv);
            }
        }
    }
    getAverageValue(-1);
    return false;
}

void RouteGenerator::checkSubPopReduction() {
    if (data->staticHgampPopulationsEnabled()) {
        return;
    }
    if (data->iteration == data->iterLimit/2)
        popman->reduceHalfSubPop();

    if (data->iteration == data->iterLimit * 0.75)
        popman->reduceHalfSubPop();
}


std::vector<hgasp::PopulationLocalSpToken>
RouteGenerator::reduceStaticPopulations(const int phase) {
    if (staticConfigurationRegistry == nullptr ||
        (phase != 1 && phase != 2)) {
        throw std::logic_error(
            "static population reduction requires a materialized registry "
            "and phase 1 or 2");
    }
    std::vector<hgasp::PopulationReductionCandidate> candidates;
    candidates.reserve(
        staticConfigurationRegistry->slots().size() + 1U);
    const std::uint64_t attempt = static_cast<std::uint64_t>(
        currentPhysicalAttempt());
    for (const auto& slot : staticConfigurationRegistry->slots()) {
        hgasp::PopulationReductionCandidate candidate;
        candidate.populationIndex = slot.slot;
        candidate.active = slot.active;
        candidate.token = slot.token();
        candidate.configuration = slot.configuration;
        candidate.protectedPopulation =
            slot.active && attempt < slot.protectedUntilAttempt;
        if (slot.active) {
            const SubPopulation& population =
                popman->popula[slot.slot];
            if (population.empty()) {
                throw std::logic_error(
                    "active static population is empty at reduction");
            }
            refreshStaticRuntime(static_cast<int>(slot.slot));
            const StaticPopulationRuntimeState& runtime =
                staticPopulationRuntime[slot.slot];
            candidate.bestObjective = population.front()->fit;
            candidate.hgaImprovementPerCpu =
                runtime.hgaCpuSeconds > 0.0
                    ? runtime.hgaImprovement /
                          runtime.hgaCpuSeconds
                    : 0.0;
            candidate.diversity =
                populationDiversity(static_cast<int>(slot.slot));
            const auto selected =
                selectedMasterColumnsByPopulation.find(
                    static_cast<int>(slot.slot));
            candidate.globalSpSelectedColumnContribution =
                selected == selectedMasterColumnsByPopulation.end()
                    ? 0.0
                    : selected->second;
        }
        candidates.push_back(std::move(candidate));
    }
    hgasp::PopulationReductionCandidate unrestricted;
    unrestricted.populationIndex =
        staticConfigurationRegistry->unrestrictedPopulationIndex();
    unrestricted.active = true;
    unrestricted.unrestricted = true;
    candidates.push_back(unrestricted);

    const hgasp::PopulationReductionPolicy policy({
        data->populationReductionObjectiveWeight,
        data->populationReductionHgaImprovementWeight,
        data->populationReductionDiversityWeight,
        data->populationReductionGlobalSpCreditWeight,
    });
    const std::size_t survivorTarget = static_cast<std::size_t>(
        phase == 1
            ? data->populationReduction1SurvivorCount
            : data->populationReduction2SurvivorCount);
    const hgasp::PopulationReductionDecision decision =
        policy.reduceToTarget(candidates, survivorTarget);
    const std::size_t freePopulationSizeBefore =
        refreshPermanentFreePopulationFloor();

    // Prepare every generation and any active challenger evidence-boundary
    // update on value copies. A stale index, empty resident, allocation
    // failure, or archive rejection therefore leaves live state untouched.
    hgasp::StaticConfigurationRegistry preparedRegistry =
        *staticConfigurationRegistry;
    std::optional<hgasp::ConfigurationCandidateArchive> preparedArchive;
    if (data->singleDynamicChallengerEffective()) {
        preparedArchive.emplace(candidateArchive);
    }
    std::vector<hgasp::PopulationLocalSpToken> retired;
    retired.reserve(decision.removedPopulationIndices.size());
    std::uint64_t retirementCooldownUntil = attempt;
    if (preparedArchive.has_value()) {
        const std::uint64_t protectionAttempts =
            static_cast<std::uint64_t>(
                data->retiredConfigurationCooldownAttempts);
        retirementCooldownUntil =
            protectionAttempts >
                    std::numeric_limits<std::uint64_t>::max() - attempt
                ? std::numeric_limits<std::uint64_t>::max()
                : attempt + protectionAttempts;
    }
    for (const std::size_t index : decision.removedPopulationIndices) {
        if (index >= preparedRegistry.slots().size() ||
            index >= popman->popula.size()) {
            throw std::logic_error(
                "static reduction selected an invalid resident slot");
        }
        const auto& slot = preparedRegistry.slots()[index];
        if (!slot.active || popman->popula[index].empty()) {
            throw std::logic_error(
                "static reduction selected an inactive or empty resident");
        }
        const auto token = slot.token();
        if (preparedArchive.has_value()) {
            double retiredBest = std::numeric_limits<double>::infinity();
            for (const Individual* individual : popman->popula[index]) {
                if (individual == nullptr ||
                    !std::isfinite(individual->fit)) {
                    throw std::logic_error(
                        "static reduction resident objective is invalid");
                }
                retiredBest = std::min(retiredBest, individual->fit);
            }
            preparedArchive->recordResidentRetirement(
                slot.configuration,
                attempt,
                retirementCooldownUntil,
                retiredBest);
        }
        preparedRegistry.deactivate(token);
        retired.push_back({token.slot, token.generation});
    }

    // No-throw commit of metadata precedes destructive resident cleanup.
    staticConfigurationRegistry->swap(preparedRegistry);
    if (preparedArchive.has_value()) {
        candidateArchive.swap(*preparedArchive);
    }
    for (const std::size_t index : decision.removedPopulationIndices) {
        SubPopulation& population = popman->popula[index];
        for (Individual* individual : population) {
            delete individual;
        }
        population.clear();
        selectedMasterColumnsByPopulation.erase(
            static_cast<int>(index));
        localSelectedMasterColumnsByPopulation.erase(
            static_cast<int>(index));
    }
    requirePermanentFreePopulationPreserved(
        freePopulationSizeBefore);

    std::ostringstream removed;
    for (std::size_t index = 0U;
         index < decision.removedPopulationIndices.size();
         ++index) {
        if (index != 0U) {
            removed << ',';
        }
        removed << decision.removedPopulationIndices[index];
    }
    std::ostringstream surviving;
    for (std::size_t index = 0U;
         index < decision.survivingPopulationIndices.size();
         ++index) {
        if (index != 0U) {
            surviving << ',';
        }
        surviving << decision.survivingPopulationIndices[index];
    }
    hgasp::RunLogger::event(
        hgasp::LogLevel::info,
        "population",
        "population_reduction_phase_" + std::to_string(phase),
        {},
        {
            {"iteration", data->iteration},
            {"phase", phase},
            {"fixed_before", static_cast<unsigned long long>(
                decision.fixedPopulationCountBefore)},
            {"fixed_after", static_cast<unsigned long long>(
                decision.actualFixedSurvivorCount)},
            {"free_population_retained", true},
            {"removed_populations", removed.str()},
            {"surviving_populations", surviving.str()},
        });
    hgasp::recordV114ComponentActivation(
        phase == 1
            ? hgasp::V114Component::PopulationReductionPhase1
            : hgasp::V114Component::PopulationReductionPhase2);
    ++staticReductionCount;
    return retired;
}

std::vector<RouteGenerator::StaticPopulationRuntimeView>
RouteGenerator::staticPopulationRuntimeViews() const {
    std::vector<StaticPopulationRuntimeView> views;
    if (staticConfigurationRegistry == nullptr) {
        return views;
    }
    views.reserve(staticConfigurationRegistry->slots().size() + 1U);
    for (const auto& slot : staticConfigurationRegistry->slots()) {
        StaticPopulationRuntimeView view;
        view.populationIndex = static_cast<int>(slot.slot);
        view.active = slot.active;
        view.localSpToken = {slot.slot, slot.generation};
        view.configuration = slot.configuration;
        if (slot.slot < staticPopulationRuntime.size()) {
            const auto& runtime = staticPopulationRuntime[slot.slot];
            view.createdAttempt = runtime.createdAttempt;
            view.lastBestImprovementAttempt =
                runtime.lastBestImprovementAttempt;
            view.verifiedGlobalInjectionCount =
                runtime.verifiedGlobalInjectionCount;
            view.bestObjective = runtime.bestObjective;
        }
        views.push_back(std::move(view));
    }
    const std::size_t freeIndex =
        staticConfigurationRegistry->unrestrictedPopulationIndex();
    StaticPopulationRuntimeView free;
    free.populationIndex = static_cast<int>(freeIndex);
    free.active = freeIndex < popman->popula.size() &&
        !popman->popula[freeIndex].empty();
    free.unrestricted = true;
    if (freeIndex < staticPopulationRuntime.size()) {
        const auto& runtime = staticPopulationRuntime[freeIndex];
        free.localSpToken = {freeIndex, runtime.generation};
        free.createdAttempt = runtime.createdAttempt;
        free.lastBestImprovementAttempt =
            runtime.lastBestImprovementAttempt;
        free.verifiedGlobalInjectionCount =
            runtime.verifiedGlobalInjectionCount;
        free.bestObjective = runtime.bestObjective;
    }
    if (free.active) {
        free.configuration = configurationKey(
            *popman->popula[freeIndex].front());
    }
    views.push_back(std::move(free));
    return views;
}

std::optional<RouteGenerator::StaticPopulationReplacementOutcome>
RouteGenerator::rotateBestFreeConfiguration(
        RoutePool* const pool,
        const std::string& source) {
    if (staticConfigurationRegistry == nullptr ||
        !data->parameters.configurationPortfolio
             .scheduledConfigurationRotation) {
        return std::nullopt;
    }
    const std::size_t freeIndex =
        staticConfigurationRegistry->unrestrictedPopulationIndex();
    if (freeIndex >= popman->popula.size()) {
        return std::nullopt;
    }
    const std::optional<hgasp::DepotConfigurationKey> globalConfig =
        global != nullptr && global->isFeasible && global->fit < Max
        ? std::optional<hgasp::DepotConfigurationKey>(
              configurationKey(*global))
        : std::nullopt;
    Individual* founder = nullptr;
    hgasp::DepotConfigurationKey founderConfiguration;
    for (Individual* candidate : popman->popula[freeIndex]) {
        if (candidate == nullptr || !candidate->isFeasible) {
            continue;
        }
        const hgasp::DepotConfigurationKey configuration =
            configurationKey(*candidate);
        if (staticConfigurationRegistry
                ->tokenForConfiguration(configuration)
                .has_value() ||
            (globalConfig.has_value() &&
             configuration == *globalConfig)) {
            continue;
        }
        if (founder == nullptr || candidate->fit < founder->fit - Min) {
            founder = candidate;
            founderConfiguration = configuration;
        }
    }
    if (founder == nullptr) {
        return std::nullopt;
    }
    return promoteConfiguration(
        founderConfiguration,
        hgasp::StaticReplacementAuthority::ScheduledExplorationLease,
        founder,
        pool,
        source,
        false,
        false);
}

RouteGenerator::ScheduledConfigurationRotationResult
RouteGenerator::rotateConfigurationFromFounders(
        const std::vector<ScheduledConfigurationFounder>& founders,
        RoutePool* const pool,
        const std::string& source) {
    ScheduledConfigurationRotationResult result;
    const auto logSkipped = [&]() noexcept {
        hgasp::RunLogger::event(
            result.status == ScheduledConfigurationRotationStatus::
                    RecoverablePromotionFailure
                ? hgasp::LogLevel::warning
                : hgasp::LogLevel::info,
            "population",
            "scheduled_configuration_rotation_skipped",
            {},
            {
                {"iteration", data->iteration},
                {"source", source},
                {"reason", result.reason},
                {"candidates_considered", static_cast<unsigned long long>(
                    result.candidatesConsidered)},
                {"resident_duplicates_skipped", static_cast<unsigned long long>(
                    result.residentDuplicatesSkipped)},
            });
    };
    if (staticConfigurationRegistry == nullptr ||
        !data->parameters.configurationPortfolio
             .scheduledConfigurationRotation) {
        result.status = ScheduledConfigurationRotationStatus::Disabled;
        result.reason = "scheduled_rotation_disabled";
        logSkipped();
        return result;
    }
    if (founders.empty()) {
        result.status = ScheduledConfigurationRotationStatus::NoCandidates;
        result.reason = "no_exploratory_founder";
        logSkipped();
        return result;
    }

    bool recoverableFailureObserved = false;
    for (const ScheduledConfigurationFounder& candidate : founders) {
        ++result.candidatesConsidered;
        std::unique_ptr<Individual> founder(new Individual());
        founder->define(data);
        founder->rebuildFromSP(candidate.routes);
        founder->decideFeasible();
        if (!founder->isFeasible) {
            throw std::logic_error(
                "scheduled configuration founder rebuilt infeasibly");
        }
        const hgasp::DepotConfigurationKey configuration =
            configurationKey(*founder);
        if (configuration != candidate.configuration) {
            throw std::logic_error(
                "scheduled configuration founder identity changed during "
                "reconstruction");
        }
        if (staticConfigurationRegistry
                ->tokenForConfiguration(configuration)
                .has_value()) {
            ++result.residentDuplicatesSkipped;
            continue;
        }

        hgasp::PromotionFailureKind failure =
            hgasp::PromotionFailureKind::None;
        std::string reason;
        auto replacement = promoteConfiguration(
            configuration,
            hgasp::StaticReplacementAuthority::ScheduledExplorationLease,
            founder.get(),
            pool,
            source,
            false,
            false,
            &failure,
            &reason);
        if (replacement.has_value()) {
            result.status = ScheduledConfigurationRotationStatus::Rotated;
            result.replacement = std::move(replacement);
            result.reason = "rotated";
            return result;
        }
        if (failure == hgasp::PromotionFailureKind::None ||
            failure == hgasp::PromotionFailureKind::ContractViolation) {
            throw std::logic_error(
                "scheduled configuration rotation returned no successor "
                "without a recoverable failure");
        }
        recoverableFailureObserved = true;
        result.reason = reason.empty()
            ? hgasp::toString(failure)
            : reason;
    }

    result.status = recoverableFailureObserved
        ? ScheduledConfigurationRotationStatus::RecoverablePromotionFailure
        : ScheduledConfigurationRotationStatus::AllCandidatesAlreadyResident;
    if (result.reason.empty()) {
        result.reason = recoverableFailureObserved
            ? "all_promotions_failed_recoverably"
            : "all_founders_already_resident";
    }
    logSkipped();
    return result;
}

void RouteGenerator::recordVerifiedGlobalInjection(
        const int populationIndex) {
    if (populationIndex < 0 ||
        static_cast<std::size_t>(populationIndex) >=
            staticPopulationRuntime.size()) {
        return;
    }
    ++staticPopulationRuntime[
        static_cast<std::size_t>(populationIndex)]
        .verifiedGlobalInjectionCount;
    refreshStaticRuntime(populationIndex);
}

bool RouteGenerator::consumeStaticRestartNotice() noexcept {
    const bool value = staticRestartNotice;
    staticRestartNotice = false;
    return value;
}

void RouteGenerator::finalizeSearch() {
    if (finalComponentSummaryEmitted) {
        return;
    }
    if (!data->searchTermination.internallyConsistent()) {
        throw std::logic_error(
            "cannot finalize search without a consistent termination record");
    }
    hgasp::RunLogger::event(
        hgasp::LogLevel::info,
        "search",
        "termination_recorded",
        {},
        {
            {"requested_iterations",
                data->searchTermination.requestedIterations},
            {"completed_iterations",
                data->searchTermination.completedIterations},
            {"termination_reason",
                std::string(hgasp::toString(
                    data->searchTermination.reason))},
            {"early_stop_evidence_class",
                std::string(hgasp::toString(
                    data->searchTermination.evidenceClass))},
            {"lp_rigorous_no_improvement",
                data->searchTermination.lpRigorousNoImprovement},
            {"mip_attempted",
                data->searchTermination.mipAttempted},
            {"mip_improved",
                data->searchTermination.mipImproved},
        });
    if (!finalComponentSummaryEmitted) {
            std::size_t nonemptyPopulations = 0U;
            for (const SubPopulation& population : popman->popula) {
                nonemptyPopulations += population.empty() ? 0U : 1U;
            }
            std::ostringstream columnCounts;
            std::vector<std::pair<int, std::uint64_t>> orderedCounts(
                newMasterColumnsByPrimaryPopulation.begin(),
                newMasterColumnsByPrimaryPopulation.end());
            std::sort(orderedCounts.begin(), orderedCounts.end());
            for (std::size_t index = 0U;
                 index < orderedCounts.size();
                 ++index) {
                if (index != 0U) {
                    columnCounts << ',';
                }
                columnCounts << orderedCounts[index].first << ':'
                             << orderedCounts[index].second;
            }
            std::ostringstream localCallCounts;
            std::vector<std::pair<std::string, std::uint64_t>>
                orderedLocalCalls(
                    localSpCallsByScope.begin(),
                    localSpCallsByScope.end());
            std::sort(
                orderedLocalCalls.begin(), orderedLocalCalls.end());
            for (std::size_t index = 0U;
                 index < orderedLocalCalls.size();
                 ++index) {
                if (index != 0U) {
                    localCallCounts << ',';
                }
                localCallCounts << orderedLocalCalls[index].first << ':'
                                << orderedLocalCalls[index].second;
            }
            std::ostringstream selectedColumnCounts;
            std::vector<std::pair<int, double>> orderedSelectedCounts(
                selectedMasterColumnsByPopulation.begin(),
                selectedMasterColumnsByPopulation.end());
            std::sort(
                orderedSelectedCounts.begin(),
                orderedSelectedCounts.end());
            for (std::size_t index = 0U;
                 index < orderedSelectedCounts.size();
                 ++index) {
                if (index != 0U) {
                    selectedColumnCounts << ',';
                }
                selectedColumnCounts
                    << orderedSelectedCounts[index].first << ':'
                    << orderedSelectedCounts[index].second;
            }
            std::ostringstream localSelectedColumnCounts;
            std::vector<std::pair<int, double>>
                orderedLocalSelectedCounts(
                    localSelectedMasterColumnsByPopulation.begin(),
                    localSelectedMasterColumnsByPopulation.end());
            std::sort(
                orderedLocalSelectedCounts.begin(),
                orderedLocalSelectedCounts.end());
            for (std::size_t index = 0U;
                 index < orderedLocalSelectedCounts.size();
                 ++index) {
                if (index != 0U) {
                    localSelectedColumnCounts << ',';
                }
                localSelectedColumnCounts
                    << orderedLocalSelectedCounts[index].first << ':'
                    << orderedLocalSelectedCounts[index].second;
            }
            std::ostringstream fallbackReasons;
            std::vector<std::pair<std::string, std::uint64_t>>
                orderedFallbackReasons(
                    sameConfigurationFallbackReasons.begin(),
                    sameConfigurationFallbackReasons.end());
            std::sort(
                orderedFallbackReasons.begin(),
                orderedFallbackReasons.end());
            for (std::size_t index = 0U;
                 index < orderedFallbackReasons.size();
                 ++index) {
                if (index != 0U) {
                    fallbackReasons << ',';
                }
                fallbackReasons
                    << orderedFallbackReasons[index].first << ':'
                    << orderedFallbackReasons[index].second;
            }
            std::ostringstream survivingConfigurations;
            std::ostringstream staticPopulationMetrics;
            std::size_t initialFixedPopulationCount = 0U;
            std::size_t finalFixedPopulationCount = 0U;
            if (staticConfigurationRegistry != nullptr) {
                initialFixedPopulationCount =
                    staticConfigurationRegistry->fixedSlotCount();
                finalFixedPopulationCount =
                    staticConfigurationRegistry
                        ->actualFixedPopulationCount();
                bool firstSurvivor = true;
                for (const auto& slot :
                     staticConfigurationRegistry->slots()) {
                    if (!slot.active) {
                        continue;
                    }
                    if (!firstSurvivor) {
                        survivingConfigurations << ';';
                        staticPopulationMetrics << ';';
                    }
                    firstSurvivor = false;
                    survivingConfigurations
                        << slot.slot << '@' << slot.generation << '='
                        << configurationLabel(slot.configuration);
                    const SubPopulation& population =
                        popman->popula[slot.slot];
                    double mean = 0.0;
                    for (const Individual* individual : population) {
                        mean += individual->fit;
                    }
                    mean = population.empty()
                        ? Max
                        : mean / static_cast<double>(population.size());
                    const StaticPopulationRuntimeState& runtime =
                        staticPopulationRuntime[slot.slot];
                    staticPopulationMetrics
                        << slot.slot << '@' << slot.generation
                        << ":best=" << runtime.bestObjective
                        << ":mean=" << mean
                        << ":diversity="
                        << populationDiversity(
                               static_cast<int>(slot.slot))
                        << ":hga_improvement="
                        << runtime.hgaImprovement
                        << ":hga_cpu=" << runtime.hgaCpuSeconds;
                }
            }
            const int freeIndex = staticConfigurationRegistry != nullptr
                ? static_cast<int>(
                      staticConfigurationRegistry
                          ->unrestrictedPopulationIndex())
                : 0;
            const std::size_t freeMembers =
                freeIndex >= 0 &&
                        static_cast<std::size_t>(freeIndex) <
                            popman->popula.size()
                    ? popman->popula[
                          static_cast<std::size_t>(freeIndex)].size()
                    : 0U;
            std::ostringstream challengerColumnCounts;
            bool firstChallengerGeneration = true;
            for (const auto& entry :
                 challengerSelectedColumnsByGeneration) {
                if (!firstChallengerGeneration) {
                    challengerColumnCounts << ';';
                }
                firstChallengerGeneration = false;
                challengerColumnCounts << entry.first << ':'
                                       << entry.second;
            }
            const auto historicalSelectionSummary = [](
                    const std::map<std::uint64_t, double>& selections) {
                std::ostringstream encoded;
                double total = 0.0;
                bool first = true;
                for (const auto& entry : selections) {
                    if (!first) {
                        encoded << ';';
                    }
                    first = false;
                    encoded << entry.first << ':' << entry.second;
                    total += entry.second;
                }
                return std::make_pair(encoded.str(), total);
            };
            const auto activeHistoricalSelections =
                historicalSelectionSummary(
                    activeHistoricalChallengerSelectionsByGeneration);
            const auto retiredHistoricalSelections =
                historicalSelectionSummary(
                    retiredHistoricalChallengerSelectionsByGeneration);
            std::size_t configurationsWithTrials = 0U;
            std::size_t configurationsWithResidentRetirements = 0U;
            for (const auto& entry : candidateArchive.trialHistory()) {
                const auto* observation = candidateArchive
                    .findObservationHistory(entry.first);
                const auto& history = entry.second;
                configurationsWithTrials += history.totalTrials > 0U
                    ? 1U
                    : 0U;
                configurationsWithResidentRetirements +=
                    history.totalResidentRetirements > 0U ? 1U : 0U;
                hgasp::RunLogger::event(
                    hgasp::LogLevel::info,
                    "population",
                    "candidate_trial_history",
                    {},
                    {
                        {"iteration", data->iteration},
                        {"configuration", configurationLabel(entry.first)},
                        {"trials", static_cast<unsigned long long>(
                            history.totalTrials)},
                        {"promotions", static_cast<unsigned long long>(
                            history.totalPromotions)},
                        {"rejections", static_cast<unsigned long long>(
                            history.totalRejections)},
                        {"last_trial_generation",
                            static_cast<unsigned long long>(
                                history.lastTrialGeneration)},
                        {"last_rejection_attempt",
                            static_cast<unsigned long long>(
                                history.lastRejectionAttempt)},
                        {"cooldown_until_attempt",
                            static_cast<unsigned long long>(
                                history.cooldownUntilAttempt)},
                        {"observations_at_rejection",
                            static_cast<unsigned long long>(
                                history.totalObservationsAtLastRejection)},
                        {"current_observations",
                            static_cast<unsigned long long>(
                                observation == nullptr
                                    ? 0U
                                    : observation->totalObservations)},
                        {"best_at_rejection",
                            std::isfinite(
                                history.bestObjectiveAtLastRejection)
                                ? history.bestObjectiveAtLastRejection
                                : -1.0},
                        {"resident_retirements",
                            static_cast<unsigned long long>(
                                history.totalResidentRetirements)},
                        {"last_retirement_attempt",
                            static_cast<unsigned long long>(
                                history.lastResidentRetirementAttempt)},
                        {"retirement_cooldown_until_attempt",
                            static_cast<unsigned long long>(
                                history.retirementCooldownUntilAttempt)},
                        {"observations_at_retirement",
                            static_cast<unsigned long long>(
                                history.totalObservationsAtLastRetirement)},
                        {"best_at_retirement",
                            std::isfinite(
                                history.bestObjectiveAtLastRetirement)
                                ? history.bestObjectiveAtLastRetirement
                                : -1.0},
                        {"latest_evidence_boundary_attempt",
                            static_cast<unsigned long long>(
                                history.latestEvidenceBoundaryAttempt)},
                        {"latest_evidence_cooldown_until_attempt",
                            static_cast<unsigned long long>(
                                history.latestEvidenceCooldownUntilAttempt)},
                        {"observations_at_latest_evidence_boundary",
                            static_cast<unsigned long long>(
                                history
                                    .totalObservationsAtLatestEvidenceBoundary)},
                        {"best_at_latest_evidence_boundary",
                            std::isfinite(
                                history.bestObjectiveAtLatestEvidenceBoundary)
                                ? history.bestObjectiveAtLatestEvidenceBoundary
                                : -1.0},
                        {"best_after_latest_evidence_boundary",
                            std::isfinite(
                                history
                                    .bestObjectiveAfterLatestEvidenceBoundary)
                                ? history
                                    .bestObjectiveAfterLatestEvidenceBoundary
                                : -1.0},
                        {"current_best", observation == nullptr
                            ? -1.0
                            : observation->bestObjective},
                        {"ever_promoted", history.everPromoted},
                    });
            }
            std::ostringstream discoveryObservedBySource;
            std::ostringstream discoveryFreeAdmittedBySource;
            for (std::size_t sourceIndex = 0U;
                 sourceIndex < hgasp::configurationDiscoverySourceCount();
                 ++sourceIndex) {
                if (sourceIndex != 0U) {
                    discoveryObservedBySource << ';';
                    discoveryFreeAdmittedBySource << ';';
                }
                const auto source = static_cast<
                    hgasp::ConfigurationDiscoverySource>(sourceIndex);
                discoveryObservedBySource
                    << hgasp::toString(source) << ':'
                    << configurationDiscoveriesBySource[sourceIndex];
                discoveryFreeAdmittedBySource
                    << hgasp::toString(source) << ':'
                    << configurationFreeAdmissionsBySource[sourceIndex];
            }
            hgasp::RunLogger::event(
                hgasp::LogLevel::info,
                "population",
                "configuration_discovery_summary",
                {},
                {
                    {"iteration", data->iteration},
                    {"observed_by_source",
                        discoveryObservedBySource.str()},
                    {"free_admitted_by_source",
                        discoveryFreeAdmittedBySource.str()},
                    {"unchanged", static_cast<unsigned long long>(
                        configurationChangesByKind[static_cast<std::size_t>(
                            hgasp::ConfigurationChangeKind::Unchanged)])},
                    {"open", static_cast<unsigned long long>(
                        configurationChangesByKind[static_cast<std::size_t>(
                            hgasp::ConfigurationChangeKind::Open)])},
                    {"close", static_cast<unsigned long long>(
                        configurationChangesByKind[static_cast<std::size_t>(
                            hgasp::ConfigurationChangeKind::Close)])},
                    {"swap", static_cast<unsigned long long>(
                        configurationChangesByKind[static_cast<std::size_t>(
                            hgasp::ConfigurationChangeKind::Swap)])},
                    {"mixed", static_cast<unsigned long long>(
                        configurationChangesByKind[static_cast<std::size_t>(
                            hgasp::ConfigurationChangeKind::Mixed)])},
                });
            hgasp::RunLogger::event(
                hgasp::LogLevel::info,
                "search",
                "population_component_summary",
                {},
                {
                    {"iteration", data->iteration},
                    {"actual_nonempty_population_count",
                        static_cast<unsigned long long>(
                            nonemptyPopulations)},
                    {"free_population_members",
                        static_cast<unsigned long long>(freeMembers)},
                    {"cross_population_matings",
                        static_cast<unsigned long long>(
                            crossPopulationMatings)},
                    {"static_mating_attempts",
                        static_cast<unsigned long long>(
                            staticMatingAttempts)},
                    {"cross_configuration_matings",
                        static_cast<unsigned long long>(
                            crossConfigurationMatings)},
                    {"same_configuration_fallback_matings",
                        static_cast<unsigned long long>(
                            sameConfigurationFallbackMatings)},
                    {"cross_configuration_mating_rate",
                        staticMatingAttempts == 0U
                            ? 0.0
                            : static_cast<double>(
                                  crossConfigurationMatings) /
                                  static_cast<double>(
                                      staticMatingAttempts)},
                    {"same_configuration_fallback_reasons",
                        fallbackReasons.str()},
                    {"initial_fixed_population_count",
                        static_cast<unsigned long long>(
                            initialFixedPopulationCount)},
                    {"final_fixed_population_count",
                        static_cast<unsigned long long>(
                            finalFixedPopulationCount)},
                    {"static_population_replacements",
                        static_cast<unsigned long long>(
                            staticReplacementCount)},
                    {"static_population_reductions",
                        static_cast<unsigned long long>(
                            staticReductionCount)},
                    {"surviving_configurations",
                        survivingConfigurations.str()},
                    {"static_population_metrics",
                        staticPopulationMetrics.str()},
                    {"true_unique_nonresident_configurations_seen",
                        static_cast<unsigned long long>(
                            uniqueNonresidentConfigurationsSeen)},
                    {"candidate_working_records_created",
                        static_cast<unsigned long long>(
                            candidateRecordsCreated)},
                    {"candidate_working_records_recreated",
                        static_cast<unsigned long long>(
                            candidateRecordsRecreated)},
                    {"candidate_working_records_evicted",
                        static_cast<unsigned long long>(
                            candidateRecordsEvicted)},
                    {"candidate_working_records_capacity_rejected",
                        static_cast<unsigned long long>(
                            candidateRecordsCapacityRejected)},
                    {"candidate_archive_size",
                        static_cast<unsigned long long>(
                            candidateArchive.size())},
                    {"candidate_archive_capacity",
                        static_cast<unsigned long long>(
                            candidateArchive.policy().maximumRecords)},
                    {"configs_with_trials",
                        static_cast<unsigned long long>(
                            configurationsWithTrials)},
                    {"configs_with_resident_retirements",
                        static_cast<unsigned long long>(
                            configurationsWithResidentRetirements)},
                    {"candidate_records_eligible",
                        static_cast<unsigned long long>(
                            candidateRecordsEligible)},
                    {"candidate_eligibility_exceptional",
                        static_cast<unsigned long long>(
                            candidateEligibilityExceptional)},
                    {"candidate_eligibility_repeated_quality",
                        static_cast<unsigned long long>(
                            candidateEligibilityRepeatedQuality)},
                    {"candidate_rejected_by_quality_floor",
                        static_cast<unsigned long long>(
                            candidateRejectedByQualityFloor)},
                    {"candidate_blocked_by_cooldown",
                        static_cast<unsigned long long>(
                            candidateBlockedByCooldown)},
                    {"candidate_blocked_no_replaceable_resident",
                        static_cast<unsigned long long>(
                            candidateBlockedNoReplaceableResident)},
                    {"challenger_active", challengerActive()},
                    {"challenger_generation",
                        static_cast<unsigned long long>(
                            challengerGeneration)},
                    {"challenger_trials_started",
                        static_cast<unsigned long long>(
                            challengerTrialsStarted)},
                    {"challenger_trials_completed",
                        static_cast<unsigned long long>(
                            challengerTrialsCompleted)},
                    {"challenger_retries_after_cooldown",
                        static_cast<unsigned long long>(
                            challengerRetriesAfterCooldown)},
                    {"challenger_promotions",
                        static_cast<unsigned long long>(
                            challengerPromotions)},
                    {"challenger_rejections",
                        static_cast<unsigned long long>(
                            challengerRejections)},
                    {"challenger_fill_failures",
                        static_cast<unsigned long long>(
                            challengerFillFailures)},
                    {"global_sp_direct_promotions",
                        static_cast<unsigned long long>(
                            globalSpDirectPromotions)},
                    {"global_sp_promoted_active_challenger",
                        static_cast<unsigned long long>(
                            globalSpPromotedActiveChallenger)},
                    {"challenger_fill_cpu_seconds",
                        challengerFillCpuSeconds},
                    {"challenger_trial_attributed_hga_cpu_seconds",
                        challengerTrialHgaCpuSeconds},
                    {"global_sp_selected_columns_by_challenger_generation",
                        challengerColumnCounts.str()},
                    {"global_sp_selected_routes_from_active_challenger",
                        activeHistoricalSelections.second},
                    {"global_sp_selected_routes_from_retired_challengers",
                        retiredHistoricalSelections.second},
                    {"global_sp_active_challenger_route_origins",
                        activeHistoricalSelections.first},
                    {"global_sp_retired_challenger_route_origins",
                        retiredHistoricalSelections.first},
                    {"new_master_columns_by_population",
                        columnCounts.str()},
                    {"new_master_column_attribution",
                        "primary_population"},
                    {"selected_columns_by_population",
                        selectedColumnCounts.str()},
                    {"global_sp_selected_columns_by_population",
                        selectedColumnCounts.str()},
                    {"local_sp_selected_columns_by_population",
                        localSelectedColumnCounts.str()},
                    {"local_sp_calls_by_scope",
                        localCallCounts.str()},
                });
            finalComponentSummaryEmitted = true;
        }
        global->isRight();
        global->outputBestSolution();
        std::cout<<global->fit<<std::endl;
        std::cout<<"----The memetic algorithm is finished, CPU time spends: "
                 <<data->elapsedCpuSeconds()<<std::endl;
        return;
}

bool RouteGenerator::checkStopCondition() {
    if (!data->stopCondition(global->fit)) {
        return false;
    }
    finalizeSearch();
    return true;
}

bool RouteGenerator::hasFeasibleIncumbent() const noexcept {
    return global != nullptr &&
           global->isFeasible &&
           global->fit < Max;
}

const Individual* RouteGenerator::globalIncumbent() const noexcept {
    return hasFeasibleIncumbent() ? global.get() : nullptr;
}


void RouteGenerator::setSpBudgetLedger(
        hgasp::SpBudgetLedger* const ledger) noexcept {
    spBudgetLedger = ledger;
}

std::uint64_t RouteGenerator::beginOffspringTrace(
        const int firstPopulationIndex,
        const int secondPopulationIndex) {
    if (nextTraceId ==
        std::numeric_limits<std::uint64_t>::max()) {
        throw std::overflow_error(
            "offspring trace identifier exhausted");
    }
    if (!lastBatchParentLineage.initialized ||
        lastBatchParentLineage.first.populationIndex !=
            firstPopulationIndex ||
        lastBatchParentLineage.second.populationIndex !=
            secondPopulationIndex) {
        throw std::logic_error(
            "offspring trace does not match its frozen parent batch");
    }
    const std::uint64_t traceId = nextTraceId++;
    traceObservations.emplace(
        traceId,
        TraceObservation{
            0.0,
            firstPopulationIndex,
            secondPopulationIndex,
            lastBatchParentLineage.first.populationGeneration,
            lastBatchParentLineage.second.populationGeneration,
        });
    return traceId;
}

hgasp::RouteSubmissionContext
RouteGenerator::submissionContext(
        const std::uint64_t traceId,
        const int firstPopulationIndex,
        const int secondPopulationIndex,
        const int childIndex) const {
    if (!lastBatchParentLineage.initialized ||
        lastBatchParentLineage.first.populationIndex !=
            firstPopulationIndex ||
        lastBatchParentLineage.second.populationIndex !=
            secondPopulationIndex) {
        throw std::logic_error(
            "offspring submission does not match its frozen parent batch");
    }
    const hgasp::ParentLineageSnapshot& first =
        lastBatchParentLineage.first;
    const hgasp::ParentLineageSnapshot& second =
        lastBatchParentLineage.second;
    hgasp::RouteSubmissionContext context;
    context.traceId = traceId;
    context.origin =
        lastMutationCommitted
            ? hgasp::RouteOriginKind::Mutation
            : hgasp::RouteOriginKind::EaxOffspring;
    context.iteration = currentPhysicalAttemptForProvenance();
    context.firstPopulation = first.populationIndex;
    context.secondPopulation = second.populationIndex;
    context.firstPopulationRole = first.populationRole;
    context.secondPopulationRole = second.populationRole;
    context.firstOriginConfiguration = first.actualConfiguration;
    context.secondOriginConfiguration = second.actualConfiguration;
    context.firstPopulationGeneration = first.populationGeneration;
    context.secondPopulationGeneration = second.populationGeneration;
    context.firstChallengerSlot = first.challengerSlot;
    context.firstChallengerGeneration = first.challengerGeneration;
    context.secondChallengerSlot = second.challengerSlot;
    context.secondChallengerGeneration = second.challengerGeneration;
    context.firstParentIndividualId = first.individualId;
    context.secondParentIndividualId = second.individualId;
    context.mutationCommitted = lastMutationCommitted;
    if (!first.spAncestryCallId.empty()) {
        context.spAncestryCallId =
            first.spAncestryCallId;
    } else {
        context.spAncestryCallId =
            second.spAncestryCallId;
    }
    if (childIndex >= 0) {
        if (static_cast<std::size_t>(childIndex) >=
            childConfigurationDiscovery.size()) {
            throw std::out_of_range(
                "offspring submission child index is out of range");
        }
        const hgasp::ConfigurationDiscoveryTrace& discovery =
            childConfigurationDiscovery[
                static_cast<std::size_t>(childIndex)];
        if (discovery.initialized) {
            hgasp::enrichConfigurationDiscoverySubmission(
                context,
                discovery.beforeConfiguration,
                discovery.afterConfiguration,
                discovery.source,
                discovery.contributors,
                discovery.evidenceUnitId);
        }
    }
    return context;
}

void RouteGenerator::reportDelayedRouteCredit(
        const std::vector<hgasp::OriginCredit>& credits,
        const std::string& sourceCallId,
        const hgasp::SpFeedbackScope scope) {
    double totalCredit = 0.0;
    std::size_t credited = 0U;
    for (const hgasp::OriginCredit& credit : credits) {
        const auto trace = traceObservations.find(credit.traceId);
        if (scope == hgasp::SpFeedbackScope::Global &&
            credit.historicalChallengerSelectedColumns > 0.0) {
            const int activeSlot = challengerActive()
                ? challengerPopulationIndex()
                : -1;
            const std::uint64_t activeGeneration = challengerActive()
                ? challengerRuntime.token.generation
                : 0U;
            auto& historicalSelections =
                hgasp::isActiveHistoricalChallengerOrigin(
                    credit, activeSlot, activeGeneration)
                    ? activeHistoricalChallengerSelectionsByGeneration
                    : retiredHistoricalChallengerSelectionsByGeneration;
            historicalSelections[credit.challengerGeneration] +=
                credit.historicalChallengerSelectedColumns;
        }

        bool currentOrigin =
            credit.population >= 0 &&
            static_cast<std::size_t>(credit.population) <
                popman->popula.size();
        if (currentOrigin && credit.challengerGeneration != 0U) {
            currentOrigin = challengerActive() &&
                credit.population == challengerPopulationIndex() &&
                credit.challengerGeneration ==
                    challengerRuntime.token.generation;
        }
        if (currentOrigin && data->staticHgampPopulationsEnabled() &&
            staticConfigurationRegistry != nullptr) {
            currentOrigin = trace != traceObservations.end();
            if (currentOrigin) {
                const bool first = credit.population ==
                    trace->second.firstPopulation;
                const bool second = credit.population ==
                    trace->second.secondPopulation;
                const std::uint64_t originGeneration = first
                    ? trace->second.firstPopulationGeneration
                    : (second
                           ? trace->second.secondPopulationGeneration
                           : 0U);
                const std::size_t index =
                    static_cast<std::size_t>(credit.population);
                std::uint64_t currentGeneration = 0U;
                if (index <
                    staticConfigurationRegistry->slots().size()) {
                    const auto& slot =
                        staticConfigurationRegistry->slots()[index];
                    currentGeneration = slot.active
                        ? slot.generation
                        : 0U;
                } else if (
                    index == staticConfigurationRegistry
                        ->unrestrictedPopulationIndex() &&
                    index < staticPopulationRuntime.size()) {
                    currentGeneration =
                        staticPopulationRuntime[index].generation;
                } else if (
                    static_cast<int>(index) ==
                        challengerPopulationIndex() &&
                    challengerRuntime.active()) {
                    currentGeneration =
                        challengerRuntime.token.generation;
                }
                currentOrigin = originGeneration != 0U &&
                    originGeneration == currentGeneration;
            }
        }

        if (currentOrigin && credit.selectedColumns > 0.0) {
            auto& selectedColumns =
                scope == hgasp::SpFeedbackScope::Global
                    ? selectedMasterColumnsByPopulation
                    : localSelectedMasterColumnsByPopulation;
            selectedColumns[credit.population] += credit.selectedColumns;
            if (scope == hgasp::SpFeedbackScope::Global &&
                credit.challengerGeneration != 0U) {
                challengerSelectedColumnsByGeneration[
                    credit.challengerGeneration] +=
                        credit.selectedColumns;
            }
        }
        if (trace == traceObservations.end() ||
            !currentOrigin || credit.credit <= 0.0) {
            continue;
        }
        const hgasp::PopulationCreditRole creditRole =
            credit.population == trace->second.firstPopulation
                ? hgasp::PopulationCreditRole::Primary
                : hgasp::PopulationCreditRole::Mate;
        searchAllocator.reportUsefulColumnCredit(
            credit.population,
            credit.credit,
            trace->second.cpuSeconds,
            creditRole);
        totalCredit += credit.credit;
        ++credited;
    }
    if (credited > 0U) {
        hgasp::RunLogger::event(
            hgasp::LogLevel::info,
            "sp",
            "delayed_credit",
            {},
            {
                {"iteration", data->iteration},
                {"credited_origins",
                    static_cast<unsigned long long>(credited)},
                {"useful_credit", totalCredit},
                {"source_call_id", sourceCallId},
            });
    }
}


void RouteGenerator::reportSearchOutcome(
        const int firstPopulationIndex,
        const int secondPopulationIndex,
        const double objectiveImprovement,
        const double cpuSeconds,
        const std::size_t newColumns,
        const std::uint64_t traceId) {
    if (!lastBatchParentLineage.initialized ||
        lastBatchParentLineage.first.populationIndex !=
            firstPopulationIndex ||
        lastBatchParentLineage.second.populationIndex !=
            secondPopulationIndex) {
        throw std::logic_error(
            "search outcome does not match its frozen parent batch");
    }
    const bool firstParentCurrent = isCurrentParentLineage(
        lastBatchParentLineage.first);
    const bool secondParentCurrent = isCurrentParentLineage(
        lastBatchParentLineage.second);
    if (firstParentCurrent && firstPopulationIndex >= 0 &&
        newColumns > 0U) {
        newMasterColumnsByPrimaryPopulation[firstPopulationIndex] +=
            static_cast<std::uint64_t>(newColumns);
    }
    if (traceId != 0U) {
        const auto trace = traceObservations.find(traceId);
        if (trace == traceObservations.end() ||
            trace->second.firstPopulation != firstPopulationIndex ||
            trace->second.secondPopulation != secondPopulationIndex ||
            trace->second.firstPopulationGeneration !=
                lastBatchParentLineage.first.populationGeneration ||
            trace->second.secondPopulationGeneration !=
                lastBatchParentLineage.second.populationGeneration) {
            throw std::logic_error(
                "search outcome trace lineage is missing or mismatched");
        }
        trace->second.cpuSeconds = std::max(0.0, cpuSeconds);
        constexpr std::size_t maximumTraceHistory = 100000U;
        if (traceObservations.size() > maximumTraceHistory) {
            const std::uint64_t oldest =
                traceId >= maximumTraceHistory
                    ? traceId - maximumTraceHistory
                    : 0U;
            traceObservations.erase(oldest);
        }
    }
    if (challengerRuntime.state ==
            hgasp::ChallengerState::ProtectedTrial &&
        ((firstParentCurrent &&
          lastBatchParentLineage.first.populationRole ==
              hgasp::RoutePopulationRole::DynamicChallenger) ||
         (secondParentCurrent &&
          lastBatchParentLineage.second.populationRole ==
              hgasp::RoutePopulationRole::DynamicChallenger))) {
        challengerRuntime.trialHgaCpuSeconds +=
            std::max(0.0, cpuSeconds);
        challengerTrialHgaCpuSeconds +=
            std::max(0.0, cpuSeconds);
        const auto objectives = populationObjectives(
            challengerPopulationIndex());
        if (!objectives.empty()) {
            challengerRuntime.bestObjective = *std::min_element(
                objectives.begin(), objectives.end());
        }
    }
    if (data->staticHgampPopulationsEnabled() &&
        staticConfigurationRegistry != nullptr) {
        std::vector<int> owners;
        if (firstParentCurrent && firstPopulationIndex >= 0 &&
            static_cast<std::size_t>(firstPopulationIndex) <
                staticPopulationRuntime.size()) {
            owners.push_back(firstPopulationIndex);
        }
        if (secondParentCurrent && secondPopulationIndex >= 0 &&
            secondPopulationIndex != firstPopulationIndex &&
            static_cast<std::size_t>(secondPopulationIndex) <
                staticPopulationRuntime.size()) {
            owners.push_back(secondPopulationIndex);
        }
        const double cpuShare = owners.empty()
            ? 0.0
            : std::max(0.0, cpuSeconds) /
                  static_cast<double>(owners.size());
        const double improvementShare = owners.empty()
            ? 0.0
            : std::max(0.0, objectiveImprovement) /
                  static_cast<double>(owners.size());
        for (const int owner : owners) {
            StaticPopulationRuntimeState& runtime =
                staticPopulationRuntime[
                    static_cast<std::size_t>(owner)];
            runtime.hgaCpuSeconds += cpuShare;
            // Population-best refreshes also observe SP admissions for the
            // Local-SP stagnation clock, but must not turn those admissions
            // into HGA productivity credit used by reduction.
            runtime.hgaImprovement += improvementShare;
            refreshStaticRuntime(owner);
        }
    }
    if (!data->adaptivePopulationAllocationEffective()) {
        return;
    }
    hgasp::SearchArmObservation observation;
    observation.objectiveImprovement =
        std::max(0.0, objectiveImprovement);
    observation.cpuSeconds = std::max(0.0, cpuSeconds);
    observation.newColumns =
        static_cast<double>(newColumns);
    // The allocator has no generation-qualified key. Suppress the whole
    // report if either physical slot was recycled during this child, rather
    // than crediting a successor population for its predecessor's work.
    if (firstParentCurrent && secondParentCurrent) {
        searchAllocator.reportMatingOutcome(
            firstPopulationIndex,
            secondPopulationIndex,
            observation);
    }
    spJustImproved = false;
}

void RouteGenerator::reportLocalSpCall(const std::string& scope) {
    if (!scope.empty()) {
        ++localSpCallsByScope[scope];
    }
}

void RouteGenerator::reportSpImprovement(
        const bool improved) {
    if (improved) {
        spJustImproved = true;
    }
}

double RouteGenerator::populationDiversity(
        const int populationIndex) const {
    if (populationIndex < 0 ||
        static_cast<std::size_t>(populationIndex) >=
            popman->popula.size()) {
        return 0.0;
    }
    const SubPopulation& population =
        popman->popula[
            static_cast<std::size_t>(populationIndex)];
    if (population.size() <= 1U) {
        return 0.0;
    }
    double total = 0.0;
    for (Individual* individual : population) {
        total += individual->averageBrokenPairsDistanceClosest(
            std::min(
                data->nbClost,
                static_cast<int>(population.size()) - 1));
    }
    // The inherited HGAMP implementation stores broken-pairs proximity as a
    // non-positive score: structurally different solutions have a larger
    // negative magnitude. Convert that convention to a [0,1] diversity
    // signal before feeding the allocation controller.
    const double normalizer =
        static_cast<double>(
            std::max(1, 20 * data->numV));
    return std::clamp(
        -total /
            static_cast<double>(population.size()) /
            normalizer,
        0.0,
        1.0);
}

bool RouteGenerator::timeLimitReached() const noexcept {
    if (data->timeLimit <= 0)
        return false;
    const double elapsedTime = data->elapsedCpuSeconds();
    return elapsedTime >= data->timeLimit;
}

bool RouteGenerator::initializationStopRequested() const {
    return hasFeasibleIncumbent() &&
        data->stopCondition(global->fit);
}

void RouteGenerator::outputprocess(Individual *indiv){
	const int reportingDecimals =
		hgasp::objectiveReportingDecimals(
			data->problemProfile().reporting);
    std::cout<<"It: "<<data->iteration<<"--G-Fit: "
             <<std::setiosflags(ios::fixed)
             <<std::setprecision(reportingDecimals)
             <<hgasp::roundObjectiveForReporting(
				 global->fit,
				 data->problemProfile().reporting);
    std::cout<<"  --R-Fit: "
             <<hgasp::roundObjectiveForReporting(
				 bestIndivRestart->fit,
				 data->problemProfile().reporting);
    std::cout<<"  Curr Sol--"
             <<hgasp::roundObjectiveForReporting(
				 indiv->fit,
				 data->problemProfile().reporting);
    std::cout<<"  IT NoImpro--"<<data->nbIterNonImpro;
    std::cout<<"  Time comsuming--"
             <<std::setprecision(2)
             <<data->elapsedCpuSeconds();
    std::cout<<std::endl;
    data->loopp = 0;
}

void RouteGenerator::printProgress(
        const RoutePool* pool) {
    if (data->iteration %
            data->checkpointIntervalAttempts == 0) {
        outputprocess(global.get());
        std::size_t populationMembers = 0U;
        for (const SubPopulation& population :
             popman->popula) {
            populationMembers += population.size();
        }
        const double elapsedCpu = data->elapsedCpuSeconds();
        const double spFamilyCpu =
            spBudgetLedger != nullptr
                ? spBudgetLedger->totalCpuSeconds()
                : 0.0;
        const double spGuardableCpu =
            spBudgetLedger != nullptr
                ? spBudgetLedger->guardableCpuSeconds()
                : 0.0;
        const double spMandatoryExemptCpu =
            spBudgetLedger != nullptr
                ? spBudgetLedger->mandatoryExemptCpuSeconds()
                : 0.0;
        hgasp::RunLogger::event(
            hgasp::LogLevel::info,
            "search",
            "checkpoint",
            {},
            {
                {"iteration", data->iteration},
                {"global_objective_raw", global->fit},
                {"restart_objective_raw",
                    bestIndivRestart->fit},
                {"non_improving", data->nbIterNonImpro},
                {"last_improvement_iteration",
                    std::max(
                        0,
                        data->iteration -
                            data->nbIterNonImpro)},
                {"cpu_seconds", elapsedCpu},
                {"master_columns",
                    static_cast<unsigned long long>(
                        pool != nullptr
                            ? pool->getPool().size()
                            : 0U)},
                {"search_archive_routes",
                    static_cast<unsigned long long>(
                        pool != nullptr
                            ? pool->getSearchArchive().size()
                            : 0U)},
                {"master_version",
                    static_cast<unsigned long long>(
                        pool != nullptr
                            ? pool->getMasterVersion()
                            : 0U)},
                {"population_count",
                    static_cast<unsigned long long>(
                        popman->popula.size())},
                {"population_members",
                    static_cast<unsigned long long>(
                        populationMembers)},
                {"sp_family_cpu_seconds", spFamilyCpu},
                {"sp_family_cpu_share",
                    elapsedCpu > 0.0
                        ? spFamilyCpu / elapsedCpu
                        : 0.0},
                {"sp_guardable_cpu_seconds", spGuardableCpu},
                {"sp_guardable_cpu_share",
                    elapsedCpu > 0.0
                        ? spGuardableCpu / elapsedCpu
                        : 0.0},
                {"sp_mandatory_exempt_cpu_seconds",
                    spMandatoryExemptCpu},
                {"sp_mandatory_exempt_cpu_share",
                    elapsedCpu > 0.0
                        ? spMandatoryExemptCpu / elapsedCpu
                        : 0.0},
                {"sp_local_cpu_seconds",
                    spBudgetLedger != nullptr
                        ? spBudgetLedger->localCpuSeconds()
                        : 0.0},
                {"sp_global_cpu_seconds",
                    spBudgetLedger != nullptr
                        ? spBudgetLedger->globalCpuSeconds()
                        : 0.0},
                {"sp_lp_probe_cpu_seconds",
                    spBudgetLedger != nullptr
                        ? spBudgetLedger->phaseCpuSeconds(
                              hgasp::SpBudgetPhase::LpProbe)
                        : 0.0},
                {"sp_local_mip_cpu_seconds",
                    spBudgetLedger != nullptr
                        ? spBudgetLedger->phaseCpuSeconds(
                              hgasp::SpBudgetPhase::LocalMip)
                        : 0.0},
                {"sp_global_mip_cpu_seconds",
                    spBudgetLedger != nullptr
                        ? spBudgetLedger->phaseCpuSeconds(
                              hgasp::SpBudgetPhase::GlobalMip)
                        : 0.0},
                {"sp_final_global_cpu_seconds",
                    spBudgetLedger != nullptr
                        ? spBudgetLedger->phaseCpuSeconds(
                              hgasp::SpBudgetPhase::FinalGlobal)
                        : 0.0},
                {"sp_guardable_lp_probe_cpu_seconds",
                    spBudgetLedger != nullptr
                        ? spBudgetLedger->guardablePhaseCpuSeconds(
                              hgasp::SpBudgetPhase::LpProbe)
                        : 0.0},
                {"sp_guardable_local_mip_cpu_seconds",
                    spBudgetLedger != nullptr
                        ? spBudgetLedger->guardablePhaseCpuSeconds(
                              hgasp::SpBudgetPhase::LocalMip)
                        : 0.0},
                {"sp_guardable_global_mip_cpu_seconds",
                    spBudgetLedger != nullptr
                        ? spBudgetLedger->guardablePhaseCpuSeconds(
                              hgasp::SpBudgetPhase::GlobalMip)
                        : 0.0},
                {"sp_guardable_final_global_cpu_seconds",
                    spBudgetLedger != nullptr
                        ? spBudgetLedger->guardablePhaseCpuSeconds(
                              hgasp::SpBudgetPhase::FinalGlobal)
                        : 0.0},
                {"challenger_active", challengerRuntime.active()},
                {"challenger_generation",
                    static_cast<unsigned long long>(
                        challengerRuntime.token.generation)},
                {"challenger_archive_records",
                    static_cast<unsigned long long>(
                        candidateArchive.size())},
                {"candidate_archive_capacity",
                    static_cast<unsigned long long>(
                        candidateArchive.policy().maximumRecords)},
                {"true_unique_nonresident_configurations_seen",
                    static_cast<unsigned long long>(
                        uniqueNonresidentConfigurationsSeen)},
                {"candidate_working_records_created",
                    static_cast<unsigned long long>(
                        candidateRecordsCreated)},
                {"candidate_working_records_recreated",
                    static_cast<unsigned long long>(
                        candidateRecordsRecreated)},
                {"candidate_working_records_evicted",
                    static_cast<unsigned long long>(
                        candidateRecordsEvicted)},
                {"candidate_working_records_capacity_rejected",
                    static_cast<unsigned long long>(
                        candidateRecordsCapacityRejected)},
                {"candidate_eligibility_exceptional",
                    static_cast<unsigned long long>(
                        candidateEligibilityExceptional)},
                {"candidate_eligibility_repeated_quality",
                    static_cast<unsigned long long>(
                        candidateEligibilityRepeatedQuality)},
                {"candidate_rejected_by_quality_floor",
                    static_cast<unsigned long long>(
                        candidateRejectedByQualityFloor)},
                {"candidate_blocked_by_cooldown",
                    static_cast<unsigned long long>(
                        candidateBlockedByCooldown)},
                {"candidate_blocked_no_replaceable_resident",
                    static_cast<unsigned long long>(
                        candidateBlockedNoReplaceableResident)},
                {"challenger_trials_started",
                    static_cast<unsigned long long>(
                        challengerTrialsStarted)},
                {"challenger_trials_completed",
                    static_cast<unsigned long long>(
                        challengerTrialsCompleted)},
                {"challenger_promotions",
                    static_cast<unsigned long long>(
                        challengerPromotions)},
                {"challenger_rejections",
                    static_cast<unsigned long long>(
                        challengerRejections)},
                {"challenger_fill_cpu_seconds",
                    challengerFillCpuSeconds},
            });
        hgasp::RunLogger::status(
            "search",
            "running",
            {
                {"iteration", data->iteration},
                {"global_objective_raw", global->fit},
                {"cpu_seconds", elapsedCpu},
                {"master_columns",
                    static_cast<unsigned long long>(
                        pool != nullptr
                            ? pool->getPool().size()
                            : 0U)},
                {"sp_family_cpu_share",
                    elapsedCpu > 0.0
                        ? spFamilyCpu / elapsedCpu
                        : 0.0},
                {"sp_guardable_cpu_share",
                    elapsedCpu > 0.0
                        ? spGuardableCpu / elapsedCpu
                        : 0.0},
            });
    }
    
    if (data->iteration > 0 && data->iteration % 50000 == 0) {
        std::cout << ">>> [机制监控] \"gamma+1\" 游离种群动态逆袭替换机制累计生效: " 
                    << popman->reverseReplacementCount << " 次 | "
                    << "当前游离种群个体数量: " 
                    << popman->popula[data->numLocConfiguration].size() << std::endl;
    }
}
