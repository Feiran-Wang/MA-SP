/******************************************************************************
 * File: RoutePool.cpp
 * Author: Feiran Wang
 * Date: 2026-06-25
 * Description: 
 *   Central pool for deduplicating and retaining feasible routes.
 ******************************************************************************/

#include "RoutePool.h"
#include "Individual.h"
#include "RouteEvaluator.h"
#include "RunLogger.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>

std::size_t MasterColumnKeyHash::operator()(
        const MasterColumnKey& key) const noexcept {
    std::size_t seed = key.customers.size();
    seed ^= std::hash<int>()(key.locId) +
            0x9e3779b9U +
            (seed << 6U) +
            (seed >> 2U);
    for (const int customer : key.customers) {
        seed ^= std::hash<int>()(customer) +
                0x9e3779b9U +
                (seed << 6U) +
                (seed >> 2U);
    }
    return seed;
}

RoutePool::RoutePool(read_data* data) {
    this->data = data;
    this->maxPoolSize =
        std::max<std::size_t>(1U, data->spMaxPoolSize);
}

RoutePool::~RoutePool() {
    uniqueSearchRoutes.clear();
    searchArchive.clear();
    bestArchivedColumns.clear();
    masterColumnIndex.clear();
    poolList.clear();
    backboneKeys.clear();
    globalIncumbentKeys.clear();
    lastSpIncumbentKeys.clear();
    populationEliteKeys.clear();
    latestCustomerDuals.clear();
    latestReducedCosts.clear();
}

MasterColumnKey RoutePool::masterKey(
        const StandaloneRoute& route) {
    MasterColumnKey key;
    key.locId = route.locId;
    key.customers = route.sequence;
    std::sort(key.customers.begin(), key.customers.end());
    return key;
}

bool RoutePool::preferredMasterRoute(
        const StandaloneRoute& candidate,
        const StandaloneRoute& incumbent) {
    constexpr double costTolerance = 1e-9;
    return candidate.routeCost <
               incumbent.routeCost - costTolerance ||
        (std::fabs(candidate.routeCost - incumbent.routeCost) <=
             costTolerance &&
         candidate.sequence < incumbent.sequence);
}

void RoutePool::admitMasterColumn(
        const StandaloneRoute& route) {
    MasterColumnKey key = masterKey(route);
    const auto existing = masterColumnIndex.find(key);
    if (existing == masterColumnIndex.end()) {
        const std::size_t index = poolList.size();
        poolList.push_back(route);
        masterColumnIndex.emplace(std::move(key), index);
        ++masterVersion;
        return;
    }

    StandaloneRoute& incumbent = poolList[existing->second];
    if (preferredMasterRoute(route, incumbent)) {
        incumbent = route;
        ++dominanceReplacementCount;
        ++masterVersion;
    } else {
        ++dominanceDiscardCount;
    }
}

bool RoutePool::archiveOrderedRoute(
        const StandaloneRoute& route) {
    const auto insertion = uniqueSearchRoutes.insert(route);
    if (!insertion.second) {
        return false;
    }
    if (searchArchive.size() >= maxPoolSize) {
        // SearchRouteArchive is diagnostic/search provenance, not the SP
        // master. Refuse excess history while still allowing the route to
        // compete for the bounded master below.
        uniqueSearchRoutes.erase(insertion.first);
        ++searchArchiveDiscardCount;
        return true;
    }
    searchArchive.push_back(route);
    return true;
}

StandaloneRoute RoutePool::archivedBestOrCandidate(
        const MasterColumnKey& key,
        const StandaloneRoute& candidate) {
    const auto found = bestArchivedColumns.find(key);
    if (found != bestArchivedColumns.end()) {
        if (preferredMasterRoute(candidate, found->second)) {
            found->second = candidate;
        }
        return found->second;
    }
    if (bestArchivedColumns.size() < maxPoolSize) {
        bestArchivedColumns.emplace(key, candidate);
    } else {
        ++bestArchiveDiscardCount;
    }
    return candidate;
}

RoutePool::ProtectedKeySet& RoutePool::protectionSet(
        const RouteProtection protection) {
    switch (protection) {
        case RouteProtection::Backbone:
            return backboneKeys;
        case RouteProtection::GlobalIncumbent:
            return globalIncumbentKeys;
        case RouteProtection::LastSpIncumbent:
            return lastSpIncumbentKeys;
        case RouteProtection::PopulationElite:
            return populationEliteKeys;
    }
    throw std::invalid_argument("unknown route-protection class");
}

bool RoutePool::isProtected(const MasterColumnKey& key) const {
    return backboneKeys.count(key) != 0U ||
        globalIncumbentKeys.count(key) != 0U ||
        lastSpIncumbentKeys.count(key) != 0U ||
        populationEliteKeys.count(key) != 0U;
}

void RoutePool::setProtectedRoutes(
        const RouteProtection protection,
        const std::vector<StandaloneRoute>& routes) {
    ProtectedKeySet replacement;
    replacement.reserve(routes.size());
    for (const StandaloneRoute& route : routes) {
        MasterColumnKey key = masterKey(route);
        replacement.insert(key);

        archiveOrderedRoute(route);
        const StandaloneRoute archivedBest =
            archivedBestOrCandidate(key, route);
        if (masterColumnIndex.find(key) ==
            masterColumnIndex.end()) {
            admitMasterColumn(archivedBest);
        } else {
            admitMasterColumn(route);
        }
    }
    protectionSet(protection) = std::move(replacement);
}

void RoutePool::pruneIfNeeded() {
    if (poolList.size() > maxPoolSize) {
        prunePool();
    }
}

void RoutePool::updateDualFeedback(
        const std::vector<StandaloneRoute>& snapshot,
        const std::vector<double>& customerDuals,
        const std::vector<double>& routeReducedCosts) {
    if (routeReducedCosts.size() != snapshot.size()) {
        throw std::invalid_argument(
            "route reduced costs do not match the solved pool snapshot");
    }
    if (customerDuals.size() !=
        static_cast<std::size_t>(data->numV)) {
        throw std::invalid_argument(
            "customer duals do not match the instance customer count");
    }
    latestCustomerDuals = customerDuals;
    latestReducedCosts.clear();
    latestReducedCosts.reserve(snapshot.size());
    for (std::size_t index = 0; index < snapshot.size(); ++index) {
        latestReducedCosts[masterKey(snapshot[index])] =
            routeReducedCosts[index];
    }
}

double RoutePool::reducedCostFor(
        const StandaloneRoute& route) const noexcept {
    const auto found =
        latestReducedCosts.find(masterKey(route));
    return found == latestReducedCosts.end()
        ? std::numeric_limits<double>::infinity()
        : found->second;
}

void RoutePool::rebuildMasterIndex() {
    masterColumnIndex.clear();
    masterColumnIndex.reserve(poolList.size());
    for (std::size_t index = 0; index < poolList.size(); ++index) {
        masterColumnIndex.emplace(masterKey(poolList[index]), index);
    }
}

void RoutePool::submitFeasibleIndividual(const Individual* indiv) {
    if (indiv == nullptr) {
        throw std::invalid_argument(
            "route-pool submission requires a non-null individual");
    }
    if (!indiv->isFeasible) {
        throw std::invalid_argument(
            "route-pool submission requires a feasible individual");
    }
    extractRoutes(indiv);
    totalFitnessSum += indiv->fit;
    ++feasibleSubmissionCount;
}

std::vector<StandaloneRoute> RoutePool::exportRoutes(
        const Individual* indiv) const {
    if (indiv == nullptr) {
        throw std::invalid_argument(
            "route export requires a non-null individual");
    }
    if (!indiv->isFeasible) {
        throw std::invalid_argument(
            "route export requires a feasible individual");
    }

    std::vector<StandaloneRoute> exported;
    for (int i = 0; i < data->numL; i++) {
        if (indiv->location[i].used) {
            for (int j = 0; j < indiv->location[i].maxVisit; j++) {
                if (indiv->location[i].depots[j].route->used && 
                    indiv->location[i].depots[j].route->nbClients > 0) {
                    
                    StandaloneRoute sr;
                    sr.locId = i;
                    sr.totalLoad = 0;
                    
                    // routeCost contains travel cost only. Vehicle and depot
                    // fixed costs are added by the SP objective.
                    sr.routeCost = 0.0;
                    const Node* current =
                        indiv->location[i].depots[j].next;
                    const Node* previous =
                        &indiv->location[i].depots[j];

                    while (!current->isDepot) {
                        sr.sequence.push_back(current->cour);
                        sr.totalLoad += current->dem;
                        // Preserve route direction for asymmetric instances.
                        sr.routeCost +=
                            data->D[previous->cour][current->cour];
                        
                        previous = current;
                        current = current->next;
                    }
                    sr.routeCost +=
                        data->D[previous->cour][current->cour];

                    if (data->problemMode ==
                        hgasp::ProblemMode::ClrpTw) {
                        const HardTimeWindowConstraint hardTimeWindows;
                        const RouteEvaluation evaluation =
                            RouteEvaluator(*data, hardTimeWindows).evaluate(
                                i, sr.sequence);
                        if (!evaluation.feasible()) {
                            throw std::runtime_error(
                                "route-pool admission rejected an infeasible "
                                "CLRP-TW route");
                        }
                        if (evaluation.load !=
                                indiv->location[i].depots[j].route->load) {
                            throw std::logic_error(
                                "route-pool admission found a stale route "
                                "load aggregate");
                        }
                        sr.totalLoad = evaluation.load;
                        sr.routeCost = evaluation.travelCost;
                        sr.returnTime = evaluation.returnTime;
                        sr.totalWaitingTime =
                            evaluation.totalWaitingTime;
                        sr.temporalFeasible =
                            evaluation.temporalFeasible;
                        sr.temporalViolationCustomer =
                            evaluation.violatingCustomer;
                    }
                    exported.push_back(std::move(sr));
                }
            }
        }
    }
    return exported;
}

void RoutePool::extractRoutes(const Individual* indiv) {
    // Callers admit only feasible individuals; each used non-empty route is
    // converted to the value representation shared with the SP solver.
    for (const StandaloneRoute& sr : exportRoutes(indiv)) {
        const bool newOrderedRoute = archiveOrderedRoute(sr);
        MasterColumnKey key = masterKey(sr);
        const StandaloneRoute archivedBest =
            archivedBestOrCandidate(key, sr);

        // Reconsider a new ordered route, and restore the cheapest archived
        // variant if bounded-pool pruning removed this key.
        if (newOrderedRoute ||
            masterColumnIndex.find(key) ==
                masterColumnIndex.end()) {
            admitMasterColumn(
                masterColumnIndex.find(key) ==
                        masterColumnIndex.end()
                    ? archivedBest
                    : sr);
        }
    }
    pruneIfNeeded();
}

void RoutePool::prunePool() {
    std::cout << "[RoutePool] Pruning pool. Current size: " << poolList.size() << std::endl;
    hgasp::RunLogger::event(
        hgasp::LogLevel::info,
        "route_pool",
        "prune_started",
        {},
        {
            {"columns", static_cast<unsigned long long>(
                poolList.size())},
            {"archive", static_cast<unsigned long long>(
                searchArchive.size())},
        });

    const std::size_t targetSize =
        std::max<std::size_t>(1, maxPoolSize * 4 / 5);
    std::vector<bool> retained(poolList.size(), false);
    std::vector<std::size_t> retainedIndices;
    retainedIndices.reserve(
        std::max(targetSize, poolList.size()));

    auto retain = [&](const std::size_t index) {
        if (retained[index]) {
            return false;
        }
        retained[index] = true;
        retainedIndices.push_back(index);
        return true;
    };

    lastPruneProtectedCount = 0U;
    lastPruneRarityCount = 0U;
    lastPruneDepotDiversityCount = 0U;
    lastPruneOrdinaryCount = 0U;

    // Tier 0: complete feasible backbones and current HGA/SP/population
    // incumbents are never removed, even if they temporarily exceed the
    // nominal target.
    for (std::size_t index = 0; index < poolList.size(); ++index) {
        if (isProtected(masterKey(poolList[index])) &&
            retain(index)) {
            ++lastPruneProtectedCount;
        }
    }

    std::vector<int> availability(data->numV, 0);
    std::vector<int> retainedCoverage(data->numV, 0);
    for (std::size_t index = 0; index < poolList.size(); ++index) {
        for (const int customer : poolList[index].sequence) {
            ++availability[customer];
            if (retained[index]) {
                ++retainedCoverage[customer];
            }
        }
    }

    auto efficiency = [](const StandaloneRoute& route) {
        return route.routeCost > 1.0e-9
            ? static_cast<double>(route.totalLoad) /
                  route.routeCost
            : 1.0e30;
    };
    auto reducedCost = [&](const StandaloneRoute& route) {
        return reducedCostFor(route);
    };
    std::vector<std::size_t> candidates(poolList.size());
    for (std::size_t index = 0; index < poolList.size(); ++index) {
        candidates[index] = index;
    }

    // Tier 1: protect sparse customer coverage. Columns that cover customers
    // with few alternatives rank first; each customer keeps up to two
    // alternatives when the target budget permits.
    auto rarityScore = [&](const StandaloneRoute& route) {
        double score = 0.0;
        for (const int customer : route.sequence) {
            score += 1.0 /
                static_cast<double>(
                    std::max(1, availability[customer]));
        }
        return score;
    };
    std::stable_sort(
        candidates.begin(),
        candidates.end(),
        [&](const std::size_t left, const std::size_t right) {
            const double leftRarity =
                rarityScore(poolList[left]);
            const double rightRarity =
                rarityScore(poolList[right]);
            if (std::fabs(leftRarity - rightRarity) > 1.0e-12) {
                return leftRarity > rightRarity;
            }
            const double leftReducedCost =
                reducedCost(poolList[left]);
            const double rightReducedCost =
                reducedCost(poolList[right]);
            if (std::isfinite(leftReducedCost) !=
                std::isfinite(rightReducedCost)) {
                return std::isfinite(leftReducedCost);
            }
            if (std::isfinite(leftReducedCost) &&
                std::isfinite(rightReducedCost) &&
                std::fabs(
                    leftReducedCost - rightReducedCost) >
                    1.0e-12) {
                return leftReducedCost < rightReducedCost;
            }
            const double leftEfficiency =
                efficiency(poolList[left]);
            const double rightEfficiency =
                efficiency(poolList[right]);
            if (std::fabs(leftEfficiency - rightEfficiency) >
                1.0e-12) {
                return leftEfficiency > rightEfficiency;
            }
            return masterKey(poolList[left]).customers <
                masterKey(poolList[right]).customers;
        });
    for (const std::size_t index : candidates) {
        if (retainedIndices.size() >= targetSize) {
            break;
        }
        bool needed = false;
        for (const int customer : poolList[index].sequence) {
            const int desired =
                std::min(2, availability[customer]);
            if (retainedCoverage[customer] < desired) {
                needed = true;
                break;
            }
        }
        if (!needed || !retain(index)) {
            continue;
        }
        ++lastPruneRarityCount;
        for (const int customer : poolList[index].sequence) {
            ++retainedCoverage[customer];
        }
    }

    // Tier 2: keep at least one column per represented depot.
    std::vector<bool> retainedDepot(data->numL, false);
    for (const std::size_t index : retainedIndices) {
        retainedDepot[poolList[index].locId] = true;
    }
    std::stable_sort(
        candidates.begin(),
        candidates.end(),
        [&](const std::size_t left, const std::size_t right) {
            const double leftReducedCost =
                reducedCost(poolList[left]);
            const double rightReducedCost =
                reducedCost(poolList[right]);
            if (std::isfinite(leftReducedCost) !=
                std::isfinite(rightReducedCost)) {
                return std::isfinite(leftReducedCost);
            }
            if (std::isfinite(leftReducedCost) &&
                std::fabs(
                    leftReducedCost - rightReducedCost) >
                    1.0e-12) {
                return leftReducedCost < rightReducedCost;
            }
            const double leftEfficiency =
                efficiency(poolList[left]);
            const double rightEfficiency =
                efficiency(poolList[right]);
            if (std::fabs(leftEfficiency - rightEfficiency) >
                1.0e-12) {
                return leftEfficiency > rightEfficiency;
            }
            return poolList[left].routeCost <
                poolList[right].routeCost;
        });
    for (const std::size_t index : candidates) {
        if (retainedIndices.size() >= targetSize) {
            break;
        }
        const int depot = poolList[index].locId;
        if (retainedDepot[depot] || !retain(index)) {
            continue;
        }
        retainedDepot[depot] = true;
        ++lastPruneDepotDiversityCount;
    }

    // Tier 3: fill remaining capacity by route efficiency.
    for (const std::size_t index : candidates) {
        if (retainedIndices.size() >= targetSize) {
            break;
        }
        if (retain(index)) {
            ++lastPruneOrdinaryCount;
        }
    }

    std::vector<StandaloneRoute> retainedPool;
    retainedPool.reserve(retainedIndices.size());
    for (const std::size_t index : retainedIndices) {
        retainedPool.push_back(std::move(poolList[index]));
    }
    poolList = std::move(retainedPool);

    rebuildMasterIndex();
    
    std::cout << "[RoutePool] Pruning complete. New size: "
              << poolList.size()
              << " tiers={protected:" << lastPruneProtectedCount
              << ",rarity:" << lastPruneRarityCount
              << ",depot:" << lastPruneDepotDiversityCount
              << ",ordinary:" << lastPruneOrdinaryCount
              << "}" << std::endl;
    hgasp::RunLogger::event(
        hgasp::LogLevel::info,
        "route_pool",
        "prune_finished",
        {},
        {
            {"columns", static_cast<unsigned long long>(
                poolList.size())},
            {"protected", static_cast<unsigned long long>(
                lastPruneProtectedCount)},
            {"rarity", static_cast<unsigned long long>(
                lastPruneRarityCount)},
            {"archive_discards", static_cast<unsigned long long>(
                searchArchiveDiscardCount)},
        });
}

void RoutePool::printStatus() const {
    std::cout << "--- Route Pool Status ---" << std::endl;
    std::cout << "Search archive:      " << searchArchive.size()
              << std::endl;
    std::cout << "SP master columns:   " << poolList.size()
              << std::endl;
    std::cout << "Dominance replaced:  "
              << dominanceReplacementCount << std::endl;
    std::cout << "Dominance discarded: "
              << dominanceDiscardCount << std::endl;
    std::cout << "Master version:      " << masterVersion
              << std::endl;
    std::cout << "-------------------------" << std::endl;
}
