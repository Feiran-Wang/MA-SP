/******************************************************************************
 * File: RoutePool.cpp
 * Author: Feiran Wang
 * Date: 2026-06-25
 * Description: 
 *   Central pool for deduplicating and retaining feasible routes.
 ******************************************************************************/

#include "RoutePool.h"
#include "ComponentProfiler.h"
#include "Individual.h"
#include "NormalizedSpUtility.h"
#include "RouteEvaluator.h"
#include "RunLogger.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace {

const char* routeOriginName(
        const hgasp::RouteOriginKind origin) noexcept {
    switch (origin) {
        case hgasp::RouteOriginKind::InitialConstruction:
            return "initial";
        case hgasp::RouteOriginKind::EaxOffspring:
            return "eax";
        case hgasp::RouteOriginKind::Mutation:
            return "mutation";
        case hgasp::RouteOriginKind::LocalSearch:
            return "local_search";
        case hgasp::RouteOriginKind::SpInjection:
            return "sp_injection";
    }
    return "unknown";
}

const char* routePopulationRoleName(
        const hgasp::RoutePopulationRole role) noexcept {
    switch (role) {
    case hgasp::RoutePopulationRole::Unspecified:
        return "unspecified";
    case hgasp::RoutePopulationRole::FixedResident:
        return "fixed_resident";
    case hgasp::RoutePopulationRole::UnrestrictedFree:
        return "unrestricted_free";
    case hgasp::RoutePopulationRole::DynamicChallenger:
        return "dynamic_challenger";
    case hgasp::RoutePopulationRole::InactiveOrSpare:
        return "inactive_or_spare";
    }
    return "unspecified";
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

RoutePool::RoutePool(
        SolverContext* data,
        const std::size_t capacityOverride,
        std::optional<hgasp::DepotConfigurationKey>
            fixedConfiguration)
    : fixedConfiguration(std::move(fixedConfiguration)) {
    if (data == nullptr) {
        throw std::invalid_argument(
            "route pool requires non-null instance data");
    }
    this->data = data;
    this->maxPoolSize = std::max<std::size_t>(
        1U,
        capacityOverride == 0U
            ? data->spMaxPoolSize
            : capacityOverride);
    depotMasterVersions.assign(
        static_cast<std::size_t>(data->numL), 0U);
    depotCustomerColumnCounts.assign(
        static_cast<std::size_t>(data->numL),
        std::vector<std::uint32_t>(
            static_cast<std::size_t>(data->numV), 0U));
    depotMasterKeySignatures.assign(
        static_cast<std::size_t>(data->numL), 0U);
    depotMasterAppendCounts.assign(
        static_cast<std::size_t>(data->numL), 0U);
    depotMasterRepresentativeReplacementCounts.assign(
        static_cast<std::size_t>(data->numL), 0U);
    depotMasterRepresentativeReplacementCostGains.assign(
        static_cast<std::size_t>(data->numL), 0.0);
}

RoutePool::~RoutePool() {
    uniqueSearchRouteKeys.clear();
    searchArchive.clear();
    overflowSearchRouteKeys.clear();
    overflowSearchRouteFifo.clear();
    bestArchivedColumns.clear();
    masterColumnIndex.clear();
    poolList.clear();
    poolMasterKeys.clear();
    backboneKeys.clear();
    globalIncumbentKeys.clear();
    lastSpIncumbentKeys.clear();
    populationEliteKeys.clear();
    latestCustomerDuals.clear();
    latestReducedCosts.clear();
    columnEvaluationStates.clear();
    columnProvenance.clear();
    archivedColumnProvenance.clear();
    depotMasterVersions.clear();
    depotCustomerColumnCounts.clear();
    depotMasterKeySignatures.clear();
    depotMasterAppendCounts.clear();
    depotMasterRepresentativeReplacementCounts.clear();
    depotMasterRepresentativeReplacementCostGains.clear();
}

MasterColumnKey RoutePool::masterKey(
        const StandaloneRoute& route) {
    hgasp::recordHotPathCounter(
        hgasp::HotPathCounter::RoutePoolMasterKeyBuilds);
    hgasp::recordHotPathCounter(
        hgasp::HotPathCounter::RoutePoolMasterKeyCustomerItems,
        static_cast<std::uint64_t>(route.sequence.size()));
    return hgasp::makeRouteKey(route);
}

std::uint64_t RoutePool::stableMasterKeyDigest(
        const MasterColumnKey& key) noexcept {
    std::uint64_t digest = 1469598103934665603ULL;
    const auto mix = [&digest](const std::uint64_t value) {
        for (unsigned int byte = 0U; byte < 8U; ++byte) {
            digest ^= (value >> (byte * 8U)) & 0xffU;
            digest *= 1099511628211ULL;
        }
    };
    mix(static_cast<std::uint64_t>(key.depot + 1));
    mix(static_cast<std::uint64_t>(key.customers.size()));
    for (const int customer : key.customers) {
        mix(static_cast<std::uint64_t>(customer + 1));
    }
    mix(0U);
    return digest;
}

void RoutePool::addActiveMasterKey(const MasterColumnKey& key) {
    if (key.depot < 0 || key.depot >= data->numL ||
        key.customers.empty() ||
        !std::is_sorted(key.customers.begin(), key.customers.end()) ||
        std::adjacent_find(
            key.customers.begin(), key.customers.end()) !=
            key.customers.end()) {
        throw std::logic_error(
            "active master append has an invalid canonical key");
    }
    const std::size_t depot =
        static_cast<std::size_t>(key.depot);
    if (depot >= depotCustomerColumnCounts.size() ||
        depot >= depotMasterKeySignatures.size()) {
        throw std::logic_error(
            "active master coverage sidecar has an invalid depot shape");
    }
    std::vector<std::uint32_t>& counts =
        depotCustomerColumnCounts[depot];
    for (const int customer : key.customers) {
        if (customer < 0 || customer >= data->numV ||
            static_cast<std::size_t>(customer) >= counts.size()) {
            throw std::logic_error(
                "active master append has an invalid customer");
        }
        if (counts[static_cast<std::size_t>(customer)] ==
            std::numeric_limits<std::uint32_t>::max()) {
            throw std::overflow_error(
                "active master coverage counter overflow");
        }
    }
    for (const int customer : key.customers) {
        ++counts[static_cast<std::size_t>(customer)];
    }
    depotMasterKeySignatures[depot] ^=
        stableMasterKeyDigest(key);
}

void RoutePool::removeActiveMasterKey(const MasterColumnKey& key) {
    if (key.depot < 0 || key.depot >= data->numL ||
        key.customers.empty() ||
        !std::is_sorted(key.customers.begin(), key.customers.end()) ||
        std::adjacent_find(
            key.customers.begin(), key.customers.end()) !=
            key.customers.end()) {
        throw std::logic_error(
            "active master removal has an invalid canonical key");
    }
    const std::size_t depot =
        static_cast<std::size_t>(key.depot);
    if (depot >= depotCustomerColumnCounts.size() ||
        depot >= depotMasterKeySignatures.size()) {
        throw std::logic_error(
            "active master coverage sidecar has an invalid depot shape");
    }
    std::vector<std::uint32_t>& counts =
        depotCustomerColumnCounts[depot];
    for (const int customer : key.customers) {
        if (customer < 0 || customer >= data->numV ||
            static_cast<std::size_t>(customer) >= counts.size() ||
            counts[static_cast<std::size_t>(customer)] == 0U) {
            throw std::logic_error(
                "active master coverage counter underflow");
        }
    }
    for (const int customer : key.customers) {
        --counts[static_cast<std::size_t>(customer)];
    }
    depotMasterKeySignatures[depot] ^=
        stableMasterKeyDigest(key);
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

RoutePool::MasterAdmission RoutePool::admitMasterColumn(
        const StandaloneRoute& route,
        const MasterColumnKey& originalKey,
        const hgasp::RouteSubmissionContext* const context) {
#ifndef NDEBUG
    if (masterKey(route) != originalKey) {
        throw std::logic_error(
            "master-column admission received a stale canonical key");
    }
#endif
    const auto existing = masterColumnIndex.find(originalKey);
    StandaloneRoute candidate = route;
    const MasterColumnKey& key = originalKey;
    const auto archived = bestArchivedColumns.find(key);
    if (archived != bestArchivedColumns.end() &&
        preferredMasterRoute(candidate, archived->second)) {
        archived->second = candidate;
    }
    if (existing == masterColumnIndex.end()) {
        const std::size_t index = poolList.size();
        poolList.push_back(candidate);
        try {
            poolMasterKeys.push_back(key);
            masterColumnIndex.emplace(key, index);
        } catch (...) {
            if (poolMasterKeys.size() > index) {
                poolMasterKeys.pop_back();
            }
            poolList.pop_back();
            throw;
        }
        addActiveMasterKey(key);
        restoreArchivedChallengerProvenance(key);
        if (context != nullptr) {
            recordContributor(key, *context, true);
        }
        ++masterVersion;
        columnEvaluationStates[key] =
            hgasp::MasterColumnEvaluationState{
                masterVersion, 0U, false};
        ++depotMasterVersions.at(
            static_cast<std::size_t>(candidate.locId));
        ++masterAppendCount;
        ++depotMasterAppendCounts.at(
            static_cast<std::size_t>(candidate.locId));
        return MasterAdmission::NewColumn;
    }

    StandaloneRoute& incumbent = poolList[existing->second];
    if (preferredMasterRoute(candidate, incumbent)) {
        const double replacementCostGain = std::max(
            0.0, incumbent.routeCost - candidate.routeCost);
        incumbent = candidate;
        if (context != nullptr) {
            recordContributor(key, *context, true);
        }
        ++dominanceReplacementCount;
        ++masterVersion;
        columnEvaluationStates[key] =
            hgasp::MasterColumnEvaluationState{
                masterVersion, 0U, false};
        latestReducedCosts.erase(key);
        ++depotMasterVersions.at(
            static_cast<std::size_t>(candidate.locId));
        ++masterRepresentativeReplacementCount;
        masterRepresentativeReplacementCostGain +=
            replacementCostGain;
        ++depotMasterRepresentativeReplacementCounts.at(
            static_cast<std::size_t>(candidate.locId));
        depotMasterRepresentativeReplacementCostGains.at(
            static_cast<std::size_t>(candidate.locId)) +=
                replacementCostGain;
        return MasterAdmission::DominanceReplacement;
    } else {
        if (context != nullptr) {
            recordContributor(key, *context, false);
        }
        ++dominanceDiscardCount;
        return MasterAdmission::Discarded;
    }
}

std::uint64_t RoutePool::getConfigurationMasterVersion(
        const hgasp::DepotConfigurationKey& configuration) const {
    std::uint64_t version = 0U;
    for (const int depot : configuration.openDepots) {
        if (depot < 0 ||
            static_cast<std::size_t>(depot) >=
                depotMasterVersions.size()) {
            throw std::out_of_range(
                "configuration master version references an invalid depot");
        }
        version += depotMasterVersions[
            static_cast<std::size_t>(depot)];
    }
    return version;
}

hgasp::ConfigurationCoverageSummary
RoutePool::getConfigurationCoverageSummary(
        const hgasp::DepotConfigurationKey& configuration) const {
    if (configuration.openDepots.empty() ||
        !std::is_sorted(
            configuration.openDepots.begin(),
            configuration.openDepots.end()) ||
        std::adjacent_find(
            configuration.openDepots.begin(),
            configuration.openDepots.end()) !=
            configuration.openDepots.end()) {
        throw std::invalid_argument(
            "coverage summary requires a non-empty canonical configuration");
    }

    for (const int depot : configuration.openDepots) {
        if (depot < 0 || depot >= data->numL) {
            throw std::out_of_range(
                "coverage summary references an invalid depot");
        }
    }
    if (depotCustomerColumnCounts.size() !=
            static_cast<std::size_t>(data->numL) ||
        depotMasterKeySignatures.size() !=
            static_cast<std::size_t>(data->numL)) {
        throw std::logic_error(
            "coverage summary sidecar has an invalid depot shape");
    }

    bool complete = data->numV > 0;
    for (int customer = 0; customer < data->numV; ++customer) {
        bool covered = false;
        for (const int depot : configuration.openDepots) {
            const std::vector<std::uint32_t>& counts =
                depotCustomerColumnCounts[
                    static_cast<std::size_t>(depot)];
            if (counts.size() !=
                static_cast<std::size_t>(data->numV)) {
                throw std::logic_error(
                    "coverage summary sidecar has an invalid customer shape");
            }
            if (counts[static_cast<std::size_t>(customer)] > 0U) {
                covered = true;
                break;
            }
        }
        if (!covered) {
            complete = false;
            break;
        }
    }

    std::uint64_t signature = 1469598103934665603ULL;
    const auto mix = [&signature](const std::uint64_t value) {
        for (unsigned int byte = 0U; byte < 8U; ++byte) {
            signature ^= (value >> (byte * 8U)) & 0xffU;
            signature *= 1099511628211ULL;
        }
    };
    mix(static_cast<std::uint64_t>(
        configuration.openDepots.size()));
    for (const int depot : configuration.openDepots) {
        mix(static_cast<std::uint64_t>(depot + 1));
        mix(depotMasterKeySignatures[
            static_cast<std::size_t>(depot)]);
    }

    hgasp::ConfigurationCoverageSummary result;
    result.completeCustomerCoverage = complete;
    result.signature = signature;
    result.version =
        getConfigurationMasterVersion(configuration);
    return result;
}

std::uint64_t RoutePool::getConfigurationMasterAppendCount(
        const hgasp::DepotConfigurationKey& configuration) const {
    std::uint64_t count = 0U;
    for (const int depot : configuration.openDepots) {
        if (depot < 0 ||
            static_cast<std::size_t>(depot) >=
                depotMasterAppendCounts.size()) {
            throw std::out_of_range(
                "configuration append count references an invalid depot");
        }
        count += depotMasterAppendCounts[
            static_cast<std::size_t>(depot)];
    }
    return count;
}

std::uint64_t
RoutePool::getConfigurationMasterRepresentativeReplacementCount(
        const hgasp::DepotConfigurationKey& configuration) const {
    std::uint64_t count = 0U;
    for (const int depot : configuration.openDepots) {
        if (depot < 0 ||
            static_cast<std::size_t>(depot) >=
                depotMasterRepresentativeReplacementCounts.size()) {
            throw std::out_of_range(
                "configuration replacement count references an invalid "
                "depot");
        }
        count += depotMasterRepresentativeReplacementCounts[
            static_cast<std::size_t>(depot)];
    }
    return count;
}

double RoutePool::getConfigurationMasterRepresentativeReplacementCostGain(
        const hgasp::DepotConfigurationKey& configuration) const {
    double gain = 0.0;
    for (const int depot : configuration.openDepots) {
        if (depot < 0 ||
            static_cast<std::size_t>(depot) >=
                depotMasterRepresentativeReplacementCostGains.size()) {
            throw std::out_of_range(
                "configuration replacement cost gain references an invalid "
                "depot");
        }
        gain += depotMasterRepresentativeReplacementCostGains[
            static_cast<std::size_t>(depot)];
    }
    return gain;
}

void RoutePool::recordContributor(
        const MasterColumnKey& key,
        const hgasp::RouteSubmissionContext& context,
        const bool bestOrigin) {
    const auto challengerOrigin = [](const int explicitSlot,
                                     const int population,
                                     const std::uint64_t generation,
                                     const hgasp::DepotConfigurationKey&
                                         configuration) {
        hgasp::ChallengerRouteOrigin origin;
        origin.slot = explicitSlot >= 0
            ? explicitSlot
            : population;
        origin.generation = generation;
        origin.configuration = configuration;
        return origin;
    };
    const hgasp::ChallengerRouteOrigin firstChallenger =
        challengerOrigin(
            context.firstChallengerSlot,
            context.firstPopulation,
            context.firstChallengerGeneration,
            context.firstOriginConfiguration);
    const hgasp::ChallengerRouteOrigin secondChallenger =
        challengerOrigin(
            context.secondChallengerSlot,
            context.secondPopulation,
            context.secondChallengerGeneration,
            context.secondOriginConfiguration);
	// Repeated ordinary resident/free submissions have no provenance consumed
	// by feedback. Avoid a map lookup and sidecar construction on that dominant
	// duplicate path; best-origin and challenger ancestry remain authoritative.
	if (!bestOrigin && !firstChallenger.valid() &&
		!secondChallenger.valid()) {
		hgasp::recordHotPathCounter(
			hgasp::HotPathCounter::RoutePoolProvenanceNoopSkips);
		return;
	}
    hgasp::ColumnProvenanceStats& stats =
        columnProvenance[key];
    const auto rememberChallenger = [&stats](
            const hgasp::ChallengerRouteOrigin& origin) {
        if (!origin.valid()) {
            return;
        }
        if (!stats.firstChallengerOrigin.valid()) {
            stats.firstChallengerOrigin = origin;
        }
        const auto existing = std::find(
            stats.historicalChallengerOrigins.begin(),
            stats.historicalChallengerOrigins.end(),
            origin);
        constexpr std::size_t maximumChallengerContributors = 8U;
        if (existing == stats.historicalChallengerOrigins.end() &&
            stats.historicalChallengerOrigins.size() <
                maximumChallengerContributors) {
            stats.historicalChallengerOrigins.push_back(origin);
        }
    };
    rememberChallenger(firstChallenger);
    rememberChallenger(secondChallenger);
    if (bestOrigin) {
        stats.bestRouteOrigin = context;
        stats.bestRouteOriginChallenger =
            firstChallenger.valid()
                ? firstChallenger
                : secondChallenger;
    }
}

void RoutePool::pruneColumnSidecars() {
    for (auto iterator = columnProvenance.begin();
         iterator != columnProvenance.end();) {
        if (masterColumnIndex.count(iterator->first) == 0U) {
            if (bestArchivedColumns.count(iterator->first) != 0U) {
                hgasp::ArchivedChallengerColumnProvenance archived;
                archived.firstChallengerOrigin =
                    iterator->second.firstChallengerOrigin;
                archived.bestRouteOriginChallenger =
                    iterator->second.bestRouteOriginChallenger;
                archived.historicalChallengerOrigins =
                    iterator->second.historicalChallengerOrigins;
                if (archived.firstChallengerOrigin.valid() ||
                    archived.bestRouteOriginChallenger.valid() ||
                    !archived.historicalChallengerOrigins.empty()) {
                    archivedColumnProvenance[iterator->first] =
                        std::move(archived);
                }
            }
            iterator = columnProvenance.erase(iterator);
        } else {
            ++iterator;
        }
    }
    for (auto iterator = latestReducedCosts.begin();
         iterator != latestReducedCosts.end();) {
        if (masterColumnIndex.count(iterator->first) == 0U) {
            iterator = latestReducedCosts.erase(iterator);
        } else {
            ++iterator;
        }
    }
    for (auto iterator = columnEvaluationStates.begin();
         iterator != columnEvaluationStates.end();) {
        if (masterColumnIndex.count(iterator->first) == 0U) {
            iterator = columnEvaluationStates.erase(iterator);
        } else {
            ++iterator;
        }
    }
}

void RoutePool::restoreArchivedChallengerProvenance(
        const MasterColumnKey& key) {
    const auto archived = archivedColumnProvenance.find(key);
    if (archived == archivedColumnProvenance.end()) {
        return;
    }
    hgasp::ColumnProvenanceStats& active = columnProvenance[key];
    active.firstChallengerOrigin =
        archived->second.firstChallengerOrigin;
    active.bestRouteOriginChallenger =
        archived->second.bestRouteOriginChallenger;
    active.historicalChallengerOrigins =
        archived->second.historicalChallengerOrigins;
}

const MasterColumnKey* RoutePool::archivedOrderedMasterKey(
        const StandaloneRoute& route) const noexcept {
    const auto retained = uniqueSearchRouteKeys.find(route);
    if (retained != uniqueSearchRouteKeys.end()) {
        hgasp::recordHotPathCounter(
            hgasp::HotPathCounter::RoutePoolMasterKeyCacheHits);
        return &retained->second;
    }
    const auto overflow = overflowSearchRouteKeys.find(route);
    if (overflow != overflowSearchRouteKeys.end()) {
        hgasp::recordHotPathCounter(
            hgasp::HotPathCounter::RoutePoolMasterKeyCacheHits);
        return &overflow->second;
    }
    return nullptr;
}

bool RoutePool::archiveOrderedRoute(
        const StandaloneRoute& route,
        const MasterColumnKey& key) {
    if (archivedOrderedMasterKey(route) != nullptr) {
        return false;
    }
#ifndef NDEBUG
    if (masterKey(route) != key) {
        throw std::logic_error(
            "ordered-route archive received a stale canonical key");
    }
#endif
    if (searchArchive.size() >= maxPoolSize) {
        // SearchRouteArchive is diagnostic/search provenance, not the SP
        // master. Refuse excess history while still allowing a first-seen
        // route to compete for the bounded master below.  Remember the exact
        // ordered identity in a bounded FIFO so a recurrence does not repeat
        // master dominance work.  Master-key recovery remains independent:
        // every caller also admits when the canonical key is no longer
        // active, even if this method reports an already-seen route.
        if (overflowSearchRouteFifo.size() >= maxPoolSize) {
            const StandaloneRoute& oldest =
                overflowSearchRouteFifo.front();
            const std::size_t erased =
                overflowSearchRouteKeys.erase(oldest);
            if (erased != 1U) {
                throw std::logic_error(
                    "ordered-route overflow cache is inconsistent");
            }
            overflowSearchRouteFifo.pop_front();
        }
        const auto insertion = overflowSearchRouteKeys.emplace(route, key);
        if (!insertion.second) {
            throw std::logic_error(
                "ordered-route overflow cache insertion disagreed with "
                "its exact membership check");
        }
        overflowSearchRouteFifo.push_back(route);
        ++searchArchiveDiscardCount;
        return true;
    }
    const auto insertion = uniqueSearchRouteKeys.emplace(route, key);
    if (!insertion.second) {
        throw std::logic_error(
            "ordered-route archive insertion disagreed with its exact "
            "membership check");
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
        const MasterColumnKey* cachedKey =
            archivedOrderedMasterKey(route);
        MasterColumnKey constructedKey;
        if (cachedKey == nullptr) {
            constructedKey = masterKey(route);
        }
        const MasterColumnKey& key = cachedKey == nullptr
            ? constructedKey
            : *cachedKey;
        replacement.insert(key);

        if (cachedKey != nullptr &&
            masterColumnIndex.find(key) != masterColumnIndex.end()) {
            continue;
        }
        const bool newOrderedRoute = archiveOrderedRoute(route, key);
        const StandaloneRoute archivedBest =
            archivedBestOrCandidate(key, route);
        if (masterColumnIndex.find(key) ==
            masterColumnIndex.end()) {
            admitMasterColumn(archivedBest, key);
        } else if (newOrderedRoute) {
            admitMasterColumn(route, key);
        }
    }
    protectionSet(protection) = std::move(replacement);
}

std::vector<StandaloneRoute> RoutePool::protectedRoutes(
        const RouteProtection protection) const {
    const ProtectedKeySet* keys = nullptr;
    switch (protection) {
        case RouteProtection::Backbone:
            keys = &backboneKeys;
            break;
        case RouteProtection::GlobalIncumbent:
            keys = &globalIncumbentKeys;
            break;
        case RouteProtection::LastSpIncumbent:
            keys = &lastSpIncumbentKeys;
            break;
        case RouteProtection::PopulationElite:
            keys = &populationEliteKeys;
            break;
    }
    std::vector<std::size_t> matchingIndices;
    matchingIndices.reserve(keys->size());
    if (poolMasterKeys.size() != poolList.size()) {
        throw std::logic_error(
            "master-key sidecar is out of sync with the route pool");
    }
    for (std::size_t index = 0U; index < poolList.size(); ++index) {
        if (keys->count(poolMasterKeys[index]) != 0U) {
            hgasp::recordHotPathCounter(
                hgasp::HotPathCounter::RoutePoolMasterKeyCacheHits);
            matchingIndices.push_back(index);
        }
    }
    std::sort(
        matchingIndices.begin(),
        matchingIndices.end(),
        [this](const std::size_t left, const std::size_t right) {
            const MasterColumnKey& leftKey = poolMasterKeys[left];
            const MasterColumnKey& rightKey = poolMasterKeys[right];
            if (leftKey != rightKey) {
                return leftKey < rightKey;
            }
            return poolList[left].sequence < poolList[right].sequence;
        });
    std::vector<StandaloneRoute> routes;
    routes.reserve(matchingIndices.size());
    for (const std::size_t index : matchingIndices) {
        hgasp::recordHotPathCounter(
            hgasp::HotPathCounter::RoutePoolMasterKeyCacheHits);
        routes.push_back(poolList[index]);
    }
    return routes;
}

void RoutePool::pruneIfNeeded() {
    if (poolList.size() > maxPoolSize) {
        prunePool();
    }
}

std::size_t RoutePool::updateDualFeedback(
        const std::vector<StandaloneRoute>& snapshot,
        const std::vector<double>& customerDuals,
        const std::vector<double>& routeReducedCosts,
        const bool allowEphemeralColumns) {
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
    std::size_t ephemeralColumns = 0U;
    for (std::size_t index = 0; index < snapshot.size(); ++index) {
        const MasterColumnKey* cachedKey =
            archivedOrderedMasterKey(snapshot[index]);
        MasterColumnKey constructedKey;
        if (cachedKey == nullptr) {
            constructedKey = masterKey(snapshot[index]);
        }
        const MasterColumnKey& key = cachedKey == nullptr
            ? constructedKey
            : *cachedKey;
        const auto state = columnEvaluationStates.find(key);
        if (state == columnEvaluationStates.end()) {
            if (allowEphemeralColumns) {
                ++ephemeralColumns;
                continue;
            }
            throw std::logic_error(
                "LP feedback references a route outside the active master");
        }
        latestReducedCosts[key] = routeReducedCosts[index];
        state->second.evaluatedByLp = true;
        state->second.lastLpVersion = masterVersion;
    }
    return ephemeralColumns;
}


double RoutePool::reducedCostFor(
        const StandaloneRoute& route) const noexcept {
    const MasterColumnKey* cachedKey = archivedOrderedMasterKey(route);
    MasterColumnKey constructedKey;
    if (cachedKey == nullptr) {
        constructedKey = masterKey(route);
    }
    const MasterColumnKey& key = cachedKey == nullptr
        ? constructedKey
        : *cachedKey;
    const auto found = latestReducedCosts.find(key);
    const auto state = columnEvaluationStates.find(key);
    return found == latestReducedCosts.end() ||
            state == columnEvaluationStates.end() ||
            !hgasp::hasCurrentReducedCost(
                state->second, masterVersion)
        ? std::numeric_limits<double>::infinity()
        : found->second;
}

std::optional<hgasp::MasterColumnEvaluationState>
RoutePool::columnEvaluationStateFor(
        const StandaloneRoute& route) const {
    const MasterColumnKey* cachedKey = archivedOrderedMasterKey(route);
    MasterColumnKey constructedKey;
    if (cachedKey == nullptr) {
        constructedKey = masterKey(route);
    }
    const MasterColumnKey& key = cachedKey == nullptr
        ? constructedKey
        : *cachedKey;
    const auto found = columnEvaluationStates.find(key);
    return found == columnEvaluationStates.end()
        ? std::nullopt
        : std::optional<hgasp::MasterColumnEvaluationState>(
              found->second);
}

void RoutePool::rebuildMasterIndex() {
    if (poolMasterKeys.size() != poolList.size()) {
        throw std::logic_error(
            "master-key sidecar is out of sync with the route pool");
    }
    masterColumnIndex.clear();
    masterColumnIndex.reserve(poolList.size());
    for (std::size_t index = 0; index < poolList.size(); ++index) {
        hgasp::recordHotPathCounter(
            hgasp::HotPathCounter::RoutePoolMasterKeyCacheHits);
        masterColumnIndex.emplace(poolMasterKeys[index], index);
    }
}

void RoutePool::submitFeasibleIndividual(
        const Individual* indiv,
        const hgasp::RouteSubmissionContext& context) {
    if (indiv == nullptr) {
        throw std::invalid_argument(
            "route-pool submission requires a non-null individual");
    }
    const std::vector<StandaloneRoute> submittedRoutes =
        exportRoutes(indiv);
    submitPreparedFeasibleIndividual(indiv, submittedRoutes, context);
    if (submissionObserver) {
        submissionObserver(indiv, submittedRoutes, context);
    }
}

void RoutePool::submitPreparedFeasibleIndividual(
        const Individual* indiv,
        const std::vector<StandaloneRoute>& submittedRoutes,
        const hgasp::RouteSubmissionContext& context) {
    hgasp::ScopedComponentTimer timer(
        hgasp::TimedComponent::RoutePoolAdmission);
    if (indiv == nullptr) {
        throw std::invalid_argument(
            "route-pool submission requires a non-null individual");
    }
    if (!indiv->isFeasible) {
        throw std::invalid_argument(
            "route-pool submission requires a feasible individual");
    }
    if (fixedConfiguration.has_value()) {
        std::vector<int> usedDepots;
        for (int depot = 0; depot < data->numL; ++depot) {
            if (indiv->location[depot].used) {
                usedDepots.push_back(depot);
            }
        }
        if (hgasp::makeDepotConfigurationKey(std::move(usedDepots)) !=
            *fixedConfiguration) {
            throw std::invalid_argument(
                "population-local route-pool submission changed its fixed "
                "depot configuration");
        }
        for (const StandaloneRoute& route : submittedRoutes) {
            if (!std::binary_search(
                    fixedConfiguration->openDepots.begin(),
                    fixedConfiguration->openDepots.end(),
                    route.locId)) {
                throw std::invalid_argument(
                    "population-local route-pool route uses a foreign depot");
            }
        }
    }
    hgasp::recordHotPathCounter(
        hgasp::HotPathCounter::RoutePoolSubmissions);
    std::size_t newColumns = 0U;
    std::size_t dominanceReplacements = 0U;
    std::size_t discardedColumns = 0U;
    const bool v3SetS = hgasp::isSetSProductionSearchProfile(
        data->parameters.profile);
    const bool privilegedHarvest =
        context.origin == hgasp::RouteOriginKind::InitialConstruction ||
        context.origin == hgasp::RouteOriginKind::SpInjection ||
        context.firstPopulationRole ==
            hgasp::RoutePopulationRole::DynamicChallenger ||
        context.secondPopulationRole ==
            hgasp::RoutePopulationRole::DynamicChallenger;
    const bool wholeSolutionHarvest = !v3SetS || privilegedHarvest ||
        feasibleSubmissionCount == 0U || indiv->fit <= getAverageFitness();
    for (const StandaloneRoute& route : submittedRoutes) {
        const MasterColumnKey* cachedKey =
            archivedOrderedMasterKey(route);
        MasterColumnKey constructedKey;
        if (cachedKey == nullptr) {
            constructedKey = masterKey(route);
        }
        const MasterColumnKey& key = cachedKey == nullptr
            ? constructedKey
            : *cachedKey;
        const auto active = masterColumnIndex.find(key);
        // An exact ordered route already evaluated against the still-active
        // key cannot alter its representative on recurrence: route cost is a
        // deterministic function of depot plus directed sequence. Preserve
        // contributor provenance, but bypass normalization, archive lookup,
        // and the identical dominance comparison.
        if (cachedKey != nullptr && active != masterColumnIndex.end()) {
            recordContributor(key, context, false);
            ++discardedColumns;
            continue;
        }
        // Role-aware Set S avoids retaining an ordinary, above-mean whole-solution
        // harvest when it cannot change the exact master.  New coefficient
        // columns and cheaper representatives still pass, so the SP column
        // set and dominance result are unchanged.  Set F and privileged
        // incumbent/challenger/SP sources keep the exhaustive boundary.
        if (!wholeSolutionHarvest && active != masterColumnIndex.end() &&
            !preferredMasterRoute(route, poolList[active->second])) {
            recordContributor(key, context, false);
            ++discardedColumns;
            continue;
        }
        const bool newOrderedRoute = archiveOrderedRoute(route, key);
        const StandaloneRoute archivedBest =
            archivedBestOrCandidate(key, route);
        const auto current = masterColumnIndex.find(key);
        if (newOrderedRoute || current == masterColumnIndex.end()) {
            const MasterAdmission admission = admitMasterColumn(
                current == masterColumnIndex.end()
                    ? archivedBest
                    : route,
                key,
                &context);
            if (admission == MasterAdmission::NewColumn) {
                ++newColumns;
            } else if (
                admission ==
                    MasterAdmission::DominanceReplacement) {
                ++dominanceReplacements;
            } else {
                ++discardedColumns;
            }
        } else {
            // Even a route whose ordered representation was seen before is a
            // contributor from this pipeline trace.
            recordContributor(key, context, false);
            ++discardedColumns;
        }
    }
    pruneIfNeeded();
    totalFitnessSum += indiv->fit;
    ++feasibleSubmissionCount;
    if (context.traceId != 0U ||
        context.origin ==
            hgasp::RouteOriginKind::SpInjection) {
        hgasp::RunLogger::event(
            context.origin ==
                        hgasp::RouteOriginKind::SpInjection
                ? hgasp::LogLevel::info
                : hgasp::LogLevel::debug,
            "route_pool",
            "route_submitted",
            {},
            {
                {"iteration", context.iteration},
                {"trace_id",
                    static_cast<unsigned long long>(
                        context.traceId)},
                {"origin", routeOriginName(context.origin)},
                {"first_population",
                    context.firstPopulation},
                {"second_population",
                    context.secondPopulation},
                {"first_population_role",
                    routePopulationRoleName(
                        context.firstPopulationRole)},
                {"second_population_role",
                    routePopulationRoleName(
                        context.secondPopulationRole)},
                {"first_origin_configuration",
                    configurationLabel(
                        context.firstOriginConfiguration)},
                {"second_origin_configuration",
                    configurationLabel(
                        context.secondOriginConfiguration)},
                {"first_challenger_slot",
                    context.firstChallengerSlot},
                {"first_challenger_generation",
                    static_cast<unsigned long long>(
                        context.firstChallengerGeneration)},
                {"second_challenger_slot",
                    context.secondChallengerSlot},
                {"second_challenger_generation",
                    static_cast<unsigned long long>(
                        context.secondChallengerGeneration)},
                {"first_parent_individual_id",
                    static_cast<unsigned long long>(
                        context.firstParentIndividualId)},
                {"second_parent_individual_id",
                    static_cast<unsigned long long>(
                        context.secondParentIndividualId)},
                {"mutation_committed",
                    context.mutationCommitted},
                {"sp_ancestry_call_id",
                    context.spAncestryCallId},
                {"routes",
                    static_cast<unsigned long long>(
                        submittedRoutes.size())},
                {"new_master_columns",
                    static_cast<unsigned long long>(
                        newColumns)},
                {"dominance_replacements",
                    static_cast<unsigned long long>(
                        dominanceReplacements)},
                {"discarded_columns",
                    static_cast<unsigned long long>(
                        discardedColumns)},
            });
    }
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

    const bool reuseCertifiedCapacityEvaluation =
        !data->hasHardTimeWindows() &&
        indiv->hasCurrentFeasibilityCertificate();
    std::vector<StandaloneRoute> exported;
    const std::vector<int>& activeRoutes = indiv->activeRouteIndices();
    exported.reserve(activeRoutes.size());
    for (const int routeIndex : activeRoutes) {
        const Route& sourceRoute = indiv->route[routeIndex];
        if (!sourceRoute.used ||
            !routeIsOccupied(sourceRoute) ||
            sourceRoute.whichLocation == nullptr ||
            !sourceRoute.whichLocation->used) {
            throw std::logic_error(
                "route-pool admission found inconsistent active route state");
        }
        const int depot =
            sourceRoute.whichLocation->cour - data->numV;
        if (depot < 0 || depot >= data->numL) {
            throw std::logic_error(
                "route-pool admission found invalid route depot");
        }
        StandaloneRoute sr;
        sr.locId = depot;
        sr.totalLoad = 0;
        sr.sequence.reserve(static_cast<std::size_t>(
            sourceRoute.nbClients));

        // routeCost contains travel cost only. Vehicle and depot fixed costs
        // are added by the SP objective.
        sr.routeCost = 0.0;
        const Node* current = sourceRoute.depot->next;
        const Node* previous = sourceRoute.depot;
        while (!current->isDepot) {
            sr.sequence.push_back(current->cour);
            sr.totalLoad += current->dem;
            // Preserve route direction for asymmetric instances.
            sr.routeCost += data->D[previous->cour][current->cour];
            previous = current;
            current = current->next;
        }
        sr.routeCost += data->D[previous->cour][current->cour];

        if (sr.totalLoad != sourceRoute.load) {
            throw std::logic_error(
                "route-pool admission found a stale route load aggregate");
        }
        if (!std::isfinite(sourceRoute.dis) ||
            std::fabs(sr.routeCost - sourceRoute.dis) > 1.0e-9) {
            throw std::logic_error(
                "route-pool admission found a stale route cost aggregate");
        }
        if (reuseCertifiedCapacityEvaluation) {
            // The current topology/problem certificate was issued by the full
            // route evaluator. The sequence and aggregates were rebuilt above.
            hgasp::recordHotPathCounter(
                hgasp::HotPathCounter::
                    RoutePoolEvaluationCertificateHits);
        } else {
            hgasp::recordHotPathCounter(
                hgasp::HotPathCounter::RoutePoolEvaluationFallbacks);
            const RouteEvaluation evaluation =
                data->problemContext().evaluateRoute(
                    *data, depot, sr.sequence);
            hgasp::recordHotPathCounter(
                hgasp::HotPathCounter::RoutePoolRoutesEvaluated);
            if (!evaluation.feasible()) {
                throw std::runtime_error(
                    "route-pool admission rejected an infeasible route");
            }
            if (evaluation.load != sourceRoute.load ||
                std::fabs(evaluation.travelCost - sourceRoute.dis) >
                    1.0e-9) {
                throw std::logic_error(
                    "route-pool admission evaluator disagreed with route "
                    "aggregates");
            }
            sr.totalLoad = evaluation.load;
            sr.routeCost = evaluation.travelCost;
            sr.returnTime = evaluation.returnTime;
            sr.totalWaitingTime = evaluation.totalWaitingTime;
            sr.temporalFeasible = evaluation.temporalFeasible;
            sr.temporalViolationCustomer = evaluation.violatingCustomer;
        }
        exported.push_back(std::move(sr));
    }
    return exported;
}

RoutePool::DepotReassignmentClosure
RoutePool::buildDepotReassignmentClosure(
        std::vector<StandaloneRoute> snapshot,
        const std::vector<StandaloneRoute>& seedRoutes) const {
    if (data->hasHardTimeWindows()) {
        throw std::invalid_argument(
            "depot-reassignment closure is capacity-profile only");
    }

    DepotReassignmentClosure closure;
    closure.snapshot = std::move(snapshot);
    std::unordered_map<
        MasterColumnKey,
        std::size_t,
        MasterColumnKeyHash> index;
    index.reserve(closure.snapshot.size() +
        seedRoutes.size() * static_cast<std::size_t>(data->numL));
    for (std::size_t position = 0U;
         position < closure.snapshot.size();
         ++position) {
        const MasterColumnKey key = masterKey(closure.snapshot[position]);
        const auto [entry, inserted] = index.emplace(key, position);
        if (!inserted && preferredMasterRoute(
                closure.snapshot[position],
                closure.snapshot[entry->second])) {
            closure.snapshot[entry->second] = closure.snapshot[position];
        }
    }

    for (const StandaloneRoute& seed : seedRoutes) {
        if (seed.sequence.empty() || seed.totalLoad < 0 ||
            seed.totalLoad > data->vehCap) {
            throw std::invalid_argument(
                "depot-reassignment closure received an invalid seed route");
        }
        for (int depot = 0; depot < data->numL; ++depot) {
            if (seed.totalLoad > data->invLoc[data->numV + depot]) {
                continue;
            }
            ++closure.candidatesEvaluated;
            const RouteEvaluation evaluation =
                data->problemContext().evaluateRoute(
                    *data, depot, seed.sequence);
            if (!evaluation.feasible()) {
                continue;
            }
            StandaloneRoute candidate = seed;
            candidate.locId = depot;
            candidate.routeCost = evaluation.travelCost;
            candidate.totalLoad = evaluation.load;
            candidate.returnTime = evaluation.returnTime;
            candidate.totalWaitingTime = evaluation.totalWaitingTime;
            candidate.temporalFeasible = evaluation.temporalFeasible;
            candidate.temporalViolationCustomer =
                evaluation.violatingCustomer;

            const MasterColumnKey key = masterKey(candidate);
            const auto found = index.find(key);
            if (found == index.end()) {
                index.emplace(key, closure.snapshot.size());
                closure.snapshot.push_back(std::move(candidate));
                ++closure.columnsAppended;
            } else if (preferredMasterRoute(
                    candidate, closure.snapshot[found->second])) {
                closure.snapshot[found->second] = std::move(candidate);
                ++closure.representativesReplaced;
            }
        }
    }
    return closure;
}

RoutePool::ExploratoryActiveView
RoutePool::buildExploratoryActiveView(
        const std::vector<StandaloneRoute>& snapshot,
        const SpResult& lpResult,
        const std::vector<StandaloneRoute>& mandatoryRoutes,
        const std::size_t targetSize) const {
    ExploratoryActiveView view;
    view.active = snapshot;
    const bool alignedFeedback = lpResult.lpFeedbackAvailable &&
        lpResult.routeReducedCosts.size() == snapshot.size() &&
        lpResult.routeLpValues.size() == snapshot.size();
    view.dualFeedbackUsable = alignedFeedback;
    if (snapshot.empty() || targetSize == 0U ||
        snapshot.size() <= targetSize) {
        return view;
    }

    const std::size_t customerCount =
        static_cast<std::size_t>(data->numV);
    const std::size_t depotCount =
        static_cast<std::size_t>(data->numL);
    std::unordered_set<MasterColumnKey, MasterColumnKeyHash> mandatory;
    mandatory.reserve(mandatoryRoutes.size());
    for (const StandaloneRoute& route : mandatoryRoutes) {
        mandatory.insert(masterKey(route));
    }

    struct Metric {
        std::size_t index = 0U;
        double reducedCost = 0.0;
        double efficiency = 0.0;
        double rarity = 0.0;
        bool mandatory = false;
        bool protectedColumn = false;
        bool lpSupport = false;
    };
    std::vector<int> availability(customerCount, 0);
    for (const StandaloneRoute& route : snapshot) {
        for (const int customer : route.sequence) {
            ++availability.at(static_cast<std::size_t>(customer));
        }
    }
    std::vector<Metric> metrics(snapshot.size());
    for (std::size_t index = 0U; index < snapshot.size(); ++index) {
        const StandaloneRoute& route = snapshot[index];
        Metric& metric = metrics[index];
        metric.index = index;
        const MasterColumnKey key = masterKey(route);
        metric.mandatory = mandatory.count(key) != 0U;
        metric.protectedColumn = isProtected(key);
        metric.lpSupport = alignedFeedback &&
            lpResult.routeLpValues[index] > 1.0e-8;
        metric.reducedCost = alignedFeedback && std::isfinite(
                lpResult.routeReducedCosts[index])
            ? lpResult.routeReducedCosts[index]
            : std::numeric_limits<double>::infinity();
        metric.efficiency = route.routeCost > 1.0e-9
            ? static_cast<double>(route.totalLoad) / route.routeCost
            : std::numeric_limits<double>::infinity();
        for (const int customer : route.sequence) {
            metric.rarity += 1.0 / static_cast<double>(
                std::max(
                    1,
                    availability.at(
                        static_cast<std::size_t>(customer))));
        }
    }

    std::vector<std::uint8_t> retained(snapshot.size(), 0U);
    std::vector<std::size_t> retainedIndices;
    retainedIndices.reserve(targetSize);
    auto retain = [&](const std::size_t index) {
        if (retained[index]) {
            return false;
        }
        retained[index] = 1U;
        retainedIndices.push_back(index);
        return true;
    };
    for (const Metric& metric : metrics) {
        if (metric.mandatory && retain(metric.index)) {
            ++view.mandatoryCount;
        }
    }
    for (const Metric& metric : metrics) {
        if (metric.protectedColumn && retain(metric.index)) {
            ++view.protectedCount;
        }
    }
    for (const Metric& metric : metrics) {
        if (metric.lpSupport && retain(metric.index)) {
            ++view.lpSupportCount;
        }
    }

    std::vector<std::size_t> candidates(snapshot.size());
    std::iota(candidates.begin(), candidates.end(), 0U);
    const auto dualOrder = [&](const std::size_t left,
                               const std::size_t right) {
        const Metric& lhs = metrics[left];
        const Metric& rhs = metrics[right];
        const bool lhsFinite = std::isfinite(lhs.reducedCost);
        const bool rhsFinite = std::isfinite(rhs.reducedCost);
        if (lhsFinite != rhsFinite) {
            return lhsFinite;
        }
        if (lhsFinite &&
            std::fabs(lhs.reducedCost - rhs.reducedCost) > 1.0e-12) {
            return lhs.reducedCost < rhs.reducedCost;
        }
        if (std::fabs(lhs.rarity - rhs.rarity) > 1.0e-12) {
            return lhs.rarity > rhs.rarity;
        }
        if (std::fabs(lhs.efficiency - rhs.efficiency) > 1.0e-12) {
            return lhs.efficiency > rhs.efficiency;
        }
        const MasterColumnKey leftKey = masterKey(snapshot[left]);
        const MasterColumnKey rightKey = masterKey(snapshot[right]);
        if (leftKey.depot != rightKey.depot) {
            return leftKey.depot < rightKey.depot;
        }
        return leftKey.customers < rightKey.customers;
    };
    std::stable_sort(candidates.begin(), candidates.end(), dualOrder);

    std::vector<int> covered(customerCount, 0);
    std::vector<bool> representedDepot(depotCount, false);
    for (const std::size_t index : retainedIndices) {
        representedDepot.at(
            static_cast<std::size_t>(snapshot[index].locId)) = true;
        for (const int customer : snapshot[index].sequence) {
            ++covered.at(static_cast<std::size_t>(customer));
        }
    }
    for (const std::size_t index : candidates) {
        if (retainedIndices.size() >= targetSize) {
            break;
        }
        bool needed = false;
        for (const int customer : snapshot[index].sequence) {
            const std::size_t customerIndex =
                static_cast<std::size_t>(customer);
            if (covered[customerIndex] <
                std::min(2, availability[customerIndex])) {
                needed = true;
                break;
            }
        }
        if (!needed || !retain(index)) {
            continue;
        }
        for (const int customer : snapshot[index].sequence) {
            ++covered[static_cast<std::size_t>(customer)];
        }
        representedDepot[static_cast<std::size_t>(
            snapshot[index].locId)] = true;
    }
    for (const std::size_t index : candidates) {
        if (retainedIndices.size() >= targetSize) {
            break;
        }
        const std::size_t depot =
            static_cast<std::size_t>(snapshot[index].locId);
        if (!representedDepot[depot] && retain(index)) {
            representedDepot[depot] = true;
        }
    }
    for (const std::size_t index : candidates) {
        if (retainedIndices.size() >= targetSize) {
            break;
        }
        retain(index);
    }

    std::sort(retainedIndices.begin(), retainedIndices.end());
    view.active.clear();
    view.active.reserve(retainedIndices.size());
    for (const std::size_t index : retainedIndices) {
        view.active.push_back(snapshot[index]);
    }
    view.reserveCount = snapshot.size() - view.active.size();
    view.reduced = view.reserveCount > 0U;
    return view;
}

void RoutePool::extractRoutes(const Individual* indiv) {
    hgasp::ScopedComponentTimer timer(
        hgasp::TimedComponent::RoutePoolAdmission);
    // Callers admit only feasible individuals; each used non-empty route is
    // converted to the value representation shared with the SP solver.
    for (const StandaloneRoute& sr : exportRoutes(indiv)) {
        const MasterColumnKey* cachedKey =
            archivedOrderedMasterKey(sr);
        MasterColumnKey constructedKey;
        if (cachedKey == nullptr) {
            constructedKey = masterKey(sr);
        }
        const MasterColumnKey& key = cachedKey == nullptr
            ? constructedKey
            : *cachedKey;
        if (cachedKey != nullptr &&
            masterColumnIndex.find(key) != masterColumnIndex.end()) {
            continue;
        }
        const bool newOrderedRoute = archiveOrderedRoute(sr, key);
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
                    : sr,
                key);
        }
    }
    pruneIfNeeded();
}

void RoutePool::prunePool() {
    hgasp::ScopedComponentTimer timer(
        hgasp::TimedComponent::RoutePoolPruning);
    hgasp::recordHotPathCounter(
        hgasp::HotPathCounter::RoutePoolPruneCalls);
    hgasp::recordHotPathCounter(
        hgasp::HotPathCounter::RoutePoolPruneColumnsScanned,
        static_cast<std::uint64_t>(poolList.size()));
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
    if (poolMasterKeys.size() != poolList.size()) {
        throw std::logic_error(
            "master-key sidecar is out of sync with the route pool");
    }

    const std::size_t targetSize =
        std::max<std::size_t>(1, maxPoolSize * 4 / 5);
    std::vector<int>& availability = pruneAvailabilityScratch;
    availability.assign(static_cast<std::size_t>(data->numV), 0);
    if (depotCustomerColumnCounts.size() !=
            static_cast<std::size_t>(data->numL)) {
        throw std::logic_error(
            "route-pool coverage sidecar is out of sync before pruning");
    }
    for (const auto& depotCounts : depotCustomerColumnCounts) {
        if (depotCounts.size() != availability.size()) {
            throw std::logic_error(
                "route-pool customer coverage has an invalid shape");
        }
        for (std::size_t customer = 0U;
             customer < availability.size(); ++customer) {
            availability[customer] += static_cast<int>(
                depotCounts[customer]);
        }
    }
    std::vector<PruneMetric>& pruneMetrics = pruneMetricScratch;
    pruneMetrics.clear();
    pruneMetrics.reserve(poolList.size());
    for (std::size_t index = 0U; index < poolList.size(); ++index) {
        const StandaloneRoute& route = poolList[index];
        const MasterColumnKey& key = poolMasterKeys[index];
        PruneMetric metric;
        metric.index = index;
        metric.reducedCost =
            std::numeric_limits<double>::infinity();
        hgasp::recordHotPathCounter(
            hgasp::HotPathCounter::RoutePoolMasterKeyCacheHits);
        metric.routeCost = route.routeCost;
        metric.efficiency = route.routeCost > 1.0e-9
            ? static_cast<double>(route.totalLoad) /
                  route.routeCost
            : 1.0e30;
        metric.depot = route.locId;
        metric.protectedColumn = isProtected(key);

        const auto state = columnEvaluationStates.find(key);
        metric.hasEvaluationState =
            state != columnEvaluationStates.end();
        if (metric.hasEvaluationState) {
            metric.lpEvaluated = state->second.evaluatedByLp;
            metric.insertedMasterVersion =
                state->second.insertedMasterVersion;
        }
        const auto reducedCost = latestReducedCosts.find(key);
        if (reducedCost != latestReducedCosts.end() &&
            metric.hasEvaluationState &&
            hgasp::hasCurrentReducedCost(
                state->second, masterVersion)) {
            metric.reducedCost = reducedCost->second;
        }
        pruneMetrics.push_back(std::move(metric));
    }
    for (PruneMetric& metric : pruneMetrics) {
        // Preserve V1.14.1 floating-point accumulation order: rarity follows
        // the retained route sequence, not the sorted master-column key.
        for (const int customer :
             poolList[metric.index].sequence) {
            metric.rarity += 1.0 /
                static_cast<double>(
                    std::max(1, availability[customer]));
        }
    }

    std::vector<std::uint8_t>& retained = pruneRetainedScratch;
    retained.assign(poolList.size(), 0U);
    std::vector<std::size_t>& retainedIndices =
        pruneRetainedIndexScratch;
    retainedIndices.clear();
    retainedIndices.reserve(
        std::max(targetSize, poolList.size()));

    auto retain = [&](const std::size_t index) {
        if (retained[index]) {
            return false;
        }
        retained[index] = 1U;
        retainedIndices.push_back(index);
        return true;
    };

    lastPruneProtectedCount = 0U;
    lastPruneGraceCount = 0U;
    lastPruneRarityCount = 0U;
    lastPruneDepotDiversityCount = 0U;
    lastPruneOrdinaryCount = 0U;

    // Tier 0: complete feasible backbones and current HGA/SP/population
    // incumbents are never removed, even if they temporarily exceed the
    // nominal target.
    for (std::size_t index = 0; index < poolList.size(); ++index) {
        if (pruneMetrics[index].protectedColumn &&
            retain(index)) {
            ++lastPruneProtectedCount;
        }
    }

    // Tier G: give a bounded number of the oldest as-yet unevaluated columns
    // one opportunity to enter the next LP snapshot. Protected columns are
    // already retained by Tier 0 and do not consume this independent quota.
    std::vector<hgasp::GraceCandidate> graceCandidates;
    for (std::size_t index = 0U; index < poolList.size(); ++index) {
        const PruneMetric& metric = pruneMetrics[index];
        if (retained[index] || !metric.hasEvaluationState ||
            metric.lpEvaluated) {
            continue;
        }
        graceCandidates.push_back(
            {index, metric.insertedMasterVersion});
    }
    for (const std::size_t index :
         hgasp::selectOldestGraceCandidates(
             std::move(graceCandidates),
             newColumnGraceCapacity())) {
        if (retain(index)) {
            ++lastPruneGraceCount;
        }
    }

    std::vector<int>& retainedCoverage =
        pruneRetainedCoverageScratch;
    retainedCoverage.assign(
        static_cast<std::size_t>(data->numV), 0);
    for (std::size_t index = 0; index < poolList.size(); ++index) {
        for (const int customer : poolList[index].sequence) {
            if (retained[index]) {
                ++retainedCoverage[customer];
            }
        }
    }

    std::vector<std::size_t>& candidates = pruneCandidateScratch;
    candidates.resize(poolList.size());
    for (std::size_t index = 0; index < poolList.size(); ++index) {
        candidates[index] = index;
    }

    // Tier 1: protect sparse customer coverage. Columns that cover customers
    // with few alternatives rank first; each customer keeps up to two
    // alternatives when the target budget permits.
    std::stable_sort(
        candidates.begin(),
        candidates.end(),
        [&](const std::size_t left, const std::size_t right) {
            const double leftRarity =
                pruneMetrics[left].rarity;
            const double rightRarity =
                pruneMetrics[right].rarity;
            if (std::fabs(leftRarity - rightRarity) > 1.0e-12) {
                return leftRarity > rightRarity;
            }
            const double leftReducedCost =
                pruneMetrics[left].reducedCost;
            const double rightReducedCost =
                pruneMetrics[right].reducedCost;
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
                pruneMetrics[left].efficiency;
            const double rightEfficiency =
                pruneMetrics[right].efficiency;
            if (std::fabs(leftEfficiency - rightEfficiency) >
                1.0e-12) {
                return leftEfficiency > rightEfficiency;
            }
            return poolMasterKeys[left].customers <
                poolMasterKeys[right].customers;
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
                pruneMetrics[left].reducedCost;
            const double rightReducedCost =
                pruneMetrics[right].reducedCost;
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
                pruneMetrics[left].efficiency;
            const double rightEfficiency =
                pruneMetrics[right].efficiency;
            if (std::fabs(leftEfficiency - rightEfficiency) >
                1.0e-12) {
                return leftEfficiency > rightEfficiency;
            }
            return pruneMetrics[left].routeCost <
                pruneMetrics[right].routeCost;
        });
    for (const std::size_t index : candidates) {
        if (retainedIndices.size() >= targetSize) {
            break;
        }
        const int depot = pruneMetrics[index].depot;
        if (retainedDepot[depot] || !retain(index)) {
            continue;
        }
        retainedDepot[depot] = true;
        ++lastPruneDepotDiversityCount;
    }

    // Tier 3: fill remaining capacity in the stable Tier-2 order (current
    // reduced cost, efficiency, then raw route cost).
    for (const std::size_t index : candidates) {
        if (retainedIndices.size() >= targetSize) {
            break;
        }
        if (retain(index)) {
            ++lastPruneOrdinaryCount;
        }
    }

    std::vector<bool> changedDepot(
        static_cast<std::size_t>(data->numL), false);
    for (std::size_t index = 0U;
         index < poolList.size();
         ++index) {
        if (!retained[index]) {
            removeActiveMasterKey(poolMasterKeys[index]);
            changedDepot.at(
                static_cast<std::size_t>(
                    poolList[index].locId)) = true;
        }
    }

    std::vector<StandaloneRoute> retainedPool;
    std::vector<MasterColumnKey> retainedKeys;
    retainedPool.reserve(retainedIndices.size());
    retainedKeys.reserve(retainedIndices.size());
    for (const std::size_t index : retainedIndices) {
        retainedPool.push_back(std::move(poolList[index]));
        retainedKeys.push_back(std::move(poolMasterKeys[index]));
    }
    hgasp::recordHotPathCounter(
        hgasp::HotPathCounter::RoutePoolMasterKeyCacheEvictions,
        static_cast<std::uint64_t>(
            poolList.size() - retainedIndices.size()));
    poolList = std::move(retainedPool);
    poolMasterKeys = std::move(retainedKeys);

    rebuildMasterIndex();
    pruneColumnSidecars();
    // A version denotes one immutable master-column snapshot. Pruning
    // removes and reorders columns after admission, so consumers must not
    // reuse an index or LP snapshot built for the pre-prune pool.
    ++masterVersion;
    for (std::size_t depot = 0U;
         depot < changedDepot.size();
         ++depot) {
        if (changedDepot[depot]) {
            ++depotMasterVersions[depot];
        }
    }
    
    std::cout << "[RoutePool] Pruning complete. New size: "
              << poolList.size()
              << " tiers={protected:" << lastPruneProtectedCount
              << ",grace:" << lastPruneGraceCount
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
            {"grace", static_cast<unsigned long long>(
                lastPruneGraceCount)},
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

RoutePool::FinalMasterDiagnostic
RoutePool::summarizeFinalMasterAgainstIncumbent(
        const Individual& incumbent) const {
    if (!incumbent.isFeasible) {
        throw std::invalid_argument(
            "final-master diagnostic requires a feasible incumbent");
    }
    if (poolList.size() != poolMasterKeys.size()) {
        throw std::logic_error(
            "final-master diagnostic found an invalid key sidecar");
    }

    FinalMasterDiagnostic diagnostic;
    diagnostic.activeColumnCount = poolList.size();
    diagnostic.archivedColumnCount = bestArchivedColumns.size();
    diagnostic.dominanceReplacementCount = dominanceReplacementCount;
    diagnostic.dominanceReplacementTotalGain =
        masterRepresentativeReplacementCostGain;

    // Recompute keys without calling masterKey(): the latter deliberately
    // records hot-path work, while this terminal-only audit must not alter
    // the counters it is intended to explain.
    const auto canonicalKey = [](const StandaloneRoute& route) {
        return hgasp::makeRouteKey(route);
    };

    std::unordered_map<
        MasterColumnKey,
        StandaloneRoute,
        MasterColumnKeyHash> bestKnownByKey;
    bestKnownByKey.reserve(
        poolList.size() + bestArchivedColumns.size());
    const auto observe = [&](const StandaloneRoute& route) {
        const MasterColumnKey key = canonicalKey(route);
        const auto found = bestKnownByKey.find(key);
        if (found == bestKnownByKey.end()) {
            bestKnownByKey.emplace(key, route);
        } else if (preferredMasterRoute(route, found->second)) {
            found->second = route;
        }
    };
    for (const StandaloneRoute& route : poolList) observe(route);
    for (const StandaloneRoute& route : searchArchive) observe(route);
    for (const auto& [key, route] : bestArchivedColumns) {
        if (canonicalKey(route) != key) {
            throw std::logic_error(
                "final-master diagnostic found a stale archive key");
        }
        observe(route);
    }

    std::unordered_set<MasterColumnKey, MasterColumnKeyHash> activeKeys;
    activeKeys.reserve(poolList.size());
    for (std::size_t index = 0U; index < poolList.size(); ++index) {
        const StandaloneRoute& route = poolList[index];
        const MasterColumnKey key = canonicalKey(route);
        if (key != poolMasterKeys[index]) {
            throw std::logic_error(
                "final-master diagnostic found a stale active key");
        }
        const bool unique = activeKeys.insert(key).second;
        const auto best = bestKnownByKey.find(key);
        if (!unique ||
            (best != bestKnownByKey.end() &&
             preferredMasterRoute(best->second, route))) {
            ++diagnostic.sameKeyActiveRepresentativeViolations;
        }
    }
    if (activeKeys.size() != masterColumnIndex.size()) {
        throw std::logic_error(
            "final-master diagnostic found an invalid active index size");
    }
    for (const auto& [key, index] : masterColumnIndex) {
        if (index >= poolMasterKeys.size() || poolMasterKeys[index] != key) {
            throw std::logic_error(
                "final-master diagnostic found an invalid active index");
        }
    }

    for (const auto& [key, route] : bestArchivedColumns) {
        const auto best = bestKnownByKey.find(key);
        if (best != bestKnownByKey.end() &&
            preferredMasterRoute(best->second, route)) {
            ++diagnostic.sameKeyArchiveRepresentativeViolations;
        }
    }

    std::vector<StandaloneRoute> incumbentRoutes;
    for (int depot = 0; depot < data->numL; ++depot) {
        if (!incumbent.location[depot].used) continue;
        for (int slot = 0;
             slot < incumbent.location[depot].maxVisit;
             ++slot) {
            const Route* route =
                incumbent.location[depot].depots[slot].route;
            if (!route->used || route->nbClients == 0) continue;
            StandaloneRoute valueRoute;
            valueRoute.locId = depot;
            valueRoute.totalLoad = route->load;
            valueRoute.routeCost = route->dis;
            for (const Node* node = route->depot->next;
                 !node->isDepot;
                 node = node->next) {
                valueRoute.sequence.push_back(node->cour);
            }
            incumbentRoutes.push_back(std::move(valueRoute));
        }
    }

    constexpr double costTolerance = 1e-9;
    for (const StandaloneRoute& route : incumbentRoutes) {
        const MasterColumnKey key = canonicalKey(route);
        const auto active = masterColumnIndex.find(key);
        if (active == masterColumnIndex.end()) {
            ++diagnostic
                .finalIncumbentRoutesWithoutMatchingMasterKey;
        } else if (
            poolList[active->second].routeCost <
                route.routeCost - costTolerance) {
            ++diagnostic
                .finalIncumbentRoutesWithCheaperMatchingActiveKey;
        }
        const auto archived = bestArchivedColumns.find(key);
        if (archived != bestArchivedColumns.end() &&
            archived->second.routeCost <
                route.routeCost - costTolerance) {
            ++diagnostic
                .finalIncumbentRoutesWithCheaperMatchingArchiveKey;
        }
    }
    return diagnostic;
}
double& RoutePool::usefulScoreBaseline(
        hgasp::ColumnProvenanceStats& stats,
        const hgasp::SpFeedbackScope scope,
        const std::optional<hgasp::DepotConfigurationKey>&
            configuration) {
    if (scope == hgasp::SpFeedbackScope::Global) {
        return stats.globalBestObservedUsefulScore;
    }
    if (!configuration.has_value()) {
        throw std::logic_error(
            "local useful-score baseline lacks a configuration");
    }

    auto found = std::find_if(
        stats.localUsefulScoreBaselines.begin(),
        stats.localUsefulScoreBaselines.end(),
        [&](const hgasp::ColumnProvenanceStats::
                LocalUsefulScoreBaseline& baseline) {
            return baseline.configuration == *configuration;
        });
    if (found != stats.localUsefulScoreBaselines.end()) {
        found->lastTouchedFeedback = creditFeedbackSequence;
        return found->bestObservedUsefulScore;
    }

    if (stats.localUsefulScoreBaselines.size() >=
        localCreditBaselineCapacity) {
        // Deterministic LRU eviction keeps per-column memory bounded by the
        // configured dynamic-island limit. The canonical configuration key
        // resolves the otherwise theoretical equal-age tie.
        const auto evicted = std::min_element(
            stats.localUsefulScoreBaselines.begin(),
            stats.localUsefulScoreBaselines.end(),
            [](const hgasp::ColumnProvenanceStats::
                    LocalUsefulScoreBaseline& left,
               const hgasp::ColumnProvenanceStats::
                    LocalUsefulScoreBaseline& right) {
                if (left.lastTouchedFeedback !=
                    right.lastTouchedFeedback) {
                    return left.lastTouchedFeedback <
                        right.lastTouchedFeedback;
                }
                return left.configuration <
                    right.configuration;
            });
        stats.localUsefulScoreBaselines.erase(evicted);
        ++localCreditBaselineEvictionCount;
    }
    stats.localUsefulScoreBaselines.push_back(
        {*configuration, 0.0, creditFeedbackSequence});
    return stats.localUsefulScoreBaselines.back()
        .bestObservedUsefulScore;
}

std::vector<hgasp::OriginCredit>
RoutePool::applySpFeedbackAndCollectCredits(
        const std::vector<StandaloneRoute>& snapshot,
        const SpResult& result,
        const int iteration,
        const hgasp::SpFeedbackScope scope,
        std::optional<hgasp::DepotConfigurationKey>
            configuration) {
    if (scope == hgasp::SpFeedbackScope::Global) {
        if (configuration.has_value()) {
            throw std::invalid_argument(
                "global SP credit feedback must not carry a local "
                "configuration");
        }
    } else {
        if (!configuration.has_value() ||
            configuration->openDepots.empty() ||
            !std::is_sorted(
                configuration->openDepots.begin(),
                configuration->openDepots.end()) ||
            std::adjacent_find(
                configuration->openDepots.begin(),
                configuration->openDepots.end()) !=
                configuration->openDepots.end()) {
            throw std::invalid_argument(
                "local SP credit feedback requires a canonical "
                "configuration");
        }
        for (const int depot : configuration->openDepots) {
            if (depot < 0 || depot >= data->numL) {
                throw std::invalid_argument(
                    "local SP credit feedback references an unknown "
                    "depot");
            }
        }
        for (const StandaloneRoute& route : snapshot) {
            if (!std::binary_search(
                    configuration->openDepots.begin(),
                    configuration->openDepots.end(),
                    route.locId)) {
                throw std::invalid_argument(
                    "local SP credit snapshot contains a route outside "
                    "its fixed configuration");
            }
        }
    }
    ++creditFeedbackSequence;

    const auto validateSize =
        [&](const std::vector<double>& values,
            const char* const name) {
            if (!values.empty() &&
                values.size() != snapshot.size()) {
                throw std::invalid_argument(
                    std::string(name) +
                    " does not match the SP snapshot");
            }
        };
    validateSize(result.routeLpValues, "route LP values");
    validateSize(
        result.routeReducedCosts, "route reduced costs");

    std::vector<double> selectionFrequency(
        snapshot.size(), 0.0);
    if (!result.candidateSolutions.empty()) {
        const double weight =
            1.0 /
            static_cast<double>(
                result.candidateSolutions.size());
        for (const SpSolution& solution :
             result.candidateSolutions) {
            for (const std::size_t index :
                 solution.selectedRouteIndices) {
                if (index >= snapshot.size()) {
                    throw std::invalid_argument(
                        "SP candidate references a route outside the "
                        "credit snapshot");
                }
                selectionFrequency[index] += weight;
            }
        }
    }
    std::vector<unsigned char> primary(
        snapshot.size(), 0U);
    if (!result.candidateSolutions.empty()) {
        for (const std::size_t index :
             result.candidateSolutions.front()
                 .selectedRouteIndices) {
            if (index >= snapshot.size()) {
                throw std::invalid_argument(
                    "primary SP candidate references a route outside "
                    "the credit snapshot");
            }
            primary[index] = 1U;
        }
    }

    using CreditKey =
        std::tuple<int, std::uint64_t, std::uint64_t>;
    struct AccumulatedOriginFeedback {
        double credit = 0.0;
        double selectedColumns = 0.0;
        double historicalChallengerSelectedColumns = 0.0;
        hgasp::DepotConfigurationKey challengerConfiguration;
    };
    std::map<CreditKey, AccumulatedOriginFeedback> accumulated;
    constexpr double emaAlpha = 0.20;
    const double reducedCostScale =
        hgasp::reducedCostRobustScale(
            result.routeReducedCosts);
    for (std::size_t index = 0;
         index < snapshot.size();
         ++index) {
        const double lpUsage =
            result.routeLpValues.empty()
                ? 0.0
                : std::clamp(
                      result.routeLpValues[index],
                      0.0,
                      1.0);
        const double poolSupport =
            std::clamp(
                selectionFrequency[index], 0.0, 1.0);
        const bool selected =
            primary[index] != 0U;
        if (lpUsage <= 1.0e-9 &&
            poolSupport <= 1.0e-9 &&
            !selected) {
            continue;
        }
        const double reducedCostQuality =
            result.routeReducedCosts.empty()
                ? 0.0
                : hgasp::normalizedReducedCostQuality(
                      result.routeReducedCosts[index],
                      reducedCostScale);
        const double usefulScore =
            0.45 * lpUsage +
            0.30 * poolSupport +
            0.15 * (selected ? 1.0 : 0.0) +
            0.10 * reducedCostQuality;
        const MasterColumnKey key =
            masterKey(snapshot[index]);
        const auto found = columnProvenance.find(key);
        if (found == columnProvenance.end()) {
            continue;
        }
        hgasp::ColumnProvenanceStats& stats =
            found->second;
        stats.lpUsageEma =
            (1.0 - emaAlpha) * stats.lpUsageEma +
            emaAlpha * lpUsage;
        stats.reducedCostQualityEma =
            (1.0 - emaAlpha) *
                stats.reducedCostQualityEma +
            emaAlpha * reducedCostQuality;
        stats.spSelectionFrequencyEma =
            (1.0 - emaAlpha) *
                stats.spSelectionFrequencyEma +
            emaAlpha * poolSupport;
        double& bestObservedUsefulScore =
            usefulScoreBaseline(
                stats, scope, configuration);
        const double credit = std::max(
            0.0,
            usefulScore - bestObservedUsefulScore);
        bestObservedUsefulScore = std::max(
            bestObservedUsefulScore,
            usefulScore);
        stats.cumulativeUsefulCredit += credit;
        if (selected || poolSupport > 0.0) {
            stats.lastSelectedIteration = iteration;
        }
        if (selected) {
            const hgasp::ChallengerRouteOrigin& historicalOrigin =
                stats.bestRouteOriginChallenger.valid()
                    ? stats.bestRouteOriginChallenger
                    : stats.firstChallengerOrigin;
            if (historicalOrigin.valid()) {
                AccumulatedOriginFeedback& historical =
                    accumulated[std::make_tuple(
                        historicalOrigin.slot,
                        0U,
                        historicalOrigin.generation)];
                historical.historicalChallengerSelectedColumns += 1.0;
                historical.challengerConfiguration =
                    historicalOrigin.configuration;
            }
        }

        const hgasp::RouteSubmissionContext& origin =
            stats.bestRouteOrigin;
        if (origin.firstPopulation < 0) {
            continue;
        }
        const auto attribute =
            [&](const CreditKey& key, const double fraction) {
                AccumulatedOriginFeedback& feedback =
                    accumulated[key];
                feedback.credit += fraction * credit;
                if (selected) {
                    feedback.selectedColumns += fraction;
                }
            };
        if (!selected && credit <= 1.0e-12) {
            continue;
        }
        if (origin.secondPopulation >= 0 &&
            origin.secondPopulation !=
                origin.firstPopulation) {
            attribute(
                std::make_tuple(
                    origin.firstPopulation,
                    origin.traceId,
                    origin.firstChallengerGeneration),
                0.5);
            attribute(
                std::make_tuple(
                    origin.secondPopulation,
                    origin.traceId,
                    origin.secondChallengerGeneration),
                0.5);
        } else {
            attribute(
                std::make_tuple(
                    origin.firstPopulation,
                    origin.traceId,
                    origin.firstChallengerGeneration),
                1.0);
        }
    }

    std::vector<hgasp::OriginCredit> credits;
    credits.reserve(accumulated.size());
    for (const auto& item : accumulated) {
        hgasp::OriginCredit credit;
        credit.population = std::get<0>(item.first);
        credit.traceId = std::get<1>(item.first);
        credit.challengerGeneration = std::get<2>(item.first);
        credit.credit = item.second.credit;
        credit.selectedColumns = item.second.selectedColumns;
        credit.historicalChallengerSelectedColumns =
            item.second.historicalChallengerSelectedColumns;
        credit.challengerConfiguration =
            item.second.challengerConfiguration;
        credits.push_back(credit);
    }
    return credits;
}
