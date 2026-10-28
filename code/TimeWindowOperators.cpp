// Author: Feiran
// Construct, repair, mutate, and improve time-window solutions.

#include "TimeWindowOperators.h"
#include "Individual.h"
#include <numeric>
#include <unordered_map>

namespace masptw {
namespace {
using Plan = std::vector<StandaloneRoute>;
bool expired(const read_data *d) {
    return double(clock()) / CLOCKS_PER_SEC >= d->timeLimit;
}
bool evaluate(StandaloneRoute &r, const read_data *d) {
    if (r.locId < 0 || r.locId >= d->numL)
        return false;
    long long load = 0;
    double cost = 0;
    int previous = d->numV + r.locId;
    for (int c : r.sequence) {
        if (c < 0 || c >= d->numV)
            return false;
        load += d->demCus[c];
        cost += d->D[previous][c];
        previous = c;
    }
    if (load > d->vehCap || !d->routeTimeFeasible(r.locId, r.sequence))
        return false;
    if (!r.sequence.empty())
        cost += d->D[previous][d->numV + r.locId];
    r.totalLoad = int(load);
    r.routeCost = cost;
    return true;
}
bool complete(Plan &p, const read_data *d) {
    std::vector<int> seen(d->numV, 0), counts(d->numL, 0);
    std::vector<long long> loads(d->numL, 0);
    for (auto &r : p) {
        if (!evaluate(r, d))
            return false;
        if (r.sequence.empty())
            continue;
        if (++counts[r.locId] > d->numV)
            return false;
        loads[r.locId] += r.totalLoad;
        if (loads[r.locId] > d->invLoc[d->numV + r.locId])
            return false;
        for (int c : r.sequence)
            if (++seen[c] != 1)
                return false;
    }
    return std::all_of(seen.begin(), seen.end(), [](int n) { return n == 1; });
}
// Keep depot/route/arc traversal and component summation aligned with Individual::evaluationDis.
double planObjective(const Plan &p, const read_data *d, double *travelOut = nullptr) {
    double travel = 0, opening = 0, vehicles = 0;

    for (int l = 0; l < d->numL; ++l) {
        int count = 0;
        for (const auto &r : p)
            if (r.locId == l && !r.sequence.empty()) {
                ++count;
                int previous = d->numV + l;
                for (int customer : r.sequence) {
                    travel += d->D[previous][customer];
                    previous = customer;
                }
                travel += d->D[previous][d->numV + l];
            }
        if (count) {
            opening += d->openCostLoc[d->numV + l];
            vehicles += d->usingCostVeh * count;
        }
    }
    if (travelOut)
        *travelOut = travel;
    return opening + vehicles + travel;
}
void compact(Plan &p) {
    p.erase(std::remove_if(p.begin(), p.end(),
                           [](const StandaloneRoute &r) { return r.sequence.empty(); }),
            p.end());
}

bool extract(const Individual *s, const read_data *d, Plan &p, bool strict) {
    std::unordered_map<const Node *, int> clients;
    for (int c = 0; c < d->numV; ++c)
        clients.emplace(&s->client[c], c);
    std::vector<int> seen(d->numV, 0);
    bool sound = true;
    for (int l = 0; l < d->numL; ++l) {
        for (int k = 0; k < s->location[l].maxVisit; ++k) {
            const Node *start = &s->location[l].depots[k];
            const Node *end = &s->location[l].endDepots[k];
            const Node *previous = start;
            const Node *node = start->next;
            StandaloneRoute r{l, 0.0, 0, {}};
            while (node != end) {
                auto found = clients.find(node);
                if (found == clients.end() || seen[found->second]) {
                    sound = false;
                    break;
                }
                int c = found->second;
                if (node->pre != previous || node->cour != c || node->isDepot ||
                    node->route != start->route)
                    sound = false;
                seen[c] = 1;
                r.sequence.push_back(c);
                previous = node;
                node = node->next;
            }
            if (node == end && end->pre != previous)
                sound = false;
            if (!r.sequence.empty())
                p.push_back(std::move(r));
        }
    }
    return !strict || sound;
}
std::vector<int> consecutiveIndices(int count) {
    std::vector<int> result(count);
    std::iota(result.begin(), result.end(), 0);
    return result;
}
struct Insertion {
    double delta;
    int route, depot, position;
};
bool insert(Plan &p, int c, const std::vector<int> &allowed, read_data *d, int shortlist,
            double exponent) {
    std::vector<long long> loads(d->numL, 0);
    std::vector<int> counts(d->numL, 0);
    for (const auto &r : p)
        if (!r.sequence.empty()) {
            loads[r.locId] += r.totalLoad;
            ++counts[r.locId];
        }
    std::vector<Insertion> best;
    auto offer = [&](Insertion item) {
        auto pos = std::lower_bound(
            best.begin(), best.end(), item,
            [](const Insertion &a, const Insertion &b) { return a.delta < b.delta; });
        best.insert(pos, item);
        if (int(best.size()) > shortlist)
            best.pop_back();
    };
    for (int l : allowed) {
        if (expired(d))
            return false;
        if (loads[l] + d->demCus[c] > d->invLoc[d->numV + l])
            continue;
        for (int i = 0; i < int(p.size()); ++i) {
            if (p[i].locId != l || p[i].totalLoad + d->demCus[c] > d->vehCap)
                continue;
            for (int at = 0; at <= int(p[i].sequence.size()); ++at) {
                auto r = p[i];
                r.sequence.insert(r.sequence.begin() + at, c);
                if (evaluate(r, d))
                    offer({r.routeCost - p[i].routeCost, i, l, at});
            }
        }
        StandaloneRoute r{l, 0.0, 0, {c}};
        if (counts[l] < d->numV && evaluate(r, d))
            offer({r.routeCost + d->usingCostVeh + (counts[l] ? 0.0 : d->openCostLoc[l + d->numV]),
                   -1, l, 0});
    }
    if (best.empty())
        return false;
    int rank =
        std::min(int(best.size()) - 1, int(std::pow(RandomDouble(0, 1), exponent) * best.size()));
    const auto chosen = best[rank];
    if (chosen.route < 0)
        p.push_back({chosen.depot, 0.0, 0, {c}});
    else
        p[chosen.route].sequence.insert(p[chosen.route].sequence.begin() + chosen.position, c);
    return evaluate(p[chosen.route < 0 ? p.size() - 1 : chosen.route], d);
}

// Floored distances need not satisfy triangle inequality; retained sequences require explicit checks.
Plan retained(const Plan &original, const read_data *d, const std::vector<bool> &remove) {
    Plan p;
    std::vector<bool> seen(d->numV, false);
    std::vector<long long> loads(d->numL, 0);
    for (const auto &source : original) {
        StandaloneRoute whole{source.locId, 0.0, 0, {}};
        for (int c : source.sequence)
            if (!remove[c] && !seen[c])
                whole.sequence.push_back(c);
        if (!whole.sequence.empty() && evaluate(whole, d) &&
            loads[whole.locId] + whole.totalLoad <= d->invLoc[d->numV + whole.locId]) {
            loads[whole.locId] += whole.totalLoad;
            for (int c : whole.sequence)
                seen[c] = true;
            p.push_back(std::move(whole));
            continue;
        }
        StandaloneRoute r{source.locId, 0.0, 0, {}};
        for (int c : source.sequence) {
            if (remove[c] || seen[c] ||
                loads[r.locId] + d->demCus[c] > d->invLoc[d->numV + r.locId])
                continue;
            auto candidate = r;
            candidate.sequence.push_back(c);
            if (!evaluate(candidate, d))
                continue;
            r = std::move(candidate);
            loads[r.locId] += d->demCus[c];
            seen[c] = true;
        }
        if (!r.sequence.empty())
            p.push_back(std::move(r));
    }
    return p;
}
bool recreate(Plan &p, read_data *d, const std::vector<int> &allowed, int attempt,
              bool construction) {
    std::vector<bool> seen(d->numV, false);
    for (const auto &r : p)
        for (int c : r.sequence)
            seen[c] = true;
    std::vector<int> missing;
    for (int c = 0; c < d->numV; ++c)
        if (!seen[c])
            missing.push_back(c);
    RandomShuffle(missing);
    if (attempt == 0)
        std::stable_sort(missing.begin(), missing.end(),
                         [&](int a, int b) { return d->dueTime[a] < d->dueTime[b]; });
    for (int c : missing)
        if (expired(d) || !insert(p, c, allowed, d, construction ? 8 : 4, construction ? 6.0 : 2.0))
            return false;
    return complete(p, d);
}
}
bool validate(const Individual *s, const read_data *d) {
    Plan p;
    if (!s || !d || !extract(s, d, p, true) || !complete(p, d))
        return false;
    double travel = 0;
    const double objective = planObjective(p, d, &travel);
    std::vector<int> loads(d->numL, 0), counts(d->numL, 0);
    for (const auto &r : p) {
        loads[r.locId] += r.totalLoad;
        ++counts[r.locId];
    }
    for (int l = 0; l < d->numL; ++l) {
        if (s->location[l].used != (counts[l] > 0) || s->location[l].remainInven != loads[l] ||
            s->location[l].splitTime != counts[l])
            return false;
        for (int k = 0; k < s->location[l].maxVisit; ++k) {
            const Node *start = &s->location[l].depots[k];
            const Node *end = &s->location[l].endDepots[k];
            const Route *route = start->route;
            if (!route || route->used != (start->next != end))
                return false;
            StandaloneRoute r{l, 0.0, 0, {}};
            for (const Node *n = start->next; n != end; n = n->next)
                r.sequence.push_back(n->cour);
            if (!evaluate(r, d) || route->load != r.totalLoad || !std::isfinite(route->dis) ||
                std::abs(route->dis - r.routeCost) > 1e-7)
                return false;
        }
    }
    return std::isfinite(s->fit) && std::isfinite(s->dis) && std::abs(s->fit - objective) <= 1e-7 &&
           std::abs(s->dis - travel) <= 1e-7;
}
bool construct(Individual *s, read_data *d, int rankPop) {
    auto allowed = consecutiveIndices(d->numL);
    if (rankPop >= 0 && rankPop < int(d->compactLocConfigure.size())) {
        allowed.clear();
        for (int v : d->compactLocConfigure[rankPop]) {
            if (v < d->numV || v >= d->numC)
                return false;
            allowed.push_back(v - d->numV);
        }
    }
    const int attempts = std::max(16, std::min(128, 2 * d->numV));
    for (int attempt = 0; attempt < attempts && !expired(d); ++attempt) {
        Plan p;
        if (recreate(p, d, allowed, attempt, true)) {
            s->rebuildFromSP(p);
            return true;
        }
    }
    return false;
}
bool repair(Individual *s, read_data *d) {
    if (validate(s, d))
        return true;
    Plan original;
    extract(s, d, original, false);
    const auto base = retained(original, d, std::vector<bool>(d->numV, false));
    const auto allowed = consecutiveIndices(d->numL);
    for (int attempt = 0; attempt < 12 && !expired(d); ++attempt) {
        auto p = base;
        if (recreate(p, d, allowed, attempt, false)) {
            s->rebuildFromSP(p);
            return true;
        }
    }
    if (d->experimentMode)
        ++d->twRepairFailures;
    return false;
}
void mutate(Individual *s, read_data *d) {
    Plan original;
    if (d->numV < 2 || !extract(s, d, original, true) || !complete(original, d))
        return;
    std::vector<double> starts(d->numV, 0);
    std::vector<int> membership(d->numV, -1);
    double distanceScale = 1, timeScale = 1, demandScale = 1;
    for (int c = 0; c < d->numV; ++c) {
        timeScale = std::max(timeScale, d->dueTime[c]);
        demandScale = std::max(demandScale, double(d->demCus[c]));
        for (int b = 0; b < d->numV; ++b)
            distanceScale = std::max(distanceScale, d->D[c][b]);
    }
    for (int i = 0; i < int(original.size()); ++i) {
        double time = 0;
        int previous = d->numV + original[i].locId;
        for (int c : original[i].sequence) {
            starts[c] = std::max(d->readyTime[c], time + d->D[previous][c]);
            time = starts[c] + d->serviceTime[c];
            previous = c;
            membership[c] = i;
        }
    }
    std::vector<bool> remove(d->numV, false);
    std::vector<int> selected{RandomInt(0, d->numV - 1)};
    remove[selected.front()] = true;
    const int count = d->experimentMode
                          ? d->requestedShawRemovalCount
                          : std::min(d->numV, std::max(2, int(d->ratioPerturbLength * d->numV)));
    while (int(selected.size()) < count && !expired(d)) {
        int a = selected[RandomInt(0, int(selected.size()) - 1)];
        std::vector<std::pair<double, int>> related;
        for (int b = 0; b < d->numV; ++b)
            if (!remove[b]) {
                double value = 0.40 * d->D[a][b] / distanceScale +
                               0.30 * std::abs(starts[a] - starts[b]) / timeScale +
                               0.15 *
                                   std::abs((d->readyTime[a] + d->dueTime[a] - d->readyTime[b] -
                                             d->dueTime[b]) *
                                            0.5) /
                                   timeScale +
                               0.10 * std::abs(double(d->demCus[a]) - d->demCus[b]) / demandScale +
                               0.05 * (membership[a] != membership[b]);
                related.emplace_back(value, b);
            }
        std::sort(related.begin(), related.end());
        int rank = std::min(int(related.size()) - 1,
                            int(std::pow(RandomDouble(0, 1), 6) * related.size()));
        selected.push_back(related[rank].second);
        remove[selected.back()] = true;
    }
    if (d->experimentMode) {
        ++d->twMutationCalls;
        d->twMutationRemovedCustomers += selected.size();
    }
    const auto base = retained(original, d, remove);
    for (int attempt = 0; attempt < 12 && !expired(d); ++attempt) {
        auto p = base;
        if (recreate(p, d, consecutiveIndices(d->numL), attempt, false)) {
            s->rebuildFromSP(p);
            return;
        }
    }
    if (d->experimentMode)
        ++d->twRepairFailures;
}
void localSearch(Individual *s, read_data *d) {
    Plan p;
    if (!extract(s, d, p, true) || !complete(p, d))
        return;
    auto objective = [&](const Plan &plan) { return planObjective(plan, d); };
    double current = objective(p);
    bool changed = false;
    std::vector<std::vector<int>> neighbours(d->numV);
    for (int a = 0; a < d->numV; ++a) {
        auto &near = neighbours[a];
        for (int b = 0; b < d->numV; ++b)
            if (a != b)
                near.push_back(b);
        std::stable_sort(near.begin(), near.end(),
                         [&](int x, int y) { return d->D[a][x] < d->D[a][y]; });
        if (near.size() > static_cast<std::size_t>(d->alpha))
            near.resize(d->alpha);
        if (d->experimentMode) {
            ++d->twNeighborhoodLists;
            d->twNeighborhoodEntries += near.size();
            d->twNeighborhoodMaximum = std::max(d->twNeighborhoodMaximum, int(near.size()));
        }
    }
    auto accept = [&](Plan candidate) {
        if (expired(d))
            return false;
        if (d->experimentMode)
            ++d->twLocalSearchCandidates;
        if (!complete(candidate, d)) {
            if (d->experimentMode)
                ++d->twLocalSearchRejected;
            return false;
        }
        double cost = objective(candidate);
        if (cost >= current - 1e-7)
            return false;
        compact(candidate);
        p = std::move(candidate);
        current = cost;
        changed = true;
        return true;
    };
    while (!expired(d)) {
        bool improved = false;
        std::vector<int> routeOf(d->numV), position(d->numV);
        for (int r = 0; r < int(p.size()); ++r)
            for (int i = 0; i < int(p[r].sequence.size()); ++i) {
                routeOf[p[r].sequence[i]] = r;
                position[p[r].sequence[i]] = i;
            }
        auto order = consecutiveIndices(d->numV);
        RandomShuffle(order);
        for (int a : order) {
            if (improved || expired(d))
                break;
            int r = routeOf[a], i = position[a];
            for (int b : neighbours[a]) {
                if (expired(d))
                    break;
                int t = routeOf[b], j = position[b];

                for (int after = 0; after < 2 && !improved; ++after) {
                    auto candidate = p;
                    candidate[r].sequence.erase(candidate[r].sequence.begin() + i);
                    int at = j + after - (r == t && i < j + after ? 1 : 0);
                    candidate[t].sequence.insert(candidate[t].sequence.begin() + at, a);
                    improved = accept(std::move(candidate));
                }
                if (improved)
                    break;
                auto candidate = p;
                std::swap(candidate[r].sequence[i], candidate[t].sequence[j]);
                if ((improved = accept(std::move(candidate))))
                    break;
                candidate = p;
                if (r == t) {
                    int first = std::min(i, j), last = std::max(i, j);
                    std::reverse(candidate[r].sequence.begin() + first,
                                 candidate[r].sequence.begin() + last + 1);
                } else {

                    auto tailR =
                        std::vector<int>(p[r].sequence.begin() + i + 1, p[r].sequence.end());
                    auto tailT =
                        std::vector<int>(p[t].sequence.begin() + j + 1, p[t].sequence.end());
                    candidate[r].sequence.resize(i + 1);
                    candidate[t].sequence.resize(j + 1);
                    candidate[r].sequence.insert(candidate[r].sequence.end(), tailT.begin(),
                                                 tailT.end());
                    candidate[t].sequence.insert(candidate[t].sequence.end(), tailR.begin(),
                                                 tailR.end());
                }
                if ((improved = accept(std::move(candidate))))
                    break;
            }
            if (improved || expired(d))
                break;

            int depot = -1;
            for (int l = 0; l < d->numL; ++l)
                if (l != p[r].locId)
                    if (depot < 0 || d->D[a][d->numV + l] < d->D[a][d->numV + depot])
                        depot = l;
            if (depot >= 0) {
                auto candidate = p;
                candidate[r].sequence.erase(candidate[r].sequence.begin() + i);
                candidate.push_back({depot, 0.0, 0, {a}});
                improved = accept(std::move(candidate));
                if (!improved) {
                    candidate = p;
                    candidate[r].locId = depot;
                    improved = accept(std::move(candidate));
                }
            }
        }
        if (!improved)
            break;
    }
    if (changed)
        s->rebuildFromSP(p);
}
}
