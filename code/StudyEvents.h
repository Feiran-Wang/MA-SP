// Author: Feiran
// Record search events and serialize improving complete solutions.

#ifndef MASP_STUDY_EVENTS_H_
#define MASP_STUDY_EVENTS_H_

#include "Individual.h"
#include <cmath>
#include <cstdlib>
#include <limits>
#include <map>
#include <stdexcept>
#include <time.h>

class StudyEvents {
  public:
    read_data *data;
    std::string source = "initialization";
    std::string candidateId, sourcePopulationId;
    long long physicalAttempts = 0;
    long long mutationDecisions = 0;
    long long mutationsExecuted = 0;
    long long invalidSPOutputs = 0;
    std::ofstream out;
    double wallOrigin;
    std::map<std::string, double> best;
    std::map<std::string, int> indices;

    explicit StudyEvents(read_data *d) : data(d), wallOrigin(monotonic()) {
        if (!d->experimentMode)
            return;
        if (const char *origin = std::getenv("MASP_RUN_MONOTONIC_START")) {
            char *end = nullptr;
            const double parsed = std::strtod(origin, &end);
            if (!end || *end || !std::isfinite(parsed) || parsed > wallOrigin)
                throw std::invalid_argument("Invalid MASP_RUN_MONOTONIC_START");
            wallOrigin = parsed;
        }
        out.exceptions(std::ios::failbit | std::ios::badbit);
        out.open(d->experimentEventsPath.empty() ? "study_events.jsonl" : d->experimentEventsPath);
        event("run_started", "\"arm\":" + quote(d->experimentArm));
    }
    static double monotonic() {
        timespec stamp;
        if (clock_gettime(CLOCK_MONOTONIC, &stamp) != 0)
            throw std::runtime_error("Cannot read monotonic study clock");
        return stamp.tv_sec + stamp.tv_nsec / 1e9;
    }
    static double cpu() {
        return static_cast<double>(clock()) / CLOCKS_PER_SEC;
    }
    double wall() const {
        return monotonic() - wallOrigin;
    }
    static std::string quote(const std::string &value) {
        std::ostringstream result;
        result << '"';
        for (unsigned char c : value) {
            if (c == '"' || c == '\\')
                result << '\\' << c;
            else if (c == '\n')
                result << "\\n";
            else if (c == '\r')
                result << "\\r";
            else if (c == '\t')
                result << "\\t";
            else if (c < 32)
                result << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c)
                       << std::dec;
            else
                result << c;
        }
        result << '"';
        return result.str();
    }
    static std::string number(double value) {
        if (!std::isfinite(value))
            return "null";
        std::ostringstream result;
        result << std::setprecision(17) << value;
        return result.str();
    }
    void event(const std::string &type, const std::string &fields = "") {
        if (!data->experimentMode)
            return;
        out << "{\"type\":" << quote(type) << ",\"main_counter\":" << data->iteration
            << ",\"physical_attempts\":" << physicalAttempts << ",\"cpu_seconds\":" << number(cpu())
            << ",\"evolution_clock_s\":" << number(data->evolutionCPU())
            << ",\"wall_seconds\":" << number(wall()) << ",\"source\":" << quote(source);
        if (!fields.empty())
            out << ',' << fields;
        out << "}\n";
        out.flush();
    }
    std::string configuration(const Individual *indiv) const {
        std::ostringstream result;
        result << '[';
        bool first = true;
        for (int l = 0; l < data->numL; ++l)
            if (indiv->location[l].used) {
                if (!first)
                    result << ',';
                first = false;
                result << l;
            }
        result << ']';
        return result.str();
    }

    bool observe(const std::string &stream, Individual *indiv, const std::string &origin) {
        if (!data->experimentMode || !indiv->isFeasible)
            return false;
        const auto existing = best.find(stream);
        if (existing != best.end() && indiv->fit >= existing->second)
            return false;
        indiv->isRight();
        best[stream] = indiv->fit;
        const int index = ++indices[stream];
        std::ostringstream fields;
        fields << "\"stream\":" << quote(stream) << ",\"event_index\":" << index
               << ",\"objective_exact\":" << number(indiv->fit)
               << ",\"solution_id\":" << quote(stream + "_" + std::to_string(index))
               << ",\"verified_for_reporting\":true,\"incumbent_source\":" << quote(origin)
               << ",\"source_population_id\":"
               << (sourcePopulationId.empty() ? "null" : quote(sourcePopulationId))
               << ",\"source_candidate_id\":" << (candidateId.empty() ? "null" : quote(candidateId))
               << ",\"parent_population_ids\":null"
               << ",\"used_depots\":" << configuration(indiv) << ",\"routes\":[";
        appendRoutes(fields, indiv);
        fields << ']';
        event("incumbent", fields.str());
        return true;
    }

  private:
    void appendRoutes(std::ostream &out, const Individual *indiv) const {
        // Serialize depot and route slots in their existing traversal order.
        bool first = true;
        for (int l = 0; l < data->numL; ++l) {
            for (int k = 0; k < indiv->location[l].maxVisit; ++k) {
                const Node *start = &indiv->location[l].depots[k];
                if (!start->route->used)
                    continue;
                if (!first)
                    out << ',';
                first = false;
                out << "{\"depot\":" << l << ",\"load\":" << start->route->load
                    << ",\"customers\":[";
                const Node *node = start->next;
                bool firstCustomer = true;
                while (!node->isDepot) {
                    if (!firstCustomer)
                        out << ',';
                    firstCustomer = false;
                    out << node->cour;
                    node = node->next;
                }
                out << "]}";
            }
        }
    }
};

#endif
