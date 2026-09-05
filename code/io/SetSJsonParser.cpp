#include "InstanceLoader.h"

#include <cmath>
#include <fstream>
#include <limits>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace hgasp {
namespace {

std::string readWholeFile(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to open instance file: " + path);
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    if (!input.good() && !input.eof()) {
        throw std::runtime_error("failed to read instance file: " + path);
    }
    return buffer.str();
}

std::string extractArray(
    const std::string& source,
    const std::string& key,
    const std::string& path) {
    const std::string marker = "\"" + key + "\"";
    const std::size_t keyPosition = source.find(marker);
    if (keyPosition == std::string::npos) {
        throw std::runtime_error(
            "invalid Set S JSON instance '" + path +
            "': missing '" + key + "' array");
    }
    const std::size_t open = source.find('[', keyPosition + marker.size());
    if (open == std::string::npos) {
        throw std::runtime_error(
            "invalid Set S JSON instance '" + path +
            "': malformed '" + key + "' array");
    }

    int depth = 0;
    bool inString = false;
    bool escaped = false;
    for (std::size_t cursor = open; cursor < source.size(); ++cursor) {
        const char character = source[cursor];
        if (inString) {
            if (escaped) {
                escaped = false;
            } else if (character == '\\') {
                escaped = true;
            } else if (character == '"') {
                inString = false;
            }
            continue;
        }
        if (character == '"') {
            inString = true;
        } else if (character == '[') {
            ++depth;
        } else if (character == ']') {
            --depth;
            if (depth == 0) {
                return source.substr(open + 1, cursor - open - 1);
            }
        }
    }
    throw std::runtime_error(
        "invalid Set S JSON instance '" + path +
        "': unterminated '" + key + "' array");
}

std::vector<std::string> extractObjects(
    const std::string& array,
    const std::string& key,
    const std::string& path) {
    std::vector<std::string> objects;
    int depth = 0;
    bool inString = false;
    bool escaped = false;
    std::size_t begin = std::string::npos;
    for (std::size_t cursor = 0; cursor < array.size(); ++cursor) {
        const char character = array[cursor];
        if (inString) {
            if (escaped) {
                escaped = false;
            } else if (character == '\\') {
                escaped = true;
            } else if (character == '"') {
                inString = false;
            }
            continue;
        }
        if (character == '"') {
            inString = true;
        } else if (character == '{') {
            if (depth == 0) {
                begin = cursor;
            }
            ++depth;
        } else if (character == '}') {
            --depth;
            if (depth < 0) {
                break;
            }
            if (depth == 0 && begin != std::string::npos) {
                objects.push_back(
                    array.substr(begin, cursor - begin + 1));
                begin = std::string::npos;
            }
        }
    }
    if (depth != 0 || objects.empty()) {
        throw std::runtime_error(
            "invalid Set S JSON instance '" + path +
            "': malformed or empty '" + key + "' array");
    }
    return objects;
}

double numericField(
    const std::string& object,
    const std::string& key,
    const std::string& path) {
    const std::regex expression(
        "\"" + key +
        "\"\\s*:\\s*(-?(?:[0-9]+(?:\\.[0-9]*)?|\\.[0-9]+)"
        "(?:[eE][+-]?[0-9]+)?)");
    std::smatch match;
    if (!std::regex_search(object, match, expression)) {
        throw std::runtime_error(
            "invalid Set S JSON instance '" + path +
            "': missing numeric field '" + key + "'");
    }
    try {
        const double value = std::stod(match[1].str());
        if (!std::isfinite(value)) {
            throw std::out_of_range(key);
        }
        return value;
    } catch (const std::exception&) {
        throw std::runtime_error(
            "invalid Set S JSON instance '" + path +
            "': field '" + key + "' is not finite");
    }
}

int integerField(
    const std::string& object,
    const std::string& key,
    const std::string& path) {
    const double value = numericField(object, key, path);
    if (std::floor(value) != value ||
        value < static_cast<double>(std::numeric_limits<int>::min()) ||
        value > static_cast<double>(std::numeric_limits<int>::max())) {
        throw std::runtime_error(
            "invalid Set S JSON instance '" + path +
            "': field '" + key + "' must be an integer");
    }
    return static_cast<int>(value);
}

void requireUniqueId(
    std::unordered_set<int>& seen,
    const int id,
    const char* kind,
    const std::string& path) {
    if (!seen.insert(id).second) {
        throw std::runtime_error(
            "invalid Set S JSON instance '" + path +
            "': duplicate " + kind + " id " + std::to_string(id));
    }
}

}  // namespace

ParsedInstance parseSetSJson(const std::string& path) {
    const std::string source = readWholeFile(path);
    const std::vector<std::string> customerObjects = extractObjects(
        extractArray(source, "customers", path), "customers", path);
    const std::vector<std::string> depotObjects = extractObjects(
        extractArray(source, "depots", path), "depots", path);

    ParsedInstance parsed;
    parsed.profile = makeSetSProblemProfile(InstanceFormat::SetSJson);
    parsed.legacyRealOrInteger = 0;

    std::unordered_set<int> customerIds;
    for (const std::string& object : customerObjects) {
        ParsedCustomer customer;
        customer.originalId = integerField(object, "index", path);
        requireUniqueId(
            customerIds, customer.originalId, "customer", path);
        customer.x = numericField(object, "x", path);
        customer.y = numericField(object, "y", path);
        customer.demand = integerField(object, "demand", path);
        customer.dueTime = std::numeric_limits<double>::infinity();
        if (customer.demand < 0) {
            throw std::runtime_error(
                "invalid Set S JSON instance '" + path +
                "': customer demand must be non-negative");
        }
        parsed.customers.push_back(customer);
    }

    std::unordered_set<int> depotIds;
    for (const std::string& object : depotObjects) {
        ParsedDepot depot;
        depot.originalId = integerField(object, "index", path);
        requireUniqueId(depotIds, depot.originalId, "depot", path);
        depot.x = numericField(object, "x", path);
        depot.y = numericField(object, "y", path);
        depot.capacity = integerField(object, "capacity", path);
        depot.openingCost = numericField(object, "costs", path);
        depot.closeTime = std::numeric_limits<double>::infinity();
        if (depot.capacity < 0) {
            throw std::runtime_error(
                "invalid Set S JSON instance '" + path +
                "': depot capacity must be non-negative");
        }
        parsed.depots.push_back(depot);
    }

    parsed.vehicleCapacity =
        integerField(source, "vehicle_capacity", path);
    parsed.vehicleFixedCost =
        numericField(source, "vehicle_costs", path);
    if (parsed.vehicleCapacity <= 0) {
        throw std::runtime_error(
            "invalid Set S JSON instance '" + path +
            "': vehicle capacity must be positive");
    }
    return parsed;
}

}  // namespace hgasp
