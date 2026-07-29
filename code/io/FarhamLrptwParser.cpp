#include "InstanceLoader.h"

#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace hgasp {
namespace {

std::vector<std::vector<double>> readNumericRows(
    const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to open instance file: " + path);
    }

    std::vector<std::vector<double>> rows;
    std::string line;
    int lineNumber = 0;
    while (std::getline(input, line)) {
        ++lineNumber;
        const std::size_t first = line.find_first_not_of(" \t\r\n");
        if (first == std::string::npos || line[first] == '#') {
            continue;
        }
        std::istringstream rowInput(line);
        std::vector<double> row;
        double value = 0.0;
        while (rowInput >> value) {
            if (!std::isfinite(value)) {
                throw std::runtime_error(
                    "invalid Farham LRPTW instance '" + path +
                    "': non-finite value on line " +
                    std::to_string(lineNumber));
            }
            row.push_back(value);
        }
        if (!rowInput.eof()) {
            throw std::runtime_error(
                "invalid Farham LRPTW instance '" + path +
                "': non-numeric token on line " +
                std::to_string(lineNumber));
        }
        if (!row.empty()) {
            rows.push_back(std::move(row));
        }
    }
    return rows;
}

int asInteger(
    const double value,
    const std::string& field,
    const std::string& path) {
    if (std::floor(value) != value ||
        value < static_cast<double>(std::numeric_limits<int>::min()) ||
        value > static_cast<double>(std::numeric_limits<int>::max())) {
        throw std::runtime_error(
            "invalid Farham LRPTW instance '" + path +
            "': " + field + " must be an integer");
    }
    return static_cast<int>(value);
}

void requireWidth(
    const std::vector<double>& row,
    const std::size_t expected,
    const std::string& section,
    const std::string& path) {
    if (row.size() != expected) {
        throw std::runtime_error(
            "invalid Farham LRPTW instance '" + path + "': " +
            section + " row has " + std::to_string(row.size()) +
            " fields; expected " + std::to_string(expected));
    }
}

std::string stemOf(const std::string& path) {
    const std::size_t slash = path.find_last_of("/\\");
    const std::size_t begin =
        slash == std::string::npos ? 0 : slash + 1;
    const std::size_t dot = path.find_last_of('.');
    const std::size_t end =
        dot == std::string::npos || dot < begin ? path.size() : dot;
    return path.substr(begin, end - begin);
}

}  // namespace

ParsedInstance parseFarhamLrptw(const std::string& path) {
    const std::vector<std::vector<double>> rows = readNumericRows(path);
    if (rows.size() < 2) {
        throw std::runtime_error(
            "invalid Farham LRPTW instance '" + path +
            "': missing SIZE or VEHICLE row");
    }

    requireWidth(rows[0], 3, "SIZE", path);
    const int cdcCount = asInteger(rows[0][0], "NUM_CDC", path);
    const int depotCount =
        asInteger(rows[0][1], "NUM_SATELLITE", path);
    const int customerCount =
        asInteger(rows[0][2], "NUM_CUSTOMER", path);
    if (cdcCount != 0) {
        throw std::runtime_error(
            "unsupported Farham instance '" + path +
            "': NUM_CDC must be 0 for single-echelon CLRP-TW");
    }
    if (depotCount <= 0 || customerCount <= 0) {
        throw std::runtime_error(
            "invalid Farham LRPTW instance '" + path +
            "': depot and customer counts must be positive");
    }

    requireWidth(rows[1], 3, "VEHICLE", path);
    const std::size_t requiredRows =
        static_cast<std::size_t>(2 + depotCount + customerCount);
    if (rows.size() < requiredRows) {
        throw std::runtime_error(
            "invalid Farham LRPTW instance '" + path +
            "': fewer depot/customer records than declared in SIZE");
    }

    ParsedInstance parsed;
    parsed.name = stemOf(path);
    parsed.requiredMode = ProblemMode::ClrpTw;
    parsed.format = InstanceFormat::FarhamText;
    // Farham et al. use integer Euclidean arc lengths, truncated downward,
    // for both route cost and travel time.  Keeping the policy explicit is
    // essential: exact/nearest Euclidean values change published optima.
    parsed.matrixPolicy.travelCostRounding = NumericRounding::Floor;
    parsed.matrixPolicy.travelTimeRounding = NumericRounding::Floor;
    parsed.vehicleFixedCost = rows[1][1];
    parsed.vehicleCapacity =
        asInteger(rows[1][2], "vehicle capacity", path);
    if (parsed.vehicleFixedCost < 0.0 ||
        parsed.vehicleCapacity <= 0) {
        throw std::runtime_error(
            "invalid Farham LRPTW instance '" + path +
            "': vehicle fixed cost must be non-negative and capacity positive");
    }

    std::unordered_set<int> depotIds;
    for (int index = 0; index < depotCount; ++index) {
        const std::vector<double>& row =
            rows[static_cast<std::size_t>(2 + index)];
        requireWidth(row, 6, "SATELLITE", path);

        ParsedDepot depot;
        depot.originalId = asInteger(row[0], "satellite id", path);
        if (!depotIds.insert(depot.originalId).second) {
            throw std::runtime_error(
                "invalid Farham LRPTW instance '" + path +
                "': duplicate satellite id " +
                std::to_string(depot.originalId));
        }
        depot.x = row[1];
        depot.y = row[2];
        depot.closeTime = row[3];
        depot.openingCost = row[4];
        depot.capacity = asInteger(row[5], "satellite capacity", path);
        if (depot.closeTime < 0.0 ||
            depot.openingCost < 0.0 ||
            depot.capacity <= 0) {
            throw std::runtime_error(
                "invalid Farham LRPTW instance '" + path +
                "': satellite close time and fixed cost must be "
                "non-negative and capacity positive");
        }
        parsed.depots.push_back(depot);
    }

    // Some published files physically contain all 100 Solomon customers
    // even when SIZE declares a 10/25/40/50-customer derived instance.
    // Only the declared prefix belongs to the active LRPTW instance.
    std::unordered_set<int> customerIds;
    for (int index = 0; index < customerCount; ++index) {
        const std::vector<double>& row =
            rows[static_cast<std::size_t>(2 + depotCount + index)];
        requireWidth(row, 7, "CUSTOMER", path);

        ParsedCustomer customer;
        customer.originalId =
            asInteger(row[0], "customer id", path);
        if (!customerIds.insert(customer.originalId).second) {
            throw std::runtime_error(
                "invalid Farham LRPTW instance '" + path +
                "': duplicate customer id " +
                std::to_string(customer.originalId));
        }
        customer.x = row[1];
        customer.y = row[2];
        customer.demand =
            asInteger(row[3], "customer demand", path);
        customer.readyTime = row[4];
        customer.dueTime = row[5];
        customer.serviceTime = row[6];
        if (customer.demand < 0 ||
            customer.readyTime < 0.0 ||
            customer.dueTime < customer.readyTime ||
            customer.serviceTime < 0.0) {
            throw std::runtime_error(
                "invalid Farham LRPTW instance '" + path +
                "': customer demand/times are inconsistent");
        }
        if (customer.demand > parsed.vehicleCapacity) {
            throw std::runtime_error(
                "invalid Farham LRPTW instance '" + path +
                "': customer demand exceeds vehicle capacity");
        }
        parsed.customers.push_back(customer);
    }

    return parsed;
}

}  // namespace hgasp
