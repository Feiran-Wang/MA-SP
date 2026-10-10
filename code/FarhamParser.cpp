#include "read_data.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace {
[[noreturn]] void invalid(const std::string& path, const std::string& reason) {
    throw std::runtime_error("invalid Farham LRPTW instance '" + path + "': " + reason);
}

int integer(double value, const std::string& path, const std::string& field) {
    if (!std::isfinite(value) || std::floor(value) != value ||
        value < std::numeric_limits<int>::min() || value > std::numeric_limits<int>::max())
        invalid(path, field + " must be a representable integer");
    return static_cast<int>(value);
}

void width(const std::vector<double>& row, std::size_t expected,
           const std::string& path, const std::string& section) {
    if (row.size() != expected)
        invalid(path, section + " must have " + std::to_string(expected) + " fields");
}
} // namespace

void read_data::readFarham(const std::string& path) {
    if (pathDepotCon != "NULL")
        throw std::runtime_error("Set F generates depot configurations internally; pass NULL as depot configuration path");
    // Same declared-prefix semantics as HGA-SP's FarhamLrptwParser.
    std::ifstream input(path);
    if (!input) invalid(path, "cannot open file");
    std::vector<std::vector<double>> rows;
    std::string line;
    std::size_t lineNumber = 0;
    while (std::getline(input, line)) {
        ++lineNumber;
        const auto first = line.find_first_not_of(" \t\r\n");
        if (first == std::string::npos || line[first] == '#') continue;
        std::istringstream stream(line);
        std::vector<double> row;
        double value;
        while (stream >> value) {
            if (!std::isfinite(value)) invalid(path, "non-finite number on line " + std::to_string(lineNumber));
            row.push_back(value);
        }
        if (!stream.eof()) invalid(path, "non-numeric token on line " + std::to_string(lineNumber));
        if (!row.empty()) rows.push_back(std::move(row));
    }
    if (input.bad()) invalid(path, "read failure");
    if (rows.size() < 2) invalid(path, "missing SIZE or VEHICLE header");
    width(rows[0], 3, path, "SIZE");
    width(rows[1], 3, path, "VEHICLE");
    if (integer(rows[0][0], path, "NUM_CDC") != 0)
        invalid(path, "NUM_CDC must be zero for single-echelon LRPTW");
    const int depots = integer(rows[0][1], path, "NUM_SATELLITE");
    const int customers = integer(rows[0][2], path, "NUM_CUSTOMER");
    if (depots <= 0 || customers <= 0 ||
        customers > std::numeric_limits<int>::max() - depots ||
        customers > std::numeric_limits<int>::max() / depots)
        invalid(path, "invalid or oversized depot/customer counts");
    if (rows.size() < std::size_t(2) + depots + customers)
        invalid(path, "fewer records than declared in SIZE");
    if (integer(rows[1][0], path, "vehicle header identifier") < 0)
        invalid(path, "negative vehicle header identifier");
    const int capacity = integer(rows[1][2], path, "vehicle capacity");
    if (capacity <= 0 || rows[1][1] < 0) invalid(path, "invalid vehicle capacity or cost");

    std::unordered_set<int> depotIds, customerIds;
    long long demandSum = 0;
    long long depotCapacitySum = 0;
    for (int depot = 0; depot < depots; ++depot) {
        const auto& row = rows[2 + depot];
        width(row, 6, path, "SATELLITE");
        if (!depotIds.insert(integer(row[0], path, "satellite id")).second)
            invalid(path, "duplicate satellite id");
        if (row[3] < 0 || row[4] < 0 || integer(row[5], path, "satellite capacity") <= 0)
            invalid(path, "invalid satellite close time, cost or capacity");
        depotCapacitySum += static_cast<int>(row[5]);
    }
    // Published reduced instances may include unused suffix customers.
    for (int customer = 0; customer < customers; ++customer) {
        const auto& row = rows[std::size_t(2) + depots + customer];
        width(row, 7, path, "CUSTOMER");
        if (!customerIds.insert(integer(row[0], path, "customer id")).second)
            invalid(path, "duplicate customer id");
        const int demand = integer(row[3], path, "customer demand");
        if (demand < 0 || demand > capacity || row[4] < 0 || row[5] < row[4] || row[6] < 0)
            invalid(path, "inconsistent customer demand or time window");
        demandSum += demand;
        if (demandSum > std::numeric_limits<int>::max()) invalid(path, "total demand exceeds integer range");
    }
    if (depotCapacitySum < demandSum) invalid(path, "total depot capacity is below customer demand");
    // The existing solver stores finite arc costs in its double distance matrix.
    for (int a = 0; a < customers + depots; ++a)
        for (int b = 0; b < a; ++b) {
            const auto& ra = rows[std::size_t(2) + a];
            const auto& rb = rows[std::size_t(2) + b];
            const double dx = ra[1] - rb[1], dy = ra[2] - rb[2];
            if (!std::isfinite(dx * dx + dy * dy)) invalid(path, "coordinate distance exceeds numeric range");
        }

    hardTimeWindows = true;
    numV = customers;
    numL = depots;
    numC = numV + numL;
    vehCap = capacity;
    usingCostVeh = rows[1][1];
    realOrInteger = 1; // The common distance pass floors arcs for this profile.
    define();
    numLocConfiguration = maxActiveConfigurations;
    readyTime.resize(numV);
    dueTime.resize(numV);
    serviceTime.resize(numV);
    originalCustomerIds.resize(numV);
    depotCloseTime.resize(numL);
    originalDepotIds.resize(numL);
    for (int depot = 0; depot < numL; ++depot) {
        const auto& row = rows[2 + depot];
        originalDepotIds[depot] = static_cast<int>(row[0]);
        x[numV + depot] = row[1]; y[numV + depot] = row[2];
        depotCloseTime[depot] = row[3];
        openCostLoc[numV + depot] = row[4];
        invLoc[numV + depot] = static_cast<int>(row[5]);
    }
    for (int customer = 0; customer < numV; ++customer) {
        const auto& row = rows[std::size_t(2) + numL + customer];
        originalCustomerIds[customer] = static_cast<int>(row[0]);
        x[customer] = row[1]; y[customer] = row[2];
        demCus[customer] = static_cast<int>(row[3]);
        readyTime[customer] = row[4]; dueTime[customer] = row[5]; serviceTime[customer] = row[6];
    }
}

bool read_data::routeTimeFeasible(int depotIndex, const std::vector<int>& sequence) const {
    if (!hardTimeWindows) return true;
    if (depotIndex < 0 || depotIndex >= numL) return false;
    double time = 0.0;
    int previous = numV + depotIndex;
    for (int customer : sequence) {
        if (customer < 0 || customer >= numV) return false;
        time = std::max(time + D[previous][customer], readyTime[customer]);
        if (!std::isfinite(time) || time > dueTime[customer] + 1e-9) return false;
        time += serviceTime[customer];
        previous = customer;
    }
    time += D[previous][numV + depotIndex];
    return std::isfinite(time) && time <= depotCloseTime[depotIndex] + 1e-9;
}
