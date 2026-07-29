#include "InstanceLoader.h"

#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>

namespace hgasp {
namespace {

template <typename Value>
void readRequired(
    std::istream& input,
    Value& value,
    const std::string& field,
    const std::string& path) {
    if (!(input >> value)) {
        throw std::runtime_error(
            "invalid Set S DAT instance '" + path +
            "': missing or invalid " + field);
    }
}

void validateParsed(const ParsedInstance& parsed, const std::string& path) {
    if (parsed.customers.empty() || parsed.depots.empty()) {
        throw std::runtime_error(
            "invalid Set S DAT instance '" + path +
            "': customer and depot counts must be positive");
    }
    if (parsed.vehicleCapacity <= 0) {
        throw std::runtime_error(
            "invalid Set S DAT instance '" + path +
            "': vehicle capacity must be positive");
    }
    for (const ParsedCustomer& customer : parsed.customers) {
        if (customer.demand < 0) {
            throw std::runtime_error(
                "invalid Set S DAT instance '" + path +
                "': customer demand must be non-negative");
        }
    }
    for (const ParsedDepot& depot : parsed.depots) {
        if (depot.capacity < 0) {
            throw std::runtime_error(
                "invalid Set S DAT instance '" + path +
                "': depot capacity must be non-negative");
        }
    }
}

}  // namespace

ParsedInstance parseSetSDat(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to open instance file: " + path);
    }

    int customerCount = 0;
    int depotCount = 0;
    readRequired(input, customerCount, "customer count", path);
    readRequired(input, depotCount, "depot count", path);
    if (customerCount <= 0 || depotCount <= 0) {
        throw std::runtime_error(
            "invalid Set S DAT instance '" + path +
            "': customer and depot counts must be positive");
    }

    ParsedInstance parsed;
    parsed.requiredMode = ProblemMode::Clrp;
    parsed.format = InstanceFormat::LegacyDat;
    parsed.customers.resize(static_cast<std::size_t>(customerCount));
    parsed.depots.resize(static_cast<std::size_t>(depotCount));

    // The legacy DAT layout stores depot coordinates before customer
    // coordinates even though the in-memory representation stores customers
    // first.
    for (int depot = 0; depot < depotCount; ++depot) {
        ParsedDepot& target = parsed.depots[static_cast<std::size_t>(depot)];
        target.originalId = depot;
        readRequired(input, target.x, "depot x-coordinate", path);
        readRequired(input, target.y, "depot y-coordinate", path);
    }
    for (int customer = 0; customer < customerCount; ++customer) {
        ParsedCustomer& target =
            parsed.customers[static_cast<std::size_t>(customer)];
        target.originalId = customer;
        readRequired(input, target.x, "customer x-coordinate", path);
        readRequired(input, target.y, "customer y-coordinate", path);
        target.dueTime = std::numeric_limits<double>::infinity();
    }

    readRequired(
        input, parsed.vehicleCapacity, "vehicle capacity", path);
    for (ParsedDepot& depot : parsed.depots) {
        readRequired(input, depot.capacity, "depot capacity", path);
        depot.closeTime = std::numeric_limits<double>::infinity();
    }
    for (ParsedCustomer& customer : parsed.customers) {
        readRequired(input, customer.demand, "customer demand", path);
    }
    for (ParsedDepot& depot : parsed.depots) {
        readRequired(input, depot.openingCost, "depot opening cost", path);
    }
    readRequired(
        input, parsed.vehicleFixedCost, "vehicle fixed cost", path);
    readRequired(
        input,
        parsed.legacyRealOrInteger,
        "legacy real-or-integer flag",
        path);

    parsed.matrixPolicy.travelCostRounding =
        parsed.legacyRealOrInteger == 0
            ? NumericRounding::CeilAfterScalingBy100
            : NumericRounding::Exact;
    parsed.matrixPolicy.travelTimeRounding = NumericRounding::Exact;
    validateParsed(parsed, path);
    return parsed;
}

}  // namespace hgasp
