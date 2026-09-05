#include "InstanceLoader.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>

namespace hgasp {
namespace {

std::string lowercase(std::string value) {
    std::transform(
        value.begin(),
        value.end(),
        value.begin(),
        [](const unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });
    return value;
}

std::string extensionOf(const std::string& path) {
    const std::size_t slash = path.find_last_of("/\\");
    const std::size_t dot = path.find_last_of('.');
    if (dot == std::string::npos ||
        (slash != std::string::npos && dot < slash)) {
        return {};
    }
    return lowercase(path.substr(dot + 1));
}

}  // namespace

ParsedInstance loadInstance(const std::string& path) {
    const std::string extension = extensionOf(path);
    ParsedInstance parsed;
    if (extension == "dat") {
        parsed = parseSetSDat(path);
    } else if (extension == "json") {
        parsed = parseSetSJson(path);
    } else if (extension == "txt") {
        parsed = parseFarhamLrptw(path);
    } else {
        throw std::runtime_error(
            "unsupported instance format: " +
            (extension.empty() ? std::string("<none>") : extension));
    }
    validateParsedInstance(parsed, path);
    return parsed;
}

void validateParsedInstance(
    const ParsedInstance& instance,
    const std::string& path) {
    validateProblemProfile(instance.profile);
    if (instance.customers.empty() || instance.depots.empty()) {
        throw std::invalid_argument(
            "instance '" + path + "' requires customers and depots");
    }
    if (instance.vehicleCapacity <= 0) {
        throw std::invalid_argument(
            "instance '" + path + "' requires positive vehicle capacity");
    }
    if (instance.profile.constraints.hasHardTimeWindows()) {
        for (const ParsedCustomer& customer : instance.customers) {
            if (!std::isfinite(customer.readyTime) ||
                !std::isfinite(customer.dueTime) ||
                !std::isfinite(customer.serviceTime) ||
                customer.readyTime > customer.dueTime ||
                customer.serviceTime < 0.0) {
                throw std::invalid_argument(
                    "hard-time-window instance '" + path +
                    "' has invalid customer time data");
            }
        }
        for (const ParsedDepot& depot : instance.depots) {
            if (!std::isfinite(depot.openTime) ||
                !std::isfinite(depot.closeTime) ||
                depot.openTime > depot.closeTime) {
                throw std::invalid_argument(
                    "hard-time-window instance '" + path +
                    "' has invalid depot time data");
            }
        }
    }
}

}  // namespace hgasp
