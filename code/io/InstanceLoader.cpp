#include "InstanceLoader.h"

#include <algorithm>
#include <cctype>
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

const char* toString(const InstanceFormat format) noexcept {
    switch (format) {
        case InstanceFormat::LegacyDat:
            return "legacy-dat";
        case InstanceFormat::SetSJson:
            return "set-s-json";
        case InstanceFormat::FarhamText:
            return "farham-text";
    }
    return "unknown";
}

ParsedInstance loadInstance(const std::string& path) {
    const std::string extension = extensionOf(path);
    if (extension == "dat") {
        return parseSetSDat(path);
    }
    if (extension == "json") {
        return parseSetSJson(path);
    }
    if (extension == "txt") {
        return parseFarhamLrptw(path);
    }
    throw std::runtime_error(
        "unsupported instance format: " +
        (extension.empty() ? std::string("<none>") : extension));
}

void validateModeCompatibility(
    const ParsedInstance& instance,
    const ProblemMode requestedMode,
    const std::string& path) {
    if (instance.requiredMode == requestedMode) {
        return;
    }
    throw std::invalid_argument(
        "problem mode '" + std::string(toString(requestedMode)) +
        "' is incompatible with " +
        std::string(toString(instance.format)) + " instance '" + path +
        "'; expected '" + std::string(toString(instance.requiredMode)) +
        "'");
}

}  // namespace hgasp
