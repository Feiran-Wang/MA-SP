#include "ProblemMode.h"

#include <stdexcept>

namespace hgasp {

ProblemMode parseProblemMode(const std::string& value) {
    if (value == "clrp") {
        return ProblemMode::Clrp;
    }
    if (value == "clrp-tw") {
        return ProblemMode::ClrpTw;
    }
    throw std::invalid_argument(
        "unsupported problem mode '" + value +
        "' (expected 'clrp' or 'clrp-tw')");
}

const char* toString(const ProblemMode mode) noexcept {
    switch (mode) {
        case ProblemMode::Clrp:
            return "clrp";
        case ProblemMode::ClrpTw:
            return "clrp-tw";
    }
    return "unknown";
}

}  // namespace hgasp
