#ifndef HGASP_OPERATORS_EAX_PARENT_MODE_H_
#define HGASP_OPERATORS_EAX_PARENT_MODE_H_

#include <stdexcept>
#include <string>

namespace hgasp {

enum class EaxParentMode {
    PaperCompatible,
    Corrected,
};

inline const char* toString(const EaxParentMode mode) noexcept {
    switch (mode) {
        case EaxParentMode::PaperCompatible:
            return "paper-compatible";
        case EaxParentMode::Corrected:
            return "corrected";
    }
    return "unknown";
}

inline EaxParentMode parseEaxParentMode(const std::string& value) {
    if (value == "paper-compatible") {
        return EaxParentMode::PaperCompatible;
    }
    if (value == "corrected") {
        return EaxParentMode::Corrected;
    }
    throw std::invalid_argument(
        "EAX parent mode must be paper-compatible or corrected");
}

}  // namespace hgasp

#endif  // HGASP_OPERATORS_EAX_PARENT_MODE_H_
