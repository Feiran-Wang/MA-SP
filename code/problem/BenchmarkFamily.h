#ifndef HGASP_PROBLEM_BENCHMARK_FAMILY_H_
#define HGASP_PROBLEM_BENCHMARK_FAMILY_H_

#include <stdexcept>
#include <string>
#include <string_view>

namespace hgasp {

// Dataset identity is intentionally independent from constraint capabilities.
// Set F currently has hard time windows and Set S does not, but neither fact is
// a safe substitute for the benchmark family's metric and reporting contract.
enum class BenchmarkFamily {
    SetS,
    SetF,
};

inline constexpr const char* toString(
        const BenchmarkFamily family) noexcept {
    switch (family) {
        case BenchmarkFamily::SetS:
            return "set-s";
        case BenchmarkFamily::SetF:
            return "set-f";
    }
    return "unknown";
}

inline BenchmarkFamily parseBenchmarkFamily(const std::string_view value) {
    if (value == "set-s") {
        return BenchmarkFamily::SetS;
    }
    if (value == "set-f") {
        return BenchmarkFamily::SetF;
    }
    throw std::invalid_argument(
        "unsupported benchmark family '" + std::string(value) + "'");
}

}  // namespace hgasp

#endif  // HGASP_PROBLEM_BENCHMARK_FAMILY_H_
