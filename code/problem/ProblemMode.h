#ifndef HGASP_PROBLEM_PROBLEMMODE_H_
#define HGASP_PROBLEM_PROBLEMMODE_H_

#include <string>

namespace hgasp {

enum class ProblemMode {
    Clrp,
    ClrpTw,
};

ProblemMode parseProblemMode(const std::string& value);
const char* toString(ProblemMode mode) noexcept;

}  // namespace hgasp

#endif  // HGASP_PROBLEM_PROBLEMMODE_H_
