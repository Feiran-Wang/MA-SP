#ifndef HGASP_PROBLEM_CONSTRAINTPROFILE_H_
#define HGASP_PROBLEM_CONSTRAINTPROFILE_H_

namespace hgasp {

enum class TimeWindowPolicy {
    None,
    Hard,
};

struct ConstraintProfile {
    TimeWindowPolicy timeWindows = TimeWindowPolicy::None;

    bool hasHardTimeWindows() const noexcept {
        return timeWindows == TimeWindowPolicy::Hard;
    }
};

const char* toString(TimeWindowPolicy policy) noexcept;

}  // namespace hgasp

#endif  // HGASP_PROBLEM_CONSTRAINTPROFILE_H_
