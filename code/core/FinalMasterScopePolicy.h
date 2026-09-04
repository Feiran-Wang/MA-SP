#ifndef HGASP_CORE_FINAL_MASTER_SCOPE_POLICY_H_
#define HGASP_CORE_FINAL_MASTER_SCOPE_POLICY_H_

namespace hgasp {

enum class FinalMasterScope {
    FullActiveMaster,
    Kernel,
    ExploratoryActiveView,
};

inline FinalMasterScope chooseFinalMasterScope(
        const bool hardMandatoryFinal,
        const bool onlineCall,
        const bool kernelAvailable) noexcept {
    if (hardMandatoryFinal) {
        return FinalMasterScope::FullActiveMaster;
    }
    return onlineCall && kernelAvailable
        ? FinalMasterScope::Kernel
        : FinalMasterScope::FullActiveMaster;
}

inline const char* toString(
        const FinalMasterScope scope) noexcept {
    switch (scope) {
    case FinalMasterScope::FullActiveMaster:
        return "full_active_master";
    case FinalMasterScope::Kernel:
        return "kernel";
    case FinalMasterScope::ExploratoryActiveView:
        return "exploratory_active_view";
    }
    return "unknown";
}

}  // namespace hgasp

#endif  // HGASP_CORE_FINAL_MASTER_SCOPE_POLICY_H_
