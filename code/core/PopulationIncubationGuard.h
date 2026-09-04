#ifndef HGASP_CORE_POPULATION_INCUBATION_GUARD_H_
#define HGASP_CORE_POPULATION_INCUBATION_GUARD_H_

#include <algorithm>
#include <cstddef>
#include <stdexcept>

namespace hgasp {

class PopulationIncubationGuard {
public:
    PopulationIncubationGuard(
            const std::size_t targetSize,
            const std::size_t initialSize,
            const std::size_t maximumAttempts,
            const std::size_t maximumNoProgress)
        : targetSize_(targetSize),
          lastSize_(initialSize),
          maximumAttempts_(maximumAttempts),
          maximumNoProgress_(maximumNoProgress) {
        if (targetSize_ == 0U ||
            maximumAttempts_ == 0U ||
            maximumNoProgress_ == 0U) {
            throw std::invalid_argument(
                "population incubation bounds must be positive");
        }
    }

    bool beginAttempt(
            const std::size_t currentSize,
            const bool stopRequested) {
        lastSize_ = std::max(lastSize_, currentSize);
        if (currentSize >= targetSize_ ||
            stopRequested ||
            attempts_ >= maximumAttempts_ ||
            noProgressAttempts_ >= maximumNoProgress_) {
            stoppedByExternalRequest_ =
                stopRequested && currentSize < targetSize_;
            return false;
        }
        ++attempts_;
        return true;
    }

    void observe(const std::size_t currentSize) noexcept {
        if (currentSize > lastSize_) {
            lastSize_ = currentSize;
            noProgressAttempts_ = 0U;
        } else {
            ++noProgressAttempts_;
        }
    }

    std::size_t attempts() const noexcept {
        return attempts_;
    }
    std::size_t noProgressAttempts() const noexcept {
        return noProgressAttempts_;
    }
    bool stoppedByExternalRequest() const noexcept {
        return stoppedByExternalRequest_;
    }

private:
    std::size_t targetSize_;
    std::size_t lastSize_;
    std::size_t maximumAttempts_;
    std::size_t maximumNoProgress_;
    std::size_t attempts_ = 0U;
    std::size_t noProgressAttempts_ = 0U;
    bool stoppedByExternalRequest_ = false;
};

}  // namespace hgasp

#endif  // HGASP_CORE_POPULATION_INCUBATION_GUARD_H_
