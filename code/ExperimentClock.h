#ifndef MASP_EXPERIMENT_CLOCK_H
#define MASP_EXPERIMENT_CLOCK_H
#include <cstdlib>
#include <time.h>
inline double experimentMonotonic() {
    timespec value;
    clock_gettime(CLOCK_MONOTONIC, &value);
    return value.tv_sec + value.tv_nsec / 1e9;
}
inline double experimentRemaining() {
    const char* end = std::getenv("MASP_RUN_DEADLINE");
    return end ? std::strtod(end, nullptr) - experimentMonotonic() : 1e30;
}
#endif
