// Author: Feiran
// Provide shared constants and access to the search random-number engine.

#ifndef BASIC_H_
#define BASIC_H_
#include <algorithm>
#include <vector>
#include <set>
#include <unordered_set>
#include <list>
#include <math.h>
#include <iostream>
#include <time.h>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <random>

#define Max 100000000
#define Min 1.0e-7
#define usingDepotConfiguration 2

extern std::mt19937 global_rng;

inline int RandomInt(int minimum, int maximum) {
    std::uniform_int_distribution<int> distribution(minimum, maximum);
    return distribution(global_rng);
}

inline double RandomDouble(double minimum, double maximum) {
    std::uniform_real_distribution<double> distribution(minimum, maximum);
    return distribution(global_rng);
}

template <typename Container> inline void RandomShuffle(Container &container) {
    std::shuffle(container.begin(), container.end(), global_rng);
}

#endif
