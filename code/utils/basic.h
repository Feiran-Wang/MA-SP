/******************************************************************************
 * File: basic.h
 * Author: Wang
 * Date: 2026-06-24
 * Description: 
 *   Common definitions, constants, macros, and inline utilities.
 ******************************************************************************/

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
///////////////////////////////////////////////////////////////////////////////
#define numSubPopPara 15
#define Max 100000000
#define Min 1.0e-7
#define usingDepotConfiguration 2

extern std::mt19937 global_rng;

// ==========================================================
// 随机数工具函数 (C++11 风格)
// ==========================================================

// 1. 生成 [min, max] 范围内的随机整数 (包含 min 和 max)
inline int RandomInt(int min, int max) {
    std::uniform_int_distribution<int> dist(min, max);
    return dist(global_rng);
}

// 2. 生成 [min, max) 范围内的随机浮点数 (包含 min，不包含 max)
inline double RandomDouble(double min, double max) {
    std::uniform_real_distribution<double> dist(min, max);
    return dist(global_rng);
}

// 3. 随机打乱容器 (替代被废弃的 random_shuffle)
template<typename Container>
inline void RandomShuffle(Container& c) {
    std::shuffle(c.begin(), c.end(), global_rng);
}

#endif /* BASIC_H_ */