#ifndef HGASP_IO_PARSEDINSTANCE_H_
#define HGASP_IO_PARSEDINSTANCE_H_

#include "ProblemProfile.h"

#include <string>
#include <vector>

namespace hgasp {

struct ParsedCustomer {
    int originalId = -1;
    double x = 0.0;
    double y = 0.0;
    int demand = 0;
    double readyTime = 0.0;
    double dueTime = 0.0;
    double serviceTime = 0.0;
};

struct ParsedDepot {
    int originalId = -1;
    double x = 0.0;
    double y = 0.0;
    int capacity = 0;
    double openingCost = 0.0;
    double openTime = 0.0;
    double closeTime = 0.0;
};

struct ParsedInstance {
    std::string name;
    ProblemProfile profile;

    std::vector<ParsedCustomer> customers;
    std::vector<ParsedDepot> depots;

    int vehicleCapacity = 0;
    double vehicleFixedCost = 0.0;
    int legacyRealOrInteger = 0;
};

}  // namespace hgasp

#endif  // HGASP_IO_PARSEDINSTANCE_H_
