#ifndef HGASP_IO_PARSEDINSTANCE_H_
#define HGASP_IO_PARSEDINSTANCE_H_

#include "ProblemMode.h"

#include <string>
#include <vector>

namespace hgasp {

enum class InstanceFormat {
    LegacyDat,
    SetSJson,
    FarhamText,
};

const char* toString(InstanceFormat format) noexcept;

enum class NumericRounding {
    Exact,
    Floor,
    CeilAfterScalingBy100,
};

struct MatrixPolicy {
    NumericRounding travelCostRounding = NumericRounding::Exact;
    NumericRounding travelTimeRounding = NumericRounding::Exact;
    bool zeroDepotToDepotEntries = true;
};

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
    ProblemMode requiredMode = ProblemMode::Clrp;
    InstanceFormat format = InstanceFormat::LegacyDat;
    MatrixPolicy matrixPolicy;

    std::vector<ParsedCustomer> customers;
    std::vector<ParsedDepot> depots;

    int vehicleCapacity = 0;
    double vehicleFixedCost = 0.0;
    int legacyRealOrInteger = 0;
};

}  // namespace hgasp

#endif  // HGASP_IO_PARSEDINSTANCE_H_
