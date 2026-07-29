#ifndef HGASP_IO_INSTANCELOADER_H_
#define HGASP_IO_INSTANCELOADER_H_

#include "ParsedInstance.h"

#include <string>

namespace hgasp {

ParsedInstance loadInstance(const std::string& path);
void validateModeCompatibility(
    const ParsedInstance& instance,
    ProblemMode requestedMode,
    const std::string& path);
ParsedInstance parseSetSDat(const std::string& path);
ParsedInstance parseSetSJson(const std::string& path);
ParsedInstance parseFarhamLrptw(const std::string& path);

}  // namespace hgasp

#endif  // HGASP_IO_INSTANCELOADER_H_
