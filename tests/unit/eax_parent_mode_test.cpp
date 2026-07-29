#include "EaxParentMode.h"
#include "crossover.h"
#include "read_data.h"

#include <iostream>
#include <random>
#include <stdexcept>
#include <string>

std::mt19937 global_rng(1705U);

namespace {

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void testParentSelectionModes() {
    using hgasp::EaxParentMode;

    require(
        crossover::selectInitialEdgeSource(
            true, false, EaxParentMode::Corrected, 0),
        "father-only edge was not selected");
    require(
        !crossover::selectInitialEdgeSource(
            false, true, EaxParentMode::PaperCompatible, 0),
        "mother-only edge was not selected");

    require(
        crossover::selectInitialEdgeSource(
            true, true, EaxParentMode::PaperCompatible, 0),
        "paper-compatible mode changed random branch 0");
    require(
        crossover::selectInitialEdgeSource(
            true, true, EaxParentMode::PaperCompatible, 1),
        "paper-compatible mode changed random branch 1");

    require(
        crossover::selectInitialEdgeSource(
            true, true, EaxParentMode::Corrected, 0),
        "corrected mode did not select father for random branch 0");
    require(
        !crossover::selectInitialEdgeSource(
            true, true, EaxParentMode::Corrected, 1),
        "corrected mode did not select mother for random branch 1");
}

void testModeParsingAndDefault() {
    read_data data;
    require(
        data.eaxParentMode ==
            hgasp::EaxParentMode::PaperCompatible,
        "default EAX mode is not paper-compatible");
    require(
        hgasp::parseEaxParentMode("corrected") ==
            hgasp::EaxParentMode::Corrected,
        "corrected EAX mode did not parse");
    require(
        std::string(hgasp::toString(
            hgasp::EaxParentMode::PaperCompatible)) ==
            "paper-compatible",
        "paper-compatible EAX mode did not serialize");
}

}  // namespace

int main() {
    try {
        testParentSelectionModes();
        testModeParsingAndDefault();
        std::cout << "eax_parent_mode_test: PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "eax_parent_mode_test: "
                  << error.what() << '\n';
        return 1;
    }
}
