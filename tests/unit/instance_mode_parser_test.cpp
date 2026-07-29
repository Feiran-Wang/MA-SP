#include "InstanceLoader.h"
#include "ProblemMode.h"
#include "commandline.h"
#include "read_data.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

std::mt19937 global_rng;

namespace {

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

template <typename Callable>
void requireInvalidArgument(
    Callable&& callable,
    const std::string& message) {
    bool rejected = false;
    try {
        callable();
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    require(rejected, message);
}

template <typename Callable>
void requireRuntimeError(
    Callable&& callable,
    const std::string& message) {
    bool rejected = false;
    try {
        callable();
    } catch (const std::runtime_error&) {
        rejected = true;
    }
    require(rejected, message);
}

CommandLine makeCommandLine(std::vector<std::string> arguments) {
    std::vector<char*> raw;
    raw.reserve(arguments.size());
    for (std::string& argument : arguments) {
        raw.push_back(argument.data());
    }
    return CommandLine(static_cast<int>(raw.size()), raw.data());
}

void testProblemModeAndCommandLine() {
    require(
        hgasp::parseProblemMode("clrp") == hgasp::ProblemMode::Clrp,
        "clrp mode was not parsed");
    require(
        hgasp::parseProblemMode("clrp-tw") ==
            hgasp::ProblemMode::ClrpTw,
        "clrp-tw mode was not parsed");
    requireInvalidArgument(
        [] { (void)hgasp::parseProblemMode("auto"); },
        "unsupported problem mode was accepted");

    const CommandLine legacy = makeCommandLine(
        {"hgasp", "instance.json", "solution.txt", "0", "0",
         "1", "2", "3"});
    require(
        legacy.problemMode == hgasp::ProblemMode::Clrp,
        "legacy CLI did not default to CLRP");
    require(
        !legacy.eaxParentMode.has_value(),
        "legacy CLI unexpectedly overrode the EAX mode");

    const CommandLine timeWindow = makeCommandLine(
        {"hgasp", "--problem-mode", "clrp-tw", "instance.txt",
         "solution.txt", "0", "0", "1", "2", "3"});
    require(
        timeWindow.problemMode == hgasp::ProblemMode::ClrpTw,
        "explicit CLRP-TW mode was not retained");
    require(
        timeWindow.pathDepotCon == "NULL",
        "mode option changed positional argument interpretation");

    const CommandLine correctedEax = makeCommandLine(
        {"hgasp", "instance.txt", "solution.txt", "0", "0",
         "1", "2", "3", "--eax-mode=corrected"});
    require(
        correctedEax.eaxParentMode ==
            hgasp::EaxParentMode::Corrected,
        "explicit corrected EAX mode was not retained");

    const CommandLine withDepotConfiguration = makeCommandLine(
        {"hgasp", "instance.json", "depots.txt", "solution.txt",
         "0", "0", "1", "2", "3", "--problem-mode=clrp"});
    require(
        withDepotConfiguration.pathDepotCon == "depots.txt",
        "depot configuration positional argument was lost");
}

void testSetSJson(const std::filesystem::path& root) {
    const hgasp::ParsedInstance instance = hgasp::loadInstance(
        (root / "instances/set_S/100-5-1e.json").string());
    require(
        instance.requiredMode == hgasp::ProblemMode::Clrp,
        "Set S JSON required the wrong mode");
    require(
        instance.format == hgasp::InstanceFormat::SetSJson,
        "Set S JSON format was not retained");
    require(
        instance.customers.size() == 100 &&
            instance.depots.size() == 5,
        "Set S JSON dimensions changed");
    require(
        instance.customers.front().demand == 18 &&
            instance.customers.front().x == 31.0 &&
            instance.customers.front().y == 6.0,
        "Set S first customer changed");
    require(
        instance.depots.front().capacity == 1583 &&
            instance.depots.front().openingCost == 83376.0,
        "Set S first depot changed");
    require(
        instance.vehicleCapacity == 70 &&
            instance.vehicleFixedCost == 1000.0,
        "Set S vehicle data changed");
    require(
        instance.matrixPolicy.travelCostRounding ==
            hgasp::NumericRounding::CeilAfterScalingBy100,
        "Set S legacy objective rounding changed");
}

void testSetSDat() {
    const auto parseWithFlag = [](const int realOrInteger) {
        const std::filesystem::path path =
            std::filesystem::temp_directory_path() /
            ("hgasp_legacy_set_s_instance_" +
             std::to_string(realOrInteger) + ".dat");
        {
            std::ofstream output(path);
            output
                << "2 2\n"
                << "10 20\n"
                << "30 40\n"
                << "1 2\n"
                << "3 4\n"
                << "7\n"
                << "8 9\n"
                << "5 6\n"
                << "100 200\n"
                << "300\n"
                << realOrInteger << "\n";
        }
        const hgasp::ParsedInstance instance =
            hgasp::loadInstance(path.string());
        std::filesystem::remove(path);
        return instance;
    };

    const hgasp::ParsedInstance scaled = parseWithFlag(0);
    require(
        scaled.requiredMode == hgasp::ProblemMode::Clrp &&
            scaled.format == hgasp::InstanceFormat::LegacyDat,
        "legacy DAT mode or format changed");
    require(
        scaled.customers.size() == 2 &&
            scaled.depots.size() == 2,
        "legacy DAT dimensions changed");
    require(
        scaled.depots[0].x == 10.0 &&
            scaled.depots[1].y == 40.0 &&
            scaled.customers[0].x == 1.0 &&
            scaled.customers[1].y == 4.0,
        "legacy DAT coordinate ordering changed");
    require(
        scaled.vehicleCapacity == 7 &&
            scaled.depots[0].capacity == 8 &&
            scaled.customers[1].demand == 6 &&
            scaled.vehicleFixedCost == 300.0,
        "legacy DAT capacities/costs changed");
    require(
        scaled.legacyRealOrInteger == 0 &&
            scaled.matrixPolicy.travelCostRounding ==
                hgasp::NumericRounding::CeilAfterScalingBy100 &&
            scaled.matrixPolicy.travelTimeRounding ==
                hgasp::NumericRounding::Exact,
        "legacy DAT flag 0 no longer selects ceil(distance * 100) costs");

    const hgasp::ParsedInstance exact = parseWithFlag(7);
    require(
        exact.legacyRealOrInteger == 7 &&
            exact.matrixPolicy.travelCostRounding ==
                hgasp::NumericRounding::Exact &&
            exact.matrixPolicy.travelTimeRounding ==
                hgasp::NumericRounding::Exact,
        "legacy DAT nonzero flag no longer selects exact travel costs");
}

void writeSetSDat(
    const std::filesystem::path& path,
    const hgasp::ParsedInstance& source,
    const int realOrInteger) {
    std::ofstream output(path);
    require(
        output.is_open(),
        "could not create temporary Set S DAT rounding fixture");

    output << source.customers.size() << ' ' << source.depots.size() << '\n';
    for (const hgasp::ParsedDepot& depot : source.depots) {
        output << depot.x << ' ' << depot.y << '\n';
    }
    for (const hgasp::ParsedCustomer& customer : source.customers) {
        output << customer.x << ' ' << customer.y << '\n';
    }
    output << source.vehicleCapacity << '\n';
    for (const hgasp::ParsedDepot& depot : source.depots) {
        output << depot.capacity << ' ';
    }
    output << '\n';
    for (const hgasp::ParsedCustomer& customer : source.customers) {
        output << customer.demand << ' ';
    }
    output << '\n';
    for (const hgasp::ParsedDepot& depot : source.depots) {
        output << depot.openingCost << ' ';
    }
    output << '\n'
           << source.vehicleFixedCost << '\n'
           << realOrInteger << '\n';
}

void testSetSDatReadDataRounding(const std::filesystem::path& root) {
    const std::filesystem::path jsonPath =
        root / "instances/set_S/100-5-1e.json";
    const std::filesystem::path depotConfigurationPath =
        root /
        "instances/depot configurations/S/"
        "depot_configuratons_100-5-1e.txt";
    const hgasp::ParsedInstance source =
        hgasp::loadInstance(jsonPath.string());
    const double exactDistance = std::sqrt(221.0);

    for (const int realOrInteger : {0, 7}) {
        const std::filesystem::path datPath =
            std::filesystem::temp_directory_path() /
            ("hgasp_set_s_rounding_" +
             std::to_string(realOrInteger) + ".dat");
        writeSetSDat(datPath, source, realOrInteger);

        global_rng.seed(0);
        read_data data;
        data.read_fun(
            datPath.string(),
            depotConfigurationPath.string(),
            (root / "set_s_dat_rounding_test_solution.txt").string(),
            0.0,
            0,
            0,
            1,
            hgasp::ProblemMode::Clrp);
        std::filesystem::remove(datPath);

        const double expectedCost =
            realOrInteger == 0
                ? std::ceil(exactDistance * 100.0)
                : exactDistance;
        require(
            std::fabs(data.D[0][1] - expectedCost) <= 1.0e-9,
            realOrInteger == 0
                ? "Set S DAT flag 0 did not build ceil(distance * 100) arcs"
                : "Set S DAT nonzero flag did not build exact-cost arcs");
        require(
            std::fabs(data.T[0][1] - exactDistance) <= 1.0e-9,
            "Set S DAT travel-time arc no longer retains exact distance");
    }
}

void testSetSJsonReadDataRounding(const std::filesystem::path& root) {
    const std::filesystem::path instancePath =
        root / "instances/set_S/100-5-1e.json";
    const std::filesystem::path depotConfigurationPath =
        root /
        "instances/depot configurations/S/"
        "depot_configuratons_100-5-1e.txt";

    global_rng.seed(0);
    read_data data;
    data.read_fun(
        instancePath.string(),
        depotConfigurationPath.string(),
        (root / "set_s_rounding_test_solution.txt").string(),
        0.0,
        0,
        0,
        1,
        hgasp::ProblemMode::Clrp);

    // Customer 5 is at (31,6) and customer 6 is at (36,20).
    // sqrt(221) is non-integral, so this one arc detects both an omitted
    // factor of 100 and any floor/nearest rounding regression.
    const double exactDistance = std::sqrt(221.0);
    require(
        std::fabs(data.D[0][1] - std::ceil(exactDistance * 100.0)) <=
            1.0e-9,
        "Set S JSON objective arc no longer uses ceil(distance * 100)");
    require(
        std::fabs(data.T[0][1] - exactDistance) <= 1.0e-9,
        "Set S JSON travel-time arc no longer retains exact distance");
}

void testFarhamPrefixAndCoordinates(const std::filesystem::path& root) {
    const hgasp::ParsedInstance randomInstance =
        hgasp::loadInstance(
            (root / "instances/set_F/R101_s1_3_10.txt").string());
    require(
        randomInstance.requiredMode == hgasp::ProblemMode::ClrpTw,
        "Set F required the wrong mode");
    require(
        randomInstance.format == hgasp::InstanceFormat::FarhamText,
        "Set F format was not retained");
    require(
        randomInstance.customers.size() == 10 &&
            randomInstance.depots.size() == 3,
        "Set F parser did not honor declared active dimensions");
    require(
        randomInstance.customers.back().originalId == 10,
        "Set F parser included physical rows after the declared prefix");
    require(
        randomInstance.vehicleCapacity == 200 &&
            randomInstance.vehicleFixedCost == 225.0,
        "Set F vehicle type id was mistaken for a fleet limit");
    require(
        randomInstance.customers.front().readyTime == 161.0 &&
            randomInstance.customers.front().dueTime == 171.0 &&
            randomInstance.customers.front().serviceTime == 10.0,
        "Set F time-window fields were not retained");
    require(
        randomInstance.matrixPolicy.travelCostRounding ==
                hgasp::NumericRounding::Floor &&
            randomInstance.matrixPolicy.travelTimeRounding ==
                hgasp::NumericRounding::Floor,
        "Set F floor-Euclidean matrix policy changed");

    const hgasp::ParsedInstance clustered =
        hgasp::loadInstance(
            (root / "instances/set_F/C101_s3_3_25.txt").string());
    const hgasp::ParsedDepot& depot = clustered.depots[1];
    const hgasp::ParsedCustomer& customer = clustered.customers[14];
    require(
        depot.x == customer.x && depot.y == customer.y,
        "co-located Set F depot and customer were altered or deduplicated");
}

void testAllFarhamFiles(const std::filesystem::path& root) {
    const std::filesystem::path directory = root / "instances/set_F";
    std::size_t parsedFiles = 0;
    for (const std::filesystem::directory_entry& entry :
         std::filesystem::directory_iterator(directory)) {
        if (!entry.is_regular_file() ||
            entry.path().extension() != ".txt") {
            continue;
        }
        const hgasp::ParsedInstance instance =
            hgasp::loadInstance(entry.path().string());
        require(
            !instance.customers.empty() && !instance.depots.empty(),
            "parsed an empty Set F instance: " +
                entry.path().filename().string());
        ++parsedFiles;
    }
    require(parsedFiles == 138, "did not parse all 138 Set F files");
}

void testModeCompatibility(const std::filesystem::path& root) {
    const std::string setSPath =
        (root / "instances/set_S/100-5-1e.json").string();
    const std::string setFPath =
        (root / "instances/set_F/R101_s1_3_10.txt").string();
    const hgasp::ParsedInstance setS = hgasp::loadInstance(setSPath);
    const hgasp::ParsedInstance setF = hgasp::loadInstance(setFPath);

    hgasp::validateModeCompatibility(
        setS, hgasp::ProblemMode::Clrp, setSPath);
    hgasp::validateModeCompatibility(
        setF, hgasp::ProblemMode::ClrpTw, setFPath);
    requireInvalidArgument(
        [&] {
            hgasp::validateModeCompatibility(
                setS, hgasp::ProblemMode::ClrpTw, setSPath);
        },
        "CLRP-TW accepted a Set S instance");
    requireInvalidArgument(
        [&] {
            hgasp::validateModeCompatibility(
                setF, hgasp::ProblemMode::Clrp, setFPath);
        },
        "CLRP accepted a Set F instance");
}

void testNonzeroCdcRejected() {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() /
        "hgasp_nonzero_cdc_instance.txt";
    {
        std::ofstream output(path);
        output
            << "1 1 1\n"
            << "1 225 200\n"
            << "1 0 0 100 1 200\n"
            << "1 1 1 1 0 10 0\n";
    }
    requireRuntimeError(
        [&] { (void)hgasp::loadInstance(path.string()); },
        "nonzero NUM_CDC was accepted");
    std::filesystem::remove(path);
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        const std::filesystem::path root =
            argc > 1 ? std::filesystem::path(argv[1])
                     : std::filesystem::current_path();
        testProblemModeAndCommandLine();
        testSetSJson(root);
        testSetSDat();
        testSetSDatReadDataRounding(root);
        testSetSJsonReadDataRounding(root);
        testFarhamPrefixAndCoordinates(root);
        testAllFarhamFiles(root);
        testModeCompatibility(root);
        testNonzeroCdcRejected();
        std::cout << "instance mode/parser tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "instance mode/parser test failed: "
                  << error.what() << '\n';
        return 1;
    }
}
