#include "read_data.h"
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>
#include <unistd.h>

std::mt19937 global_rng(1);

namespace {
struct Files {
    std::string directory;
    std::vector<std::string> paths;
    Files() {
        char pattern[] = "/tmp/masp-input-XXXXXX";
        char* created = mkdtemp(pattern);
        if (!created) throw std::runtime_error("cannot create input fixture directory");
        directory = created;
    }
    ~Files() {
        for (const auto& path : paths) std::remove(path.c_str());
        rmdir(directory.c_str());
    }
    std::string put(const std::string& name, const std::string& contents) {
        const std::string path = directory + "/" + name;
        std::ofstream stream(path);
        if (!(stream << contents)) throw std::runtime_error("cannot write input fixture");
        paths.push_back(path);
        return path;
    }
};
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
const std::string json =
    "{\"vehicle_costs\":1.25,\"vehicle_capacity\":3,"
    "\"depots\":[{\"y\":0,\"costs\":13.5,\"x\":0.25,\"capacity\":3}],"
    "\"metadata\":{\"unicode\":\"\\u0061\",\"unused\":[true,null,false]},"
    "\"customers\":[{\"y\":0,\"x\":0.5,\"demand\":1},{\"demand\":2,\"y\":0,\"x\":1.5}]}";

std::string replaced(std::string text, const std::string& from, const std::string& to) {
    const auto at = text.find(from);
    if (at == std::string::npos) throw std::runtime_error("fixture replacement is missing");
    text.replace(at, from.size(), to);
    return text;
}
void load(read_data& data, const std::string& instance, const std::string& configuration = "NULL", int limit = 10) {
    data.read_fun(instance, configuration, "unused-solution.txt", 0, 0, 60, limit);
}
void rejected(Files& files, const std::string& name, const std::string& contents,
              const std::string& configuration = "NULL") {
    const auto path = files.put(name, contents);
    bool failed = false;
    try { read_data data; load(data, path, configuration); }
    catch (const std::exception&) { failed = true; }
    require(failed, "invalid input was accepted: " + name);
}
} // namespace

int main() {
    try {
        unsetenv("MASP_SETS_EXPERIMENT"); unsetenv("MASP_SETF_EXPERIMENT");
        unsetenv("MASP_RUN_DEADLINE"); unsetenv("MASP_SP_POLICY");
        unsetenv("MASP_SP_FREQUENCY"); unsetenv("MASP_SP_BETA");
        Files files;
        const auto instance = files.put("reordered.json", json);
        const auto configuration = files.put("configuration.txt", "1\n0 e\n");
        {
            read_data data;
            load(data, instance, configuration);
            require(data.numV == 2 && data.numL == 1 && data.totalDemand == 3, "incorrect JSON dimensions or demands");
            require(data.x[0] == 0.5 && data.x[1] == 1.5 && data.x[2] == 0.25, "JSON coordinates were truncated or reordered");
            require(data.usingCostVeh == 1.25 && data.openCostLoc[2] == 13.5, "fractional cost was truncated");
            require(data.D[0][2] == 25 && data.D[1][2] == 125, "Set S ceil100 metric changed");
            data.bestDis = 10.5;
            require(data.stopCondition(10.5) && data.experimentTerminationReason == "target_reached",
                    "a supplied target still depends on an external acknowledgement");
        }
        {
            read_data data;
            load(data, instance); // Exact total capacity and no external configuration.
            require(!data.LocConfigure.empty(), "capacity-equality configuration was lost");
        }
        const std::string dat = "2 1\n0.25 0\n0.5 0\n1.5 0\n3\n3\n1 2\n13.5\n1.25\n0\n";
        {
            read_data data;
            load(data, files.put("native.dat", dat), configuration);
            require(data.D[0][2] == 25 && data.usingCostVeh == 1.25, "DAT values were truncated");
        }
        rejected(files, "fractional-demand.dat", replaced(dat, "1 2", "1.5 2"));
        rejected(files, "metric.dat", replaced(dat, "1.25\n0", "1.25\n2"));
        rejected(files, "missing.json", replaced(json, "\"demand\":1", "\"other\":1"));
        rejected(files, "zero-demand.json", replaced(json, "\"demand\":1", "\"demand\":0"));
        rejected(files, "fractional-demand.json", replaced(json, "\"demand\":1", "\"demand\":1.5"));
        rejected(files, "zero-capacity.json", replaced(json, "\"vehicle_capacity\":3", "\"vehicle_capacity\":0"));
        rejected(files, "overflow-distance.json", replaced(json, "\"x\":0.5", "\"x\":1e100"));
        rejected(files, "tail.json", json + " garbage");
        rejected(files, "duplicate.json", replaced(json, "\"vehicle_capacity\":3", "\"vehicle_capacity\":3,\"vehicle_capacity\":3"));
        rejected(files, "unsupported.input", json);
        for (const auto& item : std::vector<std::pair<std::string, std::string>>{
                {"negative", "1\n-1 e\n"}, {"outside", "1\n1 e\n"}, {"empty", "1\ne\n"},
                {"duplicate", "1\n0 0 e\n"}, {"missing-end", "1\n0\n"}, {"trailing", "1\n0 e\n0 e\n"}}) {
            const auto badConfiguration = files.put(item.first + ".cfg", item.second);
            rejected(files, item.first + ".json", json, badConfiguration);
        }
        const std::string farham = "0 1 1\n0 1.25 3\n1 0 0 9007199254740992 2.5 3\n"
            "1 0 0 1 9007199254740992 9007199254740992 1\n";
        {
            read_data data;
            load(data, files.put("large-time.txt", farham));
            require(!data.routeTimeFeasible(0, {0}), "service disappeared at a large ready time");
        }
        rejected(files, "farham-overflow.txt", replaced(farham, "0 1 1\n", "0 1 1 1e309\n"));
        rejected(files, "farham-garbage.txt", replaced(farham, "0 1 1\n", "0 1 1 nope\n"));
        {
            read_data data;
            const std::string zero = "0 1 1\n0 0 3\n1 0 0 10 0 3\n1 0 0 0 0 10 0\n";
            load(data, files.put("zero-farham.txt", zero));
            require(data.totalDemand == 0 && data.routeTimeFeasible(0, {0}), "Set F zero-demand support changed");
        }
        setenv("MASP_SETS_EXPERIMENT", "1", 1);
        setenv("MASP_SP_POLICY", "F20001_b1", 1);
        double perCall = 0;
        {
            read_data data;
            load(data, instance, configuration, 300000);
            perCall = data.spTimeLimit;
        }
        {
            read_data data;
            load(data, instance, configuration, 45000);
            require(data.spSchedule == std::vector<int>({20001, 40002}) && data.spTimeLimit == perCall,
                    "custom counter limit changed the per-call allocation or retained an invalid schedule");
        }
        unsetenv("MASP_SETS_EXPERIMENT"); unsetenv("MASP_SP_POLICY");
        std::cout << "input and configuration boundaries passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
