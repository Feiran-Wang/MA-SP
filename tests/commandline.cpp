#include "commandline.h"
#include <stdexcept>

std::mt19937 global_rng;

static CommandLine parse(std::vector<std::string> args) {
    std::vector<char*> argv;
    for (std::string& arg : args) argv.push_back(&arg[0]);
    return CommandLine(static_cast<int>(argv.size()), argv.data());
}

static void rejects(std::vector<std::string> args, std::size_t field, const std::string& value) {
    args[field] = value;
    try { parse(args); }
    catch (const std::exception&) { return; }
    throw std::runtime_error("Accepted invalid command-line field " + std::to_string(field) + ": " + value);
}

int main() {
    setenv("MASP_SETS_EXPERIMENT", "1", 1);
    const std::vector<std::string> shortForm = {"masp", "input.json", "output.txt", "0", "0", "100", "300000", "19"};
    const CommandLine shortParsed = parse(shortForm);
    if (shortParsed.timeLimit != 100 || shortParsed.iterLimit != 300000 || shortParsed.pathDepotCon != "NULL")
        throw std::runtime_error("Incorrect optional-depot argument mapping");
    const CommandLine longParsed = parse({"masp", "input.json", "depots.txt", "output.txt", "12.5", "1", "100", "300000", "19"});
    if (longParsed.pathDepotCon != "depots.txt" || longParsed.bestDis != 12.5 || longParsed.isOptimal != 1)
        throw std::runtime_error("Incorrect explicit-depot argument mapping");
    for (const std::string& bad : {"100junk", "0", "-1", "1.5", "2147483648"}) rejects(shortForm, 5, bad);
    for (const std::string& bad : {"nan", "inf", "1x", "-1", "1e309"}) rejects(shortForm, 3, bad);
    for (const std::string& bad : {"2", "-1", "0junk"}) rejects(shortForm, 4, bad);
    rejects(shortForm, 6, "-1");
    rejects(shortForm, 6, "2147483647");
    rejects(shortForm, 7, "1junk");
    std::vector<std::string> claimed = shortForm;
    claimed[4] = "1";
    rejects(claimed, 3, "0");
    return 0;
}
