// Globus Medical Software Candidate Assessment
// RTS battle-unit pathfinding — command-line interface.
//
// This is a minimal text-based UI, per the assessment's guidance that a
// backend-track candidate's UI "may be minimal (e.g., a simple text-based
// terminal)".
//
// Usage:
//   pathfinder <map.json>
//       Single-unit mode. Reads start/target from the map's tile IDs
//       (0 = start, 8 = target) and prints the resulting path.
//
//   pathfinder <map.json> --multi <r1,c1:r2,c2> [<r1,c1:r2,c2> ...]
//       Multi-unit mode (optional extra task). Ignores the map's own
//       start/target tiles and instead plans one unit per "start:target"
//       pair given on the command line, e.g.:
//         pathfinder samples/map_multi.json --multi 0,0:5,5 2,0:5,6
//
//   pathfinder <map.json> --out <result.json>
//       Also writes the resulting map (with start/target baked in) back
//       out in RiskyLab Tilemap JSON format.

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "pathfinder/AStarPathfinder.hpp"
#include "pathfinder/MultiAgentPathfinder.hpp"
#include "pathfinder/TilemapParser.hpp"

using namespace pathfinder;

namespace {

void printPath(const std::vector<Position>& path) {
    std::cout << "Path found (" << path.size() << " steps):\n";
    for (std::size_t i = 0; i < path.size(); ++i) {
        std::cout << "  " << i << ": " << path[i] << "\n";
    }
}

/// Parses "row,col" into a Position.
Position parsePosition(const std::string& text) {
    std::istringstream iss(text);
    int row, col;
    char comma;
    if (!(iss >> row >> comma >> col) || comma != ',') {
        throw std::runtime_error("Malformed position '" + text + "', expected format row,col");
    }
    return Position{row, col};
}

/// Parses "startRow,startCol:targetRow,targetCol".
void parseUnitSpec(const std::string& spec, std::vector<Position>& starts, std::vector<Position>& targets) {
    const auto colonPos = spec.find(':');
    if (colonPos == std::string::npos) {
        throw std::runtime_error("Malformed unit spec '" + spec + "', expected format startR,startC:targetR,targetC");
    }
    starts.push_back(parsePosition(spec.substr(0, colonPos)));
    targets.push_back(parsePosition(spec.substr(colonPos + 1)));
}

int runSingleUnit(const ParsedMap& map) {
    AStarPathfinder pathfinder(map.grid);
    auto path = pathfinder.findPath(*map.start, *map.target);
    if (!path.has_value()) {
        std::cout << "No path exists from " << *map.start << " to " << *map.target << ".\n";
        return 1;
    }
    printPath(*path);
    return 0;
}

int runMultiUnit(const ParsedMap& map, const std::vector<std::string>& unitSpecs) {
    std::vector<Position> starts, targets;
    for (const auto& spec : unitSpecs) {
        parseUnitSpec(spec, starts, targets);
    }

    MultiAgentPathfinder pathfinder(map.grid);
    auto paths = pathfinder.findPaths(starts, targets);
    if (!paths.has_value()) {
        std::cout << "Could not find collision-free paths for all " << starts.size() << " units.\n";
        return 1;
    }

    for (std::size_t i = 0; i < paths->size(); ++i) {
        std::cout << "Unit " << i << ": " << starts[i] << " -> " << targets[i] << "\n";
        printPath((*paths)[i]);
        std::cout << "\n";
    }
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <map.json> [--multi start:target ...] [--out result.json]\n";
        return 2;
    }

    const std::string mapPath = argv[1];
    std::vector<std::string> unitSpecs;
    std::string outPath;

    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--multi") {
            while (i + 1 < argc && std::string(argv[i + 1]).rfind("--", 0) != 0) {
                unitSpecs.push_back(argv[++i]);
            }
        } else if (arg == "--out" && i + 1 < argc) {
            outPath = argv[++i];
        } else {
            std::cerr << "Unrecognized argument: " << arg << "\n";
            return 2;
        }
    }

    try {
        ParsedMap map = TilemapParser::parseFile(mapPath);
        std::cout << "Loaded map: " << map.grid.rows() << " x " << map.grid.cols()
                  << " (rows x cols)\n";

        int status;
        if (!unitSpecs.empty()) {
            status = runMultiUnit(map, unitSpecs);
        } else {
            std::cout << "Start: " << *map.start << "  Target: " << *map.target << "\n";
            status = runSingleUnit(map);
        }

        if (!outPath.empty() && map.start.has_value() && map.target.has_value()) {
            std::ofstream outFile(outPath);
            outFile << TilemapParser::toJsonString(map.grid, *map.start, *map.target);
            std::cout << "Wrote result map to " << outPath << "\n";
        }

        return status;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
