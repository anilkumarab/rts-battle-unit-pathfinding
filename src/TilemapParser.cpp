#include "pathfinder/TilemapParser.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

#include "json.hpp"

namespace pathfinder {

using json = nlohmann::json;

namespace {

constexpr int kStartTile = 0;
constexpr int kElevatedTile = 3;
constexpr int kTargetTile = 8;

ParsedMap parseJson(const json& root) {
    if (!root.contains("width") || !root.contains("height")) {
        throw std::runtime_error("Tilemap JSON missing top-level 'width'/'height' fields");
    }
    if (!root.contains("layers") || !root["layers"].is_array() || root["layers"].empty()) {
        throw std::runtime_error("Tilemap JSON missing non-empty 'layers' array");
    }

    const int width = root["width"].get<int>();
    const int height = root["height"].get<int>();

    const json& layer0 = root["layers"][0];
    if (!layer0.contains("data") || !layer0["data"].is_array()) {
        throw std::runtime_error("Tilemap JSON missing 'layers[0].data' array");
    }
    const auto& data = layer0["data"];

    if (static_cast<int>(data.size()) != width * height) {
        std::ostringstream oss;
        oss << "layers[0].data has " << data.size() << " entries, expected width*height = "
            << width << "*" << height << " = " << (width * height);
        throw std::runtime_error(oss.str());
    }

    ParsedMap result{Grid(height, width), std::nullopt, std::nullopt};

    for (int i = 0; i < static_cast<int>(data.size()); ++i) {
        const int row = i / width;
        const int col = i % width;
        const Position pos{row, col};
        const int tile = data[static_cast<std::size_t>(i)].get<int>();

        switch (tile) {
            case kElevatedTile:
                result.grid.set(pos, CellType::Elevated);
                break;
            case kStartTile:
                result.start = pos;
                result.grid.set(pos, CellType::Ground);
                break;
            case kTargetTile:
                result.target = pos;
                result.grid.set(pos, CellType::Ground);
                break;
            default:
                // Covers -1 (reachable) and any other/unknown tile ID: treat
                // as walkable ground rather than guessing it's an obstacle.
                result.grid.set(pos, CellType::Ground);
                break;
        }
    }

    if (!result.start.has_value()) {
        throw std::runtime_error("Tilemap JSON does not contain a starting position (tile value 0)");
    }
    if (!result.target.has_value()) {
        throw std::runtime_error("Tilemap JSON does not contain a target position (tile value 8)");
    }

    return result;
}

} // namespace

ParsedMap TilemapParser::parseString(const std::string& jsonText) {
    json root;
    try {
        root = json::parse(jsonText);
    } catch (const json::parse_error& e) {
        throw std::runtime_error(std::string("Failed to parse tilemap JSON: ") + e.what());
    }
    return parseJson(root);
}

ParsedMap TilemapParser::parseFile(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open tilemap file: " + path);
    }
    std::ostringstream buffer;
    buffer << file.rdbuf();
    return parseString(buffer.str());
}

std::string TilemapParser::toJsonString(const Grid& grid, const Position& start, const Position& target) {
    json root;
    root["width"] = grid.cols();
    root["height"] = grid.rows();

    json data = json::array();
    for (int row = 0; row < grid.rows(); ++row) {
        for (int col = 0; col < grid.cols(); ++col) {
            Position p{row, col};
            int tile;
            if (p == start) {
                tile = kStartTile;
            } else if (p == target) {
                tile = kTargetTile;
            } else if (grid.at(p) == CellType::Elevated) {
                tile = kElevatedTile;
            } else {
                tile = -1;
            }
            data.push_back(tile);
        }
    }

    json layer;
    layer["data"] = data;
    layer["width"] = grid.cols();
    layer["height"] = grid.rows();
    layer["name"] = "battlefield";

    root["layers"] = json::array({layer});

    return root.dump(2);
}

} // namespace pathfinder
