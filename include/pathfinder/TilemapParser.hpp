#pragma once

#include <optional>
#include <string>

#include "pathfinder/Grid.hpp"
#include "pathfinder/Position.hpp"

namespace pathfinder {

/// Result of parsing a RiskyLab-Tilemap-style JSON map: the terrain grid
/// plus the unit's start and target positions found while scanning the
/// tile data.
struct ParsedMap {
    Grid grid;
    std::optional<Position> start;
    std::optional<Position> target;
};

/// Parses battlefield maps in the JSON tile format used by RiskyLab Tilemap
/// (https://tilemap.riskylab.com/), where layers[0].data is a row-major
/// array of tile IDs.
///
/// Tile ID convention (per the assessment brief):
///   0  -> the unit's starting position (walkable ground)
///  -1  -> a plain reachable/ground position
///   3  -> elevated terrain (blocked / unwalkable)
///   8  -> the target position (walkable ground)
///
/// NOTE ON AMBIGUITY: the brief says tile meanings are "depending on the
/// icon set," so the mapping above is the documented assumption this
/// implementation makes (see README "Design Decisions" for details).
/// Any tile value other than {-1, 3, 8} that isn't the designated start
/// tile is treated as walkable ground, so custom decorative tiles don't
/// accidentally block the unit.
class TilemapParser {
public:
    /// Parses a map from a JSON string.
    /// Throws std::runtime_error with a descriptive message if the JSON is
    /// malformed or missing required fields (width/height/layers[0].data),
    /// or if the data array length doesn't match width * height.
    static ParsedMap parseString(const std::string& jsonText);

    /// Parses a map directly from a file on disk.
    /// Throws std::runtime_error if the file can't be opened or parsed.
    static ParsedMap parseFile(const std::string& path);

    /// Serializes a ParsedMap-shaped result (or any grid + path) back out
    /// in RiskyLab Tilemap JSON format, so run results can round-trip
    /// through the same tool the assessment references. Elevated cells
    /// become 3, everything else -1, except the given start/target which
    /// are written as 0/8 respectively.
    static std::string toJsonString(const Grid& grid, const Position& start, const Position& target);
};

} // namespace pathfinder
