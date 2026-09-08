#pragma once

#include <optional>
#include <vector>

#include "pathfinder/Grid.hpp"
#include "pathfinder/Position.hpp"

namespace pathfinder {

/// Finds a shortest path for a single battle unit across a Grid using A*.
///
/// Using A* over plain BFS since the Manhattan-distance heuristic (movement
/// is 4-directional) lets it explore far fewer cells on large/maze-like
/// maps while still guaranteeing a shortest path. It also backtracks for
/// free: re-expanding a cell when a cheaper route to it turns up, so it
/// can't get stuck the way a greedy "always step toward the target" walker
/// would in a maze.
class AStarPathfinder {
public:
    explicit AStarPathfinder(const Grid& grid) : grid_(grid) {}

    /// Returns the sequence of positions from `start` to `target`
    /// (inclusive of both endpoints), or std::nullopt if no path exists.
    /// Throws std::invalid_argument if start or target sit on Elevated
    /// terrain or are out of bounds.
    std::optional<std::vector<Position>> findPath(const Position& start, const Position& target) const;

private:
    const Grid& grid_;
};

} // namespace pathfinder
