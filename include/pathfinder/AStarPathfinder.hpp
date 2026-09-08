#pragma once

#include <optional>
#include <vector>

#include "pathfinder/Grid.hpp"
#include "pathfinder/Position.hpp"

namespace pathfinder {

/// Finds a shortest path for a single battle unit across a Grid using the
/// A* search algorithm.
///
/// Why A* rather than plain BFS: both are guaranteed to find a shortest
/// path on an unweighted grid like this one, but A* uses a heuristic
/// (Manhattan distance, since movement is 4-directional) to explore far
/// fewer cells on large or maze-like maps, while still being complete and
/// optimal. That satisfies the assessment's requirement that the algorithm
/// "must be capable of backtracking and finding a valid path, even in
/// complex scenarios": A* naturally backtracks by re-expanding any cell it
/// discovers a cheaper route to, and its open/closed sets guarantee every
/// reachable cell is eventually considered, so it can't get stuck the way
/// a naive greedy "always step toward the target" approach would in a maze.
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
