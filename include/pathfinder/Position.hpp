#pragma once

#include <compare>
#include <cstddef>
#include <functional>
#include <ostream>

namespace pathfinder {

/// A single (row, column) coordinate on the battlefield grid.
/// Row 0 is the top of the map, column 0 is the left edge.
struct Position {
    int row = 0;
    int col = 0;

    // C++20 spaceship operator: defaulting this gives us ==, !=, <, <=, >,
    // and >= for free, compared lexicographically (row first, then col).
    // This is what makes Position usable as a std::map/std::set key, which
    // the A* search relies on for its open/closed sets, and is why
    // MultiAgentPathfinder's TimeState can likewise be ordered.
    auto operator<=>(const Position&) const = default;
};

inline std::ostream& operator<<(std::ostream& os, const Position& p) {
    return os << "(" << p.row << ", " << p.col << ")";
}

} // namespace pathfinder

// Hash specialization so Position can be used in std::unordered_map/std::unordered_set,
// which the A* implementation relies on for its open/closed sets.
namespace std {
template <>
struct hash<pathfinder::Position> {
    std::size_t operator()(const pathfinder::Position& p) const noexcept {
        // Simple, collision-resistant-enough combination for grid coordinates.
        return (static_cast<std::size_t>(p.row) << 32) ^ static_cast<std::size_t>(p.col);
    }
};
} // namespace std
