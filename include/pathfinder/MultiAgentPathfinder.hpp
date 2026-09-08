#pragma once

#include <optional>
#include <vector>

#include "pathfinder/Grid.hpp"
#include "pathfinder/Position.hpp"

namespace pathfinder {

/// A path for one unit, indexed by time step (path[0] is where the unit is
/// at t=0, path[1] at t=1, etc). Once a unit reaches its target it's done —
/// it has no entry for later time steps, i.e. it's off the board — which is
/// what lets multiple units share a target position.
using TimedPath = std::vector<Position>;

/// Routes multiple units at once, each with its own start and (possibly
/// shared) target, so that no two units ever occupy the same cell at the
/// same time.
///
/// Approach: prioritized planning with a space-time reservation table.
/// Units are planned one at a time, in the order given. Each one runs a
/// space-time A* search (nodes are (position, time) instead of just
/// position) against a table of cells already reserved by earlier units.
/// It's a well-known, simple approach to cooperative pathfinding — not
/// guaranteed optimal or complete in every pathological case (a true
/// optimal solver would be something like Conflict-Based Search), but
/// reliable at the grid sizes here and much easier to read and verify.
class MultiAgentPathfinder {
public:
    explicit MultiAgentPathfinder(const Grid& grid) : grid_(grid) {}

    /// Plans one path per (start, target) pair, in order, avoiding
    /// collisions with all previously planned units. Returns std::nullopt
    /// for the whole batch if any single unit cannot find a collision-free
    /// path given the others' reservations.
    std::optional<std::vector<TimedPath>> findPaths(const std::vector<Position>& starts,
                                                      const std::vector<Position>& targets) const;

private:
    const Grid& grid_;
};

} // namespace pathfinder
