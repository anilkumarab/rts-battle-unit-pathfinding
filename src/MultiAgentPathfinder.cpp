#include "pathfinder/MultiAgentPathfinder.hpp"

#include <algorithm>
#include <cstdlib>
#include <queue>
#include <set>
#include <unordered_map>
#include <unordered_set>

namespace pathfinder {

namespace {

// A node in space-TIME: same cell can be revisited at different times, so
// the search state is (position, time) rather than just position.
struct TimeState {
    Position pos;
    int time;

    bool operator==(const TimeState& other) const { return pos == other.pos && time == other.time; }
};

struct TimeStateHash {
    std::size_t operator()(const TimeState& s) const noexcept {
        return std::hash<Position>()(s.pos) ^ (static_cast<std::size_t>(s.time) * 2654435761u);
    }
};

struct SearchNode {
    TimeState state;
    int gCost;
    int fCost;
    bool operator<(const SearchNode& other) const {
        if (fCost != other.fCost) return fCost > other.fCost;
        return gCost > other.gCost;
    }
};

int manhattanDistance(const Position& a, const Position& b) {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

/// Where the unit that already has `path` is at time `t`, if it is still
/// "on the board" at that time.
///
/// DESIGN DECISION / DOCUMENTED ASSUMPTION: a unit occupies its cell for
/// every recorded step of its path (including sitting at the target during
/// the final step), but is considered to have completed its mission and
/// vacated the battlefield immediately afterward, rather than occupying
/// its target cell forever. This is what makes the assessment's explicit
/// "units may move towards a common target position" case solvable at
/// all: if a completed unit held its cell indefinitely, no second unit
/// could ever occupy that same target, since "at any given moment, each
/// ground terrain position may be occupied by at most one unit" would be
/// permanently violated by definition. The alternative reading (units
/// stay forever) is also defensible, but makes shared targets impossible
/// by construction — since the brief calls out shared targets as a
/// supported case, this implementation resolves the ambiguity in favor of
/// "vacate on completion."
std::optional<Position> positionAtTime(const TimedPath& path, int t) {
    if (t < static_cast<int>(path.size())) return path[static_cast<std::size_t>(t)];
    return std::nullopt;  // this unit has completed its path and left the board
}

bool isReserved(const std::vector<TimedPath>& reservedPaths, const Position& pos, int t) {
    for (const auto& path : reservedPaths) {
        auto occupied = positionAtTime(path, t);
        if (occupied.has_value() && *occupied == pos) return true;
    }
    return false;
}

/// Detects a "swap" collision: two units crossing the same edge in
/// opposite directions between t and t+1, which would mean they pass
/// through each other. This is a conflict even though neither unit
/// occupies the other's exact cell at the exact same instant.
bool isSwapConflict(const std::vector<TimedPath>& reservedPaths, const Position& from, const Position& to,
                     int t) {
    for (const auto& path : reservedPaths) {
        auto posAtT = positionAtTime(path, t);
        auto posAtT1 = positionAtTime(path, t + 1);
        if (posAtT.has_value() && posAtT1.has_value() && *posAtT == to && *posAtT1 == from) return true;
    }
    return false;
}

std::optional<TimedPath> spaceTimeAStar(const Grid& grid, const Position& start, const Position& target,
                                         const std::vector<TimedPath>& reservedPaths, int timeHorizon) {
    if (isReserved(reservedPaths, start, 0)) return std::nullopt;  // another unit starts here at t=0

    std::priority_queue<SearchNode> openSet;
    std::unordered_map<TimeState, int, TimeStateHash> bestGCost;
    std::unordered_map<TimeState, TimeState, TimeStateHash> cameFrom;
    std::unordered_set<TimeState, TimeStateHash> closed;

    TimeState startState{start, 0};
    bestGCost[startState] = 0;
    openSet.push(SearchNode{startState, 0, manhattanDistance(start, target)});

    while (!openSet.empty()) {
        SearchNode current = openSet.top();
        openSet.pop();
        if (closed.count(current.state)) continue;
        closed.insert(current.state);

        // Goal reached AND we can safely stay there for the rest of the
        // horizon (otherwise a later unit could still need to pass through).
        if (current.state.pos == target) {
            bool safeToStay = true;
            for (int t = current.state.time + 1; t <= timeHorizon; ++t) {
                if (isReserved(reservedPaths, target, t)) {
                    safeToStay = false;
                    break;
                }
            }
            if (safeToStay) {
                TimedPath path;
                TimeState s = current.state;
                path.push_back(s.pos);
                while (!(s == startState)) {
                    s = cameFrom.at(s);
                    path.push_back(s.pos);
                }
                std::reverse(path.begin(), path.end());
                return path;
            }
        }

        if (current.state.time >= timeHorizon) continue;  // don't expand past the horizon

        // Candidate moves: the 4 orthogonal neighbors, plus "wait in place"
        // so a unit can let another one pass before continuing.
        std::vector<Position> candidates = grid.walkableNeighbors(current.state.pos);
        candidates.push_back(current.state.pos);  // wait

        for (const Position& next : candidates) {
            const int nextTime = current.state.time + 1;
            if (isReserved(reservedPaths, next, nextTime)) continue;
            if (isSwapConflict(reservedPaths, current.state.pos, next, current.state.time)) continue;

            TimeState nextState{next, nextTime};
            if (closed.count(nextState)) continue;

            const int tentativeG = current.gCost + 1;
            auto it = bestGCost.find(nextState);
            if (it == bestGCost.end() || tentativeG < it->second) {
                bestGCost[nextState] = tentativeG;
                cameFrom[nextState] = current.state;
                const int f = tentativeG + manhattanDistance(next, target);
                openSet.push(SearchNode{nextState, tentativeG, f});
            }
        }
    }

    return std::nullopt;
}

} // namespace

std::optional<std::vector<TimedPath>> MultiAgentPathfinder::findPaths(
    const std::vector<Position>& starts, const std::vector<Position>& targets) const {
    if (starts.size() != targets.size()) {
        return std::nullopt;
    }

    std::vector<TimedPath> plannedPaths;
    // Generous horizon: worst case a unit has to snake through every cell,
    // and we add slack per already-planned unit since later units may need
    // to wait for earlier ones to clear a corridor.
    const int baseHorizon = grid_.rows() * grid_.cols();

    for (std::size_t i = 0; i < starts.size(); ++i) {
        const int horizon = baseHorizon + static_cast<int>(plannedPaths.size()) * 4;
        auto path = spaceTimeAStar(grid_, starts[i], targets[i], plannedPaths, horizon);
        if (!path.has_value()) {
            return std::nullopt;  // this unit couldn't find a collision-free path
        }
        plannedPaths.push_back(*path);
    }

    return plannedPaths;
}

} // namespace pathfinder
