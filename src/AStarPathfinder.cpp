#include "pathfinder/AStarPathfinder.hpp"

#include <algorithm>
#include <cstdlib>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace pathfinder {

namespace {

struct Node {
    Position pos;
    int gCost;  // exact cost from start to this node
    int fCost;  // gCost + heuristic estimate to target

    // std::priority_queue is a max-heap, so we invert the comparison to get
    // a min-heap ordered by lowest fCost (ties broken by lowest gCost, which
    // prefers nodes closer to the goal already).
    bool operator<(const Node& other) const {
        if (fCost != other.fCost) return fCost > other.fCost;
        return gCost > other.gCost;
    }
};

int manhattanDistance(const Position& a, const Position& b) {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

std::vector<Position> reconstructPath(const std::unordered_map<Position, Position>& cameFrom,
                                       const Position& start, const Position& target) {
    std::vector<Position> path;
    Position current = target;
    path.push_back(current);
    while (current != start) {
        current = cameFrom.at(current);
        path.push_back(current);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

} // namespace

std::optional<std::vector<Position>> AStarPathfinder::findPath(const Position& start,
                                                                 const Position& target) const {
    if (!grid_.isWalkable(start)) {
        throw std::invalid_argument("Start position is out of bounds or on elevated terrain");
    }
    if (!grid_.isWalkable(target)) {
        throw std::invalid_argument("Target position is out of bounds or on elevated terrain");
    }

    if (start == target) {
        return std::vector<Position>{start};
    }

    std::priority_queue<Node> openSet;
    std::unordered_map<Position, int> bestGCost;
    std::unordered_map<Position, Position> cameFrom;
    std::unordered_set<Position> closed;

    bestGCost[start] = 0;
    openSet.push(Node{start, 0, manhattanDistance(start, target)});

    while (!openSet.empty()) {
        Node current = openSet.top();
        openSet.pop();

        if (closed.count(current.pos)) continue;  // stale queue entry, already finalized
        closed.insert(current.pos);

        if (current.pos == target) {
            return reconstructPath(cameFrom, start, target);
        }

        for (const Position& neighbor : grid_.walkableNeighbors(current.pos)) {
            if (closed.count(neighbor)) continue;

            const int tentativeG = current.gCost + 1;  // uniform cost per step
            auto it = bestGCost.find(neighbor);
            if (it == bestGCost.end() || tentativeG < it->second) {
                // Found a cheaper (or first) route to `neighbor` — overwrite it.
                bestGCost[neighbor] = tentativeG;
                cameFrom[neighbor] = current.pos;
                const int f = tentativeG + manhattanDistance(neighbor, target);
                openSet.push(Node{neighbor, tentativeG, f});
            }
        }
    }

    return std::nullopt;  // open set exhausted without reaching target: no path exists
}

} // namespace pathfinder
