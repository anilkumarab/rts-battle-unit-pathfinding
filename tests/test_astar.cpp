#include "pathfinder/AStarPathfinder.hpp"

#include "MiniTest.hpp"

using namespace pathfinder;

namespace {

bool isValidPath(const Grid& grid, const std::vector<Position>& path, const Position& start,
                  const Position& target) {
    if (path.empty()) return false;
    if (path.front() != start || path.back() != target) return false;
    for (std::size_t i = 0; i < path.size(); ++i) {
        if (!grid.isWalkable(path[i])) return false;
        if (i > 0) {
            const auto& a = path[i - 1];
            const auto& b = path[i];
            const int stepDistance = std::abs(a.row - b.row) + std::abs(a.col - b.col);
            if (stepDistance != 1) return false;  // must be a single orthogonal step
        }
    }
    return true;
}

} // namespace

TEST(AStar_FindsPathOnOpenGrid) {
    Grid grid(5, 5);
    AStarPathfinder pathfinder(grid);
    auto path = pathfinder.findPath(Position{0, 0}, Position{4, 4});
    ASSERT_TRUE(path.has_value());
    ASSERT_TRUE(isValidPath(grid, *path, Position{0, 0}, Position{4, 4}));
    // On an open grid, Manhattan distance is the true shortest-path length.
    ASSERT_EQ(path->size(), static_cast<std::size_t>(9));  // 8 steps + 1 for start
}

TEST(AStar_StartEqualsTargetReturnsSingleCellPath) {
    Grid grid(3, 3);
    AStarPathfinder pathfinder(grid);
    auto path = pathfinder.findPath(Position{1, 1}, Position{1, 1});
    ASSERT_TRUE(path.has_value());
    ASSERT_EQ(path->size(), static_cast<std::size_t>(1));
}

TEST(AStar_ReturnsNulloptWhenTargetIsWalledOff) {
    // Fully enclose (2,2) with elevated terrain so it's unreachable.
    Grid grid(5, 5);
    grid.set(Position{1, 2}, CellType::Elevated);
    grid.set(Position{3, 2}, CellType::Elevated);
    grid.set(Position{2, 1}, CellType::Elevated);
    grid.set(Position{2, 3}, CellType::Elevated);

    AStarPathfinder pathfinder(grid);
    auto path = pathfinder.findPath(Position{0, 0}, Position{2, 2});
    ASSERT_FALSE(path.has_value());
}

TEST(AStar_RequiresBacktrackingThroughUShapedWall) {
    // A U-shaped wall that traps a naive "always move toward target"
    // greedy walker: the shortest visible direction from (2,0) toward
    // (2,4) is blocked, forcing the algorithm to detour up and around.
    //
    //   . . . . .
    //   . # # # .
    //   S # . # T   <- S=(2,0) start, T=(2,4) target, '.'=(2,2) is enclosed
    //   . # # # .
    //   . . . . .
    Grid grid(5, 5);
    for (int row = 1; row <= 3; ++row) {
        grid.set(Position{row, 1}, CellType::Elevated);
        grid.set(Position{row, 3}, CellType::Elevated);
    }
    grid.set(Position{1, 2}, CellType::Elevated);
    grid.set(Position{3, 2}, CellType::Elevated);
    // Note: (2,2) is intentionally left open but fully enclosed - it's an
    // isolated pocket the path must route around entirely, not through.

    AStarPathfinder pathfinder(grid);
    auto path = pathfinder.findPath(Position{2, 0}, Position{2, 4});
    ASSERT_TRUE(path.has_value());
    ASSERT_TRUE(isValidPath(grid, *path, Position{2, 0}, Position{2, 4}));
    // Must detour around the top or bottom of the wall; straight-line
    // Manhattan distance is 4, so a valid detour is strictly longer.
    ASSERT_TRUE(path->size() > 5);
}

TEST(AStar_ThrowsWhenStartIsOnElevatedTerrain) {
    Grid grid(3, 3);
    grid.set(Position{0, 0}, CellType::Elevated);
    AStarPathfinder pathfinder(grid);
    bool threw = false;
    try {
        pathfinder.findPath(Position{0, 0}, Position{2, 2});
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    ASSERT_TRUE(threw);
}
