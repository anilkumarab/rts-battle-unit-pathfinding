#include "pathfinder/MultiAgentPathfinder.hpp"

#include <set>

#include "MiniTest.hpp"

using namespace pathfinder;

namespace {

// Matches MultiAgentPathfinder's documented reservation model: a unit
// occupies a cell only for the recorded steps of its own path, and is
// considered to have left the board (vacated) once its path is finished.
std::optional<Position> atTime(const TimedPath& path, int t) {
    if (t < static_cast<int>(path.size())) return path[static_cast<std::size_t>(t)];
    return std::nullopt;
}

bool anyVertexCollision(const std::vector<TimedPath>& paths) {
    int maxLen = 0;
    for (const auto& p : paths) maxLen = std::max(maxLen, static_cast<int>(p.size()));
    for (int t = 0; t < maxLen; ++t) {
        std::set<Position> occupied;
        for (const auto& p : paths) {
            auto pos = atTime(p, t);
            if (pos.has_value() && !occupied.insert(*pos).second) return true;  // collision found
        }
    }
    return false;
}

} // namespace

TEST(MultiAgent_TwoUnitsToDistinctTargetsOnOpenGrid) {
    Grid grid(5, 5);
    MultiAgentPathfinder pathfinder(grid);
    auto result = pathfinder.findPaths({Position{0, 0}, Position{4, 4}}, {Position{4, 4}, Position{0, 0}});
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->size(), static_cast<std::size_t>(2));
    ASSERT_FALSE(anyVertexCollision(*result));
    ASSERT_EQ((*result)[0].back(), (Position{4, 4}));
    ASSERT_EQ((*result)[1].back(), (Position{0, 0}));
}

TEST(MultiAgent_UnitsForcedThroughNarrowCorridorDontCollide) {
    // A single-width corridor forces one unit to wait while the other passes.
    //   S1 . . . T2
    //   #  #  #  #  #
    //   .  .  .  .  .   <- corridor row
    //   #  #  #  #  #
    //   S2 . . . T1
    Grid grid(5, 5);
    for (int col = 0; col < 5; ++col) {
        if (col != 2) {
            grid.set(Position{1, col}, CellType::Elevated);
            grid.set(Position{3, col}, CellType::Elevated);
        }
    }
    MultiAgentPathfinder pathfinder(grid);
    std::vector<Position> starts{Position{0, 0}, Position{4, 0}};
    std::vector<Position> targets{Position{4, 4}, Position{0, 4}};
    auto result = pathfinder.findPaths(starts, targets);

    ASSERT_TRUE(result.has_value());
    ASSERT_FALSE(anyVertexCollision(*result));
}

TEST(MultiAgent_ReturnsNulloptWhenMismatchedInputSizes) {
    Grid grid(3, 3);
    MultiAgentPathfinder pathfinder(grid);
    auto result = pathfinder.findPaths({Position{0, 0}}, {});
    ASSERT_FALSE(result.has_value());
}

TEST(MultiAgent_SharedTargetBothUnitsArrive) {
    Grid grid(3, 3);
    MultiAgentPathfinder pathfinder(grid);
    Position sharedTarget{2, 2};
    auto result = pathfinder.findPaths({Position{0, 0}, Position{0, 2}}, {sharedTarget, sharedTarget});
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ((*result)[0].back(), sharedTarget);
    ASSERT_EQ((*result)[1].back(), sharedTarget);
    ASSERT_FALSE(anyVertexCollision(*result));
}
