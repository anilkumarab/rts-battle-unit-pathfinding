#include "pathfinder/Grid.hpp"

#include "MiniTest.hpp"

using namespace pathfinder;

TEST(Grid_DefaultsAllCellsToGround) {
    Grid grid(3, 3);
    ASSERT_TRUE(grid.isWalkable(Position{0, 0}));
    ASSERT_TRUE(grid.isWalkable(Position{2, 2}));
}

TEST(Grid_SetElevatedBlocksCell) {
    Grid grid(3, 3);
    grid.set(Position{1, 1}, CellType::Elevated);
    ASSERT_FALSE(grid.isWalkable(Position{1, 1}));
}

TEST(Grid_OutOfBoundsIsNotWalkable) {
    Grid grid(3, 3);
    ASSERT_FALSE(grid.isWalkable(Position{-1, 0}));
    ASSERT_FALSE(grid.isWalkable(Position{0, 3}));
    ASSERT_FALSE(grid.isWalkable(Position{3, 0}));
}

TEST(Grid_WalkableNeighborsExcludesElevatedAndOutOfBounds) {
    Grid grid(3, 3);
    grid.set(Position{0, 1}, CellType::Elevated);  // block one neighbor of (0,0)
    auto neighbors = grid.walkableNeighbors(Position{0, 0});
    // (0,0) only has 2 in-bounds neighbors: (1,0) and (0,1). (0,1) is
    // elevated, so only (1,0) should remain.
    ASSERT_EQ(neighbors.size(), static_cast<std::size_t>(1));
    ASSERT_EQ(neighbors[0], (Position{1, 0}));
}

TEST(Grid_ConstructorRejectsNonPositiveDimensions) {
    bool threw = false;
    try {
        Grid grid(0, 5);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    ASSERT_TRUE(threw);
}
