#include "pathfinder/TilemapParser.hpp"

#include "MiniTest.hpp"

using namespace pathfinder;

namespace {

// A tiny 3x3 map:
//  0 -1 -1
// -1  3 -1
// -1 -1  8
const char* kSimpleMapJson = R"JSON(
{
  "width": 3,
  "height": 3,
  "layers": [
    { "data": [0, -1, -1, -1, 3, -1, -1, -1, 8] }
  ]
}
)JSON";

} // namespace

TEST(Parser_ParsesStartTargetAndElevatedTiles) {
    ParsedMap map = TilemapParser::parseString(kSimpleMapJson);
    ASSERT_EQ(map.grid.rows(), 3);
    ASSERT_EQ(map.grid.cols(), 3);
    ASSERT_TRUE(map.start.has_value());
    ASSERT_TRUE(map.target.has_value());
    ASSERT_EQ(*map.start, (Position{0, 0}));
    ASSERT_EQ(*map.target, (Position{2, 2}));
    ASSERT_FALSE(map.grid.isWalkable(Position{1, 1}));  // the "3" tile
    ASSERT_TRUE(map.grid.isWalkable(Position{0, 1}));   // a "-1" tile
}

TEST(Parser_ThrowsOnMismatchedDataLength) {
    const char* badJson = R"JSON(
    { "width": 3, "height": 3, "layers": [ { "data": [0, -1, 8] } ] }
    )JSON";
    bool threw = false;
    try {
        TilemapParser::parseString(badJson);
    } catch (const std::runtime_error&) {
        threw = true;
    }
    ASSERT_TRUE(threw);
}

TEST(Parser_ThrowsWhenStartTileMissing) {
    const char* noStartJson = R"JSON(
    { "width": 2, "height": 1, "layers": [ { "data": [-1, 8] } ] }
    )JSON";
    bool threw = false;
    try {
        TilemapParser::parseString(noStartJson);
    } catch (const std::runtime_error&) {
        threw = true;
    }
    ASSERT_TRUE(threw);
}

TEST(Parser_RoundTripsThroughToJsonString) {
    ParsedMap map = TilemapParser::parseString(kSimpleMapJson);
    std::string reserialized = TilemapParser::toJsonString(map.grid, *map.start, *map.target);
    ParsedMap reparsed = TilemapParser::parseString(reserialized);

    ASSERT_EQ(reparsed.grid.rows(), map.grid.rows());
    ASSERT_EQ(reparsed.grid.cols(), map.grid.cols());
    ASSERT_EQ(*reparsed.start, *map.start);
    ASSERT_EQ(*reparsed.target, *map.target);
    ASSERT_EQ(reparsed.grid.isWalkable(Position{1, 1}), map.grid.isWalkable(Position{1, 1}));
}
