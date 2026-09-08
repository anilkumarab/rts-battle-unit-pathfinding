cmake_minimum_required(VERSION 3.15)
project(globus_pathfinding LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra)

# ---- Core library (algorithm + parser, no CLI) ----
add_library(pathfinder_core
    src/TilemapParser.cpp
    src/AStarPathfinder.cpp
    src/MultiAgentPathfinder.cpp
)
target_include_directories(pathfinder_core
    PUBLIC include
    PRIVATE third_party
)

# ---- CLI executable ----
add_executable(pathfinder src/main.cpp)
target_link_libraries(pathfinder PRIVATE pathfinder_core)

# ---- Test executable ----
add_executable(pathfinder_tests
    tests/test_main.cpp
    tests/test_grid.cpp
    tests/test_parser.cpp
    tests/test_astar.cpp
    tests/test_multiagent.cpp
)
target_link_libraries(pathfinder_tests PRIVATE pathfinder_core)

enable_testing()
add_test(NAME pathfinder_tests COMMAND pathfinder_tests)
