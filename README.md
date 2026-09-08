# Globus Medical — Software Candidate Assessment
### RTS Battle-Unit Pathfinding

A path-finding solution for a Real-Time Strategy battlefield: given a grid map with
walkable ("ground") and blocked ("elevated") terrain, a unit's starting position, and a
target position, the program computes a valid step-by-step path between them. It also
implements the optional extra task: routing multiple units simultaneously without
collisions.

Implemented in **C++20** (per the language explicitly requested in the assessment's
follow-up email — the job description itself specifies "C++17 and later," and C++20
satisfies that) with a minimal text-based CLI, matching the assessment's guidance for a
backend-track candidate ("the UI may be minimal, e.g. a simple text-based terminal")
and the Senior Software Engineer (Robotics) job description this assessment was given
for, which calls out object-oriented C++, test-driven development, and clean,
maintainable, well-documented software.

---

## 1. Design Decisions

### Algorithm choice: A* over plain BFS/DFS
Both BFS and A* are guaranteed to find a *shortest* path on this kind of unweighted
grid. A* was chosen because it uses a Manhattan-distance heuristic (valid here since
movement is strictly 4-directional) to explore far fewer cells than BFS on large or
maze-like maps, while remaining complete and optimal. Its open/closed-set structure
also naturally satisfies the assessment's requirement that the algorithm "must be
capable of backtracking and finding a valid path, even in complex scenarios": A*
re-expands any cell it finds a cheaper route to, so it cannot get permanently stuck the
way a naive "always step toward the target" greedy walker would in a maze (this exact
scenario — a U-shaped wall that traps a greedy walker — is one of the automated tests,
`AStar_RequiresBacktrackingThroughUShapedWall`).

### Multi-unit extension: prioritized planning with space-time A*
For the optional bonus, each unit is planned **one at a time**, in the order given.
Each unit's search runs in **space-time** — its search state is `(position, time)`
rather than just `position` — against a shared reservation table of cells already
claimed by earlier units at each time step. A unit may also **wait in place** for one
time step, which lets it yield to another unit in a narrow corridor rather than failing
outright.

This is a standard, well-known approach to cooperative pathfinding (often called
*prioritized planning*). It is simple to read, verify, and test, and it's more than
sufficient for the grid sizes this assessment targets. It is **not** guaranteed to find
a solution in every theoretically solvable scenario (a provably optimal/complete
solution requires substantially more expensive algorithms, e.g. Conflict-Based Search),
which is a deliberate trade-off favoring the assessment's explicit ask for "clean,
maintainable, and efficient code" over maximal sophistication.

### Documented ambiguity #1 — RiskyLab Tilemap tile IDs
The brief states tile meanings are "depending on the icon set," without pinning down a
single canonical mapping. This implementation documents and uses the mapping given in
the brief's bullet list: `0` = start, `8` = target, `3` = elevated/blocked, `-1` =
reachable ground. Any *other* tile value encountered is treated as walkable ground
rather than as an obstacle, so that decorative/unknown tile IDs in a real exported map
don't silently make the level unsolvable. This is called out explicitly in
`TilemapParser.hpp`.

### Documented ambiguity #2 — what "reaching a common target" means for multiple units
The brief allows multiple units to move toward *a common target position*, but also
states that "at any given moment, each ground terrain position may be occupied by at
most one unit." Taken completely literally forever, these two statements conflict: if a
unit that reaches the target held that cell for all time afterward, no second unit
could ever occupy it, and "common target" would be impossible by construction.

This implementation resolves the ambiguity by treating a unit as having **completed its
mission and left the battlefield** the instant after it finishes its recorded path,
rather than occupying its final cell forever. This keeps the "no two units share a cell
at the same moment" constraint intact for every step a unit is actually on the board,
while making the explicitly-supported "common target" case solvable. This is documented
in code at `MultiAgentPathfinder.cpp` (see `positionAtTime`), and is covered by the test
`MultiAgent_SharedTargetBothUnitsArrive`.

### Test framework: a small self-contained one, not a fetched dependency
Rather than pulling in GoogleTest or Catch2 (which would need to be fetched and built,
adding a build-time dependency and a step that can fail on a reviewer's machine without
internet access), the test suite uses a ~70-line self-contained header
(`tests/MiniTest.hpp`) providing `TEST`, `ASSERT_TRUE/FALSE/EQ`. It's not meant to
compete feature-for-feature with a real framework — just to keep the build trivially
reproducible while still supporting genuine TDD-style, readable tests.

### Third-party library: nlohmann/json (header-only)
JSON parsing/writing uses [nlohmann/json](https://github.com/nlohmann/json) v3.11.3, a
widely-used single-header library, vendored directly in `third_party/json.hpp`. This
was the one third-party dependency used in the whole project; everything else (the
pathfinding, the CLI, the tests) is written from scratch. Using a battle-tested JSON
library rather than hand-rolling a parser avoids introducing bugs in a part of the
system that isn't what this assessment is actually testing.

---

## 2. Project Structure

```
globus-pathfinding/
├── CMakeLists.txt              Build configuration (library + CLI + tests)
├── README.md                   This file
├── include/pathfinder/
│   ├── Position.hpp            A single (row, col) grid coordinate
│   ├── Grid.hpp                The battlefield: terrain grid + walkability queries
│   ├── TilemapParser.hpp       RiskyLab Tilemap JSON <-> Grid conversion
│   ├── AStarPathfinder.hpp     Single-unit shortest-path search
│   └── MultiAgentPathfinder.hpp  Optional bonus: multi-unit collision-free routing
├── src/
│   ├── TilemapParser.cpp
│   ├── AStarPathfinder.cpp
│   ├── MultiAgentPathfinder.cpp
│   └── main.cpp                 CLI entry point
├── tests/
│   ├── MiniTest.hpp              Minimal self-contained test framework
│   ├── test_main.cpp
│   ├── test_grid.cpp
│   ├── test_parser.cpp
│   ├── test_astar.cpp
│   └── test_multiagent.cpp
├── third_party/
│   └── json.hpp                  nlohmann/json v3.11.3 (vendored, header-only)
└── samples/
    ├── map_simple_5x5.json       Small hand-crafted map with a wall detour
    ├── map_no_path_5x5.json      Target fully walled off — exercises the "no path" case
    ├── map_maze_32x32.json       32x32 generated maze (meets the "at least 32x32" spec)
    ├── map_multi_6x6.json        Open map used for the multi-unit demo
    └── output/                   Captured sample run results (see section 4)
```

**Class responsibilities**, deliberately kept single-purpose and independently
testable:
- `Grid` knows only about terrain — no notion of units, start/target, or JSON.
- `TilemapParser` only converts between the RiskyLab JSON format and a `Grid` (+ start/
  target positions). It doesn't know how pathfinding works.
- `AStarPathfinder` only knows how to search a `Grid`. It doesn't know about JSON or the
  CLI.
- `MultiAgentPathfinder` builds on the same `Grid` abstraction, independent of the
  single-unit pathfinder.
- `main.cpp` wires these together into a CLI and is the only place that touches
  `std::cout`/argv.

---

## 3. Build Instructions

**Requirements:** a C++20 compiler (tested with g++ 13) and CMake ≥ 3.15.

```bash
mkdir build && cd build
cmake ..
cmake --build .
```

This produces two executables in `build/`:
- `pathfinder` — the CLI
- `pathfinder_tests` — the test suite

Run the tests:
```bash
./pathfinder_tests
# or: ctest
```

### Running the CLI

**Single unit** (start/target read from the map's own `0`/`8` tiles):
```bash
./pathfinder ../samples/map_simple_5x5.json
```

**Multiple units** (bonus task — start:target pairs given on the command line,
`row,col:row,col`, space-separated):
```bash
./pathfinder ../samples/map_multi_6x6.json --multi 0,0:5,5 5,0:0,5
```

**Write the result back out** as a RiskyLab-Tilemap-format JSON file:
```bash
./pathfinder ../samples/map_simple_5x5.json --out result.json
```

---

## 4. Sample Runs

All files referenced below are included under `samples/` and `samples/output/`, with
real captured output (not hand-written) from running the built binaries.

| Scenario | Input | Captured output |
|---|---|---|
| Small map requiring a detour around a wall | `samples/map_simple_5x5.json` | `samples/output/run_simple_5x5.txt`, `samples/output/map_simple_5x5_result.json` |
| 32×32 generated maze (satisfies "at least 32×32") | `samples/map_maze_32x32.json` | `samples/output/run_maze_32x32.txt`, `samples/output/map_maze_32x32_result.json` |
| Target fully walled off — no path exists | `samples/map_no_path_5x5.json` | `samples/output/run_no_path_5x5.txt` |
| Two units to distinct targets, collision-free (bonus) | `samples/map_multi_6x6.json` | `samples/output/run_multi_6x6.txt` |
| Full automated test suite | — | `samples/output/test_run_output.txt` (18/18 passing) |

Example — the small map (`map_simple_5x5.json`) is a 5×5 grid with a wall separating
start and target except for a single-cell gap in the middle:
```
. . . . .
. # # # .
S # . # T
. # # # .
. . . . .
```
The program correctly finds the 9-step detour around the top of the wall (it can't cut
through the isolated open cell in the middle, since that cell has no connection to
either side):
```
Start: (2, 0)  Target: (2, 4)
Path found (9 steps):
  0: (2, 0)
  1: (1, 0)
  2: (0, 0)
  3: (0, 1)
  4: (0, 2)
  5: (0, 3)
  6: (0, 4)
  7: (1, 4)
  8: (2, 4)
```

---

## 5. Concerns / Notes on the Problem Statement

As requested by the "General Requirements" section, a few points where the brief was
ambiguous or underspecified, and how this implementation resolves them:

1. **Tile ID meaning is icon-set-dependent** — resolved by using the literal mapping
   given in the brief and treating any other value as walkable ground (see section 1).
2. **"Common target" vs. "at most one unit per position at any moment"** are in tension
   if a unit is assumed to occupy its final cell forever — resolved by treating a unit
   as having left the battlefield once its path is complete (see section 1).
3. **The RiskyLab Tilemap sample link in the brief** wasn't directly accessible in this
   environment, so the JSON structure (`width`, `height`, `layers[0].data`) was inferred
   from the brief's own description ("The `layers[0].data` field... has *row x columns*
   entries") rather than a fetched sample file. The parser validates this shape
   explicitly and raises a clear error if a real-world file doesn't match, rather than
   failing silently.
4. **Diagonal movement** is explicitly disallowed by the brief ("travel horizontally...
   or vertically") — the `Grid::walkableNeighbors` implementation only considers the 4
   orthogonal directions, and this is covered by a unit test.

---

## 6. Feedback on the Assessment

1. **Most helpful:** the explicit constraints section (discrete movement, 4-directional
   only, must handle backtracking) made the algorithm choice and test design
   unambiguous.
2. **Challenges:** the two ambiguities noted above (tile-ID mapping, and the
   common-target vs. exclusive-occupancy tension) took the most thought to resolve
   cleanly — see section 5.
3. **Could be clearer:** a concrete example RiskyLab Tilemap JSON snippet embedded
   directly in the brief (rather than only a link) would remove all doubt about the
   exact field names/shape expected.
4. **Suggestion:** since the JD is C++/robotics-specific, explicitly stating the
   expected language up front (rather than "agreed upon when you received the project")
   would help candidates gauge scope before starting.
