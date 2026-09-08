#pragma once

#include <stdexcept>
#include <vector>

#include "pathfinder/Position.hpp"

namespace pathfinder {

/// What a single battlefield cell is made of.
enum class CellType {
    Ground,   // Occupiable by a single battle unit.
    Elevated  // Unreachable by battle units.
};

/// The battlefield: a rectangular grid of cells, each Ground or Elevated.
/// Only knows about terrain — no notion of units or start/target positions.
class Grid {
public:
    Grid(int rows, int cols)
        : rows_(rows), cols_(cols), cells_(static_cast<std::size_t>(rows) * cols, CellType::Ground) {
        if (rows <= 0 || cols <= 0) {
            throw std::invalid_argument("Grid dimensions must be positive");
        }
    }

    int rows() const { return rows_; }
    int cols() const { return cols_; }

    bool inBounds(const Position& p) const {
        return p.row >= 0 && p.row < rows_ && p.col >= 0 && p.col < cols_;
    }

    CellType at(const Position& p) const {
        checkBounds(p);
        return cells_[index(p)];
    }

    void set(const Position& p, CellType type) {
        checkBounds(p);
        cells_[index(p)] = type;
    }

    /// A cell is walkable if it is in bounds and its terrain is Ground.
    bool isWalkable(const Position& p) const {
        return inBounds(p) && cells_[index(p)] == CellType::Ground;
    }

    /// The four orthogonal neighbors of a position that are actually walkable.
    /// No diagonals.
    std::vector<Position> walkableNeighbors(const Position& p) const {
        std::vector<Position> result;
        result.reserve(4);
        static const Position deltas[4] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
        for (const auto& d : deltas) {
            Position n{p.row + d.row, p.col + d.col};
            if (isWalkable(n)) result.push_back(n);
        }
        return result;
    }

private:
    std::size_t index(const Position& p) const {
        return static_cast<std::size_t>(p.row) * cols_ + p.col;
    }

    void checkBounds(const Position& p) const {
        if (!inBounds(p)) throw std::out_of_range("Position out of grid bounds");
    }

    int rows_;
    int cols_;
    std::vector<CellType> cells_;
};

} // namespace pathfinder
