#include "Maze.h"

Maze::Maze() { reset(); }

void Maze::reset() {
  for (uint8_t y = 0; y < kMazeSize; y++) {
    for (uint8_t x = 0; x < kMazeSize; x++) {
      Cell &c = grid_[x][y];
      c.x = x;
      c.y = y;
      c.visited = false;
      c.distance = 0xFFFF;
      c.parentX = -1;
      c.parentY = -1;
      for (uint8_t i = 0; i < 4; i++) {
        c.walls[i] = false;
      }
    }
  }
}

bool Maze::isValid(uint8_t x, uint8_t y) const {
  return x < kMazeSize && y < kMazeSize;
}

Cell &Maze::cell(uint8_t x, uint8_t y) { return grid_[x][y]; }

const Cell &Maze::cell(uint8_t x, uint8_t y) const { return grid_[x][y]; }

void Maze::setWall(uint8_t x, uint8_t y, Heading dir, bool hasWall) {
  if (!isValid(x, y)) return;
  grid_[x][y].walls[dir] = hasWall;
}

bool Maze::hasWall(uint8_t x, uint8_t y, Heading dir) const {
  if (!isValid(x, y)) return true;
  return grid_[x][y].walls[dir];
}

void Maze::setVisited(uint8_t x, uint8_t y, bool visited) {
  if (!isValid(x, y)) return;
  grid_[x][y].visited = visited;
}

void Maze::setParent(uint8_t x, uint8_t y, int8_t px, int8_t py) {
  if (!isValid(x, y)) return;
  grid_[x][y].parentX = px;
  grid_[x][y].parentY = py;
}

void Maze::resetDistances(uint16_t value) {
  for (uint8_t y = 0; y < kMazeSize; y++) {
    for (uint8_t x = 0; x < kMazeSize; x++) {
      grid_[x][y].distance = value;
      grid_[x][y].parentX = -1;
      grid_[x][y].parentY = -1;
    }
  }
}
