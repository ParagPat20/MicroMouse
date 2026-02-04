#ifndef MMRF_MAZE_H
#define MMRF_MAZE_H

#include <stdint.h>
#include "Config.h"
#include "Types.h"

struct Cell {
  uint8_t x;
  uint8_t y;
  bool walls[4];
  bool visited;
  uint16_t distance;
  int8_t parentX;
  int8_t parentY;
};

class Maze {
 public:
  Maze();
  void reset();
  bool isValid(uint8_t x, uint8_t y) const;
  Cell &cell(uint8_t x, uint8_t y);
  const Cell &cell(uint8_t x, uint8_t y) const;
  void setWall(uint8_t x, uint8_t y, Heading dir, bool hasWall);
  bool hasWall(uint8_t x, uint8_t y, Heading dir) const;
  void setVisited(uint8_t x, uint8_t y, bool visited);
  void setParent(uint8_t x, uint8_t y, int8_t px, int8_t py);
  void resetDistances(uint16_t value = 0xFFFF);

 private:
  Cell grid_[kMazeSize][kMazeSize];
};

#endif
