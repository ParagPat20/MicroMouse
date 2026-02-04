#ifndef MMRF_PATHFINDING_H
#define MMRF_PATHFINDING_H

#include <stdint.h>
#include "Maze.h"
#include "Types.h"

struct PathStep {
  uint8_t x;
  uint8_t y;
};

class Pathfinding {
 public:
  uint8_t dijkstra(Maze &maze, uint8_t startX, uint8_t startY, uint8_t goalX, uint8_t goalY,
                   PathStep *steps, uint8_t maxSteps);
};

#endif
