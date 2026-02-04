#include "Pathfinding.h"

struct Node {
  uint8_t x;
  uint8_t y;
  uint16_t dist;
  bool visited;
};

uint8_t Pathfinding::dijkstra(Maze &maze, uint8_t startX, uint8_t startY, uint8_t goalX,
                              uint8_t goalY, PathStep *steps, uint8_t maxSteps) {
  maze.resetDistances();

  Node nodes[kMazeSize][kMazeSize];
  for (uint8_t y = 0; y < kMazeSize; y++) {
    for (uint8_t x = 0; x < kMazeSize; x++) {
      nodes[x][y] = {x, y, 0xFFFF, false};
    }
  }

  nodes[startX][startY].dist = 0;
  maze.cell(startX, startY).distance = 0;

  for (uint16_t iter = 0; iter < kMazeSize * kMazeSize; iter++) {
    uint16_t bestDist = 0xFFFF;
    int8_t bestX = -1;
    int8_t bestY = -1;

    for (uint8_t y = 0; y < kMazeSize; y++) {
      for (uint8_t x = 0; x < kMazeSize; x++) {
        if (!nodes[x][y].visited && nodes[x][y].dist < bestDist) {
          bestDist = nodes[x][y].dist;
          bestX = x;
          bestY = y;
        }
      }
    }

    if (bestX < 0) break;
    nodes[bestX][bestY].visited = true;
    if (bestX == goalX && bestY == goalY) break;

    Heading dirs[4] = {kNorth, kEast, kSouth, kWest};
    int8_t dx[4] = {0, 1, 0, -1};
    int8_t dy[4] = {-1, 0, 1, 0};

    for (uint8_t i = 0; i < 4; i++) {
      uint8_t nx = bestX + dx[i];
      uint8_t ny = bestY + dy[i];
      if (!maze.isValid(nx, ny)) continue;
      if (maze.hasWall(bestX, bestY, dirs[i])) continue;

      uint16_t alt = nodes[bestX][bestY].dist + 1;
      if (alt < nodes[nx][ny].dist) {
        nodes[nx][ny].dist = alt;
        maze.cell(nx, ny).distance = alt;
        maze.setParent(nx, ny, bestX, bestY);
      }
    }
  }

  uint8_t pathCount = 0;
  int8_t cx = goalX;
  int8_t cy = goalY;
  while (cx >= 0 && cy >= 0 && pathCount < maxSteps) {
    steps[pathCount++] = {static_cast<uint8_t>(cx), static_cast<uint8_t>(cy)};
    if (cx == startX && cy == startY) break;
    Cell &cell = maze.cell(cx, cy);
    int8_t px = cell.parentX;
    int8_t py = cell.parentY;
    if (px < 0 || py < 0) break;
    cx = px;
    cy = py;
  }

  for (uint8_t i = 0; i < pathCount / 2; i++) {
    PathStep temp = steps[i];
    steps[i] = steps[pathCount - 1 - i];
    steps[pathCount - 1 - i] = temp;
  }

  return pathCount;
}
