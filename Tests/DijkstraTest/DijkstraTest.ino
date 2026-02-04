#include <Arduino.h>
#include "../../MMRF/Maze.h"
#include "../../MMRF/Pathfinding.h"

Maze maze;
Pathfinding pathfinding;

void setup() {
  Serial.begin(115200);
  delay(300);

  // Sample 4x4 maze with a blocked wall between (1,0) and (1,1)
  maze.reset();
  maze.setWall(1, 0, kSouth, true);
  maze.setWall(1, 1, kNorth, true);

  PathStep steps[32];
  uint8_t count = pathfinding.dijkstra(maze, 0, 0, 3, 3, steps, 32);

  Serial.print("Path length: ");
  Serial.println(count);
  for (uint8_t i = 0; i < count; i++) {
    Serial.print("(");
    Serial.print(steps[i].x);
    Serial.print(",");
    Serial.print(steps[i].y);
    Serial.println(")");
  }
}

void loop() {}
