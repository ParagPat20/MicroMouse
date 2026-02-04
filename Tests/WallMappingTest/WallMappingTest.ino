#include <Arduino.h>
#include "../../MMRF/Maze.h"
#include "../../MMRF/Sensors.h"
#include "../../MMRF/Types.h"

Maze maze;
Sensors sensors;
Heading heading = kNorth;

void setup() {
  Serial.begin(115200);
  delay(300);
  sensors.begin();
  Serial.println("Wall Mapping Test Ready");
}

void loop() {
  sensors.read();
  WallReading walls = sensors.detectWalls(heading);

  maze.setWall(0, 0, kNorth, walls.north);
  maze.setWall(0, 0, kEast, walls.east);
  maze.setWall(0, 0, kSouth, walls.south);
  maze.setWall(0, 0, kWest, walls.west);

  Serial.print("Heading:");
  Serial.print(heading);
  Serial.print(" Walls N/E/S/W:");
  Serial.print(walls.north);
  Serial.print("/");
  Serial.print(walls.east);
  Serial.print("/");
  Serial.print(walls.south);
  Serial.print("/");
  Serial.println(walls.west);

  delay(300);
}
