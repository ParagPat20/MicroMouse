#include "Sensors.h"
#include <Arduino.h>

void Sensors::begin() {
  pinMode(SENSOR_RIGHT, INPUT);
  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_FRONT, INPUT);
  pinMode(SENSOR_CROSS_LEFT, INPUT);
  pinMode(SENSOR_CROSS_RIGHT, INPUT);
}

void Sensors::read() {
  readings_.right = analogRead(SENSOR_RIGHT);
  readings_.left = analogRead(SENSOR_LEFT);
  readings_.front = analogRead(SENSOR_FRONT);
  readings_.crossLeft = analogRead(SENSOR_CROSS_LEFT);
  readings_.crossRight = analogRead(SENSOR_CROSS_RIGHT);
}

SensorReadings Sensors::readings() const { return readings_; }

WallReading Sensors::detectWalls(Heading heading) const {
  WallReading walls = {false, false, false, false};

  bool frontWall = readings_.front > kObstacleDetected;
  bool rightWall = readings_.right > kObstacleDetected;
  bool leftWall = readings_.left > kObstacleDetected;

  switch (heading) {
    case kNorth:
      walls.north = frontWall;
      walls.east = rightWall;
      walls.west = leftWall;
      break;
    case kEast:
      walls.east = frontWall;
      walls.south = rightWall;
      walls.north = leftWall;
      break;
    case kSouth:
      walls.south = frontWall;
      walls.west = rightWall;
      walls.east = leftWall;
      break;
    case kWest:
      walls.west = frontWall;
      walls.north = rightWall;
      walls.south = leftWall;
      break;
  }

  return walls;
}
