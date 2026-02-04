#ifndef MMRF_SENSORS_H
#define MMRF_SENSORS_H

#include <stdint.h>
#include "Config.h"
#include "Pins.h"
#include "Types.h"

struct SensorReadings {
  int left;
  int right;
  int front;
  int crossLeft;
  int crossRight;
};

struct WallReading {
  bool north;
  bool east;
  bool south;
  bool west;
};

class Sensors {
 public:
  void begin();
  void read();
  SensorReadings readings() const;
  WallReading detectWalls(Heading heading) const;

 private:
  SensorReadings readings_ = {0, 0, 0, 0, 0};
};

#endif
