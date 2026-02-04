#ifndef MMRF_MOTION_H
#define MMRF_MOTION_H

#include <stdint.h>
#include "Config.h"
#include "Pins.h"
#include "Odometry.h"
#include "Types.h"

class Motion {
 public:
  void begin();
  void stop();
  void moveForward(int speed);
  void turnLeft(int speed);
  void turnRight(int speed);
  void moveForwardOneCell(Odometry &odometry);
  void turnToHeading(Odometry &odometry, Heading target);

 private:
  void setMotors(int leftSpeed, int rightSpeed);
};

#endif
