#ifndef MMRF_ODOMETRY_H
#define MMRF_ODOMETRY_H

#include <stdint.h>
#include <Adafruit_MPU6050.h>
#include "Config.h"
#include "Pins.h"

class Odometry {
 public:
  void begin();
  void update();
  void reset();
  void setTicksPerRev(int32_t ticks);
  float headingRadians() const;
  float headingDegrees() const;
  float distanceCm() const;
  int32_t leftTicks() const;
  int32_t rightTicks() const;

 private:
  static void IRAM_ATTR isrLeft();
  static void IRAM_ATTR isrRight();

  static volatile int32_t leftTicks_;
  static volatile int32_t rightTicks_;

  Adafruit_MPU6050 mpu_;
  bool mpuOk_ = false;
  float gyroZOffset_ = 0.0f;
  float headingRad_ = 0.0f;
  float distanceCm_ = 0.0f;
  int32_t ticksPerRev_ = kDefaultTicksPerRev;
  int32_t lastLeftTicks_ = 0;
  int32_t lastRightTicks_ = 0;
};

#endif
