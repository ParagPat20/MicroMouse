#include "Odometry.h"
#include <Arduino.h>

volatile int32_t Odometry::leftTicks_ = 0;
volatile int32_t Odometry::rightTicks_ = 0;

void IRAM_ATTR Odometry::isrLeft() {
  if (digitalRead(LEFT_ENC_B)) {
    leftTicks_--;
  } else {
    leftTicks_++;
  }
}

void IRAM_ATTR Odometry::isrRight() {
  if (digitalRead(RIGHT_ENC_B)) {
    rightTicks_++;
  } else {
    rightTicks_--;
  }
}

void Odometry::begin() {
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), isrLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), isrRight, RISING);

  mpuOk_ = mpu_.begin();
  if (mpuOk_) {
    mpu_.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu_.setFilterBandwidth(MPU6050_BAND_21_HZ);
    delay(100);
    float sum = 0.0f;
    for (int i = 0; i < 30; i++) {
      sensors_event_t a, g, temp;
      mpu_.getEvent(&a, &g, &temp);
      sum += g.gyro.z;
      delay(20);
    }
    gyroZOffset_ = sum / 30.0f;
  }
}

void Odometry::update() {
  int32_t left = leftTicks_;
  int32_t right = rightTicks_;
  int32_t deltaLeft = left - lastLeftTicks_;
  int32_t deltaRight = right - lastRightTicks_;

  lastLeftTicks_ = left;
  lastRightTicks_ = right;

  float leftDist = (deltaLeft / (float)ticksPerRev_) * kWheelCircumferenceCm;
  float rightDist = (deltaRight / (float)ticksPerRev_) * kWheelCircumferenceCm;
  float avgDist = (leftDist + rightDist) * 0.5f;

  float encoderHeading = (rightDist - leftDist) / kWheelBaseCm;
  float gyroHeading = 0.0f;

  if (mpuOk_) {
    sensors_event_t a, g, temp;
    mpu_.getEvent(&a, &g, &temp);
    gyroHeading = (g.gyro.z - gyroZOffset_) * 0.02f;
  }

  headingRad_ = kEncoderHeadingWeight * (headingRad_ + encoderHeading) +
               kGyroHeadingWeight * (headingRad_ + gyroHeading);
  distanceCm_ += avgDist;
}

void Odometry::reset() {
  leftTicks_ = 0;
  rightTicks_ = 0;
  lastLeftTicks_ = 0;
  lastRightTicks_ = 0;
  distanceCm_ = 0.0f;
  headingRad_ = 0.0f;
}

void Odometry::setTicksPerRev(int32_t ticks) { ticksPerRev_ = ticks; }

float Odometry::headingRadians() const { return headingRad_; }

float Odometry::headingDegrees() const { return headingRad_ * 180.0f / 3.14159f; }

float Odometry::distanceCm() const { return distanceCm_; }

int32_t Odometry::leftTicks() const { return leftTicks_; }

int32_t Odometry::rightTicks() const { return rightTicks_; }
