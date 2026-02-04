#ifndef MMRF_CONFIG_H
#define MMRF_CONFIG_H

#include <stdint.h>

// Maze configuration
static const uint8_t kMazeSize = 16;
static const float kCellSizeCm = 30.0f;

// Robot geometry
static const float kWheelDiameterCm = 4.0f;
static const float kWheelBaseCm = 13.5f;
static const float kWheelCircumferenceCm = 3.14159f * kWheelDiameterCm;

// Encoders
static const int32_t kDefaultTicksPerRev = 12;

// Wall detection thresholds
static const int kObstacleCritical = 2500;
static const int kObstacleNear = 1800;
static const int kObstacleDetected = 1000;
static const int kWallLost = 600;

// Complementary filter weight
static const float kEncoderHeadingWeight = 0.95f;
static const float kGyroHeadingWeight = 0.05f;

// Motion defaults
static const int kSpeedCruise = 180;
static const int kSpeedTurn = 160;

#endif
