#include "Motion.h"
#include <Arduino.h>

void Motion::begin() {
  pinMode(MOTOR_L_PWM, OUTPUT);
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_PWM, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);
  pinMode(MOTOR_STBY, OUTPUT);
  digitalWrite(MOTOR_STBY, HIGH);
}

void Motion::stop() { setMotors(0, 0); }

void Motion::moveForward(int speed) { setMotors(speed, speed); }

void Motion::turnLeft(int speed) { setMotors(-speed, speed); }

void Motion::turnRight(int speed) { setMotors(speed, -speed); }

void Motion::moveForwardOneCell(Odometry &odometry) {
  float startDistance = odometry.distanceCm();
  moveForward(kSpeedCruise);
  while (odometry.distanceCm() - startDistance < kCellSizeCm) {
    odometry.update();
    delay(5);
  }
  stop();
}

void Motion::turnToHeading(Odometry &odometry, Heading target) {
  float targetDeg = target * 90.0f;
  float heading = odometry.headingDegrees();
  float delta = targetDeg - heading;

  if (delta > 180.0f) delta -= 360.0f;
  if (delta < -180.0f) delta += 360.0f;

  if (delta > 0) {
    turnRight(kSpeedTurn);
  } else {
    turnLeft(kSpeedTurn);
  }

  while (abs(odometry.headingDegrees() - targetDeg) > 4.0f) {
    odometry.update();
    delay(5);
  }
  stop();
}

void Motion::setMotors(int leftSpeed, int rightSpeed) {
  if (leftSpeed >= 0) {
    digitalWrite(MOTOR_L_IN1, LOW);
    digitalWrite(MOTOR_L_IN2, HIGH);
    analogWrite(MOTOR_L_PWM, leftSpeed);
  } else {
    digitalWrite(MOTOR_L_IN1, HIGH);
    digitalWrite(MOTOR_L_IN2, LOW);
    analogWrite(MOTOR_L_PWM, -leftSpeed);
  }

  if (rightSpeed >= 0) {
    digitalWrite(MOTOR_R_IN1, LOW);
    digitalWrite(MOTOR_R_IN2, HIGH);
    analogWrite(MOTOR_R_PWM, rightSpeed);
  } else {
    digitalWrite(MOTOR_R_IN1, HIGH);
    digitalWrite(MOTOR_R_IN2, LOW);
    analogWrite(MOTOR_R_PWM, -rightSpeed);
  }
}
