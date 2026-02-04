#include <Arduino.h>
#include "../../MMRF/Config.h"
#include "../../MMRF/Odometry.h"

Odometry odometry;

void setup() {
  Serial.begin(115200);
  delay(300);
  odometry.begin();
  odometry.setTicksPerRev(kDefaultTicksPerRev);
  Serial.println("Odometry Test Ready");
}

void loop() {
  odometry.update();
  Serial.print("L:");
  Serial.print(odometry.leftTicks());
  Serial.print(" R:");
  Serial.print(odometry.rightTicks());
  Serial.print(" Dist(cm):");
  Serial.print(odometry.distanceCm(), 2);
  Serial.print(" Heading(deg):");
  Serial.println(odometry.headingDegrees(), 2);
  delay(200);
}
