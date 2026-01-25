/*
 * SensorRead - Simple sketch to read IR sensor distances and print them over
 * Serial Uses the same pin definitions as MMRF.ino for consistency.
 */

// ========== Pin Definitions (same as MMRF.ino) ==========
#define SENSOR_RIGHT 36
#define SENSOR_LEFT 39
#define SENSOR_FRONT 34
#define SENSOR_CROSS_LEFT 35
#define SENSOR_CROSS_RIGHT 32

void setup() {
  Serial.begin(115200);
  // Initialize sensor pins (optional, as analogRead works without pinMode)
  pinMode(SENSOR_RIGHT, INPUT);
  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_FRONT, INPUT);
  pinMode(SENSOR_CROSS_LEFT, INPUT);
  pinMode(SENSOR_CROSS_RIGHT, INPUT);
  Serial.println("SensorRead sketch started");
}

void loop() {
  // Read raw ADC values (0-4095 for ESP32)
  int rightVal = analogRead(SENSOR_RIGHT);
  int leftVal = analogRead(SENSOR_LEFT);
  int frontVal = analogRead(SENSOR_FRONT);
  int crossLeftVal = analogRead(SENSOR_CROSS_LEFT);
  int crossRightVal = analogRead(SENSOR_CROSS_RIGHT);

  // Print values in a single line for easy parsing
  // Print values left-to-right order: Left, Front, Right, Cross Left, Cross
  // Right
  Serial.print("L:");
  Serial.print(leftVal);
  Serial.print(" F:");
  Serial.print(frontVal);
  Serial.print(" R:");
  Serial.print(rightVal);
  Serial.print(" CL:");
  Serial.print(crossLeftVal);
  Serial.print(" CR:");
  Serial.println(crossRightVal);

  // Small non‑blocking pause – using millis would be overkill here; a short
  // delay keeps output readable
  delay(10);
}
