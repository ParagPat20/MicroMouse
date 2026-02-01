/*
 * HardwareTest - MicroMouse
 * Test Buttons (D5, D2), Encoders (L: D18,D19 | R: D27,D23), MPU6050
 * Output via Serial at 115200 baud.
 *
 * D2 = GPIO2 (BOOT on many ESP32 boards - may need external pull-up)
 * Plot mode (G): high-rate CSV for Serial Plotter
 *
 * Commands: R=run P=pause S=snapshot G=plot mode C=calibrate Z=zero ?=help
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ========== Pins (same as RightLineFollowerPro) ==========
#define BTN_MODE     5   // D5 - INPUT_PULLUP, LOW = pressed
#define BTN_MAP      33   // D2 - GPIO2 = BOOT on many ESP32, LOW = pressed
#define LEFT_ENC_A   18
#define LEFT_ENC_B   19
#define RIGHT_ENC_A  27
#define RIGHT_ENC_B  23

#define PRINT_INTERVAL_MS  200
#define PLOT_INTERVAL_MS   20   // 50 Hz for Serial Plotter

Adafruit_MPU6050 mpu;

volatile int32_t encLeftTicks = 0;
volatile int32_t encRightTicks = 0;
volatile uint32_t leftIsrCount = 0;   // Debug: ISR hit count
volatile uint32_t rightIsrCount = 0;
float gyroZOffset = 0.0f;
bool mpuOk = false;
unsigned long lastPrintMs = 0;
bool runContinuous = true;
bool plotMode = false;
bool plotHeaderDone = false;
bool rightEncRising = true;  // Try F to switch to FALLING if RISING doesn't work

void IRAM_ATTR isrLeftEnc() {
  leftIsrCount++;
  if (digitalRead(LEFT_ENC_B)) encLeftTicks--;
  else encLeftTicks++;
}
void IRAM_ATTR isrRightEnc() {
  rightIsrCount++;
  if (digitalRead(RIGHT_ENC_B)) encRightTicks++;
  else encRightTicks--;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(BTN_MODE, INPUT_PULLUP);
  pinMode(BTN_MAP, INPUT_PULLUP);
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), isrLeftEnc, RISING);
  // Right: try RISING first; if ISR_count stays 0 when wheel turns, try FALLING in code
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), isrRightEnc, RISING);

  mpuOk = mpu.begin();
  if (mpuOk) {
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    delay(100);
    // Quick calibration
    float sum = 0;
    for (int i = 0; i < 30; i++) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      sum += g.gyro.z;
      delay(30);
    }
    gyroZOffset = sum / 30.0f;
    Serial.println("MPU6050 OK, gyro Z offset calibrated.");
  } else {
    Serial.println("MPU6050 NOT FOUND (check I2C wiring).");
  }

  printHelp();
  Serial.println("\n--- HardwareTest Ready ---\n");
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    while (Serial.available()) Serial.read();
    if (c == 'S' || c == 's') {
      runContinuous = false;
      plotMode = false;
      printSnapshot();
    } else if (c == 'G' || c == 'g') {
      plotMode = true;
      runContinuous = false;
      plotHeaderDone = false;
      Serial.println("Plot mode ON (high-rate CSV for Serial Plotter). Send P to stop.");
    } else if (c == 'R' || c == 'r') {
      plotMode = false;
      runContinuous = true;
      Serial.println("Continuous read ON.");
    } else if (c == 'P' || c == 'p') {
      runContinuous = false;
      plotMode = false;
      Serial.println("Paused.");
    } else if (c == 'C' || c == 'c') {
      calibrateMPU();
    } else if (c == 'Z' || c == 'z') {
      encLeftTicks = 0;
      encRightTicks = 0;
      leftIsrCount = 0;
      rightIsrCount = 0;
      Serial.println("Encoders and ISR counts zeroed.");
    } else if (c == 'F' || c == 'f') {
      detachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A));
      rightEncRising = !rightEncRising;
      attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), isrRightEnc, rightEncRising ? RISING : FALLING);
      Serial.print("Right encoder edge: ");
      Serial.println(rightEncRising ? "RISING" : "FALLING");
    } else if (c == '?' || c == 'H' || c == 'h') {
      printHelp();
    }
  }

  if (plotMode) {
    if (!plotHeaderDone) {
      Serial.println("D2_raw,D5_raw,EncL,EncR,RightISR,GyroZ,AccZ");
      plotHeaderDone = true;
    }
    if (millis() - lastPrintMs >= PLOT_INTERVAL_MS) {
      lastPrintMs = millis();
      printPlotLine();
    }
  } else if (runContinuous && (millis() - lastPrintMs >= PRINT_INTERVAL_MS)) {
    lastPrintMs = millis();
    printSnapshot();
  }
}

void printSnapshot() {
  int d5Raw = digitalRead(BTN_MODE);
  int d2Raw = digitalRead(BTN_MAP);
  bool d5 = (d5Raw == LOW);
  bool d2 = (d2Raw == LOW);

  int leftA = digitalRead(LEFT_ENC_A);
  int leftB = digitalRead(LEFT_ENC_B);
  int rightA = digitalRead(RIGHT_ENC_A);
  int rightB = digitalRead(RIGHT_ENC_B);
  uint32_t leftIsr = leftIsrCount;
  uint32_t rightIsr = rightIsrCount;

  Serial.println("========================================");
  Serial.println("--- BUTTONS (LOW = pressed) ---");
  Serial.print("  D5 (GPIO5): raw=");
  Serial.print(d5Raw);
  Serial.print("  state=");
  Serial.println(d5 ? "PRESSED" : "released");
  Serial.print("  D2 (GPIO2): raw=");
  Serial.print(d2Raw);
  Serial.print("  state=");
  Serial.println(d2 ? "PRESSED" : "released");
  Serial.println("  (D2=BOOT on many ESP32; if raw stuck at 0/1 try external pull-up)");

  Serial.println("--- ENCODERS ---");
  Serial.print("  Left  A=");
  Serial.print(leftA);
  Serial.print(" B=");
  Serial.print(leftB);
  Serial.print("  ISR_count=");
  Serial.print(leftIsr);
  Serial.print("  ticks=");
  Serial.println(encLeftTicks);
  Serial.print("  Right A=");
  Serial.print(rightA);
  Serial.print(" B=");
  Serial.print(rightB);
  Serial.print("  ISR_count=");
  Serial.print(rightIsr);
  Serial.print("  ticks=");
  Serial.println(encRightTicks);
  Serial.println("  (If Right ISR_count=0 while wheel turns: check wiring A=27 B=23 or try swap A/B)");

  if (mpuOk) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    Serial.println("--- MPU6050 ---");
    Serial.print("  Accel X,Y,Z: ");
    Serial.print(a.acceleration.x);
    Serial.print(", ");
    Serial.print(a.acceleration.y);
    Serial.print(", ");
    Serial.println(a.acceleration.z);
    Serial.print("  Gyro X,Y,Z: ");
    Serial.print(g.gyro.x);
    Serial.print(", ");
    Serial.print(g.gyro.y);
    Serial.print(", ");
    Serial.println(g.gyro.z);
    Serial.print("  GyroZ-offset: ");
    Serial.println(g.gyro.z - gyroZOffset, 4);
    Serial.print("  Temp: ");
    Serial.println(temp.temperature);
  }
  Serial.println();
}

// High-rate CSV for Serial Plotter (send G to start)
void printPlotLine() {
  int d2Raw = digitalRead(BTN_MAP);
  int d5Raw = digitalRead(BTN_MODE);
  uint32_t rIsr = rightIsrCount;
  int32_t eL = encLeftTicks;
  int32_t eR = encRightTicks;
  float gyroZ = 0.0f, accZ = 0.0f;
  if (mpuOk) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyroZ = g.gyro.z - gyroZOffset;
    accZ = a.acceleration.z;
  }
  Serial.print(d2Raw);
  Serial.print(",");
  Serial.print(d5Raw);
  Serial.print(",");
  Serial.print(eL);
  Serial.print(",");
  Serial.print(eR);
  Serial.print(",");
  Serial.print(rIsr);
  Serial.print(",");
  Serial.print(gyroZ, 4);
  Serial.print(",");
  Serial.println(accZ, 2);
}

void calibrateMPU() {
  if (!mpuOk) {
    Serial.println("MPU6050 not available.");
    return;
  }
  Serial.println("Calibrating MPU6050 (keep robot still)...");
  float sum = 0;
  for (int i = 0; i < 50; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sum += g.gyro.z;
    delay(25);
  }
  gyroZOffset = sum / 50.0f;
  Serial.print("Gyro Z offset set to: ");
  Serial.println(gyroZOffset, 6);
}

void printHelp() {
  Serial.println("\n========== HardwareTest ==========");
  Serial.println("D5=GPIO5, D2=GPIO2 (LOW=pressed). D2 is BOOT on many ESP32.");
  Serial.println("Encoders: L A=18 B=19, R A=27 B=23. Snapshot shows raw A/B + ISR_count.");
  Serial.println("");
  Serial.println("Commands:");
  Serial.println("  R  - Run continuous (200ms)");
  Serial.println("  G  - Plot mode (50Hz CSV for Serial Plotter)");
  Serial.println("  P  - Pause");
  Serial.println("  S  - Single snapshot (debug D2 + Right encoder)");
  Serial.println("  C  - Calibrate MPU6050");
  Serial.println("  Z  - Zero encoders + ISR counts");
  Serial.println("  F  - Toggle Right encoder RISING/FALLING edge");
  Serial.println("  ?  - This help");
  Serial.println("==================================\n");
}
