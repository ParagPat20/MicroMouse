/*
 * Maze Solving Robot - Improved Forward-Biased Navigation
 * ESP32 WROOM 30 Pin
 *
 * Hardware:
 * - Sharp IR Sensors (4-30cm): Pins 36, 39, 34, 32, 35
 * - TB6612FNG Motor Driver
 * - N20 Gear Motors 600RPM with 33mm wheels
 * - 6 NeoPixel LEDs: Pin 16
 * - 0.91" OLED I2C Display
 * - Buzzer: Pin 17
 */

#include <Adafruit_GFX.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

// ========== Pin Definitions ==========
// IR Sensors
#define SENSOR_RIGHT 36
#define SENSOR_LEFT 39
#define SENSOR_FRONT 34
#define SENSOR_CROSS_LEFT 35
#define SENSOR_CROSS_RIGHT 32

// Motor Driver TB6612FNG (L/R swapped based on testing)
// Left motor (was R)
#define MOTOR_L_PWM 26
#define MOTOR_L_IN1 13
#define MOTOR_L_IN2 15
// Right motor (was L)
#define MOTOR_R_PWM 25
#define MOTOR_R_IN1 14
#define MOTOR_R_IN2 12
#define MOTOR_STBY 4

// Buzzer
#define BUZZER_PIN 17

// NeoPixel
#define NEOPIXEL_PIN 16
#define NEOPIXEL_COUNT 6

// OLED Display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1

// ========== Sensor Calibration ==========
// Distance thresholds (in ADC values, 100-4000)
#define OBSTACLE_CRITICAL 2500 // Very close - must brake hard (< 6cm)
#define OBSTACLE_NEAR 1800     // Close obstacle - brake (~10cm)
#define OBSTACLE_DETECTED 1000 // Wall/obstacle exists (~20cm)
#define WALL_LOST 600          // No wall detected (>25cm)

// Front sensor thresholds
#define FRONT_WALL_DANGER 2800 // Emergency brake! (< 5cm)
#define FRONT_WALL_STOP 2200   // Must brake and turn (~8cm)
#define FRONT_WALL_SLOW 1500   // Slow down (~12cm)
#define FRONT_WALL_CRAWL 1200  // Very slow (~15cm)

// Side balance thresholds
#define SIDE_TOO_CLOSE 2800   // Way too close to side wall
#define SIDE_BALANCE_DIFF 800 // Max difference for going straight

// ========== Motor Speed Settings ==========
#define SPEED_FORWARD 180 // Normal forward speed (reduced from 190)
#define SPEED_SLOW 120    // Slow approach speed (reduced from 140)
#define SPEED_CRAWL 80    // Very slow near walls
#define SPEED_TURN 160    // Turning speed (reduced from 170)

// ========== Algorithm Parameters ==========
#define TURN_DELAY_90 480     // ms for 90-degree turn
#define TURN_DELAY_SEARCH 250 // ms for searching turn
#define BRAKE_TIME 120        // ms for emergency brake
#define MIN_FORWARD_TIME 300  // Min time to go forward after turn

// ========== Buzzer Tones ==========
#define TONE_STARTUP 1000  // Hz
#define TONE_TURN 800      // Hz
#define TONE_OBSTACLE 1200 // Hz
#define TONE_STUCK 600     // Hz

// ========== Global Objects ==========
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// ========== Global Variables ==========
int rightSensor = 0;
int leftSensor = 0;
int frontSensor = 0;
int crossLeftSensor = 0;
int crossRightSensor = 0;

unsigned long lastTurnTime = 0;
unsigned long stuckStartTime = 0;
bool isStuck = false;
int currentSpeed = 0; // Track current speed for smooth transitions

enum RobotState {
  STATE_FORWARD,
  STATE_TURN_LEFT,
  STATE_TURN_RIGHT,
  STATE_SEARCH_TURN,
  STATE_BRAKING,
  STATE_STOP
};

RobotState currentState = STATE_FORWARD;
String lastAction = "Init";

// ========== Setup Function ==========
void setup() {
  Serial.begin(115200);

  // Initialize Motor Pins
  pinMode(MOTOR_L_PWM, OUTPUT);
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_PWM, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);
  pinMode(MOTOR_STBY, OUTPUT);

  // Initialize Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Enable motor driver
  digitalWrite(MOTOR_STBY, HIGH);

  // Initialize Sensor Pins
  pinMode(SENSOR_RIGHT, INPUT);
  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_FRONT, INPUT);
  pinMode(SENSOR_CROSS_LEFT, INPUT);
  pinMode(SENSOR_CROSS_RIGHT, INPUT);

  // Initialize NeoPixel
  pixels.begin();
  pixels.setBrightness(50);
  setNeoPixelColor(0, 0, 255); // Blue on startup
  pixels.show();

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Maze Solver V2");
  display.println("Forward-Biased");
  display.println("Enhanced Safety");
  display.display();

  // Startup sound
  beep(TONE_STARTUP, 100);
  delay(100);
  beep(TONE_STARTUP, 100);

  delay(1500);

  Serial.println("=== Maze Solver Ready ===");
  setNeoPixelColor(0, 255, 0); // Green - ready
  pixels.show();
  lastTurnTime = millis();
}

// ========== Main Loop ==========
void loop() {
  // Read all sensors
  readSensors();

  // Update display
  updateDisplay();

  // Main navigation logic
  navigate();

  // Small delay for stability
  delay(20);
}

// ========== Sensor Reading ==========
void readSensors() {
  // Read raw values
  int rawRight = analogRead(SENSOR_RIGHT);
  int rawLeft = analogRead(SENSOR_LEFT);
  int rawFront = analogRead(SENSOR_FRONT);
  int rawCrossLeft = analogRead(SENSOR_CROSS_LEFT);
  int rawCrossRight = analogRead(SENSOR_CROSS_RIGHT);

  // Simple 3-sample moving average
  static int rightHist[3] = {0};
  static int leftHist[3] = {0};
  static int frontHist[3] = {0};
  static int crossLHist[3] = {0};
  static int crossRHist[3] = {0};

  // Shift history
  rightHist[2] = rightHist[1];
  rightHist[1] = rightHist[0];
  rightHist[0] = rawRight;
  leftHist[2] = leftHist[1];
  leftHist[1] = leftHist[0];
  leftHist[0] = rawLeft;
  frontHist[2] = frontHist[1];
  frontHist[1] = frontHist[0];
  frontHist[0] = rawFront;
  crossLHist[2] = crossLHist[1];
  crossLHist[1] = crossLHist[0];
  crossLHist[0] = rawCrossLeft;
  crossRHist[2] = crossRHist[1];
  crossRHist[1] = crossRHist[0];
  crossRHist[0] = rawCrossRight;

  // Average
  rightSensor = (rightHist[0] + rightHist[1] + rightHist[2]) / 3;
  leftSensor = (leftHist[0] + leftHist[1] + leftHist[2]) / 3;
  frontSensor = (frontHist[0] + frontHist[1] + frontHist[2]) / 3;
  crossLeftSensor = (crossLHist[0] + crossLHist[1] + crossLHist[2]) / 3;
  crossRightSensor = (crossRHist[0] + crossRHist[1] + crossRHist[2]) / 3;

  // Debug output
  Serial.print("F:");
  Serial.print(frontSensor);
  Serial.print(" R:");
  Serial.print(rightSensor);
  Serial.print(" L:");
  Serial.print(leftSensor);
  Serial.print(" CR:");
  Serial.print(crossRightSensor);
  Serial.print(" CL:");
  Serial.print(crossLeftSensor);
  Serial.print(" | ");
  Serial.println(lastAction);
}

// ========== Main Navigation Logic ==========
void navigate() {
  unsigned long timeSinceLastTurn = millis() - lastTurnTime;

  // PRIORITY 0: EMERGENCY BRAKE - Front wall extremely close!
  if (frontSensor > FRONT_WALL_DANGER) {
    lastAction = "EMERGENCY!";
    currentState = STATE_BRAKING;
    setNeoPixelColor(255, 0, 0); // Red alert
    beep(TONE_OBSTACLE, 50);
    emergencyBrake();
    delay(200);
    handleFrontWall();
    return;
  }

  // PRIORITY 1: Check if we need to stop (both sides critically close)
  if (leftSensor > SIDE_TOO_CLOSE && rightSensor > SIDE_TOO_CLOSE) {
    if (!isStuck) {
      stuckStartTime = millis();
      isStuck = true;
    }

    if (millis() - stuckStartTime > 500) {
      // Stuck for too long - back up and turn
      lastAction = "Stuck! Backing";
      beep(TONE_STUCK, 200);
      backupAndTurn();
      isStuck = false;
      return;
    }
  } else {
    isStuck = false;
  }

  // PRIORITY 2: Check front obstacle (MUST TURN with braking)
  if (frontSensor > FRONT_WALL_STOP || crossLeftSensor > OBSTACLE_CRITICAL ||
      crossRightSensor > OBSTACLE_CRITICAL) {
    lastAction = "Front Wall!";
    currentState = STATE_BRAKING;
    setNeoPixelColor(255, 100, 0); // Orange
    beep(TONE_OBSTACLE, 80);
    softBrake();
    handleFrontWall();
    return;
  }

  // PRIORITY 3: Gradual speed reduction based on front distance
  if (frontSensor > FRONT_WALL_SLOW) {
    lastAction = "Slowing";
    currentState = STATE_FORWARD;
    setNeoPixelColor(255, 255, 0); // Yellow

    // Progressive speed reduction
    if (frontSensor > FRONT_WALL_SLOW + 300) {
      smoothSpeedChange(SPEED_CRAWL);
    } else {
      smoothSpeedChange(SPEED_SLOW);
    }
    return;
  } else if (frontSensor > FRONT_WALL_CRAWL) {
    lastAction = "Crawling";
    currentState = STATE_FORWARD;
    setNeoPixelColor(255, 200, 0); // Yellow-orange
    smoothSpeedChange(SPEED_CRAWL);
    return;
  }

  // PRIORITY 4: Check side walls - ONLY turn if getting too close (collision
  // avoidance) Only check for turns if enough time has passed since last turn
  if (timeSinceLastTurn > MIN_FORWARD_TIME) {

    // Check if getting too close to right wall (avoid collision)
    if (rightSensor > OBSTACLE_NEAR || crossRightSensor > OBSTACLE_NEAR) {
      lastAction = "Right Close!";
      currentState = STATE_TURN_LEFT;
      setNeoPixelColor(255, 165, 0); // Orange
      beep(TONE_TURN, 60);
      softBrake();
      turnLeft90();
      lastTurnTime = millis();
      return;
    }

    // Check if getting too close to left wall (avoid collision)
    if (leftSensor > OBSTACLE_NEAR || crossLeftSensor > OBSTACLE_NEAR) {
      lastAction = "Left Close!";
      currentState = STATE_TURN_RIGHT;
      setNeoPixelColor(255, 165, 0); // Orange
      beep(TONE_TURN, 60);
      softBrake();
      turnRight90();
      lastTurnTime = millis();
      return;
    }
  }

  // PRIORITY 5: GO STRAIGHT (default behavior)
  lastAction = "Forward";
  currentState = STATE_FORWARD;
  setNeoPixelColor(0, 255, 0); // Green

  // Check if we need minor adjustment while going forward
  int sideDifference = abs(leftSensor - rightSensor);

  if (sideDifference > SIDE_BALANCE_DIFF) {
    // Adjust slightly to center
    if (leftSensor > rightSensor) {
      // Too close to left, drift right
      smoothMoveForwardBiased(SPEED_FORWARD, SPEED_FORWARD - 30);
    } else {
      // Too close to right, drift left
      smoothMoveForwardBiased(SPEED_FORWARD - 30, SPEED_FORWARD);
    }
  } else {
    // Balanced - go straight
    smoothSpeedChange(SPEED_FORWARD);
  }
}

// ========== Handle Front Wall ==========
void handleFrontWall() {
  stopMotors();
  delay(100);

  // Check which side has more space
  int leftSpace = (leftSensor + crossLeftSensor) / 2;
  int rightSpace = (rightSensor + crossRightSensor) / 2;

  Serial.print("Left space: ");
  Serial.print(leftSpace);
  Serial.print(" | Right space: ");
  Serial.println(rightSpace);

  if (leftSpace < rightSpace) {
    // More space on left - turn left
    lastAction = "Turn Left";
    currentState = STATE_TURN_LEFT;
    setNeoPixelColor(255, 0, 0); // Red
    beep(TONE_TURN, 100);
    turnLeft90();
  } else {
    // More space on right - turn right
    lastAction = "Turn Right";
    currentState = STATE_TURN_RIGHT;
    setNeoPixelColor(255, 0, 0); // Red
    beep(TONE_TURN, 100);
    turnRight90();
  }

  lastTurnTime = millis();
}

// ========== Backup and Turn (for stuck situations) ==========
void backupAndTurn() {
  // Backup
  digitalWrite(MOTOR_L_IN1, HIGH);
  digitalWrite(MOTOR_L_IN2, LOW);
  analogWrite(MOTOR_L_PWM, SPEED_SLOW);

  digitalWrite(MOTOR_R_IN1, HIGH);
  digitalWrite(MOTOR_R_IN2, LOW);
  analogWrite(MOTOR_R_PWM, SPEED_SLOW);

  delay(400);
  stopMotors();
  delay(100);

  // Single 90-degree turn (choose direction with more space)
  int leftSpace = (leftSensor + crossLeftSensor) / 2;
  int rightSpace = (rightSensor + crossRightSensor) / 2;

  beep(TONE_TURN, 80);
  if (leftSpace < rightSpace) {
    turnLeft90();
  } else {
    turnRight90();
  }

  lastTurnTime = millis();
}

// ========== Motor Control Functions with Smooth Transitions ==========
void smoothSpeedChange(int targetSpeed) {
  // Gradually change speed to avoid jerky movements
  if (currentSpeed < targetSpeed) {
    currentSpeed = min(currentSpeed + 10, targetSpeed);
  } else if (currentSpeed > targetSpeed) {
    currentSpeed = max(currentSpeed - 10, targetSpeed);
  }

  moveForward(currentSpeed);
}

void smoothMoveForwardBiased(int leftSpeed, int rightSpeed) {
  // Apply smooth speed changes
  static int currentLeftSpeed = 0;
  static int currentRightSpeed = 0;

  if (currentLeftSpeed < leftSpeed) {
    currentLeftSpeed = min(currentLeftSpeed + 10, leftSpeed);
  } else if (currentLeftSpeed > leftSpeed) {
    currentLeftSpeed = max(currentLeftSpeed - 10, leftSpeed);
  }

  if (currentRightSpeed < rightSpeed) {
    currentRightSpeed = min(currentRightSpeed + 10, rightSpeed);
  } else if (currentRightSpeed > rightSpeed) {
    currentRightSpeed = max(currentRightSpeed - 10, rightSpeed);
  }

  moveForwardBiased(currentLeftSpeed, currentRightSpeed);
}

void moveForward(int speed) {
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, HIGH);
  analogWrite(MOTOR_L_PWM, speed);

  digitalWrite(MOTOR_R_IN1, LOW);
  digitalWrite(MOTOR_R_IN2, HIGH);
  analogWrite(MOTOR_R_PWM, speed);
}

void moveForwardBiased(int leftSpeed, int rightSpeed) {
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, HIGH);
  analogWrite(MOTOR_L_PWM, leftSpeed);

  digitalWrite(MOTOR_R_IN1, LOW);
  digitalWrite(MOTOR_R_IN2, HIGH);
  analogWrite(MOTOR_R_PWM, rightSpeed);
}

void emergencyBrake() {
  // Reverse motors briefly for instant stop
  digitalWrite(MOTOR_L_IN1, HIGH);
  digitalWrite(MOTOR_L_IN2, LOW);
  analogWrite(MOTOR_L_PWM, 255);

  digitalWrite(MOTOR_R_IN1, HIGH);
  digitalWrite(MOTOR_R_IN2, LOW);
  analogWrite(MOTOR_R_PWM, 255);

  delay(BRAKE_TIME);
  stopMotors();
  currentSpeed = 0;
}

void softBrake() {
  // Gradual deceleration
  for (int speed = currentSpeed; speed > 0; speed -= 20) {
    analogWrite(MOTOR_L_PWM, speed);
    analogWrite(MOTOR_R_PWM, speed);
    delay(15);
  }
  stopMotors();
  currentSpeed = 0;
}

void turnLeft90() {
  stopMotors();
  delay(50);

  // Smooth acceleration into turn
  for (int speed = 80; speed <= SPEED_TURN; speed += 20) {
    digitalWrite(MOTOR_L_IN1, HIGH);
    digitalWrite(MOTOR_L_IN2, LOW);
    analogWrite(MOTOR_L_PWM, speed);

    digitalWrite(MOTOR_R_IN1, LOW);
    digitalWrite(MOTOR_R_IN2, HIGH);
    analogWrite(MOTOR_R_PWM, speed);
    delay(20);
  }

  // Main turn at constant speed
  digitalWrite(MOTOR_L_IN1, HIGH);
  digitalWrite(MOTOR_L_IN2, LOW);
  analogWrite(MOTOR_L_PWM, SPEED_TURN);

  digitalWrite(MOTOR_R_IN1, LOW);
  digitalWrite(MOTOR_R_IN2, HIGH);
  analogWrite(MOTOR_R_PWM, SPEED_TURN);

  delay(TURN_DELAY_90 - 100); // Reduced to account for accel/decel

  // Smooth deceleration
  for (int speed = SPEED_TURN; speed >= 0; speed -= 30) {
    analogWrite(MOTOR_L_PWM, speed);
    analogWrite(MOTOR_R_PWM, speed);
    delay(15);
  }

  stopMotors();
  currentSpeed = 0;
  delay(100);
}

void turnRight90() {
  stopMotors();
  delay(50);

  // Smooth acceleration into turn
  for (int speed = 80; speed <= SPEED_TURN; speed += 20) {
    digitalWrite(MOTOR_L_IN1, LOW);
    digitalWrite(MOTOR_L_IN2, HIGH);
    analogWrite(MOTOR_L_PWM, speed);

    digitalWrite(MOTOR_R_IN1, HIGH);
    digitalWrite(MOTOR_R_IN2, LOW);
    analogWrite(MOTOR_R_PWM, speed);
    delay(20);
  }

  // Main turn at constant speed
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, HIGH);
  analogWrite(MOTOR_L_PWM, SPEED_TURN);

  digitalWrite(MOTOR_R_IN1, HIGH);
  digitalWrite(MOTOR_R_IN2, LOW);
  analogWrite(MOTOR_R_PWM, SPEED_TURN);

  delay(TURN_DELAY_90 - 100); // Reduced to account for accel/decel

  // Smooth deceleration
  for (int speed = SPEED_TURN; speed >= 0; speed -= 30) {
    analogWrite(MOTOR_L_PWM, speed);
    analogWrite(MOTOR_R_PWM, speed);
    delay(15);
  }

  stopMotors();
  currentSpeed = 0;
  delay(100);
}

void stopMotors() {
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, LOW);
  analogWrite(MOTOR_L_PWM, 0);

  digitalWrite(MOTOR_R_IN1, LOW);
  digitalWrite(MOTOR_R_IN2, LOW);
  analogWrite(MOTOR_R_PWM, 0);
}

// ========== Buzzer Control ==========
void beep(int frequency, int duration) {
  tone(BUZZER_PIN, frequency, duration);
  delay(duration);
  noTone(BUZZER_PIN);
}

// ========== NeoPixel Control ==========
void setNeoPixelColor(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NEOPIXEL_COUNT; i++) {
    pixels.setPixelColor(i, pixels.Color(r, g, b));
  }
  pixels.show();
}

// ========== OLED Display Update ==========
void updateDisplay() {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate < 200)
    return;
  lastUpdate = millis();

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);

  // Display action
  display.print("Act: ");
  display.println(lastAction);

  // Display sensor values
  display.print("F:");
  display.print(frontSensor);
  display.print(" R:");
  display.print(rightSensor);
  display.print(" L:");
  display.println(leftSensor);

  // Display cross sensors
  display.print("CR:");
  display.print(crossRightSensor);
  display.print(" CL:");
  display.println(crossLeftSensor);

  display.display();
}