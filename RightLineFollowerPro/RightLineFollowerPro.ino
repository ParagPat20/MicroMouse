/*
 * Right Wall Follower PRO - MicroMouse
 * 
 * Algorithm: Adaptive PID + Corner Detection (from RightLineFollower)
 * Visual Feedback: NeoPixels + OLED Display (from MMRF)
 * 
 * Features:
 * - Dual-mode PID (Wall: KP=0.13, KD=1.8 | Turn: KP=2.7, KD=1.3)
 * - External corner detection with timed turns
 * - Real-time OLED display showing state and sensor values
 * - NeoPixel color feedback for different modes
 * - Full serial tuning support
 */

#include <Adafruit_GFX.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

// ========== Pin Definitions ==========
// Sensors
#define SENSOR_RIGHT 36
#define SENSOR_LEFT 39
#define SENSOR_FRONT 34
#define SENSOR_CROSS_LEFT 35
#define SENSOR_CROSS_RIGHT 32

// Motors (corrected mapping)
#define MOTOR_L_PWM 26
#define MOTOR_L_IN1 13
#define MOTOR_L_IN2 15
#define MOTOR_R_PWM 25
#define MOTOR_R_IN1 14
#define MOTOR_R_IN2 12
#define MOTOR_STBY 4

// Visual feedback
#define BUZZER_PIN 17
#define NEOPIXEL_PIN 16
#define NEOPIXEL_COUNT 6

// OLED Display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1

// ========== Tunable Parameters ==========

// Right wall following setpoints
int RIGHT_SETPOINT = 1200;
int CROSS_RIGHT_SETPOINT = 1500;

// Obstacle avoidance thresholds
int THRESHOLD_LRF = 1800;
int THRESHOLD_CROSS = 2800;

// Wall detection
int WALL_PRESENT_MIN = 300;

// Spike detection
int SPIKE_DROP = 700;
bool SPIKE_STOP_ENABLED = true;

// Speed settings
int BASE_SPEED = 200;
int MAX_SPEED = 255;

// PID gains - Adaptive system
float KP_WALL = 0.13;  // Gentle wall following
float KD_WALL = 1.8;
float KP_TURN = 2.7;   // Aggressive corners/avoidance
float KD_TURN = 1.3;

// Active gains
float KP = KP_WALL;
float KD = KD_WALL;

// Timing
unsigned long SENSOR_INTERVAL = 100;   // 10kHz
unsigned long MOTOR_INTERVAL = 5000;   // 200Hz
unsigned long DISPLAY_INTERVAL = 200;  // 5Hz display update

// Corner turn parameters
int CORNER_TURN_DURATION = 250;
int CORNER_TURN_SPEED_L = 180;
int CORNER_TURN_SPEED_R = 80;
int CORNER_DETECT_TIME = 200;

// ========== Global Objects ==========
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// ========== Global Variables ==========
// Sensor readings
int L = 0, CL = 0, F = 0, CR = 0, R = 0;
int prevL = 0, prevCL = 0, prevF = 0, prevCR = 0, prevR = 0;

// Motor targets
int targetLeft = 0;
int targetRight = 0;

// PID tracking
int lastSteerError = 0;
bool aggressiveMode = false;

// Timing
unsigned long lastSensorMicros = 0;
unsigned long lastMotorMicros = 0;
unsigned long lastDisplayMillis = 0;
unsigned long buzzerOffTime = 0;

// Flags
bool spikeDetected = false;
bool emergencyStop = false;

// Corner turn state
enum TurnState {
  STATE_FOLLOWING,
  STATE_CORNER_TURN
};

TurnState turnState = STATE_FOLLOWING;
unsigned long cornerTurnStartTime = 0;
unsigned long lastWallSeenTime = 0;
bool wallWasPresent = false;

// Display strings
String currentStateStr = "INIT";
String currentModeStr = "WALL";

// ========== Setup ==========
void setup() {
  Serial.begin(115200);

  // Configure motor pins
  pinMode(MOTOR_L_PWM, OUTPUT);
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_PWM, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);
  pinMode(MOTOR_STBY, OUTPUT);
  digitalWrite(MOTOR_STBY, HIGH);

  // Configure sensor pins
  pinMode(SENSOR_RIGHT, INPUT);
  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_FRONT, INPUT);
  pinMode(SENSOR_CROSS_LEFT, INPUT);
  pinMode(SENSOR_CROSS_RIGHT, INPUT);

  // Configure buzzer
  pinMode(BUZZER_PIN, OUTPUT);

  // Initialize NeoPixels
  pixels.begin();
  pixels.setBrightness(50);
  setAllPixels(0, 0, 255);  // Blue on startup
  pixels.show();

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Right Wall Pro");
  display.println("Adaptive PID");
  display.println("Corner Detection");
  display.println("Initializing...");
  display.display();

  // Startup sound sequence
  tone(BUZZER_PIN, 2000, 100);
  delay(200);
  tone(BUZZER_PIN, 2500, 100);
  delay(200);
  tone(BUZZER_PIN, 3000, 100);
  delay(500);

  setAllPixels(0, 255, 0);  // Green - ready
  pixels.show();

  Serial.println("\n===== Right Wall Follower PRO =====");
  Serial.println("Visual Feedback: NeoPixels + OLED");
  Serial.println("Algorithm: Adaptive PID + Corners");
  Serial.println("Type HELP for commands");
  Serial.println("===================================\n");
  
  delay(1500);
}

// ========== Main Loop ==========
void loop() {
  unsigned long nowMicros = micros();
  unsigned long nowMillis = millis();

  // Serial commands
  if (Serial.available() > 0) {
    processCommand();
  }

  // Buzzer management
  if (buzzerOffTime > 0 && nowMillis >= buzzerOffTime) {
    noTone(BUZZER_PIN);
    buzzerOffTime = 0;
  }

  // Emergency stop
  if (emergencyStop) {
    targetLeft = 0;
    targetRight = 0;
    applyMotors();
    setAllPixels(255, 0, 0);  // Red
    pixels.show();
    return;
  }

  // Sensor loop (10kHz)
  if (nowMicros - lastSensorMicros >= SENSOR_INTERVAL) {
    lastSensorMicros = nowMicros;
    readSensors();
    detectSpikes();
  }

  // Motor loop (200Hz)
  if (nowMicros - lastMotorMicros >= MOTOR_INTERVAL) {
    lastMotorMicros = nowMicros;
    calculateControl();
    updateVisuals();  // Update LEDs based on state
    
    // Serial output
    Serial.print("L:");
    Serial.print(L);
    Serial.print(" F:");
    Serial.print(F);
    Serial.print(" R:");
    Serial.print(R);
    Serial.print(" | State:");
    Serial.print(currentStateStr);
    Serial.print(" | Mode:");
    Serial.print(currentModeStr);
    Serial.print(" | M:");
    Serial.print(targetLeft);
    Serial.print(",");
    Serial.println(targetRight);
  }

  // Display update (5Hz - readable rate)
  if (nowMillis - lastDisplayMillis >= DISPLAY_INTERVAL) {
    lastDisplayMillis = nowMillis;
    updateDisplay();
  }

  // Apply motors
  applyMotors();
}

// ========== Sensor Reading ==========
void readSensors() {
  prevL = L;
  prevCL = CL;
  prevF = F;
  prevCR = CR;
  prevR = R;

  L = analogRead(SENSOR_LEFT);
  CL = analogRead(SENSOR_CROSS_LEFT);
  F = analogRead(SENSOR_FRONT);
  CR = analogRead(SENSOR_CROSS_RIGHT);
  R = analogRead(SENSOR_RIGHT);
}

// ========== Spike Detection ==========
void detectSpikes() {
  spikeDetected = false;
  
  if (!SPIKE_STOP_ENABLED)
    return;

  if (prevL - L > SPIKE_DROP) spikeDetected = true;
  if (prevCL - CL > SPIKE_DROP) spikeDetected = true;
  if (prevF - F > SPIKE_DROP) spikeDetected = true;
  if (prevCR - CR > SPIKE_DROP) spikeDetected = true;
  if (prevR - R > SPIKE_DROP) spikeDetected = true;

  if (spikeDetected) {
    targetLeft = -100;
    targetRight = -100;
    buzz(2000, 50);
    currentStateStr = "SPIKE!";
  }
}

// ========== Main Control Logic ==========
void calculateControl() {
  if (spikeDetected)
    return;

  unsigned long now = millis();

  // ===== EXTERNAL CORNER DETECTION =====
  bool rightWallPresent = (R > WALL_PRESENT_MIN) && (CR > WALL_PRESENT_MIN);
  
  if (rightWallPresent) {
    wallWasPresent = true;
    lastWallSeenTime = now;
  }
  
  switch (turnState) {
    case STATE_FOLLOWING:
      if (wallWasPresent && !rightWallPresent) {
        unsigned long timeSinceWallLost = now - lastWallSeenTime;
        
        if (timeSinceWallLost < CORNER_DETECT_TIME) {
          turnState = STATE_CORNER_TURN;
          cornerTurnStartTime = now;
          currentStateStr = "CORNER";
          buzz(1800, 80);
        }
      }
      break;
      
    case STATE_CORNER_TURN:
      targetLeft = CORNER_TURN_SPEED_L;
      targetRight = CORNER_TURN_SPEED_R;
      
      if (now - cornerTurnStartTime > CORNER_TURN_DURATION) {
        turnState = STATE_FOLLOWING;
        wallWasPresent = false;
        currentStateStr = "FOLLOW";
        buzz(1500, 50);
      }
      return;
  }

  // ===== ADAPTIVE PID GAIN SWITCHING =====
  aggressiveMode = false;
  
  if (F > 1200) aggressiveMode = true;
  
  if (R > WALL_PRESENT_MIN && CR > WALL_PRESENT_MIN) {
    int angleDifference = CR - R;
    if (angleDifference > 800) aggressiveMode = true;
  }
  
  if (L > 1500 || CL > 2200) aggressiveMode = true;
  
  if (R < WALL_PRESENT_MIN && prevR > WALL_PRESENT_MIN + 200) {
    aggressiveMode = true;
  }
  
  if (aggressiveMode) {
    KP = KP_TURN;
    KD = KD_TURN;
    currentModeStr = "TURN";
  } else {
    KP = KP_WALL;
    KD = KD_WALL;
    currentModeStr = "WALL";
  }

  // ===== SPEED CONTROL (Front) =====
  int frontStart = 600;
  int frontVal = constrain(F, frontStart, THRESHOLD_LRF);
  float frontFactor = map(frontVal, frontStart, THRESHOLD_LRF, 0, 1000) / 1000.0;
  int desiredSpeed = (int)(BASE_SPEED * (1.0 - frontFactor));

  // ===== STEERING FORCES =====
  
  // Left obstacle avoidance
  int leftPush = 0;
  if (CL > 600) {
    leftPush += map(constrain(CL, 600, THRESHOLD_CROSS), 600, THRESHOLD_CROSS, 0, 80);
  }
  if (L > 600) {
    leftPush += map(constrain(L, 600, THRESHOLD_LRF), 600, THRESHOLD_LRF, 0, 120);
  }

  // Right wall following
  int rightError = 0;
  if (R > WALL_PRESENT_MIN) {
    rightError = (R - RIGHT_SETPOINT);
    int angleContrib = (CR - CROSS_RIGHT_SETPOINT);
    rightError += angleContrib;
  } else {
    rightError = -200;
    currentStateStr = "SEARCH";
  }

  if (turnState == STATE_FOLLOWING) {
    if (rightWallPresent) {
      currentStateStr = "FOLLOW";
    }
  }

  // ===== COMBINE & APPLY PID =====
  int steerRightForce = leftPush;
  int steerLeftForce = rightError;
  int rawSteer = steerRightForce - steerLeftForce;
  int steerDeriv = rawSteer - lastSteerError;
  lastSteerError = rawSteer;
  int totalSteer = (int)((rawSteer * KP) + (steerDeriv * KD));

  // Front blocked - force turn
  if (frontFactor > 0.8) {
    currentStateStr = "BLOCKED";
    if (L < R) {
      totalSteer = -150;
    } else {
      totalSteer = 150;
    }
  }

  // Apply to motors
  targetLeft = constrain(desiredSpeed + totalSteer, -MAX_SPEED, MAX_SPEED);
  targetRight = constrain(desiredSpeed - totalSteer, -MAX_SPEED, MAX_SPEED);
}

// ========== Visual Feedback ==========
// Update NeoPixels based on current state and mode
void updateVisuals() {
  // Color coding:
  // Green = Normal wall following
  // Cyan = Gentle mode (WALL)
  // Yellow = Aggressive mode (TURN)
  // Magenta = Corner turn
  // Red = Blocked/Spike
  // Blue = Searching for wall
  
  if (spikeDetected) {
    setAllPixels(255, 0, 0);  // Red - spike
  } else if (turnState == STATE_CORNER_TURN) {
    setAllPixels(255, 0, 255);  // Magenta - corner turn
  } else if (currentStateStr == "BLOCKED") {
    setAllPixels(255, 100, 0);  // Orange - blocked
  } else if (currentStateStr == "SEARCH") {
    setAllPixels(0, 100, 255);  // Blue - searching
  } else if (aggressiveMode) {
    setAllPixels(255, 255, 0);  // Yellow - aggressive mode
  } else {
    setAllPixels(0, 255, 100);  // Green-cyan - normal following
  }
  
  pixels.show();
}

// Set all NeoPixels to same color
void setAllPixels(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NEOPIXEL_COUNT; i++) {
    pixels.setPixelColor(i, pixels.Color(r, g, b));
  }
}

// ========== OLED Display Update ==========
void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);

  // Line 1: State and Mode
  display.print("St:");
  display.print(currentStateStr);
  display.print(" Md:");
  display.println(currentModeStr);

  // Line 2: Front, Right, Left sensors
  display.print("F:");
  display.print(F);
  display.print(" R:");
  display.print(R);
  display.print(" L:");
  display.println(L);

  // Line 3: Cross sensors
  display.print("CR:");
  display.print(CR);
  display.print(" CL:");
  display.println(CL);

  // Line 4: Motor speeds
  display.print("M: ");
  display.print(targetLeft);
  display.print(",");
  display.println(targetRight);

  display.display();
}

// ========== Motor Control ==========
void applyMotors() {
  // Left motor
  if (targetLeft >= 0) {
    digitalWrite(MOTOR_L_IN1, LOW);
    digitalWrite(MOTOR_L_IN2, HIGH);
    analogWrite(MOTOR_L_PWM, targetLeft);
  } else {
    digitalWrite(MOTOR_L_IN1, HIGH);
    digitalWrite(MOTOR_L_IN2, LOW);
    analogWrite(MOTOR_L_PWM, -targetLeft);
  }

  // Right motor
  if (targetRight >= 0) {
    digitalWrite(MOTOR_R_IN1, LOW);
    digitalWrite(MOTOR_R_IN2, HIGH);
    analogWrite(MOTOR_R_PWM, targetRight);
  } else {
    digitalWrite(MOTOR_R_IN1, HIGH);
    digitalWrite(MOTOR_R_IN2, LOW);
    analogWrite(MOTOR_R_PWM, -targetRight);
  }
}

// ========== Buzzer Control ==========
void buzz(int freq, int duration) {
  tone(BUZZER_PIN, freq);
  buzzerOffTime = millis() + duration;
}

// ========== Serial Command Processing ==========
void processCommand() {
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "HELP" || cmd == "H" || cmd == "?") {
    printHelp();
    return;
  }

  if (cmd == "STOP" || cmd == "S") {
    emergencyStop = true;
    targetLeft = 0;
    targetRight = 0;
    Serial.println("EMERGENCY STOP");
    buzz(3000, 200);
    return;
  }
  
  if (cmd == "GO" || cmd == "G") {
    emergencyStop = false;
    Serial.println("RUNNING");
    buzz(1500, 100);
    return;
  }

  if (cmd == "STATUS") {
    Serial.println("\n===== Current Settings =====");
    Serial.print("STATE: "); Serial.println(currentStateStr);
    Serial.print("MODE: "); Serial.println(currentModeStr);
    Serial.println("\nADAPTIVE PID GAINS:");
    Serial.print("  WALL: KP="); Serial.print(KP_WALL, 2);
    Serial.print(", KD="); Serial.println(KD_WALL, 2);
    Serial.print("  TURN: KP="); Serial.print(KP_TURN, 2);
    Serial.print(", KD="); Serial.println(KD_TURN, 2);
    Serial.println("\nCORNER TURN:");
    Serial.print("  DURATION: "); Serial.print(CORNER_TURN_DURATION); Serial.println(" ms");
    Serial.print("  SPEED_L: "); Serial.println(CORNER_TURN_SPEED_L);
    Serial.print("  SPEED_R: "); Serial.println(CORNER_TURN_SPEED_R);
    Serial.println("\nWALL FOLLOWING:");
    Serial.print("  RIGHT_SETPOINT: "); Serial.println(RIGHT_SETPOINT);
    Serial.print("  BASE_SPEED: "); Serial.println(BASE_SPEED);
    Serial.println("============================\n");
    return;
  }

  if (cmd == "SPIKE ON") {
    SPIKE_STOP_ENABLED = true;
    Serial.println("Spike detection ON");
    buzz(1500, 30);
    return;
  }
  
  if (cmd == "SPIKE OFF") {
    SPIKE_STOP_ENABLED = false;
    Serial.println("Spike detection OFF");
    buzz(1500, 30);
    return;
  }

  // Parameter commands
  int spacePos = cmd.indexOf(' ');
  if (spacePos > 0) {
    String param = cmd.substring(0, spacePos);
    String valueStr = cmd.substring(spacePos + 1);
    
    if (param == "RS") {
      RIGHT_SETPOINT = valueStr.toInt();
      Serial.print("RIGHT_SETPOINT: "); Serial.println(RIGHT_SETPOINT);
    }
    else if (param == "CRS") {
      CROSS_RIGHT_SETPOINT = valueStr.toInt();
      Serial.print("CROSS_RIGHT_SETPOINT: "); Serial.println(CROSS_RIGHT_SETPOINT);
    }
    else if (param == "BS") {
      BASE_SPEED = valueStr.toInt();
      Serial.print("BASE_SPEED: "); Serial.println(BASE_SPEED);
    }
    else if (param == "KPW") {
      KP_WALL = valueStr.toFloat();
      Serial.print("KP_WALL: "); Serial.println(KP_WALL, 2);
    }
    else if (param == "KDW") {
      KD_WALL = valueStr.toFloat();
      Serial.print("KD_WALL: "); Serial.println(KD_WALL, 2);
    }
    else if (param == "KPT") {
      KP_TURN = valueStr.toFloat();
      Serial.print("KP_TURN: "); Serial.println(KP_TURN, 2);
    }
    else if (param == "KDT") {
      KD_TURN = valueStr.toFloat();
      Serial.print("KD_TURN: "); Serial.println(KD_TURN, 2);
    }
    else if (param == "CTD") {
      CORNER_TURN_DURATION = valueStr.toInt();
      Serial.print("CORNER_TURN_DURATION: "); Serial.print(CORNER_TURN_DURATION); Serial.println(" ms");
    }
    else if (param == "CTL") {
      CORNER_TURN_SPEED_L = valueStr.toInt();
      Serial.print("CORNER_TURN_SPEED_L: "); Serial.println(CORNER_TURN_SPEED_L);
    }
    else if (param == "CTR") {
      CORNER_TURN_SPEED_R = valueStr.toInt();
      Serial.print("CORNER_TURN_SPEED_R: "); Serial.println(CORNER_TURN_SPEED_R);
    }
    else {
      Serial.println("Unknown parameter");
      return;
    }
    
    buzz(1500, 30);
  }
}

// ========== Help Display ==========
void printHelp() {
  Serial.println("\n========== Right Wall Follower PRO ==========");
  Serial.println("Features: Adaptive PID + Corner Detection + Visual Feedback");
  Serial.println("\nCommands:");
  Serial.println("  HELP / ?      - Show this help");
  Serial.println("  STOP / S      - Emergency stop");
  Serial.println("  GO / G        - Resume running");
  Serial.println("  STATUS        - Show current settings");
  Serial.println("  SPIKE ON/OFF  - Toggle spike detection");
  Serial.println("\nQuick Tuning:");
  Serial.println("  KPW <val>  - Wall KP (default 0.13)");
  Serial.println("  KDW <val>  - Wall KD (default 1.8)");
  Serial.println("  KPT <val>  - Turn KP (default 2.7)");
  Serial.println("  KDT <val>  - Turn KD (default 1.3)");
  Serial.println("  BS <val>   - Base speed (default 200)");
  Serial.println("  CTD <val>  - Corner duration ms (default 250)");
  Serial.println("\nColor Guide:");
  Serial.println("  Green-Cyan = Normal following");
  Serial.println("  Yellow     = Aggressive/Turn mode");
  Serial.println("  Magenta    = Corner turn");
  Serial.println("  Blue       = Searching for wall");
  Serial.println("  Orange     = Front blocked");
  Serial.println("  Red        = Spike/Emergency");
  Serial.println("============================================\n");
}
