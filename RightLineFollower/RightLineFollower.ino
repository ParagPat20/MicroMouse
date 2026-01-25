/*
 * Right Wall Follower - MicroMouse
 * 
 * Algorithm: Combines proven ObstacleAvoid logic with Right Wall Following
 * Uses tuned values: KP=2.7, KD=1.3, BASE_SPEED=200
 * 
 * Strategy:
 * - PRIMARY: Follow right wall at setpoint distance using R + CR sensors
 * - SECONDARY: Avoid front and left obstacles using F, L, CL sensors
 * - All control is smooth and proportional (no hard turns)
 * - Serial tuning support for real-time adjustments
 */

// ========== Pin Definitions ==========
#define SENSOR_RIGHT 36
#define SENSOR_LEFT 39
#define SENSOR_FRONT 34
#define SENSOR_CROSS_LEFT 35
#define SENSOR_CROSS_RIGHT 32

// Motor pins (corrected mapping)
#define MOTOR_L_PWM 26
#define MOTOR_L_IN1 13
#define MOTOR_L_IN2 15
#define MOTOR_R_PWM 25
#define MOTOR_R_IN1 14
#define MOTOR_R_IN2 12
#define MOTOR_STBY 4

#define BUZZER_PIN 17

// ========== Tunable Parameters (via Serial) ==========

// Right wall following setpoints (desired distance from wall)
// Lower values = closer to wall, Higher values = farther from wall
int RIGHT_SETPOINT = 1200;      // Target reading for R sensor (~5cm)
int CROSS_RIGHT_SETPOINT = 1500; // Target reading for CR sensor (~5cm ahead)

// Obstacle avoidance thresholds (critical proximity)
int THRESHOLD_LRF = 1800;    // Critical distance for L, R, F sensors
int THRESHOLD_CROSS = 2800;  // Critical distance for CL, CR sensors

// Wall detection
int WALL_PRESENT_MIN = 300;  // Below this = no wall detected on right

// Spike detection (sudden sensor drops indicate <2cm - noise zone)
int SPIKE_DROP = 700;
bool SPIKE_STOP_ENABLED = true;

// Speed settings
int BASE_SPEED = 200;  // Cruising speed when following wall
int MAX_SPEED = 255;   // Maximum motor PWM

// PID gains - Adaptive system with two modes
// WALL FOLLOWING MODE: Smooth tracking (discovered values)
float KP_WALL = 0.13;  // Gentle proportional for smooth following
float KD_WALL = 1.8;   // Gentle derivative for stability

// CORNER/AVOIDANCE MODE: Sharp turns (ObstacleAvoid values)
float KP_TURN = 2.7;   // Aggressive proportional for quick response
float KD_TURN = 1.3;   // Aggressive derivative for sharp turns

// Active gains (will switch between modes automatically)
float KP = KP_WALL;
float KD = KD_WALL;

// Timing
unsigned long SENSOR_INTERVAL = 100;  // Sensor loop: 10kHz
unsigned long MOTOR_INTERVAL = 5000;  // Motor loop: 200Hz

// Corner turn parameters (for external 90° corners)
int CORNER_TURN_DURATION = 250;  // Duration of quick turn in ms
int CORNER_TURN_SPEED_L = 180;   // Left motor speed during turn
int CORNER_TURN_SPEED_R = 80;    // Right motor speed during turn (slower = turn right)
int CORNER_DETECT_TIME = 200;    // Time window to detect R and CR both lost (ms)

// ========== Global Variables ==========
// Sensor readings (current and previous)
int L = 0, CL = 0, F = 0, CR = 0, R = 0;
int prevL = 0, prevCL = 0, prevF = 0, prevCR = 0, prevR = 0;

// Motor targets
int targetLeft = 0;
int targetRight = 0;

// PID error tracking
int lastSteerError = 0;

// Adaptive gain mode tracking
bool aggressiveMode = false;

// Timing
unsigned long lastSensorMicros = 0;
unsigned long lastMotorMicros = 0;
unsigned long buzzerOffTime = 0;

// Flags
bool spikeDetected = false;
bool emergencyStop = false;

// Corner turn state (for external 90° corners)
enum TurnState {
  STATE_FOLLOWING,    // Normal wall following
  STATE_CORNER_TURN   // Executing timed corner turn
};

TurnState turnState = STATE_FOLLOWING;
unsigned long cornerTurnStartTime = 0;
unsigned long lastWallSeenTime = 0;
bool wallWasPresent = false;

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

  // Configure sensor pins
  pinMode(SENSOR_RIGHT, INPUT);
  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_FRONT, INPUT);
  pinMode(SENSOR_CROSS_LEFT, INPUT);
  pinMode(SENSOR_CROSS_RIGHT, INPUT);

  // Configure buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Enable motor driver
  digitalWrite(MOTOR_STBY, HIGH);

  // Startup beeps
  tone(BUZZER_PIN, 2000, 100);
  delay(200);
  tone(BUZZER_PIN, 2500, 100);
  delay(500);

  printHelp();
  Serial.println("Right Wall Follower READY!\n");
}

// ========== Main Loop ==========
void loop() {
  unsigned long nowMicros = micros();
  unsigned long nowMillis = millis();

  // Process serial commands for tuning
  if (Serial.available() > 0) {
    processCommand();
  }

  // Turn off buzzer when timer expires
  if (buzzerOffTime > 0 && nowMillis >= buzzerOffTime) {
    noTone(BUZZER_PIN);
    buzzerOffTime = 0;
  }

  // Emergency stop mode
  if (emergencyStop) {
    targetLeft = 0;
    targetRight = 0;
    applyMotors();
    return;
  }

  // Sensor loop (10kHz) - read sensors and detect spikes
  if (nowMicros - lastSensorMicros >= SENSOR_INTERVAL) {
    lastSensorMicros = nowMicros;
    readSensors();
    detectSpikes();
  }

  // Motor loop (200Hz) - calculate control and update motors
  if (nowMicros - lastMotorMicros >= MOTOR_INTERVAL) {
    lastMotorMicros = nowMicros;
    calculateControl();
    printStatus();
  }

  // Apply motor commands
  applyMotors();
}

// ========== Sensor Reading ==========
// Read all 5 sensors and store previous values for spike detection
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
// Detect sudden sensor drops (indicates object <2cm - sensor noise zone)
// Response: Brief backward movement
void detectSpikes() {
  spikeDetected = false;
  
  if (!SPIKE_STOP_ENABLED)
    return;

  // Check each sensor for sudden drop
  if (prevL - L > SPIKE_DROP) spikeDetected = true;
  if (prevCL - CL > SPIKE_DROP) spikeDetected = true;
  if (prevF - F > SPIKE_DROP) spikeDetected = true;
  if (prevCR - CR > SPIKE_DROP) spikeDetected = true;
  if (prevR - R > SPIKE_DROP) spikeDetected = true;

  if (spikeDetected) {
    // Back up briefly
    targetLeft = -100;
    targetRight = -100;
    buzz(2000, 50);
  }
}

// ========== Main Control Logic ==========
// Priority-based control system:
// 1. Front obstacle → Slow down proportionally
// 2. Left obstacles → Steer right to avoid
// 3. Right wall → Follow at setpoint distance
// 4. No right wall → Arc right to find it
void calculateControl() {
  // Skip control if spike detected (backing up)
  if (spikeDetected)
    return;

  /*
   * CONTROL STRATEGY:
   * 
   * STATE MACHINE (External Corner Detection):
   *   - Detect when both R and CR lose wall within 200ms window
   *   - Execute timed sharp right turn to follow corner
   *   - Return to normal following after turn completes
   * 
   * ADAPTIVE PID GAINS:
   *   - WALL MODE (KP=0.13, KD=1.8): Smooth wall following
   *   - TURN MODE (KP=2.7, KD=1.3): Sharp corners and obstacle avoidance
   *   - Auto-switch based on corner detection
   * 
   * SPEED CONTROL (Front Sensor):
   *   - Far from front (F low): Full BASE_SPEED
   *   - Approaching front (F rising): Gradually reduce speed
   *   - Very close (F high): Near zero speed
   * 
   * STEERING CONTROL (Combined forces):
   *   A. RIGHT WALL FOLLOWING (Primary):
   *      - Measure error between R and RIGHT_SETPOINT
   *      - Use CR to detect angle to wall (predictive)
   *      - Steer to maintain setpoint distance and parallel alignment
   *   
   *   B. LEFT OBSTACLE AVOIDANCE (Override when necessary):
   *      - If L or CL detect obstacles, bias steering right
   *      - Higher urgency overrides wall following
   * 
   * All steering is differential (speed up one wheel, slow down other)
   */

  unsigned long now = millis();

  // ===== EXTERNAL CORNER DETECTION & STATE MACHINE =====
  // Track when wall is present vs lost
  bool rightWallPresent = (R > WALL_PRESENT_MIN) && (CR > WALL_PRESENT_MIN);
  
  if (rightWallPresent) {
    wallWasPresent = true;
    lastWallSeenTime = now;
  }
  
  // State machine for corner turns
  switch (turnState) {
    case STATE_FOLLOWING:
      // Check for external corner: both R and CR lost within time window
      if (wallWasPresent && !rightWallPresent) {
        unsigned long timeSinceWallLost = now - lastWallSeenTime;
        
        if (timeSinceWallLost < CORNER_DETECT_TIME) {
          // External 90° corner detected! Start quick turn
          turnState = STATE_CORNER_TURN;
          cornerTurnStartTime = now;
          buzz(1800, 80);  // Audio feedback
        }
      }
      break;
      
    case STATE_CORNER_TURN:
      // Execute timed corner turn (arc right to follow wall)
      targetLeft = CORNER_TURN_SPEED_L;
      targetRight = CORNER_TURN_SPEED_R;
      
      // Check if turn duration completed
      if (now - cornerTurnStartTime > CORNER_TURN_DURATION) {
        turnState = STATE_FOLLOWING;
        wallWasPresent = false;  // Reset for next corner
        buzz(1500, 50);  // Turn complete beep
      }
      
      // Early exit - corner turn overrides all other logic
      return;
  }

  // ===== STEP 0: CORNER DETECTION - Switch PID gains adaptively =====
  // Detect situations requiring aggressive gains (sharp turns)
  aggressiveMode = false;
  
  // Condition 1: Front obstacle approaching (90° corner ahead)
  if (F > 1200) {
    aggressiveMode = true;
  }
  
  // Condition 2: Internal corner (CR much higher than R = angling sharply toward wall)
  if (R > WALL_PRESENT_MIN && CR > WALL_PRESENT_MIN) {
    int angleDifference = CR - R;
    if (angleDifference > 800) {  // Sharp angle toward wall
      aggressiveMode = true;
    }
  }
  
  // Condition 3: Left obstacle very close (need quick avoidance)
  if (L > 1500 || CL > 2200) {
    aggressiveMode = true;
  }
  
  // Condition 4: Right wall suddenly lost (external corner)
  if (R < WALL_PRESENT_MIN && prevR > WALL_PRESENT_MIN + 200) {
    aggressiveMode = true;
  }
  
  // Switch PID gains based on mode
  if (aggressiveMode) {
    KP = KP_TURN;  // Aggressive: 2.7
    KD = KD_TURN;  // Aggressive: 1.3
  } else {
    KP = KP_WALL;  // Gentle: 0.13
    KD = KD_WALL;  // Gentle: 1.8
  }

  // ===== STEP 1: FRONT OBSTACLE - Speed Control =====
  // Map front sensor from "start reacting" to "critical close"
  int frontStart = 600;  // Start slowing down at this reading
  int frontVal = constrain(F, frontStart, THRESHOLD_LRF);
  
  // Calculate how much to reduce speed (0.0 = no reduction, 1.0 = full stop)
  float frontFactor = map(frontVal, frontStart, THRESHOLD_LRF, 0, 1000) / 1000.0;
  
  // Desired speed: reduce from BASE_SPEED as we approach obstacle
  int desiredSpeed = (int)(BASE_SPEED * (1.0 - frontFactor));

  // ===== STEP 2: STEERING - Calculate turning forces =====
  
  // A. LEFT OBSTACLE AVOIDANCE FORCE (pushes us right)
  int leftPush = 0;
  
  // CL (cross left) - longer range, early warning
  if (CL > 600) {
    leftPush += map(constrain(CL, 600, THRESHOLD_CROSS), 
                    600, THRESHOLD_CROSS, 0, 80);
  }
  
  // L (direct left) - shorter range, higher urgency
  if (L > 600) {
    leftPush += map(constrain(L, 600, THRESHOLD_LRF), 
                    600, THRESHOLD_LRF, 0, 120);
  }

  // B. RIGHT WALL FOLLOWING FORCE (maintains setpoint)
  int rightError = 0;
  
  if (R > WALL_PRESENT_MIN) {
    // Right wall detected - follow it
    
    // Distance error: R - setpoint
    // If R > setpoint: too close to wall, steer left (positive error)
    // If R < setpoint: too far from wall, steer right (negative error)
    rightError = (R - RIGHT_SETPOINT);
    
    // Angle error: Use CR to predict if we're heading toward/away from wall
    // If CR > setpoint: angling toward wall, need to steer left (positive)
    // If CR < setpoint: angling away from wall, need to steer right (negative)
    int angleContrib = (CR - CROSS_RIGHT_SETPOINT);
    
    // Combine distance and angle errors
    rightError += angleContrib;
    
  } else {
    // No right wall detected - bias to turn right to find it
    rightError = -200;  // Negative = turn right
  }

  // ===== STEP 3: Combine forces into steering command =====
  
  // Define steering convention:
  // Positive steer = Turn RIGHT (left motor faster, right motor slower)
  // Negative steer = Turn LEFT (left motor slower, right motor faster)
  
  int steerRightForce = leftPush;   // Left obstacles make us turn RIGHT
  int steerLeftForce = rightError;  // Too close to right wall = turn LEFT
  
  // Net steering = forces to turn right - forces to turn left
  int rawSteer = steerRightForce - steerLeftForce;

  // ===== STEP 4: Apply PID =====
  
  // Calculate derivative (rate of change of error)
  int steerDeriv = rawSteer - lastSteerError;
  lastSteerError = rawSteer;
  
  // PID output
  int totalSteer = (int)((rawSteer * KP) + (steerDeriv * KD));

  // ===== STEP 5: Special case - Front blocked =====
  // If front is very close, force a turn toward the more open side
  if (frontFactor > 0.8) {
    if (L < R) {
      // Left is more open
      totalSteer = -150;  // Turn left
    } else {
      // Right is more open
      totalSteer = 150;   // Turn right
    }
  }

  // ===== STEP 6: Apply to motors =====
  
  // Differential steering:
  // Left motor: base speed + steer (faster when turning right)
  // Right motor: base speed - steer (slower when turning right)
  targetLeft = constrain(desiredSpeed + totalSteer, -MAX_SPEED, MAX_SPEED);
  targetRight = constrain(desiredSpeed - totalSteer, -MAX_SPEED, MAX_SPEED);
}

// ========== Motor Control ==========
// Apply calculated motor speeds to hardware
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

// ========== Status Output ==========
// Print sensor values, state, mode, and motor commands at 200Hz (readable rate)
void printStatus() {
  Serial.print("L:");
  Serial.print(L);
  Serial.print(" CL:");
  Serial.print(CL);
  Serial.print(" F:");
  Serial.print(F);
  Serial.print(" CR:");
  Serial.print(CR);
  Serial.print(" R:");
  Serial.print(R);
  Serial.print(" | State:");
  Serial.print(turnState == STATE_CORNER_TURN ? "CORNER" : "FOLLOW");
  Serial.print(" | Mode:");
  Serial.print(aggressiveMode ? "TURN" : "WALL");
  Serial.print(" | Steer:");
  Serial.print(lastSteerError);
  Serial.print(" | M:");
  Serial.print(targetLeft);
  Serial.print(",");
  Serial.println(targetRight);
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

  // Help command
  if (cmd == "HELP" || cmd == "H" || cmd == "?") {
    printHelp();
    return;
  }

  // Stop/Go commands
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

  // Status command
  if (cmd == "STATUS") {
    Serial.println("\n===== Current Settings =====");
    Serial.print("STATE: ");
    Serial.println(turnState == STATE_CORNER_TURN ? "CORNER_TURN" : "FOLLOWING");
    Serial.println("\nADAPTIVE PID GAINS:");
    Serial.print("  WALL MODE: KP="); Serial.print(KP_WALL, 2);
    Serial.print(", KD="); Serial.println(KD_WALL, 2);
    Serial.print("  TURN MODE: KP="); Serial.print(KP_TURN, 2);
    Serial.print(", KD="); Serial.println(KD_TURN, 2);
    Serial.print("  Current Mode: "); Serial.println(aggressiveMode ? "TURN" : "WALL");
    Serial.println("\nCORNER TURN (External 90°):");
    Serial.print("  DURATION: "); Serial.print(CORNER_TURN_DURATION); Serial.println(" ms");
    Serial.print("  SPEED_L: "); Serial.println(CORNER_TURN_SPEED_L);
    Serial.print("  SPEED_R: "); Serial.println(CORNER_TURN_SPEED_R);
    Serial.print("  DETECT_TIME: "); Serial.print(CORNER_DETECT_TIME); Serial.println(" ms");
    Serial.println("\nWALL FOLLOWING:");
    Serial.print("  RIGHT_SETPOINT: "); Serial.println(RIGHT_SETPOINT);
    Serial.print("  CROSS_RIGHT_SETPOINT: "); Serial.println(CROSS_RIGHT_SETPOINT);
    Serial.print("  WALL_PRESENT_MIN: "); Serial.println(WALL_PRESENT_MIN);
    Serial.println("\nOBSTACLE THRESHOLDS:");
    Serial.print("  THRESHOLD_LRF: "); Serial.println(THRESHOLD_LRF);
    Serial.print("  THRESHOLD_CROSS: "); Serial.println(THRESHOLD_CROSS);
    Serial.println("\nSPEED & SAFETY:");
    Serial.print("  BASE_SPEED: "); Serial.println(BASE_SPEED);
    Serial.print("  MAX_SPEED: "); Serial.println(MAX_SPEED);
    Serial.print("  SPIKE_DROP: "); Serial.println(SPIKE_DROP);
    Serial.print("  SPIKE_STOP: "); Serial.println(SPIKE_STOP_ENABLED ? "ON" : "OFF");
    Serial.println("============================\n");
    return;
  }

  // Spike detection toggle
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

  // Parameter adjustment commands
  int spacePos = cmd.indexOf(' ');
  if (spacePos > 0) {
    String param = cmd.substring(0, spacePos);
    String valueStr = cmd.substring(spacePos + 1);
    
    if (param == "RS") {  // Right Setpoint
      RIGHT_SETPOINT = valueStr.toInt();
      Serial.print("RIGHT_SETPOINT: "); Serial.println(RIGHT_SETPOINT);
    }
    else if (param == "CRS") {  // Cross Right Setpoint
      CROSS_RIGHT_SETPOINT = valueStr.toInt();
      Serial.print("CROSS_RIGHT_SETPOINT: "); Serial.println(CROSS_RIGHT_SETPOINT);
    }
    else if (param == "TL") {  // Threshold LRF
      THRESHOLD_LRF = valueStr.toInt();
      Serial.print("THRESHOLD_LRF: "); Serial.println(THRESHOLD_LRF);
    }
    else if (param == "TC") {  // Threshold Cross
      THRESHOLD_CROSS = valueStr.toInt();
      Serial.print("THRESHOLD_CROSS: "); Serial.println(THRESHOLD_CROSS);
    }
    else if (param == "WM") {  // Wall Min
      WALL_PRESENT_MIN = valueStr.toInt();
      Serial.print("WALL_PRESENT_MIN: "); Serial.println(WALL_PRESENT_MIN);
    }
    else if (param == "SD") {  // Spike Drop
      SPIKE_DROP = valueStr.toInt();
      Serial.print("SPIKE_DROP: "); Serial.println(SPIKE_DROP);
    }
    else if (param == "BS") {  // Base Speed
      BASE_SPEED = valueStr.toInt();
      Serial.print("BASE_SPEED: "); Serial.println(BASE_SPEED);
    }
    else if (param == "MS") {  // Max Speed
      MAX_SPEED = valueStr.toInt();
      Serial.print("MAX_SPEED: "); Serial.println(MAX_SPEED);
    }
    else if (param == "KPW") {  // KP Wall following
      KP_WALL = valueStr.toFloat();
      Serial.print("KP_WALL: "); Serial.println(KP_WALL, 2);
    }
    else if (param == "KDW") {  // KD Wall following
      KD_WALL = valueStr.toFloat();
      Serial.print("KD_WALL: "); Serial.println(KD_WALL, 2);
    }
    else if (param == "KPT") {  // KP Turn/avoidance
      KP_TURN = valueStr.toFloat();
      Serial.print("KP_TURN: "); Serial.println(KP_TURN, 2);
    }
    else if (param == "KDT") {  // KD Turn/avoidance
      KD_TURN = valueStr.toFloat();
      Serial.print("KD_TURN: "); Serial.println(KD_TURN, 2);
    }
    else if (param == "CTD") {  // Corner Turn Duration
      CORNER_TURN_DURATION = valueStr.toInt();
      Serial.print("CORNER_TURN_DURATION: "); Serial.print(CORNER_TURN_DURATION); Serial.println(" ms");
    }
    else if (param == "CTL") {  // Corner Turn Speed Left
      CORNER_TURN_SPEED_L = valueStr.toInt();
      Serial.print("CORNER_TURN_SPEED_L: "); Serial.println(CORNER_TURN_SPEED_L);
    }
    else if (param == "CTR") {  // Corner Turn Speed Right
      CORNER_TURN_SPEED_R = valueStr.toInt();
      Serial.print("CORNER_TURN_SPEED_R: "); Serial.println(CORNER_TURN_SPEED_R);
    }
    else if (param == "CDT") {  // Corner Detect Time
      CORNER_DETECT_TIME = valueStr.toInt();
      Serial.print("CORNER_DETECT_TIME: "); Serial.print(CORNER_DETECT_TIME); Serial.println(" ms");
    }
    else {
      Serial.println("Unknown parameter");
      return;
    }
    
    buzz(1500, 30);  // Confirmation beep
  }
}

// ========== Help Display ==========
void printHelp() {
  Serial.println("\n========== Right Wall Follower (Adaptive PID + Corner Detection) ==========");
  Serial.println("Strategy: Follow right wall, detect & execute 90° corner turns");
  Serial.println("Algorithm: State machine + AUTO-SWITCHING dual-mode PID");
  Serial.println("  WALL Mode: Gentle (KP=0.13, KD=1.8) - smooth tracking");
  Serial.println("  TURN Mode: Aggressive (KP=2.7, KD=1.3) - sharp corners");
  Serial.println("  CORNER State: Timed turn when R+CR both lost (external corner)");
  Serial.println("\nCommands:");
  Serial.println("  HELP / ?      - Show this help");
  Serial.println("  STOP / S      - Emergency stop");
  Serial.println("  GO / G        - Resume running");
  Serial.println("  STATUS        - Show all current settings");
  Serial.println("  SPIKE ON/OFF  - Toggle spike detection");
  Serial.println("\nPID Tuning (Adaptive Gains):");
  Serial.println("  KPW <val>  - KP_WALL (wall following, default 0.13)");
  Serial.println("  KDW <val>  - KD_WALL (wall following, default 1.8)");
  Serial.println("  KPT <val>  - KP_TURN (corners/avoid, default 2.7)");
  Serial.println("  KDT <val>  - KD_TURN (corners/avoid, default 1.3)");
  Serial.println("\nCorner Turn (External 90°):");
  Serial.println("  CTD <val>  - CORNER_TURN_DURATION (ms, default 250)");
  Serial.println("  CTL <val>  - CORNER_TURN_SPEED_L (left motor, default 180)");
  Serial.println("  CTR <val>  - CORNER_TURN_SPEED_R (right motor, default 80)");
  Serial.println("  CDT <val>  - CORNER_DETECT_TIME (ms window, default 200)");
  Serial.println("\nWall Following:");
  Serial.println("  RS <val>   - RIGHT_SETPOINT (target distance)");
  Serial.println("  CRS <val>  - CROSS_RIGHT_SETPOINT (angle target)");
  Serial.println("  WM <val>   - WALL_PRESENT_MIN (detection threshold)");
  Serial.println("\nObstacle Thresholds:");
  Serial.println("  TL <val>   - THRESHOLD_LRF (L,R,F critical)");
  Serial.println("  TC <val>   - THRESHOLD_CROSS (CL,CR critical)");
  Serial.println("\nSpeed & Safety:");
  Serial.println("  BS <val>   - BASE_SPEED (cruising speed)");
  Serial.println("  MS <val>   - MAX_SPEED (maximum PWM)");
  Serial.println("  SD <val>   - SPIKE_DROP (spike sensitivity)");
  Serial.println("===========================================================================\n");
}
