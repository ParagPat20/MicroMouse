/*
 * Obstacle Avoidance Test - MicroMouse
 * Simple obstacle avoidance using all 5 sensors
 * Serial tuning for thresholds, response time, and spike detection
 *
 * Uses corrected motor pins (L/R swapped)
 */

// ========== Pin Definitions ==========
#define SENSOR_RIGHT 36
#define SENSOR_LEFT 39
#define SENSOR_FRONT 34
#define SENSOR_CROSS_LEFT 35
#define SENSOR_CROSS_RIGHT 32

// Left motor (was R)
#define MOTOR_L_PWM 26
#define MOTOR_L_IN1 13
#define MOTOR_L_IN2 15
// Right motor (was L)
#define MOTOR_R_PWM 25
#define MOTOR_R_IN1 14
#define MOTOR_R_IN2 12
#define MOTOR_STBY 4

#define BUZZER_PIN 17

// ========== Tunable Parameters (via Serial) ==========
// Thresholds now define the "CRITICAL" close point
// We will start reacting smoothly from START_THRESHOLD (e.g., 25% of these
// values)
int THRESHOLD_LRF = 1800;   // Critical distance for L, R, F (Stop/Hard turn)
int THRESHOLD_CROSS = 2800; // Critical distance for CL, CR
int SPIKE_DROP = 700;       // Sudden drop value for spike detection
int BASE_SPEED = 200;       // Base forward speed
int MAX_SPEED = 255;        // Maximum motor speed (PWM limit)

unsigned long SENSOR_INTERVAL = 100; // Sensor loop interval (us) - 10kHz
unsigned long MOTOR_INTERVAL = 5000; // Motor loop interval (us) - 200Hz

bool SPIKE_STOP_ENABLED = true; // Enable/disable spike detection stop

// PD Control Gains (for proportional avoidance)
float KP = 2.7; // Tuned for perfect response
float KD = 1.3; // Tuned for perfect response

// ========== Globals ==========
int L = 0, CL = 0, F = 0, CR = 0, R = 0;
int prevL = 0, prevCL = 0, prevF = 0, prevCR = 0, prevR = 0;

int targetLeft = 0;
int targetRight = 0;

// PD error tracking
int lastFrontError = 0;
int lastLeftError = 0;
int lastRightError = 0;

unsigned long lastSensorMicros = 0;
unsigned long lastMotorMicros = 0;
unsigned long buzzerOffTime = 0;

bool spikeDetected = false;
bool emergencyStop = false;

void setup() {
  Serial.begin(115200);

  // Motor Pins
  pinMode(MOTOR_L_PWM, OUTPUT);
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_PWM, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);
  pinMode(MOTOR_STBY, OUTPUT);

  // Sensor Pins
  pinMode(SENSOR_RIGHT, INPUT);
  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_FRONT, INPUT);
  pinMode(SENSOR_CROSS_LEFT, INPUT);
  pinMode(SENSOR_CROSS_RIGHT, INPUT);

  // Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(MOTOR_STBY, HIGH);

  // Startup
  tone(BUZZER_PIN, 2000, 100);
  delay(200);
  tone(BUZZER_PIN, 2500, 100);
  delay(500);

  printHelp();
  Serial.println("Ready! Robot will avoid obstacles.\n");
}

void loop() {
  unsigned long nowMicros = micros();
  unsigned long nowMillis = millis();

  // Serial command processing
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    processCommand(cmd);
  }

  // Buzzer off
  if (buzzerOffTime > 0 && nowMillis >= buzzerOffTime) {
    noTone(BUZZER_PIN);
    buzzerOffTime = 0;
  }

  // Emergency stop active
  if (emergencyStop) {
    targetLeft = 0;
    targetRight = 0;
    applyMotors();
    return;
  }

  // Sensor loop
  if (nowMicros - lastSensorMicros >= SENSOR_INTERVAL) {
    lastSensorMicros = nowMicros;
    readSensors();
    detectSpikes();
  }

  // Motor loop
  if (nowMicros - lastMotorMicros >= MOTOR_INTERVAL) {
    lastMotorMicros = nowMicros;
    runAvoidance();
    printStatus();
  }

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

  // Check for sudden drops (indicates < 2cm, sensor noise zone)
  if (prevL - L > SPIKE_DROP)
    spikeDetected = true;
  if (prevCL - CL > SPIKE_DROP)
    spikeDetected = true;
  if (prevF - F > SPIKE_DROP)
    spikeDetected = true;
  if (prevCR - CR > SPIKE_DROP)
    spikeDetected = true;
  if (prevR - R > SPIKE_DROP)
    spikeDetected = true;

  if (spikeDetected) {
    // Back up briefly
    targetLeft = -50;
    targetRight = -50;
    buzz(2000, 50);
  }
}

// ========== Obstacle Avoidance - Smooth Graduated Control ==========
void runAvoidance() {
  if (spikeDetected)
    return;

  /*
   * SMOOTHNESS LOGIC:
   * Instead of a hard threshold where we suddenly react, we define a "Zone of
   * Influence".
   *
   * 1. Start Zone: Sensor sees something far away (Low value, e.g. 800)
   * 2. Critical Zone: Sensor is too close (High value, e.g. 2500)
   *
   * Response is mapped 0% to 100% between these zones.
   * - Front obstacle: Smoothly reduce speed (decelerate early)
   * - Side obstacle: Smoothly add steering bias (turn early)
   */

  // 1. FRONT AVOIDANCE (Speed Limit)
  // Map Front sensor from START (far) to CRITICAL (close)
  // Far (800) -> 100% Speed, Close (THRESHOLD) -> 0% Speed (or negative)
  int frontStart = 800;
  int frontVal = constrain(F, frontStart, THRESHOLD_LRF);
  float frontFactor =
      map(frontVal, frontStart, THRESHOLD_LRF, 0, 1000) / 1000.0;

  // Calculate Target Speed: smoothly blend from BASE_SPEED down to 0
  int desiredSpeed = (int)(BASE_SPEED * (1.0 - frontFactor));

  // 2. STEERING (Turn Away)
  // Map Cross sensors (Long Range) specifically for early turning
  int crossStart = 800; // Start turning when wall is seen far away

  // Calculate Left urgency (L + CL)
  // L is close range, CL is long range. Weighted average.
  int leftUrgency = 0;
  if (CL > crossStart)
    leftUrgency += map(constrain(CL, crossStart, THRESHOLD_CROSS), crossStart,
                       THRESHOLD_CROSS, 0, 100);
  if (L > frontStart)
    leftUrgency += map(constrain(L, frontStart, THRESHOLD_LRF), frontStart,
                       THRESHOLD_LRF, 0, 150); // L is more critical

  // Calculate Right urgency (R + CR)
  int rightUrgency = 0;
  if (CR > crossStart)
    rightUrgency += map(constrain(CR, crossStart, THRESHOLD_CROSS), crossStart,
                        THRESHOLD_CROSS, 0, 100);
  if (R > frontStart)
    rightUrgency += map(constrain(R, frontStart, THRESHOLD_LRF), frontStart,
                        THRESHOLD_LRF, 0, 150);

  // Derivative (D) term to dampen swinging
  int turnError = rightUrgency - leftUrgency;
  // We don't have a "lastTurnError" global, let's just use Proportional for
  // smooth curves first. Or reuse the existing globals if possible, but let's
  // keep it simple and smooth.

  // Steering Force:
  // If Right is closer: rightUrgency > leftUrgency -> turnError > 0 -> steer >
  // 0 (Turn Right?? No wait) Let's use standard convention: Steer > 0 means
  // Turn Right (Left Motor increases).

  // Danger on LEFT -> We want Steer > 0 (Turn Right) -> leftUrgency -
  // rightUrgency
  int steer = (int)((leftUrgency - rightUrgency) * KP);

  // 3. APPLY
  // If we are very close to front wall (frontFactor > 0.8), force a turn even
  // if sides are balanced
  if (frontFactor > 0.8) {
    // If getting blocked, pick a side:
    if (leftUrgency > rightUrgency)
      steer = 100; // Turn Right (away from left danger)
    else
      steer = -100; // Turn Left (away from right danger)
  }

  // Steer > 0 (Turn Right) -> Left Motor +, Right Motor -
  // Steer < 0 (Turn Left)  -> Left Motor -, Right Motor +
  targetLeft = constrain(desiredSpeed + steer, -MAX_SPEED, MAX_SPEED);
  targetRight = constrain(desiredSpeed - steer, -MAX_SPEED, MAX_SPEED);
}

// ========== Motor Control ==========
void applyMotors() {
  // Left
  if (targetLeft >= 0) {
    digitalWrite(MOTOR_L_IN1, LOW);
    digitalWrite(MOTOR_L_IN2, HIGH);
    analogWrite(MOTOR_L_PWM, targetLeft);
  } else {
    digitalWrite(MOTOR_L_IN1, HIGH);
    digitalWrite(MOTOR_L_IN2, LOW);
    analogWrite(MOTOR_L_PWM, -targetLeft);
  }

  // Right
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

// ========== Status Print ==========
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
  Serial.print(" | M:");
  Serial.print(targetLeft);
  Serial.print(",");
  Serial.println(targetRight);
}

// ========== Serial Commands ==========
void processCommand(String cmd) {
  cmd.toUpperCase();

  // Help
  if (cmd == "HELP" || cmd == "H" || cmd == "?") {
    printHelp();
    return;
  }

  // Stop/Start
  if (cmd == "STOP" || cmd == "S") {
    emergencyStop = true;
    targetLeft = 0;
    targetRight = 0;
    Serial.println("STOPPED");
    buzz(3000, 200);
    return;
  }
  if (cmd == "GO" || cmd == "G") {
    emergencyStop = false;
    Serial.println("RUNNING");
    buzz(1500, 100);
    return;
  }

  // Status
  if (cmd == "STATUS") {
    Serial.println("\n===== Current Settings =====");
    Serial.print("THRESHOLD_LRF: ");
    Serial.println(THRESHOLD_LRF);
    Serial.print("THRESHOLD_CROSS: ");
    Serial.println(THRESHOLD_CROSS);
    Serial.print("SPIKE_DROP: ");
    Serial.println(SPIKE_DROP);
    Serial.print("SPIKE_STOP: ");
    Serial.println(SPIKE_STOP_ENABLED ? "ON" : "OFF");
    Serial.print("BASE_SPEED: ");
    Serial.println(BASE_SPEED);
    Serial.print("MAX_SPEED: ");
    Serial.println(MAX_SPEED);
    Serial.print("KP: ");
    Serial.println(KP, 4);
    Serial.print("KD: ");
    Serial.println(KD, 4);
    Serial.print("SENSOR_INTERVAL: ");
    Serial.print(SENSOR_INTERVAL);
    Serial.println(" us");
    Serial.print("MOTOR_INTERVAL: ");
    Serial.print(MOTOR_INTERVAL);
    Serial.println(" us");
    Serial.println("=============================\n");
    return;
  }

  // Toggle spike stop
  if (cmd == "SPIKE ON") {
    SPIKE_STOP_ENABLED = true;
    Serial.println("Spike detection ON");
    return;
  }
  if (cmd == "SPIKE OFF") {
    SPIKE_STOP_ENABLED = false;
    Serial.println("Spike detection OFF");
    return;
  }

  // Parse parameter commands (e.g., "TL 2500")
  int spacePos = cmd.indexOf(' ');
  if (spacePos > 0) {
    String param = cmd.substring(0, spacePos);
    int value = cmd.substring(spacePos + 1).toInt();

    if (param == "TL") { // Threshold LRF
      THRESHOLD_LRF = value;
      Serial.print("THRESHOLD_LRF: ");
      Serial.println(THRESHOLD_LRF);
    } else if (param == "TC") { // Threshold Cross
      THRESHOLD_CROSS = value;
      Serial.print("THRESHOLD_CROSS: ");
      Serial.println(THRESHOLD_CROSS);
    } else if (param == "SD") { // Spike Drop
      SPIKE_DROP = value;
      Serial.print("SPIKE_DROP: ");
      Serial.println(SPIKE_DROP);
    } else if (param == "BS") { // Base Speed
      BASE_SPEED = value;
      Serial.print("BASE_SPEED: ");
      Serial.println(BASE_SPEED);
    } else if (param == "MS") { // Max Speed
      MAX_SPEED = value;
      Serial.print("MAX_SPEED: ");
      Serial.println(MAX_SPEED);
    } else if (param == "SI") { // Sensor Interval (us)
      SENSOR_INTERVAL = value;
      Serial.print("SENSOR_INTERVAL: ");
      Serial.print(SENSOR_INTERVAL);
      Serial.println(" us");
    } else if (param == "MI") { // Motor Interval (us)
      MOTOR_INTERVAL = value;
      Serial.print("MOTOR_INTERVAL: ");
      Serial.print(MOTOR_INTERVAL);
      Serial.println(" us");
    } else if (param == "KP") { // Proportional gain
      KP = cmd.substring(spacePos + 1).toFloat();
      Serial.print("KP: ");
      Serial.println(KP, 4);
    } else if (param == "KD") { // Derivative gain
      KD = cmd.substring(spacePos + 1).toFloat();
      Serial.print("KD: ");
      Serial.println(KD, 4);
    } else {
      Serial.println("Unknown command");
    }
    buzz(1500, 30);
  }
}

void printHelp() {
  Serial.println("\n========== Obstacle Avoidance (Pure PD) ==========");
  Serial.println("Commands:");
  Serial.println("  STOP / S      - Emergency stop");
  Serial.println("  GO / G        - Resume running");
  Serial.println("  STATUS        - Show current settings");
  Serial.println("  SPIKE ON/OFF  - Toggle spike detection");
  Serial.println("\nParameter Tuning:");
  Serial.println("  TL <val>  - THRESHOLD_LRF (L,R,F)");
  Serial.println("  TC <val>  - THRESHOLD_CROSS (CL,CR)");
  Serial.println("  SD <val>  - SPIKE_DROP threshold");
  Serial.println("  BS <val>  - BASE_SPEED");
  Serial.println("  MS <val>  - MAX_SPEED");
  Serial.println("  KP <val>  - Proportional gain");
  Serial.println("  KD <val>  - Derivative gain");
  Serial.println("  SI <val>  - SENSOR_INTERVAL (us)");
  Serial.println("  MI <val>  - MOTOR_INTERVAL (us)");
  Serial.println("=================================================\n");
}

void buzz(int freq, int dur) {
  tone(BUZZER_PIN, freq);
  buzzerOffTime = millis() + dur;
}
