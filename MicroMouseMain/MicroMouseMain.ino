/*
 * Right Wall Follower PRO - MicroMouse (FreeRTOS)
 *
 * RTOS Task Layout:
 *   HIGH   : Sensor Reading, PID Calculation
 *   MEDIUM : Motor Control
 *   LOW    : Display, Buzzer, NeoPixels
 *
 * Algorithm: Adaptive PID + Corner Detection (from RightLineFollower)
 * Visual Feedback: NeoPixels + OLED (from MMRF)
 * Sync: Mutexes for shared data; queue for buzzer requests.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <Adafruit_GFX.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <string.h>

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

// Buttons
#define BTN_MODE 5   // D5: Change Mode
#define BTN_ACTION 2 // D2: Action/Calibrate

// Encoders
#define ENC_L_C1 18 // D18
#define ENC_L_C2 19 // D19
#define ENC_R_C1 27 // D27
#define ENC_R_C2 23 // D23

// OLED Display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1

// ========== FreeRTOS Configuration ==========
#define TASK_PRIORITY_HIGH 5
#define TASK_PRIORITY_MEDIUM 3
#define TASK_PRIORITY_LOW 1

#define TASK_STACK_SENSOR 2048
#define TASK_STACK_PID 3072
#define TASK_STACK_MOTOR 2048
#define TASK_STACK_UI 4096

#define SENSOR_PERIOD_MS 1 // 1 kHz sensor loop
#define PID_PERIOD_MS 5    // 200 Hz PID loop
#define MOTOR_PERIOD_MS 5  // 200 Hz motor loop
#define UI_PERIOD_MS 200   // 5 Hz display/LED/buzzer

#define BUZZ_QUEUE_LEN 4

// Corner turn state (shared via control data)
enum TurnState { STATE_FOLLOWING, STATE_CORNER_TURN };

// Main Operation Modes
enum ControlMode {
  MODE_RIGHT_WALL,
  MODE_LEFT_WALL,
  MODE_OBSTACLE,
  MODE_MAPPING,
  MODE_IDLE
};

// Shared sensor data (written by Sensor task, read by PID task)
typedef struct {
  int L, CL, F, CR, R;
  int prevL, prevCL, prevF, prevCR, prevR;
  bool spikeDetected;
  float yaw;   // Wrapped 0-360
  float gyroZ; // Rad/s
  long encLeft;
  long encRight;
} SensorData_t;

// Shared control data (written by PID/Sensor on spike, read by Motor + UI)
typedef struct {
  int targetLeft;
  int targetRight;
  bool emergencyStop;
  char currentStateStr[16];
  char currentModeStr[16]; // Increased size
  TurnState turnState;
  ControlMode controlMode; // Active Mode
  bool aggressiveMode;
  bool spikeDetected;
} ControlData_t;

// Buzzer request (any task -> UI task)
typedef struct {
  int freq;
  int durationMs;
} BuzzCmd_t;

static SensorData_t s_sensor = {0};
static ControlData_t s_control = {0};

static SemaphoreHandle_t s_mutexSensor = NULL;
static SemaphoreHandle_t s_mutexControl = NULL;
static QueueHandle_t s_queueBuzz = NULL;

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
float KP_WALL = 0.13; // Gentle wall following
float KD_WALL = 1.8;
float KP_TURN = 2.7; // Aggressive corners/avoidance
float KD_TURN = 1.3;

// Active gains
float KP = KP_WALL;
float KD = KD_WALL;

// Corner turn parameters
int CORNER_TURN_DURATION = 250;
int CORNER_TURN_SPEED_L = 180;
int CORNER_TURN_SPEED_R = 80;
int CORNER_DETECT_TIME = 200;

// ========== Global Objects ==========
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_MPU6050 mpu;

// Encoder Volatiles
volatile long volatileEncLeft = 0;
volatile long volatileEncRight = 0;

// Button States
int lastBtnModeState = HIGH;
int lastBtnActionState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long buttonHoldTime = 0;

// IMU Calibration
float gyroOpsetZ = 0;
float currentHeading = 0;

// PID-internal state (only touched by PID task)
static int s_lastSteerError = 0;
static unsigned long s_cornerTurnStartTime = 0;
static unsigned long s_lastWallSeenTime = 0;
static bool s_wallWasPresent = false;
static TurnState s_turnState = STATE_FOLLOWING;

// ========== ISRs for Encoders ==========
void IRAM_ATTR isrEncLeft() {
  if (digitalRead(ENC_L_C1) == digitalRead(ENC_L_C2))
    volatileEncLeft++;
  else
    volatileEncLeft--;
}

void IRAM_ATTR isrEncRight() {
  if (digitalRead(ENC_R_C1) == digitalRead(ENC_R_C2))
    volatileEncRight--;
  else
    volatileEncRight++;
}

// ========== Buzzer request (thread-safe, for PID/Sensor tasks) ==========
static void buzzRequest(int freq, int durationMs) {
  BuzzCmd_t cmd = {.freq = freq, .durationMs = durationMs};
  if (s_queueBuzz != NULL)
    xQueueSend(s_queueBuzz, &cmd, 0);
}

// ========== Task: Sensor Reading (HIGH priority) ==========
static void taskSensor(void *arg) {
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    if (xSemaphoreTake(s_mutexSensor, pdMS_TO_TICKS(10)) == pdTRUE) {
      s_sensor.prevL = s_sensor.L;
      s_sensor.prevCL = s_sensor.CL;
      s_sensor.prevF = s_sensor.F;
      s_sensor.prevCR = s_sensor.CR;
      s_sensor.prevR = s_sensor.R;
      s_sensor.L = analogRead(SENSOR_LEFT);
      s_sensor.CL = analogRead(SENSOR_CROSS_LEFT);
      s_sensor.F = analogRead(SENSOR_FRONT);
      s_sensor.CR = analogRead(SENSOR_CROSS_RIGHT);
      s_sensor.R = analogRead(SENSOR_RIGHT);

      // Read MPU6050
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      s_sensor.gyroZ = g.gyro.z;
      // Integrate Yaw (Basic Euler, assuming steady sample rate)
      // SENSOR_PERIOD_MS is 1ms -> 0.001s
      s_sensor.yaw +=
          s_sensor.gyroZ * (float)(SENSOR_PERIOD_MS) / 1000.0f * 180.0f / PI;
      if (s_sensor.yaw > 360)
        s_sensor.yaw -= 360;
      if (s_sensor.yaw < 0)
        s_sensor.yaw += 360;

      // Read Encoders
      s_sensor.encLeft = volatileEncLeft;
      s_sensor.encRight = volatileEncRight;

      s_sensor.spikeDetected = false;
      if (SPIKE_STOP_ENABLED) {
        if (s_sensor.prevL - s_sensor.L > SPIKE_DROP)
          s_sensor.spikeDetected = true;
        if (s_sensor.prevCL - s_sensor.CL > SPIKE_DROP)
          s_sensor.spikeDetected = true;
        if (s_sensor.prevF - s_sensor.F > SPIKE_DROP)
          s_sensor.spikeDetected = true;
        if (s_sensor.prevCR - s_sensor.CR > SPIKE_DROP)
          s_sensor.spikeDetected = true;
        if (s_sensor.prevR - s_sensor.R > SPIKE_DROP)
          s_sensor.spikeDetected = true;
      }

      if (s_sensor.spikeDetected) {
        xSemaphoreGive(s_mutexSensor);
        if (xSemaphoreTake(s_mutexControl, pdMS_TO_TICKS(10)) == pdTRUE) {
          s_control.targetLeft = -100;
          s_control.targetRight = -100;
          s_control.spikeDetected = true;
          strncpy(s_control.currentStateStr, "SPIKE!",
                  sizeof(s_control.currentStateStr) - 1);
          s_control.currentStateStr[sizeof(s_control.currentStateStr) - 1] =
              '\0';
          xSemaphoreGive(s_mutexControl);
        }
        buzzRequest(2000, 50);
      } else {
        xSemaphoreGive(s_mutexSensor);
      }
    }
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
  }
}

// ========== Task: PID Calculation (HIGH priority) ==========
static void taskPID(void *arg) {
  TickType_t lastWake = xTaskGetTickCount();
  SensorData_t localSensor;
  ControlData_t localControl;

  for (;;) {
    if (xSemaphoreTake(s_mutexSensor, pdMS_TO_TICKS(10)) == pdTRUE) {
      memcpy(&localSensor, &s_sensor, sizeof(SensorData_t));
      xSemaphoreGive(s_mutexSensor);
    }

    computeControl(&localSensor, &localControl);

    if (xSemaphoreTake(s_mutexControl, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (!localSensor.spikeDetected) {
        s_control.targetLeft = localControl.targetLeft;
        s_control.targetRight = localControl.targetRight;
        s_control.turnState = localControl.turnState;
        s_control.controlMode = localControl.controlMode;
        s_control.aggressiveMode = localControl.aggressiveMode;
        s_control.spikeDetected = false;
        strncpy(s_control.currentStateStr, localControl.currentStateStr,
                sizeof(s_control.currentStateStr) - 1);
        s_control.currentStateStr[sizeof(s_control.currentStateStr) - 1] = '\0';
        strncpy(s_control.currentModeStr, localControl.currentModeStr,
                sizeof(s_control.currentModeStr) - 1);
        s_control.currentModeStr[sizeof(s_control.currentModeStr) - 1] = '\0';
      }
      xSemaphoreGive(s_mutexControl);
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PID_PERIOD_MS));
  }
}

// ========== Task: Motor Control (MEDIUM priority) ==========
static void taskMotor(void *arg) {
  TickType_t lastWake = xTaskGetTickCount();
  int left = 0, right = 0;
  bool stop = false;

  for (;;) {
    if (xSemaphoreTake(s_mutexControl, pdMS_TO_TICKS(10)) == pdTRUE) {
      left = s_control.targetLeft;
      right = s_control.targetRight;
      stop = s_control.emergencyStop;
      xSemaphoreGive(s_mutexControl);
    }
    if (stop) {
      left = 0;
      right = 0;
    }
    applyMotors(left, right);
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MOTOR_PERIOD_MS));
  }
}

// ========== Task: Display, Buzzer, Lights (LOW priority) ==========
static void taskUI(void *arg) {
  unsigned long buzzerOffTime = 0;
  BuzzCmd_t cmd;
  SensorData_t snapSensor;
  ControlData_t snapControl;
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    // Poll Buttons
    int btnMode = digitalRead(BTN_MODE);
    int btnAction = digitalRead(BTN_ACTION);
    unsigned long now = millis();

    if (now - lastDebounceTime > 50) {
      // D5 Logic
      if (btnMode == LOW && lastBtnModeState == HIGH) {
        if (xSemaphoreTake(s_mutexControl, pdMS_TO_TICKS(10)) == pdTRUE) {
          int nm = (int)s_control.controlMode + 1;
          if (nm > MODE_MAPPING)
            nm = MODE_RIGHT_WALL;
          s_control.controlMode = (ControlMode)nm;
          buzzRequest(4000, 50);
          xSemaphoreGive(s_mutexControl);
        }
      }
      lastBtnModeState = btnMode;

      // D2 Logic
      if (btnAction == LOW && lastBtnActionState == HIGH) {
        if (xSemaphoreTake(s_mutexControl, pdMS_TO_TICKS(10)) == pdTRUE) {
          if (s_control.controlMode == MODE_MAPPING) {
            s_control.emergencyStop = !s_control.emergencyStop;
            if (!s_control.emergencyStop) {
              // Started Mapping
            }
          } else {
            s_control.emergencyStop = !s_control.emergencyStop;
          }
          buzzRequest(s_control.emergencyStop ? 1000 : 3000, 100);
          xSemaphoreGive(s_mutexControl);
        }
      }
      lastBtnActionState = btnAction;
      lastDebounceTime = now;
    }

    // Drain buzz queue
    while (s_queueBuzz != NULL &&
           xQueueReceive(s_queueBuzz, &cmd, 0) == pdTRUE) {
      tone(BUZZER_PIN, cmd.freq);
      buzzerOffTime = millis() + (unsigned long)cmd.durationMs;
    }
    if (buzzerOffTime > 0 && millis() >= buzzerOffTime) {
      noTone(BUZZER_PIN);
      buzzerOffTime = 0;
    }

    // Snapshot
    if (xSemaphoreTake(s_mutexSensor, pdMS_TO_TICKS(5)) == pdTRUE) {
      memcpy(&snapSensor, &s_sensor, sizeof(SensorData_t));
      xSemaphoreGive(s_mutexSensor);
      if (xSemaphoreTake(s_mutexControl, pdMS_TO_TICKS(5)) == pdTRUE) {
        memcpy(&snapControl, &s_control, sizeof(ControlData_t));
        xSemaphoreGive(s_mutexControl);
      }
    }
    updateDisplaySnapshot(&snapSensor, &snapControl);
    updateVisualsSnapshot(&snapControl);

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(UI_PERIOD_MS));
  }
}

// ========== Setup ==========
void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_L_PWM, OUTPUT);
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_PWM, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);
  pinMode(MOTOR_STBY, OUTPUT);
  digitalWrite(MOTOR_STBY, HIGH);

  pinMode(SENSOR_RIGHT, INPUT);
  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_FRONT, INPUT);
  pinMode(SENSOR_CROSS_LEFT, INPUT);
  pinMode(SENSOR_CROSS_RIGHT, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Buttons
  pinMode(BTN_MODE, INPUT_PULLUP);
  pinMode(BTN_ACTION, INPUT_PULLUP);

  // Encoders
  pinMode(ENC_L_C1, INPUT_PULLUP);
  pinMode(ENC_L_C2, INPUT_PULLUP);
  pinMode(ENC_R_C1, INPUT_PULLUP);
  pinMode(ENC_R_C2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_C1), isrEncLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_C1), isrEncRight, RISING);

  // MPU6050 Init
  Wire.begin(); // Ensure Wire is started
  if (!mpu.begin()) {
    Serial.println("MPU6050 Not Found!");
  } else {
    Serial.println("MPU6050 Found!");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  pixels.begin();
  pixels.setBrightness(50);
  setAllPixels(0, 0, 255);
  pixels.show();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Right Wall Pro");
  display.println("FreeRTOS");
  display.println("Init...");
  display.display();

  tone(BUZZER_PIN, 2000, 100);
  delay(200);
  tone(BUZZER_PIN, 2500, 100);
  delay(200);
  tone(BUZZER_PIN, 3000, 100);
  delay(500);
  setAllPixels(0, 255, 0);
  pixels.show();

  // Init shared state
  memset(s_control.currentStateStr, 0, sizeof(s_control.currentStateStr));
  memset(s_control.currentModeStr, 0, sizeof(s_control.currentModeStr));
  strncpy(s_control.currentStateStr, "INIT",
          sizeof(s_control.currentStateStr) - 1);
  strncpy(s_control.currentModeStr, "WALL",
          sizeof(s_control.currentModeStr) - 1);
  s_control.turnState = STATE_FOLLOWING;
  s_control.emergencyStop = true; // Start in stop mode
  s_control.controlMode = MODE_IDLE;
  s_control.spikeDetected = false;
  strncpy(s_control.currentModeStr, "IDLE",
          sizeof(s_control.currentModeStr) - 1);

  s_mutexSensor = xSemaphoreCreateMutex();
  s_mutexControl = xSemaphoreCreateMutex();
  s_queueBuzz = xQueueCreate(BUZZ_QUEUE_LEN, sizeof(BuzzCmd_t));
  if (s_mutexSensor == NULL || s_mutexControl == NULL || s_queueBuzz == NULL) {
    Serial.println("FATAL: RTOS objects creation failed");
    for (;;)
      delay(1000);
  }

  xTaskCreatePinnedToCore(taskSensor, "sensor", TASK_STACK_SENSOR, NULL,
                          TASK_PRIORITY_HIGH, NULL, 0);
  xTaskCreatePinnedToCore(taskPID, "pid", TASK_STACK_PID, NULL,
                          TASK_PRIORITY_HIGH, NULL, 0);
  xTaskCreatePinnedToCore(taskMotor, "motor", TASK_STACK_MOTOR, NULL,
                          TASK_PRIORITY_MEDIUM, NULL, 0);
  xTaskCreatePinnedToCore(taskUI, "ui", TASK_STACK_UI, NULL, TASK_PRIORITY_LOW,
                          NULL, 1);

  Serial.println("\n===== Right Wall Follower PRO (FreeRTOS) =====");
  Serial.println("HIGH: Sensor, PID | MED: Motor | LOW: Display, Buzzer, LEDs");
  Serial.println("Type HELP for commands");
  Serial.println("=============================================\n");
  delay(1500);
}

// ========== Main Loop (Serial only, runs at default priority) ==========
void loop() {
  if (Serial.available() > 0)
    processCommand();
  vTaskDelay(pdMS_TO_TICKS(10));
}

// ========== PID Control Logic (runs in PID task, uses shared snapshot)
// ==========
static void computeControl(const SensorData_t *sens, ControlData_t *out) {
  if (sens->spikeDetected) {
    out->targetLeft = -100;
    out->targetRight = -100;
    strncpy(out->currentStateStr, "SPIKE!", sizeof(out->currentStateStr) - 1);
    out->currentStateStr[sizeof(out->currentStateStr) - 1] = '\0';
    return;
  }

  // Determine Mode string
  switch (out->controlMode) {
  case MODE_RIGHT_WALL:
    strncpy(out->currentModeStr, "R-WALL", 15);
    break;
  case MODE_LEFT_WALL:
    strncpy(out->currentModeStr, "L-WALL", 15);
    break;
  case MODE_OBSTACLE:
    strncpy(out->currentModeStr, "AVOID", 15);
    break;
  case MODE_MAPPING:
    strncpy(out->currentModeStr, "MAP", 15);
    break;
  default:
    strncpy(out->currentModeStr, "IDLE", 15);
    break;
  }
  out->currentModeStr[15] = '\0';

  if (out->emergencyStop) {
    out->targetLeft = 0;
    out->targetRight = 0;
    strncpy(out->currentStateStr, "STOPPED", sizeof(out->currentStateStr) - 1);
    out->currentStateStr[sizeof(out->currentStateStr) - 1] = '\0';
    return;
  }

  // Parameter Extraction
  int L = sens->L, CL = sens->CL, F = sens->F, CR = sens->CR, R = sens->R;
  unsigned long now = millis();

  // --- MODE SPECIFIC LOGIC ---
  int WallL = L, WallCL = CL, WallF = F, WallCR = CR, WallR = R;
  int WallVal = R;
  int CrossVal = CR;
  int Setpoint = RIGHT_SETPOINT;
  int CrossSetpoint = CROSS_RIGHT_SETPOINT;
  float Kp = KP_WALL, Kd = KD_WALL;
  bool isLeftMode = (out->controlMode == MODE_LEFT_WALL);

  if (isLeftMode) {
    WallVal = L;
    CrossVal = CL;
    // Assuming symmetric sensors roughly
    // Using same setpoints for simplicity if sensors match
  }

  // Obstacle Avoidance specific
  if (out->controlMode == MODE_OBSTACLE) {
    // Simple Avoidance:
    // If F > THRESHOLD -> Turn Left/Right depending on space
    // Else -> Go Straight
    if (F > THRESHOLD_LRF) {
      if (L < R) { // More space on Left or actually Right sensor is giving low
                   // reading meaning space?
        // IR Sensors: High Value = Close Obstacle. Low Value = Far.
        // So L < R means Left is Further (more space).
        out->targetLeft = -100;
        out->targetRight = 100;
      } else {
        out->targetLeft = 100;
        out->targetRight = -100;
      }
      strncpy(out->currentStateStr, "OBSTACLE", 15);
      return;
    }
    out->targetLeft = BASE_SPEED;
    out->targetRight = BASE_SPEED;
    strncpy(out->currentStateStr, "CRUISE", 15);
    return;
  }

  // Mapping Mode specific (Default to R-Wall for now)
  if (out->controlMode == MODE_MAPPING) {
    // Default to Right Wall with Encoder logging happening in background
    // Just fall through to Wall Following logic
  }

  bool wallPresent =
      (WallVal > WALL_PRESENT_MIN) && (CrossVal > WALL_PRESENT_MIN);
  int prevWallVal = isLeftMode ? sens->prevL : sens->prevR;

  if (wallPresent) {
    s_wallWasPresent = true;
    s_lastWallSeenTime = now;
  }

  switch (s_turnState) {
  case STATE_FOLLOWING:
    if (s_wallWasPresent && !wallPresent) {
      unsigned long timeSinceWallLost = now - s_lastWallSeenTime;
      if (timeSinceWallLost < (unsigned long)CORNER_DETECT_TIME) {
        s_turnState = STATE_CORNER_TURN;
        s_cornerTurnStartTime = now;
        strncpy(out->currentStateStr, "CORNER",
                sizeof(out->currentStateStr) - 1);
        out->currentStateStr[sizeof(out->currentStateStr) - 1] = '\0';
        buzzRequest(1800, 80);
      }
    }
    break;

  case STATE_CORNER_TURN:
    // Left turn for R-Wall, Right turn for L-Wall
    if (isLeftMode) {
      out->targetLeft = CORNER_TURN_SPEED_R; // Turn Right
      out->targetRight = CORNER_TURN_SPEED_L;
    } else {
      out->targetLeft = CORNER_TURN_SPEED_L; // Turn Left
      out->targetRight = CORNER_TURN_SPEED_R;
    }

    out->turnState = s_turnState;
    strncpy(out->currentStateStr, "CORNER", sizeof(out->currentStateStr) - 1);
    out->currentStateStr[sizeof(out->currentStateStr) - 1] = '\0';
    strncpy(out->currentModeStr, "WALL", sizeof(out->currentModeStr) - 1);
    out->currentModeStr[sizeof(out->currentModeStr) - 1] = '\0';
    out->aggressiveMode = false;
    if (now - s_cornerTurnStartTime > (unsigned long)CORNER_TURN_DURATION) {
      s_turnState = STATE_FOLLOWING;
      s_wallWasPresent = false;
      buzzRequest(1500, 50);
    }
    return;
  }

  out->turnState = s_turnState;

  // Adaptive PID gain switching
  bool aggressive = false;
  if (F > 1200)
    aggressive = true;
  if (wallPresent) {
    if (abs(CrossVal - WallVal) > 800)
      aggressive = true; // Use abs for general case?
  }
  // Simplified logic for brevity, keeping original structure where possible
  if (WallVal < WALL_PRESENT_MIN && prevWallVal > WALL_PRESENT_MIN + 200)
    aggressive = true;

  float kp = aggressive ? KP_TURN : KP_WALL;
  float kd = aggressive ? KD_TURN : KD_WALL;
  out->aggressiveMode = aggressive;
  strncpy(out->currentModeStr, aggressive ? "TURN" : "WALL",
          sizeof(out->currentModeStr) - 1);
  out->currentModeStr[sizeof(out->currentModeStr) - 1] = '\0';

  int frontStart = 600;
  int frontVal = constrain(F, frontStart, THRESHOLD_LRF);
  float frontFactor =
      (float)map(frontVal, frontStart, THRESHOLD_LRF, 0, 1000) / 1000.0f;
  int desiredSpeed = (int)(BASE_SPEED * (1.0f - frontFactor));

  // Push away from other wall if too close (Centering instinct)
  int otherSidePush = 0;
  // If R-Wall following, check L sensors to avoid hitting left wall
  // Logic from original:
  /*
  int leftPush = 0;
  if (CL > 600) leftPush += ...
  if (L > 600) leftPush += ...
  */
  // We need to generalize this "Push"
  // If following Right, Push comes from Left sensors.
  // If following Left, Push comes from Right sensors.

  int PushL = L, PushCL = CL; // Default for Right Wall follow (Push from Left)
  if (isLeftMode) {
    PushL = R;
    PushCL = CR; // If Left Wall follow, Push from Right
  }

  if (PushCL > 600)
    otherSidePush += map(constrain(PushCL, 600, THRESHOLD_CROSS), 600,
                         THRESHOLD_CROSS, 0, 80);
  if (PushL > 600)
    otherSidePush +=
        map(constrain(PushL, 600, THRESHOLD_LRF), 600, THRESHOLD_LRF, 0, 120);

  int wallError = 0;
  if (wallPresent) {
    wallError =
        (WallVal - Setpoint) +
        (CrossVal - CrossSetpoint); // Positive Error = Too Close to Wall
    strncpy(out->currentStateStr, "FOLLOW", sizeof(out->currentStateStr) - 1);
  } else {
    wallError = -200; // Too Far (lost wall) -> Turn towards wall
    strncpy(out->currentStateStr, "SEARCH", sizeof(out->currentStateStr) - 1);
  }

  if (wallPresent && s_turnState == STATE_FOLLOWING)
    strncpy(out->currentStateStr, "FOLLOW", sizeof(out->currentStateStr) - 1);
  out->currentStateStr[sizeof(out->currentStateStr) - 1] = '\0';

  // Steering:
  // If Right Wall: Too Close (Positive Error) -> Turn Left. Too Far (Negative)
  // -> Turn Right. Turn Left = Left Motor < Right Motor. diff = Right - Left.
  // Steer = -Error. (If Error > 0, Steer < 0 -> Turn Left).
  // Plus "Push" from other side (Left sensors). If Left sensors see wall, we
  // want to Turn Right (Increase Steer). TotalSteer = OtherSidePush -
  // WallError.

  // If Left Wall: Too Close (Left Wall) -> Turn Right.
  // Turn Right = Left > Right. Steer > 0.
  // So Error > 0 -> Steer > 0.
  // "Push" from Right Sensors -> Turn Left (Steer < 0).
  // TotalSteer = WallError - OtherSidePush.

  int rawSteer = 0;
  if (!isLeftMode) {
    rawSteer = otherSidePush - wallError;
  } else {
    rawSteer = wallError - otherSidePush;
  }

  int steerDeriv = rawSteer - s_lastSteerError;
  s_lastSteerError = rawSteer;
  int totalSteer = (int)((rawSteer * kp) + (steerDeriv * kd));

  if (frontFactor > 0.8f) {
    strncpy(out->currentStateStr, "BLOCKED", sizeof(out->currentStateStr) - 1);
    out->currentStateStr[sizeof(out->currentStateStr) - 1] = '\0';
    // If blocked, turn away from wall? Or just spin?
    // Original: totalSteer = (L < R) ? -150 : 150; (Turn towards open space)
    totalSteer = (L < R) ? -150 : 150;
  }

  out->targetLeft = constrain(desiredSpeed + totalSteer, -MAX_SPEED, MAX_SPEED);
  out->targetRight =
      constrain(desiredSpeed - totalSteer, -MAX_SPEED, MAX_SPEED);
}

// ========== Visual Feedback (UI task, uses snapshot) ==========
static void updateVisualsSnapshot(const ControlData_t *c) {
  if (c->emergencyStop || c->spikeDetected)
    setAllPixels(255, 0, 0);
  else if (c->turnState == STATE_CORNER_TURN)
    setAllPixels(255, 0, 255);
  else if (strcmp(c->currentStateStr, "BLOCKED") == 0)
    setAllPixels(255, 100, 0);
  else if (strcmp(c->currentStateStr, "SEARCH") == 0)
    setAllPixels(0, 100, 255);
  else if (c->aggressiveMode)
    setAllPixels(255, 255, 0);
  else
    setAllPixels(0, 255, 100);
  pixels.show();
}

// Set all NeoPixels to same color
void setAllPixels(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NEOPIXEL_COUNT; i++) {
    pixels.setPixelColor(i, pixels.Color(r, g, b));
  }
}

// ========== OLED Display Update (UI task, uses snapshot) ==========
static void updateDisplaySnapshot(const SensorData_t *sens,
                                  const ControlData_t *c) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("St:");
  display.print(c->currentStateStr);
  display.print(" Md:");
  display.println(c->currentModeStr);
  display.print("F:");
  display.print(sens->F);
  display.print(" R:");
  display.print(sens->R);
  display.print(" L:");
  display.println(sens->L);
  display.print("CR:");
  display.print(sens->CR);
  display.print(" CL:");
  display.println(sens->CL);
  display.print("M: ");
  display.print(c->targetLeft);
  display.print(",");
  display.println(c->targetRight);
  display.display();
}

// ========== Motor Control (Motor task calls with snapshot values) ==========
static void applyMotors(int left, int right) {
  if (left >= 0) {
    digitalWrite(MOTOR_L_IN1, LOW);
    digitalWrite(MOTOR_L_IN2, HIGH);
    analogWrite(MOTOR_L_PWM, left);
  } else {
    digitalWrite(MOTOR_L_IN1, HIGH);
    digitalWrite(MOTOR_L_IN2, LOW);
    analogWrite(MOTOR_L_PWM, -left);
  }
  if (right >= 0) {
    digitalWrite(MOTOR_R_IN1, LOW);
    digitalWrite(MOTOR_R_IN2, HIGH);
    analogWrite(MOTOR_R_PWM, right);
  } else {
    digitalWrite(MOTOR_R_IN1, HIGH);
    digitalWrite(MOTOR_R_IN2, LOW);
    analogWrite(MOTOR_R_PWM, -right);
  }
}

// Buzzer is handled by UI task via buzzRequest() / s_queueBuzz

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
    if (s_mutexControl != NULL &&
        xSemaphoreTake(s_mutexControl, pdMS_TO_TICKS(50)) == pdTRUE) {
      s_control.emergencyStop = true;
      s_control.targetLeft = 0;
      s_control.targetRight = 0;
      xSemaphoreGive(s_mutexControl);
    }
    Serial.println("EMERGENCY STOP");
    buzzRequest(3000, 200);
    return;
  }

  if (cmd == "GO" || cmd == "G") {
    if (s_mutexControl != NULL &&
        xSemaphoreTake(s_mutexControl, pdMS_TO_TICKS(50)) == pdTRUE) {
      s_control.emergencyStop = false;
      xSemaphoreGive(s_mutexControl);
    }
    Serial.println("RUNNING");
    buzzRequest(1500, 100);
    return;
  }

  if (cmd == "STATUS") {
    char st[16], md[8];
    if (s_mutexControl != NULL &&
        xSemaphoreTake(s_mutexControl, pdMS_TO_TICKS(50)) == pdTRUE) {
      strncpy(st, s_control.currentStateStr, sizeof(st) - 1);
      st[sizeof(st) - 1] = '\0';
      strncpy(md, s_control.currentModeStr, sizeof(md) - 1);
      md[sizeof(md) - 1] = '\0';
      xSemaphoreGive(s_mutexControl);
    } else {
      st[0] = md[0] = '\0';
    }
    Serial.println("\n===== Current Settings =====");
    Serial.print("STATE: ");
    Serial.println(st);
    Serial.print("MODE: ");
    Serial.println(md);
    Serial.println("\nADAPTIVE PID GAINS:");
    Serial.print("  WALL: KP=");
    Serial.print(KP_WALL, 2);
    Serial.print(", KD=");
    Serial.println(KD_WALL, 2);
    Serial.print("  TURN: KP=");
    Serial.print(KP_TURN, 2);
    Serial.print(", KD=");
    Serial.println(KD_TURN, 2);
    Serial.println("\nCORNER TURN:");
    Serial.print("  DURATION: ");
    Serial.print(CORNER_TURN_DURATION);
    Serial.println(" ms");
    Serial.print("  SPEED_L: ");
    Serial.println(CORNER_TURN_SPEED_L);
    Serial.print("  SPEED_R: ");
    Serial.println(CORNER_TURN_SPEED_R);
    Serial.println("\nWALL FOLLOWING:");
    Serial.print("  RIGHT_SETPOINT: ");
    Serial.println(RIGHT_SETPOINT);
    Serial.print("  BASE_SPEED: ");
    Serial.println(BASE_SPEED);
    Serial.println("============================\n");
    return;
  }

  if (cmd == "SPIKE ON") {
    SPIKE_STOP_ENABLED = true;
    Serial.println("Spike detection ON");
    buzzRequest(1500, 30);
    return;
  }

  if (cmd == "SPIKE OFF") {
    SPIKE_STOP_ENABLED = false;
    Serial.println("Spike detection OFF");
    buzzRequest(1500, 30);
    return;
  }

  // Parameter commands
  int spacePos = cmd.indexOf(' ');
  if (spacePos > 0) {
    String param = cmd.substring(0, spacePos);
    String valueStr = cmd.substring(spacePos + 1);

    if (param == "RS") {
      RIGHT_SETPOINT = valueStr.toInt();
      Serial.print("RIGHT_SETPOINT: ");
      Serial.println(RIGHT_SETPOINT);
    } else if (param == "CRS") {
      CROSS_RIGHT_SETPOINT = valueStr.toInt();
      Serial.print("CROSS_RIGHT_SETPOINT: ");
      Serial.println(CROSS_RIGHT_SETPOINT);
    } else if (param == "BS") {
      BASE_SPEED = valueStr.toInt();
      Serial.print("BASE_SPEED: ");
      Serial.println(BASE_SPEED);
    } else if (param == "KPW") {
      KP_WALL = valueStr.toFloat();
      Serial.print("KP_WALL: ");
      Serial.println(KP_WALL, 2);
    } else if (param == "KDW") {
      KD_WALL = valueStr.toFloat();
      Serial.print("KD_WALL: ");
      Serial.println(KD_WALL, 2);
    } else if (param == "KPT") {
      KP_TURN = valueStr.toFloat();
      Serial.print("KP_TURN: ");
      Serial.println(KP_TURN, 2);
    } else if (param == "KDT") {
      KD_TURN = valueStr.toFloat();
      Serial.print("KD_TURN: ");
      Serial.println(KD_TURN, 2);
    } else if (param == "CTD") {
      CORNER_TURN_DURATION = valueStr.toInt();
      Serial.print("CORNER_TURN_DURATION: ");
      Serial.print(CORNER_TURN_DURATION);
      Serial.println(" ms");
    } else if (param == "CTL") {
      CORNER_TURN_SPEED_L = valueStr.toInt();
      Serial.print("CORNER_TURN_SPEED_L: ");
      Serial.println(CORNER_TURN_SPEED_L);
    } else if (param == "CTR") {
      CORNER_TURN_SPEED_R = valueStr.toInt();
      Serial.print("CORNER_TURN_SPEED_R: ");
      Serial.println(CORNER_TURN_SPEED_R);
    } else {
      Serial.println("Unknown parameter");
      return;
    }
    buzzRequest(1500, 30);
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
