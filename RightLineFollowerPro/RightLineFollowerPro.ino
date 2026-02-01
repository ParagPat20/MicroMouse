/*
 * Right Wall Follower PRO - MicroMouse (FreeRTOS)
 *
 * Menu: 1) FreeRun (sub: RWF/LWF/OA) - 3s countdown with beeps then run; BTN_MODE = stop & menu
 *       2) Record (sub: RWF/LWF/OA) - 3s countdown then record; BTN_MAP = save & menu, BTN_MODE = discard & menu
 *       3) Play - only if mission saved; 3s countdown then play (faster); BTN_MODE hold = stop & menu, BTN_MAP hold = delete & menu, BTN_MAP click = pause/resume
 *       4) Settings - PID and params; D5 change, D2 next/save to EEPROM
 * Startup: Harry Potter theme + light effect. Buzzer/LED by situation.
 */

#include <Adafruit_GFX.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include <string.h>

// ========== Pin Definitions (each GPIO used once) ==========
// Sensors
#define SENSOR_RIGHT 36
#define SENSOR_LEFT 39
#define SENSOR_FRONT 34
#define SENSOR_CROSS_LEFT 35
#define SENSOR_CROSS_RIGHT 32

// Motors (driver)
#define MOTOR_L_PWM 26
#define MOTOR_L_IN1 13
#define MOTOR_L_IN2 15
#define MOTOR_R_PWM 25
#define MOTOR_R_IN1 14
#define MOTOR_R_IN2 12
#define MOTOR_STBY 4

// Motor encoders (for mapping / dead reckoning)
#define LEFT_ENC_A  18   // C1
#define LEFT_ENC_B  19   // C2
#define RIGHT_ENC_A 27  // C1
#define RIGHT_ENC_B 23   // C2

// Buttons (INPUT_PULLUP; LOW = pressed)
#define BTN_MODE 5   // Mode: cycle R-Wall / L-Wall / OA
#define BTN_MAP  33  // Map: record/stop/follow; 

// Visual feedback
#define BUZZER_PIN 17
#define NEOPIXEL_PIN 16
#define NEOPIXEL_COUNT 6

// OLED Display (I2C: SDA/SCL usually 21/22)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1

// Pin summary: 4,5,12,13,14,15,16,17,18,19,23,25,26,27,32,34,35,36,39,33 (+ I2C)

// ========== FreeRTOS Configuration ==========
#define TASK_PRIORITY_HIGH   5
#define TASK_PRIORITY_MEDIUM 3
#define TASK_PRIORITY_LOW    1

#define TASK_STACK_SENSOR    2048
#define TASK_STACK_PID       3072
#define TASK_STACK_MOTOR     2048
#define TASK_STACK_UI        4096

#define SENSOR_PERIOD_MS     1    // 1 kHz sensor loop
#define PID_PERIOD_MS        5    // 200 Hz PID loop
#define MOTOR_PERIOD_MS      5    // 200 Hz motor loop
#define UI_PERIOD_MS         200  // 5 Hz display/LED/buzzer
#define BUTTON_PERIOD_MS     20   // 50 Hz button poll

#define BUZZ_QUEUE_LEN       4

#define BTN_DEBOUNCE_MS      50
#define BTN_HOLD_MS          600   // Long press threshold
#define COUNTDOWN_MS         3000  // 3s countdown with beeps (FreeRun/Record/Play)
#define EEPROM_SIZE          256
#define EEPROM_MAGIC         0xA5

#define PATH_MAX             300   // Max path segments (50ms each = 15s)
#define RECORD_INTERVAL_MS   50
#define REPLAY_SEGMENT_MS    50    // Recording replay step (ms)
#define REPLAY_PLAY_MS       25    // Play mission faster (25ms = 2x speed)

// Dead reckoning: mm per encoder tick (calibrate for your wheels; ~2.5 = 20 tick/rev, 50mm circum.)
#define MM_PER_TICK          2.5f
#define DEG_TO_RAD           0.0174532925f

// UI screens
enum UIScreen {
  SCREEN_MENU = 0,         // Main: FreeRun, Record, Play, Settings
  SCREEN_FREERUN_MODE,     // Sub: RWF, LWF, OA → D2 = 3s countdown then run
  SCREEN_RECORD_MODE,      // Sub: RWF, LWF, OA → D2 = 3s countdown then record
  SCREEN_COUNTDOWN,        // 3s countdown with beeps (shared)
  SCREEN_FREERUN,          // Free run active; BTN_MODE = stop & menu
  SCREEN_RECORDING,        // Recording; BTN_MAP = save & menu, BTN_MODE = discard & menu
  SCREEN_PLAY_INFO,        // Play submenu: waypoints, segments, dist, time, turns; D2 = Start 3s
  SCREEN_MISSION,          // Play mission; BTN_MODE hold = stop, BTN_MAP hold = delete, BTN_MAP click = pause
  SCREEN_SETTINGS,         // Settings list
  SCREEN_SETTINGS_ITEM     // Edit one param; D5 change, D2 next/save
};

// What countdown is for (set when entering SCREEN_COUNTDOWN)
enum CountdownReason {
  COUNTDOWN_FREERUN = 0,
  COUNTDOWN_RECORD,
  COUNTDOWN_PLAY
};

// Drive mode (D5 cycles)
enum DriveMode {
  MODE_RIGHT_WALL = 0,
  MODE_LEFT_WALL,
  MODE_OBSTACLE_AVOID,
  MODE_COUNT
};

// Mapping / mission state (D2 short: stop mapping -> start mission)
enum MapState {
  MAP_IDLE = 0,       // Normal run
  MAP_RECORDING,     // Recording path (encoders + heading)
  MAP_READY,         // Mapping done, ready to run mission
  MISSION_RUNNING    // Running stored path (future)
};

// Corner turn state (shared via control data)
enum TurnState {
  STATE_FOLLOWING,
  STATE_CORNER_TURN
};

// Path segment for recording/replay (speed profile + position; steering = IR wall-follow on play)
typedef struct {
  int16_t spdL;
  int16_t spdR;
  float   headingDeg;  // MPU6050 integrated gyro Z
  float   posXmm;     // Dead-reckoned X (mm)
  float   posYmm;     // Dead-reckoned Y (mm)
  float   cumDistMm;  // Cumulative distance from start to this segment (filled on save)
} PathSegment_t;

// 90° turn waypoint: at this distance (mm) execute a sharp turn (1=right, 0=left)
#define MAX_TURN_WAYPOINTS 32
typedef struct { float distMm; uint8_t turnRight; } TurnWaypoint_t;

// Shared sensor data (written by Sensor task, read by PID task)
typedef struct {
  int L, CL, F, CR, R;
  int prevL, prevCL, prevF, prevCR, prevR;
  bool spikeDetected;
  int32_t encLeft;
  int32_t encRight;
  float headingDeg;   // From MPU6050 (integrated gyro Z)
} SensorData_t;

// Shared control data (written by PID/Sensor/Buttons, read by Motor + UI)
typedef struct {
  int targetLeft;
  int targetRight;
  bool emergencyStop;
  char currentStateStr[16];
  char currentModeStr[8];
  TurnState turnState;
  bool aggressiveMode;
  bool spikeDetected;
  DriveMode driveMode;
  MapState mapState;
  float headingDeg;       // Current MPU6050 heading (deg)
  float lastPathHeading;  // Last recorded heading in path (for mapping UI)
  float mapXmm;           // Last recorded / current map X (mm) for UI
  float mapYmm;           // Last recorded / current map Y (mm) for UI
  float missionDistMm;    // Current mission distance (playback) for UI
  float missionTotalDistMm;// Total mission distance for UI
  int pathLen;
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

// Left wall following setpoints (mirror of right)
int LEFT_SETPOINT = 1200;
int CROSS_LEFT_SETPOINT = 1500;

// Left corner turn (turn left when left wall lost)
int LEFT_CORNER_TURN_DURATION = 250;
int LEFT_CORNER_TURN_SPEED_L = 80;   // Left slower
int LEFT_CORNER_TURN_SPEED_R = 180;

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

// Corner turn parameters: forward at CORNER_FORWARD_SPEED for CT_WAIT ms, then turn for CORNER_TURN_DURATION
int CT_WAIT = 150;                  // ms to go forward at 80 before taking the turn
int CORNER_FORWARD_SPEED = 80;     // speed during corner forward phase (both wheels)
int CORNER_TURN_DURATION = 250;
int CORNER_TURN_SPEED_L = 180;
int CORNER_TURN_SPEED_R = 80;
int CORNER_DETECT_TIME = 200;

// ========== Global Objects ==========
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// PID-internal state (only touched by PID task)
static int s_lastSteerError = 0;
static unsigned long s_cornerTurnStartTime = 0;
static unsigned long s_lastWallSeenTime = 0;
static bool s_wallWasPresent = false;
static TurnState s_turnState = STATE_FOLLOWING;
// Left wall state (for MODE_LEFT_WALL)
static unsigned long s_leftWallSeenTime = 0;
static bool s_leftWallWasPresent = false;
static TurnState s_leftTurnState = STATE_FOLLOWING;
static unsigned long s_leftCornerTurnStartTime = 0;
static int s_lastLeftSteerError = 0;

// Encoder counts (ISR-updated, read by sensor task)
static volatile int32_t s_encLeftTicks = 0;
static volatile int32_t s_encRightTicks = 0;

// Path recording/replay (PID task)
static PathSegment_t s_path[PATH_MAX];
static int s_pathLen = 0;
static int s_replayIndex = 0;
static unsigned long s_replayStartMs = 0;
static unsigned long s_lastRecordMs = 0;
// Mission playback: distance-based (encoder), speed from path, 90° turns at waypoints
static float s_missionTotalDistMm = 0.0f;
static int32_t s_missionStartEncL = 0, s_missionStartEncR = 0;
static TurnWaypoint_t s_turnWaypoints[MAX_TURN_WAYPOINTS];
static int s_turnWaypointCount = 0;
static int s_missionNextTurnIdx = 0;
static bool s_missionInTurn = false;
static bool s_missionTurnRight = true;  // current 90° turn direction
static unsigned long s_missionTurnStartMs = 0;
#define MISSION_TURN_DEG_THRESH  35.0f  // single-segment heading change (deg) to count as 90° turn
#define MISSION_TURN_CUMUL_DEG   50.0f  // cumulative heading over 3 segments to count as turn
// Dead reckoning state during MAP_RECORDING (encoder + heading -> x,y)
static int32_t s_lastEncL = 0, s_lastEncR = 0;
static float s_mapXmm = 0.0f, s_mapYmm = 0.0f;

// MPU6050 and heading integration
static Adafruit_MPU6050 s_mpu;
static float s_gyroZOffset = 0.0f;
static bool s_mpuOk = false;

// UI menu state (button + UI task)
static UIScreen s_uiScreen = SCREEN_MENU;
static uint8_t s_menuIndex = 0;
static unsigned long s_countdownStartMs = 0;
static CountdownReason s_countdownReason = COUNTDOWN_FREERUN;
static int s_countdownLastBeepSec = 99;  // last second we beeped (avoid repeat)
static bool s_missionPaused = false;
static uint8_t s_settingsIndex = 0;       // which setting (0..N-1)
static uint8_t s_settingsTotal = 0;       // set when entering settings
static SemaphoreHandle_t s_mutexUI = NULL;
#define PATH_SMOOTH_WINDOW 3   // moving average for path smoothing
#define MISSION_WALL_BLEND  0.25f  // 25% wall-follow assist during replay (0=replay only, 1=wall only)

// Smooth recorded path in-place (moving average on motor commands)
static void smoothPath(void) {
  if (s_pathLen < 3) return;
  for (int i = 1; i < s_pathLen - 1; i++) {
    int sumL = 0, sumR = 0;
    int n = 0;
    for (int j = i - 1; j <= i + 1 && j < s_pathLen; j++) {
      if (j >= 0) { sumL += s_path[j].spdL; sumR += s_path[j].spdR; n++; }
    }
    if (n > 0) {
      s_path[i].spdL = (int16_t)(sumL / n);
      s_path[i].spdR = (int16_t)(sumR / n);
    }
  }
}

// Build mission from path: cumulative distance per segment + 90° turn waypoints. Call after smoothPath() when saving.
static void buildMissionFromPath(void) {
  if (s_pathLen <= 0) { s_missionTotalDistMm = 0.0f; s_turnWaypointCount = 0; return; }
  s_path[0].cumDistMm = 0.0f;
  for (int i = 1; i < s_pathLen; i++) {
    float dx = s_path[i].posXmm - s_path[i - 1].posXmm;
    float dy = s_path[i].posYmm - s_path[i - 1].posYmm;
    s_path[i].cumDistMm = s_path[i - 1].cumDistMm + sqrtf(dx * dx + dy * dy);
  }
  s_missionTotalDistMm = s_path[s_pathLen - 1].cumDistMm;
  s_turnWaypointCount = 0;
  int lastTurnSeg = -10;
  for (int i = 1; i < s_pathLen && s_turnWaypointCount < MAX_TURN_WAYPOINTS; i++) {
    float dHeading = s_path[i].headingDeg - s_path[i - 1].headingDeg;
    if (dHeading > 180.0f) dHeading -= 360.0f;
    if (dHeading < -180.0f) dHeading += 360.0f;
    float cumul = dHeading;
    if (i >= 2) {
      float d2 = s_path[i - 1].headingDeg - s_path[i - 2].headingDeg;
      if (d2 > 180.0f) d2 -= 360.0f;
      if (d2 < -180.0f) d2 += 360.0f;
      cumul += d2;
    }
    if (i >= 3) {
      float d3 = s_path[i - 2].headingDeg - s_path[i - 3].headingDeg;
      if (d3 > 180.0f) d3 -= 360.0f;
      if (d3 < -180.0f) d3 += 360.0f;
      cumul += d3;
    }
    if (i - lastTurnSeg >= 2 && (fabsf(dHeading) > MISSION_TURN_DEG_THRESH || fabsf(cumul) > MISSION_TURN_CUMUL_DEG)) {
      s_turnWaypoints[s_turnWaypointCount].distMm = s_path[i].cumDistMm;
      s_turnWaypoints[s_turnWaypointCount].turnRight = (cumul > 0.0f) ? 1 : 0;
      s_turnWaypointCount++;
      lastTurnSeg = i;
    }
  }
}

// ========== Settings (EEPROM save/load + adjust) ==========
#define SETTINGS_COUNT 13  // 0..11 params, 12 = Calibrate
static void settingsAdjustValue(uint8_t idx, int delta) {
  if (idx >= 12) return;
  switch (idx) {
    case 0: KP_WALL = constrain(KP_WALL + delta * 0.01f, 0.01f, 2.0f); break;
    case 1: KD_WALL = constrain(KD_WALL + delta * 0.1f, 0.1f, 5.0f); break;
    case 2: KP_TURN = constrain(KP_TURN + delta * 0.01f, 0.01f, 5.0f); break;
    case 3: KD_TURN = constrain(KD_TURN + delta * 0.1f, 0.1f, 5.0f); break;
    case 4: BASE_SPEED = constrain(BASE_SPEED + delta * 5, 50, 255); break;
    case 5: RIGHT_SETPOINT = constrain(RIGHT_SETPOINT + delta * 50, 200, 3000); break;
    case 6: CROSS_RIGHT_SETPOINT = constrain(CROSS_RIGHT_SETPOINT + delta * 50, 200, 3000); break;
    case 7: CORNER_TURN_DURATION = constrain(CORNER_TURN_DURATION + delta * 25, 100, 500); break;
    case 8: CORNER_TURN_SPEED_L = constrain(CORNER_TURN_SPEED_L + delta * 10, 50, 255); break;
    case 9: CORNER_TURN_SPEED_R = constrain(CORNER_TURN_SPEED_R + delta * 10, 50, 255); break;
    case 10: LEFT_SETPOINT = constrain(LEFT_SETPOINT + delta * 50, 200, 3000); break;
    case 11: CROSS_LEFT_SETPOINT = constrain(CROSS_LEFT_SETPOINT + delta * 50, 200, 3000); break;
    default: break;
  }
}
static const char* settingsName(uint8_t idx) {
  switch (idx) {
    case 0: return "KP_WALL"; case 1: return "KD_WALL"; case 2: return "KP_TURN"; case 3: return "KD_TURN";
    case 4: return "BASE_SPD"; case 5: return "R_SET"; case 6: return "CR_SET"; case 7: return "CT_DUR";
    case 8: return "CT_L"; case 9: return "CT_R"; case 10: return "L_SET"; case 11: return "CL_SET";
    case 12: return "Calibrate"; default: return "?";
  }
}
static void settingsSaveToEEPROM(void) {
  EEPROM.write(0, EEPROM_MAGIC);
  int a = 1;
  EEPROM.put(a, KP_WALL); a += sizeof(float);
  EEPROM.put(a, KD_WALL); a += sizeof(float);
  EEPROM.put(a, KP_TURN); a += sizeof(float);
  EEPROM.put(a, KD_TURN); a += sizeof(float);
  EEPROM.put(a, BASE_SPEED); a += sizeof(int);
  EEPROM.put(a, RIGHT_SETPOINT); a += sizeof(int);
  EEPROM.put(a, CROSS_RIGHT_SETPOINT); a += sizeof(int);
  EEPROM.put(a, CORNER_TURN_DURATION); a += sizeof(int);
  EEPROM.put(a, CORNER_TURN_SPEED_L); a += sizeof(int);
  EEPROM.put(a, CORNER_TURN_SPEED_R); a += sizeof(int);
  EEPROM.put(a, LEFT_SETPOINT); a += sizeof(int);
  EEPROM.put(a, CROSS_LEFT_SETPOINT);
  EEPROM.commit();
}
static void settingsLoadFromEEPROM(void) {
  if (EEPROM.read(0) != EEPROM_MAGIC) return;
  int a = 1;
  EEPROM.get(a, KP_WALL); a += sizeof(float);
  EEPROM.get(a, KD_WALL); a += sizeof(float);
  EEPROM.get(a, KP_TURN); a += sizeof(float);
  EEPROM.get(a, KD_TURN); a += sizeof(float);
  EEPROM.get(a, BASE_SPEED); a += sizeof(int);
  EEPROM.get(a, RIGHT_SETPOINT); a += sizeof(int);
  EEPROM.get(a, CROSS_RIGHT_SETPOINT); a += sizeof(int);
  EEPROM.get(a, CORNER_TURN_DURATION); a += sizeof(int);
  EEPROM.get(a, CORNER_TURN_SPEED_L); a += sizeof(int);
  EEPROM.get(a, CORNER_TURN_SPEED_R); a += sizeof(int);
  EEPROM.get(a, LEFT_SETPOINT); a += sizeof(int);
  EEPROM.get(a, CROSS_LEFT_SETPOINT);
}

// ========== Buzzer request (thread-safe, for PID/Sensor tasks) ==========
static void buzzRequest(int freq, int durationMs) {
  BuzzCmd_t cmd = { .freq = freq, .durationMs = durationMs };
  if (s_queueBuzz != NULL)
    xQueueSend(s_queueBuzz, &cmd, 0);
}

// ========== Encoder ISRs (quadrature: A edge, read B for direction) ==========
void IRAM_ATTR isrLeftEnc() {
  if (digitalRead(LEFT_ENC_B)) s_encLeftTicks--;
  else s_encLeftTicks++;
}
void IRAM_ATTR isrRightEnc() {
  if (digitalRead(RIGHT_ENC_B)) s_encRightTicks++;
  else s_encRightTicks--;
}

// ========== Task: Sensor Reading (HIGH priority) ==========
static void taskSensor(void* arg) {
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
      s_sensor.encLeft = s_encLeftTicks;
      s_sensor.encRight = s_encRightTicks;
      {  // MPU: read every 10ms, integrate gyro Z to heading
        static uint8_t mpuCnt = 0;
        if (s_mpuOk && (++mpuCnt >= 10)) {
          mpuCnt = 0;
          sensors_event_t a, g, temp;
          s_mpu.getEvent(&a, &g, &temp);
          float dt = 0.01f;
          s_sensor.headingDeg += (g.gyro.z - s_gyroZOffset) * dt * 57.2958f;
        }
        if (!s_mpuOk) s_sensor.headingDeg = 0.0f;
      }

      s_sensor.spikeDetected = false;
      if (SPIKE_STOP_ENABLED) {
        if (s_sensor.prevL - s_sensor.L > SPIKE_DROP) s_sensor.spikeDetected = true;
        if (s_sensor.prevCL - s_sensor.CL > SPIKE_DROP) s_sensor.spikeDetected = true;
        if (s_sensor.prevF - s_sensor.F > SPIKE_DROP) s_sensor.spikeDetected = true;
        if (s_sensor.prevCR - s_sensor.CR > SPIKE_DROP) s_sensor.spikeDetected = true;
        if (s_sensor.prevR - s_sensor.R > SPIKE_DROP) s_sensor.spikeDetected = true;
      }

      if (s_sensor.spikeDetected) {
        xSemaphoreGive(s_mutexSensor);
        if (xSemaphoreTake(s_mutexControl, pdMS_TO_TICKS(10)) == pdTRUE) {
          s_control.targetLeft = -100;
          s_control.targetRight = -100;
          s_control.spikeDetected = true;
          strncpy(s_control.currentStateStr, "SPIKE!", sizeof(s_control.currentStateStr) - 1);
          s_control.currentStateStr[sizeof(s_control.currentStateStr) - 1] = '\0';
          xSemaphoreGive(s_mutexControl);
        }
      } else {
        xSemaphoreGive(s_mutexSensor);
      }
    }
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
  }
}

// ========== Task: PID Calculation (HIGH priority) ==========
static void taskPID(void* arg) {
  TickType_t lastWake = xTaskGetTickCount();
  SensorData_t localSensor;
  ControlData_t localControl;
  DriveMode driveMode = MODE_RIGHT_WALL;
  MapState mapState = MAP_IDLE;
  unsigned long nowMs = 0;

  for (;;) {
    nowMs = millis();
    if (xSemaphoreTake(s_mutexSensor, pdMS_TO_TICKS(10)) == pdTRUE) {
      memcpy(&localSensor, &s_sensor, sizeof(SensorData_t));
      xSemaphoreGive(s_mutexSensor);
    }
    if (xSemaphoreTake(s_mutexControl, pdMS_TO_TICKS(5)) == pdTRUE) {
      driveMode = s_control.driveMode;
      mapState = s_control.mapState;
      xSemaphoreGive(s_mutexControl);
    }
    UIScreen screen = SCREEN_MENU;
    if (s_mutexUI && xSemaphoreTake(s_mutexUI, pdMS_TO_TICKS(5)) == pdTRUE) {
      screen = s_uiScreen;
      xSemaphoreGive(s_mutexUI);
    }
    bool inMenuOrIdle = (screen == SCREEN_MENU || screen == SCREEN_FREERUN_MODE || screen == SCREEN_RECORD_MODE ||
                        screen == SCREEN_COUNTDOWN || screen == SCREEN_PLAY_INFO || screen == SCREEN_SETTINGS || screen == SCREEN_SETTINGS_ITEM);
    if (inMenuOrIdle) {
      if (xSemaphoreTake(s_mutexControl, pdMS_TO_TICKS(10)) == pdTRUE) {
        s_control.targetLeft = 0;
        s_control.targetRight = 0;
        xSemaphoreGive(s_mutexControl);
      }
      if (screen == SCREEN_COUNTDOWN && (nowMs - s_countdownStartMs) >= (unsigned long)COUNTDOWN_MS) {
        if (s_countdownReason == COUNTDOWN_FREERUN) {
          if (s_mutexUI && xSemaphoreTake(s_mutexUI, pdMS_TO_TICKS(20)) == pdTRUE) {
            s_uiScreen = SCREEN_FREERUN;
            xSemaphoreGive(s_mutexUI);
          }
          if (xSemaphoreTake(s_mutexControl, pdMS_TO_TICKS(10)) == pdTRUE) {
            s_control.mapState = MAP_IDLE;
            xSemaphoreGive(s_mutexControl);
          }
        } else if (s_countdownReason == COUNTDOWN_RECORD) {
          s_pathLen = 0;
          s_lastRecordMs = nowMs;
          if (xSemaphoreTake(s_mutexControl, pdMS_TO_TICKS(50)) == pdTRUE) {
            s_control.mapState = MAP_RECORDING;
            xSemaphoreGive(s_mutexControl);
          }
          if (s_mutexUI && xSemaphoreTake(s_mutexUI, pdMS_TO_TICKS(20)) == pdTRUE) {
            s_uiScreen = SCREEN_RECORDING;
            xSemaphoreGive(s_mutexUI);
          }
        } else if (s_countdownReason == COUNTDOWN_PLAY) {
          s_replayIndex = 0;
          s_replayStartMs = nowMs;
          s_missionPaused = false;
          s_missionInTurn = false;
          s_missionNextTurnIdx = 0;
          s_missionStartEncL = localSensor.encLeft;
          s_missionStartEncR = localSensor.encRight;
          if (xSemaphoreTake(s_mutexControl, pdMS_TO_TICKS(50)) == pdTRUE) {
            s_control.mapState = MISSION_RUNNING;
            xSemaphoreGive(s_mutexControl);
          }
          if (s_mutexUI && xSemaphoreTake(s_mutexUI, pdMS_TO_TICKS(20)) == pdTRUE) {
            s_uiScreen = SCREEN_MISSION;
            xSemaphoreGive(s_mutexUI);
          }
        }
      }
      vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PID_PERIOD_MS));
      continue;
    }

    // SCREEN_MISSION: distance-based playback (encoder); speed from path; wall-follow steering; 90° turns at waypoints
    if (screen == SCREEN_MISSION && mapState == MISSION_RUNNING) {
      if (s_missionPaused) {
        if (xSemaphoreTake(s_mutexControl, pdMS_TO_TICKS(10)) == pdTRUE) {
          s_control.targetLeft = 0;
          s_control.targetRight = 0;
          xSemaphoreGive(s_mutexControl);
        }
      } else {
        float distMm = 0.5f * ((float)(localSensor.encLeft - s_missionStartEncL) + (float)(localSensor.encRight - s_missionStartEncR)) * MM_PER_TICK;
        if (distMm < 0.0f) distMm = 0.0f;

        if (distMm >= s_missionTotalDistMm) {
          if (xSemaphoreTake(s_mutexControl, pdMS_TO_TICKS(10)) == pdTRUE) {
            s_control.mapState = MAP_READY;
            s_control.targetLeft = 0;
            s_control.targetRight = 0;
            xSemaphoreGive(s_mutexControl);
          }
          if (s_mutexUI && xSemaphoreTake(s_mutexUI, pdMS_TO_TICKS(20)) == pdTRUE) {
            s_uiScreen = SCREEN_MENU;
            s_menuIndex = 0;
            xSemaphoreGive(s_mutexUI);
          }
        } else if (s_missionInTurn) {
          unsigned long turnElapsed = nowMs - s_missionTurnStartMs;
          int turnDur = CORNER_TURN_DURATION;
          if (turnElapsed >= (unsigned long)turnDur) {
            s_missionInTurn = false;
            buzzRequest(1200, 100);
            buzzRequest(1200, 10);
            buzzRequest(1200, 1);
          } else if (xSemaphoreTake(s_mutexControl, pdMS_TO_TICKS(10)) == pdTRUE) {
            s_control.targetLeft = s_missionTurnRight ? CORNER_TURN_SPEED_L : LEFT_CORNER_TURN_SPEED_L;
            s_control.targetRight = s_missionTurnRight ? CORNER_TURN_SPEED_R : LEFT_CORNER_TURN_SPEED_R;
            s_control.missionDistMm = distMm;
            s_control.missionTotalDistMm = s_missionTotalDistMm;
            xSemaphoreGive(s_mutexControl);
          }
        } else {
          if (s_missionNextTurnIdx < s_turnWaypointCount && distMm >= s_turnWaypoints[s_missionNextTurnIdx].distMm) {
            s_missionInTurn = true;
            s_missionTurnStartMs = nowMs;
            s_missionTurnRight = (s_turnWaypoints[s_missionNextTurnIdx].turnRight != 0);
            s_missionNextTurnIdx++;
          }
          if (!s_missionInTurn && xSemaphoreTake(s_mutexControl, pdMS_TO_TICKS(10)) == pdTRUE) {
            int seg = 0;
            for (int i = 0; i < s_pathLen - 1; i++)
              if (distMm >= s_path[i].cumDistMm) seg = i;
            float baseSpeed = 0.5f * ((float)s_path[seg].spdL + (float)s_path[seg].spdR);
            if (baseSpeed < 80.0f) baseSpeed = 80.0f;
            if (baseSpeed < (float)BASE_SPEED) baseSpeed = (float)BASE_SPEED;
            computeControl(&localSensor, &localControl, MODE_RIGHT_WALL);
            int wallL = localControl.targetLeft;
            int wallR = localControl.targetRight;
            float mag = (fabsf((float)wallL) + fabsf((float)wallR)) * 0.5f;
            if (mag > baseSpeed && mag > 1.0f) {
              float scale = baseSpeed / mag;
              wallL = (int)((float)wallL * scale);
              wallR = (int)((float)wallR * scale);
            }
            s_control.targetLeft = wallL;
            s_control.targetRight = wallR;
            s_control.missionDistMm = distMm;
            s_control.missionTotalDistMm = s_missionTotalDistMm;
            xSemaphoreGive(s_mutexControl);
          }
        }
      }
      vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PID_PERIOD_MS));
      continue;
    }

    // SCREEN_FREERUN or SCREEN_RECORDING: run wall-follow / OA
    computeControl(&localSensor, &localControl, driveMode);

    if (mapState == MAP_RECORDING && s_pathLen < PATH_MAX && (nowMs - s_lastRecordMs) >= (unsigned long)RECORD_INTERVAL_MS) {
      s_lastRecordMs = nowMs;
      if (s_pathLen == 0) {
        s_lastEncL = localSensor.encLeft;
        s_lastEncR = localSensor.encRight;
        s_mapXmm = 0.0f;
        s_mapYmm = 0.0f;
      } else {
        int32_t dL = localSensor.encLeft - s_lastEncL;
        int32_t dR = localSensor.encRight - s_lastEncR;
        s_lastEncL = localSensor.encLeft;
        s_lastEncR = localSensor.encRight;
        float distMm = 0.5f * ((float)dL + (float)dR) * MM_PER_TICK;
        float hRad = localSensor.headingDeg * DEG_TO_RAD;
        s_mapXmm += distMm * cosf(hRad);
        s_mapYmm += distMm * sinf(hRad);
      }
      s_path[s_pathLen].spdL = (int16_t)localControl.targetLeft;
      s_path[s_pathLen].spdR = (int16_t)localControl.targetRight;
      s_path[s_pathLen].headingDeg = localSensor.headingDeg;
      s_path[s_pathLen].posXmm = s_mapXmm;
      s_path[s_pathLen].posYmm = s_mapYmm;
      s_pathLen++;
    }

    if (xSemaphoreTake(s_mutexControl, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (!localSensor.spikeDetected) {
        s_control.targetLeft = localControl.targetLeft;
        s_control.targetRight = localControl.targetRight;
        s_control.turnState = localControl.turnState;
        s_control.aggressiveMode = localControl.aggressiveMode;
        s_control.spikeDetected = false;
        s_control.headingDeg = localSensor.headingDeg;
        s_control.pathLen = s_pathLen;
        s_control.lastPathHeading = (s_pathLen > 0) ? s_path[s_pathLen - 1].headingDeg : 0.0f;
        s_control.mapXmm = s_mapXmm;
        s_control.mapYmm = s_mapYmm;
        strncpy(s_control.currentStateStr, localControl.currentStateStr, sizeof(s_control.currentStateStr) - 1);
        s_control.currentStateStr[sizeof(s_control.currentStateStr) - 1] = '\0';
        strncpy(s_control.currentModeStr, localControl.currentModeStr, sizeof(s_control.currentModeStr) - 1);
        s_control.currentModeStr[sizeof(s_control.currentModeStr) - 1] = '\0';
      }
      xSemaphoreGive(s_mutexControl);
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PID_PERIOD_MS));
  }
}

// ========== Task: Motor Control (MEDIUM priority) ==========
static void taskMotor(void* arg) {
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
static void taskUI(void* arg) {
  unsigned long buzzerOffTime = 0;
  BuzzCmd_t cmd;
  SensorData_t snapSensor;
  ControlData_t snapControl;
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    UIScreen screen = SCREEN_MENU;
    uint8_t menuIdx = 0;
    if (s_mutexUI && xSemaphoreTake(s_mutexUI, pdMS_TO_TICKS(5)) == pdTRUE) {
      screen = s_uiScreen;
      menuIdx = s_menuIndex;
      xSemaphoreGive(s_mutexUI);
    }
    if (screen == SCREEN_MENU || screen == SCREEN_FREERUN_MODE || screen == SCREEN_RECORD_MODE ||
        screen == SCREEN_PLAY_INFO || screen == SCREEN_SETTINGS || screen == SCREEN_SETTINGS_ITEM) {
      noTone(BUZZER_PIN);
      while (s_queueBuzz != NULL && xQueueReceive(s_queueBuzz, &cmd, 0) == pdTRUE) { }
      buzzerOffTime = 0;
    } else {
      while (s_queueBuzz != NULL && xQueueReceive(s_queueBuzz, &cmd, 0) == pdTRUE) {
        tone(BUZZER_PIN, cmd.freq);
        buzzerOffTime = millis() + (unsigned long)cmd.durationMs;
      }
      if (buzzerOffTime > 0 && millis() >= buzzerOffTime) {
        noTone(BUZZER_PIN);
        buzzerOffTime = 0;
      }
    }

    if (xSemaphoreTake(s_mutexSensor, pdMS_TO_TICKS(5)) == pdTRUE) {
      memcpy(&snapSensor, &s_sensor, sizeof(SensorData_t));
      xSemaphoreGive(s_mutexSensor);
      if (xSemaphoreTake(s_mutexControl, pdMS_TO_TICKS(5)) == pdTRUE) {
        memcpy(&snapControl, &s_control, sizeof(ControlData_t));
        xSemaphoreGive(s_mutexControl);
      }
    }
    if (screen == SCREEN_COUNTDOWN) {
      unsigned long elapsed = millis() - s_countdownStartMs;
      int secLeft = ((int)COUNTDOWN_MS - (int)elapsed) / 1000;
      if (secLeft < 0) secLeft = 0;
      if (secLeft <= 3 && secLeft != s_countdownLastBeepSec) {
        s_countdownLastBeepSec = secLeft;
        if (secLeft > 0)
          buzzRequest(900, 100);
        else
          buzzRequest(1400, 200);
      }
    }
    updateDisplayMenu(screen, menuIdx, &snapSensor, &snapControl);
    updateVisualsSnapshot(screen, &snapControl);

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(UI_PERIOD_MS));
  }
}

// ========== Task: Buttons D5 / D2 (LOW priority) ==========
static void taskButtons(void* arg) {
  TickType_t lastWake = xTaskGetTickCount();
  bool lastD5 = true, lastD2 = true;
  uint32_t d2PressTime = 0, d5PressTime = 0;

  for (;;) {
    bool d5 = (digitalRead(BTN_MODE) == LOW);
    bool d2 = (digitalRead(BTN_MAP) == LOW);
    UIScreen screen = SCREEN_MENU;
    if (s_mutexUI && xSemaphoreTake(s_mutexUI, pdMS_TO_TICKS(5)) == pdTRUE) {
      screen = s_uiScreen;
      xSemaphoreGive(s_mutexUI);
    }

    // ----- Main menu: FreeRun, Record, Play, Settings -----
    if (screen == SCREEN_MENU) {
      if (d5 && !lastD5) d5PressTime = millis();
      if (!d5 && lastD5 && (millis() - d5PressTime) >= BTN_DEBOUNCE_MS) {
        if (s_mutexUI && xSemaphoreTake(s_mutexUI, pdMS_TO_TICKS(20)) == pdTRUE) {
          s_menuIndex = (s_menuIndex + 1) % 4;
          xSemaphoreGive(s_mutexUI);
        }
      }
      lastD5 = d5;
      if (d2 && !lastD2) d2PressTime = millis();
      if (!d2 && lastD2 && (millis() - d2PressTime) < BTN_HOLD_MS && (millis() - d2PressTime) >= BTN_DEBOUNCE_MS) {
        if (s_mutexUI) xSemaphoreTake(s_mutexUI, portMAX_DELAY);
        if (s_menuIndex == 0) {
          s_uiScreen = SCREEN_FREERUN_MODE;
          s_menuIndex = 0;
        } else if (s_menuIndex == 1) {
          s_uiScreen = SCREEN_RECORD_MODE;
          s_menuIndex = 0;
        } else if (s_menuIndex == 2) {
          if (s_pathLen > 0)
            s_uiScreen = SCREEN_PLAY_INFO;
        } else {
          s_uiScreen = SCREEN_SETTINGS;
          s_settingsIndex = 0;
          s_settingsTotal = SETTINGS_COUNT;
        }
        if (s_mutexUI) xSemaphoreGive(s_mutexUI);
      }
      lastD2 = d2;
      vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(BUTTON_PERIOD_MS));
      continue;
    }

    // ----- FreeRun submenu: RWF, LWF, OA; D2 = 3s countdown then run -----
    if (screen == SCREEN_FREERUN_MODE) {
      if (d5 && !lastD5) d5PressTime = millis();
      if (!d5 && lastD5 && (millis() - d5PressTime) >= BTN_DEBOUNCE_MS) {
        if (xSemaphoreTake(s_mutexControl, pdMS_TO_TICKS(30)) == pdTRUE) {
          s_control.driveMode = (DriveMode)((s_control.driveMode + 1) % MODE_COUNT);
          xSemaphoreGive(s_mutexControl);
        }
      }
      lastD5 = d5;
      if (d2 && !lastD2) d2PressTime = millis();
      if (!d2 && lastD2 && (millis() - d2PressTime) >= BTN_DEBOUNCE_MS) {
        if (s_mutexUI) xSemaphoreTake(s_mutexUI, portMAX_DELAY);
        s_countdownReason = COUNTDOWN_FREERUN;
        s_countdownStartMs = millis();
        s_countdownLastBeepSec = 99;
        s_uiScreen = SCREEN_COUNTDOWN;
        if (s_mutexUI) xSemaphoreGive(s_mutexUI);
      }
      lastD2 = d2;
      vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(BUTTON_PERIOD_MS));
      continue;
    }

    // ----- Record submenu: RWF, LWF, OA; D2 = 3s countdown then record -----
    if (screen == SCREEN_RECORD_MODE) {
      if (d5 && !lastD5) d5PressTime = millis();
      if (!d5 && lastD5 && (millis() - d5PressTime) >= BTN_DEBOUNCE_MS) {
        if (xSemaphoreTake(s_mutexControl, pdMS_TO_TICKS(30)) == pdTRUE) {
          s_control.driveMode = (DriveMode)((s_control.driveMode + 1) % MODE_COUNT);
          xSemaphoreGive(s_mutexControl);
        }
      }
      lastD5 = d5;
      if (d2 && !lastD2) d2PressTime = millis();
      if (!d2 && lastD2 && (millis() - d2PressTime) >= BTN_DEBOUNCE_MS) {
        if (s_mutexUI) xSemaphoreTake(s_mutexUI, portMAX_DELAY);
        s_countdownReason = COUNTDOWN_RECORD;
        s_countdownStartMs = millis();
        s_countdownLastBeepSec = 99;
        s_uiScreen = SCREEN_COUNTDOWN;
        if (s_mutexUI) xSemaphoreGive(s_mutexUI);
      }
      lastD2 = d2;
      vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(BUTTON_PERIOD_MS));
      continue;
    }

    // ----- FreeRun active: BTN_MODE = stop & return to menu -----
    if (screen == SCREEN_FREERUN) {
      if (d5 && !lastD5) d5PressTime = millis();
      if (!d5 && lastD5 && (millis() - d5PressTime) >= BTN_DEBOUNCE_MS) {
        if (xSemaphoreTake(s_mutexControl, pdMS_TO_TICKS(50)) == pdTRUE) {
          s_control.targetLeft = 0;
          s_control.targetRight = 0;
          xSemaphoreGive(s_mutexControl);
        }
        if (s_mutexUI && xSemaphoreTake(s_mutexUI, pdMS_TO_TICKS(30)) == pdTRUE) {
          s_uiScreen = SCREEN_MENU;
          s_menuIndex = 0;
          xSemaphoreGive(s_mutexUI);
        }
      }
      lastD5 = d5;
      lastD2 = d2;
      vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(BUTTON_PERIOD_MS));
      continue;
    }

    // ----- Recording: BTN_MAP = save & menu, BTN_MODE = discard & menu -----
    if (screen == SCREEN_RECORDING) {
      if (d5 && !lastD5) d5PressTime = millis();
      if (!d5 && lastD5 && (millis() - d5PressTime) >= BTN_DEBOUNCE_MS) {
        if (xSemaphoreTake(s_mutexControl, pdMS_TO_TICKS(50)) == pdTRUE) {
          s_control.mapState = MAP_IDLE;
          s_control.targetLeft = 0;
          s_control.targetRight = 0;
          xSemaphoreGive(s_mutexControl);
        }
        if (s_mutexUI && xSemaphoreTake(s_mutexUI, pdMS_TO_TICKS(30)) == pdTRUE) {
          s_uiScreen = SCREEN_MENU;
          s_menuIndex = 0;
          xSemaphoreGive(s_mutexUI);
        }
      }
      lastD5 = d5;
      if (d2 && !lastD2) d2PressTime = millis();
      if (!d2 && lastD2 && (millis() - d2PressTime) >= BTN_DEBOUNCE_MS) {
        smoothPath();
        buildMissionFromPath();
        if (xSemaphoreTake(s_mutexControl, pdMS_TO_TICKS(50)) == pdTRUE) {
          s_control.mapState = MAP_READY;
          s_control.targetLeft = 0;
          s_control.targetRight = 0;
          s_control.pathLen = s_pathLen;
          if (s_pathLen > 0) {
            s_control.mapXmm = s_path[s_pathLen - 1].posXmm;
            s_control.mapYmm = s_path[s_pathLen - 1].posYmm;
            s_control.lastPathHeading = s_path[s_pathLen - 1].headingDeg;
          }
          xSemaphoreGive(s_mutexControl);
        }
        if (s_mutexUI && xSemaphoreTake(s_mutexUI, pdMS_TO_TICKS(30)) == pdTRUE) {
          s_uiScreen = SCREEN_MENU;
          s_menuIndex = 0;
          xSemaphoreGive(s_mutexUI);
        }
      }
      lastD2 = d2;
      vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(BUTTON_PERIOD_MS));
      continue;
    }

    // ----- Play info submenu: waypoints, segments, dist, time, turns; D2 = Start 3s, D5 = Back -----
    if (screen == SCREEN_PLAY_INFO) {
      if (d5 && !lastD5) d5PressTime = millis();
      if (!d5 && lastD5 && (millis() - d5PressTime) >= BTN_DEBOUNCE_MS) {
        if (s_mutexUI && xSemaphoreTake(s_mutexUI, pdMS_TO_TICKS(20)) == pdTRUE) {
          s_uiScreen = SCREEN_MENU;
          s_menuIndex = 2;
          xSemaphoreGive(s_mutexUI);
        }
      }
      lastD5 = d5;
      if (d2 && !lastD2) d2PressTime = millis();
      if (!d2 && lastD2 && (millis() - d2PressTime) >= BTN_DEBOUNCE_MS) {
        if (s_mutexUI) xSemaphoreTake(s_mutexUI, portMAX_DELAY);
        s_countdownReason = COUNTDOWN_PLAY;
        s_countdownStartMs = millis();
        s_countdownLastBeepSec = 99;
        s_uiScreen = SCREEN_COUNTDOWN;
        if (s_mutexUI) xSemaphoreGive(s_mutexUI);
      }
      lastD2 = d2;
      vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(BUTTON_PERIOD_MS));
      continue;
    }

    // ----- Mission (Play): BTN_MODE hold = stop & menu, BTN_MAP hold = delete & menu, BTN_MAP click = pause/resume -----
    if (screen == SCREEN_MISSION) {
      if (d5 && !lastD5) d5PressTime = millis();
      if (!d5 && lastD5) {
        uint32_t held = millis() - d5PressTime;
        if (held >= BTN_HOLD_MS) {
          if (xSemaphoreTake(s_mutexControl, pdMS_TO_TICKS(50)) == pdTRUE) {
            s_control.mapState = MAP_READY;
            s_control.targetLeft = 0;
            s_control.targetRight = 0;
            xSemaphoreGive(s_mutexControl);
          }
          if (s_mutexUI && xSemaphoreTake(s_mutexUI, pdMS_TO_TICKS(30)) == pdTRUE) {
            s_uiScreen = SCREEN_MENU;
            s_menuIndex = 0;
            xSemaphoreGive(s_mutexUI);
          }
        }
      }
      lastD5 = d5;
      if (d2 && !lastD2) d2PressTime = millis();
      if (!d2 && lastD2) {
        uint32_t held = millis() - d2PressTime;
        if (held >= BTN_HOLD_MS) {
          s_pathLen = 0;
          if (xSemaphoreTake(s_mutexControl, pdMS_TO_TICKS(50)) == pdTRUE) {
            s_control.mapState = MAP_IDLE;
            s_control.targetLeft = 0;
            s_control.targetRight = 0;
            xSemaphoreGive(s_mutexControl);
          }
          if (s_mutexUI && xSemaphoreTake(s_mutexUI, pdMS_TO_TICKS(30)) == pdTRUE) {
            s_uiScreen = SCREEN_MENU;
            s_menuIndex = 0;
            xSemaphoreGive(s_mutexUI);
          }
        } else if (held >= BTN_DEBOUNCE_MS) {
          s_missionPaused = !s_missionPaused;
        }
      }
      lastD2 = d2;
      vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(BUTTON_PERIOD_MS));
      continue;
    }

    // ----- Settings: D5 = + / next, D2 = - / select; D5 long = Discard, D2 long = Save -----
    if (screen == SCREEN_SETTINGS || screen == SCREEN_SETTINGS_ITEM) {
      if (d5 && !lastD5) d5PressTime = millis();
      if (!d5 && lastD5) {
        uint32_t held5 = millis() - d5PressTime;
        if (held5 >= BTN_HOLD_MS) {
          if (s_mutexUI && xSemaphoreTake(s_mutexUI, pdMS_TO_TICKS(30)) == pdTRUE) {
            if (screen == SCREEN_SETTINGS_ITEM)
              s_uiScreen = SCREEN_SETTINGS;
            else {
              s_uiScreen = SCREEN_MENU;
              s_menuIndex = 3;
            }
            xSemaphoreGive(s_mutexUI);
          }
        } else if (held5 >= BTN_DEBOUNCE_MS) {
          if (screen == SCREEN_SETTINGS) {
            if (s_mutexUI && xSemaphoreTake(s_mutexUI, pdMS_TO_TICKS(20)) == pdTRUE) {
              s_settingsIndex = (s_settingsIndex + 1) % s_settingsTotal;
              xSemaphoreGive(s_mutexUI);
            }
          } else {
            settingsAdjustValue(s_settingsIndex, 1);
          }
        }
      }
      lastD5 = d5;
      if (d2 && !lastD2) d2PressTime = millis();
      if (!d2 && lastD2) {
        uint32_t held2 = millis() - d2PressTime;
        if (held2 >= BTN_HOLD_MS) {
          if (s_mutexUI) xSemaphoreTake(s_mutexUI, portMAX_DELAY);
          if (screen == SCREEN_SETTINGS) {
            settingsSaveToEEPROM();
            s_uiScreen = SCREEN_MENU;
            s_menuIndex = 3;
          } else {
            if (s_settingsIndex < 12) settingsSaveToEEPROM();
            s_uiScreen = SCREEN_SETTINGS;
          }
          if (s_mutexUI) xSemaphoreGive(s_mutexUI);
        } else if (held2 >= BTN_DEBOUNCE_MS) {
          if (s_mutexUI) xSemaphoreTake(s_mutexUI, portMAX_DELAY);
          if (screen == SCREEN_SETTINGS) {
            if (s_settingsIndex == 12) {
              if (s_mpuOk) {
                float sum = 0;
                for (int i = 0; i < 30; i++) {
                  sensors_event_t a, g, temp;
                  s_mpu.getEvent(&a, &g, &temp);
                  sum += g.gyro.z;
                  vTaskDelay(pdMS_TO_TICKS(30));
                }
                s_gyroZOffset = sum / 30.0f;
              }
              s_encLeftTicks = 0;
              s_encRightTicks = 0;
              if (xSemaphoreTake(s_mutexControl, pdMS_TO_TICKS(50)) == pdTRUE) {
                s_control.headingDeg = 0.0f;
                xSemaphoreGive(s_mutexControl);
              }
              if (xSemaphoreTake(s_mutexSensor, pdMS_TO_TICKS(20)) == pdTRUE) {
                s_sensor.headingDeg = 0.0f;
                xSemaphoreGive(s_mutexSensor);
              }
              s_uiScreen = SCREEN_MENU;
              s_menuIndex = 3;
            } else
              s_uiScreen = SCREEN_SETTINGS_ITEM;
          } else {
            settingsAdjustValue(s_settingsIndex, -1);
          }
          if (s_mutexUI) xSemaphoreGive(s_mutexUI);
        }
      }
      lastD2 = d2;
      vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(BUTTON_PERIOD_MS));
      continue;
    }

    lastD5 = d5;
    lastD2 = d2;
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(BUTTON_PERIOD_MS));
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

  pinMode(BTN_MODE, INPUT_PULLUP);
  pinMode(BTN_MAP, INPUT_PULLUP);
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), isrLeftEnc, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), isrRightEnc, RISING);

  s_mpuOk = s_mpu.begin();
  if (s_mpuOk) {
    s_mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    s_mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    delay(100);
    float sum = 0; for (int i = 0; i < 50; i++) {
      sensors_event_t a, g, temp; s_mpu.getEvent(&a, &g, &temp);
      sum += g.gyro.z; delay(20);
    }
    s_gyroZOffset = sum / 50.0f;
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

  EEPROM.begin(EEPROM_SIZE);
  settingsLoadFromEEPROM();

  // Harry Potter (Hedwig's Theme) startup melody - first phrase E5 B5 E5 G5 A5 G5 E5
  const int hpNotes[] = { 659, 988, 659, 784, 880, 784, 659 };
  const int hpDurs[] = { 200, 200, 200, 200, 400, 200, 400 };
  for (int i = 0; i < 7; i++) {
    tone(BUZZER_PIN, hpNotes[i], hpDurs[i]);
    for (int p = 0; p < NEOPIXEL_COUNT; p++) {
      pixels.setPixelColor(p, pixels.Color(0, 0, 0));
      pixels.setPixelColor((p + 1) % NEOPIXEL_COUNT, pixels.Color(80, 40, 180));
      pixels.show();
      delay(hpDurs[i] / NEOPIXEL_COUNT);
    }
    delay(hpDurs[i] / 2);
  }
  noTone(BUZZER_PIN);
  setAllPixels(0, 80, 180);
  pixels.show();
  delay(300);

  // Init shared state
  memset(s_control.currentStateStr, 0, sizeof(s_control.currentStateStr));
  memset(s_control.currentModeStr, 0, sizeof(s_control.currentModeStr));
  strncpy(s_control.currentStateStr, "INIT", sizeof(s_control.currentStateStr) - 1);
  strncpy(s_control.currentModeStr, "WALL", sizeof(s_control.currentModeStr) - 1);
  s_control.turnState = STATE_FOLLOWING;
  s_control.emergencyStop = false;
  s_control.spikeDetected = false;
  s_control.driveMode = MODE_RIGHT_WALL;
  s_control.mapState = MAP_IDLE;
  s_control.headingDeg = 0.0f;
  s_control.pathLen = 0;

  s_mutexSensor = xSemaphoreCreateMutex();
  s_mutexControl = xSemaphoreCreateMutex();
  s_mutexUI = xSemaphoreCreateMutex();
  s_queueBuzz = xQueueCreate(BUZZ_QUEUE_LEN, sizeof(BuzzCmd_t));
  if (s_mutexSensor == NULL || s_mutexControl == NULL || s_mutexUI == NULL || s_queueBuzz == NULL) {
    Serial.println("FATAL: RTOS objects creation failed");
    for (;;) delay(1000);
  }

  xTaskCreatePinnedToCore(taskSensor, "sensor", TASK_STACK_SENSOR, NULL, TASK_PRIORITY_HIGH, NULL, 0);
  xTaskCreatePinnedToCore(taskPID, "pid", TASK_STACK_PID, NULL, TASK_PRIORITY_HIGH, NULL, 0);
  xTaskCreatePinnedToCore(taskMotor, "motor", TASK_STACK_MOTOR, NULL, TASK_PRIORITY_MEDIUM, NULL, 0);
  xTaskCreatePinnedToCore(taskUI, "ui", TASK_STACK_UI, NULL, TASK_PRIORITY_LOW, NULL, 1);
  xTaskCreatePinnedToCore(taskButtons, "btn", 2048, NULL, TASK_PRIORITY_LOW, NULL, 1);

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

// ========== PID Control Dispatcher ==========
static void computeControl(const SensorData_t* sens, ControlData_t* out, DriveMode mode) {
  if (sens->spikeDetected) {
    out->targetLeft = -100;
    out->targetRight = -100;
    strncpy(out->currentStateStr, "SPIKE!", sizeof(out->currentStateStr) - 1);
    out->currentStateStr[sizeof(out->currentStateStr) - 1] = '\0';
    return;
  }
  switch (mode) {
    case MODE_RIGHT_WALL:  computeRightWall(sens, out); break;
    case MODE_LEFT_WALL:  computeLeftWall(sens, out);  break;
    case MODE_OBSTACLE_AVOID: computeObstacleAvoid(sens, out); break;
    default: computeRightWall(sens, out); break;
  }
}

// ========== Right Wall Following ==========
static void computeRightWall(const SensorData_t* sens, ControlData_t* out) {
  int L = sens->L, CL = sens->CL, F = sens->F, CR = sens->CR, R = sens->R;
  int prevR = sens->prevR;
  unsigned long now = millis();
  bool rightWallPresent = (R > WALL_PRESENT_MIN) && (CR > WALL_PRESENT_MIN);
  if (rightWallPresent) { s_wallWasPresent = true; s_lastWallSeenTime = now; }

  switch (s_turnState) {
    case STATE_FOLLOWING:
      if (s_wallWasPresent && !rightWallPresent && (now - s_lastWallSeenTime) < (unsigned long)CORNER_DETECT_TIME) {
        s_turnState = STATE_CORNER_TURN;
        s_cornerTurnStartTime = now;
        strncpy(out->currentStateStr, "CORNER", sizeof(out->currentStateStr) - 1);
        out->currentStateStr[sizeof(out->currentStateStr) - 1] = '\0';
        buzzRequest(1800, 80);
      }
      break;
    case STATE_CORNER_TURN: {
      unsigned long elapsed = now - s_cornerTurnStartTime;
      out->turnState = s_turnState;
      strncpy(out->currentStateStr, "CORNER", sizeof(out->currentStateStr) - 1);
      strncpy(out->currentModeStr, "WALL", sizeof(out->currentModeStr) - 1);
      out->aggressiveMode = false;
      if (elapsed < (unsigned long)CT_WAIT) {
        out->targetLeft = CORNER_FORWARD_SPEED;
        out->targetRight = CORNER_FORWARD_SPEED;
      } else if (elapsed < (unsigned long)(CT_WAIT + CORNER_TURN_DURATION)) {
        out->targetLeft = CORNER_TURN_SPEED_L;
        out->targetRight = CORNER_TURN_SPEED_R;
      } else {
        s_turnState = STATE_FOLLOWING;
        s_wallWasPresent = false;
      }
      return;
    }
  }
  out->turnState = s_turnState;

  bool aggressive = (F > 1200) || (L > 1500 || CL > 2200) || (R < WALL_PRESENT_MIN && prevR > WALL_PRESENT_MIN + 200);
  if (R > WALL_PRESENT_MIN && CR > WALL_PRESENT_MIN && (CR - R) > 800) aggressive = true;
  float kp = aggressive ? KP_TURN : KP_WALL;
  float kd = aggressive ? KD_TURN : KD_WALL;
  out->aggressiveMode = aggressive;
  strncpy(out->currentModeStr, aggressive ? "TURN" : "WALL", sizeof(out->currentModeStr) - 1);
  out->currentModeStr[sizeof(out->currentModeStr) - 1] = '\0';

  int frontStart = 600;
  int frontVal = constrain(F, frontStart, THRESHOLD_LRF);
  float frontFactor = (float)map(frontVal, frontStart, THRESHOLD_LRF, 0, 1000) / 1000.0f;
  int desiredSpeed = (int)(BASE_SPEED * (1.0f - frontFactor));

  int leftPush = 0;
  if (CL > 600) leftPush += map(constrain(CL, 600, THRESHOLD_CROSS), 600, THRESHOLD_CROSS, 0, 80);
  if (L > 600) leftPush += map(constrain(L, 600, THRESHOLD_LRF), 600, THRESHOLD_LRF, 0, 120);
  int rightError = (R > WALL_PRESENT_MIN) ? (R - RIGHT_SETPOINT) + (CR - CROSS_RIGHT_SETPOINT) : -200;
  strncpy(out->currentStateStr, rightWallPresent ? "FOLLOW" : (R > WALL_PRESENT_MIN ? "FOLLOW" : "SEARCH"), sizeof(out->currentStateStr) - 1);
  out->currentStateStr[sizeof(out->currentStateStr) - 1] = '\0';

  int rawSteer = leftPush - rightError;
  int steerDeriv = rawSteer - s_lastSteerError;
  s_lastSteerError = rawSteer;
  int totalSteer = (int)((rawSteer * kp) + (steerDeriv * kd));
  if (frontFactor > 0.8f) { strncpy(out->currentStateStr, "BLOCKED", sizeof(out->currentStateStr) - 1); totalSteer = (L < R) ? -150 : 150; }
  out->targetLeft = constrain(desiredSpeed + totalSteer, -MAX_SPEED, MAX_SPEED);
  out->targetRight = constrain(desiredSpeed - totalSteer, -MAX_SPEED, MAX_SPEED);
}

// ========== Left Wall Following (mirror of right) ==========
static void computeLeftWall(const SensorData_t* sens, ControlData_t* out) {
  int L = sens->L, CL = sens->CL, F = sens->F, CR = sens->CR, R = sens->R;
  int prevL = sens->prevL;
  unsigned long now = millis();
  bool leftWallPresent = (L > WALL_PRESENT_MIN) && (CL > WALL_PRESENT_MIN);
  if (leftWallPresent) { s_leftWallWasPresent = true; s_leftWallSeenTime = now; }

  switch (s_leftTurnState) {
    case STATE_FOLLOWING:
      if (s_leftWallWasPresent && !leftWallPresent && (now - s_leftWallSeenTime) < (unsigned long)CORNER_DETECT_TIME) {
        s_leftTurnState = STATE_CORNER_TURN;
        s_leftCornerTurnStartTime = now;
        strncpy(out->currentStateStr, "CORNER", sizeof(out->currentStateStr) - 1);
        out->currentStateStr[sizeof(out->currentStateStr) - 1] = '\0';
        buzzRequest(1800, 80);
      }
      break;
    case STATE_CORNER_TURN: {
      unsigned long elapsed = now - s_leftCornerTurnStartTime;
      out->turnState = s_leftTurnState;
      strncpy(out->currentStateStr, "CORNER", sizeof(out->currentStateStr) - 1);
      strncpy(out->currentModeStr, "WALL", sizeof(out->currentModeStr) - 1);
      out->aggressiveMode = false;
      if (elapsed < (unsigned long)CT_WAIT) {
        out->targetLeft = CORNER_FORWARD_SPEED;
        out->targetRight = CORNER_FORWARD_SPEED;
      } else if (elapsed < (unsigned long)(CT_WAIT + LEFT_CORNER_TURN_DURATION)) {
        out->targetLeft = LEFT_CORNER_TURN_SPEED_L;
        out->targetRight = LEFT_CORNER_TURN_SPEED_R;
      } else {
        s_leftTurnState = STATE_FOLLOWING;
        s_leftWallWasPresent = false;
      }
      return;
    }
  }
  out->turnState = s_leftTurnState;

  bool aggressive = (F > 1200) || (R > 1500 || CR > 2200) || (L < WALL_PRESENT_MIN && prevL > WALL_PRESENT_MIN + 200);
  if (L > WALL_PRESENT_MIN && CL > WALL_PRESENT_MIN && (CL - L) > 800) aggressive = true;
  float kp = aggressive ? KP_TURN : KP_WALL;
  float kd = aggressive ? KD_TURN : KD_WALL;
  out->aggressiveMode = aggressive;
  strncpy(out->currentModeStr, aggressive ? "TURN" : "WALL", sizeof(out->currentModeStr) - 1);
  out->currentModeStr[sizeof(out->currentModeStr) - 1] = '\0';

  int frontStart = 600;
  int frontVal = constrain(F, frontStart, THRESHOLD_LRF);
  float frontFactor = (float)map(frontVal, frontStart, THRESHOLD_LRF, 0, 1000) / 1000.0f;
  int desiredSpeed = (int)(BASE_SPEED * (1.0f - frontFactor));

  int rightPush = 0;
  if (CR > 600) rightPush += map(constrain(CR, 600, THRESHOLD_CROSS), 600, THRESHOLD_CROSS, 0, 80);
  if (R > 600) rightPush += map(constrain(R, 600, THRESHOLD_LRF), 600, THRESHOLD_LRF, 0, 120);
  int leftError = (L > WALL_PRESENT_MIN) ? (L - LEFT_SETPOINT) + (CL - CROSS_LEFT_SETPOINT) : -200;
  strncpy(out->currentStateStr, leftWallPresent ? "FOLLOW" : (L > WALL_PRESENT_MIN ? "FOLLOW" : "SEARCH"), sizeof(out->currentStateStr) - 1);
  out->currentStateStr[sizeof(out->currentStateStr) - 1] = '\0';

  int rawSteer = rightPush - leftError;
  int steerDeriv = rawSteer - s_lastLeftSteerError;
  s_lastLeftSteerError = rawSteer;
  int totalSteer = (int)((rawSteer * kp) + (steerDeriv * kd));
  if (frontFactor > 0.8f) { strncpy(out->currentStateStr, "BLOCKED", sizeof(out->currentStateStr) - 1); totalSteer = (R < L) ? 150 : -150; }
  out->targetLeft = constrain(desiredSpeed - totalSteer, -MAX_SPEED, MAX_SPEED);
  out->targetRight = constrain(desiredSpeed + totalSteer, -MAX_SPEED, MAX_SPEED);
}

// ========== Obstacle Avoidance (no wall follow) ==========
static void computeObstacleAvoid(const SensorData_t* sens, ControlData_t* out) {
  int L = sens->L, CL = sens->CL, F = sens->F, CR = sens->CR, R = sens->R;
  out->turnState = STATE_FOLLOWING;
  strncpy(out->currentModeStr, "OA", sizeof(out->currentModeStr) - 1);
  out->currentModeStr[sizeof(out->currentModeStr) - 1] = '\0';

  int frontStart = 800;
  int frontVal = constrain(F, frontStart, THRESHOLD_LRF);
  float frontFactor = (float)map(frontVal, frontStart, THRESHOLD_LRF, 0, 1000) / 1000.0f;
  int desiredSpeed = (int)(BASE_SPEED * (1.0f - frontFactor));

  int crossStart = 800;
  int leftUrgency = 0;
  if (CL > crossStart) leftUrgency += map(constrain(CL, crossStart, THRESHOLD_CROSS), crossStart, THRESHOLD_CROSS, 0, 100);
  if (L > frontStart) leftUrgency += map(constrain(L, frontStart, THRESHOLD_LRF), frontStart, THRESHOLD_LRF, 0, 150);
  int rightUrgency = 0;
  if (CR > crossStart) rightUrgency += map(constrain(CR, crossStart, THRESHOLD_CROSS), crossStart, THRESHOLD_CROSS, 0, 100);
  if (R > frontStart) rightUrgency += map(constrain(R, frontStart, THRESHOLD_LRF), frontStart, THRESHOLD_LRF, 0, 150);

  int steer = (int)((leftUrgency - rightUrgency) * KP_TURN);
  if (frontFactor > 0.8f) steer = (leftUrgency > rightUrgency) ? 100 : -100;

  out->aggressiveMode = (frontFactor > 0.5f || leftUrgency > 80 || rightUrgency > 80);
  strncpy(out->currentStateStr, frontFactor > 0.8f ? "BLOCKED" : "AVOID", sizeof(out->currentStateStr) - 1);
  out->currentStateStr[sizeof(out->currentStateStr) - 1] = '\0';
  out->targetLeft = constrain(desiredSpeed + steer, -MAX_SPEED, MAX_SPEED);
  out->targetRight = constrain(desiredSpeed - steer, -MAX_SPEED, MAX_SPEED);
}

// ========== Visual Feedback (UI task) ==========
static void updateVisualsSnapshot(UIScreen screen, const ControlData_t* c) {
  if (screen == SCREEN_MENU)
    setAllPixels(40, 80, 180);
  else if (screen == SCREEN_FREERUN_MODE || screen == SCREEN_RECORD_MODE || screen == SCREEN_PLAY_INFO)
    setAllPixels(60, 100, 200);
  else if (screen == SCREEN_COUNTDOWN)
    setAllPixels(255, 200, 0);
  else if (screen == SCREEN_SETTINGS || screen == SCREEN_SETTINGS_ITEM)
    setAllPixels(200, 120, 0);
  else if (screen == SCREEN_FREERUN) {
    if (c->emergencyStop || c->spikeDetected) setAllPixels(255, 0, 0);
    else if (c->turnState == STATE_CORNER_TURN) setAllPixels(255, 0, 255);
    else if (c->driveMode == MODE_OBSTACLE_AVOID) setAllPixels(255, 200, 0);
    else if (c->driveMode == MODE_LEFT_WALL) setAllPixels(0, 255, 200);
    else setAllPixels(0, 255, 80);
  }
  else if (screen == SCREEN_RECORDING)
    setAllPixels(0, 200, 255);
  else if (screen == SCREEN_MISSION) {
    if (s_missionPaused) setAllPixels(150, 150, 0);
    else setAllPixels(160, 0, 255);
  }
  else {
    if (c->emergencyStop || c->spikeDetected) setAllPixels(255, 0, 0);
    else if (c->turnState == STATE_CORNER_TURN) setAllPixels(255, 0, 255);
    else if (c->driveMode == MODE_OBSTACLE_AVOID) setAllPixels(255, 200, 0);
    else if (c->driveMode == MODE_LEFT_WALL) setAllPixels(0, 255, 200);
    else setAllPixels(0, 255, 80);
  }
  pixels.show();
}

// Set all NeoPixels to same color
void setAllPixels(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NEOPIXEL_COUNT; i++) {
    pixels.setPixelColor(i, pixels.Color(r, g, b));
  }
}

// ========== OLED Display Update (UI task, uses snapshot) ==========
static const char* mapStateStr(MapState s) {
  switch (s) { case MAP_IDLE: return "Idle"; case MAP_RECORDING: return "Rec"; case MAP_READY: return "Ready"; case MISSION_RUNNING: return "Mission"; }
  return "?";
}
static const char* driveModeStr(DriveMode m) {
  switch (m) { case MODE_RIGHT_WALL: return "R-Wall"; case MODE_LEFT_WALL: return "L-Wall"; case MODE_OBSTACLE_AVOID: return "OA"; default: return "?"; }
}

static void updateDisplayMenu(UIScreen screen, uint8_t menuIdx, const SensorData_t* sens, const ControlData_t* c) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);

  switch (screen) {
    case SCREEN_MENU: {
      const char* items[] = { "FreeRun", "Record", "Play", "Settings" };
      for (int i = 0; i < 4; i++) {
        if (i == menuIdx) display.print(">");
        else display.print(" ");
        display.print(items[i]);
        if (i == 2 && s_pathLen == 0) display.println(" (no mission)");
        else display.println();
      }
      break;
    }
    case SCREEN_FREERUN_MODE:
      display.println("FreeRun (D5=algo)");
      display.print(" ");
      display.println(driveModeStr(c->driveMode));
      display.println("D2=Start 3s");
      break;
    case SCREEN_RECORD_MODE:
      display.println("Record (D5=algo)");
      display.print(" ");
      display.println(driveModeStr(c->driveMode));
      display.println("D2=Start 3s");
      break;
    case SCREEN_COUNTDOWN: {
      unsigned long elapsed = millis() - s_countdownStartMs;
      int secLeft = ((int)COUNTDOWN_MS - (int)elapsed) / 1000;
      if (secLeft < 0) secLeft = 0;
      display.println("Get ready...");
      if (secLeft > 0) {
        display.print(secLeft);
        display.println(" sec");
      } else
        display.println("GO!");
      break;
    }
    case SCREEN_FREERUN:
      display.print(driveModeStr(c->driveMode));
      display.print(" ");
      display.println(c->currentStateStr);
      display.print("F:");
      display.print(sens->F);
      display.print(" R:");
      display.println(sens->R);
      display.println("BTN_MODE=Stop");
      break;
    case SCREEN_RECORDING:
      display.println("RECORDING");
      display.print("Pts:");
      display.println(c->pathLen);
      display.println("BTN_MAP=Save");
      display.println("BTN_MODE=Discard");
      break;
    case SCREEN_PLAY_INFO: {
      unsigned int timeSec = (unsigned int)((unsigned long)s_pathLen * (unsigned long)RECORD_INTERVAL_MS / 1000UL);
      display.println("MISSION INFO");
      display.print("Waypts:");
      display.print(s_turnWaypointCount);
      display.print(" Seg:");
      display.println(s_pathLen);
      display.print("Dist:");
      display.print((int)s_missionTotalDistMm);
      display.print("mm T:");
      display.print(timeSec);
      display.println("s");
      display.print("Turns:");
      display.println(s_turnWaypointCount);
      display.println("D2=Start 3s");
      break;
    }
    case SCREEN_MISSION: {
      int pct = (c->missionTotalDistMm > 0.1f) ? (int)(100.0f * c->missionDistMm / c->missionTotalDistMm) : 0;
      if (pct > 100) pct = 100;
      display.println(s_missionPaused ? "PAUSED" : "PLAY");
      display.print(pct);
      display.print("% ");
      display.print((int)c->missionDistMm);
      display.print("/");
      display.println((int)c->missionTotalDistMm);
      display.println("MODE=Stop MAP=Del/Pause");
      break;
    }
    case SCREEN_SETTINGS: {
      display.println("Settings");
      display.print(" ");
      display.println(settingsName(s_settingsIndex));
      display.println("D5=Next D2=Edit");
      display.println("D5 long=Discard D2 long=Save");
      break;
    }
    case SCREEN_SETTINGS_ITEM: {
      display.println(settingsName(s_settingsIndex));
      if (s_settingsIndex == 12) {
        display.println("D2 short=Calibrate");
      } else {
        display.print(" ");
        if (s_settingsIndex <= 3) display.println(s_settingsIndex == 0 ? KP_WALL : s_settingsIndex == 1 ? KD_WALL : s_settingsIndex == 2 ? KP_TURN : KD_TURN, 2);
        else if (s_settingsIndex == 4) display.println(BASE_SPEED);
        else if (s_settingsIndex == 5) display.println(RIGHT_SETPOINT);
        else if (s_settingsIndex == 6) display.println(CROSS_RIGHT_SETPOINT);
        else if (s_settingsIndex == 7) display.println(CORNER_TURN_DURATION);
        else if (s_settingsIndex == 8) display.println(CORNER_TURN_SPEED_L);
        else if (s_settingsIndex == 9) display.println(CORNER_TURN_SPEED_R);
        else if (s_settingsIndex == 10) display.println(LEFT_SETPOINT);
        else display.println(CROSS_LEFT_SETPOINT);
        display.println("D5=+ D2=-");
        display.println("D5 long=Back D2 long=Save");
      }
      break;
    }
    default:
      display.print(driveModeStr(c->driveMode));
      display.print(" ");
      display.print(mapStateStr(c->mapState));
      display.print(" ");
      display.println(c->currentStateStr);
      display.print("F:");
      display.print(sens->F);
      display.print(" L:");
      display.print(sens->L);
      display.print(" R:");
      display.println(sens->R);
      display.print("Enc:");
      display.print(sens->encLeft);
      display.print(",");
      display.print(sens->encRight);
      display.print(" H:");
      display.print((int)c->headingDeg);
      display.print(" M:");
      display.print(c->targetLeft);
      display.print(",");
      display.println(c->targetRight);
      break;
  }
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
    if (s_mutexControl != NULL && xSemaphoreTake(s_mutexControl, pdMS_TO_TICKS(50)) == pdTRUE) {
      s_control.emergencyStop = true;
      s_control.targetLeft = 0;
      s_control.targetRight = 0;
      xSemaphoreGive(s_mutexControl);
    }
    Serial.println("EMERGENCY STOP");
    return;
  }

  if (cmd == "GO" || cmd == "G") {
    if (s_mutexControl != NULL && xSemaphoreTake(s_mutexControl, pdMS_TO_TICKS(50)) == pdTRUE) {
      s_control.emergencyStop = false;
      xSemaphoreGive(s_mutexControl);
    }
    Serial.println("RUNNING");
    return;
  }

  if (cmd == "STATUS") {
    char st[16], md[8];
    DriveMode dm = MODE_RIGHT_WALL;
    MapState ms = MAP_IDLE;
    if (s_mutexControl != NULL && xSemaphoreTake(s_mutexControl, pdMS_TO_TICKS(50)) == pdTRUE) {
      strncpy(st, s_control.currentStateStr, sizeof(st) - 1);
      st[sizeof(st) - 1] = '\0';
      strncpy(md, s_control.currentModeStr, sizeof(md) - 1);
      md[sizeof(md) - 1] = '\0';
      dm = s_control.driveMode;
      ms = s_control.mapState;
      xSemaphoreGive(s_mutexControl);
    } else { st[0] = md[0] = '\0'; }
    Serial.println("\n===== Current Settings =====");
    Serial.print("Drive: "); Serial.println(driveModeStr(dm));
    Serial.print("Map: "); Serial.println(mapStateStr(ms));
    Serial.print("STATE: "); Serial.println(st);
    Serial.print("MODE: "); Serial.println(md);
    Serial.println("\nADAPTIVE PID GAINS:");
    Serial.print("  WALL: KP="); Serial.print(KP_WALL, 2);
    Serial.print(", KD="); Serial.println(KD_WALL, 2);
    Serial.print("  TURN: KP="); Serial.print(KP_TURN, 2);
    Serial.print(", KD="); Serial.println(KD_TURN, 2);
    Serial.println("\nCORNER TURN:");
    Serial.print("  DURATION: "); Serial.print(CORNER_TURN_DURATION); Serial.println(" ms");
    Serial.print("  CT_WAIT: "); Serial.print(CT_WAIT); Serial.println(" ms");
    Serial.print("  CORNER_FORWARD_SPEED: "); Serial.println(CORNER_FORWARD_SPEED);
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
    return;
  }

  if (cmd == "SPIKE OFF") {
    SPIKE_STOP_ENABLED = false;
    Serial.println("Spike detection OFF");
    return;
  }

  if (cmd == "SAVE") {
    settingsSaveToEEPROM();
    Serial.println("Settings saved to EEPROM.");
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
    else if (param == "CTW") {
      CT_WAIT = valueStr.toInt();
      Serial.print("CT_WAIT: "); Serial.print(CT_WAIT); Serial.println(" ms");
    }
    else if (param == "CFWD") {
      CORNER_FORWARD_SPEED = valueStr.toInt();
      Serial.print("CORNER_FORWARD_SPEED: "); Serial.println(CORNER_FORWARD_SPEED);
    }
    else {
      Serial.println("Unknown parameter");
      return;
    }
  }
}

// ========== Help Display ==========
void printHelp() {
  Serial.println("\n========== Right Wall Follower PRO ==========");
  Serial.println("UI: Menu type. D2 long = open menu.");
  Serial.println("D5 = scroll / change mode. D2 short = select / action.");
  Serial.println("");
  Serial.println("D2 during run:");
  Serial.println("  Idle + D2 short = Start recording (map)");
  Serial.println("  Recording + D2 short = Stop bot, show mapping");
  Serial.println("  Mapping shown + D2 short = Follow path (mission)");
  Serial.println("  When follow done = Auto-stop, back to mapping view");
  Serial.println("D2 long = Open menu (Run, Mode, Map Rec, Calibrate)");
  Serial.println("MPU6050: heading for mapping. Calibrate from menu.");
  Serial.println("\nCommands: HELP, STOP, GO, STATUS, SAVE, SPIKE ON/OFF");
  Serial.println("Params (e.g. RS 120): RS, CRS, BS, KPW, KDW, KPT, KDT,");
  Serial.println("  CTD, CTL, CTR, CTW, CFWD. SAVE = write all to EEPROM.");
  Serial.println("============================================\n");
}
