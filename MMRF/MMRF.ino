/*
 * Maze Solving Robot - Main Firmware
 * ESP32 WROOM 30 Pin
 */

#include <Arduino.h>
#include <Wire.h>
#include "Config.h"
#include "Maze.h"
#include "Motion.h"
#include "Odometry.h"
#include "Pathfinding.h"
#include "Sensors.h"
#include "Types.h"
#include "UI.h"

Maze maze;
Odometry odometry;
Sensors sensors;
Motion motion;
UI ui;
Pathfinding pathfinding;

RobotState robotState = STATE_INIT;
ExploreState exploreState = EXPLORE_CHECK_WALLS;
MenuItem selectedMenu = MENU_EXPLORE;

uint8_t currentX = 0;
uint8_t currentY = 0;
Heading currentHeading = kNorth;

void setup() {
  Serial.begin(115200);
  delay(300);

  ui.begin();
  sensors.begin();
  motion.begin();
  odometry.begin();

  ui.showSplash();
  ui.setStatusColor(0, 255, 0);

  robotState = STATE_MENU;
}

void loop() {
  switch (robotState) {
    case STATE_MENU:
      ui.showMenu(selectedMenu);
      if (ui.modePressed()) {
        selectedMenu = static_cast<MenuItem>((selectedMenu + 1) % MENU_COUNT);
        delay(250);
      }
      if (ui.selectPressed()) {
        if (selectedMenu == MENU_EXPLORE) robotState = STATE_EXPLORE;
        if (selectedMenu == MENU_SPEED_RUN) robotState = STATE_SPEED_RUN;
        if (selectedMenu == MENU_EXPLORE_AND_RUN) robotState = STATE_EXPLORE;
        if (selectedMenu == MENU_FREE_RUN) robotState = STATE_FREE_RUN;
        if (selectedMenu == MENU_DIAGNOSTICS) robotState = STATE_DIAGNOSTICS;
        delay(300);
      }
      break;
    case STATE_EXPLORE:
      sensors.read();
      maze.setVisited(currentX, currentY, true);
      {
        WallReading walls = sensors.detectWalls(currentHeading);
        maze.setWall(currentX, currentY, kNorth, walls.north);
        maze.setWall(currentX, currentY, kEast, walls.east);
        maze.setWall(currentX, currentY, kSouth, walls.south);
        maze.setWall(currentX, currentY, kWest, walls.west);
      }
      ui.showState("Explore", currentX, currentY);
      delay(200);
      break;
    case STATE_SPEED_RUN:
      ui.showState("SpeedRun", currentX, currentY);
      delay(200);
      break;
    case STATE_FREE_RUN:
      ui.showState("FreeRun", currentX, currentY);
      delay(200);
      break;
    case STATE_DIAGNOSTICS:
      ui.showState("Diag", currentX, currentY);
      delay(200);
      break;
    case STATE_PLAN_PATH: {
      PathStep steps[64];
      uint8_t count = pathfinding.dijkstra(maze, 0, 0, currentX, currentY, steps, 64);
      Serial.print("Path steps: ");
      Serial.println(count);
      robotState = STATE_MENU;
      break;
    }
    default:
      robotState = STATE_MENU;
      break;
  }
}
