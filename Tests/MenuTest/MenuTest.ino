#include <Arduino.h>
#include "../../MMRF/Types.h"
#include "../../MMRF/UI.h"

UI ui;
MenuItem selected = MENU_EXPLORE;

void setup() {
  Serial.begin(115200);
  delay(300);
  ui.begin();
  ui.showMenu(selected);
  Serial.println("Menu Test Ready");
}

void loop() {
  if (ui.modePressed()) {
    selected = static_cast<MenuItem>((selected + 1) % MENU_COUNT);
    ui.showMenu(selected);
    delay(250);
  }

  if (ui.selectPressed()) {
    Serial.print("Selected menu index: ");
    Serial.println(selected);
    delay(300);
  }
}
