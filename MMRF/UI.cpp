#include "UI.h"
#include <Arduino.h>

UI::UI() : display_(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET),
           pixels_(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800) {}

void UI::begin() {
  pinMode(BTN_MODE, INPUT_PULLUP);
  pinMode(BTN_SELECT, INPUT_PULLUP);

  pixels_.begin();
  pixels_.setBrightness(50);
  setStatusColor(0, 0, 255);

  display_.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display_.clearDisplay();
  display_.setTextSize(1);
  display_.setTextColor(SSD1306_WHITE);
  display_.display();
}

void UI::setStatusColor(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NEOPIXEL_COUNT; i++) {
    pixels_.setPixelColor(i, pixels_.Color(r, g, b));
  }
  pixels_.show();
}

void UI::showSplash() {
  display_.clearDisplay();
  display_.setCursor(0, 0);
  display_.println("MicroMouse");
  display_.println("Ready");
  display_.display();
}

void UI::showMenu(MenuItem selected) {
  static const char *items[MENU_COUNT] = {
      "Explore Maze", "Speed Run", "Explore+Run", "Free Run", "Diagnostics"};

  display_.clearDisplay();
  display_.setCursor(0, 0);
  display_.println("Select Mode:");
  for (uint8_t i = 0; i < MENU_COUNT; i++) {
    if (i == selected) {
      display_.print("> ");
    } else {
      display_.print("  ");
    }
    display_.println(items[i]);
  }
  display_.display();
}

void UI::showState(const char *state, uint8_t x, uint8_t y) {
  display_.clearDisplay();
  display_.setCursor(0, 0);
  display_.print("State: ");
  display_.println(state);
  display_.print("Cell: ");
  display_.print(x);
  display_.print(",");
  display_.println(y);
  display_.display();
}

bool UI::modePressed() const { return digitalRead(BTN_MODE) == LOW; }

bool UI::selectPressed() const { return digitalRead(BTN_SELECT) == LOW; }
