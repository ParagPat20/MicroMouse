#ifndef MMRF_UI_H
#define MMRF_UI_H

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include "Pins.h"
#include "Types.h"

class UI {
 public:
  UI();
  void begin();
  void setStatusColor(uint8_t r, uint8_t g, uint8_t b);
  void showSplash();
  void showMenu(MenuItem selected);
  void showState(const char *state, uint8_t x, uint8_t y);
  bool modePressed() const;
  bool selectPressed() const;

 private:
  Adafruit_SSD1306 display_;
  Adafruit_NeoPixel pixels_;
};

#endif
