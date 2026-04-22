#pragma once

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

class StatusLed
{
public:
  explicit StatusLed(int pin);

  void begin(uint8_t brightness);
  void set_error();
  void set_waiting();
  void mark_activity();
  void update(unsigned long now_ms);

private:
  void set_color(uint8_t r, uint8_t g, uint8_t b);

  Adafruit_NeoPixel led_;
  volatile bool activity_flag_ = false;
  uint32_t base_color_ = 0;
  uint32_t last_shown_color_ = 0xFFFFFFFF;
  unsigned long flash_end_ms_ = 0;
};

