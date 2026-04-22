#include "status_led.h"

namespace
{
constexpr uint8_t kLedCount = 1;
constexpr uint8_t kErrorR = 200;
constexpr uint8_t kErrorG = 0;
constexpr uint8_t kErrorB = 0;
constexpr uint8_t kWaitingR = 200;
constexpr uint8_t kWaitingG = 100;
constexpr uint8_t kWaitingB = 0;
constexpr uint8_t kReadyR = 0;
constexpr uint8_t kReadyG = 200;
constexpr uint8_t kReadyB = 0;
constexpr uint8_t kActivityR = 0;
constexpr uint8_t kActivityG = 0;
constexpr uint8_t kActivityB = 200;
constexpr unsigned long kActivityFlashMs = 300;
}

StatusLed::StatusLed(int pin) : led_(kLedCount, pin, NEO_GRB + NEO_KHZ800)
{
}

void StatusLed::set_color(uint8_t r, uint8_t g, uint8_t b)
{
  led_.setPixelColor(0, led_.Color(r, g, b));
  led_.show();
}

void StatusLed::begin(uint8_t brightness)
{
  led_.begin();
  led_.setBrightness(brightness);
  set_waiting();
}

void StatusLed::set_error()
{
  base_color_ = led_.Color(kErrorR, kErrorG, kErrorB);
  set_color(kErrorR, kErrorG, kErrorB);
  last_shown_color_ = base_color_;
}

void StatusLed::set_waiting()
{
  base_color_ = led_.Color(kWaitingR, kWaitingG, kWaitingB);
  set_color(kWaitingR, kWaitingG, kWaitingB);
  last_shown_color_ = base_color_;
}

void StatusLed::mark_activity()
{
  activity_flag_ = true;
}

void StatusLed::update(unsigned long now_ms)
{
  if (activity_flag_)
  {
    activity_flag_ = false;
    const uint32_t error_color = led_.Color(kErrorR, kErrorG, kErrorB);
    if (base_color_ != error_color)
    {
      base_color_ = led_.Color(kReadyR, kReadyG, kReadyB);
    }
    flash_end_ms_ = now_ms + kActivityFlashMs;
    set_color(kActivityR, kActivityG, kActivityB);
    last_shown_color_ = led_.Color(kActivityR, kActivityG, kActivityB);
    return;
  }

  if (now_ms <= flash_end_ms_)
  {
    return;
  }

  if (base_color_ != last_shown_color_)
  {
    const uint8_t r = (base_color_ >> 16) & 0xFF;
    const uint8_t g = (base_color_ >> 8) & 0xFF;
    const uint8_t b = base_color_ & 0xFF;
    set_color(r, g, b);
    last_shown_color_ = base_color_;
  }
}

