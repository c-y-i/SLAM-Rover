#pragma once

#include <Arduino.h>

namespace sensor
{

constexpr uint8_t kGridWidth = 8;
constexpr uint8_t kGridHeight = 8;
constexpr size_t kZoneCount = kGridWidth * kGridHeight;
constexpr uint8_t kValidTargetStatus = 5;
constexpr uint8_t kValidTargetStatusNoWrap = 9;

struct ImuSample
{
  uint32_t ts_us = 0;
  float accel_mps2[3]{};
  float gyro_rads[3]{};
  float temp_c = 0.0f;
  bool valid = false;
};

struct TofFrame
{
  uint32_t seq = 0;
  uint32_t ts_us = 0;
  uint16_t distances_mm[kZoneCount]{};
  uint8_t status[kZoneCount]{};
  uint8_t valid_count = 0;
  bool valid = false;
};

inline bool is_tof_status_valid(uint8_t status)
{
  return status == kValidTargetStatus || status == kValidTargetStatusNoWrap;
}

} // namespace sensor
