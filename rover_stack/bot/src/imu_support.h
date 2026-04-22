#pragma once

#include <Adafruit_BNO08x.h>
#include <Arduino.h>

struct ImuState
{
  bool ready = false;
  bool have_accel = false;
  bool have_gyro = false;
  bool have_yaw = false;
  uint8_t address = 0;
  unsigned long last_init_attempt_ms = 0;
  unsigned long last_sample_ms = 0;
  unsigned long last_telemetry_ms = 0;
  uint32_t ts_us = 0;
  float accel_mps2[3]{};
  float gyro_rads[3]{};
  float yaw_deg = 0.0f;
  float yaw_rate_dps = 0.0f;
};

using StatusEmitter = void (*)(const char *stage, const char *detail);

void imu_setup_i2c(int sda_pin,
                   int scl_pin,
                   uint32_t i2c_clock_hz,
                   StatusEmitter emit_status);

void imu_poll(Adafruit_BNO08x &imu_sensor,
              ImuState &imu_state,
              uint8_t primary_address,
              uint8_t alternate_address,
              uint32_t report_interval_us,
              unsigned long retry_ms,
              StatusEmitter emit_status);

bool imu_is_healthy(const ImuState &imu_state,
                    unsigned long imu_fresh_ms,
                    unsigned long now_ms);

