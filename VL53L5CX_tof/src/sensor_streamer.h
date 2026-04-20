#pragma once

#include <Arduino.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <Wire.h>

#include "config.h"
#include "tof_data.h"

#if IMU_TYPE == IMU_BNO085
#include <Adafruit_BNO08x.h>
#else
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#endif

namespace sensor
{

class SensorStreamer
{
public:
  void begin();
  void poll();

private:
  static constexpr uint8_t kWirePacketSize = 128;

  bool probe_address(uint8_t address);
  void initialize_wire();
  void initialize_tof();
  void initialize_imu();
  void try_initialize_missing_devices();
  void poll_tof();
  void poll_imu(uint32_t now_us);

  void emit_status_packet(const char *stage, const char *detail);
  void emit_i2c_scan_status();
  void emit_tof_error_packet(const char *action);
  void emit_tof_packet();
  void emit_imu_packet();

  bool wire_ready_ = false;
  bool tof_ready_ = false;
  bool imu_ready_ = false;
  uint8_t imu_address_ = 0;
  uint32_t last_recovery_attempt_ms_ = 0;
  uint32_t last_imu_emit_us_ = 0;

  SparkFun_VL53L5CX tof_sensor_;
#if IMU_TYPE == IMU_BNO085
  Adafruit_BNO08x imu_sensor_;
#else
  Adafruit_MPU6050 imu_sensor_;
#endif
  VL53L5CX_ResultsData tof_results_{};
  TofFrame tof_frame_{};
  ImuSample imu_sample_{};
};

} // namespace sensor
