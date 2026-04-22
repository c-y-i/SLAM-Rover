#pragma once

#include <Arduino.h>
#include <esp_now.h>

#include "lidar_reader.h"

struct ImuTelemetrySample
{
  uint32_t ts_us = 0;
  float accel_mps2[3]{};
  float gyro_rads[3]{};
  float yaw_deg = 0.0f;
  float yaw_rate_dps = 0.0f;
  bool imu_ok = false;
};

class TelemetryLink
{
public:
  void update_peer_from_source(const esp_now_recv_info *info);
  bool has_controller_peer() const;
  void send_scan(const lidar::ScanFrame &scan);
  void send_imu(const ImuTelemetrySample &sample);

private:
  uint16_t telemetry_frame_id_ = 0;
  uint16_t imu_frame_id_ = 0;
  bool controller_peer_known_ = false;
  uint8_t controller_peer_addr_[6]{};
};

