#include "telemetry_link.h"

#include <string.h>

namespace
{
constexpr uint8_t kTelemetryMagic = 0xA5;
constexpr uint8_t kTelemetryVersion = 1;
constexpr uint8_t kTelemetryTypeScanChunk = 1;
constexpr uint8_t kTelemetryTypeImu = 2;
constexpr uint8_t kTelemetryPointsPerChunk = 48;
constexpr uint8_t kTelemetryMaxPoints = 96;

struct __attribute__((packed)) TelemetryHeader
{
  uint8_t magic;
  uint8_t version;
  uint8_t type;
  uint16_t frame_id;
  uint8_t chunk_index;
  uint8_t chunk_count;
  uint8_t point_count;
  uint8_t total_points;
};

struct __attribute__((packed)) TelemetryPoint
{
  int16_t x_mm;
  int16_t y_mm;
  uint8_t intensity;
};

struct __attribute__((packed)) TelemetryImuPayload
{
  uint32_t ts_us;
  float accel_mps2[3];
  float gyro_rads[3];
  float yaw_deg;
  float yaw_rate_dps;
  uint8_t imu_ok;
};
} // namespace

void TelemetryLink::update_peer_from_source(const esp_now_recv_info *info)
{
  if (info == nullptr || info->src_addr == nullptr)
  {
    return;
  }

  if (controller_peer_known_ &&
      memcmp(controller_peer_addr_, info->src_addr, sizeof(controller_peer_addr_)) == 0)
  {
    return;
  }

  if (controller_peer_known_ && esp_now_is_peer_exist(controller_peer_addr_))
  {
    esp_now_del_peer(controller_peer_addr_);
  }
  controller_peer_known_ = false;

  esp_now_peer_info_t controller_peer = {};
  memcpy(controller_peer.peer_addr, info->src_addr, sizeof(controller_peer.peer_addr));
  controller_peer.channel = 0;
  controller_peer.encrypt = false;

  if (esp_now_add_peer(&controller_peer) == ESP_OK)
  {
    memcpy(controller_peer_addr_, info->src_addr, sizeof(controller_peer_addr_));
    controller_peer_known_ = true;
  }
}

bool TelemetryLink::has_controller_peer() const
{
  return controller_peer_known_;
}

void TelemetryLink::send_scan(const lidar::ScanFrame &scan)
{
  if (!controller_peer_known_)
  {
    return;
  }

  TelemetryPoint sampled_points[kTelemetryMaxPoints]{};
  uint8_t sampled_count = 0;

  if (scan.valid_point_count > 0)
  {
    const uint16_t desired_points =
        (scan.valid_point_count < kTelemetryMaxPoints)
            ? scan.valid_point_count
            : static_cast<uint16_t>(kTelemetryMaxPoints);
    uint16_t valid_index = 0;
    uint8_t selected = 0;

    for (uint16_t index = 0; index < scan.point_count && selected < desired_points; ++index)
    {
      const lidar::ScanPoint &point = scan.points[index];
      if (!point.valid)
      {
        continue;
      }

      const uint16_t bucket =
          static_cast<uint32_t>(valid_index) * desired_points / scan.valid_point_count;
      if (bucket == selected)
      {
        sampled_points[selected].x_mm = point.x_mm;
        sampled_points[selected].y_mm = point.y_mm;
        sampled_points[selected].intensity = point.intensity;
        ++selected;
      }
      ++valid_index;
    }
    sampled_count = selected;
  }

  if (sampled_count == 0)
  {
    return;
  }

  ++telemetry_frame_id_;
  const uint8_t chunk_count =
      (sampled_count + kTelemetryPointsPerChunk - 1) / kTelemetryPointsPerChunk;

  for (uint8_t chunk_index = 0; chunk_index < chunk_count; ++chunk_index)
  {
    const uint8_t point_offset = chunk_index * kTelemetryPointsPerChunk;
    const uint8_t remaining = sampled_count - point_offset;
    const uint8_t points_in_chunk =
        (remaining < kTelemetryPointsPerChunk) ? remaining : kTelemetryPointsPerChunk;
    const size_t packet_size = sizeof(TelemetryHeader) + points_in_chunk * sizeof(TelemetryPoint);

    uint8_t packet_buffer[sizeof(TelemetryHeader) +
                          kTelemetryPointsPerChunk * sizeof(TelemetryPoint)]{};

    TelemetryHeader header{
        kTelemetryMagic,
        kTelemetryVersion,
        kTelemetryTypeScanChunk,
        telemetry_frame_id_,
        chunk_index,
        chunk_count,
        points_in_chunk,
        sampled_count,
    };

    memcpy(packet_buffer, &header, sizeof(header));
    memcpy(packet_buffer + sizeof(header),
           &sampled_points[point_offset],
           points_in_chunk * sizeof(TelemetryPoint));
    esp_now_send(controller_peer_addr_, packet_buffer, packet_size);
  }
}

void TelemetryLink::send_imu(const ImuTelemetrySample &sample)
{
  if (!controller_peer_known_)
  {
    return;
  }

  TelemetryHeader header{
      kTelemetryMagic,
      kTelemetryVersion,
      kTelemetryTypeImu,
      imu_frame_id_++,
      0,
      1,
      0,
      0,
  };

  TelemetryImuPayload payload{};
  payload.ts_us = sample.ts_us;
  payload.accel_mps2[0] = sample.accel_mps2[0];
  payload.accel_mps2[1] = sample.accel_mps2[1];
  payload.accel_mps2[2] = sample.accel_mps2[2];
  payload.gyro_rads[0] = sample.gyro_rads[0];
  payload.gyro_rads[1] = sample.gyro_rads[1];
  payload.gyro_rads[2] = sample.gyro_rads[2];
  payload.yaw_deg = sample.yaw_deg;
  payload.yaw_rate_dps = sample.yaw_rate_dps;
  payload.imu_ok = sample.imu_ok ? 1U : 0U;

  uint8_t packet[sizeof(TelemetryHeader) + sizeof(TelemetryImuPayload)]{};
  memcpy(packet, &header, sizeof(header));
  memcpy(packet + sizeof(header), &payload, sizeof(payload));
  esp_now_send(controller_peer_addr_, packet, sizeof(packet));
}
