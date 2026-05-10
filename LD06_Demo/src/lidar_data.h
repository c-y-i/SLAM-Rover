#pragma once

#include <LD06_Packet.h>

namespace lidar
{

constexpr uint8_t kPacketHeader = kLD06PacketHeader;
constexpr uint8_t kPacketLength = kLD06PacketLength;
constexpr size_t kPointsPerPacket = kLD06PointsPerPacket;
constexpr size_t kPacketSize = kLD06PacketSize;
constexpr size_t kMaxPointsPerScan = kLD06MaxPointsPerScan;
constexpr float kMaxAngleStepDeg = kLD06MaxAngleStepDeg;
constexpr uint16_t kMaxDisplayDistanceMm = kLD06MaxDisplayDistanceMm;

using ScanPoint = LD06ScanPoint;
using PacketSummary = LD06PacketSummary;
using ScanFrame = LD06ScanFrame;

} // namespace lidar
