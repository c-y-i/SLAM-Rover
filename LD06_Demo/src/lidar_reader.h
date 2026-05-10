#pragma once

#include <LD06_LiDAR.h>

namespace lidar
{

class Reader : public LD06_LiDAR
{
public:
  explicit Reader(HardwareSerial &serial_port) : LD06_LiDAR(serial_port) {}

  void begin(int rx_pin, int tx_pin, uint32_t baud_rate)
  {
    LD06_LiDAR::begin(baud_rate, rx_pin, tx_pin);
  }
  bool read_scan() { return update(); }
  void print_packet_summary(Stream &stream) const { printPacketSummary(stream); }
  const LD06PacketSummary &last_packet() const { return lastPacket(); }
  const LD06ScanFrame &latest_scan() const { return latestScan(); }
  uint32_t packets_seen() const { return packetsSeen(); }
};

} // namespace lidar
