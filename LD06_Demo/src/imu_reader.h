#pragma once

#include <Arduino.h>
#include "config.h"

#if IMU_TYPE == IMU_BNO085
#include <Adafruit_BNO08x.h>
#endif

namespace imu
{

struct Reading
{
  float accel_mps2[3];
  float gyro_rads[3];
  uint32_t ts_us;
};

class Mpu6050Reader
{
public:
  bool begin(uint8_t addr = 0x68);
  bool read(Reading &out);

private:
  uint8_t addr_ = 0x68;
};

#if IMU_TYPE == IMU_BNO085
class Bno085Reader
{
public:
  bool begin(uint8_t addr = 0x4A);
  bool read(Reading &out);

private:
  Adafruit_BNO08x bno_;
};
#endif

} // namespace imu
