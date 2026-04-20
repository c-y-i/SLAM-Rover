#include "imu_reader.h"

#include <Wire.h>

namespace imu
{

namespace
{

constexpr float kG = 9.80665f;
constexpr float kAccelScale = kG / 16384.0f;                    // ±2g range
constexpr float kGyroScale = (float)(M_PI / 180.0) / 131.0f;   // ±250 deg/s range

} // namespace

bool Mpu6050Reader::begin(uint8_t addr)
{
  addr_ = addr;

  Wire.beginTransmission(addr_);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0x00); // wake up
  if (Wire.endTransmission() != 0)
    return false;

  Wire.beginTransmission(addr_);
  Wire.write(0x1C); // ACCEL_CONFIG
  Wire.write(0x00); // ±2g
  Wire.endTransmission();

  Wire.beginTransmission(addr_);
  Wire.write(0x1B); // GYRO_CONFIG
  Wire.write(0x00); // ±250 deg/s
  Wire.endTransmission();

  return true;
}

bool Mpu6050Reader::read(Reading &out)
{
  Wire.beginTransmission(addr_);
  Wire.write(0x3B); // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0)
    return false;

  if (Wire.requestFrom(addr_, static_cast<uint8_t>(14)) != 14)
    return false;

  int16_t raw[7];
  for (int i = 0; i < 7; ++i)
    raw[i] = static_cast<int16_t>((Wire.read() << 8) | Wire.read());

  out.accel_mps2[0] = raw[0] * kAccelScale;
  out.accel_mps2[1] = raw[1] * kAccelScale;
  out.accel_mps2[2] = raw[2] * kAccelScale;
  // raw[3] is temperature, skipped
  out.gyro_rads[0] = raw[4] * kGyroScale;
  out.gyro_rads[1] = raw[5] * kGyroScale;
  out.gyro_rads[2] = raw[6] * kGyroScale;
  out.ts_us = micros();

  return true;
}


#if IMU_TYPE == IMU_BNO085

bool Bno085Reader::begin(uint8_t addr)
{
  if (!bno_.begin_I2C(addr, &Wire))
    return false;

  bno_.enableReport(SH2_ACCELEROMETER, 10000);
  bno_.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
  return true;
}

bool Bno085Reader::read(Reading &out)
{
  sh2_SensorValue_t val;
  bool got_accel = false;
  bool got_gyro = false;

  while (bno_.getSensorEvent(&val))
  {
    switch (val.sensorId)
    {
    case SH2_ACCELEROMETER:
      out.accel_mps2[0] = val.un.accelerometer.x;
      out.accel_mps2[1] = val.un.accelerometer.y;
      out.accel_mps2[2] = val.un.accelerometer.z;
      got_accel = true;
      break;
    case SH2_GYROSCOPE_CALIBRATED:
      out.gyro_rads[0] = val.un.gyroscope.x;
      out.gyro_rads[1] = val.un.gyroscope.y;
      out.gyro_rads[2] = val.un.gyroscope.z;
      got_gyro = true;
      break;
    }
  }

  out.ts_us = micros();
  return got_accel && got_gyro;
}

#endif

} // namespace imu
