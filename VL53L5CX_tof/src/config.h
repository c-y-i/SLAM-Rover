#pragma once

#include <Arduino.h>

#define IMU_MPU6050 0
#define IMU_BNO085  1
#ifndef IMU_TYPE
#define IMU_TYPE IMU_MPU6050
#endif

namespace config
{

constexpr char kProtocolVersion[] = "0.2.0";
constexpr uint32_t kUsbBaud = 460800;
constexpr uint32_t kTofI2cClockHz = 400000;
constexpr uint8_t kTofResolution = 8 * 8;
constexpr uint8_t kTofRangingFrequencyHz = 15;
constexpr uint32_t kImuIntervalUs = 10000;
constexpr uint32_t kRecoveryRetryMs = 5000;

constexpr int kTofSdaPin = 22;
constexpr int kTofSclPin = 20;
constexpr uint8_t kTofAddress = 0x29;
constexpr uint8_t kImuPrimaryAddress = 0x68;
constexpr uint8_t kImuAlternateAddress = 0x69;
constexpr uint8_t kBno085PrimaryAddress = 0x4A;
constexpr uint8_t kBno085AlternateAddress = 0x4B;

} // namespace config
