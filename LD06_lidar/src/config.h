#pragma once

#include <Arduino.h>

#ifndef LIDAR_DEBUG_LOGS
#define LIDAR_DEBUG_LOGS 0
#endif

#define IMU_MPU6050 0
#define IMU_BNO085  1
#ifndef IMU_TYPE
#define IMU_TYPE IMU_MPU6050
#endif

namespace config
{

constexpr uint32_t kUsbBaud = 460800;
constexpr uint32_t kLd06Baud = 230400;
constexpr uint32_t kWifiConnectTimeoutMs = 15000;

constexpr int kLd06RxPin = 7;
constexpr int kLd06TxPin = 8;

constexpr int kI2cSda = 22;
constexpr int kI2cScl = 20;
constexpr uint32_t kI2cClockHz = 400000;
constexpr uint8_t kImuPrimaryAddr = 0x68;
constexpr uint8_t kImuAlternateAddr = 0x69;
constexpr uint8_t kBno085PrimaryAddr = 0x4A;
constexpr uint8_t kBno085AlternateAddr = 0x4B;
constexpr char kWifiSsid[] = "Verizon69";
constexpr char kWifiPassword[] = "Based69420";
const IPAddress kWifiStaticIp(192, 168, 1, 69);
const IPAddress kWifiGateway(192, 168, 1, 1);
const IPAddress kWifiSubnet(255, 255, 255, 0);
const IPAddress kWifiDns(8, 8, 8, 8);

} // namespace config
