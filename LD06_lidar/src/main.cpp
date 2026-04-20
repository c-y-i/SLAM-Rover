#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "imu_reader.h"
#include "lidar_reader.h"
#include "web_viewer.h"

namespace
{

HardwareSerial &lidarSerial = Serial1;
lidar::Reader lidar_reader(lidarSerial);
WebViewer web_viewer(lidar_reader);
#if IMU_TYPE == IMU_BNO085
imu::Bno085Reader imu_reader;
#else
imu::Mpu6050Reader imu_reader;
#endif
bool imu_present = false;
uint8_t imu_addr = 0;

constexpr uint32_t kImuIntervalMs = 10; // 100 Hz
uint32_t last_imu_ms = 0;

void print_json_status(const char *stage, const char *detail)
{
  Serial.print(F("{\"t\":\"status\",\"stage\":\""));
  Serial.print(stage);
  Serial.print(F("\",\"detail\":\""));
  Serial.print(detail);
  Serial.println(F("\"}"));
}

void print_json_scan(const lidar::ScanFrame &scan)
{
  Serial.print(F("{\"t\":\"scan\",\"x\":["));
  bool first = true;
  for (uint16_t i = 0; i < scan.point_count; ++i)
  {
    if (!scan.points[i].valid)
      continue;
    if (!first)
      Serial.print(',');
    Serial.print(scan.points[i].x_mm);
    first = false;
  }
  Serial.print(F("],\"y\":["));
  first = true;
  for (uint16_t i = 0; i < scan.point_count; ++i)
  {
    if (!scan.points[i].valid)
      continue;
    if (!first)
      Serial.print(',');
    Serial.print(scan.points[i].y_mm);
    first = false;
  }
  Serial.print(F("],\"i\":["));
  first = true;
  for (uint16_t i = 0; i < scan.point_count; ++i)
  {
    if (!scan.points[i].valid)
      continue;
    if (!first)
      Serial.print(',');
    Serial.print(scan.points[i].intensity);
    first = false;
  }
  Serial.println(F("]}"));
}

void print_json_imu(const imu::Reading &r)
{
  Serial.print(F("{\"t\":\"imu\",\"accel_mps2\":["));
  Serial.print(r.accel_mps2[0], 4);
  Serial.print(',');
  Serial.print(r.accel_mps2[1], 4);
  Serial.print(',');
  Serial.print(r.accel_mps2[2], 4);
  Serial.print(F("],\"gyro_rads\":["));
  Serial.print(r.gyro_rads[0], 5);
  Serial.print(',');
  Serial.print(r.gyro_rads[1], 5);
  Serial.print(',');
  Serial.print(r.gyro_rads[2], 5);
  Serial.print(F("],\"ts_us\":"));
  Serial.print(r.ts_us);
  Serial.println(F("}"));
}

constexpr uint32_t kHeartbeatIntervalMs = 3000;
uint32_t last_heartbeat_ms = 0;

void print_json_heartbeat()
{
  Serial.print(F("{\"t\":\"status\",\"stage\":\"heartbeat\",\"detail\":\"uptime_ms="));
  Serial.print(millis());
  Serial.print(F(",packets="));
  Serial.print(lidar_reader.packets_seen());
  Serial.print(F(",imu="));
  Serial.print(imu_present ? F("1") : F("0"));
  Serial.println(F("\"}"));
}

} // namespace

void setup()
{
  Serial.begin(config::kUsbBaud);
  delay(2000); // wait for serial monitor to open

  print_json_status("startup", "LD06 lidar reader starting");

  Wire.begin(config::kI2cSda, config::kI2cScl);
  Wire.setClock(config::kI2cClockHz);

#if IMU_TYPE == IMU_BNO085
  for (uint8_t candidate : {config::kBno085PrimaryAddr, config::kBno085AlternateAddr})
#else
  for (uint8_t candidate : {config::kImuPrimaryAddr, config::kImuAlternateAddr})
#endif
  {
    Wire.beginTransmission(candidate);
    if (Wire.endTransmission() == 0)
    {
      imu_addr = candidate;
      break;
    }
  }

  if (imu_addr != 0)
  {
    imu_present = imu_reader.begin(imu_addr);
    print_json_status("imu", imu_present ? "found" : "init_failed");
  }
  else
  {
    print_json_status("imu", "not_found");
  }

  lidar_reader.begin(config::kLd06RxPin, config::kLd06TxPin, config::kLd06Baud);

  print_json_status("startup", "wifi_connecting");
  web_viewer.begin(); // may block up to 15 s for WiFi
  print_json_status("startup", "running");
}

void loop()
{
  if (lidar_reader.read_scan())
    print_json_scan(lidar_reader.latest_scan());

  if (imu_present)
  {
    const uint32_t now = millis();
    if (now - last_imu_ms >= kImuIntervalMs)
    {
      last_imu_ms = now;
      imu::Reading r;
      if (imu_reader.read(r))
        print_json_imu(r);
    }
  }

  const uint32_t now = millis();
  if (now - last_heartbeat_ms >= kHeartbeatIntervalMs)
  {
    last_heartbeat_ms = now;
    print_json_heartbeat();
  }

  web_viewer.loop();
}
