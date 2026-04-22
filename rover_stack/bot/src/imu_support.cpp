#include "imu_support.h"

#include <Wire.h>
#include <math.h>
#include <stdio.h>

namespace
{
bool probe_i2c(uint8_t address)
{
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

float yaw_from_quaternion_wxyz(float w, float x, float y, float z)
{
  const float siny_cosp = 2.0f * (w * z + x * y);
  const float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
  return atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;
}

void initialize_imu(Adafruit_BNO08x &imu_sensor,
                    ImuState &imu_state,
                    uint8_t primary_address,
                    uint8_t alternate_address,
                    uint32_t report_interval_us,
                    unsigned long retry_ms,
                    StatusEmitter emit_status)
{
  if (imu_state.ready)
  {
    return;
  }

  const unsigned long now_ms = millis();
  if (now_ms - imu_state.last_init_attempt_ms < retry_ms)
  {
    return;
  }
  imu_state.last_init_attempt_ms = now_ms;

  uint8_t candidate_address = 0;
  if (probe_i2c(primary_address))
  {
    candidate_address = primary_address;
  }
  else if (probe_i2c(alternate_address))
  {
    candidate_address = alternate_address;
  }

  if (candidate_address == 0)
  {
    emit_status("imu", "probe_failed");
    return;
  }

  if (!imu_sensor.begin_I2C(candidate_address, &Wire))
  {
    emit_status("imu", "begin_failed");
    return;
  }

  imu_sensor.enableReport(SH2_ACCELEROMETER, report_interval_us);
  imu_sensor.enableReport(SH2_GYROSCOPE_CALIBRATED, report_interval_us);
  imu_sensor.enableReport(SH2_ROTATION_VECTOR, report_interval_us);

  imu_state.ready = true;
  imu_state.have_accel = false;
  imu_state.have_gyro = false;
  imu_state.have_yaw = false;
  imu_state.address = candidate_address;
  imu_state.last_sample_ms = 0;
  imu_state.ts_us = 0;

  char detail[32];
  snprintf(detail, sizeof(detail), "ready_0x%02X", candidate_address);
  emit_status("imu", detail);
}
} // namespace

void imu_setup_i2c(int sda_pin,
                   int scl_pin,
                   uint32_t i2c_clock_hz,
                   StatusEmitter emit_status)
{
  Wire.begin(sda_pin, scl_pin);
  Wire.setClock(i2c_clock_hz);
  emit_status("imu", "i2c_ready");
}

void imu_poll(Adafruit_BNO08x &imu_sensor,
              ImuState &imu_state,
              uint8_t primary_address,
              uint8_t alternate_address,
              uint32_t report_interval_us,
              unsigned long retry_ms,
              StatusEmitter emit_status)
{
  initialize_imu(imu_sensor,
                 imu_state,
                 primary_address,
                 alternate_address,
                 report_interval_us,
                 retry_ms,
                 emit_status);
  if (!imu_state.ready)
  {
    return;
  }

  sh2_SensorValue_t event;
  bool got_event = false;
  while (imu_sensor.getSensorEvent(&event))
  {
    got_event = true;
    switch (event.sensorId)
    {
      case SH2_ACCELEROMETER:
        imu_state.accel_mps2[0] = event.un.accelerometer.x;
        imu_state.accel_mps2[1] = event.un.accelerometer.y;
        imu_state.accel_mps2[2] = event.un.accelerometer.z;
        imu_state.have_accel = true;
        break;

      case SH2_GYROSCOPE_CALIBRATED:
        imu_state.gyro_rads[0] = event.un.gyroscope.x;
        imu_state.gyro_rads[1] = event.un.gyroscope.y;
        imu_state.gyro_rads[2] = event.un.gyroscope.z;
        imu_state.yaw_rate_dps = event.un.gyroscope.z * RAD_TO_DEG;
        imu_state.have_gyro = true;
        break;

      case SH2_ROTATION_VECTOR:
      {
        const float qw = event.un.rotationVector.real;
        const float qx = event.un.rotationVector.i;
        const float qy = event.un.rotationVector.j;
        const float qz = event.un.rotationVector.k;
        imu_state.yaw_deg = yaw_from_quaternion_wxyz(qw, qx, qy, qz);
        imu_state.have_yaw = true;
        break;
      }

      default:
        break;
    }
  }

  if (got_event)
  {
    imu_state.ts_us = micros();
    imu_state.last_sample_ms = millis();
  }
}

bool imu_is_healthy(const ImuState &imu_state,
                    unsigned long imu_fresh_ms,
                    unsigned long now_ms)
{
  if (!imu_state.ready || !imu_state.have_accel || !imu_state.have_gyro)
  {
    return false;
  }
  return (now_ms - imu_state.last_sample_ms) <= imu_fresh_ms;
}
