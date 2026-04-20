#include "sensor_streamer.h"

#include <stdio.h>

#include "config.h"

namespace
{

const char *tof_error_name(SF_VL53L5CX_ERROR_TYPE code)
{
  switch (code)
  {
  case SF_VL53L5CX_ERROR_TYPE::NO_ERROR:
    return "no_error";
  case SF_VL53L5CX_ERROR_TYPE::I2C_INITIALIZATION_ERROR:
    return "i2c_initialization_error";
  case SF_VL53L5CX_ERROR_TYPE::I2C_NOT_RESPONDING:
    return "i2c_not_responding";
  case SF_VL53L5CX_ERROR_TYPE::DEVICE_INITIALIZATION_ERROR:
    return "device_initialization_error";
  case SF_VL53L5CX_ERROR_TYPE::INVALID_DEVICE:
    return "invalid_device";
  case SF_VL53L5CX_ERROR_TYPE::INVALID_FREQUENCY_SETTING:
    return "invalid_frequency_setting";
  case SF_VL53L5CX_ERROR_TYPE::INVALID_RANGING_MODE:
    return "invalid_ranging_mode";
  case SF_VL53L5CX_ERROR_TYPE::CANNOT_CHANGE_I2C_ADDRESS:
    return "cannot_change_i2c_address";
  case SF_VL53L5CX_ERROR_TYPE::CANNOT_START_RANGING:
    return "cannot_start_ranging";
  case SF_VL53L5CX_ERROR_TYPE::CANNOT_STOP_RANGING:
    return "cannot_stop_ranging";
  case SF_VL53L5CX_ERROR_TYPE::CANNOT_GET_DATA_READY:
    return "cannot_get_data_ready";
  case SF_VL53L5CX_ERROR_TYPE::CANNOT_GET_RESOLUTION:
    return "cannot_get_resolution";
  case SF_VL53L5CX_ERROR_TYPE::CANNOT_GET_RANGING_DATA:
    return "cannot_get_ranging_data";
  case SF_VL53L5CX_ERROR_TYPE::CANNOT_SET_RESOLUTION:
    return "cannot_set_resolution";
  case SF_VL53L5CX_ERROR_TYPE::CANNOT_SET_POWER_MODE:
    return "cannot_set_power_mode";
  case SF_VL53L5CX_ERROR_TYPE::CANNOT_GET_POWER_MODE:
    return "cannot_get_power_mode";
  case SF_VL53L5CX_ERROR_TYPE::DEVICE_NOT_ALIVE:
    return "device_not_alive";
  case SF_VL53L5CX_ERROR_TYPE::INVALID_INTEGRATION_TIME:
    return "invalid_integration_time";
  case SF_VL53L5CX_ERROR_TYPE::CANNOT_SET_INTEGRATION_TIME:
    return "cannot_set_integration_time";
  case SF_VL53L5CX_ERROR_TYPE::CANNOT_GET_INTEGRATION_TIME:
    return "cannot_get_integration_time";
  case SF_VL53L5CX_ERROR_TYPE::INVALID_SHARPENER_VALUE:
    return "invalid_sharpener_value";
  case SF_VL53L5CX_ERROR_TYPE::CANNOT_SET_SHARPENER_VALUE:
    return "cannot_set_sharpener_value";
  case SF_VL53L5CX_ERROR_TYPE::CANNOT_GET_SHARPENER_VALUE:
    return "cannot_get_sharpener_value";
  case SF_VL53L5CX_ERROR_TYPE::CANNOT_SET_TARGET_ORDER:
    return "cannot_set_target_order";
  case SF_VL53L5CX_ERROR_TYPE::CANNOT_GET_TARGET_ORDER:
    return "cannot_get_target_order";
  case SF_VL53L5CX_ERROR_TYPE::INVALID_TARGET_ORDER:
    return "invalid_target_order";
  case SF_VL53L5CX_ERROR_TYPE::UNKNOWN_ERROR:
    return "unknown_error";
  default:
    return "unmapped_error";
  }
}

void print_json_escaped(const char *text)
{
  for (const char *cursor = text; *cursor != '\0'; ++cursor)
  {
    const char ch = *cursor;
    if (ch == '"' || ch == '\\')
    {
      Serial.write('\\');
    }

    if (ch == '\n' || ch == '\r')
    {
      Serial.write(' ');
    }
    else
    {
      Serial.write(ch);
    }
  }
}

} // namespace

namespace sensor
{

bool SensorStreamer::probe_address(uint8_t address)
{
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void SensorStreamer::emit_status_packet(const char *stage, const char *detail)
{
  Serial.print(F("{\"t\":\"status\",\"stage\":\""));
  print_json_escaped(stage);
  Serial.print(F("\",\"detail\":\""));
  print_json_escaped(detail);
  Serial.print(F("\",\"v\":\""));
  Serial.print(config::kProtocolVersion);
  Serial.println(F("\"}"));
}

void SensorStreamer::emit_i2c_scan_status()
{
  char detail[96];
  size_t used = 0;
  bool found_any = false;

  used += static_cast<size_t>(snprintf(detail + used,
                                       sizeof(detail) - used,
                                       "found"));

  for (uint8_t address = 1; address < 0x7F && used < (sizeof(detail) - 6); ++address)
  {
    if (!probe_address(address))
    {
      continue;
    }

    found_any = true;
    used += static_cast<size_t>(snprintf(detail + used,
                                         sizeof(detail) - used,
                                         "_0x%02X",
                                         address));
  }

  if (!found_any)
  {
    snprintf(detail, sizeof(detail), "none");
  }

  emit_status_packet("i2c_scan", detail);
}

void SensorStreamer::initialize_wire()
{
  if (wire_ready_)
  {
    return;
  }

  Wire.begin(config::kTofSdaPin, config::kTofSclPin);
  Wire.setClock(config::kTofI2cClockHz);
  wire_ready_ = true;
  last_recovery_attempt_ms_ = millis();

  emit_status_packet("boot", "i2c_ready");
  emit_i2c_scan_status();
}

void SensorStreamer::emit_tof_error_packet(const char *action)
{
  char detail[128];
  snprintf(detail,
           sizeof(detail),
           "%s_%s_0x%08lX",
           action,
           tof_error_name(tof_sensor_.lastError.lastErrorCode),
           static_cast<unsigned long>(tof_sensor_.lastError.lastErrorValue));
  emit_status_packet("tof", detail);
}

void SensorStreamer::initialize_tof()
{
  if (tof_ready_)
  {
    return;
  }

  if (!probe_address(config::kTofAddress))
  {
    emit_status_packet("tof", "probe_failed_0x29");
    return;
  }

  if (!tof_sensor_.begin(config::kTofAddress, Wire))
  {
    emit_tof_error_packet("begin_failed");
    return;
  }

  tof_sensor_.setWireMaxPacketSize(kWirePacketSize);

  if (!tof_sensor_.isConnected())
  {
    emit_tof_error_packet("alive_check_failed");
    return;
  }

  if (!tof_sensor_.setResolution(config::kTofResolution))
  {
    emit_tof_error_packet("set_resolution_failed");
    return;
  }

  if (!tof_sensor_.setRangingFrequency(config::kTofRangingFrequencyHz))
  {
    emit_tof_error_packet("set_frequency_failed");
    return;
  }

  if (!tof_sensor_.startRanging())
  {
    emit_tof_error_packet("start_ranging_failed");
    return;
  }

  tof_ready_ = true;
  emit_status_packet("tof", "ready");
}

void SensorStreamer::initialize_imu()
{
  if (imu_ready_)
  {
    return;
  }

  uint8_t candidate_address = 0;
#if IMU_TYPE == IMU_BNO085
  if (probe_address(config::kBno085PrimaryAddress))
  {
    candidate_address = config::kBno085PrimaryAddress;
  }
  else if (probe_address(config::kBno085AlternateAddress))
  {
    candidate_address = config::kBno085AlternateAddress;
  }
#else
  if (probe_address(config::kImuPrimaryAddress))
  {
    candidate_address = config::kImuPrimaryAddress;
  }
  else if (probe_address(config::kImuAlternateAddress))
  {
    candidate_address = config::kImuAlternateAddress;
  }
#endif

  if (candidate_address == 0)
  {
    emit_status_packet("imu", "probe_failed");
    return;
  }

#if IMU_TYPE == IMU_BNO085
  if (!imu_sensor_.begin_I2C(candidate_address, &Wire))
  {
    emit_status_packet("imu", "begin_failed");
    return;
  }
  imu_sensor_.enableReport(SH2_ACCELEROMETER, config::kImuIntervalUs);
  imu_sensor_.enableReport(SH2_GYROSCOPE_CALIBRATED, config::kImuIntervalUs);
#else
  if (!imu_sensor_.begin(candidate_address, &Wire))
  {
    emit_status_packet("imu", "begin_failed");
    return;
  }
  imu_sensor_.setAccelerometerRange(MPU6050_RANGE_8_G);
  imu_sensor_.setGyroRange(MPU6050_RANGE_500_DEG);
  imu_sensor_.setFilterBandwidth(MPU6050_BAND_44_HZ);
#endif

  imu_address_ = candidate_address;
  imu_ready_ = true;
  last_imu_emit_us_ = 0;

  char detail[24];
  snprintf(detail, sizeof(detail), "ready_0x%02X", imu_address_);
  emit_status_packet("imu", detail);
}

void SensorStreamer::try_initialize_missing_devices()
{
  if (tof_ready_ && imu_ready_)
  {
    return;
  }

  const uint32_t now_ms = millis();
  if (now_ms - last_recovery_attempt_ms_ < config::kRecoveryRetryMs)
  {
    return;
  }

  last_recovery_attempt_ms_ = now_ms;
  emit_i2c_scan_status();

  if (!tof_ready_)
  {
    initialize_tof();
  }

  if (!imu_ready_)
  {
    initialize_imu();
  }
}

void SensorStreamer::emit_tof_packet()
{
  Serial.print(F("{\"t\":\"tof\",\"seq\":"));
  Serial.print(tof_frame_.seq);
  Serial.print(F(",\"ts_us\":"));
  Serial.print(tof_frame_.ts_us);
  Serial.print(F(",\"distances\":["));

  for (size_t i = 0; i < kZoneCount; ++i)
  {
    if (i > 0)
    {
      Serial.write(',');
    }
    Serial.print(tof_frame_.distances_mm[i]);
  }

  Serial.print(F("],\"status\":["));
  for (size_t i = 0; i < kZoneCount; ++i)
  {
    if (i > 0)
    {
      Serial.write(',');
    }
    Serial.print(tof_frame_.status[i]);
  }

  Serial.print(F("],\"tof_ok\":"));
  Serial.print(tof_ready_ ? F("true") : F("false"));
  Serial.print(F(",\"v\":\""));
  Serial.print(config::kProtocolVersion);
  Serial.println(F("\"}"));
}

void SensorStreamer::emit_imu_packet()
{
  Serial.print(F("{\"t\":\"imu\",\"ts_us\":"));
  Serial.print(imu_sample_.ts_us);
  Serial.print(F(",\"accel_mps2\":["));
  Serial.print(imu_sample_.accel_mps2[0], 6);
  Serial.write(',');
  Serial.print(imu_sample_.accel_mps2[1], 6);
  Serial.write(',');
  Serial.print(imu_sample_.accel_mps2[2], 6);
  Serial.print(F("],\"gyro_rads\":["));
  Serial.print(imu_sample_.gyro_rads[0], 6);
  Serial.write(',');
  Serial.print(imu_sample_.gyro_rads[1], 6);
  Serial.write(',');
  Serial.print(imu_sample_.gyro_rads[2], 6);
  Serial.print(F("],\"temp_c\":"));
  Serial.print(imu_sample_.temp_c, 4);
  Serial.print(F(",\"imu_ok\":"));
  Serial.print(imu_ready_ ? F("true") : F("false"));
  Serial.print(F(",\"v\":\""));
  Serial.print(config::kProtocolVersion);
  Serial.println(F("\"}"));
}

void SensorStreamer::poll_tof()
{
  if (!tof_sensor_.isDataReady())
  {
    if (tof_sensor_.lastError.lastErrorCode != SF_VL53L5CX_ERROR_TYPE::NO_ERROR)
    {
      tof_ready_ = false;
      emit_tof_error_packet("data_ready_failed");
    }
    return;
  }

  if (!tof_sensor_.getRangingData(&tof_results_))
  {
    tof_ready_ = false;
    emit_tof_error_packet("read_failed");
    return;
  }

  tof_frame_.seq += 1;
  tof_frame_.ts_us = micros();
  tof_frame_.valid_count = 0;
  tof_frame_.valid = true;

  for (size_t index = 0; index < kZoneCount; ++index)
  {
    const size_t result_index = index * VL53L5CX_NB_TARGET_PER_ZONE;
    const int16_t raw_distance_mm = tof_results_.distance_mm[result_index];
    const uint8_t target_status = tof_results_.target_status[result_index];
    const uint16_t distance_mm =
        raw_distance_mm > 0 ? static_cast<uint16_t>(raw_distance_mm) : 0U;

    tof_frame_.distances_mm[index] = distance_mm;
    tof_frame_.status[index] = target_status;

    if (distance_mm > 0 && is_tof_status_valid(target_status))
    {
      ++tof_frame_.valid_count;
    }
  }

  emit_tof_packet();
}

void SensorStreamer::poll_imu(uint32_t now_us)
{
#if IMU_TYPE == IMU_BNO085
  sh2_SensorValue_t val;
  bool got_accel = false;
  bool got_gyro = false;

  while (imu_sensor_.getSensorEvent(&val))
  {
    switch (val.sensorId)
    {
    case SH2_ACCELEROMETER:
      imu_sample_.accel_mps2[0] = val.un.accelerometer.x;
      imu_sample_.accel_mps2[1] = val.un.accelerometer.y;
      imu_sample_.accel_mps2[2] = val.un.accelerometer.z;
      got_accel = true;
      break;
    case SH2_GYROSCOPE_CALIBRATED:
      imu_sample_.gyro_rads[0] = val.un.gyroscope.x;
      imu_sample_.gyro_rads[1] = val.un.gyroscope.y;
      imu_sample_.gyro_rads[2] = val.un.gyroscope.z;
      got_gyro = true;
      break;
    }
  }

  if (!got_accel || !got_gyro)
  {
    return;
  }

  imu_sample_.ts_us = now_us;
  imu_sample_.temp_c = 0.0f;
  imu_sample_.valid = true;
  last_imu_emit_us_ = now_us;
  emit_imu_packet();
#else
  if (now_us - last_imu_emit_us_ < config::kImuIntervalUs)
  {
    return;
  }

  sensors_event_t accel_event;
  sensors_event_t gyro_event;
  sensors_event_t temp_event;
  imu_sensor_.getEvent(&accel_event, &gyro_event, &temp_event);

  imu_sample_.ts_us = now_us;
  imu_sample_.accel_mps2[0] = accel_event.acceleration.x;
  imu_sample_.accel_mps2[1] = accel_event.acceleration.y;
  imu_sample_.accel_mps2[2] = accel_event.acceleration.z;
  imu_sample_.gyro_rads[0] = gyro_event.gyro.x;
  imu_sample_.gyro_rads[1] = gyro_event.gyro.y;
  imu_sample_.gyro_rads[2] = gyro_event.gyro.z;
  imu_sample_.temp_c = temp_event.temperature;
  imu_sample_.valid = true;
  last_imu_emit_us_ = now_us;
  emit_imu_packet();
#endif
}

void SensorStreamer::begin()
{
  emit_status_packet("boot", "starting");
  initialize_wire();
  initialize_tof();
  initialize_imu();
  emit_status_packet("boot", (tof_ready_ || imu_ready_) ? "active" : "waiting_for_devices");
}

void SensorStreamer::poll()
{
  if (!wire_ready_)
  {
    initialize_wire();
  }

  try_initialize_missing_devices();

  if (imu_ready_)
  {
    poll_imu(micros());
  }

  if (tof_ready_)
  {
    poll_tof();
  }
}

} // namespace sensor
