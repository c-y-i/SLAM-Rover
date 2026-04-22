#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <WiFi.h>
#include <ctype.h>
#include <esp_now.h>

#include "autonomy_controller.h"
#include "imu_support.h"
#include "lidar_reader.h"
#include "motor_driver.h"
#include "status_led.h"
#include "telemetry_link.h"

namespace
{

constexpr int kLedPin = 2;
constexpr uint32_t kUsbBaud = 115200;

constexpr int kLidarTxPin = 1;
constexpr int kLidarRxPin = 0;
constexpr uint32_t kLidarBaud = 230400;
constexpr unsigned long kLidarFreshMs = 700;
constexpr uint16_t kLidarIgnoreNearMm = 40;
constexpr int kFrontHalfWidthMm = 180;
constexpr int kSideMinForwardMm = -40;
constexpr int kSideMinLateralMm = 80;
constexpr unsigned long kLidarStatusIntervalMs = 3000;
constexpr unsigned long kTelemetryIntervalMs = 120;
constexpr unsigned long kImuTelemetryIntervalMs = 80;
constexpr unsigned long kImuFreshMs = 1200;

constexpr int kImuSdaPin = 22;
constexpr int kImuSclPin = 20;
constexpr uint32_t kImuI2cClockHz = 400000;
constexpr uint8_t kBno085PrimaryAddress = 0x4A;
constexpr uint8_t kBno085AlternateAddress = 0x4B;
constexpr uint32_t kImuReportIntervalUs = 10000;
constexpr unsigned long kImuRetryMs = 5000;

constexpr int kTeleopDriveSpeed = 170;
constexpr int kTeleopTurnFast = 200;
constexpr int kTeleopTurnSlow = 60;
constexpr int kTeleopSpinSpeed = 180;
constexpr unsigned long kTeleopTimeoutMs = 800;

struct LidarState
{
  bool have_scan = false;
  unsigned long last_scan_ms = 0;
  uint16_t front_min_mm = 0;
  uint16_t left_min_mm = 0;
  uint16_t right_min_mm = 0;
  uint16_t valid_points = 0;
  uint16_t scan_points = 0;
  uint32_t crc_fail_count = 0;
  uint32_t packets_seen = 0;
};

StatusLed status_led(kLedPin);
HardwareSerial lidar_serial(1);
lidar::Reader lidar_reader(lidar_serial);
Adafruit_BNO08x imu_sensor(55);
TelemetryLink telemetry_link;
AutonomyController autonomy_controller;

volatile char mode = 'x';
volatile char direction = 'x';
volatile unsigned long last_cmd_time = 0;

unsigned long last_lidar_status_ms = 0;

LidarState lidar_state;
ImuState imu_state;

MotorDriver &motor_driver = default_motor_driver();

int current_left_pwm = 0;
int current_right_pwm = 0;

void led_error()
{
  status_led.set_error();
}

void trigger_activity()
{
  status_led.mark_activity();
}

void setup_led()
{
  status_led.begin(40);
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

void emit_status(const char *stage, const char *detail)
{
  Serial.print(F("{\"t\":\"status\",\"stage\":\""));
  print_json_escaped(stage);
  Serial.print(F("\",\"detail\":\""));
  print_json_escaped(detail);
  Serial.println(F("\"}"));
}

uint16_t choose_min_distance(uint16_t current_mm, uint16_t candidate_mm)
{
  if (candidate_mm == 0)
  {
    return current_mm;
  }
  if (current_mm == 0 || candidate_mm < current_mm)
  {
    return candidate_mm;
  }
  return current_mm;
}

bool lidar_is_fresh()
{
  return lidar_state.have_scan &&
         (millis() - lidar_state.last_scan_ms) <= kLidarFreshMs;
}

LidarSnapshot current_lidar_snapshot()
{
  LidarSnapshot snapshot{};
  snapshot.have_scan = lidar_state.have_scan;
  snapshot.front_min_mm = lidar_state.front_min_mm;
  snapshot.left_min_mm = lidar_state.left_min_mm;
  snapshot.right_min_mm = lidar_state.right_min_mm;
  return snapshot;
}

void update_direction_from_pwm(int left_pwm, int right_pwm)
{
  if (left_pwm == 0 && right_pwm == 0)
  {
    direction = 'x';
    return;
  }
  if (left_pwm >= 0 && right_pwm >= 0)
  {
    if (left_pwm == right_pwm)
    {
      direction = 'w';
    }
    else if (left_pwm > right_pwm)
    {
      direction = 'd';
    }
    else
    {
      direction = 'a';
    }
    return;
  }
  if (left_pwm <= 0 && right_pwm <= 0)
  {
    direction = 's';
    return;
  }
  if (left_pwm < 0 && right_pwm > 0)
  {
    direction = 'q';
  }
  else
  {
    direction = 'e';
  }
}

void drive(int left_speed, int right_speed)
{
  current_left_pwm = constrain(left_speed, -255, 255);
  current_right_pwm = constrain(right_speed, -255, 255);
  motor_driver.set_wheel_pwm(current_left_pwm, current_right_pwm);
  update_direction_from_pwm(current_left_pwm, current_right_pwm);
}

void stop_drive()
{
  drive(0, 0);
}

void maybe_send_imu_telemetry()
{
  if (!telemetry_link.has_controller_peer())
  {
    return;
  }

  const unsigned long now_ms = millis();
  if (now_ms - imu_state.last_telemetry_ms < kImuTelemetryIntervalMs)
  {
    return;
  }
  imu_state.last_telemetry_ms = now_ms;

  ImuTelemetrySample sample{};
  sample.ts_us = imu_state.ts_us;
  sample.accel_mps2[0] = imu_state.accel_mps2[0];
  sample.accel_mps2[1] = imu_state.accel_mps2[1];
  sample.accel_mps2[2] = imu_state.accel_mps2[2];
  sample.gyro_rads[0] = imu_state.gyro_rads[0];
  sample.gyro_rads[1] = imu_state.gyro_rads[1];
  sample.gyro_rads[2] = imu_state.gyro_rads[2];
  sample.yaw_deg = imu_state.yaw_deg;
  sample.yaw_rate_dps = imu_state.yaw_rate_dps;
  sample.imu_ok = imu_is_healthy(imu_state, kImuFreshMs, now_ms);

  telemetry_link.send_imu(sample);
}

void refresh_lidar_state(const lidar::ScanFrame &scan)
{
  lidar_state.have_scan = scan.valid_point_count > 0;
  lidar_state.last_scan_ms = millis();
  lidar_state.front_min_mm = 0;
  lidar_state.left_min_mm = 0;
  lidar_state.right_min_mm = 0;
  lidar_state.valid_points = scan.valid_point_count;
  lidar_state.scan_points = scan.point_count;
  lidar_state.crc_fail_count = scan.crc_fail_count;
  lidar_state.packets_seen = lidar_reader.packets_seen();

  for (uint16_t index = 0; index < scan.point_count; ++index)
  {
    const lidar::ScanPoint &point = scan.points[index];
    if (!point.valid || point.distance_mm < kLidarIgnoreNearMm)
    {
      continue;
    }

    if (point.x_mm > 0 && abs(point.y_mm) <= kFrontHalfWidthMm)
    {
      lidar_state.front_min_mm = choose_min_distance(lidar_state.front_min_mm, point.distance_mm);
    }

    if (point.x_mm >= kSideMinForwardMm && point.y_mm >= kSideMinLateralMm)
    {
      lidar_state.left_min_mm = choose_min_distance(lidar_state.left_min_mm, point.distance_mm);
    }
    else if (point.x_mm >= kSideMinForwardMm && point.y_mm <= -kSideMinLateralMm)
    {
      lidar_state.right_min_mm =
          choose_min_distance(lidar_state.right_min_mm, point.distance_mm);
    }
  }
}

void print_lidar_status()
{
  Serial.printf("Lidar packets=%lu scan_points=%u valid=%u front=%u left=%u right=%u crc_fail=%lu\n",
                static_cast<unsigned long>(lidar_state.packets_seen),
                static_cast<unsigned>(lidar_state.scan_points),
                static_cast<unsigned>(lidar_state.valid_points),
                static_cast<unsigned>(lidar_state.front_min_mm),
                static_cast<unsigned>(lidar_state.left_min_mm),
                static_cast<unsigned>(lidar_state.right_min_mm),
                static_cast<unsigned long>(lidar_state.crc_fail_count));
}

void maybe_report_lidar()
{
  if (!lidar_is_fresh())
  {
    return;
  }

  const unsigned long now_ms = millis();
  if (now_ms - last_lidar_status_ms >= kLidarStatusIntervalMs)
  {
    last_lidar_status_ms = now_ms;
    print_lidar_status();
  }
}

void set_mode(char new_mode)
{
  mode = new_mode;
  direction = 'x';
  last_cmd_time = millis();
  stop_drive();
  autonomy_controller.on_mode_changed(mode);

  char detail[32];
  snprintf(detail, sizeof(detail), "mode_%c", mode);
  emit_status("mode", detail);
}

void update_lidar()
{
  if (!lidar_reader.read_scan())
  {
    return;
  }

  const lidar::ScanFrame &scan = lidar_reader.latest_scan();
  refresh_lidar_state(scan);
  const unsigned long now_ms = millis();
  autonomy_controller.on_lidar_update(current_lidar_snapshot(),
                                      lidar_is_fresh(),
                                      now_ms,
                                      emit_status);

  static unsigned long last_telemetry_ms = 0;
  if (now_ms - last_telemetry_ms >= kTelemetryIntervalMs)
  {
    last_telemetry_ms = now_ms;
    telemetry_link.send_scan(scan);
  }
}

void apply_motor_cmd(char motor, char dir, uint8_t pwm)
{
  if (autonomy_controller.is_safety_active() && !(dir == 's'))
  {
    return;
  }

  const int speed = (dir == 'f') ? static_cast<int>(pwm)
                                 : (dir == 'b') ? -static_cast<int>(pwm) : 0;

  if (motor == 'L')
  {
    current_left_pwm = speed;
  }
  else
  {
    current_right_pwm = speed;
  }

  drive(current_left_pwm, current_right_pwm);
  trigger_activity();
  last_cmd_time = millis();
}

void control_motor(char cmd)
{
  if (autonomy_controller.is_safety_active())
  {
    return;
  }

  switch (cmd)
  {
    case 'w':
      drive(kTeleopDriveSpeed, kTeleopDriveSpeed);
      break;
    case 's':
      drive(-kTeleopDriveSpeed, -kTeleopDriveSpeed);
      break;
    case 'a':
      drive(kTeleopTurnSlow, kTeleopTurnFast);
      break;
    case 'd':
      drive(kTeleopTurnFast, kTeleopTurnSlow);
      break;
    case 'q':
      drive(-kTeleopSpinSpeed, kTeleopSpinSpeed);
      break;
    case 'e':
      drive(kTeleopSpinSpeed, -kTeleopSpinSpeed);
      break;
    case 'x':
      stop_drive();
      break;
    default:
      break;
  }
}

void on_data_recv(const esp_now_recv_info *info, const uint8_t *data, int len)
{
  if (len < 1)
  {
    return;
  }

  telemetry_link.update_peer_from_source(info);

  if (len == 3 && (data[0] == 'L' || data[0] == 'R'))
  {
    apply_motor_cmd(static_cast<char>(data[0]), static_cast<char>(data[1]), data[2]);
    return;
  }

  if (!isascii(data[0]))
  {
    return;
  }

  const char cmd = static_cast<char>(data[0]);
  if (cmd == 'v')
  {
    return;
  }

  trigger_activity();
  if (cmd == '1' || cmd == '2' || cmd == 'x')
  {
    set_mode(cmd);
    return;
  }

  if (mode == '2')
  {
    direction = cmd;
    last_cmd_time = millis();
    control_motor(cmd);
  }
}

void setup_espnow()
{
  WiFi.mode(WIFI_STA);
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW init failed, restarting");
    led_error();
    delay(2000);
    ESP.restart();
  }

  esp_now_register_recv_cb(on_data_recv);
}

void setup_motor()
{
  motor_driver.begin();
  emit_status("motor", default_motor_driver_name());
}

void setup_lidar()
{
  lidar_reader.begin(kLidarRxPin, kLidarTxPin, kLidarBaud);
  char detail[80];
  snprintf(detail, sizeof(detail), "ready_rx%d_tx%d_baud%lu",
           kLidarRxPin,
           kLidarTxPin,
           static_cast<unsigned long>(kLidarBaud));
  emit_status("lidar", detail);
}

void setup_imu()
{
  imu_setup_i2c(kImuSdaPin, kImuSclPin, kImuI2cClockHz, emit_status);
}

void handle_usb_serial()
{
  static String serial_buf;

  while (Serial.available())
  {
    const char key = static_cast<char>(Serial.read());
    if (key == '\r')
    {
      continue;
    }

    if (key == '\n')
    {
      if (serial_buf == "lp")
      {
        lidar_reader.print_packet_summary(Serial);
      }
      else if (serial_buf == "ls")
      {
        print_lidar_status();
      }
      else if (serial_buf.length() >= 2)
      {
        const char motor = serial_buf[0];
        const char dir = serial_buf[1];
        const uint8_t pwm = serial_buf.length() > 2
                                ? static_cast<uint8_t>(
                                      constrain(serial_buf.substring(2).toInt(), 0, 255))
                                : 0;
        apply_motor_cmd(motor, dir, pwm);
      }
      serial_buf = "";
      continue;
    }

    if (serial_buf.length() > 0)
    {
      serial_buf += key;
      continue;
    }

    if (key == '1' || key == '2' || key == 'x')
    {
      set_mode(key);
    }
    else if (key == 'L' || key == 'R' || key == 'l')
    {
      serial_buf = key;
    }
    else if (mode == '2')
    {
      direction = key;
      last_cmd_time = millis();
      control_motor(key);
    }
  }
}

} // namespace

void setup()
{
  Serial.begin(kUsbBaud);
  delay(200);
  setup_led();
  randomSeed(esp_random());
  setup_motor();
  setup_lidar();
  setup_imu();
  setup_espnow();
  set_mode('x');

  emit_status("boot", "ready");
}

void loop()
{
  handle_usb_serial();
  imu_poll(imu_sensor,
           imu_state,
           kBno085PrimaryAddress,
           kBno085AlternateAddress,
           kImuReportIntervalUs,
           kImuRetryMs,
           emit_status);
  update_lidar();
  maybe_send_imu_telemetry();

  const unsigned long now_ms = millis();
  const bool autonomy_owns_drive =
      autonomy_controller.update(mode,
                                 current_lidar_snapshot(),
                                 lidar_is_fresh(),
                                 now_ms,
                                 drive,
                                 emit_status);
  if (!autonomy_owns_drive)
  {
    switch (mode)
    {
      case '2':
        if (direction != 'x' && now_ms - last_cmd_time > kTeleopTimeoutMs)
        {
          direction = 'x';
          stop_drive();
          emit_status("teleop", "timeout_stopped");
        }
        break;

      case 'x':
      default:
        stop_drive();
        break;
    }
  }

  maybe_report_lidar();
  status_led.update(millis());
  delay(10);
}
