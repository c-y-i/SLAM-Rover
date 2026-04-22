#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <esp_now.h>
#include <WiFi.h>
#include <string.h>

namespace
{

constexpr int kLedPin = 2;
constexpr uint32_t kUsbBaud = 460800;
constexpr unsigned long kKeepaliveMs = 500;
constexpr unsigned long kViewerHandshakeMs = 1000;

constexpr uint8_t kBotMac[6] = {0x34, 0xB7, 0xDA, 0xF2, 0x36, 0xC4};
constexpr uint8_t kChannel = 1;

constexpr uint8_t kTelemetryMagic = 0xA5;
constexpr uint8_t kTelemetryVersion = 1;
constexpr uint8_t kTelemetryTypeScanChunk = 1;
constexpr uint8_t kTelemetryTypeImu = 2;
constexpr uint8_t kTelemetryPointsPerChunk = 48;
constexpr uint8_t kTelemetryMaxPoints = 96;
constexpr uint8_t kTelemetryMaxChunks =
    (kTelemetryMaxPoints + kTelemetryPointsPerChunk - 1) / kTelemetryPointsPerChunk;

struct __attribute__((packed)) TelemetryHeader
{
  uint8_t magic;
  uint8_t version;
  uint8_t type;
  uint16_t frame_id;
  uint8_t chunk_index;
  uint8_t chunk_count;
  uint8_t point_count;
  uint8_t total_points;
};

struct __attribute__((packed)) TelemetryPoint
{
  int16_t x_mm;
  int16_t y_mm;
  uint8_t intensity;
};

struct __attribute__((packed)) TelemetryImuPayload
{
  uint32_t ts_us;
  float accel_mps2[3];
  float gyro_rads[3];
  float yaw_deg;
  float yaw_rate_dps;
  uint8_t imu_ok;
};

struct TelemetryAssembly
{
  uint16_t frame_id = 0;
  uint8_t chunk_count = 0;
  uint8_t total_points = 0;
  bool chunk_received[kTelemetryMaxChunks]{};
  TelemetryPoint points[kTelemetryMaxPoints]{};
};

struct ReadyScan
{
  volatile bool pending = false;
  uint16_t point_count = 0;
  TelemetryPoint points[kTelemetryMaxPoints]{};
};

struct ReadyImu
{
  volatile bool pending = false;
  TelemetryImuPayload payload{};
};

Adafruit_NeoPixel status_led(1, kLedPin, NEO_GRB + NEO_KHZ800);
esp_now_peer_info_t bot_peer = {};

volatile bool activity_flag = false;
uint32_t led_base_color = 0;
unsigned long led_flash_end = 0;

char current_mode = 'x';
char held_cmd = 'x';
char last_sent_cmd = 0;
unsigned long last_send_time = 0;
unsigned long last_handshake_time = 0;
String cmd_buf;

TelemetryAssembly telemetry_assembly;
ReadyScan ready_scan;
ReadyImu ready_imu;

void set_led_color(uint8_t r, uint8_t g, uint8_t b)
{
  status_led.setPixelColor(0, status_led.Color(r, g, b));
  status_led.show();
}

void led_error()
{
  led_base_color = status_led.Color(200, 0, 0);
  set_led_color(200, 0, 0);
}

void led_waiting()
{
  led_base_color = status_led.Color(200, 100, 0);
  set_led_color(200, 100, 0);
}

void trigger_activity()
{
  activity_flag = true;
}

void update_led()
{
  if (activity_flag)
  {
    activity_flag = false;
    if (led_base_color != status_led.Color(200, 0, 0))
    {
      led_base_color = status_led.Color(0, 200, 0);
    }
    led_flash_end = millis() + 300;
    set_led_color(0, 0, 200);
  }
  else if (millis() > led_flash_end)
  {
    const uint8_t r = (led_base_color >> 16) & 0xFF;
    const uint8_t g = (led_base_color >> 8) & 0xFF;
    const uint8_t b = led_base_color & 0xFF;
    static uint32_t last_shown = 0xFFFFFFFF;
    if (led_base_color != last_shown)
    {
      set_led_color(r, g, b);
      last_shown = led_base_color;
    }
  }
}

void setup_led()
{
  status_led.begin();
  status_led.setBrightness(40);
  led_waiting();
}

void reset_telemetry_assembly(uint16_t frame_id, uint8_t chunk_count, uint8_t total_points)
{
  telemetry_assembly.frame_id = frame_id;
  telemetry_assembly.chunk_count = min(chunk_count, kTelemetryMaxChunks);
  telemetry_assembly.total_points = min(total_points, kTelemetryMaxPoints);
  memset(telemetry_assembly.chunk_received, 0, sizeof(telemetry_assembly.chunk_received));
}

bool assembly_complete()
{
  if (telemetry_assembly.total_points == 0 ||
      telemetry_assembly.chunk_count == 0)
  {
    return false;
  }

  for (uint8_t i = 0; i < telemetry_assembly.chunk_count; ++i)
  {
    if (!telemetry_assembly.chunk_received[i])
    {
      return false;
    }
  }

  return true;
}

void stage_ready_scan()
{
  if (telemetry_assembly.total_points == 0)
  {
    return;
  }

  ready_scan.point_count = telemetry_assembly.total_points;
  memcpy(ready_scan.points,
         telemetry_assembly.points,
         static_cast<size_t>(ready_scan.point_count) * sizeof(TelemetryPoint));
  ready_scan.pending = true;
}

void handle_scan_chunk(const TelemetryHeader &header, const uint8_t *data, int len)
{
  if (header.chunk_count == 0 ||
      header.chunk_count > kTelemetryMaxChunks ||
      header.total_points == 0 ||
      header.total_points > kTelemetryMaxPoints ||
      header.chunk_index >= header.chunk_count ||
      header.point_count > kTelemetryPointsPerChunk)
  {
    return;
  }

  const int expected_len =
      static_cast<int>(sizeof(TelemetryHeader) +
                       header.point_count * sizeof(TelemetryPoint));
  if (len != expected_len)
  {
    return;
  }

  if (telemetry_assembly.frame_id != header.frame_id ||
      telemetry_assembly.chunk_count != header.chunk_count ||
      telemetry_assembly.total_points != header.total_points)
  {
    reset_telemetry_assembly(header.frame_id, header.chunk_count, header.total_points);
  }

  const uint8_t point_offset = header.chunk_index * kTelemetryPointsPerChunk;
  if (point_offset + header.point_count > telemetry_assembly.total_points)
  {
    return;
  }

  memcpy(&telemetry_assembly.points[point_offset],
         data + sizeof(TelemetryHeader),
         header.point_count * sizeof(TelemetryPoint));
  telemetry_assembly.chunk_received[header.chunk_index] = true;

  if (assembly_complete())
  {
    stage_ready_scan();
  }
}

void handle_imu_packet(const uint8_t *data, int len)
{
  const int expected_len =
      static_cast<int>(sizeof(TelemetryHeader) + sizeof(TelemetryImuPayload));
  if (len != expected_len)
  {
    return;
  }

  memcpy(&ready_imu.payload,
         data + sizeof(TelemetryHeader),
         sizeof(TelemetryImuPayload));
  ready_imu.pending = true;
}

void handle_telemetry_packet(const uint8_t *data, int len)
{
  if (len < static_cast<int>(sizeof(TelemetryHeader)))
  {
    return;
  }

  TelemetryHeader header;
  memcpy(&header, data, sizeof(header));
  if (header.magic != kTelemetryMagic || header.version != kTelemetryVersion)
  {
    return;
  }

  if (header.type == kTelemetryTypeScanChunk)
  {
    handle_scan_chunk(header, data, len);
    return;
  }

  if (header.type == kTelemetryTypeImu)
  {
    handle_imu_packet(data, len);
  }
}

void on_data_sent(const wifi_tx_info_t *info, esp_now_send_status_t status)
{
  (void)info;
  if (status == ESP_NOW_SEND_SUCCESS)
  {
    trigger_activity();
  }
}

void on_data_recv(const esp_now_recv_info *info, const uint8_t *data, int len)
{
  (void)info;
  handle_telemetry_packet(data, len);
}

void send_raw(const uint8_t *data, size_t len, bool log_text, bool track_command)
{
  if (esp_now_send(bot_peer.peer_addr, data, len) == ESP_OK)
  {
    if (log_text)
    {
      Serial.printf("-> %c\n", static_cast<char>(data[0]));
    }
  }
  else if (log_text)
  {
    Serial.printf("-> %c FAILED\n", static_cast<char>(data[0]));
  }

  last_send_time = millis();
  if (track_command && len == 1)
  {
    last_sent_cmd = static_cast<char>(data[0]);
  }
}

void send_cmd(char cmd)
{
  const uint8_t buf[1] = {static_cast<uint8_t>(cmd)};
  send_raw(buf, sizeof(buf), true, true);
}

void send_viewer_handshake()
{
  const uint8_t buf[1] = {'v'};
  send_raw(buf, sizeof(buf), false, false);
}

void send_motor_cmd(const String &s)
{
  if (s.length() < 2)
  {
    return;
  }

  const char motor = s[0];
  const char dir = s[1];
  if ((motor != 'L' && motor != 'R') ||
      (dir != 'f' && dir != 'b' && dir != 's'))
  {
    return;
  }

  int pwm = (s.length() > 2) ? constrain(s.substring(2).toInt(), 0, 255) : 0;
  if (dir == 's')
  {
    pwm = 0;
  }

  const uint8_t buf[3] = {
      static_cast<uint8_t>(motor),
      static_cast<uint8_t>(dir),
      static_cast<uint8_t>(pwm),
  };

  if (esp_now_send(bot_peer.peer_addr, buf, sizeof(buf)) == ESP_OK)
  {
    Serial.printf("-> Motor %c %c PWM %d\n", motor, dir, pwm);
  }
  else
  {
    Serial.println("-> Motor cmd FAILED");
  }
}

void print_json_scan(const ReadyScan &scan)
{
  Serial.print(F("{\"t\":\"scan\",\"x\":["));
  for (uint16_t i = 0; i < scan.point_count; ++i)
  {
    if (i > 0)
    {
      Serial.print(',');
    }
    Serial.print(scan.points[i].x_mm);
  }

  Serial.print(F("],\"y\":["));
  for (uint16_t i = 0; i < scan.point_count; ++i)
  {
    if (i > 0)
    {
      Serial.print(',');
    }
    Serial.print(scan.points[i].y_mm);
  }

  Serial.print(F("],\"i\":["));
  for (uint16_t i = 0; i < scan.point_count; ++i)
  {
    if (i > 0)
    {
      Serial.print(',');
    }
    Serial.print(scan.points[i].intensity);
  }

  Serial.println(F("]}"));
}

void print_json_imu(const TelemetryImuPayload &imu)
{
  Serial.print(F("{\"t\":\"imu\",\"ts_us\":"));
  Serial.print(imu.ts_us);

  Serial.print(F(",\"accel_mps2\":["));
  Serial.print(imu.accel_mps2[0], 6);
  Serial.print(',');
  Serial.print(imu.accel_mps2[1], 6);
  Serial.print(',');
  Serial.print(imu.accel_mps2[2], 6);
  Serial.print(']');

  Serial.print(F(",\"gyro_rads\":["));
  Serial.print(imu.gyro_rads[0], 6);
  Serial.print(',');
  Serial.print(imu.gyro_rads[1], 6);
  Serial.print(',');
  Serial.print(imu.gyro_rads[2], 6);
  Serial.print(']');

  Serial.print(F(",\"yaw_deg\":"));
  Serial.print(imu.yaw_deg, 4);
  Serial.print(F(",\"yaw_rate_dps\":"));
  Serial.print(imu.yaw_rate_dps, 4);
  Serial.print(F(",\"imu_ok\":"));
  Serial.print(imu.imu_ok ? F("true") : F("false"));
  Serial.println('}');
}

void flush_ready_scan()
{
  if (!ready_scan.pending)
  {
    return;
  }

  ReadyScan local_scan;
  local_scan.pending = false;
  local_scan.point_count = ready_scan.point_count;
  memcpy(local_scan.points,
         ready_scan.points,
         static_cast<size_t>(ready_scan.point_count) * sizeof(TelemetryPoint));
  ready_scan.pending = false;

  print_json_scan(local_scan);
}

void flush_ready_imu()
{
  if (!ready_imu.pending)
  {
    return;
  }

  TelemetryImuPayload payload{};
  memcpy(&payload, &ready_imu.payload, sizeof(payload));
  ready_imu.pending = false;
  print_json_imu(payload);
}

void setup_espnow()
{
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW init failed, restarting");
    led_error();
    delay(2000);
    ESP.restart();
  }

  esp_now_register_send_cb(on_data_sent);
  esp_now_register_recv_cb(on_data_recv);

  memset(&bot_peer, 0, sizeof(bot_peer));
  memcpy(bot_peer.peer_addr, kBotMac, sizeof(kBotMac));
  bot_peer.channel = kChannel;
  bot_peer.encrypt = false;

  if (esp_now_add_peer(&bot_peer) != ESP_OK)
  {
    Serial.println("Peer add failed");
    led_error();
  }
}

} // namespace

void setup()
{
  Serial.begin(kUsbBaud);
  delay(500);
  setup_led();
  setup_espnow();
  send_viewer_handshake();

  Serial.println("{\"t\":\"status\",\"stage\":\"controller\",\"detail\":\"ready\"}");
}

void loop()
{
  const unsigned long now = millis();

  while (Serial.available())
  {
    const char k = static_cast<char>(Serial.read());

    if (k == '\r')
    {
      continue;
    }

    if (k == '\n')
    {
      if (cmd_buf.length() >= 2)
      {
        send_motor_cmd(cmd_buf);
      }
      cmd_buf = "";
      continue;
    }

    if (cmd_buf.length() > 0)
    {
      cmd_buf += k;
      continue;
    }

    if (k == '1' || k == '2' || k == 'x')
    {
      current_mode = k;
      held_cmd = 'x';
      send_cmd(k);
      Serial.printf("Mode -> %c\n", current_mode);
    }
    else if (k == 'w' || k == 's' || k == 'a' || k == 'd' ||
             k == 'q' || k == 'e')
    {
      held_cmd = k;
    }
    else if (k == 'L' || k == 'R')
    {
      cmd_buf = k;
    }
  }

  if (current_mode == '2' && held_cmd != 'x')
  {
    const bool changed = (held_cmd != last_sent_cmd);
    const bool keepalive = (now - last_send_time >= kKeepaliveMs);
    if (changed || keepalive)
    {
      send_cmd(held_cmd);
    }
  }

  if (now - last_handshake_time >= kViewerHandshakeMs)
  {
    last_handshake_time = now;
    send_viewer_handshake();
  }

  flush_ready_scan();
  flush_ready_imu();
  update_led();
  delay(10);
}
