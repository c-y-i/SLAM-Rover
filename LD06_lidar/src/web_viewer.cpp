#include "web_viewer.h"

#include "config.h"
#include "lidar_data.h"

namespace
{

constexpr char kViewerPage[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>LD06 Viewer</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #09111f;
      --panel: rgba(14, 27, 46, 0.88);
      --line: rgba(129, 182, 255, 0.18);
      --text: #ebf4ff;
      --muted: #9eb6d2;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      font-family: "Segoe UI", sans-serif;
      color: var(--text);
      background:
        radial-gradient(circle at top, rgba(62, 110, 175, 0.35), transparent 35%),
        radial-gradient(circle at bottom, rgba(30, 87, 153, 0.3), transparent 30%),
        linear-gradient(180deg, #04070d, var(--bg));
      display: grid;
      place-items: center;
      padding: 20px;
    }
    .shell {
      width: min(100%, 960px);
      background: var(--panel);
      border: 1px solid rgba(150, 195, 255, 0.14);
      border-radius: 24px;
      overflow: hidden;
      box-shadow: 0 18px 60px rgba(0, 0, 0, 0.35);
      backdrop-filter: blur(14px);
    }
    .header {
      display: flex;
      justify-content: space-between;
      gap: 16px;
      padding: 20px 24px 12px;
      align-items: end;
    }
    h1 {
      margin: 0;
      font-size: clamp(1.5rem, 4vw, 2.4rem);
      letter-spacing: 0.04em;
    }
    .sub {
      color: var(--muted);
      margin-top: 6px;
      font-size: 0.95rem;
    }
    .stats {
      display: grid;
      grid-template-columns: repeat(4, minmax(72px, 1fr));
      gap: 12px;
      padding: 0 24px 20px;
    }
    .card {
      background: rgba(6, 16, 29, 0.72);
      border: 1px solid rgba(147, 187, 237, 0.08);
      border-radius: 16px;
      padding: 12px 14px;
    }
    .label {
      color: var(--muted);
      font-size: 0.78rem;
      text-transform: uppercase;
      letter-spacing: 0.08em;
    }
    .value {
      margin-top: 6px;
      font-size: 1.25rem;
      font-weight: 700;
    }
    .canvas-wrap {
      padding: 0 18px 18px;
    }
    canvas {
      width: 100%;
      aspect-ratio: 1 / 1;
      display: block;
      background:
        radial-gradient(circle at center, rgba(35, 70, 124, 0.22), rgba(2, 5, 10, 0.96));
      border-radius: 22px;
      border: 1px solid rgba(135, 182, 241, 0.1);
    }
    @media (max-width: 700px) {
      .header { flex-direction: column; align-items: start; }
      .stats { grid-template-columns: repeat(2, minmax(72px, 1fr)); }
    }
  </style>
</head>
<body>
  <main class="shell">
    <section class="header">
      <div>
        <h1>LD06 Live Viewer</h1>
        <div class="sub">Browser render served directly from the ESP32</div>
      </div>
      <div class="sub" id="status">Connecting...</div>
    </section>
    <section class="stats">
      <div class="card"><div class="label">Packets</div><div class="value" id="packets">0</div></div>
      <div class="card"><div class="label">Speed</div><div class="value" id="speed">0</div></div>
      <div class="card"><div class="label">Points</div><div class="value" id="points">0</div></div>
      <div class="card"><div class="label">Valid</div><div class="value" id="valid">0</div></div>
      <div class="card"><div class="label">CRC Fails</div><div class="value" id="crc">0</div></div>
      <div class="card"><div class="label">Timestamp</div><div class="value" id="stamp">0</div></div>
    </section>
    <section class="canvas-wrap">
      <canvas id="plot" width="900" height="900"></canvas>
    </section>
  </main>
  <script>
    const canvas = document.getElementById('plot');
    const ctx = canvas.getContext('2d');
    const packetsEl = document.getElementById('packets');
    const speedEl = document.getElementById('speed');
    const pointsEl = document.getElementById('points');
    const validEl = document.getElementById('valid');
    const crcEl = document.getElementById('crc');
    const stampEl = document.getElementById('stamp');
    const statusEl = document.getElementById('status');

    function drawGrid(size, center, radius) {
      ctx.clearRect(0, 0, size, size);
      ctx.fillStyle = '#081221';
      ctx.fillRect(0, 0, size, size);

      ctx.save();
      ctx.translate(center, center);
      ctx.strokeStyle = 'rgba(129, 182, 255, 0.18)';
      ctx.lineWidth = 1;

      for (let ring = 1; ring <= 4; ring++) {
        ctx.beginPath();
        ctx.arc(0, 0, (radius * ring) / 4, 0, Math.PI * 2);
        ctx.stroke();
      }

      for (let deg = 0; deg < 360; deg += 30) {
        const rad = (deg - 90) * Math.PI / 180;
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(Math.cos(rad) * radius, Math.sin(rad) * radius);
        ctx.stroke();
      }

      ctx.restore();
    }

    function drawScan(data) {
      const size = canvas.width;
      const center = size / 2;
      const radius = size * 0.44;
      drawGrid(size, center, radius);

      ctx.save();
      ctx.translate(center, center);

      for (const point of data.points) {
        if (!point.valid || point.distance <= 0) {
          continue;
        }

        const maxRange = data.max_range || 12000;
        const scale = Math.min(point.distance, maxRange) / Math.max(point.distance, 1);
        const x = (point.x * scale / maxRange) * radius;
        const y = (point.y * scale / maxRange) * radius;
        const alpha = Math.max(0.15, point.intensity / 255);

        ctx.fillStyle = `rgba(124, 231, 255, ${alpha.toFixed(3)})`;
        ctx.beginPath();
        ctx.arc(x, y, 2.2, 0, Math.PI * 2);
        ctx.fill();
      }

      ctx.strokeStyle = 'rgba(140, 255, 156, 0.8)';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(0, -radius);
      ctx.stroke();
      ctx.restore();
    }

    async function refresh() {
      try {
        const response = await fetch('/scan.json', { cache: 'no-store' });
        const data = await response.json();
        packetsEl.textContent = data.packets;
        speedEl.textContent = data.speed;
        pointsEl.textContent = data.point_count;
        validEl.textContent = data.valid_points;
        crcEl.textContent = data.crc_fail_count;
        stampEl.textContent = data.timestamp;
        statusEl.textContent = `Updated ${new Date().toLocaleTimeString()}`;
        drawScan(data);
      } catch (error) {
        statusEl.textContent = 'Waiting for scan data...';
      }
    }

    refresh();
    setInterval(refresh, 120);
  </script>
</body>
</html>
)HTML";

} // namespace

WebViewer::WebViewer(lidar::Reader &reader) : reader_(reader)
{
}

void WebViewer::handle_root()
{
  server_.send_P(200, "text/html", kViewerPage);
}

void WebViewer::handle_scan_json()
{
  const lidar::ScanFrame &scan = reader_.latest_scan();
  String body;
  body.reserve(256 + (static_cast<size_t>(scan.point_count) * 88));
  body += F("{\"packets\":");
  body += reader_.packets_seen();
  body += F(",\"speed\":");
  body += scan.speed_raw;
  body += F(",\"point_count\":");
  body += scan.point_count;
  body += F(",\"valid_points\":");
  body += scan.valid_point_count;
  body += F(",\"timestamp\":");
  body += scan.timestamp_ms;
  body += F(",\"crc_fail_count\":");
  body += scan.crc_fail_count;
  body += F(",\"max_range\":");
  body += lidar::kMaxDisplayDistanceMm;
  body += F(",\"points\":[");

  for (uint16_t i = 0; i < scan.point_count; ++i)
  {
    if (i > 0)
    {
      body += ',';
    }

    const lidar::ScanPoint &point = scan.points[i];
    body += F("{\"angle\":");
    body += String(point.angle_deg, 2);
    body += F(",\"distance\":");
    body += point.distance_mm;
    body += F(",\"intensity\":");
    body += point.intensity;
    body += F(",\"x\":");
    body += point.x_mm;
    body += F(",\"y\":");
    body += point.y_mm;
    body += F(",\"valid\":");
    body += point.valid ? F("true") : F("false");
    body += '}';
  }

  body += F("]}");
  server_.send(200, "application/json", body);
}

void WebViewer::start_web_server()
{
  if (web_server_started_)
  {
    return;
  }

  server_.on("/", HTTP_GET, [this]() { handle_root(); });
  server_.on("/scan.json", HTTP_GET, [this]() { handle_scan_json(); });
  server_.begin();
  web_server_started_ = true;

#if LIDAR_DEBUG_LOGS
  Serial.println("Web viewer ready");
  Serial.printf("Open http://%s/\n", WiFi.localIP().toString().c_str());
#endif
}

void WebViewer::connect_wifi_sta()
{
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  if (!WiFi.config(config::kWifiStaticIp,
                   config::kWifiGateway,
                   config::kWifiSubnet,
                   config::kWifiDns))
  {
#if LIDAR_DEBUG_LOGS
    Serial.println("Failed to apply static IP config, falling back to DHCP.");
#endif
  }

  WiFi.begin(config::kWifiSsid, config::kWifiPassword);

#if LIDAR_DEBUG_LOGS
  Serial.printf("Connecting to Wi-Fi SSID \"%s\"", config::kWifiSsid);
#endif
  const uint32_t start_ms = millis();
  while (WiFi.status() != WL_CONNECTED &&
         (millis() - start_ms) < config::kWifiConnectTimeoutMs)
  {
    delay(250);
#if LIDAR_DEBUG_LOGS
    Serial.print('.');
#endif
  }
#if LIDAR_DEBUG_LOGS
  Serial.println();
#endif

  if (WiFi.status() == WL_CONNECTED)
  {
#if LIDAR_DEBUG_LOGS
    Serial.printf("Wi-Fi connected, IP address: %s\n",
                  WiFi.localIP().toString().c_str());
    Serial.printf("Expected viewer URL: http://%s/\n",
                  config::kWifiStaticIp.toString().c_str());
#endif
    start_web_server();
  }
  else
  {
#if LIDAR_DEBUG_LOGS
    Serial.println("Wi-Fi connection timed out, viewer will stay offline until reconnect.");
#endif
  }
}

void WebViewer::ensure_wifi_connection()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    if (!web_server_started_)
    {
      start_web_server();
    }
    return;
  }

  if (millis() - last_wifi_retry_ms_ < 5000)
  {
    return;
  }

  last_wifi_retry_ms_ = millis();
#if LIDAR_DEBUG_LOGS
  Serial.println("Wi-Fi disconnected, retrying STA connection...");
#endif
  WiFi.disconnect();
  WiFi.begin(config::kWifiSsid, config::kWifiPassword);
}

void WebViewer::begin()
{
  connect_wifi_sta();
}

void WebViewer::loop()
{
  ensure_wifi_connection();

  if (web_server_started_)
  {
    server_.handleClient();
  }
}
