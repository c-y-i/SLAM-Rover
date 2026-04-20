#pragma once

#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>

#include "lidar_reader.h"

class WebViewer
{
public:
  explicit WebViewer(lidar::Reader &reader);

  void begin();
  void loop();

private:
  void connect_wifi_sta();
  void ensure_wifi_connection();
  void start_web_server();
  void handle_root();
  void handle_scan_json();

  lidar::Reader &reader_;
  WebServer server_{80};
  uint32_t last_wifi_retry_ms_ = 0;
  bool web_server_started_ = false;
};
