#pragma once

#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>

#include <LD06_LiDAR.h>

class WebViewer
{
public:
  explicit WebViewer(LD06_LiDAR &reader);

  void begin();
  void loop();

private:
  void connect_wifi_sta();
  void ensure_wifi_connection();
  void start_web_server();
  void handle_root();
  void handle_scan_json();

  LD06_LiDAR &reader_;
  WebServer server_{80};
  uint32_t last_wifi_retry_ms_ = 0;
  bool web_server_started_ = false;
};
