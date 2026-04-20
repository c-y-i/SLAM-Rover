#include <Arduino.h>

#include "config.h"
#include "sensor_streamer.h"

namespace
{

sensor::SensorStreamer sensor_streamer;

} // namespace

void setup()
{
  Serial.begin(config::kUsbBaud);
  delay(250);
  sensor_streamer.begin();
}

void loop()
{
  sensor_streamer.poll();
  delay(1);
}
