#pragma once

#include <Arduino.h>

#define MOTOR_DRIVER_L298 0
#define MOTOR_DRIVER_TB6612 1
#ifndef MOTOR_DRIVER_TYPE
#define MOTOR_DRIVER_TYPE MOTOR_DRIVER_TB6612
#endif

#ifndef TB6612_STBY_PIN
#define TB6612_STBY_PIN -1
#endif

class MotorDriver
{
public:
  virtual void begin() = 0;
  virtual void set_wheel_pwm(int left_pwm, int right_pwm) = 0;
  virtual ~MotorDriver() = default;
};

MotorDriver &default_motor_driver();
const char *default_motor_driver_name();
