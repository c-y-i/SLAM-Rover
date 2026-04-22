#include "motor_driver.h"

namespace
{

constexpr uint8_t kLedcBits = 14;
constexpr uint32_t kLedcMax = (1UL << kLedcBits) - 1;
constexpr uint32_t kLedcFreqHz = 30;

constexpr int kLeftPwmPin = 4;
constexpr int kLeftMotorForwPin = 5;
constexpr int kLeftMotorBackwPin = 6;
constexpr int kRightMotorForwPin = 7;
constexpr int kRightMotorBackwPin = 8;
constexpr int kRightPwmPin = 10;

constexpr int kTb6612LeftPwmPin = 4;
constexpr int kTb6612LeftIn1Pin = 5;
constexpr int kTb6612LeftIn2Pin = 6;
constexpr int kTb6612RightPwmPin = 10;
constexpr int kTb6612RightIn1Pin = 7;
constexpr int kTb6612RightIn2Pin = 8;
constexpr int kTb6612StbyPin = TB6612_STBY_PIN;

class L298Driver final : public MotorDriver
{
public:
  L298Driver(int left_pwm_pin,
             int left_forward_pin,
             int left_backward_pin,
             int right_pwm_pin,
             int right_forward_pin,
             int right_backward_pin)
      : left_pwm_pin_(left_pwm_pin),
        left_forward_pin_(left_forward_pin),
        left_backward_pin_(left_backward_pin),
        right_pwm_pin_(right_pwm_pin),
        right_forward_pin_(right_forward_pin),
        right_backward_pin_(right_backward_pin) {}

  void begin() override
  {
    ledcAttach(left_pwm_pin_, kLedcFreqHz, kLedcBits);
    ledcAttach(right_pwm_pin_, kLedcFreqHz, kLedcBits);

    pinMode(left_forward_pin_, OUTPUT);
    pinMode(left_backward_pin_, OUTPUT);
    pinMode(right_forward_pin_, OUTPUT);
    pinMode(right_backward_pin_, OUTPUT);
    set_wheel_pwm(0, 0);
  }

  void set_wheel_pwm(int left_pwm, int right_pwm) override
  {
    apply_wheel(left_pwm_pin_, left_forward_pin_, left_backward_pin_, left_pwm);
    apply_wheel(right_pwm_pin_, right_forward_pin_, right_backward_pin_, right_pwm);
  }

private:
  static void apply_wheel(int pwm_pin, int forward_pin, int backward_pin, int pwm)
  {
    int clamped = constrain(pwm, -255, 255);
    if (clamped > 0)
    {
      digitalWrite(forward_pin, HIGH);
      digitalWrite(backward_pin, LOW);
    }
    else if (clamped < 0)
    {
      digitalWrite(forward_pin, LOW);
      digitalWrite(backward_pin, HIGH);
      clamped = -clamped;
    }
    else
    {
      digitalWrite(forward_pin, LOW);
      digitalWrite(backward_pin, LOW);
    }

    const uint32_t duty = static_cast<uint32_t>(clamped) * kLedcMax / 255U;
    ledcWrite(pwm_pin, duty);
  }

  int left_pwm_pin_;
  int left_forward_pin_;
  int left_backward_pin_;
  int right_pwm_pin_;
  int right_forward_pin_;
  int right_backward_pin_;
};

class Tb6612Driver final : public MotorDriver
{
public:
  Tb6612Driver(int left_pwm_pin,
               int left_in1_pin,
               int left_in2_pin,
               int right_pwm_pin,
               int right_in1_pin,
               int right_in2_pin,
               int stby_pin)
      : left_pwm_pin_(left_pwm_pin),
        left_in1_pin_(left_in1_pin),
        left_in2_pin_(left_in2_pin),
        right_pwm_pin_(right_pwm_pin),
        right_in1_pin_(right_in1_pin),
        right_in2_pin_(right_in2_pin),
        stby_pin_(stby_pin) {}

  void begin() override
  {
    ledcAttach(left_pwm_pin_, kLedcFreqHz, kLedcBits);
    ledcAttach(right_pwm_pin_, kLedcFreqHz, kLedcBits);

    pinMode(left_in1_pin_, OUTPUT);
    pinMode(left_in2_pin_, OUTPUT);
    pinMode(right_in1_pin_, OUTPUT);
    pinMode(right_in2_pin_, OUTPUT);
    if (stby_pin_ >= 0)
    {
      pinMode(stby_pin_, OUTPUT);
      digitalWrite(stby_pin_, HIGH);
    }

    set_wheel_pwm(0, 0);
  }

  void set_wheel_pwm(int left_pwm, int right_pwm) override
  {
    apply_wheel(left_pwm_pin_, left_in1_pin_, left_in2_pin_, left_pwm);
    apply_wheel(right_pwm_pin_, right_in1_pin_, right_in2_pin_, right_pwm);
  }

private:
  static void apply_wheel(int pwm_pin, int in1_pin, int in2_pin, int pwm)
  {
    int clamped = constrain(pwm, -255, 255);
    if (clamped > 0)
    {
      digitalWrite(in1_pin, HIGH);
      digitalWrite(in2_pin, LOW);
    }
    else if (clamped < 0)
    {
      digitalWrite(in1_pin, LOW);
      digitalWrite(in2_pin, HIGH);
      clamped = -clamped;
    }
    else
    {
      digitalWrite(in1_pin, LOW);
      digitalWrite(in2_pin, LOW);
    }

    const uint32_t duty = static_cast<uint32_t>(clamped) * kLedcMax / 255U;
    ledcWrite(pwm_pin, duty);
  }

  int left_pwm_pin_;
  int left_in1_pin_;
  int left_in2_pin_;
  int right_pwm_pin_;
  int right_in1_pin_;
  int right_in2_pin_;
  int stby_pin_;
};

#if MOTOR_DRIVER_TYPE == MOTOR_DRIVER_TB6612
Tb6612Driver g_motor_driver(
    kTb6612LeftPwmPin,
    kTb6612LeftIn1Pin,
    kTb6612LeftIn2Pin,
    kTb6612RightPwmPin,
    kTb6612RightIn1Pin,
    kTb6612RightIn2Pin,
    kTb6612StbyPin);
constexpr const char *kMotorDriverName = "driver_tb6612";
#else
L298Driver g_motor_driver(
    kLeftPwmPin,
    kLeftMotorForwPin,
    kLeftMotorBackwPin,
    kRightPwmPin,
    kRightMotorForwPin,
    kRightMotorBackwPin);
constexpr const char *kMotorDriverName = "driver_l298";
#endif

} // namespace

MotorDriver &default_motor_driver()
{
  return g_motor_driver;
}

const char *default_motor_driver_name()
{
  return kMotorDriverName;
}
