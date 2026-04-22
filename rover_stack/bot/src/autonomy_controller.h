#pragma once

#include <Arduino.h>

struct LidarSnapshot
{
  bool have_scan = false;
  uint16_t front_min_mm = 0;
  uint16_t left_min_mm = 0;
  uint16_t right_min_mm = 0;
};

using AutonomyDriveCallback = void (*)(int left_pwm, int right_pwm);
using AutonomyStatusEmitter = void (*)(const char *stage, const char *detail);

class AutonomyController
{
public:
  void on_mode_changed(char mode);
  void clear_safety_override();
  bool is_safety_active() const;

  void on_lidar_update(const LidarSnapshot &snapshot,
                       bool lidar_fresh,
                       unsigned long now_ms,
                       AutonomyStatusEmitter emit_status);

  bool update(char mode,
              const LidarSnapshot &snapshot,
              bool lidar_fresh,
              unsigned long now_ms,
              AutonomyDriveCallback drive_cb,
              AutonomyStatusEmitter emit_status);

private:
  enum class WallSide : uint8_t
  {
    Unknown = 0,
    Left = 1,
    Right = 2,
  };

  enum class AutoState : uint8_t
  {
    AcquireWall = 0,
    FollowWall = 1,
    LostWallSearch = 2,
    RecoverOpen = 3,
  };

  enum class SafetyPhase : uint8_t
  {
    None = 0,
    Reverse = 1,
    Turn = 2,
    Release = 3,
  };

  struct AutonomyContext
  {
    AutoState state = AutoState::AcquireWall;
    WallSide wall_side = WallSide::Unknown;
    unsigned long state_deadline_ms = 0;
  };

  struct SafetyController
  {
    bool active = false;
    SafetyPhase phase = SafetyPhase::None;
    bool turn_left = true;
    unsigned long phase_deadline_ms = 0;
  };

  static uint16_t clearance_or_default(uint16_t distance_mm);
  static WallSide choose_nearest_wall_side(const LidarSnapshot &snapshot);

  void reset_autonomy();
  void set_autonomy_state(AutoState state,
                          const char *detail,
                          unsigned long hold_ms,
                          unsigned long now_ms,
                          AutonomyStatusEmitter emit_status);
  void start_safety_override(const char *reason,
                             const LidarSnapshot &snapshot,
                             unsigned long now_ms,
                             AutonomyStatusEmitter emit_status);
  bool run_safety(char mode,
                  const LidarSnapshot &snapshot,
                  unsigned long now_ms,
                  AutonomyDriveCallback drive_cb,
                  AutonomyStatusEmitter emit_status);
  void run_autonomy(const LidarSnapshot &snapshot,
                    bool lidar_fresh,
                    unsigned long now_ms,
                    AutonomyDriveCallback drive_cb,
                    AutonomyStatusEmitter emit_status);

  AutonomyContext auto_ctx_;
  SafetyController safety_controller_;
  uint16_t previous_front_mm_ = 0;
  unsigned long previous_front_ms_ = 0;
};

