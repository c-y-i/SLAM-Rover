#include "autonomy_controller.h"

namespace
{
constexpr uint16_t kLidarEmergencyFrontMm = 220;
constexpr uint16_t kLidarCautionFrontMm = 380;
constexpr uint16_t kLidarPreferredClearMm = 520;
constexpr uint16_t kLidarDefaultOpenMm = 5000;

constexpr int kAutoBasePwm = 105;
constexpr int kAutoSlowPwm = 82;
constexpr int kAutoMinPwm = 70;
constexpr int kAutoMaxPwm = 170;
constexpr int kAutoSteerMax = 68;
constexpr float kAutoSteerGain = 0.22f;
constexpr uint16_t kAutoTargetWallMm = 340;
constexpr uint16_t kAutoWallLostMm = 1150;
constexpr unsigned long kAutoLostSearchMs = 1800;
constexpr unsigned long kAutoRecoverTurnMs = 700;
constexpr int kAutoSearchPwm = 92;
constexpr int kAutoSearchBias = 28;
constexpr int kAutoTurnPwm = 135;
constexpr int kAutoReversePwm = 80;

constexpr uint16_t kSafetyEmergencyFrontMm = 210;
constexpr uint16_t kSafetySuddenFrontMm = 420;
constexpr uint16_t kSafetySuddenDropMm = 280;
constexpr unsigned long kSafetySuddenWindowMs = 300;
constexpr unsigned long kSafetyReverseMs = 170;
constexpr unsigned long kSafetyTurnMs = 470;
constexpr unsigned long kSafetyReleaseMs = 210;
constexpr int kSafetyReversePwm = 125;
constexpr int kSafetyTurnPwm = 190;
constexpr int kSafetyReleasePwm = 90;
} // namespace

void AutonomyController::on_mode_changed(char mode)
{
  if (mode == '1')
  {
    reset_autonomy();
  }
  else if (mode == 'x')
  {
    clear_safety_override();
  }
}

void AutonomyController::clear_safety_override()
{
  safety_controller_.active = false;
  safety_controller_.phase = SafetyPhase::None;
  safety_controller_.phase_deadline_ms = 0;
}

bool AutonomyController::is_safety_active() const
{
  return safety_controller_.active;
}

uint16_t AutonomyController::clearance_or_default(uint16_t distance_mm)
{
  return distance_mm == 0 ? kLidarDefaultOpenMm : distance_mm;
}

AutonomyController::WallSide AutonomyController::choose_nearest_wall_side(const LidarSnapshot &snapshot)
{
  const uint16_t left = snapshot.left_min_mm;
  const uint16_t right = snapshot.right_min_mm;
  if (left == 0 && right == 0)
  {
    return WallSide::Unknown;
  }
  if (left == 0)
  {
    return WallSide::Right;
  }
  if (right == 0)
  {
    return WallSide::Left;
  }
  return (left <= right) ? WallSide::Left : WallSide::Right;
}

void AutonomyController::set_autonomy_state(AutoState state,
                                            const char *detail,
                                            unsigned long hold_ms,
                                            unsigned long now_ms,
                                            AutonomyStatusEmitter emit_status)
{
  if (auto_ctx_.state != state)
  {
    emit_status("auto", detail);
  }
  auto_ctx_.state = state;
  auto_ctx_.state_deadline_ms = now_ms + hold_ms;
}

void AutonomyController::reset_autonomy()
{
  auto_ctx_.state = AutoState::AcquireWall;
  auto_ctx_.wall_side = WallSide::Unknown;
  auto_ctx_.state_deadline_ms = 0;
}

void AutonomyController::start_safety_override(const char *reason,
                                               const LidarSnapshot &snapshot,
                                               unsigned long now_ms,
                                               AutonomyStatusEmitter emit_status)
{
  if (safety_controller_.active)
  {
    return;
  }

  safety_controller_.active = true;
  safety_controller_.phase = SafetyPhase::Reverse;
  safety_controller_.phase_deadline_ms = now_ms + kSafetyReverseMs;
  safety_controller_.turn_left =
      clearance_or_default(snapshot.left_min_mm) >=
      clearance_or_default(snapshot.right_min_mm);

  emit_status("safety", reason);
}

void AutonomyController::on_lidar_update(const LidarSnapshot &snapshot,
                                         bool lidar_fresh,
                                         unsigned long now_ms,
                                         AutonomyStatusEmitter emit_status)
{
  if (!lidar_fresh || !snapshot.have_scan)
  {
    return;
  }

  const uint16_t front_mm = snapshot.front_min_mm;

  if (front_mm > 0 && front_mm <= kSafetyEmergencyFrontMm)
  {
    start_safety_override("front_emergency", snapshot, now_ms, emit_status);
  }

  if (previous_front_mm_ > 0 && front_mm > 0 &&
      (previous_front_mm_ > front_mm) &&
      (previous_front_mm_ - front_mm) >= kSafetySuddenDropMm &&
      front_mm <= kSafetySuddenFrontMm &&
      (now_ms - previous_front_ms_) <= kSafetySuddenWindowMs)
  {
    start_safety_override("sudden_obstacle", snapshot, now_ms, emit_status);
  }

  previous_front_mm_ = front_mm;
  previous_front_ms_ = now_ms;
}

bool AutonomyController::run_safety(char mode,
                                    const LidarSnapshot &snapshot,
                                    unsigned long now_ms,
                                    AutonomyDriveCallback drive_cb,
                                    AutonomyStatusEmitter emit_status)
{
  if (!safety_controller_.active)
  {
    return false;
  }

  switch (safety_controller_.phase)
  {
    case SafetyPhase::Reverse:
      drive_cb(-kSafetyReversePwm, -kSafetyReversePwm);
      if (now_ms >= safety_controller_.phase_deadline_ms)
      {
        safety_controller_.phase = SafetyPhase::Turn;
        safety_controller_.phase_deadline_ms = now_ms + kSafetyTurnMs;
      }
      return true;

    case SafetyPhase::Turn:
      if (safety_controller_.turn_left)
      {
        drive_cb(-kSafetyTurnPwm, kSafetyTurnPwm);
      }
      else
      {
        drive_cb(kSafetyTurnPwm, -kSafetyTurnPwm);
      }
      if (now_ms >= safety_controller_.phase_deadline_ms)
      {
        safety_controller_.phase = SafetyPhase::Release;
        safety_controller_.phase_deadline_ms = now_ms + kSafetyReleaseMs;
      }
      return true;

    case SafetyPhase::Release:
      drive_cb(kSafetyReleasePwm, kSafetyReleasePwm);
      if (now_ms >= safety_controller_.phase_deadline_ms)
      {
        clear_safety_override();
        emit_status("safety", "clear");
        if (mode == '1')
        {
          set_autonomy_state(AutoState::RecoverOpen, "recover_open", kAutoRecoverTurnMs, now_ms, emit_status);
        }
      }
      return true;

    case SafetyPhase::None:
    default:
      clear_safety_override();
      return false;
  }
}

void AutonomyController::run_autonomy(const LidarSnapshot &snapshot,
                                      bool lidar_fresh,
                                      unsigned long now_ms,
                                      AutonomyDriveCallback drive_cb,
                                      AutonomyStatusEmitter emit_status)
{
  if (!lidar_fresh)
  {
    drive_cb(-kAutoSearchBias, kAutoSearchBias);
    return;
  }

  if (auto_ctx_.wall_side == WallSide::Unknown)
  {
    auto_ctx_.wall_side = choose_nearest_wall_side(snapshot);
    if (auto_ctx_.wall_side == WallSide::Unknown)
    {
      auto_ctx_.wall_side = WallSide::Right;
    }
  }

  const uint16_t front_mm = snapshot.front_min_mm;
  const uint16_t side_mm =
      (auto_ctx_.wall_side == WallSide::Left) ? snapshot.left_min_mm : snapshot.right_min_mm;

  switch (auto_ctx_.state)
  {
    case AutoState::AcquireWall:
      if (front_mm > 0 && front_mm <= kLidarCautionFrontMm)
      {
        set_autonomy_state(AutoState::RecoverOpen, "recover_open", kAutoRecoverTurnMs, now_ms, emit_status);
        break;
      }
      if (side_mm > 0 && side_mm <= kAutoWallLostMm)
      {
        set_autonomy_state(AutoState::FollowWall, "follow_wall", 0, now_ms, emit_status);
        break;
      }
      if (auto_ctx_.wall_side == WallSide::Left)
      {
        drive_cb(kAutoSearchPwm - kAutoSearchBias, kAutoSearchPwm + kAutoSearchBias);
      }
      else
      {
        drive_cb(kAutoSearchPwm + kAutoSearchBias, kAutoSearchPwm - kAutoSearchBias);
      }
      break;

    case AutoState::FollowWall:
      if (front_mm > 0 && front_mm <= kLidarCautionFrontMm)
      {
        set_autonomy_state(AutoState::RecoverOpen, "recover_open", kAutoRecoverTurnMs, now_ms, emit_status);
        break;
      }
      if (side_mm == 0 || side_mm > kAutoWallLostMm)
      {
        set_autonomy_state(AutoState::LostWallSearch, "lost_wall_search", kAutoLostSearchMs, now_ms, emit_status);
        break;
      }
      {
        int base_pwm = (front_mm > 0 && front_mm < kLidarPreferredClearMm) ? kAutoSlowPwm : kAutoBasePwm;
        int error_mm = static_cast<int>(kAutoTargetWallMm) - static_cast<int>(side_mm);
        int steer_pwm = constrain(static_cast<int>(error_mm * kAutoSteerGain), -kAutoSteerMax, kAutoSteerMax);

        int left_pwm = base_pwm;
        int right_pwm = base_pwm;
        if (auto_ctx_.wall_side == WallSide::Right)
        {
          left_pwm = base_pwm - steer_pwm;
          right_pwm = base_pwm + steer_pwm;
        }
        else
        {
          left_pwm = base_pwm + steer_pwm;
          right_pwm = base_pwm - steer_pwm;
        }

        left_pwm = constrain(left_pwm, kAutoMinPwm, kAutoMaxPwm);
        right_pwm = constrain(right_pwm, kAutoMinPwm, kAutoMaxPwm);
        drive_cb(left_pwm, right_pwm);
      }
      break;

    case AutoState::LostWallSearch:
      if (side_mm > 0 && side_mm <= kAutoWallLostMm)
      {
        set_autonomy_state(AutoState::FollowWall, "follow_wall", 0, now_ms, emit_status);
        break;
      }
      if (now_ms >= auto_ctx_.state_deadline_ms)
      {
        set_autonomy_state(AutoState::AcquireWall, "acquire_wall", 0, now_ms, emit_status);
        break;
      }
      if (auto_ctx_.wall_side == WallSide::Left)
      {
        drive_cb(kAutoSearchPwm - kAutoSearchBias, kAutoSearchPwm + kAutoSearchBias);
      }
      else
      {
        drive_cb(kAutoSearchPwm + kAutoSearchBias, kAutoSearchPwm - kAutoSearchBias);
      }
      break;

    case AutoState::RecoverOpen:
    {
      const bool turn_left =
          clearance_or_default(snapshot.left_min_mm) >= clearance_or_default(snapshot.right_min_mm);
      if (now_ms < auto_ctx_.state_deadline_ms)
      {
        if (front_mm > 0 && front_mm <= kLidarEmergencyFrontMm)
        {
          if (turn_left)
          {
            drive_cb(-kAutoReversePwm, kAutoTurnPwm);
          }
          else
          {
            drive_cb(kAutoTurnPwm, -kAutoReversePwm);
          }
        }
        else
        {
          if (turn_left)
          {
            drive_cb(-kAutoTurnPwm, kAutoTurnPwm);
          }
          else
          {
            drive_cb(kAutoTurnPwm, -kAutoTurnPwm);
          }
        }
        break;
      }

      auto_ctx_.wall_side = choose_nearest_wall_side(snapshot);
      if (auto_ctx_.wall_side == WallSide::Unknown)
      {
        set_autonomy_state(AutoState::AcquireWall, "acquire_wall", 0, now_ms, emit_status);
      }
      else
      {
        set_autonomy_state(AutoState::FollowWall, "follow_wall", 0, now_ms, emit_status);
      }
      break;
    }
  }
}

bool AutonomyController::update(char mode,
                                const LidarSnapshot &snapshot,
                                bool lidar_fresh,
                                unsigned long now_ms,
                                AutonomyDriveCallback drive_cb,
                                AutonomyStatusEmitter emit_status)
{
  if (run_safety(mode, snapshot, now_ms, drive_cb, emit_status))
  {
    return true;
  }

  if (mode != '1')
  {
    return false;
  }

  run_autonomy(snapshot, lidar_fresh, now_ms, drive_cb, emit_status);
  return true;
}

