#include "system/runtime_state.h"

#include <freertos/FreeRTOS.h>

namespace {

constexpr float kDefaultBalanceTargetPitchDeg = 0.795f;
constexpr float kDefaultBalanceKp = 1.69f;
constexpr float kDefaultBalanceKd = 0.028f;
constexpr float kDefaultBalanceKv = 0.5f;
constexpr bool kDefaultBalanceUseLqr = false;
constexpr float kDefaultBalanceLqrPitch = -119.86971f;
constexpr float kDefaultBalanceLqrPitchRate = -18.810969f;
constexpr float kDefaultBalanceLqrWheelVelocity = -1.208787f;
constexpr float kDefaultBalanceLqrOutputSlewRate = 80.0f;
constexpr float kDefaultBalanceOutputDirection = -1.0f;
constexpr float kDefaultBalanceMaxVelocity = 10.0f;
constexpr float kDefaultBalanceStartAngleDeg = 10.0f;
constexpr float kDefaultBalanceMaxAngleDeg = 35.0f;
constexpr float kDefaultRemoteVelocity = 0.0f;
constexpr float kDefaultRemoteTurnVelocity = 0.0f;
constexpr float kDefaultLegTargetX = 2.0f;
constexpr float kDefaultLegHeightCm = 20.0f;
constexpr uint16_t kDefaultLegMoveTimeMs = 500;

runtime_state::SystemSnapshot g_system_snapshot = {};
runtime_state::MotorCommand g_left_motor_command = {};
runtime_state::MotorCommand g_right_motor_command = {};
runtime_state::ServoCommand g_servo_command = {
    .has_height = false,
    .target_x = kDefaultLegTargetX,
    .target_height = kDefaultLegHeightCm,
    .time_ms = kDefaultLegMoveTimeMs,
    .updated_ms = 0,
    .sequence = 0,
};
runtime_state::BalanceCommand g_balance_command = {
    .enable = false,
    .stop = false,
    .has_enable = false,
    .has_tuning = false,
    .has_remote_velocity = false,
    .has_remote_turn_velocity = false,
    .target_pitch_deg = kDefaultBalanceTargetPitchDeg,
    .kp = kDefaultBalanceKp,
    .kd = kDefaultBalanceKd,
    .kv = kDefaultBalanceKv,
    .use_lqr = kDefaultBalanceUseLqr,
    .lqr_pitch = kDefaultBalanceLqrPitch,
    .lqr_pitch_rate = kDefaultBalanceLqrPitchRate,
    .lqr_wheel_velocity = kDefaultBalanceLqrWheelVelocity,
    .lqr_output_slew_rate = kDefaultBalanceLqrOutputSlewRate,
    .output_direction = kDefaultBalanceOutputDirection,
    .max_velocity = kDefaultBalanceMaxVelocity,
    .start_angle_deg = kDefaultBalanceStartAngleDeg,
    .max_angle_deg = kDefaultBalanceMaxAngleDeg,
    .remote_velocity = kDefaultRemoteVelocity,
    .remote_turn_velocity = kDefaultRemoteTurnVelocity,
    .remote_sequence = 0,
    .updated_ms = 0,
    .sequence = 0,
};
portMUX_TYPE g_runtime_state_mux = portMUX_INITIALIZER_UNLOCKED;

}  // namespace

namespace runtime_state {

void begin() {
  portENTER_CRITICAL(&g_runtime_state_mux);
  g_system_snapshot = {};
  g_left_motor_command = {};
  g_right_motor_command = {};
  g_servo_command = {
      .has_height = false,
      .target_x = kDefaultLegTargetX,
      .target_height = kDefaultLegHeightCm,
      .time_ms = kDefaultLegMoveTimeMs,
      .updated_ms = 0,
      .sequence = 0,
  };
  g_balance_command = {
      .enable = false,
      .stop = false,
      .has_enable = false,
      .has_tuning = false,
      .has_remote_velocity = false,
      .has_remote_turn_velocity = false,
      .target_pitch_deg = kDefaultBalanceTargetPitchDeg,
      .kp = kDefaultBalanceKp,
      .kd = kDefaultBalanceKd,
      .kv = kDefaultBalanceKv,
      .use_lqr = kDefaultBalanceUseLqr,
      .lqr_pitch = kDefaultBalanceLqrPitch,
      .lqr_pitch_rate = kDefaultBalanceLqrPitchRate,
      .lqr_wheel_velocity = kDefaultBalanceLqrWheelVelocity,
      .lqr_output_slew_rate = kDefaultBalanceLqrOutputSlewRate,
      .output_direction = kDefaultBalanceOutputDirection,
      .max_velocity = kDefaultBalanceMaxVelocity,
      .start_angle_deg = kDefaultBalanceStartAngleDeg,
      .max_angle_deg = kDefaultBalanceMaxAngleDeg,
      .remote_velocity = kDefaultRemoteVelocity,
      .remote_turn_velocity = kDefaultRemoteTurnVelocity,
      .remote_sequence = 0,
      .updated_ms = 0,
      .sequence = 0,
  };
  portEXIT_CRITICAL(&g_runtime_state_mux);
}

void updateControlSnapshot(const ControlSnapshot& snapshot) {
  portENTER_CRITICAL(&g_runtime_state_mux);
  g_system_snapshot.control = snapshot;
  portEXIT_CRITICAL(&g_runtime_state_mux);
}

void updateServiceSnapshot(const ServiceSnapshot& snapshot) {
  portENTER_CRITICAL(&g_runtime_state_mux);
  g_system_snapshot.service = snapshot;
  portEXIT_CRITICAL(&g_runtime_state_mux);
}

void updateImuSnapshot(const ImuSnapshot& snapshot) {
  portENTER_CRITICAL(&g_runtime_state_mux);
  g_system_snapshot.imu = snapshot;
  portEXIT_CRITICAL(&g_runtime_state_mux);
}

void updateLeftMotorCommand(const MotorCommand& command) {
  portENTER_CRITICAL(&g_runtime_state_mux);
  g_left_motor_command = command;
  portEXIT_CRITICAL(&g_runtime_state_mux);
}

void updateRightMotorCommand(const MotorCommand& command) {
  portENTER_CRITICAL(&g_runtime_state_mux);
  g_right_motor_command = command;
  portEXIT_CRITICAL(&g_runtime_state_mux);
}

void updateBalanceCommand(const BalanceCommand& command) {
  portENTER_CRITICAL(&g_runtime_state_mux);
  g_balance_command = command;
  portEXIT_CRITICAL(&g_runtime_state_mux);
}

void updateServoCommand(const ServoCommand& command) {
  portENTER_CRITICAL(&g_runtime_state_mux);
  g_servo_command = command;
  portEXIT_CRITICAL(&g_runtime_state_mux);
}

MotorCommand leftMotorCommand() {
  portENTER_CRITICAL(&g_runtime_state_mux);
  const MotorCommand command = g_left_motor_command;
  portEXIT_CRITICAL(&g_runtime_state_mux);
  return command;
}

MotorCommand rightMotorCommand() {
  portENTER_CRITICAL(&g_runtime_state_mux);
  const MotorCommand command = g_right_motor_command;
  portEXIT_CRITICAL(&g_runtime_state_mux);
  return command;
}

BalanceCommand balanceCommand() {
  portENTER_CRITICAL(&g_runtime_state_mux);
  const BalanceCommand command = g_balance_command;
  portEXIT_CRITICAL(&g_runtime_state_mux);
  return command;
}

ServoCommand servoCommand() {
  portENTER_CRITICAL(&g_runtime_state_mux);
  const ServoCommand command = g_servo_command;
  portEXIT_CRITICAL(&g_runtime_state_mux);
  return command;
}

SystemSnapshot snapshot() {
  portENTER_CRITICAL(&g_runtime_state_mux);
  const SystemSnapshot current_snapshot = g_system_snapshot;
  portEXIT_CRITICAL(&g_runtime_state_mux);
  return current_snapshot;
}

}  // namespace runtime_state
