#include "system/runtime_state.h"

#include <freertos/FreeRTOS.h>

namespace {

constexpr float kDefaultBalanceTargetPitchDeg = 0.0f;
constexpr float kDefaultBalanceKp = 0.6f;
constexpr float kDefaultBalanceKd = 0.03f;
constexpr float kDefaultBalanceMaxVelocity = 4.0f;
constexpr float kDefaultBalanceMaxAngleDeg = 25.0f;

runtime_state::SystemSnapshot g_system_snapshot = {};
runtime_state::MotorCommand g_left_motor_command = {};
runtime_state::MotorCommand g_right_motor_command = {};
runtime_state::BalanceCommand g_balance_command = {
    .enable = false,
    .stop = false,
    .has_enable = false,
    .has_tuning = false,
    .target_pitch_deg = kDefaultBalanceTargetPitchDeg,
    .kp = kDefaultBalanceKp,
    .kd = kDefaultBalanceKd,
    .max_velocity = kDefaultBalanceMaxVelocity,
    .max_angle_deg = kDefaultBalanceMaxAngleDeg,
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
  g_balance_command = {
      .enable = false,
      .stop = false,
      .has_enable = false,
      .has_tuning = false,
      .target_pitch_deg = kDefaultBalanceTargetPitchDeg,
      .kp = kDefaultBalanceKp,
      .kd = kDefaultBalanceKd,
      .max_velocity = kDefaultBalanceMaxVelocity,
      .max_angle_deg = kDefaultBalanceMaxAngleDeg,
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

SystemSnapshot snapshot() {
  portENTER_CRITICAL(&g_runtime_state_mux);
  const SystemSnapshot current_snapshot = g_system_snapshot;
  portEXIT_CRITICAL(&g_runtime_state_mux);
  return current_snapshot;
}

}  // namespace runtime_state
