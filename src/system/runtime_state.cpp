#include "system/runtime_state.h"

#include <freertos/FreeRTOS.h>

namespace {

runtime_state::SystemSnapshot g_system_snapshot = {};
runtime_state::MotorCommand g_left_motor_command = {};
portMUX_TYPE g_runtime_state_mux = portMUX_INITIALIZER_UNLOCKED;

}  // namespace

namespace runtime_state {

void begin() {
  portENTER_CRITICAL(&g_runtime_state_mux);
  g_system_snapshot = {};
  g_left_motor_command = {};
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

MotorCommand leftMotorCommand() {
  portENTER_CRITICAL(&g_runtime_state_mux);
  const MotorCommand command = g_left_motor_command;
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
