#include "system/runtime_state.h"

#include <freertos/FreeRTOS.h>

namespace {

runtime_state::SystemSnapshot g_system_snapshot = {};
portMUX_TYPE g_runtime_state_mux = portMUX_INITIALIZER_UNLOCKED;

}  // namespace

namespace runtime_state {

void begin() {
  portENTER_CRITICAL(&g_runtime_state_mux);
  g_system_snapshot = {};
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

SystemSnapshot snapshot() {
  portENTER_CRITICAL(&g_runtime_state_mux);
  const SystemSnapshot current_snapshot = g_system_snapshot;
  portEXIT_CRITICAL(&g_runtime_state_mux);
  return current_snapshot;
}

}  // namespace runtime_state
