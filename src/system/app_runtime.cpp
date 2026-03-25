#include "system/app_runtime.h"

#include <Arduino.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "modules/drive/drive_module.h"
#include "modules/encoder/encoder_module.h"
#include "modules/imu/imu_module.h"
#include "modules/servo/servo_module.h"
#include "system/app_runtime_config.h"
#include "system/runtime_state.h"
#include "system/wifi_debug_server.h"

namespace {

TaskHandle_t g_control_task_handle = nullptr;
TaskHandle_t g_service_task_handle = nullptr;
bool g_runtime_started = false;

void controlTaskEntry(void* /*context*/) {
  Serial.println("[Runtime] Control task starting");

  // 将所有与实时控制链路直接相关的模块放在控制任务中初始化。
  drive::begin();
  encoder::begin();
  imu::begin();
  servo::begin();

  TickType_t last_wake_tick = xTaskGetTickCount();
  uint32_t control_loop_counter = 0;

  for (;;) {
    ++control_loop_counter;

    runtime_state::ControlSnapshot snapshot = {};
    snapshot.loop_counter = control_loop_counter;
    snapshot.last_update_ms = millis();
    snapshot.task_period_ms = app_runtime_config::kControlTaskPeriodMs;
    snapshot.drive_fault_level = drive::readFaultLevel();
    snapshot.drive_enabled = drive::isEnabled();
    snapshot.core_id = static_cast<uint8_t>(xPortGetCoreID());
    runtime_state::updateControlSnapshot(snapshot);

    // 当前先保留任务骨架与状态发布。
    // 后续可在这里接入编码器采样、IMU 读取和 FOC 控制循环。
    vTaskDelayUntil(&last_wake_tick,
                    pdMS_TO_TICKS(app_runtime_config::kControlTaskPeriodMs));
  }
}

void serviceTaskEntry(void* /*context*/) {
  Serial.println("[Runtime] Service task starting");

  // Wi-Fi 与网页调试只在服务任务中运行，避免干扰控制任务。
  WiFiDebugServer::instance().begin();

  uint32_t service_loop_counter = 0;
  for (;;) {
    ++service_loop_counter;
    WiFiDebugServer::instance().loop();

    runtime_state::ServiceSnapshot snapshot = {};
    snapshot.loop_counter = service_loop_counter;
    snapshot.last_update_ms = millis();
    snapshot.task_period_ms = app_runtime_config::kServiceTaskPeriodMs;
    snapshot.wifi_server_started = true;
    snapshot.core_id = static_cast<uint8_t>(xPortGetCoreID());
    runtime_state::updateServiceSnapshot(snapshot);

    vTaskDelay(pdMS_TO_TICKS(app_runtime_config::kServiceTaskPeriodMs));
  }
}

}  // namespace

namespace app_runtime {

void begin() {
  if (g_runtime_started) {
    return;
  }

  runtime_state::begin();

  // 先创建控制任务，再创建服务任务，保证控制链路优先就绪。
  const BaseType_t control_result = xTaskCreatePinnedToCore(
      controlTaskEntry,
      "control_task",
      app_runtime_config::kControlTaskStackBytes,
      nullptr,
      app_runtime_config::kControlTaskPriority,
      &g_control_task_handle,
      app_runtime_config::kControlCore);

  const BaseType_t service_result = xTaskCreatePinnedToCore(
      serviceTaskEntry,
      "service_task",
      app_runtime_config::kServiceTaskStackBytes,
      nullptr,
      app_runtime_config::kServiceTaskPriority,
      &g_service_task_handle,
      app_runtime_config::kServiceCore);

  if (control_result != pdPASS || service_result != pdPASS) {
    Serial.println("[Runtime] Failed to create tasks");
    return;
  }

  g_runtime_started = true;
  Serial.println("[Runtime] Dual-core task framework started");
}

bool started() {
  return g_runtime_started;
}

}  // namespace app_runtime
