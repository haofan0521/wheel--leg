#include "system/app_runtime.h"

#include <Arduino.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "modules/drive/drive_module.h"
#include "modules/drive/left_motor_test.h"
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
constexpr uint32_t kMotorCommandTimeoutMs = 30000;
uint32_t g_last_left_motor_command_sequence = 0;

void applyLeftMotorCommand() {
  const auto command = runtime_state::leftMotorCommand();
  const uint32_t now_ms = millis();

  if (command.sequence != 0 && command.sequence != g_last_left_motor_command_sequence) {
    g_last_left_motor_command_sequence = command.sequence;

    if (command.has_open_loop) {
      drive::left_motor_test::setOpenLoop(command.open_loop);
    }

    if (command.has_voltage_limit) {
      drive::left_motor_test::setVoltageLimit(command.voltage_limit);
    }

    if (command.stop) {
      drive::left_motor_test::emergencyStop();
      return;
    }

    if (command.has_enable) {
      drive::left_motor_test::setEnabled(command.enable);
    }
    if (command.enable && command.has_velocity_target) {
      drive::left_motor_test::setTargetVelocity(command.target_velocity);
    }
  }

  if (command.sequence != 0 && command.enable && now_ms - command.updated_ms > kMotorCommandTimeoutMs) {
    drive::left_motor_test::emergencyStop();
  }
}

void controlTaskEntry(void* /*context*/) {
  // 初始化电驱 GPIO
  drive::begin();
  
  // 先初始化编码器，因为它需要配置 SPI 总线，而电机初始化可能需要传感器
  encoder::begin();

  // 必须先开启电驱总使能（EN），电机驱动板才能开始响应 PWM
  // 对于某些驱动芯片，initFOC() 需要驱动器处于 Ready 状态
  drive::setEnabled(true);

  // 再初始化当前活动电机控制器。
  drive::left_motor_test::init();

  imu::begin();
  servo::begin();

  TickType_t last_wake_tick = xTaskGetTickCount();
  uint32_t control_loop_counter = 0;
  uint32_t imu_sequence = 0;

  for (;;) {
    ++control_loop_counter;

    applyLeftMotorCommand();
    drive::left_motor_test::update();

    const auto left_motor_status = drive::left_motor_test::status();

    runtime_state::ControlSnapshot snapshot = {};
    snapshot.loop_counter = control_loop_counter;
    snapshot.last_update_ms = millis();
    snapshot.task_period_ms = app_runtime_config::kControlTaskPeriodMs;
    snapshot.drive_fault_level = drive::readFaultLevel();
    snapshot.drive_enabled = drive::isEnabled();
    snapshot.core_id = static_cast<uint8_t>(xPortGetCoreID());
    snapshot.left_motor.initialized = left_motor_status.initialized;
    snapshot.left_motor.foc_ready = left_motor_status.foc_ready;
    snapshot.left_motor.enabled = left_motor_status.enabled;
    snapshot.left_motor.emergency_stopped = left_motor_status.emergency_stopped;
    snapshot.left_motor.open_loop = left_motor_status.open_loop;
    snapshot.left_motor.target_velocity = left_motor_status.target_velocity;
    snapshot.left_motor.measured_velocity = left_motor_status.measured_velocity;
    snapshot.left_motor.shaft_angle = left_motor_status.shaft_angle;
    snapshot.left_motor.voltage_limit = left_motor_status.voltage_limit;
    const auto current_command = runtime_state::leftMotorCommand();
    snapshot.left_motor.command_age_ms = current_command.sequence == 0 ? 0 : millis() - current_command.updated_ms;
    runtime_state::updateControlSnapshot(snapshot);

    if (control_loop_counter % 100 == 0) {
      Serial.printf("[MotorSpeed] target=%.3f rad/s measured=%.3f rad/s\n",
                    left_motor_status.target_velocity,
                    left_motor_status.measured_velocity);
    }

    // 当前先保留任务骨架与状态发布。
    // 后续可在这里接入编码器采样、IMU 读取和 FOC 控制循环。
    vTaskDelayUntil(&last_wake_tick,
                    pdMS_TO_TICKS(app_runtime_config::kControlTaskPeriodMs));
  }
}

void serviceTaskEntry(void* /*context*/) {
  // Wi-Fi 与网页调试只在服务任务中运行，避免干扰控制任务。
  WiFiDebugServer::instance().begin();

  uint32_t service_loop_counter = 0;
  for (;;) {
    ++service_loop_counter;
    WiFiDebugServer::instance().loop();

    // 解析串口输入的命令控制左侧电机
    // drive::left_motor_test::processSerial();

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
    return;
  }

  g_runtime_started = true;
}

bool started() {
  return g_runtime_started;
}

}  // namespace app_runtime
