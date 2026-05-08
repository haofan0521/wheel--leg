#include "system/app_runtime.h"

#include <Arduino.h>
#include <stddef.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "modules/drive/drive_module.h"
#include "modules/drive/left_motor_test.h"
#include "modules/drive/right_motor_test.h"
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
constexpr uint32_t kMotorCommandTimeoutMs = 10000;
uint32_t g_last_left_motor_command_sequence = 0;
uint32_t g_last_right_motor_command_sequence = 0;

void sendVofaFrame(const float* values, const size_t value_count) {
  for (size_t i = 0; i < value_count; ++i) {
    Serial.write(reinterpret_cast<const uint8_t*>(&values[i]), sizeof(values[i]));
  }

  const uint8_t tail[] = {0x00, 0x00, 0x80, 0x7F};
  Serial.write(tail, sizeof(tail));
}

void sendLeftMotorVofaTelemetry(const drive::left_motor_test::Status& status) {
  const float values[] = {
      status.target_velocity,
      status.measured_velocity,
      status.velocity_error,
      status.velocity_p_output,
      status.velocity_i_output,
  };
  sendVofaFrame(values, sizeof(values) / sizeof(values[0]));
}

using SetOpenLoopFn = void (*)(bool);
using SetEnabledFn = void (*)(bool);
using SetTargetVelocityFn = void (*)(float);
using EmergencyStopFn = void (*)();
using SetVoltageLimitFn = void (*)(float);
using SetVelocityPidFn = void (*)(float, float, float, float);

using SetCurrentLimitFn = void (*)(float);
using SetTorqueModeFn = void (*)(drive::DriveMotorController::TorqueMode);

void applyMotorCommand(const runtime_state::MotorCommand& command,
                       uint32_t& last_sequence,
                       SetOpenLoopFn set_open_loop,
                       SetEnabledFn set_enabled,
                       SetTargetVelocityFn set_target_velocity,
                       EmergencyStopFn emergency_stop,
                       SetVoltageLimitFn set_voltage_limit,
                       SetVelocityPidFn set_velocity_pid,
                       SetCurrentLimitFn set_current_limit,
                       SetTorqueModeFn set_torque_mode) {
  const uint32_t now_ms = millis();

  if (command.sequence != 0 && command.sequence != last_sequence) {
    last_sequence = command.sequence;

    if (command.has_open_loop) {
      set_open_loop(command.open_loop);
    }

    if (command.has_voltage_limit) {
      set_voltage_limit(command.voltage_limit);
    }

    if (command.has_current_limit) {
      set_current_limit(command.current_limit);
    }

    if (command.has_torque_mode) {
      set_torque_mode(static_cast<drive::DriveMotorController::TorqueMode>(command.torque_mode));
    }

    if (command.has_tuning) {
      set_velocity_pid(command.velocity_p,
                       command.velocity_i,
                       command.velocity_d,
                       command.velocity_lpf_tf);
    }

    if (command.stop) {
      emergency_stop();
      return;
    }

    if (command.has_enable) {
      set_enabled(command.enable);
    }
    if (command.enable && command.has_velocity_target) {
      set_target_velocity(command.target_velocity);
    }
  }

  if (command.sequence != 0 && command.enable && now_ms - command.updated_ms > kMotorCommandTimeoutMs) {
    emergency_stop();
  }
}

void applyLeftMotorCommand() {
  applyMotorCommand(runtime_state::leftMotorCommand(),
                    g_last_left_motor_command_sequence,
                    drive::left_motor_test::setOpenLoop,
                    drive::left_motor_test::setEnabled,
                    drive::left_motor_test::setTargetVelocity,
                    drive::left_motor_test::emergencyStop,
                    drive::left_motor_test::setVoltageLimit,
                    drive::left_motor_test::setVelocityPid,
                    drive::left_motor_test::setCurrentLimit,
                    drive::left_motor_test::setTorqueMode);
}

void applyRightMotorCommand() {
  applyMotorCommand(runtime_state::rightMotorCommand(),
                    g_last_right_motor_command_sequence,
                    drive::right_motor_test::setOpenLoop,
                    drive::right_motor_test::setEnabled,
                    drive::right_motor_test::setTargetVelocity,
                    drive::right_motor_test::emergencyStop,
                    drive::right_motor_test::setVoltageLimit,
                    drive::right_motor_test::setVelocityPid,
                    drive::right_motor_test::setCurrentLimit,
                    drive::right_motor_test::setTorqueMode);
}

void fillMotorSnapshot(runtime_state::MotorSnapshot& snapshot,
                       const drive::DriveMotorController::Status& status,
                       const runtime_state::MotorCommand& command) {
  snapshot.initialized = status.initialized;
  snapshot.foc_ready = status.foc_ready;
  snapshot.enabled = status.enabled;
  snapshot.emergency_stopped = status.emergency_stopped;
  snapshot.open_loop = status.open_loop;
  snapshot.torque_mode = static_cast<uint8_t>(status.torque_mode);
  snapshot.current_sense_ready = status.current_sense_ready;
  snapshot.simplefoc_current_sense_ready = status.simplefoc_current_sense_ready;
  snapshot.over_current = status.over_current;
  snapshot.target_velocity = status.target_velocity;
  snapshot.measured_velocity = status.measured_velocity;
  snapshot.shaft_angle = status.shaft_angle;
  snapshot.voltage_limit = status.voltage_limit;
  snapshot.velocity_p = status.velocity_p;
  snapshot.velocity_i = status.velocity_i;
  snapshot.velocity_d = status.velocity_d;
  snapshot.velocity_lpf_tf = status.velocity_lpf_tf;
  snapshot.velocity_error = status.velocity_error;
  snapshot.velocity_p_output = status.velocity_p_output;
  snapshot.velocity_i_output = status.velocity_i_output;
  snapshot.velocity_pid_output = status.velocity_pid_output;
  snapshot.current_limit = status.current_limit;
  snapshot.current_q = status.current_q;
  snapshot.current_d = status.current_d;
  snapshot.current_sp = status.current_sp;
  snapshot.voltage_q = status.voltage_q;
  snapshot.voltage_d = status.voltage_d;
  snapshot.phase_current_a = status.phase_current_a;
  snapshot.phase_current_b = status.phase_current_b;
  snapshot.phase_current_c = status.phase_current_c;
  snapshot.phase_voltage_b = status.phase_voltage_b;
  snapshot.phase_voltage_c = status.phase_voltage_c;
  snapshot.phase_offset_voltage_b = status.phase_offset_voltage_b;
  snapshot.phase_offset_voltage_c = status.phase_offset_voltage_c;
  snapshot.phase_voltage_delta_b = status.phase_voltage_delta_b;
  snapshot.phase_voltage_delta_c = status.phase_voltage_delta_c;
  snapshot.command_age_ms = command.sequence == 0 ? 0 : millis() - command.updated_ms;
}

const char* torqueModeName(const uint8_t mode) {
  if (mode == 1) return "dc_current";
  if (mode == 2) return "foc_current";
  return "voltage";
}

void printMotorDebugLine(const char* name, const runtime_state::MotorSnapshot& motor) {
  Serial.printf("[%s] mode=%s foc=%d en=%d oc=%d target=%.2f vel=%.2f Iabc=%.3f/%.3f/%.3f Idq=%.3f/%.3f sp=%.3f Uq=%.3f Ud=%.3f Vbc=%.3f/%.3f dVbc=%.3f/%.3f\n",
                name,
                torqueModeName(motor.torque_mode),
                motor.foc_ready,
                motor.enabled,
                motor.over_current,
                motor.target_velocity,
                motor.measured_velocity,
                motor.phase_current_a,
                motor.phase_current_b,
                motor.phase_current_c,
                motor.current_d,
                motor.current_q,
                motor.current_sp,
                motor.voltage_q,
                motor.voltage_d,
                motor.phase_voltage_b,
                motor.phase_voltage_c,
                motor.phase_voltage_delta_b,
                motor.phase_voltage_delta_c);
}

void printMotorDebugTelemetry(const runtime_state::ControlSnapshot& snapshot) {
  printMotorDebugLine("L", snapshot.left_motor);
  printMotorDebugLine("R", snapshot.right_motor);
}

void controlTaskEntry(void* /*context*/) {
  drive::begin();
  encoder::begin();

  drive::setEnabled(true);

  drive::left_motor_test::init();
  drive::right_motor_test::init();

  imu::begin();
  servo::begin();

  TickType_t last_wake_tick = xTaskGetTickCount();
  uint32_t control_loop_counter = 0;
  uint32_t imu_sequence = 0;

  for (;;) {
    ++control_loop_counter;

    applyLeftMotorCommand();
    applyRightMotorCommand();
    drive::left_motor_test::update();
    drive::right_motor_test::update();
    imu::update();

    const auto attitude = imu::getAttitude();
    const auto imu_data = imu::getData();
    runtime_state::ImuSnapshot imu_snapshot = {};
    imu_snapshot.pitch_deg = attitude.pitch;
    imu_snapshot.roll_deg = attitude.roll;
    imu_snapshot.yaw_deg = attitude.yaw;
    imu_snapshot.acc_z = imu_data.acc_z;
    imu_snapshot.last_update_ms = millis();
    imu_snapshot.sequence = ++imu_sequence;
    imu_snapshot.valid = true;
    runtime_state::updateImuSnapshot(imu_snapshot);

    const auto left_motor_status = drive::left_motor_test::status();
    const auto right_motor_status = drive::right_motor_test::status();
    const auto left_command = runtime_state::leftMotorCommand();
    const auto right_command = runtime_state::rightMotorCommand();

    runtime_state::ControlSnapshot snapshot = {};
    snapshot.loop_counter = control_loop_counter;
    snapshot.last_update_ms = millis();
    snapshot.task_period_ms = app_runtime_config::kControlTaskPeriodMs;
    snapshot.drive_fault_level = drive::readFaultLevel();
    snapshot.drive_enabled = drive::isEnabled();
    snapshot.core_id = static_cast<uint8_t>(xPortGetCoreID());
    fillMotorSnapshot(snapshot.left_motor, left_motor_status, left_command);
    fillMotorSnapshot(snapshot.right_motor, right_motor_status, right_command);
    runtime_state::updateControlSnapshot(snapshot);

    if (control_loop_counter % 100 == 0) {
      printMotorDebugTelemetry(snapshot);
    }

    if (app_runtime_config::kEnableVofaTelemetry) {
      if (control_loop_counter % app_runtime_config::kVofaTelemetryDecimation == 0) {
        sendLeftMotorVofaTelemetry(left_motor_status);
      }
    } else if (control_loop_counter % 100 == 0) {
      Serial.printf("[MotorSpeed] L target=%.3f measured=%.3f | R target=%.3f measured=%.3f\n",
                    left_motor_status.target_velocity,
                    left_motor_status.measured_velocity,
                    right_motor_status.target_velocity,
                    right_motor_status.measured_velocity);
    }

    vTaskDelayUntil(&last_wake_tick,
                    pdMS_TO_TICKS(app_runtime_config::kControlTaskPeriodMs));
  }
}

void serviceTaskEntry(void* /*context*/) {
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
