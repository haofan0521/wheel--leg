#include "system/app_runtime.h"

#include <Arduino.h>
#include <stddef.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "modules/balance/balance_controller.h"
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
constexpr float kLeftForwardVelocitySign = -1.0f;
constexpr float kRightForwardVelocitySign = -1.0f;
uint32_t g_last_left_motor_command_sequence = 0;
uint32_t g_last_right_motor_command_sequence = 0;
bool g_balance_drive_prepared = false;

using SetOpenLoopFn = void (*)(bool);
using SetEnabledFn = void (*)(bool);
using SetTargetVelocityFn = void (*)(float);
using EmergencyStopFn = void (*)();
using SetVoltageLimitFn = void (*)(float);
using SetVelocityPidFn = void (*)(float, float, float, float);

void applyMotorCommand(const runtime_state::MotorCommand& command,
                       uint32_t& last_sequence,
                       SetOpenLoopFn set_open_loop,
                       SetEnabledFn set_enabled,
                       SetTargetVelocityFn set_target_velocity,
                       EmergencyStopFn emergency_stop,
                       SetVoltageLimitFn set_voltage_limit,
                       SetVelocityPidFn set_velocity_pid) {
  const uint32_t now_ms = millis();

  if (command.sequence != 0 && command.sequence != last_sequence) {
    last_sequence = command.sequence;

    if (command.has_open_loop) {
      set_open_loop(command.open_loop);
    }

    if (command.has_voltage_limit) {
      set_voltage_limit(command.voltage_limit);
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
                    drive::left_motor_test::setVelocityPid);
}

void applyRightMotorCommand() {
  applyMotorCommand(runtime_state::rightMotorCommand(),
                    g_last_right_motor_command_sequence,
                    drive::right_motor_test::setOpenLoop,
                    drive::right_motor_test::setEnabled,
                    drive::right_motor_test::setTargetVelocity,
                    drive::right_motor_test::emergencyStop,
                    drive::right_motor_test::setVoltageLimit,
                    drive::right_motor_test::setVelocityPid);
}

void applyBalanceCommand() {
  static uint32_t last_sequence = 0;
  const auto command = runtime_state::balanceCommand();
  if (command.sequence == 0 || command.sequence == last_sequence) return;

  last_sequence = command.sequence;
  if (command.has_tuning) {
    balance::Config config = balance::config();
    config.target_pitch_deg = command.target_pitch_deg;
    config.kp = command.kp;
    config.kd = command.kd;
    config.kv = command.kv;
    config.output_direction = command.output_direction;
    config.max_velocity = command.max_velocity;
    config.start_angle_deg = command.start_angle_deg;
    config.max_angle_deg = command.max_angle_deg;
    balance::setConfig(config);
  }

  if (command.stop) {
    balance::setEnabled(false);
    g_balance_drive_prepared = false;
    drive::left_motor_test::emergencyStop();
    drive::right_motor_test::emergencyStop();
    return;
  }

  if (command.has_enable) {
    balance::setEnabled(command.enable);
    if (!command.enable) {
      g_balance_drive_prepared = false;
      drive::left_motor_test::emergencyStop();
      drive::right_motor_test::emergencyStop();
    }
  }
}

void applyServoCommand() {
  static uint32_t last_sequence = 0;
  const auto command = runtime_state::servoCommand();
  if (command.sequence == 0 || command.sequence == last_sequence) return;

  last_sequence = command.sequence;
  if (command.has_height) {
    servo::IK_Result ik_res = {};
    // 居中位置 x=2.5, 目标高度 y=command.target_height
    if (servo::solveIK(2.5f, command.target_height, &ik_res)) {
      servo::BusServo_Move_Param params[4] = {};
      for (int i = 0; i < 4; ++i) {
        params[i].id = i + 1;
        params[i].angle = ik_res.servo_values[i];
        params[i].time = command.time_ms;
      }
      servo::moveMulti(params, 4);
    }
  }
}

void fillMotorSnapshot(runtime_state::MotorSnapshot& snapshot,
                       const drive::DriveMotorController::Status& status,
                       const runtime_state::MotorCommand& command) {
  snapshot.initialized = status.initialized;
  snapshot.foc_ready = status.foc_ready;
  snapshot.enabled = status.enabled;
  snapshot.emergency_stopped = status.emergency_stopped;
  snapshot.open_loop = status.open_loop;
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
  snapshot.voltage_q = status.voltage_q;
  snapshot.voltage_d = status.voltage_d;
  snapshot.command_age_ms = command.sequence == 0 ? 0 : millis() - command.updated_ms;
}

void fillBalanceSnapshot(runtime_state::BalanceSnapshot& snapshot,
                         const balance::Output& output,
                         const uint32_t now_ms) {
  snapshot.enabled = output.enabled;
  snapshot.active = output.active;
  snapshot.fault = output.fault;
  snapshot.target_pitch_deg = output.target_pitch_deg;
  snapshot.pitch_deg = output.pitch_deg;
  snapshot.pitch_rate_dps = output.pitch_rate_dps;
  snapshot.wheel_velocity = output.wheel_velocity;
  snapshot.output_velocity = output.output_velocity;
  snapshot.kp = output.kp;
  snapshot.kd = output.kd;
  snapshot.kv = output.kv;
  snapshot.output_direction = output.output_direction;
  snapshot.max_velocity = output.max_velocity;
  snapshot.start_angle_deg = output.start_angle_deg;
  snapshot.max_angle_deg = output.max_angle_deg;
  snapshot.last_update_ms = now_ms;
}

void controlTaskEntry(void* /*context*/) {
  drive::begin();
  encoder::begin();

  drive::setEnabled(true);

  drive::left_motor_test::init();
  drive::right_motor_test::init();
  drive::left_motor_test::setOpenLoop(false);
  drive::right_motor_test::setOpenLoop(false);

  imu::begin();
  balance::begin();
  servo::begin();

  // 上电自动控制舵机到预设高度 (例如 20cm) 进行测试
  {
    runtime_state::ServoCommand init_cmd = {};
    init_cmd.has_height = true;
    init_cmd.target_height = 20.0f;
    init_cmd.time_ms = 1000;
    init_cmd.updated_ms = millis();
    init_cmd.sequence = 1;
    runtime_state::updateServoCommand(init_cmd);
  }

  TickType_t last_wake_tick = xTaskGetTickCount();
  uint32_t control_loop_counter = 0;
  uint32_t imu_sequence = 0;

  for (;;) {
    ++control_loop_counter;

    applyLeftMotorCommand();
    applyRightMotorCommand();
    applyBalanceCommand();
    applyServoCommand();
    imu::update();

    const uint32_t now_ms = millis();
    const auto attitude = imu::getAttitude();
    const auto imu_data = imu::getData();
    const float pitch_rate_dps = imu_data.gyro_y * 180.0f / PI;
    const float left_forward_velocity = encoder::leftVelocity() * kLeftForwardVelocitySign;
    const float right_forward_velocity = encoder::rightVelocity() * kRightForwardVelocitySign;
    const float wheel_velocity = 0.5f * (left_forward_velocity + right_forward_velocity);
    runtime_state::ImuSnapshot imu_snapshot = {};
    imu_snapshot.pitch_deg = attitude.pitch;
    imu_snapshot.roll_deg = attitude.roll;
    imu_snapshot.yaw_deg = attitude.yaw;
    imu_snapshot.acc_z = imu_data.acc_z;
    imu_snapshot.last_update_ms = now_ms;
    imu_snapshot.sequence = ++imu_sequence;
    imu_snapshot.valid = true;
    runtime_state::updateImuSnapshot(imu_snapshot);

    const auto balance_output = balance::update({
        .pitch_deg = attitude.pitch,
        .pitch_rate_dps = pitch_rate_dps,
        .wheel_velocity = wheel_velocity,
        .now_ms = now_ms,
    });
    if (balance_output.active) {
      if (!g_balance_drive_prepared) {
        drive::left_motor_test::setOpenLoop(false);
        drive::right_motor_test::setOpenLoop(false);
        drive::left_motor_test::setEnabled(true);
        drive::right_motor_test::setEnabled(true);
        g_balance_drive_prepared = true;
      }
      drive::left_motor_test::setTargetVelocity(balance_output.output_velocity);
      drive::right_motor_test::setTargetVelocity(balance_output.output_velocity);
    } else if (balance_output.fault) {
      g_balance_drive_prepared = false;
      drive::left_motor_test::emergencyStop();
      drive::right_motor_test::emergencyStop();
    }

    drive::left_motor_test::update();
    drive::right_motor_test::update();

    const auto left_motor_status = drive::left_motor_test::status();
    const auto right_motor_status = drive::right_motor_test::status();
    const auto left_command = runtime_state::leftMotorCommand();
    const auto right_command = runtime_state::rightMotorCommand();

    runtime_state::ControlSnapshot snapshot = {};
    snapshot.loop_counter = control_loop_counter;
    snapshot.last_update_ms = now_ms;
    snapshot.task_period_ms = app_runtime_config::kControlTaskPeriodMs;
    snapshot.drive_fault_level = drive::readFaultLevel();
    snapshot.drive_enabled = drive::isEnabled();
    snapshot.core_id = static_cast<uint8_t>(xPortGetCoreID());
    fillBalanceSnapshot(snapshot.balance, balance_output, now_ms);
    fillMotorSnapshot(snapshot.left_motor, left_motor_status, left_command);
    fillMotorSnapshot(snapshot.right_motor, right_motor_status, right_command);
    runtime_state::updateControlSnapshot(snapshot);

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
    servo::update();

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
