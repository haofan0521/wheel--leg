#include "modules/drive/left_motor_test.h"

#include <Arduino.h>
#include <SimpleFOC.h>
#include <math.h>

#include "modules/drive/drive_module.h"
#include "modules/drive/drive_pins.h"
#include "modules/encoder/encoder_module.h"

namespace {

constexpr float kDefaultVoltageLimit = 6.0f;
constexpr float kMinVoltageLimit = 0.5f;
constexpr float kMaxVoltageLimit = 8.0f;
constexpr float kVelocityLimit = 20.0f;
constexpr float kPowerSupplyVoltage = 12.0f;
constexpr float kFocAlignVoltage = 6.0f;
constexpr uint32_t kOpenLoopPwmFrequencyHz = 20000;
constexpr uint8_t kOpenLoopPwmResolutionBits = 10;
constexpr uint8_t kOpenLoopPwmChannelA = 0;
constexpr uint8_t kOpenLoopPwmChannelB = 1;
constexpr uint8_t kOpenLoopPwmChannelC = 2;
constexpr float kOpenLoopElectricalVelocityScale = 11.0f;
constexpr bool kSkipAutoDirectionCalibration = false;

BLDCMotor g_motor = BLDCMotor(11);
BLDCDriver3PWM g_driver = BLDCDriver3PWM(
    drive::pins::kLeftMotorPwm.phase_a,
    drive::pins::kLeftMotorPwm.phase_b,
    drive::pins::kLeftMotorPwm.phase_c,
    NOT_SET);

float g_target_velocity = 0.0f;
float g_open_loop_voltage_limit = kDefaultVoltageLimit;
float g_open_loop_electrical_angle = 0.0f;
uint32_t g_last_open_loop_update_us = 0;
bool g_initialized = false;
bool g_foc_ready = false;
bool g_enabled = false;
bool g_emergency_stopped = true;
bool g_open_loop = true;
bool g_sensor_available = false;
bool g_foc_attempted = false;

float clampVoltageLimit(const float limit) {
  if (limit < kMinVoltageLimit) return kMinVoltageLimit;
  if (limit > kMaxVoltageLimit) return kMaxVoltageLimit;
  return limit;
}

void configureOpenLoopPwm() {
  ledcSetup(kOpenLoopPwmChannelA, kOpenLoopPwmFrequencyHz, kOpenLoopPwmResolutionBits);
  ledcSetup(kOpenLoopPwmChannelB, kOpenLoopPwmFrequencyHz, kOpenLoopPwmResolutionBits);
  ledcSetup(kOpenLoopPwmChannelC, kOpenLoopPwmFrequencyHz, kOpenLoopPwmResolutionBits);
  ledcAttachPin(drive::pins::kLeftMotorPwm.phase_a, kOpenLoopPwmChannelA);
  ledcAttachPin(drive::pins::kLeftMotorPwm.phase_b, kOpenLoopPwmChannelB);
  ledcAttachPin(drive::pins::kLeftMotorPwm.phase_c, kOpenLoopPwmChannelC);
}

void writeOpenLoopDuty(const float phase_a, const float phase_b, const float phase_c) {
  const uint32_t max_duty = (1UL << kOpenLoopPwmResolutionBits) - 1UL;
  ledcWrite(kOpenLoopPwmChannelA, static_cast<uint32_t>(constrain(phase_a, 0.0f, 1.0f) * max_duty));
  ledcWrite(kOpenLoopPwmChannelB, static_cast<uint32_t>(constrain(phase_b, 0.0f, 1.0f) * max_duty));
  ledcWrite(kOpenLoopPwmChannelC, static_cast<uint32_t>(constrain(phase_c, 0.0f, 1.0f) * max_duty));
}

void centerOpenLoopPwm() {
  writeOpenLoopDuty(0.5f, 0.5f, 0.5f);
}

void updateOpenLoopPwm() {
  const uint32_t now_us = micros();
  if (g_last_open_loop_update_us == 0) {
    g_last_open_loop_update_us = now_us;
  }
  const float dt = static_cast<float>(now_us - g_last_open_loop_update_us) * 1e-6f;
  g_last_open_loop_update_us = now_us;

  g_open_loop_electrical_angle += g_target_velocity * kOpenLoopElectricalVelocityScale * dt;
  while (g_open_loop_electrical_angle >= _2PI) g_open_loop_electrical_angle -= _2PI;
  while (g_open_loop_electrical_angle < 0.0f) g_open_loop_electrical_angle += _2PI;

  const float modulation = constrain(g_open_loop_voltage_limit / kPowerSupplyVoltage, 0.0f, 0.45f);
  writeOpenLoopDuty(
      0.5f + modulation * sinf(g_open_loop_electrical_angle),
      0.5f + modulation * sinf(g_open_loop_electrical_angle - _2PI / 3.0f),
      0.5f + modulation * sinf(g_open_loop_electrical_angle + _2PI / 3.0f));
}

void configureMotorForCurrentMode() {
  g_motor.controller = g_open_loop ? MotionControlType::velocity_openloop : MotionControlType::velocity;
}

bool initializeClosedLoopFoc() {
  if (!g_sensor_available) {
    return false;
  }

  if (g_foc_attempted) {
    return g_foc_ready;
  }

  g_foc_attempted = true;
  g_open_loop = false;
  configureMotorForCurrentMode();
  g_driver.init();
  g_motor.init();
  const int foc_result = g_motor.initFOC();
  g_foc_ready = foc_result != 0;
  return g_foc_ready;
}

void disableMotorOutput() {
  g_target_velocity = 0.0f;
  g_enabled = false;
  if (!g_open_loop) {
    g_motor.disable();
  }
  g_last_open_loop_update_us = 0;
  centerOpenLoopPwm();
}

}  // namespace

namespace drive {
namespace left_motor_test {

void init() {
  g_driver.voltage_power_supply = kPowerSupplyVoltage;
  g_driver.voltage_limit = kMaxVoltageLimit;
  configureOpenLoopPwm();
  centerOpenLoopPwm();

  Sensor* sensor = encoder::leftSensor();
  g_sensor_available = sensor != nullptr;

  g_motor.linkDriver(&g_driver);
  if (g_sensor_available) {
    g_motor.linkSensor(sensor);
  }
  g_motor.useMonitoring(Serial);

  g_motor.voltage_limit = kDefaultVoltageLimit;
  g_motor.voltage_sensor_align = kFocAlignVoltage;
  g_motor.velocity_limit = kVelocityLimit;
  g_motor.torque_controller = TorqueControlType::voltage;
  g_motor.PID_velocity.P = 0.2f;
  g_motor.PID_velocity.I = 2.0f;
  g_motor.PID_velocity.D = 0.0f;
  g_motor.LPF_velocity.Tf = 0.02f;
  if (kSkipAutoDirectionCalibration) {
    g_motor.sensor_direction = Direction::CW;
  }
  configureMotorForCurrentMode();

  g_initialized = true;
  g_foc_ready = false;
  g_foc_attempted = false;
  g_open_loop = true;
  centerOpenLoopPwm();
  configureMotorForCurrentMode();
  disableMotorOutput();
  g_emergency_stopped = true;
}

void update() {
  if (!g_initialized) return;

  if (g_open_loop) {
    if (!g_enabled || g_emergency_stopped) {
      centerOpenLoopPwm();
      return;
    }

    updateOpenLoopPwm();
    return;
  }

  configureMotorForCurrentMode();

  if (g_foc_ready) {
    g_motor.loopFOC();
  }

  if (!g_enabled || g_emergency_stopped) {
    g_motor.move(0.0f);
    return;
  }

  if (g_foc_ready) {
    g_motor.move(g_target_velocity);
  }
}

void processSerial() {}

void setOpenLoop(const bool open_loop) {
  if (!g_initialized) return;

  if (!open_loop && !g_foc_ready && !initializeClosedLoopFoc()) {
    g_open_loop = true;
    configureOpenLoopPwm();
    centerOpenLoopPwm();
    configureMotorForCurrentMode();
    g_target_velocity = 0.0f;
    g_emergency_stopped = true;
    g_enabled = false;
    g_motor.disable();
    return;
  }

  g_open_loop = open_loop;
  if (g_open_loop) {
    configureOpenLoopPwm();
    centerOpenLoopPwm();
  }
  configureMotorForCurrentMode();
  g_target_velocity = 0.0f;
  g_emergency_stopped = true;
  g_enabled = false;
  if (!g_open_loop) {
    g_motor.disable();
  }
}

void setEnabled(const bool enabled) {
  if (!g_initialized) return;
  if (!g_open_loop && !g_foc_ready) return;

  if (enabled) {
    g_emergency_stopped = false;
    g_enabled = true;
    if (!g_open_loop) {
      g_motor.enable();
    }
  } else {
    disableMotorOutput();
    g_emergency_stopped = true;
  }
}

void setTargetVelocity(const float velocity) {
  if (!g_initialized) return;
  if (!g_open_loop && !g_foc_ready) return;

  g_target_velocity = constrain(velocity, -kVelocityLimit, kVelocityLimit);
  g_emergency_stopped = false;
  if (!g_enabled) {
    g_enabled = true;
    if (!g_open_loop) {
      g_motor.enable();
    }
  }
}

void emergencyStop() {
  if (!g_initialized) {
    g_target_velocity = 0.0f;
    g_enabled = false;
    g_emergency_stopped = true;
    return;
  }

  disableMotorOutput();
  g_emergency_stopped = true;
}

void setVoltageLimit(const float limit) {
  const float clamped_limit = clampVoltageLimit(limit);
  g_open_loop_voltage_limit = clamped_limit;
  g_motor.voltage_limit = clamped_limit;
  g_driver.voltage_limit = clamped_limit;
}

float getAngle() {
  return encoder::leftAngle();
}

Status status() {
  Status current = {};
  current.initialized = g_initialized;
  current.foc_ready = g_foc_ready;
  current.enabled = g_enabled;
  current.emergency_stopped = g_emergency_stopped;
  current.open_loop = g_open_loop;
  current.target_velocity = g_target_velocity;
  current.measured_velocity = encoder::leftVelocity();
  current.shaft_angle = encoder::leftAngle();
  current.voltage_limit = g_open_loop ? g_open_loop_voltage_limit : g_motor.voltage_limit;
  return current;
}

}  // namespace left_motor_test
}  // namespace drive
