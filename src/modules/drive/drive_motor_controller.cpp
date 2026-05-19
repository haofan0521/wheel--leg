#include "modules/drive/drive_motor_controller.h"

#include <Arduino.h>
#include <math.h>

namespace {

constexpr float kDefaultVoltageLimit = 6.0f;
constexpr float kMinVoltageLimit = 0.5f;
constexpr float kMaxVoltageLimit = 8.0f;
constexpr float kVelocityLimit = 20.0f;
constexpr float kPowerSupplyVoltage = 22.7f;
constexpr float kFocAlignVoltage = 6.0f;
constexpr uint32_t kOpenLoopPwmFrequencyHz = 20000;
constexpr uint8_t kOpenLoopPwmResolutionBits = 10;
constexpr float kOpenLoopElectricalVelocityScale = 11.0f;
constexpr bool kSkipAutoDirectionCalibration = false;

}  // namespace

namespace drive {

DriveMotorController::DriveMotorController(const Config& config)
    : config_(config),
      motor_(config.pole_pairs),
      driver_(config.pwm_pins.phase_a, config.pwm_pins.phase_b, config.pwm_pins.phase_c, NOT_SET),
      target_velocity_(0.0f),
      open_loop_voltage_limit_(kDefaultVoltageLimit),
      velocity_p_(0.2f),
      velocity_i_(2.0f),
      velocity_d_(0.0f),
      velocity_lpf_tf_(0.02f),
      open_loop_electrical_angle_(0.0f),
      last_open_loop_update_us_(0),
      initialized_(false),
      foc_ready_(false),
      enabled_(false),
      emergency_stopped_(true),
      open_loop_(true),
      sensor_available_(false),
      foc_attempted_(false),
      velocity_error_(0.0f),
      velocity_p_output_(0.0f),
      velocity_i_output_(0.0f),
      velocity_pid_output_(0.0f),
      velocity_error_prev_(0.0f),
      velocity_telemetry_update_us_(0) {}

float DriveMotorController::clampVoltageLimit(const float limit) const {
  if (limit < kMinVoltageLimit) return kMinVoltageLimit;
  if (limit > kMaxVoltageLimit) return kMaxVoltageLimit;
  return limit;
}

void DriveMotorController::configureOpenLoopPwm() {
  ledcSetup(config_.pwm_channel_a, kOpenLoopPwmFrequencyHz, kOpenLoopPwmResolutionBits);
  ledcSetup(config_.pwm_channel_b, kOpenLoopPwmFrequencyHz, kOpenLoopPwmResolutionBits);
  ledcSetup(config_.pwm_channel_c, kOpenLoopPwmFrequencyHz, kOpenLoopPwmResolutionBits);
  ledcAttachPin(config_.pwm_pins.phase_a, config_.pwm_channel_a);
  ledcAttachPin(config_.pwm_pins.phase_b, config_.pwm_channel_b);
  ledcAttachPin(config_.pwm_pins.phase_c, config_.pwm_channel_c);
}

void DriveMotorController::writeOpenLoopDuty(const float phase_a, const float phase_b, const float phase_c) {
  const uint32_t max_duty = (1UL << kOpenLoopPwmResolutionBits) - 1UL;
  ledcWrite(config_.pwm_channel_a, static_cast<uint32_t>(constrain(phase_a, 0.0f, 1.0f) * max_duty));
  ledcWrite(config_.pwm_channel_b, static_cast<uint32_t>(constrain(phase_b, 0.0f, 1.0f) * max_duty));
  ledcWrite(config_.pwm_channel_c, static_cast<uint32_t>(constrain(phase_c, 0.0f, 1.0f) * max_duty));
}

void DriveMotorController::centerOpenLoopPwm() {
  writeOpenLoopDuty(0.5f, 0.5f, 0.5f);
}

void DriveMotorController::updateOpenLoopPwm() {
  const uint32_t now_us = micros();
  if (last_open_loop_update_us_ == 0) {
    last_open_loop_update_us_ = now_us;
  }
  const float dt = static_cast<float>(now_us - last_open_loop_update_us_) * 1e-6f;
  last_open_loop_update_us_ = now_us;

  open_loop_electrical_angle_ += target_velocity_ * config_.velocity_direction * kOpenLoopElectricalVelocityScale * dt;
  while (open_loop_electrical_angle_ >= _2PI) open_loop_electrical_angle_ -= _2PI;
  while (open_loop_electrical_angle_ < 0.0f) open_loop_electrical_angle_ += _2PI;

  const float modulation = constrain(open_loop_voltage_limit_ / kPowerSupplyVoltage, 0.0f, 0.45f);
  writeOpenLoopDuty(0.5f + modulation * sinf(open_loop_electrical_angle_),
                    0.5f + modulation * sinf(open_loop_electrical_angle_ - _2PI / 3.0f),
                    0.5f + modulation * sinf(open_loop_electrical_angle_ + _2PI / 3.0f));
}

void DriveMotorController::applyVelocityTuning() {
  motor_.PID_velocity.P = velocity_p_;
  motor_.PID_velocity.I = velocity_i_;
  motor_.PID_velocity.D = velocity_d_;
  motor_.LPF_velocity.Tf = velocity_lpf_tf_;
}

void DriveMotorController::resetVelocityTelemetry() {
  velocity_error_ = 0.0f;
  velocity_p_output_ = 0.0f;
  velocity_i_output_ = 0.0f;
  velocity_pid_output_ = 0.0f;
  velocity_error_prev_ = 0.0f;
  velocity_telemetry_update_us_ = 0;
}

void DriveMotorController::updateVelocityTelemetry(const float measured_velocity) {
  const uint32_t now_us = micros();
  float dt = 1e-3f;
  if (velocity_telemetry_update_us_ != 0) {
    dt = static_cast<float>(now_us - velocity_telemetry_update_us_) * 1e-6f;
    if (dt <= 0.0f || dt > 0.5f) {
      dt = 1e-3f;
    }
  }
  velocity_telemetry_update_us_ = now_us;

  velocity_error_ = target_velocity_ - measured_velocity;
  velocity_p_output_ = velocity_p_ * velocity_error_;
  velocity_i_output_ += velocity_i_ * dt * 0.5f * (velocity_error_ + velocity_error_prev_);
  velocity_i_output_ = constrain(velocity_i_output_, -motor_.PID_velocity.limit, motor_.PID_velocity.limit);
  velocity_pid_output_ = constrain(velocity_p_output_ + velocity_i_output_,
                                   -motor_.PID_velocity.limit,
                                   motor_.PID_velocity.limit);
  velocity_error_prev_ = velocity_error_;
}

void DriveMotorController::configureMotorForCurrentMode() {
  motor_.controller = open_loop_ ? MotionControlType::velocity_openloop : MotionControlType::velocity;
  motor_.torque_controller = TorqueControlType::voltage;
}

bool DriveMotorController::initializeClosedLoopFoc() {
  if (!sensor_available_) {
    return false;
  }

  if (foc_attempted_) {
    return foc_ready_;
  }

  foc_attempted_ = true;
  open_loop_ = false;
  configureMotorForCurrentMode();
  driver_.init();
  motor_.init();
  const int foc_result = motor_.initFOC();
  foc_ready_ = foc_result != 0;
  return foc_ready_;
}

void DriveMotorController::disableMotorOutput() {
  target_velocity_ = 0.0f;
  enabled_ = false;
  resetVelocityTelemetry();
  if (foc_ready_) {
    motor_.move(0.0f);
    motor_.disable();
  } else {
    last_open_loop_update_us_ = 0;
    centerOpenLoopPwm();
  }
}

void DriveMotorController::init() {
  driver_.voltage_power_supply = kPowerSupplyVoltage;
  driver_.voltage_limit = kMaxVoltageLimit;
  configureOpenLoopPwm();
  centerOpenLoopPwm();

  Sensor* sensor = config_.sensor_provider ? config_.sensor_provider() : nullptr;
  sensor_available_ = sensor != nullptr;

  motor_.linkDriver(&driver_);
  if (sensor_available_) {
    motor_.linkSensor(sensor);
  }

  motor_.voltage_limit = kDefaultVoltageLimit;
  motor_.voltage_sensor_align = kFocAlignVoltage;
  motor_.velocity_limit = kVelocityLimit;
  motor_.torque_controller = TorqueControlType::voltage;
  applyVelocityTuning();
  if (kSkipAutoDirectionCalibration) {
    motor_.sensor_direction = Direction::CW;
  }
  configureMotorForCurrentMode();

  initialized_ = true;
  foc_ready_ = false;
  foc_attempted_ = false;
  open_loop_ = true;
  centerOpenLoopPwm();
  configureMotorForCurrentMode();
  disableMotorOutput();
  emergency_stopped_ = true;
}

void DriveMotorController::update() {
  if (!initialized_) return;

  if (open_loop_) {
    if (!enabled_ || emergency_stopped_) {
      centerOpenLoopPwm();
      return;
    }

    const float measured_velocity = config_.read_velocity ? config_.read_velocity() : 0.0f;
    updateVelocityTelemetry(measured_velocity);
    updateOpenLoopPwm();
    return;
  }

  configureMotorForCurrentMode();

  if (foc_ready_) {
    motor_.loopFOC();
  }

  if (!enabled_ || emergency_stopped_) {
    motor_.move(0.0f);
    return;
  }

  const float measured_velocity = config_.read_velocity ? config_.read_velocity() : 0.0f;
  updateVelocityTelemetry(measured_velocity);

  if (foc_ready_) {
    motor_.move(target_velocity_ * config_.velocity_direction);
  }
}

void DriveMotorController::processSerial() {}

void DriveMotorController::setOpenLoop(const bool open_loop) {
  if (!initialized_) return;

  if (!open_loop && !foc_ready_ && !initializeClosedLoopFoc()) {
    open_loop_ = true;
    configureOpenLoopPwm();
    centerOpenLoopPwm();
    configureMotorForCurrentMode();
    target_velocity_ = 0.0f;
    emergency_stopped_ = true;
    enabled_ = false;
    motor_.disable();
    return;
  }

  open_loop_ = open_loop;
  if (open_loop_) {
    configureOpenLoopPwm();
    centerOpenLoopPwm();
  }
  configureMotorForCurrentMode();
  target_velocity_ = 0.0f;
  emergency_stopped_ = true;
  enabled_ = false;
  if (foc_ready_) {
    motor_.move(0.0f);
    motor_.disable();
  }
}

void DriveMotorController::setEnabled(const bool enabled) {
  if (!initialized_) return;
  if (!open_loop_ && !foc_ready_) return;

  if (enabled) {
    emergency_stopped_ = false;
    enabled_ = true;
    if (!open_loop_) {
      motor_.enable();
    }
  } else {
    disableMotorOutput();
    emergency_stopped_ = true;
  }
}

void DriveMotorController::setTargetVelocity(const float velocity) {
  if (!initialized_) return;
  if (!open_loop_ && !foc_ready_) return;

  target_velocity_ = constrain(velocity, -kVelocityLimit, kVelocityLimit);
  emergency_stopped_ = false;
  if (!enabled_) {
    enabled_ = true;
    if (!open_loop_) {
      motor_.enable();
    }
  }
}

void DriveMotorController::emergencyStop() {
  if (!initialized_) {
    target_velocity_ = 0.0f;
    enabled_ = false;
    emergency_stopped_ = true;
    return;
  }

  disableMotorOutput();
  emergency_stopped_ = true;
}

void DriveMotorController::setVoltageLimit(const float limit) {
  const float clamped_limit = clampVoltageLimit(limit);
  open_loop_voltage_limit_ = clamped_limit;
  motor_.voltage_limit = clamped_limit;
  driver_.voltage_limit = clamped_limit;
}

void DriveMotorController::setVelocityPid(const float p, const float i, const float d, const float lpf_tf) {
  velocity_p_ = constrain(p, 0.0f, 2.0f);
  velocity_i_ = constrain(i, 0.0f, 20.0f);
  velocity_d_ = constrain(d, 0.0f, 1.0f);
  velocity_lpf_tf_ = constrain(lpf_tf, 0.001f, 0.5f);
  applyVelocityTuning();
  resetVelocityTelemetry();
  motor_.PID_velocity.reset();
}

float DriveMotorController::getAngle() {
  return config_.read_angle ? config_.read_angle() : 0.0f;
}

DriveMotorController::Status DriveMotorController::status() {
  Status current = {};
  current.initialized = initialized_;
  current.foc_ready = foc_ready_;
  current.enabled = enabled_;
  current.emergency_stopped = emergency_stopped_;
  current.open_loop = open_loop_;
  current.target_velocity = target_velocity_;
  current.measured_velocity = config_.read_velocity ? config_.read_velocity() : 0.0f;
  current.shaft_angle = config_.read_angle ? config_.read_angle() : 0.0f;
  current.voltage_limit = open_loop_ ? open_loop_voltage_limit_ : motor_.voltage_limit;
  current.velocity_p = velocity_p_;
  current.velocity_i = velocity_i_;
  current.velocity_d = velocity_d_;
  current.velocity_lpf_tf = velocity_lpf_tf_;
  current.velocity_error = velocity_error_;
  current.velocity_p_output = velocity_p_output_;
  current.velocity_i_output = velocity_i_output_;
  current.velocity_pid_output = velocity_pid_output_;
  if (open_loop_) {
    current.voltage_q = open_loop_voltage_limit_;
    current.voltage_d = 0.0f;
  } else {
    current.voltage_q = motor_.voltage.q;
    current.voltage_d = motor_.voltage.d;
  }
  return current;
}

}  // namespace drive
