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
constexpr float kCurrentSenseShuntOhms = 0.01f;
constexpr float kIna240A2Gain = 50.0f;
constexpr float kDefaultCurrentLimitA = 0.8f;
constexpr float kOverCurrentLimitA = 2.5f;
constexpr uint16_t kCurrentOffsetSampleCount = 256;
constexpr uint16_t kCurrentOffsetSampleDelayUs = 100;

}  // namespace

namespace drive {

DriveMotorController::DriveMotorController(const Config& config)
    : config_(config),
      motor_(config.pole_pairs),
      driver_(config.pwm_pins.phase_a, config.pwm_pins.phase_b, config.pwm_pins.phase_c, NOT_SET),
      current_sense_(kCurrentSenseShuntOhms,
                     kIna240A2Gain,
                     NOT_SET,
                     config.feedback_pins.channel_b,
                     config.feedback_pins.channel_c),
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
      torque_mode_(TorqueMode::kVoltage),
      current_sense_ready_(false),
      simplefoc_current_sense_ready_(false),
      over_current_(false),
      current_limit_(kDefaultCurrentLimitA),
      velocity_error_(0.0f),
      velocity_p_output_(0.0f),
      velocity_i_output_(0.0f),
      velocity_pid_output_(0.0f),
      velocity_error_prev_(0.0f),
      velocity_telemetry_update_us_(0),
      phase_current_a_(0.0f),
      phase_current_b_(0.0f),
      phase_current_c_(0.0f),
      phase_voltage_b_(0.0f),
      phase_voltage_c_(0.0f),
      phase_offset_voltage_b_(0.0f),
      phase_offset_voltage_c_(0.0f),
      phase_voltage_delta_b_(0.0f),
      phase_voltage_delta_c_(0.0f) {}

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
  motor_.torque_controller = simpleFocTorqueMode();
}

TorqueControlType DriveMotorController::simpleFocTorqueMode() const {
  if (torque_mode_ == TorqueMode::kDcCurrent) return TorqueControlType::dc_current;
  if (torque_mode_ == TorqueMode::kFocCurrent) return TorqueControlType::foc_current;
  return TorqueControlType::voltage;
}

const char* DriveMotorController::torqueModeName() const {
  if (torque_mode_ == TorqueMode::kDcCurrent) return "dc_current";
  if (torque_mode_ == TorqueMode::kFocCurrent) return "foc_current";
  return "voltage";
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

void DriveMotorController::configureCurrentSensePins() {
  pinMode(config_.feedback_pins.channel_b, INPUT);
  pinMode(config_.feedback_pins.channel_c, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(config_.feedback_pins.channel_b, ADC_11db);
  analogSetPinAttenuation(config_.feedback_pins.channel_c, ADC_11db);
}

float DriveMotorController::readAdcVoltage(const uint8_t pin) const {
#if defined(ARDUINO_ARCH_ESP32)
  return static_cast<float>(analogReadMilliVolts(pin)) * 0.001f;
#else
  return static_cast<float>(analogRead(pin)) * (3.3f / 4095.0f);
#endif
}

void DriveMotorController::calibrateCurrentOffsets() {
  float sum_b = 0.0f;
  float sum_c = 0.0f;
  for (uint16_t i = 0; i < kCurrentOffsetSampleCount; ++i) {
    sum_b += readAdcVoltage(config_.feedback_pins.channel_b);
    sum_c += readAdcVoltage(config_.feedback_pins.channel_c);
    delayMicroseconds(kCurrentOffsetSampleDelayUs);
  }
  phase_offset_voltage_b_ = sum_b / kCurrentOffsetSampleCount;
  phase_offset_voltage_c_ = sum_c / kCurrentOffsetSampleCount;
  current_sense_ready_ = true;
}

void DriveMotorController::updateCurrentSample() {
  if (!current_sense_ready_) return;

  phase_voltage_b_ = readAdcVoltage(config_.feedback_pins.channel_b);
  phase_voltage_c_ = readAdcVoltage(config_.feedback_pins.channel_c);
  phase_voltage_delta_b_ = phase_voltage_b_ - phase_offset_voltage_b_;
  phase_voltage_delta_c_ = phase_voltage_c_ - phase_offset_voltage_c_;
  phase_current_b_ = phase_voltage_delta_b_ / (kCurrentSenseShuntOhms * kIna240A2Gain);
  phase_current_c_ = phase_voltage_delta_c_ / (kCurrentSenseShuntOhms * kIna240A2Gain);
  phase_current_a_ = -(phase_current_b_ + phase_current_c_);

  const float electrical_angle = (!open_loop_ && foc_ready_) ? motor_.electrical_angle : open_loop_electrical_angle_;
  const float i_alpha = phase_current_a_;
  const float i_beta = _1_SQRT3 * phase_current_a_ + _2_SQRT3 * phase_current_b_;
  float sin_angle;
  float cos_angle;
  _sincos(electrical_angle, &sin_angle, &cos_angle);
  motor_.current.d = i_alpha * cos_angle + i_beta * sin_angle;
  motor_.current.q = i_beta * cos_angle - i_alpha * sin_angle;

  if (fabsf(phase_current_a_) > kOverCurrentLimitA ||
      fabsf(phase_current_b_) > kOverCurrentLimitA ||
      fabsf(phase_current_c_) > kOverCurrentLimitA) {
    over_current_ = true;
    disableMotorOutput();
    emergency_stopped_ = true;
  }
}

void DriveMotorController::init() {
  driver_.voltage_power_supply = kPowerSupplyVoltage;
  driver_.voltage_limit = kMaxVoltageLimit;
  configureOpenLoopPwm();
  centerOpenLoopPwm();
  configureCurrentSensePins();
  calibrateCurrentOffsets();
  updateCurrentSample();

  Sensor* sensor = config_.sensor_provider ? config_.sensor_provider() : nullptr;
  sensor_available_ = sensor != nullptr;

  motor_.linkDriver(&driver_);
  if (sensor_available_) {
    motor_.linkSensor(sensor);
  }
  current_sense_.linkDriver(&driver_);
  simplefoc_current_sense_ready_ = current_sense_.init() != 0;
  if (simplefoc_current_sense_ready_) {
    motor_.linkCurrentSense(&current_sense_);
  }

  motor_.voltage_limit = kDefaultVoltageLimit;
  motor_.voltage_sensor_align = kFocAlignVoltage;
  motor_.velocity_limit = kVelocityLimit;
  motor_.current_limit = current_limit_;
  motor_.PID_current_q.P = 1.0f;
  motor_.PID_current_q.I = 50.0f;
  motor_.PID_current_q.D = 0.0f;
  motor_.PID_current_d.P = 1.0f;
  motor_.PID_current_d.I = 50.0f;
  motor_.PID_current_d.D = 0.0f;
  motor_.LPF_current_q.Tf = 0.005f;
  motor_.LPF_current_d.Tf = 0.005f;
  motor_.torque_controller = simpleFocTorqueMode();
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

  updateCurrentSample();

  if (open_loop_) {
    if (!enabled_ || emergency_stopped_) {
      if (foc_ready_) {
        motor_.move(0.0f);
      } else {
        centerOpenLoopPwm();
      }
      return;
    }

    const float measured_velocity = config_.read_velocity ? config_.read_velocity() : 0.0f;
    updateVelocityTelemetry(measured_velocity);
    if (foc_ready_) {
      motor_.move(target_velocity_ * config_.velocity_direction);
    } else {
      updateOpenLoopPwm();
    }
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
  if (open_loop_ && !foc_ready_) {
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
    if (over_current_) return;
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

  if (over_current_) return;

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

void DriveMotorController::setCurrentLimit(const float limit) {
  current_limit_ = constrain(limit, 0.1f, kOverCurrentLimitA);
  motor_.current_limit = current_limit_;
}

void DriveMotorController::setTorqueMode(const TorqueMode mode) {
  if (mode != TorqueMode::kVoltage && !simplefoc_current_sense_ready_) return;
  torque_mode_ = mode;
  configureMotorForCurrentMode();
}

float DriveMotorController::getAngle() {
  return config_.read_angle ? config_.read_angle() : 0.0f;
}

DriveMotorController::Status DriveMotorController::status() {
  updateCurrentSample();

  Status current = {};
  current.initialized = initialized_;
  current.foc_ready = foc_ready_;
  current.enabled = enabled_;
  current.emergency_stopped = emergency_stopped_;
  current.open_loop = open_loop_;
  current.torque_mode = torque_mode_;
  current.current_sense_ready = current_sense_ready_;
  current.simplefoc_current_sense_ready = simplefoc_current_sense_ready_;
  current.over_current = over_current_;
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
  current.current_limit = current_limit_;
  current.current_q = motor_.current.q;
  current.current_d = motor_.current.d;
  if (open_loop_) {
    current.current_sp = target_velocity_;
    current.voltage_q = open_loop_voltage_limit_;
    current.voltage_d = 0.0f;
  } else if (torque_mode_ == TorqueMode::kVoltage) {
    current.current_sp = velocity_pid_output_;
    current.voltage_q = motor_.voltage.q;
    current.voltage_d = motor_.voltage.d;
  } else {
    current.current_sp = motor_.current_sp;
    current.voltage_q = motor_.voltage.q;
    current.voltage_d = motor_.voltage.d;
  }
  current.phase_current_a = phase_current_a_;
  current.phase_current_b = phase_current_b_;
  current.phase_current_c = phase_current_c_;
  current.phase_voltage_b = phase_voltage_b_;
  current.phase_voltage_c = phase_voltage_c_;
  current.phase_offset_voltage_b = phase_offset_voltage_b_;
  current.phase_offset_voltage_c = phase_offset_voltage_c_;
  current.phase_voltage_delta_b = phase_voltage_delta_b_;
  current.phase_voltage_delta_c = phase_voltage_delta_c_;
  return current;
}

}  // namespace drive
