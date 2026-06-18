#include "modules/balance/balance_controller.h"

#include <Arduino.h>

namespace {

constexpr float kDefaultTargetPitchDeg = 0.795f;
constexpr float kDefaultKp = 1.69f;
constexpr float kDefaultKd = 0.028f;
constexpr float kDefaultKv = 0.5f;
constexpr bool kDefaultUseLqr = false;
constexpr float kDefaultLqrPitch = -119.86971f;
constexpr float kDefaultLqrPitchRate = -18.810969f;
constexpr float kDefaultLqrWheelVelocity = -1.208787f;
constexpr float kDefaultLqrOutputSlewRate = 80.0f;
constexpr float kDefaultOutputDirection = -1.0f;
constexpr float kDefaultMaxVelocity = 10.0f;
constexpr float kDefaultStartAngleDeg = 15.0f; // 放宽启动角度 (原 10.0)
constexpr float kDefaultMaxAngleDeg = 35.0f;
constexpr float kDegToRad = 0.01745329252f;

balance::Config g_config = {
    .target_pitch_deg = kDefaultTargetPitchDeg,
    .kp = kDefaultKp,
    .kd = kDefaultKd,
    .kv = kDefaultKv,
    .use_lqr = kDefaultUseLqr,
    .lqr_pitch = kDefaultLqrPitch,
    .lqr_pitch_rate = kDefaultLqrPitchRate,
    .lqr_wheel_velocity = kDefaultLqrWheelVelocity,
    .lqr_output_slew_rate = kDefaultLqrOutputSlewRate,
    .output_direction = kDefaultOutputDirection,
    .max_velocity = kDefaultMaxVelocity,
    .start_angle_deg = kDefaultStartAngleDeg,
    .max_angle_deg = kDefaultMaxAngleDeg,
};
balance::Output g_output = {};
bool g_enabled = false;
float g_lqr_output_velocity_prev = 0.0f;
uint32_t g_lqr_output_update_ms = 0;

float clampAbs(const float value, const float limit) {
  const float abs_limit = fabsf(limit);
  return constrain(value, -abs_limit, abs_limit);
}

void resetLqrOutputSlew(const uint32_t now_ms = 0) {
  g_lqr_output_velocity_prev = 0.0f;
  g_lqr_output_update_ms = now_ms;
}

float applyLqrOutputSlew(const float target_velocity, const uint32_t now_ms) {
  float dt = 0.001f;
  if (g_lqr_output_update_ms != 0U && now_ms != 0U) {
    const uint32_t elapsed_ms = now_ms - g_lqr_output_update_ms;
    dt = static_cast<float>(elapsed_ms) * 0.001f;
    if (dt <= 0.0f || dt > 0.1f) {
      dt = 0.001f;
    }
  }
  g_lqr_output_update_ms = now_ms;

  const float max_delta = g_config.lqr_output_slew_rate * dt;
  const float delta = constrain(target_velocity - g_lqr_output_velocity_prev,
                                -max_delta,
                                max_delta);
  g_lqr_output_velocity_prev += delta;
  return g_lqr_output_velocity_prev;
}

void fillCommonOutput(const float pitch_deg, const float pitch_rate_dps, const float wheel_velocity) {
  g_output.enabled = g_enabled;
  g_output.target_pitch_deg = g_config.target_pitch_deg;
  g_output.pitch_deg = pitch_deg;
  g_output.pitch_rate_dps = pitch_rate_dps;
  g_output.wheel_velocity = wheel_velocity;
  g_output.kp = g_config.kp;
  g_output.kd = g_config.kd;
  g_output.kv = g_config.kv;
  g_output.use_lqr = g_config.use_lqr;
  g_output.lqr_pitch = g_config.lqr_pitch;
  g_output.lqr_pitch_rate = g_config.lqr_pitch_rate;
  g_output.lqr_wheel_velocity = g_config.lqr_wheel_velocity;
  g_output.lqr_output_slew_rate = g_config.lqr_output_slew_rate;
  g_output.output_direction = g_config.output_direction;
  g_output.max_velocity = g_config.max_velocity;
  g_output.start_angle_deg = g_config.start_angle_deg;
  g_output.max_angle_deg = g_config.max_angle_deg;
}

}  // namespace

namespace balance {

void begin() {
  g_enabled = false;
  resetLqrOutputSlew();
  g_output = {};
  g_output.target_pitch_deg = g_config.target_pitch_deg;
  g_output.kp = g_config.kp;
  g_output.kd = g_config.kd;
  g_output.kv = g_config.kv;
  g_output.use_lqr = g_config.use_lqr;
  g_output.lqr_pitch = g_config.lqr_pitch;
  g_output.lqr_pitch_rate = g_config.lqr_pitch_rate;
  g_output.lqr_wheel_velocity = g_config.lqr_wheel_velocity;
  g_output.lqr_output_slew_rate = g_config.lqr_output_slew_rate;
  g_output.output_direction = g_config.output_direction;
  g_output.max_velocity = g_config.max_velocity;
  g_output.start_angle_deg = g_config.start_angle_deg;
  g_output.max_angle_deg = g_config.max_angle_deg;
}

void setEnabled(const bool enabled) {
  g_enabled = enabled;
  if (!enabled) {
    g_output.active = false;
    g_output.fault = false;
    g_output.output_velocity = 0.0f;
    resetLqrOutputSlew();
  }
}

void setConfig(const Config& config) {
  g_config.target_pitch_deg = constrain(config.target_pitch_deg, -20.0f, 20.0f);
  g_config.kp = constrain(config.kp, -20.0f, 20.0f);
  g_config.kd = constrain(config.kd, -5.0f, 5.0f);
  g_config.kv = constrain(config.kv, 0.0f, 5.0f);
  g_config.use_lqr = config.use_lqr;
  g_config.lqr_pitch = constrain(config.lqr_pitch, -500.0f, 500.0f);
  g_config.lqr_pitch_rate = constrain(config.lqr_pitch_rate, -100.0f, 100.0f);
  g_config.lqr_wheel_velocity = constrain(config.lqr_wheel_velocity, -20.0f, 20.0f);
  g_config.lqr_output_slew_rate = constrain(config.lqr_output_slew_rate, 10.0f, 500.0f);
  g_config.output_direction = config.output_direction >= 0.0f ? 1.0f : -1.0f;
  g_config.max_velocity = constrain(config.max_velocity, 0.2f, 20.0f);
  g_config.max_angle_deg = constrain(config.max_angle_deg, 5.0f, 60.0f);
  g_config.start_angle_deg = constrain(config.start_angle_deg, 1.0f, g_config.max_angle_deg);
}

Config config() {
  return g_config;
}

Output update(const Input& input) {
  fillCommonOutput(input.pitch_deg, input.pitch_rate_dps, input.wheel_velocity);

  const float pitch_error = input.pitch_deg - g_config.target_pitch_deg;
  if (!g_enabled) {
    g_output.active = false;
    g_output.fault = false;
    g_output.output_velocity = 0.0f;
    resetLqrOutputSlew(input.now_ms);
    return g_output;
  }

  if (fabsf(pitch_error) > g_config.max_angle_deg) {
    g_output.enabled = g_enabled;
    g_output.active = false;
    g_output.fault = true;
    g_output.output_velocity = 0.0f;
    resetLqrOutputSlew(input.now_ms);
    return g_output;
  }

  if (!g_output.active && fabsf(pitch_error) > g_config.start_angle_deg) {
    g_output.enabled = g_enabled;
    g_output.active = false;
    g_output.fault = true;
    g_output.output_velocity = 0.0f;
    resetLqrOutputSlew(input.now_ms);
    return g_output;
  }

  g_output.active = true;
  g_output.fault = false;
  float control_velocity = 0.0f;
  if (g_config.use_lqr) {
    const float theta = pitch_error * kDegToRad;
    const float theta_dot = input.pitch_rate_dps * kDegToRad;
    control_velocity = -(g_config.lqr_pitch * theta +
                         g_config.lqr_pitch_rate * theta_dot +
                         g_config.lqr_wheel_velocity * input.wheel_velocity);
  } else {
    control_velocity = g_config.kp * pitch_error +
                       g_config.kd * input.pitch_rate_dps -
                       g_config.kv * input.wheel_velocity;
  }
  const float target_output_velocity = clampAbs(g_config.output_direction * control_velocity,
                                                g_config.max_velocity);
  if (g_config.use_lqr) {
    g_output.output_velocity = applyLqrOutputSlew(target_output_velocity, input.now_ms);
  } else {
    resetLqrOutputSlew(input.now_ms);
    g_output.output_velocity = target_output_velocity;
  }
  return g_output;
}

Output status() {
  return g_output;
}

}  // namespace balance
