#include "modules/balance/balance_controller.h"

#include <Arduino.h>

namespace {

constexpr float kDefaultTargetPitchDeg = 0.0f;
constexpr float kDefaultKp = 0.6f;
constexpr float kDefaultKd = 0.03f;
constexpr float kDefaultKv = 0.0f;
constexpr float kDefaultOutputDirection = 1.0f;
constexpr float kDefaultMaxVelocity = 4.0f;
constexpr float kDefaultStartAngleDeg = 10.0f;
constexpr float kDefaultMaxAngleDeg = 25.0f;

balance::Config g_config = {
    .target_pitch_deg = kDefaultTargetPitchDeg,
    .kp = kDefaultKp,
    .kd = kDefaultKd,
    .kv = kDefaultKv,
    .output_direction = kDefaultOutputDirection,
    .max_velocity = kDefaultMaxVelocity,
    .start_angle_deg = kDefaultStartAngleDeg,
    .max_angle_deg = kDefaultMaxAngleDeg,
};
balance::Output g_output = {};
bool g_enabled = false;

float clampAbs(const float value, const float limit) {
  const float abs_limit = fabsf(limit);
  return constrain(value, -abs_limit, abs_limit);
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
  g_output.output_direction = g_config.output_direction;
  g_output.max_velocity = g_config.max_velocity;
  g_output.start_angle_deg = g_config.start_angle_deg;
  g_output.max_angle_deg = g_config.max_angle_deg;
}

}  // namespace

namespace balance {

void begin() {
  g_enabled = false;
  g_output = {};
  g_output.target_pitch_deg = g_config.target_pitch_deg;
  g_output.kp = g_config.kp;
  g_output.kd = g_config.kd;
  g_output.kv = g_config.kv;
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
  }
}

void setConfig(const Config& config) {
  g_config.target_pitch_deg = constrain(config.target_pitch_deg, -20.0f, 20.0f);
  g_config.kp = constrain(config.kp, -20.0f, 20.0f);
  g_config.kd = constrain(config.kd, -5.0f, 5.0f);
  g_config.kv = constrain(config.kv, 0.0f, 5.0f);
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
    return g_output;
  }

  if (fabsf(pitch_error) > g_config.max_angle_deg) {
    g_output.enabled = g_enabled;
    g_output.active = false;
    g_output.fault = true;
    g_output.output_velocity = 0.0f;
    return g_output;
  }

  if (!g_output.active && fabsf(pitch_error) > g_config.start_angle_deg) {
    g_output.enabled = g_enabled;
    g_output.active = false;
    g_output.fault = true;
    g_output.output_velocity = 0.0f;
    return g_output;
  }

  g_output.active = true;
  g_output.fault = false;
  g_output.output_velocity = clampAbs(g_config.output_direction *
                                          (g_config.kp * pitch_error +
                                           g_config.kd * input.pitch_rate_dps -
                                           g_config.kv * input.wheel_velocity),
                                      g_config.max_velocity);
  return g_output;
}

Output status() {
  return g_output;
}

}  // namespace balance
