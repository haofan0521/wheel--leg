#include "modules/balance/balance_controller.h"

#include <Arduino.h>

namespace {

constexpr float kDefaultTargetPitchDeg = 0.0f;
constexpr float kDefaultKp = 0.6f;
constexpr float kDefaultKd = 0.03f;
constexpr float kDefaultMaxVelocity = 4.0f;
constexpr float kDefaultMaxAngleDeg = 25.0f;

balance::Config g_config = {
    .target_pitch_deg = kDefaultTargetPitchDeg,
    .kp = kDefaultKp,
    .kd = kDefaultKd,
    .max_velocity = kDefaultMaxVelocity,
    .max_angle_deg = kDefaultMaxAngleDeg,
};
balance::Output g_output = {};
bool g_enabled = false;
bool g_have_last_pitch = false;
float g_last_pitch_deg = 0.0f;
uint32_t g_last_update_ms = 0;

float clampAbs(const float value, const float limit) {
  const float abs_limit = fabsf(limit);
  return constrain(value, -abs_limit, abs_limit);
}

void fillCommonOutput(const float pitch_deg, const float pitch_rate_dps) {
  g_output.enabled = g_enabled;
  g_output.target_pitch_deg = g_config.target_pitch_deg;
  g_output.pitch_deg = pitch_deg;
  g_output.pitch_rate_dps = pitch_rate_dps;
  g_output.kp = g_config.kp;
  g_output.kd = g_config.kd;
  g_output.max_velocity = g_config.max_velocity;
  g_output.max_angle_deg = g_config.max_angle_deg;
}

}  // namespace

namespace balance {

void begin() {
  g_enabled = false;
  g_have_last_pitch = false;
  g_last_pitch_deg = 0.0f;
  g_last_update_ms = 0;
  g_output = {};
  g_output.target_pitch_deg = g_config.target_pitch_deg;
  g_output.kp = g_config.kp;
  g_output.kd = g_config.kd;
  g_output.max_velocity = g_config.max_velocity;
  g_output.max_angle_deg = g_config.max_angle_deg;
}

void setEnabled(const bool enabled) {
  g_enabled = enabled;
  if (!enabled) {
    g_have_last_pitch = false;
    g_output.active = false;
    g_output.fault = false;
    g_output.output_velocity = 0.0f;
  }
}

void setConfig(const Config& config) {
  g_config.target_pitch_deg = constrain(config.target_pitch_deg, -20.0f, 20.0f);
  g_config.kp = constrain(config.kp, -20.0f, 20.0f);
  g_config.kd = constrain(config.kd, -5.0f, 5.0f);
  g_config.max_velocity = constrain(config.max_velocity, 0.2f, 20.0f);
  g_config.max_angle_deg = constrain(config.max_angle_deg, 5.0f, 60.0f);
}

Config config() {
  return g_config;
}

Output update(const Input& input) {
  float pitch_rate_dps = 0.0f;
  if (g_have_last_pitch && input.now_ms != g_last_update_ms) {
    const float dt = static_cast<float>(input.now_ms - g_last_update_ms) * 0.001f;
    if (dt > 0.0f && dt < 0.5f) {
      pitch_rate_dps = (input.pitch_deg - g_last_pitch_deg) / dt;
    }
  }

  g_have_last_pitch = true;
  g_last_pitch_deg = input.pitch_deg;
  g_last_update_ms = input.now_ms;

  fillCommonOutput(input.pitch_deg, pitch_rate_dps);

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

  g_output.active = true;
  g_output.fault = false;
  g_output.output_velocity = clampAbs(g_config.kp * pitch_error + g_config.kd * pitch_rate_dps,
                                      g_config.max_velocity);
  return g_output;
}

Output status() {
  return g_output;
}

}  // namespace balance
