#pragma once

#include <stdint.h>

namespace balance {

struct Config {
  float target_pitch_deg;
  float kp;
  float kd;
  float kv;
  float output_direction;
  float max_velocity;
  float start_angle_deg;
  float max_angle_deg;
  float remote_velocity;
  float remote_turn_velocity;
};

struct Input {
  float pitch_deg;
  float pitch_rate_dps;
  float wheel_velocity;
  uint32_t now_ms;
};

struct Output {
  bool enabled;
  bool active;
  bool fault;
  float target_pitch_deg;
  float pitch_deg;
  float pitch_rate_dps;
  float wheel_velocity;
  float output_velocity;
  float kp;
  float kd;
  float kv;
  float output_direction;
  float max_velocity;
  float start_angle_deg;
  float max_angle_deg;
  float remote_velocity;
  float remote_turn_velocity;
};

void begin();
void setEnabled(bool enabled);
void setConfig(const Config& config);
Config config();
Output update(const Input& input);
Output status();

}  // namespace balance
