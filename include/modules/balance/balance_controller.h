#pragma once

#include <stdint.h>

namespace balance {

struct Config {
  float target_pitch_deg;
  float kp;
  float kd;
  float max_velocity;
  float max_angle_deg;
};

struct Input {
  float pitch_deg;
  uint32_t now_ms;
};

struct Output {
  bool enabled;
  bool active;
  bool fault;
  float target_pitch_deg;
  float pitch_deg;
  float pitch_rate_dps;
  float output_velocity;
  float kp;
  float kd;
  float max_velocity;
  float max_angle_deg;
};

void begin();
void setEnabled(bool enabled);
void setConfig(const Config& config);
Config config();
Output update(const Input& input);
Output status();

}  // namespace balance
