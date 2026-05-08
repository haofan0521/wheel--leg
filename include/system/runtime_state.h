#pragma once

#include <stdint.h>

namespace runtime_state {

struct MotorCommand {
  bool enable;
  bool stop;
  bool has_enable;
  bool has_velocity_target;
  bool has_voltage_limit;
  bool has_open_loop;
  bool has_tuning;
  bool has_current_limit;
  bool has_torque_mode;
  bool open_loop;
  uint8_t torque_mode;
  float target_velocity;
  float voltage_limit;
  float current_limit;
  float velocity_p;
  float velocity_i;
  float velocity_d;
  float velocity_lpf_tf;
  uint32_t updated_ms;
  uint32_t sequence;
};

struct MotorSnapshot {
  bool initialized;
  bool foc_ready;
  bool enabled;
  bool emergency_stopped;
  bool open_loop;
  uint8_t torque_mode;
  bool current_sense_ready;
  bool simplefoc_current_sense_ready;
  bool over_current;
  float target_velocity;
  float measured_velocity;
  float shaft_angle;
  float voltage_limit;
  float velocity_p;
  float velocity_i;
  float velocity_d;
  float velocity_lpf_tf;
  float velocity_error;
  float velocity_p_output;
  float velocity_i_output;
  float velocity_pid_output;
  float current_limit;
  float current_q;
  float current_d;
  float current_sp;
  float voltage_q;
  float voltage_d;
  float phase_current_a;
  float phase_current_b;
  float phase_current_c;
  float phase_voltage_b;
  float phase_voltage_c;
  float phase_offset_voltage_b;
  float phase_offset_voltage_c;
  float phase_voltage_delta_b;
  float phase_voltage_delta_c;
  uint32_t command_age_ms;
};

struct ControlSnapshot {
  uint32_t loop_counter;
  uint32_t last_update_ms;
  uint32_t task_period_ms;
  int drive_fault_level;
  bool drive_enabled;
  uint8_t core_id;
  MotorSnapshot left_motor;
  MotorSnapshot right_motor;
};

struct ServiceSnapshot {
  uint32_t loop_counter;
  uint32_t last_update_ms;
  uint32_t task_period_ms;
  bool wifi_server_started;
  uint8_t core_id;
};

struct ImuSnapshot {
  float pitch_deg;
  float roll_deg;
  float yaw_deg;
  float acc_z;
  uint32_t last_update_ms;
  uint32_t sequence;
  bool valid;
};

struct SystemSnapshot {
  ControlSnapshot control;
  ServiceSnapshot service;
  ImuSnapshot imu;
};

void begin();
void updateControlSnapshot(const ControlSnapshot& snapshot);
void updateServiceSnapshot(const ServiceSnapshot& snapshot);
void updateImuSnapshot(const ImuSnapshot& snapshot);
void updateLeftMotorCommand(const MotorCommand& command);
void updateRightMotorCommand(const MotorCommand& command);
MotorCommand leftMotorCommand();
MotorCommand rightMotorCommand();
SystemSnapshot snapshot();

}  // namespace runtime_state
