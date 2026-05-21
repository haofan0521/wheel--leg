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
  bool open_loop;
  float target_velocity;
  float voltage_limit;
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
  float voltage_q;
  float voltage_d;
  uint32_t command_age_ms;
};

struct BalanceCommand {
  bool enable;
  bool stop;
  bool has_enable;
  bool has_tuning;
  float target_pitch_deg;
  float kp;
  float kd;
  float kv;
  float output_direction;
  float max_velocity;
  float start_angle_deg;
  float max_angle_deg;
  uint32_t updated_ms;
  uint32_t sequence;
};

struct BalanceSnapshot {
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
  uint32_t last_update_ms;
};

struct ControlSnapshot {
  uint32_t loop_counter;
  uint32_t last_update_ms;
  uint32_t task_period_ms;
  int drive_fault_level;
  bool drive_enabled;
  uint8_t core_id;
  BalanceSnapshot balance;
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

struct ServoCommand {
  bool has_height;
  float target_height;
  uint16_t time_ms;
  uint32_t updated_ms;
  uint32_t sequence;
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
void updateBalanceCommand(const BalanceCommand& command);
void updateServoCommand(const ServoCommand& command);
MotorCommand leftMotorCommand();
MotorCommand rightMotorCommand();
BalanceCommand balanceCommand();
ServoCommand servoCommand();
SystemSnapshot snapshot();

}  // namespace runtime_state
