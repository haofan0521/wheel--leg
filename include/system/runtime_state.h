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
  bool open_loop;
  float target_velocity;
  float voltage_limit;
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
  uint32_t command_age_ms;
};

// 控制任务对外发布的状态快照。
struct ControlSnapshot {
  uint32_t loop_counter;
  uint32_t last_update_ms;
  uint32_t task_period_ms;
  int drive_fault_level;
  bool drive_enabled;
  uint8_t core_id;
  MotorSnapshot left_motor;
};

// 服务任务对外发布的状态快照。
struct ServiceSnapshot {
  uint32_t loop_counter;
  uint32_t last_update_ms;
  uint32_t task_period_ms;
  bool wifi_server_started;
  uint8_t core_id;
};

// IMU 姿态快照，由控制任务发布，网页调试接口只读。
struct ImuSnapshot {
  float pitch_deg;
  float roll_deg;
  float yaw_deg;
  float acc_z;
  uint32_t last_update_ms;
  uint32_t sequence;
  bool valid;
};

// 网页调试接口读取的统一系统状态。
struct SystemSnapshot {
  ControlSnapshot control;
  ServiceSnapshot service;
  ImuSnapshot imu;
};

// 初始化共享状态。
void begin();

// 更新控制任务状态。
void updateControlSnapshot(const ControlSnapshot& snapshot);

// 更新服务任务状态。
void updateServiceSnapshot(const ServiceSnapshot& snapshot);

// 更新 IMU 姿态快照。
void updateImuSnapshot(const ImuSnapshot& snapshot);

// 更新左轮控制命令。
void updateLeftMotorCommand(const MotorCommand& command);

// 获取左轮控制命令。
MotorCommand leftMotorCommand();

// 获取当前系统状态快照。
SystemSnapshot snapshot();

}  // namespace runtime_state
