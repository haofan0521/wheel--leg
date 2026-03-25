#pragma once

#include <stdint.h>

namespace runtime_state {

// 控制任务对外发布的状态快照。
struct ControlSnapshot {
  uint32_t loop_counter;
  uint32_t last_update_ms;
  uint32_t task_period_ms;
  int drive_fault_level;
  bool drive_enabled;
  uint8_t core_id;
};

// 服务任务对外发布的状态快照。
struct ServiceSnapshot {
  uint32_t loop_counter;
  uint32_t last_update_ms;
  uint32_t task_period_ms;
  bool wifi_server_started;
  uint8_t core_id;
};

// 网页调试接口读取的统一系统状态。
struct SystemSnapshot {
  ControlSnapshot control;
  ServiceSnapshot service;
};

// 初始化共享状态。
void begin();

// 更新控制任务状态。
void updateControlSnapshot(const ControlSnapshot& snapshot);

// 更新服务任务状态。
void updateServiceSnapshot(const ServiceSnapshot& snapshot);

// 获取当前系统状态快照。
SystemSnapshot snapshot();

}  // namespace runtime_state
