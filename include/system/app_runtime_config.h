#pragma once

#include <stdint.h>

#include <freertos/FreeRTOS.h>

namespace app_runtime_config {

// 双核任务划分：
// Core 1 负责控制链路，Core 0 负责 Wi-Fi 与调试服务。
inline constexpr BaseType_t kControlCore = 1;
inline constexpr BaseType_t kServiceCore = 0;

// 控制任务优先级高于服务任务，保证实时链路优先运行。
inline constexpr UBaseType_t kControlTaskPriority = 4;
inline constexpr UBaseType_t kServiceTaskPriority = 2;

// 任务栈大小。
inline constexpr uint32_t kControlTaskStackBytes = 6144;
inline constexpr uint32_t kServiceTaskStackBytes = 8192;

// 当前阶段的基础调度周期。
// 后续接入 FOC 时，控制任务可进一步改为定时器驱动或更短周期。
inline constexpr uint32_t kControlTaskPeriodMs = 2;
inline constexpr uint32_t kServiceTaskPeriodMs = 2;

}  // namespace app_runtime_config
