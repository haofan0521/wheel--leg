#pragma once

#include <stdint.h>

#include <freertos/FreeRTOS.h>

namespace app_runtime_config {

// 双核任务划分：
// Core 1 负责控制链路，Core 0 负责 Wi-Fi 与调试服务。
constexpr BaseType_t kControlCore = 1;
constexpr BaseType_t kServiceCore = 0;

// 控制任务优先级高于服务任务，保证实时链路优先运行。
constexpr UBaseType_t kControlTaskPriority = 4;
constexpr UBaseType_t kServiceTaskPriority = 2;

// 任务栈大小。
constexpr uint32_t kControlTaskStackBytes = 6144;
constexpr uint32_t kServiceTaskStackBytes = 8192;

// 当前阶段的基础调度周期。
// 闭环 FOC 建议保持在 1ms 或更低，当前设定为 1ms 以提升速度环响应。
constexpr uint32_t kControlTaskPeriodMs = 1;
constexpr uint32_t kServiceTaskPeriodMs = 2;

constexpr bool kEnableVofaTelemetry = false;
constexpr uint32_t kVofaTelemetryDecimation = 10;

}  // namespace app_runtime_config
