#pragma once

#include <stdint.h>

namespace drive::pins {

// 三相电机 PWM 引脚定义。
struct ThreePhasePwmPins {
  uint8_t phase_a;
  uint8_t phase_b;
  uint8_t phase_c;
};

// 电驱反馈采样引脚定义。
struct FeedbackPins {
  uint8_t channel_a;
  uint8_t channel_b;
  uint8_t channel_c;
};

// 电驱总使能与故障反馈引脚。
inline constexpr uint8_t kEnable = 14;
inline constexpr uint8_t kFault = 21;

// 左轮电机三相 PWM 输出。
inline constexpr ThreePhasePwmPins kLeftMotorPwm = {
    .phase_a = 39,
    .phase_b = 40,
    .phase_c = 41,
};

// 右轮电机三相 PWM 输出。
// 注意：原设计的 48, 47 已恢复（用户确认使用 47 48）。
inline constexpr ThreePhasePwmPins kRightMotorPwm = {
    .phase_a = 48,
    .phase_b = 47,
    .phase_c = 42,
};

// 原理图中沿用 SQA/SQB/SQC 命名。
// 在固件中，这 3 路信号作为左侧电驱反馈采样输入管理。
inline constexpr FeedbackPins kLeftMotorFeedback = {
    .channel_a = 1,
    .channel_b = 2,
    .channel_c = 3,
};

// 原理图中沿用 SQA/SQB/SQC 命名。
// 在固件中，这 3 路信号作为右侧电驱反馈采样输入管理。
inline constexpr FeedbackPins kRightMotorFeedback = {
    .channel_a = 4,
    .channel_b = 5,
    .channel_c = 6,
};

}  // namespace drive::pins
