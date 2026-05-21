#pragma once

#include <stdint.h>

namespace drive::pins {

// 三相电机 PWM 引脚定义。
struct ThreePhasePwmPins {
  uint8_t phase_a;
  uint8_t phase_b;
  uint8_t phase_c;
};

// 电驱总使能与故障反馈引脚。
constexpr uint8_t kEnable = 14;
constexpr uint8_t kFault = 21;

// 左轮电机三相 PWM 输出。
constexpr ThreePhasePwmPins kLeftMotorPwm = {
    .phase_a = 39,
    .phase_b = 40,
    .phase_c = 41,
};

// 右轮电机三相 PWM 输出。
// 注意：原设计的 48, 47 已恢复（用户确认使用 47 48）。
constexpr ThreePhasePwmPins kRightMotorPwm = {
    .phase_a = 48,
    .phase_b = 47,
    .phase_c = 42,
};

}  // namespace drive::pins
