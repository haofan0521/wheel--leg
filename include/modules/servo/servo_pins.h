#pragma once

#include <stdint.h>

namespace servo::pins {

// 舵机串口控制引脚定义。
inline constexpr uint8_t kTx = 8;
inline constexpr uint8_t kRx = 18;
inline constexpr uint8_t kData = 8; // 添加 kData 兼容 servo_module.cpp

}  // namespace servo::pins
