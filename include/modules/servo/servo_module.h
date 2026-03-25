#pragma once

#include <stdint.h>

#include "modules/servo/servo_pins.h"

namespace servo {

// 单总线数据方向。
enum class LineMode {
  kReceive,
  kTransmit,
};

// 初始化舵机数据引脚。
void begin();

// 设置单总线当前工作方向。
void setLineMode(LineMode mode);

// 查询当前单总线工作方向。
LineMode lineMode();

// 写数据线电平。
// 仅在发送模式下使用，便于后续挂接自定义通信协议。
void writeDataLevel(bool high);

// 读取数据线电平。
int readDataLevel();

// 将数据线恢复为空闲状态。
void setIdle();

// 返回舵机数据引脚编号。
uint8_t dataPin();

}  // namespace servo
