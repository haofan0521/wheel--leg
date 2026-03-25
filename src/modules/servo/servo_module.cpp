#include "modules/servo/servo_module.h"

#include <Arduino.h>

namespace {

// 记录当前单总线方向，便于协议层查询。
servo::LineMode g_line_mode = servo::LineMode::kReceive;

void applyLineMode(const servo::LineMode mode) {
  if (mode == servo::LineMode::kTransmit) {
    // 发送模式下将数据脚配置为输出。
    pinMode(servo::pins::kData, OUTPUT);
  } else {
    // 接收模式下切回输入，避免与外部设备争用总线。
    pinMode(servo::pins::kData, INPUT);
  }
}

}  // namespace

namespace servo {

void begin() {
  // 上电默认进入接收模式，减少误发送风险。
  g_line_mode = LineMode::kReceive;
  applyLineMode(g_line_mode);
}

void setLineMode(const LineMode mode) {
  g_line_mode = mode;
  applyLineMode(mode);
}

LineMode lineMode() {
  return g_line_mode;
}

void writeDataLevel(const bool high) {
  // 若上层在发送前未显式切换方向，这里自动切到发送模式。
  if (g_line_mode != LineMode::kTransmit) {
    setLineMode(LineMode::kTransmit);
  }

  digitalWrite(pins::kData, high ? HIGH : LOW);
}

int readDataLevel() {
  // 读取前自动切回接收模式，保证读值来自总线而非本地输出锁存。
  if (g_line_mode != LineMode::kReceive) {
    setLineMode(LineMode::kReceive);
  }

  return digitalRead(pins::kData);
}

void setIdle() {
  // 当前阶段将空闲态统一定义为接收模式。
  // 后续若舵机协议要求总线保持高电平，可在这里集中调整。
  setLineMode(LineMode::kReceive);
}

uint8_t dataPin() {
  return pins::kData;
}

}  // namespace servo
