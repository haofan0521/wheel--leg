#pragma once

#include "modules/encoder/encoder_pins.h"

namespace encoder {

// 编码器侧别，用于统一控制左右编码器片选。
enum class Side {
  kLeft,
  kRight,
};

// 初始化 MT6835 编码器相关 GPIO。
void begin();

// 片选指定一侧编码器。
void select(Side side);

// 释放指定一侧编码器片选。
void deselect(Side side);

// 同时释放左右编码器片选，避免总线冲突。
void deselectAll();

// 对外暴露 SPI 总线和左右器件引脚定义，便于后续驱动层复用。
const pins::SpiBusPins& spiBusPins();
const pins::SensorPins& leftSensorPins();
const pins::SensorPins& rightSensorPins();

}  // namespace encoder
