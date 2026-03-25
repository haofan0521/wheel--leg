#pragma once

#include <stdint.h>

namespace encoder::pins {

// MT6835 编码器共用的 SPI 总线引脚定义。
struct SpiBusPins {
  uint8_t sck;
  uint8_t miso;
  uint8_t mosi;
};

// 单个编码器器件的片选引脚定义。
struct SensorPins {
  uint8_t chip_select;
};

// 左右 MT6835 共用一组 SPI 总线。
inline constexpr SpiBusPins kSpiBus = {
    .sck = 35,
    .miso = 36,
    .mosi = 37,
};

// 左编码器片选。
inline constexpr SensorPins kLeftSensor = {
    .chip_select = 7,
};

// 右编码器片选。
inline constexpr SensorPins kRightSensor = {
    .chip_select = 15,
};

}  // namespace encoder::pins
