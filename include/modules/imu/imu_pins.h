#pragma once

#include <stdint.h>

namespace imu::pins {

// BMI088 共用的 SPI 总线引脚定义。
struct SpiBusPins {
  uint8_t mosi;
  uint8_t sck;
  uint8_t miso;
};

// BMI088 器件引脚定义。
struct SensorPins {
  uint8_t chip_select;
};

// BMI088 中断引脚定义。
struct InterruptPins {
  uint8_t int1;
  uint8_t int3;
};

// BMI088 加速度计与陀螺仪共用一组 SPI 总线。
constexpr SpiBusPins kSpiBus = {
    .mosi = 11,
    .sck = 12,
    .miso = 13,
};

// BMI088 加速度计片选。
constexpr SensorPins kAccelSensor = {
    .chip_select = 16,
};

// BMI088 陀螺仪片选。
constexpr SensorPins kGyroSensor = {
    .chip_select = 17,
};

// BMI088 中断输出。
constexpr InterruptPins kInterrupts = {
    .int1 = 9,
    .int3 = 10,
};

}  // namespace imu::pins
