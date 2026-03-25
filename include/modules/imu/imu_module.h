#pragma once

#include "modules/imu/imu_pins.h"

namespace imu {

// BMI088 片内器件类型。
enum class SensorType {
  kAccel,
  kGyro,
};

// 初始化 BMI088 相关 GPIO。
void begin();

// 选中指定器件。
void select(SensorType sensor);

// 释放指定器件片选。
void deselect(SensorType sensor);

// 同时释放加速度计与陀螺仪片选。
void deselectAll();

// 读取 BMI088 中断引脚电平。
int readInt1Level();
int readInt3Level();

// 对外暴露总线与引脚定义，便于后续驱动层复用。
const pins::SpiBusPins& spiBusPins();
const pins::SensorPins& accelSensorPins();
const pins::SensorPins& gyroSensorPins();
const pins::InterruptPins& interruptPins();

}  // namespace imu
