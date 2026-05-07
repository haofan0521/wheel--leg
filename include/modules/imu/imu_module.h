#pragma once

#include "modules/imu/imu_pins.h"

namespace imu {

// BMI088 片内器件类型。
enum class SensorType {
  kAccel,
  kGyro,
};

// IMU 原始数据结构。
struct RawData {
  int16_t x;
  int16_t y;
  int16_t z;
};

// IMU 物理单位数据结构 (单位: m/s^2, rad/s)。
struct ImuData {
  float acc_x;
  float acc_y;
  float acc_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
};

// 姿态角 (单位: degree)。
struct Attitude {
  float pitch;
  float roll;
  float yaw;
};

// 初始化 BMI088 相关 GPIO 与 SPI。
bool begin();

// 更新并获取最新数据。
void update();

// 获取最新的加速度原始数据。
RawData getAccelRaw();

// 获取最新的陀螺仪原始数据。
RawData getGyroRaw();

// 获取转换后的物理单位数据。
ImuData getData();

// 获取姿态角。
Attitude getAttitude();

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
