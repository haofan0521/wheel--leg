#pragma once

#include "modules/encoder/encoder_pins.h"
#include <SimpleFOC.h>

namespace encoder {

// 编码器侧别，用于统一控制左右编码器片选。
enum class Side {
  kLeft,
  kRight,
};

// 初始化 MT6835 编码器相关 GPIO 和 SPI 接口。
void begin();

// 获取适配 SimpleFOC 的右侧编码器实例指针 (Sensor)
Sensor* rightSensor();

// 获取适配 SimpleFOC 的左侧编码器实例指针 (Sensor)
Sensor* leftSensor();

// 获取左侧编码器角度 (rad)，会先刷新传感器采样。
float leftAngle();

// 获取左侧编码器速度 (rad/s)，会先刷新传感器采样。
float leftVelocity();

// 获取右侧编码器角度 (rad)，会先刷新传感器采样。
float rightAngle();

// 获取右侧编码器速度 (rad/s)，会先刷新传感器采样。
float rightVelocity();

// 【测试函数】打印左右编码器的角度和速度。
void testPrintEncoders();

// 【测试函数】直接通过 SPI 读取左侧 MT6835 的原始数据帧并打印至串口。
void testReadLeftEncoder();

// 【测试函数】直接通过 SPI 读取右侧 MT6835 的原始数据帧并打印至串口。
void testReadRightEncoder();

}  // namespace encoder
