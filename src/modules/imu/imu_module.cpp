#include "modules/imu/imu_module.h"

#include <Arduino.h>
#include <SPI.h>

namespace {

SPIClass imu_spi(HSPI);

// BMI088 寄存器定义
namespace regs {
// Accel
const uint8_t kAccChipId = 0x00;
const uint8_t kAccData = 0x12;
const uint8_t kAccConf = 0x40;
const uint8_t kAccRange = 0x41;
const uint8_t kAccPwrConf = 0x7C;
const uint8_t kAccPwrCtrl = 0x7D;
const uint8_t kAccSoftReset = 0x7E;

// Gyro
const uint8_t kGyroChipId = 0x00;
const uint8_t kGyroData = 0x02;
const uint8_t kGyroRange = 0x0F;
const uint8_t kGyroBandwidth = 0x10;
const uint8_t kGyroSoftReset = 0x14;
}  // namespace regs

imu::RawData latest_accel = {0, 0, 0};
imu::RawData latest_gyro = {0, 0, 0};
imu::ImuData processed_data = {0, 0, 0, 0, 0, 0};
imu::Attitude attitude = {0, 0, 0};

// 转换常量
constexpr float kAccelRange6gLSB = 5461.33f;  // 32768 / 6
constexpr float kGyroRange2000LSB = 16.384f;  // 32768 / 2000
constexpr float kGravity = 9.80665f;
constexpr float kDeg2Rad = PI / 180.0f;
constexpr float kRad2Deg = 180.0f / PI;

// 姿态解算参数 (简单互补滤波)
constexpr float kFilterAlpha = 0.98f;
uint32_t last_update_time = 0;

void writeRegister(imu::SensorType sensor, uint8_t reg, uint8_t data) {
  imu::select(sensor);
  imu_spi.transfer(reg & 0x7F);  // 写操作，最高位为0
  imu_spi.transfer(data);
  imu::deselect(sensor);
}

uint8_t readRegister(imu::SensorType sensor, uint8_t reg) {
  imu::select(sensor);
  imu_spi.transfer(reg | 0x80);  // 读操作，最高位为1
  if (sensor == imu::SensorType::kAccel) {
    imu_spi.transfer(0x00);  // Accel 读操作需要一个 Dummy Byte
  }
  uint8_t data = imu_spi.transfer(0x00);
  imu::deselect(sensor);
  return data;
}

void readBurst(imu::SensorType sensor, uint8_t reg, uint8_t* buffer, uint8_t len) {
  imu::select(sensor);
  imu_spi.transfer(reg | 0x80);
  if (sensor == imu::SensorType::kAccel) {
    imu_spi.transfer(0x00);  // Accel 读操作需要一个 Dummy Byte
  }
  for (uint8_t i = 0; i < len; i++) {
    buffer[i] = imu_spi.transfer(0x00);
  }
  imu::deselect(sensor);
}

void configureChipSelectPin(const imu::pins::SensorPins& sensor_pins) {
  // BMI088 使用低电平片选，因此初始化后默认拉高。
  pinMode(sensor_pins.chip_select, OUTPUT);
  digitalWrite(sensor_pins.chip_select, HIGH);
}

void configureInterruptPins(const imu::pins::InterruptPins& interrupt_pins) {
  // 中断引脚当前作为普通输入初始化，
  // 后续接入外部中断服务时可进一步补充 attachInterrupt。
  pinMode(interrupt_pins.int1, INPUT);
  pinMode(interrupt_pins.int3, INPUT);
}

uint8_t chipSelectPinFromSensor(const imu::SensorType sensor) {
  return sensor == imu::SensorType::kAccel ? imu::pins::kAccelSensor.chip_select
                                           : imu::pins::kGyroSensor.chip_select;
}

}  // namespace

namespace imu {

bool begin() {
  // 初始化片选和中断引脚
  configureChipSelectPin(pins::kAccelSensor);
  configureChipSelectPin(pins::kGyroSensor);
  configureInterruptPins(pins::kInterrupts);

  // 初始化 SPI 总线
  imu_spi.begin(pins::kSpiBus.sck, pins::kSpiBus.miso, pins::kSpiBus.mosi, -1);
  imu_spi.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE3));

  // --- 加速度计初始化 ---
  writeRegister(SensorType::kAccel, regs::kAccSoftReset, 0xB6);
  delay(50);
  writeRegister(SensorType::kAccel, regs::kAccPwrConf, 0x00);  // Active mode
  delay(10);
  writeRegister(SensorType::kAccel, regs::kAccPwrCtrl, 0x04);  // Accel enable
  delay(10);
  writeRegister(SensorType::kAccel, regs::kAccRange, 0x01);   // ±6g
  writeRegister(SensorType::kAccel, regs::kAccConf, 0xA9);    // ODR 800Hz, BW 140Hz

  // --- 陀螺仪初始化 ---
  writeRegister(SensorType::kGyro, regs::kGyroSoftReset, 0xB6);
  delay(50);
  writeRegister(SensorType::kGyro, regs::kGyroRange, 0x00);      // ±2000°/s
  writeRegister(SensorType::kGyro, regs::kGyroBandwidth, 0x02);  // ODR 1000Hz, BW 116Hz

  // 验证 Chip ID (可选)
  uint8_t acc_id = readRegister(SensorType::kAccel, regs::kAccChipId);
  uint8_t gyro_id = readRegister(SensorType::kGyro, regs::kGyroChipId);

  last_update_time = micros();
  return (acc_id == 0x1E && gyro_id == 0x0F);
}

void update() {
  uint8_t buf[6];
  uint32_t now = micros();
  float dt = (now - last_update_time) / 1000000.0f;
  last_update_time = now;

  // 如果 dt 异常 (例如初次运行)，直接返回
  if (dt <= 0 || dt > 0.1f) dt = 0.002f;

  // 读取加速度
  readBurst(SensorType::kAccel, regs::kAccData, buf, 6);
  latest_accel.x = (int16_t)((buf[1] << 8) | buf[0]);
  latest_accel.y = (int16_t)((buf[3] << 8) | buf[2]);
  latest_accel.z = (int16_t)((buf[5] << 8) | buf[4]);

  // 读取陀螺仪
  readBurst(SensorType::kGyro, regs::kGyroData, buf, 6);
  latest_gyro.x = (int16_t)((buf[1] << 8) | buf[0]);
  latest_gyro.y = (int16_t)((buf[3] << 8) | buf[2]);
  latest_gyro.z = (int16_t)((buf[5] << 8) | buf[4]);

  // 1. 转换为物理单位
  // 加速度 m/s^2
  processed_data.acc_x = (float)latest_accel.x / kAccelRange6gLSB * kGravity;
  processed_data.acc_y = (float)latest_accel.y / kAccelRange6gLSB * kGravity;
  processed_data.acc_z = (float)latest_accel.z / kAccelRange6gLSB * kGravity;
  // 陀螺仪 rad/s (芯片输出为 deg/s)
  processed_data.gyro_x = ((float)latest_gyro.x / kGyroRange2000LSB) * kDeg2Rad;
  processed_data.gyro_y = ((float)latest_gyro.y / kGyroRange2000LSB) * kDeg2Rad;
  processed_data.gyro_z = ((float)latest_gyro.z / kGyroRange2000LSB) * kDeg2Rad;

  // 2. 姿态解算 (简单互补滤波)
  // 通过加速度解算倾角 (假设 Z 轴向上)
  float acc_pitch = atan2(-processed_data.acc_x, 
                          sqrt(processed_data.acc_y * processed_data.acc_y + 
                               processed_data.acc_z * processed_data.acc_z)) * kRad2Deg;
  float acc_roll = atan2(processed_data.acc_y, processed_data.acc_z) * kRad2Deg;

  // 互补滤波: 融合陀螺仪角速度积分与加速度静态夹角
  // 陀螺仪解算使用的是 deg/s
  float gyro_pitch_deg = processed_data.gyro_y * kRad2Deg;
  float gyro_roll_deg = processed_data.gyro_x * kRad2Deg;

  attitude.pitch = kFilterAlpha * (attitude.pitch + gyro_pitch_deg * dt) + (1.0f - kFilterAlpha) * acc_pitch;
  attitude.roll = kFilterAlpha * (attitude.roll + gyro_roll_deg * dt) + (1.0f - kFilterAlpha) * acc_roll;
  // Yaw 仅能通过陀螺仪积分，会存在漂移
  attitude.yaw += processed_data.gyro_z * kRad2Deg * dt;
}

RawData getAccelRaw() {
  return latest_accel;
}

RawData getGyroRaw() {
  return latest_gyro;
}

ImuData getData() {
  return processed_data;
}

Attitude getAttitude() {
  return attitude;
}

void select(const SensorType sensor) {
  digitalWrite(chipSelectPinFromSensor(sensor), LOW);
}

void deselect(const SensorType sensor) {
  digitalWrite(chipSelectPinFromSensor(sensor), HIGH);
}

void deselectAll() {
  digitalWrite(pins::kAccelSensor.chip_select, HIGH);
  digitalWrite(pins::kGyroSensor.chip_select, HIGH);
}

int readInt1Level() {
  return digitalRead(pins::kInterrupts.int1);
}

int readInt3Level() {
  return digitalRead(pins::kInterrupts.int3);
}

const pins::SpiBusPins& spiBusPins() {
  return pins::kSpiBus;
}

const pins::SensorPins& accelSensorPins() {
  return pins::kAccelSensor;
}

const pins::SensorPins& gyroSensorPins() {
  return pins::kGyroSensor;
}

const pins::InterruptPins& interruptPins() {
  return pins::kInterrupts;
}

}  // namespace imu
