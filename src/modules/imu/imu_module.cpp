#include "modules/imu/imu_module.h"

#include <Arduino.h>

namespace {

void configureSpiBusPins(const imu::pins::SpiBusPins& spi_pins) {
  // 当前阶段先完成基础 GPIO 初始化。
  // 后续正式接入 SPI 驱动时，可在此基础上切换为 SPI 外设复用。
  pinMode(spi_pins.sck, OUTPUT);
  pinMode(spi_pins.mosi, OUTPUT);
  pinMode(spi_pins.miso, INPUT);

  digitalWrite(spi_pins.sck, LOW);
  digitalWrite(spi_pins.mosi, LOW);
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
  return sensor == imu::SensorType::kAccel
             ? imu::pins::kAccelSensor.chip_select
             : imu::pins::kGyroSensor.chip_select;
}

}  // namespace

namespace imu {

void begin() {
  configureSpiBusPins(pins::kSpiBus);
  configureChipSelectPin(pins::kAccelSensor);
  configureChipSelectPin(pins::kGyroSensor);
  configureInterruptPins(pins::kInterrupts);
}

void select(const SensorType sensor) {
  // 选中目标器件前，先释放全部片选，避免总线冲突。
  deselectAll();
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
