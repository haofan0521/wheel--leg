#include "modules/encoder/encoder_module.h"

#include <Arduino.h>

namespace {

void configureSpiBusPins(const encoder::pins::SpiBusPins& spi_pins) {
  // 当前阶段先完成基础 GPIO 初始化。
  // 后续正式接入 SPI 驱动时，可在此基础上切换为 SPI 外设复用。
  pinMode(spi_pins.sck, OUTPUT);
  pinMode(spi_pins.mosi, OUTPUT);
  pinMode(spi_pins.miso, INPUT);

  digitalWrite(spi_pins.sck, LOW);
  digitalWrite(spi_pins.mosi, LOW);
}

void configureChipSelectPin(const encoder::pins::SensorPins& sensor_pins) {
  // MT6835 使用低电平片选，因此初始化后默认拉高。
  pinMode(sensor_pins.chip_select, OUTPUT);
  digitalWrite(sensor_pins.chip_select, HIGH);
}

uint8_t chipSelectPinFromSide(const encoder::Side side) {
  return side == encoder::Side::kLeft
             ? encoder::pins::kLeftSensor.chip_select
             : encoder::pins::kRightSensor.chip_select;
}

}  // namespace

namespace encoder {

void begin() {
  configureSpiBusPins(pins::kSpiBus);
  configureChipSelectPin(pins::kLeftSensor);
  configureChipSelectPin(pins::kRightSensor);
}

void select(const Side side) {
  // 选中目标编码器前，先释放所有片选，确保同一时刻只有一个器件挂到总线上。
  deselectAll();
  digitalWrite(chipSelectPinFromSide(side), LOW);
}

void deselect(const Side side) {
  digitalWrite(chipSelectPinFromSide(side), HIGH);
}

void deselectAll() {
  digitalWrite(pins::kLeftSensor.chip_select, HIGH);
  digitalWrite(pins::kRightSensor.chip_select, HIGH);
}

const pins::SpiBusPins& spiBusPins() {
  return pins::kSpiBus;
}

const pins::SensorPins& leftSensorPins() {
  return pins::kLeftSensor;
}

const pins::SensorPins& rightSensorPins() {
  return pins::kRightSensor;
}

}  // namespace encoder
