#include "modules/encoder/encoder_module.h"

#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>

namespace {

SPIClass* g_spi_bus = nullptr;
constexpr float kVelocityLpfTf = 0.02f;

void configureSpiBusPins(const encoder::pins::SpiBusPins& spi_pins) {
  if (g_spi_bus == nullptr) {
    g_spi_bus = new SPIClass(FSPI);
    g_spi_bus->begin(spi_pins.sck, spi_pins.miso, spi_pins.mosi, -1);
  }
}

class MT6835Sensor : public Sensor {
 public:
  explicit MT6835Sensor(uint8_t cs_pin) : cs_pin_(cs_pin) {}

  void init() override {
    pinMode(cs_pin_, OUTPUT);
    digitalWrite(cs_pin_, HIGH);
    Sensor::init();
  }

  float getSensorAngle() override {
    const uint32_t data_24bit = readRawFrame();
    const uint32_t angle_raw_21bit = data_24bit >> 3;
    return static_cast<float>(angle_raw_21bit) * _2PI / 2097152.0f;
  }

  uint32_t readRawFrame() {
    if (!g_spi_bus) return 0;

    uint8_t rx_buf[6] = {0};
    g_spi_bus->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(cs_pin_, LOW);
    delayMicroseconds(1);

    rx_buf[0] = g_spi_bus->transfer(0xA0);
    rx_buf[1] = g_spi_bus->transfer(0x00);
    rx_buf[2] = g_spi_bus->transfer(0x00);
    rx_buf[3] = g_spi_bus->transfer(0x00);
    rx_buf[4] = g_spi_bus->transfer(0x00);
    rx_buf[5] = g_spi_bus->transfer(0x00);

    delayMicroseconds(1);
    digitalWrite(cs_pin_, HIGH);
    g_spi_bus->endTransaction();

    return (static_cast<uint32_t>(rx_buf[2]) << 16) |
           (static_cast<uint32_t>(rx_buf[3]) << 8) |
           static_cast<uint32_t>(rx_buf[4]);
  }

 private:
  uint8_t cs_pin_;
};

MT6835Sensor g_left_sensor(encoder::pins::kLeftSensor.chip_select);
MT6835Sensor g_right_sensor(encoder::pins::kRightSensor.chip_select);
float g_left_filtered_velocity = 0.0f;
float g_right_filtered_velocity = 0.0f;
uint32_t g_left_velocity_update_us = 0;
uint32_t g_right_velocity_update_us = 0;
bool g_left_velocity_ready = false;
bool g_right_velocity_ready = false;

float filterVelocity(const float raw_velocity,
                     float& filtered_velocity,
                     uint32_t& update_us,
                     bool& ready) {
  const uint32_t now_us = micros();
  if (!ready) {
    filtered_velocity = raw_velocity;
    update_us = now_us;
    ready = true;
    return filtered_velocity;
  }

  float dt = static_cast<float>(now_us - update_us) * 1e-6f;
  update_us = now_us;
  if (dt <= 0.0f || dt > 0.5f) {
    dt = 1e-3f;
  }

  const float alpha = dt / (kVelocityLpfTf + dt);
  filtered_velocity += alpha * (raw_velocity - filtered_velocity);
  return filtered_velocity;
}

}  // namespace

namespace encoder {

void begin() {
  configureSpiBusPins(pins::kSpiBus);
  g_left_sensor.init();
  g_right_sensor.init();
}

Sensor* leftSensor() {
  return &g_left_sensor;
}

Sensor* rightSensor() {
  return &g_right_sensor;
}

float leftAngle() {
  g_left_sensor.update();
  return g_left_sensor.getAngle();
}

float leftVelocity() {
  g_left_sensor.update();
  return filterVelocity(g_left_sensor.getVelocity(),
                        g_left_filtered_velocity,
                        g_left_velocity_update_us,
                        g_left_velocity_ready);
}

float rightAngle() {
  g_right_sensor.update();
  return g_right_sensor.getAngle();
}

float rightVelocity() {
  g_right_sensor.update();
  return filterVelocity(g_right_sensor.getVelocity(),
                        g_right_filtered_velocity,
                        g_right_velocity_update_us,
                        g_right_velocity_ready);
}

void testPrintEncoders() {
  g_left_sensor.update();
  g_right_sensor.update();

  const float left_angle_deg = g_left_sensor.getAngle() * 180.0f / _PI;
  const float left_vel_rad_s = filterVelocity(g_left_sensor.getVelocity(),
                                              g_left_filtered_velocity,
                                              g_left_velocity_update_us,
                                              g_left_velocity_ready);
  const float right_angle_deg = g_right_sensor.getAngle() * 180.0f / _PI;
  const float right_vel_rad_s = filterVelocity(g_right_sensor.getVelocity(),
                                               g_right_filtered_velocity,
                                               g_right_velocity_update_us,
                                               g_right_velocity_ready);

  Serial.printf("[Encoder] L Pos: %.2f deg | L Vel: %.2f rad/s | R Pos: %.2f deg | R Vel: %.2f rad/s\n",
                left_angle_deg,
                left_vel_rad_s,
                right_angle_deg,
                right_vel_rad_s);
}

void testReadLeftEncoder() {
  g_left_sensor.update();
  const uint32_t raw_frame = g_left_sensor.readRawFrame();
  const uint32_t raw_angle = raw_frame >> 3;
  const float angle_deg = static_cast<float>(raw_angle) * 360.0f / 2097152.0f;

  Serial.printf("[Encoder] L raw24: 0x%06lX | raw21: %lu | angle: %.2f deg | vel: %.2f rad/s\n",
                static_cast<unsigned long>(raw_frame),
                static_cast<unsigned long>(raw_angle),
                angle_deg,
                filterVelocity(g_left_sensor.getVelocity(),
                               g_left_filtered_velocity,
                               g_left_velocity_update_us,
                               g_left_velocity_ready));
}

void testReadRightEncoder() {
  g_right_sensor.update();
  const uint32_t raw_frame = g_right_sensor.readRawFrame();
  const uint32_t raw_angle = raw_frame >> 3;
  const float angle_deg = static_cast<float>(raw_angle) * 360.0f / 2097152.0f;

  Serial.printf("[Encoder] R raw24: 0x%06lX | raw21: %lu | angle: %.2f deg | vel: %.2f rad/s\n",
                static_cast<unsigned long>(raw_frame),
                static_cast<unsigned long>(raw_angle),
                angle_deg,
                filterVelocity(g_right_sensor.getVelocity(),
                               g_right_filtered_velocity,
                               g_right_velocity_update_us,
                               g_right_velocity_ready));
}

}  // namespace encoder
