#pragma once

namespace drive {
namespace left_motor_test {

struct Status {
  bool initialized;
  bool foc_ready;
  bool enabled;
  bool emergency_stopped;
  bool open_loop;
  float target_velocity;
  float measured_velocity;
  float shaft_angle;
  float voltage_limit;
};

void init();
void update();
void processSerial();
void setOpenLoop(bool open_loop);
void setEnabled(bool enabled);
void setTargetVelocity(float velocity);
void emergencyStop();
void setVoltageLimit(float limit);
float getAngle();
Status status();

}  // namespace left_motor_test
}  // namespace drive
