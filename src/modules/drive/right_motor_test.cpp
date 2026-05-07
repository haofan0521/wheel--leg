#include "modules/drive/right_motor_test.h"

namespace drive {
namespace right_motor_test {

void init() {}
void update() {}
void processSerial() {}
void setOpenLoop(bool /*open_loop*/) {}
void setEnabled(bool /*enabled*/) {}
void setTargetVelocity(float /*velocity*/) {}
void emergencyStop() {}
void setVoltageLimit(float /*limit*/) {}

float getAngle() {
  return 0.0f;
}

Status status() {
  Status current = {};
  return current;
}

}  // namespace right_motor_test
}  // namespace drive
