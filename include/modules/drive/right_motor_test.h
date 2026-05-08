#pragma once

#include "modules/drive/drive_motor_controller.h"

namespace drive {
namespace right_motor_test {

using Status = DriveMotorController::Status;

void init();
void update();
void processSerial();
void setOpenLoop(bool open_loop);
void setEnabled(bool enabled);
void setTargetVelocity(float velocity);
void emergencyStop();
void setVoltageLimit(float limit);
void setVelocityPid(float p, float i, float d, float lpf_tf);
void setCurrentLimit(float limit);
void setTorqueMode(DriveMotorController::TorqueMode mode);
float getAngle();
Status status();

}  // namespace right_motor_test
}  // namespace drive
