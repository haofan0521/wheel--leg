#include "modules/drive/right_motor_test.h"

#include "modules/drive/drive_pins.h"
#include "modules/encoder/encoder_module.h"

namespace {

drive::DriveMotorController g_controller({
    .name = "right",
    .pole_pairs = 11,
    .pwm_pins = drive::pins::kRightMotorPwm,
    .sensor_provider = encoder::rightSensor,
    .read_angle = encoder::rightAngle,
    .read_velocity = encoder::rightVelocity,
    .pwm_channel_a = 3,
    .pwm_channel_b = 4,
    .pwm_channel_c = 5,
    .velocity_direction = 1.0f,
});

}  // namespace

namespace drive {
namespace right_motor_test {

void init() { g_controller.init(); }
void update() { g_controller.update(); }
void processSerial() { g_controller.processSerial(); }
void setOpenLoop(const bool open_loop) { g_controller.setOpenLoop(open_loop); }
void setEnabled(const bool enabled) { g_controller.setEnabled(enabled); }
void setTargetVelocity(const float velocity) { g_controller.setTargetVelocity(velocity); }
void emergencyStop() { g_controller.emergencyStop(); }
void setVoltageLimit(const float limit) { g_controller.setVoltageLimit(limit); }
void setVelocityPid(const float p, const float i, const float d, const float lpf_tf) {
  g_controller.setVelocityPid(p, i, d, lpf_tf);
}
float getAngle() { return g_controller.getAngle(); }
Status status() { return g_controller.status(); }

}  // namespace right_motor_test
}  // namespace drive
