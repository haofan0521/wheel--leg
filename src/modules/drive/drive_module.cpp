#include "modules/drive/drive_module.h"

#include <Arduino.h>

namespace {

// 记录当前软件层面的电驱使能状态。
bool g_drive_enabled = false;

void configurePwmPins(const drive::pins::ThreePhasePwmPins& pwm_pins) {
  // 三相 PWM 引脚先配置为输出，并默认拉低，避免上电误触发。
  pinMode(pwm_pins.phase_a, OUTPUT);
  pinMode(pwm_pins.phase_b, OUTPUT);
  pinMode(pwm_pins.phase_c, OUTPUT);

  digitalWrite(pwm_pins.phase_a, LOW);
  digitalWrite(pwm_pins.phase_b, LOW);
  digitalWrite(pwm_pins.phase_c, LOW);
}

}  // namespace

namespace drive {

void begin() {
  // 上电默认关闭电驱，待控制逻辑准备完成后再主动使能。
  pinMode(pins::kEnable, OUTPUT);
  digitalWrite(pins::kEnable, LOW);

  // 故障引脚用于读取电驱状态。
  pinMode(pins::kFault, INPUT);

  configurePwmPins(pins::kLeftMotorPwm);
  configurePwmPins(pins::kRightMotorPwm);

  g_drive_enabled = false;
}

void setEnabled(const bool enabled) {
  // 软件状态与硬件使能引脚保持同步。
  g_drive_enabled = enabled;
  digitalWrite(pins::kEnable, enabled ? HIGH : LOW);
}

bool isEnabled() {
  return g_drive_enabled;
}

void disableAllPwmOutputs() {
  // 在异常、停机或切换控制模式时，可快速关闭全部相位输出。
  digitalWrite(pins::kLeftMotorPwm.phase_a, LOW);
  digitalWrite(pins::kLeftMotorPwm.phase_b, LOW);
  digitalWrite(pins::kLeftMotorPwm.phase_c, LOW);
  digitalWrite(pins::kRightMotorPwm.phase_a, LOW);
  digitalWrite(pins::kRightMotorPwm.phase_b, LOW);
  digitalWrite(pins::kRightMotorPwm.phase_c, LOW);
}

int readFaultLevel() {
  // 当前直接返回 GPIO 电平，后续可根据驱动芯片逻辑封装为故障状态。
  return digitalRead(pins::kFault);
}

const pins::ThreePhasePwmPins& leftMotorPwmPins() {
  return pins::kLeftMotorPwm;
}

const pins::ThreePhasePwmPins& rightMotorPwmPins() {
  return pins::kRightMotorPwm;
}

}  // namespace drive
