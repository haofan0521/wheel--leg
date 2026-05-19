#pragma once

#include "modules/drive/drive_pins.h"

namespace drive {

// 初始化电驱相关 GPIO。
void begin();

// 控制电驱总使能引脚。
void setEnabled(bool enabled);

// 查询当前软件记录的电驱使能状态。
bool isEnabled();

// 将所有 PWM 输出拉低，避免误驱动。
void disableAllPwmOutputs();

// 读取电驱故障输入电平。
int readFaultLevel();

// 对外暴露左右电机引脚配置，便于后续驱动层复用。
const pins::ThreePhasePwmPins& leftMotorPwmPins();
const pins::ThreePhasePwmPins& rightMotorPwmPins();

}  // namespace drive
