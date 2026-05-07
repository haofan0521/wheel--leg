#pragma once

namespace drive {
namespace right_motor_test {

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

// 初始化右轮电机（编码器闭环 FOC 模式）
void init();

// 在控制循环中周期调用，更新闭环 FOC 与目标速度
void update();

// 检查串口输入并设置目标速度（非阻塞）
void processSerial();

// 设置右轮控制模式，true 为开环速度测试，false 为闭环 FOC。
void setOpenLoop(bool open_loop);

// 设置右轮 FOC 输出使能
void setEnabled(bool enabled);

// 设置闭环目标速度 (rad/s)
void setTargetVelocity(float velocity);

// 急停并关闭右轮输出。
void emergencyStop();

// 设置闭环控制电压限制 (V)
void setVoltageLimit(float limit);

// 获取当前电机角度 (rad)
float getAngle();

// 获取右轮闭环状态。
Status status();

}  // namespace right_motor_test
}  // namespace drive
