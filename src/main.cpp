#include <Arduino.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "system/app_runtime.h"

namespace {

// 串口用于启动日志与早期调试输出。
constexpr uint32_t kSerialBaudRate = 115200;

}  // namespace

void setup() {
  Serial.begin(kSerialBaudRate);
  delay(300);

  Serial.println();
  Serial.println("Wheel-leg robot booting...");

  // 启动双核运行时框架。
  // 控制链路与 Wi-Fi/调试链路在此之后分任务运行。
  app_runtime::begin();
}

void loop() {
  // Arduino 默认 loop 任务在当前工程中不再承担主业务逻辑，
  // 仅保持空闲，让 FreeRTOS 任务接管系统调度。
  vTaskDelay(pdMS_TO_TICKS(1000));
}
