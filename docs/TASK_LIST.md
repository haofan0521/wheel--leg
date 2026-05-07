# 轮腿机器人固件任务列表

## 任务状态

| 编号 | 任务 | 状态 | 目标文件 |
| --- | --- | --- | --- |
| `Task 1` | 新增 Wi-Fi 功能，连接手机 Wi-Fi，提供浏览器调试界面 | `已完成` | `include/config/wifi_debug_config.h` `include/system/wifi_debug_server.h` `src/system/wifi_debug_server.cpp` `src/main.cpp` |
| `Task 2` | 新建电驱模块文件，定义电驱相关引脚 | `已完成` | `include/modules/drive/drive_pins.h` `include/modules/drive/drive_module.h` `src/modules/drive/drive_module.cpp` |
| `Task 3` | 新建编码器模块文件，定义编码器相关引脚 | `已完成` | `include/modules/encoder/encoder_pins.h` `include/modules/encoder/encoder_module.h` `src/modules/encoder/encoder_module.cpp` |
| `Task 4` | 新建 IMU 模块文件，定义 IMU 相关引脚 | `已完成` | `include/modules/imu/imu_pins.h` `include/modules/imu/imu_module.h` `src/modules/imu/imu_module.cpp` |
| `Task 5` | 新建舵机模块文件，定义舵机相关引脚 | `已完成` | `include/modules/servo/servo_pins.h` `include/modules/servo/servo_module.h` `src/modules/servo/servo_module.cpp` |
| `Task 6` | 实现电驱功能，接入控制任务并完善底层控制接口 | `待完成` | `include/modules/drive/drive_module.h` `src/modules/drive/drive_module.cpp` `src/system/app_runtime.cpp` |
| `Task 7` | 实现 `BMI088` 功能，完成初始化、寄存器配置与数据读取 | `待完成` | `include/modules/imu/imu_module.h` `src/modules/imu/imu_module.cpp` `src/system/app_runtime.cpp` |
| `Task 8` | 实现编码器功能，完成 `MT6835` SPI 通信与角度读取 | `待完成` | `include/modules/encoder/encoder_module.h` `src/modules/encoder/encoder_module.cpp` `src/system/app_runtime.cpp` |

## 当前目录规划

```text
docs/
  TASK_LIST.md

include/
  config/
    wifi_debug_config.h
  system/
    wifi_debug_server.h
  modules/
    drive/
    encoder/
    imu/
    servo/

src/
  main.cpp
  system/
    wifi_debug_server.cpp
  modules/
    drive/
    encoder/
    imu/
    servo/
```

## Task 1 说明

- 工作模式：优先以 `STA` 模式连接手机热点或手机共享 Wi-Fi。
- 回退策略：若没有配置账号密码，或在超时时间内连接失败，则自动切换到 `AP` 调试模式。
- 调试方式：通过浏览器访问设备 IP，打开简易调试页面查看联网状态、IP、RSSI、运行时间、堆内存等信息。

## 代码要求

- 已完成任务与后续新增任务中的代码，统一补充中文注释。
- 注释重点说明模块职责、引脚用途、初始化目的和关键流程，不写无意义注释。

## 当前运行时架构

- `Core 1`：控制任务，负责电驱、编码器、IMU、舵机等实时控制链路
- `Core 0`：服务任务，负责 Wi-Fi、网页调试与后续遥控/遥测服务
- 网页调试接口只读取共享状态快照，不直接参与实时控制
- 当前已完成双核任务框架重构，后续可在控制任务中继续接入 FOC 主循环

## 后续执行顺序

1. 实现电驱功能，完善电驱使能、PWM 输出与控制任务接入。
2. 实现 `BMI088` 功能，完成初始化、寄存器配置与数据读取。
3. 实现编码器功能，完成 `MT6835` SPI 通信与角度读取。
