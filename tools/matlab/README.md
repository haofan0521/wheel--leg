# MATLAB LQR 建模说明

本目录用于离线计算轮腿机器人第一版 LQR 增益。当前工程的平衡外环输出是 `output_velocity`，随后下发给左右轮 `setTargetVelocity()`，因此第一版模板采用“速度命令型 LQR”，不是力矩型 LQR。

## 文件

- `lqr_velocity_model_template.m`：MATLAB 脚本模板，支持粗模型计算和实车日志辨识两种路径。

## 第一版模型约定

状态向量：

```text
x = [theta;
     theta_dot;
     wheel_velocity]
```

输入：

```text
u = output_velocity
```

单位约定：

```text
theta          rad
theta_dot      rad/s
wheel_velocity 与固件里的 wheel_velocity 保持一致
output_velocity 与固件里的 output_velocity 保持一致
```

固件里 `pitch_deg` 和 `pitch_rate_dps` 是角度制，写入 LQR 前需要转成弧度制。

## 需要测量或确认的实车参数

第一阶段至少需要这些参数：

| 参数 | 目的 | 建议测法 |
| --- | --- | --- |
| `Ts` 控制周期 | 离散 LQR 的采样周期 | 当前工程为 `1 ms`，来自 `include/system/app_runtime_config.h` |
| `target_pitch_deg` | 平衡点零偏 | 用现有 PID 基本站住后，从 WiFi 状态读稳定时的目标 Pitch |
| `pitch_deg` 正方向 | 判断车身前倒/后倒符号 | 手扶车体小角度前倾，观察 WiFi Pitch 增大还是减小 |
| `output_velocity` 正方向 | 判断控制输出推动方向 | 手动给 `/api/motor?side=both&v=小正值`，观察轮子推车向前还是向后 |
| `wheel_velocity` 正方向 | 判断速度反馈符号 | 手推车体向前滚动，观察状态中的 `wheel_velocity` 正负 |
| 底层速度环响应时间 `tau` | 粗模型中的速度环动态 | 给小阶跃目标轮速，记录实际轮速到达 63% 目标值的时间 |
| 轮半径 `r` | 后续从角速度换算线速度 | 用尺测轮子半径，单位 m |
| 轮轴到质心高度 `l` | 后续物理倒立摆模型 | 调到平衡姿态后，估计轮轴到整机质心的垂直距离，单位 m |
| 整机质量 `M_total` | 后续物理模型 | 称重，单位 kg |
| 上半身等效质量 `m_body` | 后续物理模型 | 可先估计轮轴以上主要质量，单位 kg |
| 俯仰转动惯量 `I_pitch` | 后续物理模型 | 第一版可先粗估，后续再辨识 |
| 腿部高度/足端 X | 模型适用姿态 | 记录测试时 WiFi 页面上的腿高和 `target_x` |
| 电池电压 | 输出能力变化 | 测试时记录电压，避免低电压数据混入 |

其中 `Ts`、三个符号方向、`tau`、当前平衡点 `target_pitch_deg` 最关键。质量、质心、转动惯量主要用于后续物理模型，不影响第一版速度命令型辨识。

## 推荐日志列

后续做实车辨识时，CSV 推荐列名：

```text
time_ms,pitch_deg,target_pitch_deg,pitch_rate_dps,wheel_velocity,output_velocity
```

可选增加：

```text
left_measured_velocity,right_measured_velocity,left_target_velocity,right_target_velocity,balance_active,emergency_stopped,battery_mv,target_x,target_height
```

## 通过 WiFi 生成日志

当前固件的 `/api/status` 已经返回 `balance.pitch`、`balance.targetPitch`、`balance.pitchRate`、`balance.wheelVelocity` 和 `balance.outputVelocity`。电脑连上机器人同一个 WiFi 后，可以用本目录的脚本采集：

MATLAB 方式：

```matlab
run("tools/matlab/collect_balance_log_matlab.m")
```

如果机器人 IP 不是 `192.168.4.1`，先打开脚本修改 `robot_host`。

Python 方式：

```bash
python tools/matlab/collect_balance_log.py --host 192.168.4.1 --duration 20 --rate 20 --output balance_log.csv
```

如果手机热点或路由器分配了其他 IP，就把 `--host` 换成 WiFi 调试页面上显示的 IP。若 mDNS 可用，也可以直接尝试：

```bash
python tools/matlab/collect_balance_log.py --host wheel-leg-debug.local --duration 20 --rate 20 --output balance_log.csv
```

建议第一次采集先用 `10-20 Hz`，采 `15-30 s`。这个频率只适合粗略观察状态范围和符号方向，不建议作为最终 LQR 辨识数据源。

## 通过串口生成高频日志

当前固件已预留串口 CSV 遥测开关：

```cpp
include/system/app_runtime_config.h
```

```cpp
constexpr bool kEnableVofaTelemetry = true;
constexpr uint32_t kVofaTelemetryDecimation = 10;
```

控制任务周期是 `1 ms`，`decimation = 10` 时串口约 `100 Hz` 输出一行 CSV。串口波特率在 `src/main.cpp` 中为 `921600`。

串口输出第一行是表头：

```text
time_ms,loop_counter,pitch_deg,target_pitch_deg,pitch_rate_dps,wheel_velocity,output_velocity,balance_enabled,balance_active,balance_fault,kp,kd,kv,direction,max_velocity,left_target_velocity,left_measured_velocity,right_target_velocity,right_measured_velocity,left_forward_velocity,right_forward_velocity
```

使用方法：

1. 重新烧录固件。
2. 用串口助手或 PlatformIO Serial Monitor 连接 USB CDC 串口，波特率 `921600`。
3. 等表头出现后开始保存日志。
4. 开启当前 PD 平衡，让车稳定后轻轻给几次小扰动。
5. 保存 `15-30 s` 日志。
6. 将纯 CSV 部分保存为 `balance_log.csv`，放到 MATLAB 当前工作目录。

如果串口日志中混有启动提示或其他非 CSV 行，整理时保留从表头开始的 CSV 内容即可。

## MATLAB 使用顺序

1. 先打开 `lqr_velocity_model_template.m`。
2. 保持 `mode = "rough_model"`，运行一次，确认脚本能输出 `K`。
3. 记录实车平衡日志后，把 CSV 命名为 `balance_log.csv` 并放到 MATLAB 当前工作目录。
4. 修改脚本为 `mode = "identified"`。
5. 重新运行，使用辨识得到的 `Ad/Bd` 计算 `K`。
6. 将输出的 `lqr_pitch`、`lqr_pitch_rate`、`lqr_wheel_velocity` 填回固件或 WiFi LQR 调参入口。

## 安全提醒

第一版 LQR 上车前应保留现有 PD 模式作为回退路径，并继续使用当前工程的 `max_velocity`、`start_angle_deg`、`max_angle_deg` 和急停逻辑。若一开启就往同一方向冲，优先检查 `pitch`、`output_velocity`、`wheel_velocity` 的符号方向，不要先盲目调 `Q/R`。
