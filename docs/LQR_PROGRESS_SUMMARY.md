# LQR 控制阶段总结

本文档记录当前轮腿平衡控制从 PID/PD+Kv 过渡到 LQR 的阶段性工作，便于后续继续调试和回退。

## 当前目标

在不覆盖原有 PID/PD+Kv 平衡控制的前提下，引入一版可实车验证的 LQR 控制模式。

当前控制模式可以通过 WiFi 调试页面切换：

- `PID/PD+Kv`：保留原有平衡控制路径。
- `LQR`：新增的 LQR 速度命令控制路径。

## LQR 模型约定

当前采用的是“速度命令输入”的简化模型，不是直接以电机力矩作为输入。

状态量：

```text
x = [theta, theta_dot, wheel_velocity]
```

含义：

- `theta`：车身 Pitch 误差，单位 `rad`
- `theta_dot`：Pitch 角速度，单位 `rad/s`
- `wheel_velocity`：左右轮前进方向平均轮速，单位 `rad/s`

控制输入：

```text
u = output_velocity
```

含义：

- `output_velocity` 是下发给左右轮速度环的目标轮速，单位 `rad/s`

固件中的 LQR 公式：

```cpp
control_velocity = -(lqr_pitch * theta +
                     lqr_pitch_rate * theta_dot +
                     lqr_wheel_velocity * wheel_velocity);

output_velocity = output_direction * control_velocity;
```

Pitch 修正通过 `target_pitch_deg` 完成：

```text
theta = pitch_deg - target_pitch_deg
```

因此 PID 调出来的机械平衡角，例如 `1.2 deg`，应直接填到 WiFi 页面里的“目标 Pitch”，而不是额外加到 IMU 原始值或 LQR 公式中。

## 物理建模参数

MATLAB 脚本位置：

```text
tools/matlab/lqr_velocity_model_template.m
```

当前用于粗模型的主要参数：

```text
r_wheel = 0.0255 m
l_com = 0.3100 m
M_total = 1.300 kg
m_body = 1.020 kg
M_base = 0.280 kg
body_height = 0.100 m
body_depth = 0.110 m
tau_left = 0.0520 s
tau_right = 0.0500 s
tau = 0.0510 s
```

当前 MATLAB 粗模型推荐过的第一版实车参数约为：

```text
LQR Pitch       = -119.86971
LQR Pitch Rate  = -18.810969
LQR Wheel V     = -1.208787
```

这些参数的单位不是 PID 的“每度输出”，其中 `LQR Pitch` 对应的是 `rad` 下的车身角度反馈增益。

## 固件已完成内容

### 1. LQR 模式接入

核心文件：

```text
include/modules/balance/balance_controller.h
src/modules/balance/balance_controller.cpp
include/system/runtime_state.h
src/system/runtime_state.cpp
src/system/app_runtime.cpp
src/system/wifi_debug_server.cpp
```

已增加的主要参数：

```text
use_lqr
lqr_pitch
lqr_pitch_rate
lqr_wheel_velocity
lqr_output_slew_rate
```

默认状态：

```text
use_lqr = false
```

也就是说烧录后默认仍走原 PID/PD+Kv 路径，LQR 需要在 WiFi 页面手动切换。

### 2. WiFi 页面调参

WiFi 调试页面已经增加：

```text
控制模式
LQR Pitch
LQR Pitch Rate
LQR Wheel V
LQR 爬坡(rad/s^2)
```

`/api/balance` 支持的相关参数：

```text
mode=lqr 或 mode=pid
lqrp
lqrd
lqrv
lqrslew
```

LQR 参数支持保存到 NVS：

```text
use_lqr
lqr_p
lqr_d
lqr_v
lqr_slew
```

### 3. LQR 输出爬坡限制

为了避免 LQR 目标轮速突变导致电机抖动或电驱保护，新增了 LQR 专用输出爬坡限制。

参数：

```text
lqr_output_slew_rate
```

单位：

```text
rad/s^2
```

默认值：

```text
80.0
```

该限制只在 LQR 模式生效。PID/PD+Kv 模式仍保持原输出方式。

调参含义：

- 值小：平衡点附近更温和，但抗扰动较弱。
- 值大：扰动响应更快，但更容易放大抖动。

## 轮速方向约定

固件中用于平衡控制的车体前进方向轮速为：

```text
left_forward_velocity  = encoder::leftVelocity()  * kLeftForwardVelocitySign
right_forward_velocity = encoder::rightVelocity() * kRightForwardVelocitySign
wheel_velocity = 0.5 * (left_forward_velocity + right_forward_velocity)
```

当前需要注意：

- 右轮实际前进方向和原始编码器/电机方向可能相反。
- Tau 测试和日志中右轮前进速度曾按 `-right_motor.measured_velocity` 处理。
- 不应直接混用原始左右轮速度作为 LQR 的 `wheel_velocity`。

## 当前实车现象

当前 LQR 已经可以基本平衡，但还存在两个问题：

```text
1. 平衡点附近有抖动。
2. 抗外部扰动能力不够。
```

已观察到的规律：

```text
LQR 爬坡 = 80:
  勉强能平衡，抖动较小，但抗扰动不强。

LQR 爬坡增大:
  抗扰动增强，但抖动明显变大，严重时无法平衡。
```

这说明当前主要问题不是 LQR 方向完全错误，而是：

```text
大扰动响应 和 小信号抖动 被同一组参数同时放大了。
```

## 当前推荐调试方向

后续不要只继续增大 `LQR 爬坡`，应同时处理 Pitch Rate 噪声和速度环响应。

建议优先测试：

```text
目标 Pitch: 以 PID 已验证的机械平衡角为准，例如 1.20 deg
LQR Pitch: -119.87
LQR Pitch Rate: -14.0
LQR Wheel V: -1.20
LQR 爬坡: 120
最大轮速: 6 ~ 8 rad/s
```

如果平衡点附近仍明显抖动：

```text
LQR Pitch Rate: -14 -> -12
速度环 P: 0.5 -> 0.3
速度环 I: 5.0 -> 2.0
速度 LPF: 0.01 -> 0.02 或 0.03
```

如果抗扰动仍不足：

```text
LQR Pitch: -119.87 -> -135 -> -150
LQR 爬坡: 120 -> 160
```

如果推一下后轮子持续跑远：

```text
LQR Wheel V: -1.20 -> -1.50 -> -1.80
```

## 后续可做的固件改进

如果调参仍然表现为“抗扰动和抖动互相打架”，建议下一步增加：

```text
LQR Pitch Rate 低通滤波
```

原因：

- `pitch_rate_dps` 来自陀螺仪，噪声较容易放大。
- 当前提高 `LQR 爬坡` 后，小的角速度噪声会更快传递到电机目标轮速。
- 对 Pitch Rate 做低通，比直接给输出加死区更合理，因为死区会削弱平衡点附近必要的小修正。

建议新增参数：

```text
lqr_rate_lpf_tf
```

初始可测试：

```text
0.01 ~ 0.03 s
```

## 回退方式

如果 LQR 实车表现异常，可以直接在 WiFi 页面切回：

```text
控制模式 = PID/PD+Kv
```

由于 `use_lqr` 默认是 `false`，且 PID 公式未被替换，原有 PID/PD+Kv 调试路径仍保留。

