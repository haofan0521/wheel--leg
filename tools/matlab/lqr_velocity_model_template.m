%% 轮腿机器人速度命令型 LQR 模板
% 适用当前 wheel--leg 工程的第一版 LQR 验证。
%
% 工程侧约定：
% 1. 控制任务周期为 1 ms：include/system/app_runtime_config.h::kControlTaskPeriodMs。
% 2. 平衡外环输出 output_velocity，并下发给左右轮 setTargetVelocity()。
% 3. wheel_velocity 应使用工程中已按前后方向归一化后的左右轮平均速度。
% 4. theta 和 theta_dot 在 MATLAB 内部统一使用 rad 和 rad/s。
%
% 第一版状态向量：
%   x = [theta; theta_dot; wheel_velocity]
%
% 第一版输入：
%   u = output_velocity，也就是固件里的目标轮速命令。
%
% 控制律：
%   u = -K * x
%
% 注意：如果你建的是力矩/推力输入模型，算出的 K 不能直接作为
% output_velocity 的反馈增益使用。

clear; clc; close all;

%% 基本配置
Ts = 0.001;  % s，当前工程控制周期为 1 ms

% 选择 LQR 计算方式：
%   "rough_model" : 用简化倒立摆 + 速度环模型粗算 K
%   "identified" : 从实车日志拟合离散 Ad/Bd 后计算 K
mode = "rough_model";

%% 路径 1：简化速度命令模型
% 模型：
%   theta_dot       = omega
%   omega_dot       = a * theta + b * u - c * omega
%   wheel_v_dot     = (u - wheel_v) / tau
%
% 参数含义：
%   a   : 车身小角度发散强度，由 m_body、l_com、I_pitch 估算
%   wheel_accel_to_theta_ddot : 车轮线加速度对车身角加速度的粗略影响
%   c   : 车身角速度阻尼
%   tau : 底层速度环响应时间常数，单位 s
%
% 这些值先用于起步，不是最终精确模型。后续应优先用实车日志辨识。
% Measured/estimated physical parameters for the rough model.
% l_com is the wheel axle to whole-robot COM height, currently about 31 cm.
g = 9.81;
r_wheel = 0.0255;
l_com = 0.31;
M_total = 1.30;
m_body = 1.02;
M_base = M_total - m_body;

% 轮轴以上机身的俯仰转动惯量，单位 kg*m^2。
% body_height 是轮轴以上主要质量在竖直方向的尺寸，不是轮轴到质心高度。
% body_depth 是轮轴以上主要质量在前后方向的尺寸，也就是车体侧视图里的厚度。
% 第一版按侧视矩形粗估：I = m * (height^2 + depth^2) / 12。
% 如果暂时没有精确尺寸，可以先测轮轴以上机身外形，再按质量集中区域略微修正。
body_height = 0.10;
body_depth = 0.11;
I_pitch = m_body * (body_height^2 + body_depth^2) / 12;

% WiFi Tau 测试结果，单位 s。
% 目前稳定结果大约为：左轮 52 ms，右轮 50 ms，平均约 52 ms。
% 单轨倒立摆粗模型先使用平均 tau；左右轮值保留用于检查两侧速度环一致性。
tau_left = 0.052;
tau_right = 0.050;
tau = 0.5 * (tau_left + tau_right);

J_pitch = I_pitch + m_body * l_com^2;
a = m_body * g * l_com / J_pitch;
wheel_accel_to_theta_ddot = m_body * l_com / J_pitch;
c = 1.5;

A_rough = [0,  1,                 0;
           a, -c,   wheel_accel_to_theta_ddot * r_wheel / tau;
           0,  0,            -1/tau];

B_rough = [0;
          -wheel_accel_to_theta_ddot * r_wheel / tau;
           1/tau];

fprintf("rough model parameters:\n");
fprintf("  r_wheel = %.4f m\n", r_wheel);
fprintf("  l_com = %.4f m\n", l_com);
fprintf("  M_total = %.3f kg, m_body = %.3f kg, M_base = %.3f kg\n", M_total, m_body, M_base);
fprintf("  I_pitch = %.5f kg*m^2, J_pitch = %.5f kg*m^2\n", I_pitch, J_pitch);
fprintf("  body_height = %.3f m, body_depth = %.3f m\n", body_height, body_depth);
fprintf("  tau_left = %.4f s, tau_right = %.4f s, tau = %.4f s\n", tau_left, tau_right, tau);

%% 路径 2：从实车日志辨识离散模型
% 日志 CSV 推荐列名：
%   time_ms,pitch_deg,target_pitch_deg,pitch_rate_dps,wheel_velocity,output_velocity
%
% pitch_deg 和 pitch_rate_dps 来自固件状态；
% wheel_velocity 使用固件里归一化后的平均轮速；
% output_velocity 使用平衡控制器实际输出。
log_file = "data.csv";

Ad_id = [];
Bd_id = [];

if mode == "identified"
    if ~isfile(log_file)
        error("找不到日志文件：%s。请把实车日志放到当前 MATLAB 工作目录。", log_file);
    end

    data = readtable(log_file);

    requireVars(data, ["pitch_deg", "target_pitch_deg", ...
                       "pitch_rate_dps", "wheel_velocity", ...
                       "output_velocity"]);

    raw_sample_count = height(data);
    theta_rad = deg2rad(data.pitch_deg - data.target_pitch_deg);
    theta_dot_rad_s = deg2rad(data.pitch_rate_dps);
    wheel_velocity = data.wheel_velocity;
    output_velocity = data.output_velocity;

    valid = isfinite(theta_rad) & isfinite(theta_dot_rad_s) & ...
            isfinite(wheel_velocity) & isfinite(output_velocity);

    if tableHasVar(data, "balance_enabled")
        valid = valid & logical(data.balance_enabled);
    end
    if tableHasVar(data, "balance_active")
        valid = valid & logical(data.balance_active);
    end
    if tableHasVar(data, "emergency_stopped")
        valid = valid & ~logical(data.emergency_stopped);
    end
    if tableHasVar(data, "balance_fault")
        valid = valid & ~logical(data.balance_fault);
    end

    if tableHasVar(data, "time_ms")
        valid_time_ms = data.time_ms(valid);
        if numel(valid_time_ms) >= 2
            log_Ts = median(diff(valid_time_ms)) / 1000.0;
        else
            log_Ts = NaN;
        end
    else
        log_Ts = NaN;
    end

    theta_rad = theta_rad(valid);
    theta_dot_rad_s = theta_dot_rad_s(valid);
    wheel_velocity = wheel_velocity(valid);
    output_velocity = output_velocity(valid);

    valid_sample_count = numel(theta_rad);
    fprintf("identified samples: raw=%d, valid=%d\n", raw_sample_count, valid_sample_count);
    if valid_sample_count < 30
        error("有效样本过少，无法可靠辨识。请延长采集时间，且避免保护触发。");
    end
    if isfinite(log_Ts)
        fprintf("log sample Ts median = %.4f s (%.1f Hz)\n", log_Ts, 1.0 / log_Ts);
        if abs(log_Ts - Ts) > 0.005
            warning("日志采样周期与固件控制周期不同。该 Ad/Bd 是日志采样周期下的模型，K 不宜直接上车。");
        end
    end
    fprintf("theta range = %.3f .. %.3f deg\n", rad2deg(min(theta_rad)), rad2deg(max(theta_rad)));
    fprintf("theta_dot range = %.3f .. %.3f deg/s\n", rad2deg(min(theta_dot_rad_s)), rad2deg(max(theta_dot_rad_s)));
    fprintf("wheel_velocity range = %.3f .. %.3f\n", min(wheel_velocity), max(wheel_velocity));
    fprintf("output_velocity range = %.3f .. %.3f\n", min(output_velocity), max(output_velocity));

    X = [theta_rad.'; theta_dot_rad_s.'; wheel_velocity.'];
    U = output_velocity.';

    X0 = X(:, 1:end-1);
    X1 = X(:, 2:end);
    U0 = U(:, 1:end-1);
    Phi = [X0; U0];

    % 最小二乘拟合：
    %   X1 = [Ad Bd] * [X0; U0]
    phi_rank = rank(Phi);
    phi_cond = cond(Phi);
    fprintf("regression rank = %d / %d, cond = %.3e\n", phi_rank, size(Phi, 1), phi_cond);
    if phi_rank < size(Phi, 1) || phi_cond > 1e6
        warning("回归矩阵病态或激励不足，辨识出的 Ad/Bd 可能不可信。");
    end

    AB = X1 / Phi;
    X1_pred = AB * Phi;
    rmse = sqrt(mean((X1 - X1_pred).^2, 2));
    fprintf("one-step RMSE: theta=%.6g rad, theta_dot=%.6g rad/s, wheel_v=%.6g\n", ...
            rmse(1), rmse(2), rmse(3));

    Ad_id = AB(:, 1:3);
    Bd_id = AB(:, 4);
    if max(abs(Ad_id(:))) > 10
        warning("Ad 元素量级过大，通常表示日志采样、符号、激励或闭环辨识存在问题。不要直接使用本次 K 上车。");
    end
end

%% 选择实际用于 LQR 的模型
switch mode
    case "rough_model"
        sysc = ss(A_rough, B_rough, eye(3), zeros(3, 1));
        sysd = c2d(sysc, Ts);
        Ad = sysd.A;
        Bd = sysd.B;

    case "identified"
        Ad = Ad_id;
        Bd = Bd_id;

    otherwise
        error("未知 mode：%s", mode);
end

%% 可控性检查
Co = ctrb(Ad, Bd);
rankCo = rank(Co);
fprintf("controllability rank = %d / %d\n", rankCo, size(Ad, 1));
if rankCo < size(Ad, 1)
    warning("模型不可完全控，K 可能不可用。请检查符号、日志质量或模型参数。");
end

%% LQR 权重
% 状态顺序：
%   1 theta(rad)
%   2 theta_dot(rad/s)
%   3 wheel_velocity
%
% 调参方向：
%   Q_theta 大：更强力拉回竖直
%   Q_theta_dot 大：更抑制摆动
%   Q_wheel_velocity 大：更抑制跑车
%   R 大：输出更保守
Q = diag([120, 4, 0.8]);

% 第一版实车验证先保守一些。
% R=2.5 时 5 deg 初始倾角需要约 19 rad/s，已经偏大；
% R=20 通常会把初始输出压到约 7 rad/s 量级，更适合首次小幅验证。
R = 20.0;

K = dlqr(Ad, Bd, Q, R);

fprintf("\nAd =\n");
disp(Ad);

fprintf("Bd =\n");
disp(Bd);

fprintf("K = [k_theta, k_theta_dot, k_wheel_velocity]\n");
disp(K);

%% 固件填参参考
% 固件中建议这样使用：
%
%   theta = (pitch_deg - target_pitch_deg) * PI / 180.0f;
%   theta_dot = pitch_rate_dps * PI / 180.0f;
%   wheel_v = wheel_velocity;
%
%   output_velocity = -(K(1) * theta +
%                       K(2) * theta_dot +
%                       K(3) * wheel_v);
%
% 然后继续乘 output_direction，并做 max_velocity 限幅。
fprintf("firmware gains:\n");
fprintf("  lqr_pitch = %.8g\n", K(1));
fprintf("  lqr_pitch_rate = %.8g\n", K(2));
fprintf("  lqr_wheel_velocity = %.8g\n", K(3));

%% 闭环离散极点
Acl = Ad - Bd * K;
fprintf("\nclosed-loop poles:\n");
disp(eig(Acl));

%% 首次上车缩放比例扫描
x0 = [deg2rad(5.0); 0; 0];
N = round(2.0 / Ts);
scale_candidates = 0.10:0.05:1.00;
scale_table = [];
for scale = scale_candidates
    K_scaled = scale * K;
    Acl_scaled = Ad - Bd * K_scaled;
    poles_scaled = eig(Acl_scaled);
    [~, Usim_scaled] = simulateClosedLoop(Ad, Bd, K_scaled, x0, N);
    scale_table = [scale_table; scale, max(abs(poles_scaled)), max(abs(Usim_scaled))]; %#ok<AGROW>
end

fprintf("\nfirst-test scale scan:\n");
fprintf("  scale   max|pole|   max|u| for 5 deg\n");
for i = 1:size(scale_table, 1)
    fprintf("  %.2f    %.6f    %.3f rad/s\n", ...
            scale_table(i, 1), scale_table(i, 2), scale_table(i, 3));
end

stable_scales = scale_table(scale_table(:, 2) < 1.0, :);
if isempty(stable_scales)
    warning("0.10 到 1.00 范围内没有找到稳定缩放比例，请检查模型符号或提高完整 K。");
    first_test_scale = 1.0;
else
    min_stable_scale = stable_scales(1, 1);
    first_test_scale = min(1.0, min_stable_scale + 0.10);
end

K_first_test = first_test_scale * K;
fprintf("\nrecommended first on-car test gains, %.0f%% of K:\n", first_test_scale * 100);
fprintf("  lqr_pitch = %.8g\n", K_first_test(1));
fprintf("  lqr_pitch_rate = %.8g\n", K_first_test(2));
fprintf("  lqr_wheel_velocity = %.8g\n", K_first_test(3));

Acl_first_test = Ad - Bd * K_first_test;
fprintf("first-test closed-loop poles:\n");
disp(eig(Acl_first_test));

%% 简单仿真：给一个初始倾角，看输出是否量级合理
[Xsim, Usim] = simulateClosedLoop(Ad, Bd, K, x0, N);
[Xsim_first_test, Usim_first_test] = simulateClosedLoop(Ad, Bd, K_first_test, x0, N);

fprintf("\nsimulation check:\n");
fprintf("  full K max |u| for 5 deg initial pitch = %.3f rad/s\n", max(abs(Usim)));
fprintf("  first-test K max |u| for 5 deg initial pitch = %.3f rad/s\n", max(abs(Usim_first_test)));
if max(abs(Usim)) > 10
    warning("完整 K 的 5 度初始倾角仿真输出超过 10 rad/s，不建议直接上车。首次验证请优先使用 first-test K，并保留固件 max_velocity 限幅。");
end
if any(abs(eig(Acl_first_test)) >= 1)
    warning("first-test K 闭环极点不全在单位圆内，当前缩放比例理论上不能稳定该粗模型。");
end
first_test_output_warning = 12.0;
if max(abs(Usim_first_test)) > first_test_output_warning
    warning("first-test K 的 5 度初始倾角仿真输出超过 %.1f rad/s。首次上车应保留较小 max_velocity 限幅，并先验证符号。", first_test_output_warning);
end

t = (0:N-1) * Ts;

figure;
subplot(4, 1, 1);
plot(t, rad2deg(Xsim(1, :)));
grid on;
ylabel("theta deg");

subplot(4, 1, 2);
plot(t, rad2deg(Xsim(2, :)));
grid on;
ylabel("theta dot deg/s");

subplot(4, 1, 3);
plot(t, Xsim(3, :));
grid on;
ylabel("wheel v");

subplot(4, 1, 4);
plot(t, Usim);
grid on;
ylabel("u");
xlabel("time s");

%% 本文件局部函数
function requireVars(data, names)
    existing = string(data.Properties.VariableNames);
    missing = names(~ismember(names, existing));
    if ~isempty(missing)
        error("CSV 缺少列：%s", strjoin(missing, ", "));
    end
end

function ok = tableHasVar(data, name)
    ok = any(strcmp(string(data.Properties.VariableNames), string(name)));
end

function [Xsim, Usim] = simulateClosedLoop(Ad, Bd, K, x0, N)
    Xsim = zeros(3, N);
    Usim = zeros(1, N);
    Xsim(:, 1) = x0;
    for k = 1:N-1
        Usim(k) = -K * Xsim(:, k);
        Xsim(:, k+1) = Ad * Xsim(:, k) + Bd * Usim(k);
    end
end
