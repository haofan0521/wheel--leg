%% 通过 WiFi 调试接口采集 balance_log.csv
% 使用前：
% 1. 电脑连接到机器人 WiFi 或与机器人处于同一局域网。
% 2. 打开浏览器确认 http://<robot_host>/api/status 能访问。
% 3. 修改下面的 robot_host、duration_s 和 rate_hz。

clear; clc;

robot_host = "10.248.243.196";          % 可改成 WiFi 页面显示的 IP
% robot_host = "wheel-leg-debug.local";
duration_s = 20;                     % 采集时长
rate_hz = 20;                        % 采样频率，建议 10-30 Hz
output_file = "balance_log.csv";

if startsWith(robot_host, "http://") || startsWith(robot_host, "https://")
    status_url = robot_host + "/api/status";
else
    status_url = "http://" + robot_host + "/api/status";
end

opts = weboptions("Timeout", 1.0);
period_s = 1.0 / rate_hz;
max_samples = ceil(duration_s * rate_hz);

time_ms = zeros(max_samples, 1);
pc_time_s = zeros(max_samples, 1);
sample_index = zeros(max_samples, 1);
pitch_deg = nan(max_samples, 1);
target_pitch_deg = nan(max_samples, 1);
pitch_rate_dps = nan(max_samples, 1);
wheel_velocity = nan(max_samples, 1);
output_velocity = nan(max_samples, 1);
balance_active = false(max_samples, 1);
balance_enabled = false(max_samples, 1);
emergency_stopped = false(max_samples, 1);
balance_fault = false(max_samples, 1);
kp = nan(max_samples, 1);
kd = nan(max_samples, 1);
kv = nan(max_samples, 1);
direction = nan(max_samples, 1);
max_velocity = nan(max_samples, 1);
start_angle_deg = nan(max_samples, 1);
max_angle_deg = nan(max_samples, 1);
left_target_velocity = nan(max_samples, 1);
left_measured_velocity = nan(max_samples, 1);
right_target_velocity = nan(max_samples, 1);
right_measured_velocity = nan(max_samples, 1);

fprintf("status url: %s\n", status_url);
fprintf("duration: %.1f s, rate: %.1f Hz\n", duration_s, rate_hz);

t0 = tic;
count = 0;
errors = 0;

while toc(t0) < duration_s && count < max_samples
    loop_start = tic;
    count = count + 1;

    try
        data = webread(status_url, opts);
        b = data.balance;
        l = data.leftMotor;
        r = data.rightMotor;

        time_ms(count) = round(toc(t0) * 1000);
        pc_time_s(count) = posixtime(datetime("now"));
        sample_index(count) = count - 1;

        pitch_deg(count) = fieldOrNan(b, "pitch");
        target_pitch_deg(count) = fieldOrNan(b, "targetPitch");
        pitch_rate_dps(count) = fieldOrNan(b, "pitchRate");
        wheel_velocity(count) = fieldOrNan(b, "wheelVelocity");
        output_velocity(count) = fieldOrNan(b, "outputVelocity");
        balance_active(count) = fieldOrFalse(b, "active");
        balance_enabled(count) = fieldOrFalse(b, "enabled");
        emergency_stopped(count) = fieldOrFalse(b, "emergencyStopped");
        balance_fault(count) = fieldOrFalse(b, "fault");
        kp(count) = fieldOrNan(b, "kp");
        kd(count) = fieldOrNan(b, "kd");
        kv(count) = fieldOrNan(b, "kv");
        direction(count) = fieldOrNan(b, "direction");
        max_velocity(count) = fieldOrNan(b, "maxVelocity");
        start_angle_deg(count) = fieldOrNan(b, "startAngle");
        max_angle_deg(count) = fieldOrNan(b, "maxAngle");

        left_target_velocity(count) = fieldOrNan(l, "targetVelocity");
        left_measured_velocity(count) = fieldOrNan(l, "measuredVelocity");
        right_target_velocity(count) = fieldOrNan(r, "targetVelocity");
        right_measured_velocity(count) = fieldOrNan(r, "measuredVelocity");
    catch err
        errors = errors + 1;
        fprintf("sample %d error: %s\n", count, err.message);
    end

    pause(max(0, period_s - toc(loop_start)));
end

if count == 0
    error("没有采集到有效样本。请先确认浏览器能打开 %s。", status_url);
end

idx = 1:count;
log_table = table( ...
    time_ms(idx), pc_time_s(idx), sample_index(idx), ...
    pitch_deg(idx), target_pitch_deg(idx), pitch_rate_dps(idx), ...
    wheel_velocity(idx), output_velocity(idx), ...
    balance_active(idx), balance_enabled(idx), emergency_stopped(idx), balance_fault(idx), ...
    kp(idx), kd(idx), kv(idx), direction(idx), max_velocity(idx), ...
    start_angle_deg(idx), max_angle_deg(idx), ...
    left_target_velocity(idx), left_measured_velocity(idx), ...
    right_target_velocity(idx), right_measured_velocity(idx), ...
    'VariableNames', { ...
        'time_ms', 'pc_time_s', 'sample_index', ...
        'pitch_deg', 'target_pitch_deg', 'pitch_rate_dps', ...
        'wheel_velocity', 'output_velocity', ...
        'balance_active', 'balance_enabled', 'emergency_stopped', 'balance_fault', ...
        'kp', 'kd', 'kv', 'direction', 'max_velocity', ...
        'start_angle_deg', 'max_angle_deg', ...
        'left_target_velocity', 'left_measured_velocity', ...
        'right_target_velocity', 'right_measured_velocity' ...
    });

writetable(log_table, char(output_file));
fprintf("done: %d samples, %d errors, file: %s\n", count, errors, output_file);

function value = fieldOrNan(s, name)
    if isfield(s, name)
        value = double(s.(name));
    else
        value = NaN;
    end
end

function value = fieldOrFalse(s, name)
    if isfield(s, name)
        value = logical(s.(name));
    else
        value = false;
    end
end
