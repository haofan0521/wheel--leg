#include "system/wifi_debug_server.h"

#include <Arduino.h>
#include <WiFi.h>
#include <cstring>

#include "config/wifi_debug_config.h"
#include "system/runtime_state.h"

namespace {

bool hasConfiguredStationCredentials() {
  return std::strlen(wifi_debug_config::kStaSsid) > 0 &&
         std::strcmp(wifi_debug_config::kStaSsid, "YOUR_PHONE_WIFI_SSID") != 0;
}

String wifiModeToString(const bool station_connected) {
  return station_connected ? "STA" : "AP";
}

float clampFloat(const float value, const float min_value, const float max_value) {
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

void resetCommandFlags(runtime_state::MotorCommand& command) {
  command.stop = false;
  command.has_enable = false;
  command.has_velocity_target = false;
  command.has_voltage_limit = false;
  command.has_open_loop = false;
  command.has_tuning = false;
}

void applyRequestToCommand(WebServer& server, runtime_state::MotorCommand& command) {
  if (server.hasArg("mode")) {
    const String mode = server.arg("mode");
    command.open_loop = mode == "open" || mode == "open_loop" || mode == "1";
    command.has_open_loop = true;
  }

  if (server.hasArg("enable")) {
    command.enable = server.arg("enable").toInt() != 0;
    command.has_enable = true;
  }
  if (server.hasArg("stop")) {
    command.stop = true;
    command.enable = false;
    command.has_enable = true;
    command.target_velocity = 0.0f;
    command.has_velocity_target = true;
  }
  if (server.hasArg("v")) {
    command.target_velocity = clampFloat(server.arg("v").toFloat(), -20.0f, 20.0f);
    command.has_velocity_target = true;
    command.enable = true;
    command.has_enable = true;
  }
  if (server.hasArg("l")) {
    command.voltage_limit = clampFloat(server.arg("l").toFloat(), 0.5f, 8.0f);
    command.has_voltage_limit = true;
  }

  if (server.hasArg("p") || server.hasArg("i") || server.hasArg("d") || server.hasArg("tf")) {
    command.velocity_p = server.hasArg("p") ? clampFloat(server.arg("p").toFloat(), 0.0f, 2.0f) : command.velocity_p;
    command.velocity_i = server.hasArg("i") ? clampFloat(server.arg("i").toFloat(), 0.0f, 20.0f) : command.velocity_i;
    command.velocity_d = server.hasArg("d") ? clampFloat(server.arg("d").toFloat(), 0.0f, 1.0f) : command.velocity_d;
    command.velocity_lpf_tf = server.hasArg("tf") ? clampFloat(server.arg("tf").toFloat(), 0.001f, 0.5f) : command.velocity_lpf_tf;
    command.has_tuning = true;
  }
}

String motorSnapshotJson(const runtime_state::MotorSnapshot& motor) {
  char buffer[768];
  snprintf(buffer, sizeof(buffer),
           "{\"initialized\":%s,\"focReady\":%s,\"enabled\":%s,\"openLoop\":%s,"
           "\"emergencyStopped\":%s,"
           "\"targetVelocity\":%.3f,\"measuredVelocity\":%.3f,\"angle\":%.3f,\"voltageLimit\":%.3f,"
           "\"velocityP\":%.4f,\"velocityI\":%.4f,\"velocityD\":%.4f,\"velocityTf\":%.4f,"
           "\"voltageQ\":%.4f,\"voltageD\":%.4f,\"commandAgeMs\":%lu}",
           motor.initialized ? "true" : "false",
           motor.foc_ready ? "true" : "false",
           motor.enabled ? "true" : "false",
           motor.open_loop ? "true" : "false",
           motor.emergency_stopped ? "true" : "false",
           motor.target_velocity,
           motor.measured_velocity,
           motor.shaft_angle,
           motor.voltage_limit,
           motor.velocity_p,
           motor.velocity_i,
           motor.velocity_d,
           motor.velocity_lpf_tf,
           motor.voltage_q,
           motor.voltage_d,
           (unsigned long)motor.command_age_ms);
  return String(buffer);
}

String balanceSnapshotJson(const runtime_state::BalanceSnapshot& balance) {
  char buffer[384];
  snprintf(buffer, sizeof(buffer),
           "{\"enabled\":%s,\"active\":%s,\"fault\":%s,"
           "\"targetPitch\":%.3f,\"pitch\":%.3f,\"pitchRate\":%.3f,\"outputVelocity\":%.3f,"
           "\"kp\":%.4f,\"kd\":%.4f,\"maxVelocity\":%.3f,\"maxAngle\":%.3f,\"lastUpdateMs\":%lu}",
           balance.enabled ? "true" : "false",
           balance.active ? "true" : "false",
           balance.fault ? "true" : "false",
           balance.target_pitch_deg,
           balance.pitch_deg,
           balance.pitch_rate_dps,
           balance.output_velocity,
           balance.kp,
           balance.kd,
           balance.max_velocity,
           balance.max_angle_deg,
           (unsigned long)balance.last_update_ms);
  return String(buffer);
}

uint32_t g_motor_command_sequence = 0;
uint32_t g_balance_command_sequence = 0;

}  // namespace

WiFiDebugServer& WiFiDebugServer::instance() {
  static WiFiDebugServer server;
  return server;
}

WiFiDebugServer::WiFiDebugServer() : server_(80), started_(false), stationConnected_(false) {}

void WiFiDebugServer::begin() {
  if (started_) return;
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  configureRoutes();
  stationConnected_ = connectToStation();
  if (!stationConnected_) startFallbackAccessPoint();
  server_.begin();
  started_ = true;
}

void WiFiDebugServer::loop() {
  if (!started_) return;
  server_.handleClient();
}

void WiFiDebugServer::configureRoutes() {
  server_.on("/", HTTP_GET, [this]() { handleRoot(); });
  server_.on("/api/status", HTTP_GET, [this]() { handleStatus(); });
  server_.on("/api/attitude", HTTP_GET, [this]() { handleAttitude(); });
  server_.on("/api/restart", HTTP_POST, [this]() { handleRestart(); });
  server_.on("/api/motor", HTTP_POST, [this]() { handleMotorCommand(); });
  server_.on("/api/balance", HTTP_POST, [this]() { handleBalanceCommand(); });
  server_.onNotFound([this]() { server_.send(404, "text/plain", "Not Found"); });
}

bool WiFiDebugServer::connectToStation() {
  if (!hasConfiguredStationCredentials()) return false;
  WiFi.mode(WIFI_MODE_STA);
  WiFi.setHostname(wifi_debug_config::kHostname);
  WiFi.begin(wifi_debug_config::kStaSsid, wifi_debug_config::kStaPassword);
  const uint32_t start_ms = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start_ms < wifi_debug_config::kConnectTimeoutMs) {
    delay(250);
  }
  return WiFi.status() == WL_CONNECTED;
}

void WiFiDebugServer::startFallbackAccessPoint() {
  WiFi.disconnect(true, true);
  delay(100);
  WiFi.mode(WIFI_MODE_AP);
  WiFi.softAP(wifi_debug_config::kFallbackApSsid, wifi_debug_config::kFallbackApPassword);
}

void WiFiDebugServer::handleRoot() { server_.send(200, "text/html; charset=utf-8", buildDebugPage()); }
void WiFiDebugServer::handleStatus() { server_.send(200, "application/json; charset=utf-8", buildStatusJson()); }
void WiFiDebugServer::handleAttitude() { server_.send(200, "application/json; charset=utf-8", buildAttitudeJson()); }
void WiFiDebugServer::handleRestart() {
  server_.send(200, "application/json; charset=utf-8", "{\"ok\":true,\"message\":\"Device will restart\"}");
  delay(200);
  ESP.restart();
}

void WiFiDebugServer::handleMotorCommand() {
  const String side = server_.hasArg("side") ? server_.arg("side") : "left";
  const bool apply_left = side == "left" || side == "both";
  const bool apply_right = side == "right" || side == "both";

  if (!apply_left && !apply_right) {
    server_.send(400, "application/json; charset=utf-8", "{\"ok\":false,\"error\":\"invalid side\"}");
    return;
  }

  const uint32_t now_ms = millis();
  const uint32_t sequence = ++g_motor_command_sequence;

  if (apply_left) {
    auto command = runtime_state::leftMotorCommand();
    resetCommandFlags(command);
    applyRequestToCommand(server_, command);
    command.updated_ms = now_ms;
    command.sequence = sequence;
    runtime_state::updateLeftMotorCommand(command);
  }

  if (apply_right) {
    auto command = runtime_state::rightMotorCommand();
    resetCommandFlags(command);
    applyRequestToCommand(server_, command);
    command.updated_ms = now_ms;
    command.sequence = sequence;
    runtime_state::updateRightMotorCommand(command);
  }

  server_.send(200, "application/json; charset=utf-8", "{\"ok\":true}");
}

void WiFiDebugServer::handleBalanceCommand() {
  auto command = runtime_state::balanceCommand();
  command.stop = false;
  command.has_enable = false;
  command.has_tuning = false;

  if (server_.hasArg("enable")) {
    command.enable = server_.arg("enable").toInt() != 0;
    command.has_enable = true;
  }
  if (server_.hasArg("stop")) {
    command.stop = true;
    command.enable = false;
    command.has_enable = true;
  }
  if (server_.hasArg("target") || server_.hasArg("kp") || server_.hasArg("kd") ||
      server_.hasArg("maxv") || server_.hasArg("maxa")) {
    const auto state = runtime_state::snapshot();
    const auto& balance = state.control.balance;
    const bool has_live_config = balance.max_velocity > 0.0f && balance.max_angle_deg > 0.0f;
    command.target_pitch_deg = server_.hasArg("target") ? clampFloat(server_.arg("target").toFloat(), -20.0f, 20.0f) :
                               (has_live_config ? balance.target_pitch_deg : command.target_pitch_deg);
    command.kp = server_.hasArg("kp") ? clampFloat(server_.arg("kp").toFloat(), -20.0f, 20.0f) :
                 (has_live_config ? balance.kp : command.kp);
    command.kd = server_.hasArg("kd") ? clampFloat(server_.arg("kd").toFloat(), -5.0f, 5.0f) :
                 (has_live_config ? balance.kd : command.kd);
    command.max_velocity = server_.hasArg("maxv") ? clampFloat(server_.arg("maxv").toFloat(), 0.2f, 20.0f) :
                           (has_live_config ? balance.max_velocity : command.max_velocity);
    command.max_angle_deg = server_.hasArg("maxa") ? clampFloat(server_.arg("maxa").toFloat(), 5.0f, 60.0f) :
                            (has_live_config ? balance.max_angle_deg : command.max_angle_deg);
    command.has_tuning = true;
  }

  command.updated_ms = millis();
  command.sequence = ++g_balance_command_sequence;
  runtime_state::updateBalanceCommand(command);
  server_.send(200, "application/json; charset=utf-8", "{\"ok\":true}");
}

String WiFiDebugServer::buildDebugPage() const {
  String html = R"RAW(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Wheel-Leg Debug</title>
  <style>
    * { box-sizing: border-box; }
    body { font-family: Arial, sans-serif; background: #f0f2f5; color: #1f2933; text-align: center; padding: 20px; margin: 0; }
    .layout { display: flex; flex-wrap: wrap; gap: 20px; justify-content: center; align-items: flex-start; }
    .card { background: white; padding: 24px; border-radius: 12px; box-shadow: 0 4px 12px rgba(0,0,0,0.1); width: min(100%, 480px); }
    button { padding: 12px 20px; font-size: 16px; border: none; border-radius: 8px; margin: 8px; cursor: pointer; color: white; display: inline-block; }
    .btn-fwd { background: #4caf50; width: 38%; }
    .btn-rev { background: #2196f3; width: 38%; }
    .btn-stop { background: #f44336; width: 82%; }
    .input-grp { margin: 16px 0; }
    .status { margin-top: 14px; font-size: 14px; color: #666; text-align: left; line-height: 1.7; }
    .motor-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 10px; text-align: left; }
    .motor-card { background: #f8fafc; border-radius: 8px; padding: 12px; font-size: 14px; line-height: 1.7; }
    .balance-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 10px; margin-top: 12px; }
    .balance-grid label { display: block; text-align: left; font-size: 13px; color: #475569; }
    .balance-grid input { width: 100%; font-size: 16px; padding: 8px; margin-top: 4px; }
    .scene { width: 220px; height: 220px; margin: 30px auto; perspective: 700px; display: flex; align-items: center; justify-content: center; }
    .cube { position: relative; width: 120px; height: 120px; transform-style: preserve-3d; transform: rotateX(0deg) rotateY(0deg) rotateZ(0deg); transition: transform 80ms linear; }
    .face { position: absolute; width: 120px; height: 120px; border: 2px solid rgba(255,255,255,0.8); display: flex; align-items: center; justify-content: center; font-weight: bold; color: white; text-shadow: 0 1px 2px rgba(0,0,0,0.35); opacity: 0.92; }
    .front { background: #2563eb; transform: translateZ(60px); }
    .back { background: #1d4ed8; transform: rotateY(180deg) translateZ(60px); }
    .right { background: #16a34a; transform: rotateY(90deg) translateZ(60px); }
    .left { background: #15803d; transform: rotateY(-90deg) translateZ(60px); }
    .top { background: #f97316; transform: rotateX(90deg) translateZ(60px); }
    .bottom { background: #dc2626; transform: rotateX(-90deg) translateZ(60px); }
    .attitude-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; margin-top: 16px; }
    .attitude-item { background: #f8fafc; border-radius: 8px; padding: 10px 6px; }
    .attitude-label { color: #64748b; font-size: 12px; }
    .attitude-value { color: #0f172a; font-size: 18px; font-weight: bold; margin-top: 4px; }
    .fresh { color: #16a34a; }
    .stale { color: #dc2626; }
  </style>
</head>
<body>
  <div class="layout">
    <div class="card">
      <h2>左右轮控制调试</h2>
      <div class="input-grp">
        <label>控制对象: </label>
        <select id="side" style="font-size: 16px; padding: 6px;">
          <option value="left">左轮</option>
          <option value="right">右轮</option>
          <option value="both">左右同时</option>
        </select>
      </div>
      <div class="input-grp">
        <button onclick="setMode('open')" style="background:#8b5cf6;">开环模式</button>
        <button onclick="setMode('closed')" style="background:#0f766e;">闭环 FOC</button>
      </div>
      <div class="input-grp">
        <label>目标速度 (rad/s): </label>
        <input type="number" id="speed" value="5.0" step="1.0" style="font-size: 18px; width: 80px; text-align: center;">
      </div>
      <button class="btn-fwd" onclick="setSpeed(document.getElementById('speed').value)">正转</button>
      <button class="btn-rev" onclick="setSpeed(-document.getElementById('speed').value)">反转</button>
      <button class="btn-stop" onclick="emergencyStop()">急停</button>
      <div class="status" id="uptime">连接中...</div>
      <div class="motor-grid">
        <div class="motor-card" id="leftMotorStatus">左轮状态: --</div>
        <div class="motor-card" id="rightMotorStatus">右轮状态: --</div>
      </div>
      <h3>IMU 平衡</h3>
      <div class="balance-grid">
        <label>目标 Pitch<input type="number" id="balanceTarget" value="0" step="0.5"></label>
        <label>Kp<input type="number" id="balanceKp" value="0.6" step="0.1"></label>
        <label>Kd<input type="number" id="balanceKd" value="0.03" step="0.01"></label>
        <label>最大轮速<input type="number" id="balanceMaxV" value="4.0" step="0.5"></label>
        <label>保护角度<input type="number" id="balanceMaxA" value="25.0" step="1.0"></label>
      </div>
      <button onclick="applyBalanceTuning()" style="background:#475569;">写入平衡参数</button>
      <button onclick="setBalance(1)" style="background:#0f766e;">开启平衡</button>
      <button onclick="setBalance(0)" style="background:#f44336;">关闭平衡</button>
      <div class="status" id="balanceStatus">平衡状态: --</div>
    </div>

    <div class="card">
      <h2>IMU 姿态显示</h2>
      <div class="scene"><div class="cube" id="cube">
        <div class="face front">FRONT</div><div class="face back">BACK</div><div class="face right">RIGHT</div>
        <div class="face left">LEFT</div><div class="face top">TOP</div><div class="face bottom">BOTTOM</div>
      </div></div>
      <div class="attitude-grid">
        <div class="attitude-item"><div class="attitude-label">Pitch</div><div class="attitude-value" id="pitch">--</div></div>
        <div class="attitude-item"><div class="attitude-label">Roll</div><div class="attitude-value" id="roll">--</div></div>
        <div class="attitude-item"><div class="attitude-label">Yaw</div><div class="attitude-value" id="yaw">--</div></div>
      </div>
      <div class="status" id="attitudeStatus">等待 IMU...</div>
    </div>
  </div>

  <script>
    const pitchSign = -1;
    const rollSign = 1;
    const yawSign = 1;
    let attitudeInFlight = false;
    function selectedSide() { return document.getElementById('side').value; }
    async function setMode(mode) { await fetch('/api/motor?side=' + selectedSide() + '&mode=' + encodeURIComponent(mode), { method: 'POST' }); updateStatus(); }
    async function setSpeed(v) { await fetch('/api/motor?side=' + selectedSide() + '&enable=1&v=' + encodeURIComponent(v), { method: 'POST' }); updateStatus(); }
    async function emergencyStop() { await fetch('/api/motor?side=' + selectedSide() + '&stop=1', { method: 'POST' }); updateStatus(); }
    async function applyBalanceTuning() {
      const params = new URLSearchParams({
        target: document.getElementById('balanceTarget').value,
        kp: document.getElementById('balanceKp').value,
        kd: document.getElementById('balanceKd').value,
        maxv: document.getElementById('balanceMaxV').value,
        maxa: document.getElementById('balanceMaxA').value
      });
      await fetch('/api/balance?' + params.toString(), { method: 'POST' });
      updateStatus();
    }
    async function setBalance(enable) {
      await applyBalanceTuning();
      await fetch('/api/balance?enable=' + encodeURIComponent(enable), { method: 'POST' });
      updateStatus();
    }
    function motorText(name, m) {
      if (!m) return name + ': --';
      return name + '<br>' +
        '模式: ' + (m.openLoop ? '开环' : '闭环') +
        ' | FOC: ' + (m.focReady ? '就绪' : '未就绪') + '<br>' +
        '使能: ' + (m.enabled ? '是' : '否') + '<br>' +
        '目标速度: ' + Number(m.targetVelocity).toFixed(2) + ' rad/s<br>' +
        '实际速度: ' + Number(m.measuredVelocity).toFixed(2) + ' rad/s';
    }
    function balanceText(b) {
      if (!b) return '平衡状态: --';
      return '平衡状态: ' + (b.enabled ? '已开启' : '关闭') +
        ' | 输出: ' + Number(b.outputVelocity).toFixed(2) + ' rad/s<br>' +
        'Pitch: ' + Number(b.pitch).toFixed(2) + '°' +
        ' | Rate: ' + Number(b.pitchRate).toFixed(2) + ' °/s<br>' +
        'Kp/Kd: ' + Number(b.kp).toFixed(3) + ' / ' + Number(b.kd).toFixed(3) +
        ' | 保护: ' + (b.fault ? '触发' : '正常');
    }
    async function updateStatus() {
      try {
        const res = await fetch('/api/status', { cache: 'no-store' });
        const data = await res.json();
        document.getElementById('uptime').innerText = '通信状态: 正常 | 运行: ' + data.uptime + ' | WiFi: ' + data.wifiMode + ' ' + data.ip;
        document.getElementById('leftMotorStatus').innerHTML = motorText('左轮', data.leftMotor);
        document.getElementById('rightMotorStatus').innerHTML = motorText('右轮', data.rightMotor);
        document.getElementById('balanceStatus').innerHTML = balanceText(data.balance);
      } catch (e) {
        document.getElementById('uptime').innerText = '通信状态: 断开';
      }
    }
    async function updateAttitude() {
      if (attitudeInFlight) return;
      attitudeInFlight = true;
      try {
        const res = await fetch('/api/attitude', { cache: 'no-store' });
        const data = await res.json();
        const status = document.getElementById('attitudeStatus');
        if (!data.valid) { status.className = 'status stale'; status.innerText = '等待 IMU 数据...'; return; }
        const pitch = Number(data.pitch), roll = Number(data.roll), yaw = Number(data.yaw);
        document.getElementById('cube').style.transform = `rotateX(${pitchSign * pitch}deg) rotateY(${rollSign * roll}deg) rotateZ(${yawSign * yaw}deg)`;
        document.getElementById('pitch').innerText = pitch.toFixed(2) + '°';
        document.getElementById('roll').innerText = roll.toFixed(2) + '°';
        document.getElementById('yaw').innerText = yaw.toFixed(2) + '°';
        const isFresh = data.ageMs <= 500;
        status.className = isFresh ? 'status fresh' : 'status stale';
        status.innerText = (isFresh ? '数据正常' : '数据过期') + ' | 延迟: ' + data.ageMs + ' ms | 序号: ' + data.sequence;
      } catch (e) {
        const status = document.getElementById('attitudeStatus');
        status.className = 'status stale';
        status.innerText = '姿态通信断开';
      } finally { attitudeInFlight = false; }
    }
    updateStatus(); updateAttitude(); setInterval(updateStatus, 500); setInterval(updateAttitude, 100);
  </script>
</body>
</html>
)RAW";
  return html;
}

String WiFiDebugServer::buildStatusJson() const {
  const auto state = runtime_state::snapshot();
  String json = "{\"uptime\":\"" + String(millis() / 1000) + " s\",";
  json += "\"wifiMode\":\"" + wifiModeToString(stationConnected_) + "\",";
  json += "\"ip\":\"" + currentIpString() + "\",";
  json += "\"driveEnabled\":" + String(state.control.drive_enabled ? "true" : "false") + ",";
  json += "\"driveFault\":" + String(state.control.drive_fault_level) + ",";
  json += "\"balance\":" + balanceSnapshotJson(state.control.balance) + ",";
  json += "\"leftMotor\":" + motorSnapshotJson(state.control.left_motor) + ",";
  json += "\"rightMotor\":" + motorSnapshotJson(state.control.right_motor) + "}";
  return json;
}

String WiFiDebugServer::buildAttitudeJson() const {
  const auto state = runtime_state::snapshot();
  const auto& imu = state.imu;
  const uint32_t now_ms = millis();
  const uint32_t age_ms = imu.valid ? now_ms - imu.last_update_ms : 0;

  char buffer[192];
  snprintf(buffer, sizeof(buffer),
           "{\"valid\":%s,\"pitch\":%.2f,\"roll\":%.2f,\"yaw\":%.2f,\"accZ\":%.2f,\"ageMs\":%lu,\"sequence\":%lu}",
           imu.valid ? "true" : "false",
           imu.pitch_deg,
           imu.roll_deg,
           imu.yaw_deg,
           imu.acc_z,
           (unsigned long)age_ms,
           (unsigned long)imu.sequence);
  return String(buffer);
}

String WiFiDebugServer::currentModeLabel() const {
  return wifiModeToString(stationConnected_);
}

String WiFiDebugServer::currentIpString() const {
  return stationConnected_ ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
}
