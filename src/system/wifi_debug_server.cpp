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
uint32_t g_motor_command_sequence = 0;
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
  delay(200); ESP.restart();
}

void WiFiDebugServer::handleMotorCommand() {
  auto command = runtime_state::leftMotorCommand();
  command.stop = false;
  command.has_enable = false;
  command.has_velocity_target = false;
  command.has_voltage_limit = false;
  command.has_open_loop = false;

  if (server_.hasArg("mode")) {
    const String mode = server_.arg("mode");
    command.open_loop = mode == "open" || mode == "open_loop" || mode == "1";
    command.has_open_loop = true;
  }

  if (server_.hasArg("enable")) {
    command.enable = server_.arg("enable").toInt() != 0;
    command.has_enable = true;
  }
  if (server_.hasArg("stop")) {
    command.stop = true;
    command.enable = false;
    command.has_enable = true;
    command.target_velocity = 0.0f;
    command.has_velocity_target = true;
  }
  if (server_.hasArg("v")) {
    command.target_velocity = clampFloat(server_.arg("v").toFloat(), -20.0f, 20.0f);
    command.has_velocity_target = true;
    command.enable = true;
    command.has_enable = true;
  }
  if (server_.hasArg("l")) {
    command.voltage_limit = clampFloat(server_.arg("l").toFloat(), 0.5f, 8.0f);
    command.has_voltage_limit = true;
  }

  command.updated_ms = millis();
  command.sequence = ++g_motor_command_sequence;
  runtime_state::updateLeftMotorCommand(command);
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
    .card { background: white; padding: 24px; border-radius: 12px; box-shadow: 0 4px 12px rgba(0,0,0,0.1); width: min(100%, 430px); }
    button { padding: 15px 30px; font-size: 18px; border: none; border-radius: 8px; margin: 10px; cursor: pointer; color: white; display: inline-block; width: 40%; }
    .btn-fwd { background: #4caf50; }
    .btn-rev { background: #2196f3; }
    .btn-stop { background: #f44336; width: 85%; }
    .input-grp { margin: 20px 0; }
    .status { margin-top: 20px; font-size: 14px; color: #666; }
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
      <h2>左轮控制调试</h2>

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

      <hr style="margin: 20px 0; border: 0; border-top: 1px solid #ddd;">

      <div class="input-grp">
        <label>电压限制 (V): </label>
        <input type="number" id="voltage" value="6.0" step="0.5" style="font-size: 14px; width: 60px; text-align: center;">
        <button onclick="setVoltage()" style="width: auto; padding: 5px 10px; font-size: 14px; background: #607d8b;">应用</button>
      </div>

      <div class="status" id="uptime">连接中...</div>
      <div class="status" id="motorStatus">左轮状态: --</div>
    </div>

    <div class="card">
      <h2>IMU 姿态显示</h2>
      <div class="scene">
        <div class="cube" id="cube">
          <div class="face front">FRONT</div>
          <div class="face back">BACK</div>
          <div class="face right">RIGHT</div>
          <div class="face left">LEFT</div>
          <div class="face top">TOP</div>
          <div class="face bottom">BOTTOM</div>
        </div>
      </div>
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

    async function setMode(mode) {
      await fetch('/api/motor?mode=' + encodeURIComponent(mode) + '&stop=1', { method: 'POST' });
    }

    async function setSpeed(v) {
      await fetch('/api/motor?enable=1&v=' + encodeURIComponent(v), { method: 'POST' });
    }

    async function emergencyStop() {
      await fetch('/api/motor?stop=1', { method: 'POST' });
    }

    async function setVoltage() {
      const l = document.getElementById('voltage').value;
      await fetch('/api/motor?l=' + encodeURIComponent(l), { method: 'POST' });
    }

    async function updateStatus() {
      try {
        const res = await fetch('/api/status', { cache: 'no-store' });
        const data = await res.json();
        document.getElementById('uptime').innerText = '通信状态: 正常 | 运行: ' + data.uptime;
        if (data.leftMotor) {
          const m = data.leftMotor;
          document.getElementById('motorStatus').innerText =
            '模式: ' + (m.openLoop ? '开环' : '闭环') +
            ' | FOC: ' + (m.focReady ? '就绪' : '未就绪') +
            ' | 使能: ' + (m.enabled ? '是' : '否') +
            ' | 目标: ' + Number(m.targetVelocity).toFixed(2) + ' rad/s' +
            ' | 实际: ' + Number(m.measuredVelocity).toFixed(2) + ' rad/s' +
            ' | 角度: ' + Number(m.angle).toFixed(2) + ' rad' +
            ' | 电压限制: ' + Number(m.voltageLimit).toFixed(2) + ' V' +
            ' | 命令延迟: ' + m.commandAgeMs + ' ms';
        }
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

        if (!data.valid) {
          status.className = 'status stale';
          status.innerText = '等待 IMU 数据...';
          return;
        }

        const pitch = Number(data.pitch);
        const roll = Number(data.roll);
        const yaw = Number(data.yaw);
        const cube = document.getElementById('cube');
        cube.style.transform = `rotateX(${pitchSign * pitch}deg) rotateY(${rollSign * roll}deg) rotateZ(${yawSign * yaw}deg)`;

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
      } finally {
        attitudeInFlight = false;
      }
    }

    updateStatus();
    updateAttitude();
    setInterval(updateStatus, 1000);
    setInterval(updateAttitude, 100);
  </script>
</body>
</html>
)RAW";
  return html;
}

String WiFiDebugServer::buildStatusJson() const {
  const auto state = runtime_state::snapshot();
  const auto& left_motor = state.control.left_motor;

  char buffer[512];
  snprintf(buffer, sizeof(buffer),
           "{\"uptime\":\"%lu s\",\"driveEnabled\":%s,\"driveFault\":%d,"
           "\"leftMotor\":{\"initialized\":%s,\"focReady\":%s,\"enabled\":%s,\"openLoop\":%s,"
           "\"emergencyStopped\":%s,\"targetVelocity\":%.3f,\"measuredVelocity\":%.3f,"
           "\"angle\":%.3f,\"voltageLimit\":%.3f,\"commandAgeMs\":%lu}}",
           (unsigned long)(millis() / 1000),
           state.control.drive_enabled ? "true" : "false",
           state.control.drive_fault_level,
           left_motor.initialized ? "true" : "false",
           left_motor.foc_ready ? "true" : "false",
           left_motor.enabled ? "true" : "false",
           left_motor.open_loop ? "true" : "false",
           left_motor.emergency_stopped ? "true" : "false",
           left_motor.target_velocity,
           left_motor.measured_velocity,
           left_motor.shaft_angle,
           left_motor.voltage_limit,
           (unsigned long)left_motor.command_age_ms);
  return String(buffer);
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

String WiFiDebugServer::currentIpString() const {
  return stationConnected_ ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
}
