#include "system/wifi_debug_server.h"

#include <Arduino.h>
#include <WiFi.h>
#include <cstring>

#include "config/wifi_debug_config.h"
#include "modules/servo/servo_module.h"
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
  char buffer[512];
  snprintf(buffer, sizeof(buffer),
           "{\"enabled\":%s,\"active\":%s,\"fault\":%s,"
           "\"targetPitch\":%.3f,\"pitch\":%.3f,\"pitchRate\":%.3f,"
           "\"wheelVelocity\":%.3f,\"outputVelocity\":%.3f,"
           "\"kp\":%.4f,\"kd\":%.4f,\"kv\":%.4f,\"direction\":%.1f,"
           "\"maxVelocity\":%.3f,\"startAngle\":%.3f,\"maxAngle\":%.3f,\"lastUpdateMs\":%lu}",
           balance.enabled ? "true" : "false",
           balance.active ? "true" : "false",
           balance.fault ? "true" : "false",
           balance.target_pitch_deg,
           balance.pitch_deg,
           balance.pitch_rate_dps,
           balance.wheel_velocity,
           balance.output_velocity,
           balance.kp,
           balance.kd,
           balance.kv,
           balance.output_direction,
           balance.max_velocity,
           balance.start_angle_deg,
           balance.max_angle_deg,
           (unsigned long)balance.last_update_ms);
  return String(buffer);
}

String servoStatusJson() {
  const auto status = servo::status();
  const auto read_result = servo::lastReadResult();
  String json = "{\"initialized\":";
  json += status.initialized ? "true" : "false";
  json += ",\"rxFlag\":";
  json += String(status.rx_flag);
  json += ",\"expectedLen\":";
  json += String(status.expected_len);
  json += ",\"availableBytes\":";
  json += String(status.available_bytes);
  json += ",\"lastCommand\":";
  json += String(status.last_command);
  json += ",\"lastRxLen\":";
  json += String(status.last_rx_len);
  json += ",\"lastParseOk\":";
  json += status.last_parse_ok ? "true" : "false";
  json += ",\"batteryMv\":";
  json += String(status.battery_mv);
  json += ",\"readValid\":";
  json += read_result.valid ? "true" : "false";
  json += ",\"positions\":[";
  for (uint8_t id = 1; id <= 4; ++id) {
    if (id > 1) json += ",";
    json += "{\"id\":";
    json += String(id);
    json += ",\"position\":";
    int position = -1;
    if (read_result.valid) {
      for (uint8_t i = 0; i < read_result.count; ++i) {
        if (read_result.id[i] == id) {
          position = read_result.position[i];
          break;
        }
      }
    }
    json += String(position);
    json += "}";
  }
  json += "]}";
  return json;
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
  server_.on("/api/servo", HTTP_POST, [this]() { handleServoCommand(); });
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
      server_.hasArg("kv") ||
      server_.hasArg("dir") || server_.hasArg("maxv") || server_.hasArg("starta") ||
      server_.hasArg("maxa")) {
    const auto state = runtime_state::snapshot();
    const auto& balance = state.control.balance;
    const bool has_live_config = balance.max_velocity > 0.0f && balance.max_angle_deg > 0.0f;
    command.target_pitch_deg = server_.hasArg("target") ? clampFloat(server_.arg("target").toFloat(), -20.0f, 20.0f) :
                               (has_live_config ? balance.target_pitch_deg : command.target_pitch_deg);
    command.kp = server_.hasArg("kp") ? clampFloat(server_.arg("kp").toFloat(), -20.0f, 20.0f) :
                 (has_live_config ? balance.kp : command.kp);
    command.kd = server_.hasArg("kd") ? clampFloat(server_.arg("kd").toFloat(), -5.0f, 5.0f) :
                 (has_live_config ? balance.kd : command.kd);
    command.kv = server_.hasArg("kv") ? clampFloat(server_.arg("kv").toFloat(), 0.0f, 5.0f) :
                 (has_live_config ? balance.kv : command.kv);
    command.output_direction = server_.hasArg("dir") ?
                               (server_.arg("dir").toFloat() >= 0.0f ? 1.0f : -1.0f) :
                               (has_live_config ? balance.output_direction : command.output_direction);
    command.max_velocity = server_.hasArg("maxv") ? clampFloat(server_.arg("maxv").toFloat(), 0.2f, 20.0f) :
                           (has_live_config ? balance.max_velocity : command.max_velocity);
    command.start_angle_deg = server_.hasArg("starta") ? clampFloat(server_.arg("starta").toFloat(), 1.0f, 30.0f) :
                              (has_live_config ? balance.start_angle_deg : command.start_angle_deg);
    command.max_angle_deg = server_.hasArg("maxa") ? clampFloat(server_.arg("maxa").toFloat(), 5.0f, 60.0f) :
                            (has_live_config ? balance.max_angle_deg : command.max_angle_deg);
    command.has_tuning = true;
  }

  command.updated_ms = millis();
  command.sequence = ++g_balance_command_sequence;
  runtime_state::updateBalanceCommand(command);
  server_.send(200, "application/json; charset=utf-8", "{\"ok\":true}");
}

void WiFiDebugServer::handleServoCommand() {
  const String action = server_.hasArg("action") ? server_.arg("action") : "move";
  const uint16_t time_ms = server_.hasArg("time") ?
                           static_cast<uint16_t>(clampFloat(server_.arg("time").toFloat(), 0.0f, 10000.0f)) :
                           500;

  if (action == "read") {
    uint8_t ids[4] = {1, 2, 3, 4};
    servo::readMulti(ids, 4);
    server_.send(200, "application/json; charset=utf-8", "{\"ok\":true,\"message\":\"read requested\"}");
    return;
  }

  if (action == "battery") {
    servo::readBatteryVoltage();
    server_.send(200, "application/json; charset=utf-8", "{\"ok\":true,\"message\":\"battery requested\"}");
    return;
  }

  if (action == "set_height") {
    const float height = server_.hasArg("h") ? clampFloat(server_.arg("h").toFloat(), 10.0f, 35.0f) : 20.0f;
    auto command = runtime_state::servoCommand();
    command.target_height = height;
    command.time_ms = time_ms;
    command.has_height = true;
    command.updated_ms = millis();
    command.sequence++;
    runtime_state::updateServoCommand(command);
    server_.send(200, "application/json; charset=utf-8", "{\"ok\":true,\"height\":" + String(height) + ",\"time\":" + String(time_ms) + "}");
    return;
  }

  if (action == "leg_mix") {
    const uint16_t right_center = server_.hasArg("right") ?
                                  static_cast<uint16_t>(clampFloat(server_.arg("right").toFloat(), 0.0f, 1000.0f)) :
                                  500;
    const uint16_t left_center = server_.hasArg("left") ?
                                 static_cast<uint16_t>(clampFloat(server_.arg("left").toFloat(), 0.0f, 1000.0f)) :
                                 500;
    const float pitch = server_.hasArg("pitch") ?
                        clampFloat(server_.arg("pitch").toFloat(), -300.0f, 300.0f) :
                        0.0f;

    // 简易腿部混控：ID1=右后，ID2=右前，ID3=左后，ID4=左前。
    // 左右腿前后差值采用镜像关系，先用于验证四个舵机方向与联动关系。
    servo::BusServo_Move_Param params[4] = {};
    params[0] = {1, static_cast<uint16_t>(clampFloat(right_center + pitch, 0.0f, 1000.0f)), time_ms, 0};
    params[1] = {2, static_cast<uint16_t>(clampFloat(right_center - pitch, 0.0f, 1000.0f)), time_ms, 0};
    params[2] = {3, static_cast<uint16_t>(clampFloat(left_center - pitch, 0.0f, 1000.0f)), time_ms, 0};
    params[3] = {4, static_cast<uint16_t>(clampFloat(left_center + pitch, 0.0f, 1000.0f)), time_ms, 0};
    servo::moveMulti(params, 4);
    server_.send(200, "application/json; charset=utf-8", "{\"ok\":true}");
    return;
  }

  if (action == "move_all") {
    servo::BusServo_Move_Param params[4] = {};
    for (uint8_t i = 0; i < 4; ++i) {
      const uint8_t id = i + 1;
      const String arg_name = "p" + String(id);
      params[i].id = id;
      params[i].angle = server_.hasArg(arg_name) ?
                        static_cast<uint16_t>(clampFloat(server_.arg(arg_name).toFloat(), 0.0f, 1000.0f)) :
                        500;
      params[i].time = time_ms;
      params[i].feedback = 0;
    }
    servo::moveMulti(params, 4);
    server_.send(200, "application/json; charset=utf-8", "{\"ok\":true}");
    return;
  }

  const uint8_t id = server_.hasArg("id") ?
                     static_cast<uint8_t>(clampFloat(server_.arg("id").toFloat(), 1.0f, 4.0f)) :
                     1;
  const uint16_t position = server_.hasArg("position") ?
                            static_cast<uint16_t>(clampFloat(server_.arg("position").toFloat(), 0.0f, 1000.0f)) :
                            500;
  servo::moveServo(id, position, time_ms);
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
    .servo-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 10px; margin-top: 12px; }
    .servo-item { background: #f8fafc; border-radius: 8px; padding: 10px; text-align: left; font-size: 13px; color: #475569; }
    .servo-item input { width: 100%; font-size: 16px; padding: 8px; margin: 4px 0 8px; }
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
        <label>Kv<input type="number" id="balanceKv" value="0.0" step="0.02"></label>
        <label>输出方向<input type="number" id="balanceDir" value="1" step="2"></label>
        <label>最大轮速<input type="number" id="balanceMaxV" value="4.0" step="0.5"></label>
        <label>启动角度<input type="number" id="balanceStartA" value="10.0" step="1.0"></label>
        <label>保护角度<input type="number" id="balanceMaxA" value="25.0" step="1.0"></label>
      </div>
      <button onclick="applyBalanceTuning()" style="background:#475569;">写入平衡参数</button>
      <button onclick="setBalance(1)" style="background:#0f766e;">开启平衡</button>
      <button onclick="setBalance(0)" style="background:#f44336;">关闭平衡</button>
      <div class="status" id="balanceStatus">平衡状态: --</div>
      <h3>幻尔总线舵机</h3>
      <div class="input-grp">
        <label>移动时间 (ms): </label>
        <input type="number" id="servoTime" value="500" step="100" min="0" max="10000" style="font-size: 16px; width: 90px; text-align: center;">
      </div>
      <div class="servo-grid">
        <div class="servo-item">ID 1<input type="number" id="servoPos1" value="500" step="10" min="0" max="1000"><button onclick="moveServo(1)" style="background:#2563eb;">移动</button><div id="servoRead1">位置: --</div></div>
        <div class="servo-item">ID 2<input type="number" id="servoPos2" value="500" step="10" min="0" max="1000"><button onclick="moveServo(2)" style="background:#2563eb;">移动</button><div id="servoRead2">位置: --</div></div>
        <div class="servo-item">ID 3<input type="number" id="servoPos3" value="500" step="10" min="0" max="1000"><button onclick="moveServo(3)" style="background:#2563eb;">移动</button><div id="servoRead3">位置: --</div></div>
        <div class="servo-item">ID 4<input type="number" id="servoPos4" value="500" step="10" min="0" max="1000"><button onclick="moveServo(4)" style="background:#2563eb;">移动</button><div id="servoRead4">位置: --</div></div>
      </div>
      <button onclick="moveAllServos()" style="background:#475569;">同步移动 1-4</button>
      <button onclick="readServos()" style="background:#0f766e;">读取位置</button>
      <button onclick="readServoBattery()" style="background:#0369a1;">读取控制板电压</button>
      <h3>高度与简易解算</h3>
      <div class="balance-grid">
        <label>预设高度 (cm)<input type="number" id="legHeight" value="20" step="1" min="10" max="35"></label>
        <label>右腿中心<input type="number" id="legRightCenter" value="500" step="10" min="0" max="1000"></label>
        <label>左腿中心<input type="number" id="legLeftCenter" value="500" step="10" min="0" max="1000"></label>
        <label>前后差值<input type="number" id="legPitchMix" value="0" step="10" min="-300" max="300"></label>
      </div>
      <button onclick="applyHeight()" style="background:#0f766e;">应用预设高度</button>
      <button onclick="applyLegMix()" style="background:#7c3aed;">应用简易解算</button>
      <div class="status" id="servoStatus">舵机状态: --</div>
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
        kv: document.getElementById('balanceKv').value,
        dir: document.getElementById('balanceDir').value,
        maxv: document.getElementById('balanceMaxV').value,
        starta: document.getElementById('balanceStartA').value,
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
    function servoTime() { return document.getElementById('servoTime').value; }
    async function moveServo(id) {
      const pos = document.getElementById('servoPos' + id).value;
      await fetch('/api/servo?action=move&id=' + encodeURIComponent(id) +
        '&position=' + encodeURIComponent(pos) +
        '&time=' + encodeURIComponent(servoTime()), { method: 'POST' });
      updateStatus();
    }
    async function moveAllServos() {
      const params = new URLSearchParams({ action: 'move_all', time: servoTime() });
      for (let id = 1; id <= 4; id++) params.set('p' + id, document.getElementById('servoPos' + id).value);
      await fetch('/api/servo?' + params.toString(), { method: 'POST' });
      updateStatus();
    }
    async function readServos() {
      await fetch('/api/servo?action=read', { method: 'POST' });
      setTimeout(updateStatus, 80);
    }
    async function readServoBattery() {
      await fetch('/api/servo?action=battery', { method: 'POST' });
      setTimeout(updateStatus, 80);
    }
    async function applyLegMix() {
      const params = new URLSearchParams({
        action: 'leg_mix',
        time: servoTime(),
        right: document.getElementById('legRightCenter').value,
        left: document.getElementById('legLeftCenter').value,
        pitch: document.getElementById('legPitchMix').value
      });
      await fetch('/api/servo?' + params.toString(), { method: 'POST' });
      updateStatus();
    }
    async function applyHeight() {
      const h = document.getElementById('legHeight').value;
      const time = servoTime();
      await fetch(`/api/servo?action=set_height&h=${h}&time=${time}`, { method: 'POST' });
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
        '轮速: ' + Number(b.wheelVelocity).toFixed(2) + ' rad/s<br>' +
        'Kp/Kd/Kv: ' + Number(b.kp).toFixed(3) + ' / ' + Number(b.kd).toFixed(3) + ' / ' + Number(b.kv).toFixed(3) +
        ' | Dir: ' + Number(b.direction).toFixed(0) + '<br>' +
        '启动/保护角: ' + Number(b.startAngle).toFixed(1) + '° / ' + Number(b.maxAngle).toFixed(1) + '°' +
        ' | 保护: ' + (b.fault ? '触发' : '正常');
    }
    function updateServoStatus(s) {
      if (!s) {
        document.getElementById('servoStatus').innerText = '舵机状态: --';
        return;
      }
      document.getElementById('servoStatus').innerText =
        '舵机状态: ' + (s.initialized ? '已初始化' : '未初始化') +
        ' | RX: ' + s.rxFlag +
        ' | 期望/缓存: ' + s.expectedLen + '/' + s.availableBytes +
        ' | 命令: 0x' + Number(s.lastCommand).toString(16).padStart(2, '0') +
        ' | 收到: ' + s.lastRxLen +
        ' | 解析: ' + (s.lastParseOk ? '成功' : '失败') +
        ' | 电压: ' + (Number(s.batteryMv) > 0 ? (Number(s.batteryMv) / 1000).toFixed(2) + 'V' : '--') +
        ' | 读回: ' + (s.readValid ? '有效' : '无');
      if (Array.isArray(s.positions)) {
        s.positions.forEach((item) => {
          const el = document.getElementById('servoRead' + item.id);
          if (el) el.innerText = '位置: ' + (Number(item.position) >= 0 ? item.position : '--');
        });
      }
    }
    async function updateStatus() {
      try {
        const res = await fetch('/api/status', { cache: 'no-store' });
        const data = await res.json();
        document.getElementById('uptime').innerText = '通信状态: 正常 | 运行: ' + data.uptime + ' | WiFi: ' + data.wifiMode + ' ' + data.ip;
        document.getElementById('leftMotorStatus').innerHTML = motorText('左轮', data.leftMotor);
        document.getElementById('rightMotorStatus').innerHTML = motorText('右轮', data.rightMotor);
        document.getElementById('balanceStatus').innerHTML = balanceText(data.balance);
        updateServoStatus(data.servo);
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
  json += "\"servo\":" + servoStatusJson() + ",";
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
