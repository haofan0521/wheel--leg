#include "system/wifi_debug_server.h"

#include <Arduino.h>
#include <Preferences.h>
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
  char buffer[896];
  snprintf(buffer, sizeof(buffer),
           "{\"initialized\":%s,\"focReady\":%s,\"enabled\":%s,\"openLoop\":%s,"
           "\"emergencyStopped\":%s,"
           "\"targetVelocity\":%.3f,\"measuredVelocity\":%.3f,\"angle\":%.3f,\"voltageLimit\":%.3f,"
           "\"velocityP\":%.4f,\"velocityI\":%.4f,\"velocityD\":%.4f,\"velocityTf\":%.4f,"
           "\"velocityError\":%.4f,\"velocityPOutput\":%.4f,\"velocityIOutput\":%.4f,\"velocityPidOutput\":%.4f,"
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
           motor.velocity_error,
           motor.velocity_p_output,
           motor.velocity_i_output,
           motor.velocity_pid_output,
           motor.voltage_q,
           motor.voltage_d,
           (unsigned long)motor.command_age_ms);
  return String(buffer);
}

String balanceSnapshotJson(const runtime_state::BalanceSnapshot& balance) {
  char buffer[640];
  snprintf(buffer, sizeof(buffer),
           "{\"enabled\":%s,\"active\":%s,\"fault\":%s,"
           "\"targetPitch\":%.3f,\"pitch\":%.3f,\"pitchRate\":%.3f,"
           "\"wheelVelocity\":%.3f,\"outputVelocity\":%.3f,"
           "\"kp\":%.4f,\"kd\":%.4f,\"kv\":%.4f,\"direction\":%.1f,"
           "\"maxVelocity\":%.3f,\"startAngle\":%.3f,\"maxAngle\":%.3f,"
           "\"remoteVelocity\":%.3f,\"remoteTurnVelocity\":%.3f,\"lastUpdateMs\":%lu}",
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
           balance.remote_velocity,
           balance.remote_turn_velocity,
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
constexpr float kDefaultLegTargetX = 2.0f;
constexpr float kDefaultLegHeightCm = 20.0f;
constexpr char kBalancePrefsNamespace[] = "balance";
constexpr char kBalancePrefsMagicKey[] = "magic";
constexpr uint32_t kBalancePrefsMagic = 0xB14AACE1;

bool loadSavedBalanceCommand(runtime_state::BalanceCommand& command) {
  Preferences prefs;
  if (!prefs.begin(kBalancePrefsNamespace, true)) return false;

  const bool valid = prefs.getUInt(kBalancePrefsMagicKey, 0) == kBalancePrefsMagic;
  if (valid) {
    command.target_pitch_deg = clampFloat(prefs.getFloat("target", command.target_pitch_deg), -20.0f, 20.0f);
    command.kp = clampFloat(prefs.getFloat("kp", command.kp), -20.0f, 20.0f);
    command.kd = clampFloat(prefs.getFloat("kd", command.kd), -5.0f, 5.0f);
    command.kv = clampFloat(prefs.getFloat("kv", command.kv), 0.0f, 5.0f);
    command.output_direction = prefs.getFloat("dir", command.output_direction) >= 0.0f ? 1.0f : -1.0f;
    command.max_velocity = clampFloat(prefs.getFloat("maxv", command.max_velocity), 0.2f, 20.0f);
    command.start_angle_deg = clampFloat(prefs.getFloat("starta", command.start_angle_deg), 1.0f, 30.0f);
    command.max_angle_deg = clampFloat(prefs.getFloat("maxa", command.max_angle_deg), 5.0f, 60.0f);
    if (command.start_angle_deg > command.max_angle_deg) command.start_angle_deg = command.max_angle_deg;
  }

  prefs.end();
  return valid;
}

bool saveBalanceCommand(const runtime_state::BalanceCommand& command) {
  Preferences prefs;
  if (!prefs.begin(kBalancePrefsNamespace, false)) return false;

  bool ok = true;
  ok = prefs.putFloat("target", command.target_pitch_deg) == sizeof(float) && ok;
  ok = prefs.putFloat("kp", command.kp) == sizeof(float) && ok;
  ok = prefs.putFloat("kd", command.kd) == sizeof(float) && ok;
  ok = prefs.putFloat("kv", command.kv) == sizeof(float) && ok;
  ok = prefs.putFloat("dir", command.output_direction) == sizeof(float) && ok;
  ok = prefs.putFloat("maxv", command.max_velocity) == sizeof(float) && ok;
  ok = prefs.putFloat("starta", command.start_angle_deg) == sizeof(float) && ok;
  ok = prefs.putFloat("maxa", command.max_angle_deg) == sizeof(float) && ok;
  ok = prefs.putUInt(kBalancePrefsMagicKey, kBalancePrefsMagic) == sizeof(uint32_t) && ok;

  prefs.end();
  return ok;
}

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
  loadSavedBalanceConfig();
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
  command.has_remote_velocity = false;
  command.has_remote_turn_velocity = false;

  if (server_.hasArg("enable")) {
    command.enable = server_.arg("enable").toInt() != 0;
    command.has_enable = true;
  }
  if (server_.hasArg("stop")) {
    command.stop = true;
    command.enable = false;
    command.has_enable = true;
    command.remote_velocity = 0.0f;
    command.has_remote_velocity = true;
    command.remote_turn_velocity = 0.0f;
    command.has_remote_turn_velocity = true;
  }
  if (server_.hasArg("remotev")) {
    command.remote_velocity = clampFloat(server_.arg("remotev").toFloat(), -20.0f, 20.0f);
    command.has_remote_velocity = true;
  }
  if (server_.hasArg("turnv")) {
    command.remote_turn_velocity = clampFloat(server_.arg("turnv").toFloat(), -20.0f, 20.0f);
    command.has_remote_turn_velocity = true;
  }
  if (server_.hasArg("rseq")) {
    command.remote_sequence = static_cast<uint32_t>(server_.arg("rseq").toInt());
  }
  if (server_.hasArg("target") || server_.hasArg("kp") || server_.hasArg("kd") ||
      server_.hasArg("kv") ||
      server_.hasArg("dir") || server_.hasArg("maxv") || server_.hasArg("starta") ||
      server_.hasArg("maxa") || server_.hasArg("save")) {
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

  const bool save_requested = server_.hasArg("save") && server_.arg("save").toInt() != 0;
  const bool saved = save_requested && saveBalanceCommand(command);

  command.updated_ms = millis();
  command.sequence = ++g_balance_command_sequence;
  runtime_state::updateBalanceCommand(command);
  if (save_requested && !saved) {
    server_.send(500, "application/json; charset=utf-8", "{\"ok\":false,\"error\":\"save failed\"}");
    return;
  }
  server_.send(200, "application/json; charset=utf-8",
               String("{\"ok\":true,\"saved\":") + (saved ? "true" : "false") + "}");
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
    const float height = server_.hasArg("h") ? clampFloat(server_.arg("h").toFloat(), 10.0f, 35.0f) : kDefaultLegHeightCm;
    const float target_x = server_.hasArg("x") ? clampFloat(server_.arg("x").toFloat(), -5.0f, 10.0f) : kDefaultLegTargetX;
    auto command = runtime_state::servoCommand();
    command.target_x = target_x;
    command.target_height = height;
    command.time_ms = time_ms;
    command.has_height = true;
    command.updated_ms = millis();
    command.sequence++;
    runtime_state::updateServoCommand(command);
    server_.send(200, "application/json; charset=utf-8",
                 "{\"ok\":true,\"x\":" + String(target_x) +
                 ",\"height\":" + String(height) +
                 ",\"time\":" + String(time_ms) + "}");
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
    .tuning-notes { margin-top: 14px; padding: 12px 14px; border-left: 4px solid #0369a1; background: #f8fafc; text-align: left; font-size: 13px; line-height: 1.65; color: #334155; }
    .tuning-notes h4 { margin: 0 0 6px; font-size: 15px; color: #0f172a; }
    .tuning-notes ol { margin: 0; padding-left: 18px; }
    .tuning-notes li { margin: 3px 0; }
    .tuning-notes strong { color: #b91c1c; }
    .balance-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 10px; margin-top: 12px; }
    .balance-grid label { display: block; text-align: left; font-size: 13px; color: #475569; }
    .balance-grid input { width: 100%; font-size: 16px; padding: 8px; margin-top: 4px; }
    .fresh { color: #16a34a; }
    .stale { color: #dc2626; }
    .joystick-wrap { display: flex; flex-direction: column; align-items: center; gap: 10px; margin: 14px 0 18px; }
    .joystick { position: relative; width: 170px; height: 170px; border-radius: 50%; background: #eef2f7; border: 2px solid #cbd5e1; box-shadow: inset 0 2px 8px rgba(15,23,42,0.12); touch-action: none; user-select: none; }
    .joystick::before, .joystick::after { content: ""; position: absolute; background: #cbd5e1; left: 50%; top: 50%; transform: translate(-50%, -50%); }
    .joystick::before { width: 2px; height: 128px; }
    .joystick::after { width: 128px; height: 2px; }
    .joystick-knob { position: absolute; left: 50%; top: 50%; width: 64px; height: 64px; border-radius: 50%; background: #2563eb; transform: translate(-50%, -50%); box-shadow: 0 8px 18px rgba(37,99,235,0.35); touch-action: none; }
    .joystick-value { min-height: 22px; font-size: 14px; color: #475569; }
  </style>
</head>
<body>
  <div class="layout">
    <div class="card">
      <h2>电机正反转测试</h2>
      <div class="input-grp">
        <label>目标速度 (rad/s): </label>
        <input type="number" id="speed" value="5.0" step="1.0" style="font-size: 18px; width: 80px; text-align: center;">
      </div>
      <button class="btn-fwd" onclick="setSpeed(document.getElementById('speed').value)">正转</button>
      <button class="btn-rev" onclick="setSpeed(-document.getElementById('speed').value)">反转</button>
      <button class="btn-stop" onclick="stopMotors()">停止</button>
      <div class="status" id="uptime">连接中...</div>
      <h3>IMU 平衡</h3>
      <div class="input-grp">
        <label>前后速度 (rad/s): </label>
        <input type="number" id="remoteSpeed" value="0.8" step="0.1" min="0" max="5" style="font-size: 18px; width: 90px; text-align: center;">
      </div>
      <div class="input-grp">
        <label>转向速度 (rad/s): </label>
        <input type="number" id="remoteTurnSpeed" value="0.5" step="0.1" min="0" max="5" style="font-size: 18px; width: 90px; text-align: center;">
      </div>
      <div class="joystick-wrap">
        <div class="joystick" id="remoteJoystick">
          <div class="joystick-knob" id="remoteJoystickKnob"></div>
        </div>
        <div class="joystick-value" id="remoteJoystickValue">前后: 0.00 | 转向: 0.00 rad/s</div>
      </div>
      <div class="balance-grid">
        <label>目标 Pitch<input type="number" id="balanceTarget" value="0.795" step="0.1"></label>
        <label>Kp<input type="number" id="balanceKp" value="1.69" step="0.05"></label>
        <label>Kd<input type="number" id="balanceKd" value="0.028" step="0.005"></label>
        <label>Kv<input type="number" id="balanceKv" value="0.5" step="0.02"></label>
        <label>输出方向<input type="number" id="balanceDir" value="-1" step="2"></label>
        <label>最大轮速<input type="number" id="balanceMaxV" value="10.0" step="0.5"></label>
        <label>启动角度<input type="number" id="balanceStartA" value="10.0" step="1.0"></label>
        <label>保护角度<input type="number" id="balanceMaxA" value="35.0" step="1.0"></label>
      </div>
      <button onclick="applyBalanceTuning()" style="background:#475569;">写入平衡参数</button>
      <button onclick="saveBalanceTuning()" style="background:#0369a1;">保存平衡参数</button>
      <button onclick="setBalance(1)" style="background:#0f766e;">开启平衡</button>
      <button onclick="setBalance(0)" style="background:#f44336;">关闭平衡</button>
      <div class="status" id="balanceSaveStatus">参数保存: --</div>
      <div class="tuning-notes">
        <h4>调车注意事项</h4>
        <ol>
          <li>先把最大轮速限制在 5~6 rad/s，车轮离地确认输出方向；方向错时不要继续加 Kp/Kd。</li>
          <li>一次只改一个参数：先调 Kp，再加少量 Kd 抑制摆动，最后用 Kv 抑制轮速越跑越大。</li>
          <li>如果刚能站住但随后来回晃动并冲出去，先降 Kp 或 Kd，把 Kv 从 0.03~0.05 小步加起。</li>
          <li>如果车一直往同一方向慢跑，优先每次 0.3~0.5 度微调目标 Pitch，再考虑加 Kv。</li>
          <li>状态里的轮速必须与车体前后移动方向一致；轮速方向不对时，Kv 可能变成反阻尼。</li>
          <li><strong>确认一组参数稳定后再保存</strong>，保存后掉电重启会自动恢复调参值，但不会自动开启平衡。</li>
        </ol>
      </div>
      <div class="status" id="balanceStatus">平衡状态: --</div>
    </div>

  </div>

  <script>
    let balanceInputsSynced = false;
    let joystickActive = false;
    let joystickVelocity = 0;
    let joystickTurnVelocity = 0;
    let remoteTimer = null;
    let remoteInFlight = false;
    let pendingRemote = null;
    let remoteSequence = 0;
    async function setSpeed(v) { await fetch('/api/motor?side=both&enable=1&v=' + encodeURIComponent(v), { method: 'POST' }); updateStatus(); }
    async function stopMotors() { await fetch('/api/motor?side=both&stop=1', { method: 'POST' }); updateStatus(); }
    async function flushRemoteVelocity() {
      if (remoteInFlight || !pendingRemote) return;
      const current = pendingRemote;
      pendingRemote = null;
      remoteInFlight = true;
      const params = new URLSearchParams({
        remotev: String(current.v),
        turnv: String(current.turn),
        rseq: String(current.sequence)
      });
      try {
        await fetch('/api/balance?' + params.toString(), { method: 'POST' });
      } finally {
        remoteInFlight = false;
        if (current.refresh) updateStatus();
        if (pendingRemote) flushRemoteVelocity();
      }
    }
    function setRemoteVelocity(v, turn, refresh = false) {
      pendingRemote = { v, turn, refresh, sequence: ++remoteSequence };
      flushRemoteVelocity();
    }
    async function sendRemoteStopNow(refresh = false) {
      const sequence = ++remoteSequence;
      const params = new URLSearchParams({
        remotev: '0',
        turnv: '0',
        rseq: String(sequence)
      });
      try {
        await fetch('/api/balance?' + params.toString(), { method: 'POST' });
      } finally {
        if (refresh) updateStatus();
      }
    }
    function updateJoystickUi(x, y) {
      const knob = document.getElementById('remoteJoystickKnob');
      const label = document.getElementById('remoteJoystickValue');
      const maxOffset = 53;
      knob.style.transform = `translate(calc(-50% + ${x * maxOffset}px), calc(-50% + ${-y * maxOffset}px))`;
      label.innerText = '前后: ' + joystickVelocity.toFixed(2) +
        ' | 转向: ' + joystickTurnVelocity.toFixed(2) + ' rad/s';
    }
    function setJoystickVelocity(x, y) {
      const maxSpeed = Math.abs(Number(document.getElementById('remoteSpeed').value || 0));
      const maxTurnSpeed = Math.abs(Number(document.getElementById('remoteTurnSpeed').value || 0));
      const deadband = 0.08;
      const normalizedX = Math.abs(x) < deadband ? 0 : x;
      const normalizedY = Math.abs(y) < deadband ? 0 : y;
      joystickVelocity = normalizedY * maxSpeed;
      joystickTurnVelocity = normalizedX * maxTurnSpeed;
      updateJoystickUi(normalizedX, normalizedY);
    }
    function updateJoystickFromPointer(event) {
      const joystick = document.getElementById('remoteJoystick');
      const rect = joystick.getBoundingClientRect();
      const center_x = rect.left + rect.width * 0.5;
      const center_y = rect.top + rect.height * 0.5;
      const radius = rect.width * 0.5;
      let x = (event.clientX - center_x) / radius;
      let y = (center_y - event.clientY) / radius;
      const length = Math.hypot(x, y);
      if (length > 1.0) {
        x /= length;
        y /= length;
      }
      setJoystickVelocity(x, y);
    }
    function stopRemoteVelocity(shouldUpdate = true) {
      if (remoteTimer) clearInterval(remoteTimer);
      remoteTimer = null;
      joystickActive = false;
      joystickVelocity = 0;
      joystickTurnVelocity = 0;
      updateJoystickUi(0, 0);
      pendingRemote = null;
      if (shouldUpdate) sendRemoteStopNow(true);
    }
    function bindRemoteJoystick() {
      const joystick = document.getElementById('remoteJoystick');
      joystick.addEventListener('pointerdown', (event) => {
        event.preventDefault();
        joystickActive = true;
        joystick.setPointerCapture(event.pointerId);
        updateJoystickFromPointer(event);
        setRemoteVelocity(joystickVelocity, joystickTurnVelocity);
        remoteTimer = setInterval(() => setRemoteVelocity(joystickVelocity, joystickTurnVelocity), 50);
      });
      joystick.addEventListener('pointermove', (event) => {
        if (!joystickActive) return;
        event.preventDefault();
        updateJoystickFromPointer(event);
        setRemoteVelocity(joystickVelocity, joystickTurnVelocity);
      });
      joystick.addEventListener('pointerup', () => stopRemoteVelocity());
      joystick.addEventListener('pointercancel', () => stopRemoteVelocity());
      joystick.addEventListener('lostpointercapture', () => stopRemoteVelocity());
      window.addEventListener('blur', () => stopRemoteVelocity());
    }
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
    async function saveBalanceTuning() {
      const params = new URLSearchParams({
        target: document.getElementById('balanceTarget').value,
        kp: document.getElementById('balanceKp').value,
        kd: document.getElementById('balanceKd').value,
        kv: document.getElementById('balanceKv').value,
        dir: document.getElementById('balanceDir').value,
        maxv: document.getElementById('balanceMaxV').value,
        starta: document.getElementById('balanceStartA').value,
        maxa: document.getElementById('balanceMaxA').value,
        save: '1'
      });
      const res = await fetch('/api/balance?' + params.toString(), { method: 'POST' });
      document.getElementById('balanceSaveStatus').innerText = res.ok ? '参数保存: 已保存，重启后自动恢复' : '参数保存: 失败';
      updateStatus();
    }
    async function setBalance(enable) {
      await applyBalanceTuning();
      await fetch('/api/balance?enable=' + encodeURIComponent(enable), { method: 'POST' });
      updateStatus();
    }
    function motorText(label, m) {
      if (!m) return label + ': --';
      return label + ': 目标 ' + Number(m.targetVelocity).toFixed(2) +
        ' / 实测 ' + Number(m.measuredVelocity).toFixed(2) +
        ' / 误差 ' + Number(m.velocityError || 0).toFixed(2);
    }
    function balanceText(b, leftMotor, rightMotor) {
      if (!b) return '平衡状态: --';
      return '平衡状态: ' + (b.enabled ? '已开启' : '关闭') +
        ' | 输出: ' + Number(b.outputVelocity).toFixed(2) + ' rad/s<br>' +
        '遥控: ' + Number(b.remoteVelocity || 0).toFixed(2) + ' / ' +
        Number(b.remoteTurnVelocity || 0).toFixed(2) + ' rad/s' +
        ' | 最大轮速: ' + Number(b.maxVelocity).toFixed(2) + ' rad/s<br>' +
        'Pitch: ' + Number(b.pitch).toFixed(2) + '°' +
        ' | Rate: ' + Number(b.pitchRate).toFixed(2) + ' °/s<br>' +
        '轮速: ' + Number(b.wheelVelocity).toFixed(2) + ' rad/s<br>' +
        'Kp/Kd/Kv: ' + Number(b.kp).toFixed(3) + ' / ' + Number(b.kd).toFixed(3) + ' / ' + Number(b.kv).toFixed(3) +
        ' | Dir: ' + Number(b.direction).toFixed(0) + '<br>' +
        '启动/保护角: ' + Number(b.startAngle).toFixed(1) + '° / ' + Number(b.maxAngle).toFixed(1) + '°' +
        ' | 保护: ' + (b.fault ? '触发' : '正常') + '<br>' +
        motorText('左轮', leftMotor) + '<br>' +
        motorText('右轮', rightMotor);
    }
    function syncBalanceInputs(b) {
      if (!b || balanceInputsSynced) return;
      if (!(Number(b.maxVelocity) > 0 && Number(b.maxAngle) > 0)) return;
      document.getElementById('balanceTarget').value = Number(b.targetPitch).toFixed(2);
      document.getElementById('balanceKp').value = Number(b.kp).toFixed(3);
      document.getElementById('balanceKd').value = Number(b.kd).toFixed(3);
      document.getElementById('balanceKv').value = Number(b.kv).toFixed(3);
      document.getElementById('balanceDir').value = Number(b.direction).toFixed(0);
      document.getElementById('balanceMaxV').value = Number(b.maxVelocity).toFixed(2);
      document.getElementById('balanceStartA').value = Number(b.startAngle).toFixed(1);
      document.getElementById('balanceMaxA').value = Number(b.maxAngle).toFixed(1);
      balanceInputsSynced = true;
    }
    async function updateStatus() {
      try {
        const res = await fetch('/api/status', { cache: 'no-store' });
        const data = await res.json();
        document.getElementById('uptime').innerText = '通信状态: 正常 | 运行: ' + data.uptime + ' | WiFi: ' + data.wifiMode + ' ' + data.ip;
        document.getElementById('balanceStatus').innerHTML = balanceText(data.balance, data.leftMotor, data.rightMotor);
        syncBalanceInputs(data.balance);
      } catch (e) {
        document.getElementById('uptime').innerText = '通信状态: 断开';
      }
    }
    bindRemoteJoystick(); updateJoystickUi(0, 0); updateStatus(); setInterval(updateStatus, 800);
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

void WiFiDebugServer::loadSavedBalanceConfig() {
  auto command = runtime_state::balanceCommand();
  if (!loadSavedBalanceCommand(command)) return;

  command.stop = false;
  command.enable = false;
  command.has_enable = false;
  command.has_tuning = true;
  command.updated_ms = millis();
  command.sequence = ++g_balance_command_sequence;
  runtime_state::updateBalanceCommand(command);
}
