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
  char buffer[768];
  snprintf(buffer, sizeof(buffer),
           "{\"enabled\":%s,\"active\":%s,\"fault\":%s,"
           "\"targetPitch\":%.3f,\"pitch\":%.3f,\"pitchRate\":%.3f,"
           "\"wheelVelocity\":%.3f,\"outputVelocity\":%.3f,"
           "\"kp\":%.4f,\"kd\":%.4f,\"kv\":%.4f,"
           "\"useLqr\":%s,\"lqrPitch\":%.6f,\"lqrPitchRate\":%.6f,\"lqrWheelVelocity\":%.6f,"
           "\"lqrOutputSlewRate\":%.3f,"
           "\"direction\":%.1f,"
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
           balance.use_lqr ? "true" : "false",
           balance.lqr_pitch,
           balance.lqr_pitch_rate,
           balance.lqr_wheel_velocity,
           balance.lqr_output_slew_rate,
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
  const auto command = runtime_state::servoCommand();
  String json = "{\"initialized\":";
  json += status.initialized ? "true" : "false";
  json += ",\"targetX\":";
  json += String(command.target_x);
  json += ",\"targetHeight\":";
  json += String(command.target_height);
  json += ",\"moveTimeMs\":";
  json += String(command.time_ms);
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
uint32_t g_tau_test_sequence = 0;
constexpr float kDefaultLegTargetX = -3.5f;
constexpr float kDefaultLegHeightCm = 20.0f;
constexpr char kBalancePrefsNamespace[] = "balance";
constexpr char kBalancePrefsMagicKey[] = "magic";
constexpr uint32_t kBalancePrefsMagic = 0xB14AACE1;
constexpr char kServoPrefsNamespace[] = "servo";
constexpr char kServoPrefsMagicKey[] = "magic";
constexpr uint32_t kServoPrefsMagic = 0x5E120001;

struct TauChannelState {
  float threshold_velocity = 0.0f;
  float start_velocity = 0.0f;
  float final_velocity = 0.0f;
  float measured_velocity = 0.0f;
  float tau_ms = 0.0f;
  bool reached = false;
  uint32_t reached_ms = 0;
};

struct TauTestState {
  bool running = false;
  bool done = false;
  bool command_posted = false;
  float target_velocity = 0.0f;
  TauChannelState left;
  TauChannelState right;
  TauChannelState average;
  uint32_t start_ms = 0;
  uint32_t duration_ms = 0;
  uint32_t arm_ms = 0;
  uint32_t sequence = 0;
};

TauTestState g_tau_test;

float leftForwardWheelVelocity(const runtime_state::SystemSnapshot& state) {
  return state.control.left_motor.measured_velocity;
}

float rightForwardWheelVelocity(const runtime_state::SystemSnapshot& state) {
  return -state.control.right_motor.measured_velocity;
}

float averageForwardWheelVelocity(const runtime_state::SystemSnapshot& state) {
  // 与 app_runtime.cpp 中的 wheel_velocity 方向约定保持一致：
  // 左轮实测速度直接作为车体前进方向，右轮实测速度取反后作为车体前进方向。
  return 0.5f * (leftForwardWheelVelocity(state) + rightForwardWheelVelocity(state));
}

void resetTauChannel(TauChannelState& channel,
                     const float start_velocity,
                     const float target_velocity) {
  channel.start_velocity = start_velocity;
  channel.measured_velocity = start_velocity;
  channel.threshold_velocity = start_velocity + 0.632f * (target_velocity - start_velocity);
  channel.final_velocity = start_velocity;
  channel.tau_ms = 0.0f;
  channel.reached = false;
  channel.reached_ms = 0;
}

void updateTauChannel(TauChannelState& channel,
                      const float measured_velocity,
                      const float target_velocity,
                      const uint32_t now_ms,
                      const uint32_t start_ms) {
  channel.measured_velocity = measured_velocity;
  const bool positive_step = target_velocity >= channel.start_velocity;
  const bool reached = positive_step ?
                       measured_velocity >= channel.threshold_velocity :
                       measured_velocity <= channel.threshold_velocity;
  if (!channel.reached && reached) {
    channel.reached = true;
    channel.reached_ms = now_ms;
    channel.tau_ms = static_cast<float>(now_ms - start_ms);
  }
}

String tauChannelJson(const TauChannelState& channel) {
  char buffer[192];
  snprintf(buffer, sizeof(buffer),
           "{\"reached\":%s,\"startVelocity\":%.3f,\"measuredVelocity\":%.3f,"
           "\"thresholdVelocity\":%.3f,\"finalVelocity\":%.3f,\"tauMs\":%.1f}",
           channel.reached ? "true" : "false",
           channel.start_velocity,
           channel.measured_velocity,
           channel.threshold_velocity,
           channel.final_velocity,
           channel.tau_ms);
  return String(buffer);
}

void resetMotorCommandForTau(runtime_state::MotorCommand& command,
                             const float target_velocity,
                             const uint32_t now_ms,
                             const uint32_t sequence) {
  command.stop = false;
  command.enable = true;
  command.has_enable = true;
  command.has_velocity_target = true;
  command.has_voltage_limit = false;
  command.has_open_loop = false;
  command.has_tuning = false;
  command.open_loop = false;
  command.target_velocity = target_velocity;
  command.updated_ms = now_ms;
  command.sequence = sequence;
}

void postBothMotorVelocityForTau(const float target_velocity) {
  const uint32_t now_ms = millis();
  const uint32_t sequence = ++g_motor_command_sequence;

  auto left_command = runtime_state::leftMotorCommand();
  auto right_command = runtime_state::rightMotorCommand();
  resetMotorCommandForTau(left_command, target_velocity, now_ms, sequence);
  resetMotorCommandForTau(right_command, target_velocity, now_ms, sequence);
  runtime_state::updateLeftMotorCommand(left_command);
  runtime_state::updateRightMotorCommand(right_command);
}

void stopBothMotorsForTau() {
  const uint32_t now_ms = millis();
  const uint32_t sequence = ++g_motor_command_sequence;

  auto left_command = runtime_state::leftMotorCommand();
  auto right_command = runtime_state::rightMotorCommand();
  resetMotorCommandForTau(left_command, 0.0f, now_ms, sequence);
  resetMotorCommandForTau(right_command, 0.0f, now_ms, sequence);
  left_command.stop = true;
  right_command.stop = true;
  runtime_state::updateLeftMotorCommand(left_command);
  runtime_state::updateRightMotorCommand(right_command);
}

String tauTestJson() {
  char buffer[384];
  snprintf(buffer, sizeof(buffer),
           "{\"running\":%s,\"done\":%s,\"reached\":%s,"
           "\"targetVelocity\":%.3f,\"startVelocity\":%.3f,\"measuredVelocity\":%.3f,"
           "\"thresholdVelocity\":%.3f,\"finalVelocity\":%.3f,"
           "\"tauMs\":%.1f,\"elapsedMs\":%lu,\"durationMs\":%lu,\"armed\":%s,\"sequence\":%lu",
           g_tau_test.running ? "true" : "false",
           g_tau_test.done ? "true" : "false",
           g_tau_test.average.reached ? "true" : "false",
           g_tau_test.target_velocity,
           g_tau_test.average.start_velocity,
           g_tau_test.average.measured_velocity,
           g_tau_test.average.threshold_velocity,
           g_tau_test.average.final_velocity,
           g_tau_test.average.tau_ms,
           (unsigned long)(g_tau_test.running && g_tau_test.command_posted ? millis() - g_tau_test.start_ms : 0),
           (unsigned long)g_tau_test.duration_ms,
           g_tau_test.command_posted ? "true" : "false",
           (unsigned long)g_tau_test.sequence);
  String json(buffer);
  json += ",\"left\":";
  json += tauChannelJson(g_tau_test.left);
  json += ",\"right\":";
  json += tauChannelJson(g_tau_test.right);
  json += ",\"average\":";
  json += tauChannelJson(g_tau_test.average);
  json += "}";
  return json;
}

bool loadSavedBalanceCommand(runtime_state::BalanceCommand& command) {
  Preferences prefs;
  if (!prefs.begin(kBalancePrefsNamespace, true)) return false;

  const bool valid = prefs.getUInt(kBalancePrefsMagicKey, 0) == kBalancePrefsMagic;
  if (valid) {
    command.target_pitch_deg = clampFloat(prefs.getFloat("target", command.target_pitch_deg), -20.0f, 20.0f);
    command.kp = clampFloat(prefs.getFloat("kp", command.kp), -20.0f, 20.0f);
    command.kd = clampFloat(prefs.getFloat("kd", command.kd), -5.0f, 5.0f);
    command.kv = clampFloat(prefs.getFloat("kv", command.kv), 0.0f, 5.0f);
    command.use_lqr = prefs.getBool("use_lqr", command.use_lqr);
    command.lqr_pitch = clampFloat(prefs.getFloat("lqr_p", command.lqr_pitch), -500.0f, 500.0f);
    command.lqr_pitch_rate = clampFloat(prefs.getFloat("lqr_d", command.lqr_pitch_rate), -100.0f, 100.0f);
    command.lqr_wheel_velocity = clampFloat(prefs.getFloat("lqr_v", command.lqr_wheel_velocity), -20.0f, 20.0f);
    command.lqr_output_slew_rate = clampFloat(prefs.getFloat("lqr_slew", command.lqr_output_slew_rate), 10.0f, 500.0f);
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
  ok = prefs.putBool("use_lqr", command.use_lqr) > 0 && ok;
  ok = prefs.putFloat("lqr_p", command.lqr_pitch) == sizeof(float) && ok;
  ok = prefs.putFloat("lqr_d", command.lqr_pitch_rate) == sizeof(float) && ok;
  ok = prefs.putFloat("lqr_v", command.lqr_wheel_velocity) == sizeof(float) && ok;
  ok = prefs.putFloat("lqr_slew", command.lqr_output_slew_rate) == sizeof(float) && ok;
  ok = prefs.putFloat("dir", command.output_direction) == sizeof(float) && ok;
  ok = prefs.putFloat("maxv", command.max_velocity) == sizeof(float) && ok;
  ok = prefs.putFloat("starta", command.start_angle_deg) == sizeof(float) && ok;
  ok = prefs.putFloat("maxa", command.max_angle_deg) == sizeof(float) && ok;
  ok = prefs.putUInt(kBalancePrefsMagicKey, kBalancePrefsMagic) == sizeof(uint32_t) && ok;

  prefs.end();
  return ok;
}

bool loadSavedServoCommand(runtime_state::ServoCommand& command) {
  Preferences prefs;
  if (!prefs.begin(kServoPrefsNamespace, true)) return false;

  const bool valid = prefs.getUInt(kServoPrefsMagicKey, 0) == kServoPrefsMagic;
  if (valid) {
    command.target_x = clampFloat(prefs.getFloat("target_x", command.target_x), -5.0f, 10.0f);
    command.target_height = clampFloat(prefs.getFloat("height", command.target_height), 10.0f, 35.0f);
    command.time_ms = static_cast<uint16_t>(clampFloat(prefs.getFloat("time_ms", command.time_ms),
                                                       0.0f,
                                                       10000.0f));
  }

  prefs.end();
  return valid;
}

bool saveServoCommand(const runtime_state::ServoCommand& command) {
  Preferences prefs;
  if (!prefs.begin(kServoPrefsNamespace, false)) return false;

  bool ok = true;
  ok = prefs.putFloat("target_x", command.target_x) == sizeof(float) && ok;
  ok = prefs.putFloat("height", command.target_height) == sizeof(float) && ok;
  ok = prefs.putFloat("time_ms", static_cast<float>(command.time_ms)) == sizeof(float) && ok;
  ok = prefs.putUInt(kServoPrefsMagicKey, kServoPrefsMagic) == sizeof(uint32_t) && ok;

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
  loadSavedServoConfig();
  configureRoutes();
  stationConnected_ = connectToStation();
  if (!stationConnected_) startFallbackAccessPoint();
  server_.begin();
  started_ = true;
}

void WiFiDebugServer::loop() {
  if (!started_) return;
  server_.handleClient();
  updateTauTest();
}

void WiFiDebugServer::configureRoutes() {
  server_.on("/", HTTP_GET, [this]() { handleRoot(); });
  server_.on("/api/status", HTTP_GET, [this]() { handleStatus(); });
  server_.on("/api/attitude", HTTP_GET, [this]() { handleAttitude(); });
  server_.on("/api/restart", HTTP_POST, [this]() { handleRestart(); });
  server_.on("/api/motor", HTTP_POST, [this]() { handleMotorCommand(); });
  server_.on("/api/balance", HTTP_POST, [this]() { handleBalanceCommand(); });
  server_.on("/api/tau_test", HTTP_POST, [this]() { handleTauTestCommand(); });
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
      server_.hasArg("kv") || server_.hasArg("lqr") || server_.hasArg("mode") ||
      server_.hasArg("lqrp") || server_.hasArg("lqrd") || server_.hasArg("lqrv") ||
      server_.hasArg("lqrslew") ||
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
    if (server_.hasArg("mode")) {
      const String mode = server_.arg("mode");
      command.use_lqr = mode == "lqr" || mode == "LQR" || mode == "1";
    } else if (server_.hasArg("lqr")) {
      command.use_lqr = server_.arg("lqr").toInt() != 0;
    } else {
      command.use_lqr = has_live_config ? balance.use_lqr : command.use_lqr;
    }
    command.lqr_pitch = server_.hasArg("lqrp") ? clampFloat(server_.arg("lqrp").toFloat(), -500.0f, 500.0f) :
                        (has_live_config ? balance.lqr_pitch : command.lqr_pitch);
    command.lqr_pitch_rate = server_.hasArg("lqrd") ? clampFloat(server_.arg("lqrd").toFloat(), -100.0f, 100.0f) :
                             (has_live_config ? balance.lqr_pitch_rate : command.lqr_pitch_rate);
    command.lqr_wheel_velocity = server_.hasArg("lqrv") ? clampFloat(server_.arg("lqrv").toFloat(), -20.0f, 20.0f) :
                                 (has_live_config ? balance.lqr_wheel_velocity : command.lqr_wheel_velocity);
    command.lqr_output_slew_rate = server_.hasArg("lqrslew") ? clampFloat(server_.arg("lqrslew").toFloat(), 10.0f, 500.0f) :
                                   (has_live_config ? balance.lqr_output_slew_rate : command.lqr_output_slew_rate);
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

void WiFiDebugServer::handleTauTestCommand() {
  if (server_.hasArg("stop")) {
    stopBothMotorsForTau();
    g_tau_test.running = false;
    g_tau_test.done = false;
    g_tau_test.left.reached = false;
    g_tau_test.right.reached = false;
    g_tau_test.average.reached = false;
    server_.send(200, "application/json; charset=utf-8", "{\"ok\":true,\"stopped\":true}");
    return;
  }

  if (g_tau_test.running) {
    server_.send(409, "application/json; charset=utf-8", "{\"ok\":false,\"error\":\"tau test running\"}");
    return;
  }

  // tau 测试会接管左右轮速度命令，测试前先关闭平衡，避免平衡输出叠加干扰阶跃响应。
  auto balance_command = runtime_state::balanceCommand();
  balance_command.stop = true;
  balance_command.enable = false;
  balance_command.has_enable = true;
  balance_command.has_tuning = false;
  balance_command.updated_ms = millis();
  balance_command.sequence = ++g_balance_command_sequence;
  runtime_state::updateBalanceCommand(balance_command);

  const auto state = runtime_state::snapshot();
  const float target_velocity = server_.hasArg("v") ?
                                clampFloat(server_.arg("v").toFloat(), -6.0f, 6.0f) :
                                2.0f;
  const uint32_t duration_ms = server_.hasArg("duration") ?
                               static_cast<uint32_t>(clampFloat(server_.arg("duration").toFloat(), 500.0f, 8000.0f)) :
                               2500;

  g_tau_test = {};
  g_tau_test.running = true;
  g_tau_test.done = false;
  g_tau_test.command_posted = false;
  g_tau_test.target_velocity = target_velocity;
  resetTauChannel(g_tau_test.left, leftForwardWheelVelocity(state), target_velocity);
  resetTauChannel(g_tau_test.right, rightForwardWheelVelocity(state), target_velocity);
  resetTauChannel(g_tau_test.average, averageForwardWheelVelocity(state), target_velocity);
  g_tau_test.start_ms = millis();
  g_tau_test.arm_ms = g_tau_test.start_ms;
  g_tau_test.duration_ms = duration_ms;
  g_tau_test.sequence = ++g_tau_test_sequence;

  server_.send(200, "application/json; charset=utf-8",
               String("{\"ok\":true,\"tauTest\":") + tauTestJson() + "}");
}

void WiFiDebugServer::updateTauTest() {
  if (!g_tau_test.running) return;

  const uint32_t now_ms = millis();
  const auto state = runtime_state::snapshot();
  const float left_velocity = leftForwardWheelVelocity(state);
  const float right_velocity = rightForwardWheelVelocity(state);
  const float average_velocity = 0.5f * (left_velocity + right_velocity);
  g_tau_test.left.measured_velocity = left_velocity;
  g_tau_test.right.measured_velocity = right_velocity;
  g_tau_test.average.measured_velocity = average_velocity;

  if (!g_tau_test.command_posted) {
    if (now_ms - g_tau_test.arm_ms < 120) return;
    g_tau_test.command_posted = true;
    g_tau_test.start_ms = now_ms;
    resetTauChannel(g_tau_test.left, left_velocity, g_tau_test.target_velocity);
    resetTauChannel(g_tau_test.right, right_velocity, g_tau_test.target_velocity);
    resetTauChannel(g_tau_test.average, average_velocity, g_tau_test.target_velocity);
    postBothMotorVelocityForTau(g_tau_test.target_velocity);
    return;
  }

  updateTauChannel(g_tau_test.left, left_velocity, g_tau_test.target_velocity, now_ms, g_tau_test.start_ms);
  updateTauChannel(g_tau_test.right, right_velocity, g_tau_test.target_velocity, now_ms, g_tau_test.start_ms);
  updateTauChannel(g_tau_test.average, average_velocity, g_tau_test.target_velocity, now_ms, g_tau_test.start_ms);

  if (now_ms - g_tau_test.start_ms >= g_tau_test.duration_ms) {
    g_tau_test.running = false;
    g_tau_test.done = true;
    g_tau_test.left.final_velocity = left_velocity;
    g_tau_test.right.final_velocity = right_velocity;
    g_tau_test.average.final_velocity = average_velocity;
    postBothMotorVelocityForTau(0.0f);
  }
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
    const bool save_requested = server_.hasArg("save") && server_.arg("save").toInt() != 0;
    const bool saved = save_requested && saveServoCommand(command);
    if (save_requested && !saved) {
      server_.send(500, "application/json; charset=utf-8", "{\"ok\":false,\"error\":\"save failed\"}");
      return;
    }
    server_.send(200, "application/json; charset=utf-8",
                 String("{\"ok\":true,\"saved\":") + (saved ? "true" : "false") +
                 ",\"x\":" + String(target_x) +
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
    .joystick-wrap { display: flex; flex-direction: column; align-items: center; gap: 10px; margin: 14px 0 18px; }
    .joystick { position: relative; width: 170px; height: 170px; border-radius: 50%; background: #eef2f7; border: 2px solid #cbd5e1; box-shadow: inset 0 2px 8px rgba(15,23,42,0.12); touch-action: none; user-select: none; }
    .joystick::before, .joystick::after { content: ""; position: absolute; background: #cbd5e1; left: 50%; top: 50%; transform: translate(-50%, -50%); }
    .joystick::before { width: 2px; height: 128px; }
    .joystick::after { width: 128px; height: 2px; }
    .joystick-knob { position: absolute; left: 50%; top: 50%; width: 64px; height: 64px; border-radius: 50%; background: #2563eb; transform: translate(-50%, -50%); box-shadow: 0 8px 18px rgba(37,99,235,0.35); touch-action: none; }
    .joystick-value { min-height: 22px; font-size: 14px; color: #475569; }
    .height-preset { margin-top: 12px; padding: 12px; background: #f8fafc; border-radius: 8px; text-align: left; }
    .height-preset-head { display: flex; align-items: baseline; justify-content: space-between; gap: 12px; color: #334155; font-size: 13px; }
    .height-preset-value { color: #0f172a; font-size: 20px; font-weight: bold; }
    .height-preset input[type="range"] { width: 100%; margin: 10px 0 4px; }
    .height-preset-scale { display: flex; justify-content: space-between; color: #64748b; font-size: 12px; }
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
      <h3>轮速 Tau 测试</h3>
      <div class="balance-grid">
        <label>阶跃速度<input type="number" id="tauVelocity" value="2.0" step="0.2"></label>
        <label>测试时长(ms)<input type="number" id="tauDuration" value="2500" step="500"></label>
      </div>
      <button onclick="startTauTest()" style="background:#7c3aed;">开始 Tau 测试</button>
      <button onclick="stopTauTest()" style="background:#f44336;">停止 Tau 测试</button>
      <div class="status" id="tauStatus">Tau 测试: --</div>
      <h3>IMU 平衡</h3>
      <div class="input-grp">
        <label>前后速度 (rad/s): </label>
        <input type="number" id="remoteSpeed" value="5" step="0.1" min="0" max="5" style="font-size: 18px; width: 90px; text-align: center;">
      </div>
      <div class="input-grp">
        <label>转向速度 (rad/s): </label>
        <input type="number" id="remoteTurnSpeed" value="5" step="0.1" min="0" max="5" style="font-size: 18px; width: 90px; text-align: center;">
      </div>
      <div class="joystick-wrap">
        <div class="joystick" id="remoteJoystick">
          <div class="joystick-knob" id="remoteJoystickKnob"></div>
        </div>
        <div class="joystick-value" id="remoteJoystickValue">前后: 0.00 | 转向: 0.00 rad/s</div>
      </div>
      <div class="balance-grid">
        <label>控制模式<select id="balanceMode" style="width:100%; font-size:16px; padding:8px; margin-top:4px;"><option value="pid">PID/PD+Kv</option><option value="lqr">LQR</option></select></label>
        <label>目标 Pitch<input type="number" id="balanceTarget" value="0.795" step="0.1"></label>
        <label>Kp<input type="number" id="balanceKp" value="1.69" step="0.05"></label>
        <label>Kd<input type="number" id="balanceKd" value="0.028" step="0.005"></label>
        <label>Kv<input type="number" id="balanceKv" value="0.5" step="0.02"></label>
        <label>LQR Pitch<input type="number" id="balanceLqrP" value="120.86971" step="1.0"></label>
        <label>LQR Pitch Rate<input type="number" id="balanceLqrD" value="-18.810969" step="0.5"></label>
        <label>LQR Wheel V<input type="number" id="balanceLqrV" value="-1.208787" step="0.05"></label>
        <label>LQR 爬坡(rad/s²)<input type="number" id="balanceLqrSlew" value="120.0" step="20.0"></label>
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
      <h3>幻尔总线舵机</h3>
      <div class="input-grp">
        <label>移动时间 (ms): </label>
        <input type="number" id="servoTime" value="100" step="100" min="0" max="10000" style="font-size: 16px; width: 90px; text-align: center;">
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
      <div class="height-preset">
        <div class="height-preset-head">
          <span>预设高度滑轨</span>
          <span class="height-preset-value"><span id="legHeightSliderValue">20.0</span> cm</span>
        </div>
        <input type="range" id="legHeightSlider" min="20" max="28" step="0.1" value="20">
        <div class="height-preset-scale"><span>20</span><span>22</span><span>24</span><span>26</span><span>28</span></div>
        <div class="status" id="legPresetStatus">拟合参数: --</div>
      </div>
      <div class="balance-grid">
        <label>中心位置 X (cm)<input type="number" id="legTargetX" value="-3.5" step="0.5" min="-5" max="10"></label>
        <label>预设高度 (cm)<input type="number" id="legHeight" value="20" step="1" min="10" max="35"></label>
        <label>右腿中心<input type="number" id="legRightCenter" value="500" step="10" min="0" max="1000"></label>
        <label>左腿中心<input type="number" id="legLeftCenter" value="500" step="10" min="0" max="1000"></label>
        <label>前后差值<input type="number" id="legPitchMix" value="0" step="10" min="-300" max="300"></label>
      </div>
      <button onclick="applyHeight()" style="background:#0f766e;">应用预设高度</button>
      <button onclick="saveLegDefaults()" style="background:#0369a1;">保存腿部参数</button>
      <button onclick="applyLegMix()" style="background:#7c3aed;">应用简易解算</button>
      <div class="status" id="legSaveStatus">腿部参数保存: --</div>
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
    let balanceInputsSynced = false;
    let servoInputsSynced = false;
    let joystickActive = false;
    let joystickVelocity = 0;
    let joystickTurnVelocity = 0;
    let remoteTimer = null;
    let remoteInFlight = false;
    let pendingRemote = null;
    let remoteSequence = 0;
    const heightBalancePreset = [
      { h: 20, pitch: 1.2, kp: 1.65, kd: 0.035, kv: 2.0 },
      { h: 21, pitch: 0.5, kp: 1.68, kd: 0.045, kv: 2.0 },
      { h: 22, pitch: 0.0, kp: 1.68, kd: 0.050, kv: 2.0 },
      { h: 23, pitch: -2.0, kp: 1.68, kd: 0.055, kv: 2.0 },
      { h: 24, pitch: -2.5, kp: 1.70, kd: 0.060, kv: 2.0 },
      { h: 25, pitch: -4.0, kp: 1.72, kd: 0.065, kv: 2.0 },
      { h: 26, pitch: -5.5, kp: 1.75, kd: 0.070, kv: 2.0 },
      { h: 27, pitch: -7.2, kp: 1.82, kd: 0.075, kv: 2.0 },
      { h: 28, pitch: -8.0, kp: 1.85, kd: 0.080, kv: 2.0 }
    ];
    function clampValue(value, minValue, maxValue) {
      return Math.min(maxValue, Math.max(minValue, value));
    }
    function smoothPresetValue(y0, y1, y2, y3, t) {
      const t2 = t * t;
      const t3 = t2 * t;
      return 0.5 * ((2 * y1) + (-y0 + y2) * t +
        (2 * y0 - 5 * y1 + 4 * y2 - y3) * t2 +
        (-y0 + 3 * y1 - 3 * y2 + y3) * t3);
    }
    function balancePresetForHeight(height) {
      const h = clampValue(Number(height) || 20, 20, 28);
      const lowIndex = Math.min(heightBalancePreset.length - 2, Math.max(0, Math.floor(h) - 20));
      const t = h - heightBalancePreset[lowIndex].h;
      const p0 = heightBalancePreset[Math.max(0, lowIndex - 1)];
      const p1 = heightBalancePreset[lowIndex];
      const p2 = heightBalancePreset[lowIndex + 1];
      const p3 = heightBalancePreset[Math.min(heightBalancePreset.length - 1, lowIndex + 2)];
      const value = (key) => smoothPresetValue(p0[key], p1[key], p2[key], p3[key], t);
      return {
        height: h,
        pitch: value('pitch'),
        kp: value('kp'),
        kd: value('kd'),
        kv: value('kv')
      };
    }
    function applyHeightPresetToInputs(height) {
      const preset = balancePresetForHeight(height);
      document.getElementById('legHeightSlider').value = preset.height.toFixed(1);
      document.getElementById('legHeightSliderValue').innerText = preset.height.toFixed(1);
      document.getElementById('legHeight').value = preset.height.toFixed(1);
      document.getElementById('balanceTarget').value = preset.pitch.toFixed(2);
      document.getElementById('balanceKp').value = preset.kp.toFixed(3);
      document.getElementById('balanceKd').value = preset.kd.toFixed(3);
      document.getElementById('balanceKv').value = preset.kv.toFixed(3);
      document.getElementById('legPresetStatus').innerText =
        '拟合参数: Pitch ' + preset.pitch.toFixed(2) +
        ' | Kp ' + preset.kp.toFixed(3) +
        ' | Kd ' + preset.kd.toFixed(3) +
        ' | Kv ' + preset.kv.toFixed(3);
      return preset;
    }
    async function applyHeightPresetNow() {
      applyHeightPresetToInputs(document.getElementById('legHeightSlider').value);
      await applyHeight();
      await applyBalanceTuning();
    }
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
    function shapeJoystickAxis(value, deadband) {
      const absValue = Math.abs(value);
      if (absValue <= deadband) return 0;
      const remapped = (absValue - deadband) / (1 - deadband);
      return Math.sign(value) * remapped * remapped;
    }
    function setJoystickVelocity(x, y) {
      const maxSpeed = Math.abs(Number(document.getElementById('remoteSpeed').value || 0));
      const maxTurnSpeed = Math.abs(Number(document.getElementById('remoteTurnSpeed').value || 0));
      const deadband = 0.12;
      const shapedX = shapeJoystickAxis(x, deadband);
      const shapedY = shapeJoystickAxis(y, deadband);
      // 摇杆显示保留物理位移，速度命令使用死区重映射和二次曲线，便于小速度微调。
      joystickVelocity = -shapedY * maxSpeed;
      joystickTurnVelocity = shapedX * maxTurnSpeed;
      updateJoystickUi(x, y);
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
    function bindHeightPresetSlider() {
      const slider = document.getElementById('legHeightSlider');
      const heightInput = document.getElementById('legHeight');
      slider.addEventListener('input', () => applyHeightPresetToInputs(slider.value));
      slider.addEventListener('change', () => applyHeightPresetNow());
      heightInput.addEventListener('change', () => applyHeightPresetToInputs(heightInput.value));
      applyHeightPresetToInputs(heightInput.value);
    }
    async function startTauTest() {
      const params = new URLSearchParams({
        v: document.getElementById('tauVelocity').value,
        duration: document.getElementById('tauDuration').value
      });
      await fetch('/api/tau_test?' + params.toString(), { method: 'POST' });
      updateStatus();
    }
    async function stopTauTest() {
      await fetch('/api/tau_test?stop=1', { method: 'POST' });
      updateStatus();
    }
    async function applyBalanceTuning() {
      const params = new URLSearchParams({
        target: document.getElementById('balanceTarget').value,
        kp: document.getElementById('balanceKp').value,
        kd: document.getElementById('balanceKd').value,
        kv: document.getElementById('balanceKv').value,
        mode: document.getElementById('balanceMode').value,
        lqrp: document.getElementById('balanceLqrP').value,
        lqrd: document.getElementById('balanceLqrD').value,
        lqrv: document.getElementById('balanceLqrV').value,
        lqrslew: document.getElementById('balanceLqrSlew').value,
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
        mode: document.getElementById('balanceMode').value,
        lqrp: document.getElementById('balanceLqrP').value,
        lqrd: document.getElementById('balanceLqrD').value,
        lqrv: document.getElementById('balanceLqrV').value,
        lqrslew: document.getElementById('balanceLqrSlew').value,
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
      const x = document.getElementById('legTargetX').value;
      const h = document.getElementById('legHeight').value;
      const time = servoTime();
      await fetch(`/api/servo?action=set_height&x=${x}&h=${h}&time=${time}`, { method: 'POST' });
      updateStatus();
    }
    async function saveLegDefaults() {
      applyHeightPresetToInputs(document.getElementById('legHeight').value);
      const x = document.getElementById('legTargetX').value;
      const h = document.getElementById('legHeight').value;
      const time = servoTime();
      const res = await fetch(`/api/servo?action=set_height&x=${x}&h=${h}&time=${time}&save=1`, { method: 'POST' });
      document.getElementById('legSaveStatus').innerText = res.ok ? '腿部参数保存: 已保存，重启后自动恢复' : '腿部参数保存: 失败';
      await saveBalanceTuning();
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
        ' | 模式: ' + (b.useLqr ? 'LQR' : 'PID/PD+Kv') +
        ' | 输出: ' + Number(b.outputVelocity).toFixed(2) + ' rad/s<br>' +
        '遥控: ' + Number(b.remoteVelocity || 0).toFixed(2) + ' / ' +
        Number(b.remoteTurnVelocity || 0).toFixed(2) + ' rad/s' +
        ' | 最大轮速: ' + Number(b.maxVelocity).toFixed(2) + ' rad/s<br>' +
        'Pitch: ' + Number(b.pitch).toFixed(2) + '°' +
        ' | Rate: ' + Number(b.pitchRate).toFixed(2) + ' °/s<br>' +
        '轮速: ' + Number(b.wheelVelocity).toFixed(2) + ' rad/s<br>' +
        'Kp/Kd/Kv: ' + Number(b.kp).toFixed(3) + ' / ' + Number(b.kd).toFixed(3) + ' / ' + Number(b.kv).toFixed(3) +
        '<br>LQR: ' + Number(b.lqrPitch).toFixed(3) + ' / ' +
        Number(b.lqrPitchRate).toFixed(3) + ' / ' +
        Number(b.lqrWheelVelocity).toFixed(3) +
        ' | 爬坡: ' + Number(b.lqrOutputSlewRate).toFixed(1) +
        ' | Dir: ' + Number(b.direction).toFixed(0) + '<br>' +
        '启动/保护角: ' + Number(b.startAngle).toFixed(1) + '° / ' + Number(b.maxAngle).toFixed(1) + '°' +
        ' | 保护: ' + (b.fault ? '触发' : '正常') + '<br>' +
        motorText('左轮', leftMotor) + '<br>' +
        motorText('右轮', rightMotor);
    }
    function tauText(t) {
      if (!t) return 'Tau 测试: --';
      const row = (name, c) => {
        if (!c) return name + ': --';
        return name + ': ' +
          'v ' + Number(c.measuredVelocity).toFixed(2) +
          ' / th ' + Number(c.thresholdVelocity).toFixed(2) +
          ' | Tau ' + (c.reached ? Number(c.tauMs).toFixed(1) + ' ms' : '--');
      };
      return 'Tau 测试: ' + (t.running ? '运行中' : (t.done ? '完成' : '待机')) +
        ' | 目标: ' + Number(t.targetVelocity).toFixed(2) + ' rad/s' +
        ' | 经过: ' + Number(t.elapsedMs).toFixed(0) + ' ms<br>' +
        row('平均', t.average || t) + '<br>' +
        row('左轮', t.left) + '<br>' +
        row('右轮', t.right);
    }
    function syncBalanceInputs(b) {
      if (!b || balanceInputsSynced) return;
      if (!(Number(b.maxVelocity) > 0 && Number(b.maxAngle) > 0)) return;
      document.getElementById('balanceMode').value = b.useLqr ? 'lqr' : 'pid';
      document.getElementById('balanceTarget').value = Number(b.targetPitch).toFixed(2);
      document.getElementById('balanceKp').value = Number(b.kp).toFixed(3);
      document.getElementById('balanceKd').value = Number(b.kd).toFixed(3);
      document.getElementById('balanceKv').value = Number(b.kv).toFixed(3);
      document.getElementById('balanceLqrP').value = Number(b.lqrPitch).toFixed(5);
      document.getElementById('balanceLqrD').value = Number(b.lqrPitchRate).toFixed(5);
      document.getElementById('balanceLqrV').value = Number(b.lqrWheelVelocity).toFixed(5);
      document.getElementById('balanceLqrSlew').value = Number(b.lqrOutputSlewRate).toFixed(1);
      document.getElementById('balanceDir').value = Number(b.direction).toFixed(0);
      document.getElementById('balanceMaxV').value = Number(b.maxVelocity).toFixed(2);
      document.getElementById('balanceStartA').value = Number(b.startAngle).toFixed(1);
      document.getElementById('balanceMaxA').value = Number(b.maxAngle).toFixed(1);
      balanceInputsSynced = true;
    }
    function syncServoInputs(s) {
      if (!s || servoInputsSynced) return;
      document.getElementById('legTargetX').value = Number(s.targetX).toFixed(1);
      document.getElementById('legHeight').value = Number(s.targetHeight).toFixed(1);
      document.getElementById('servoTime').value = Number(s.moveTimeMs || 100).toFixed(0);
      applyHeightPresetToInputs(s.targetHeight);
      servoInputsSynced = true;
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
        document.getElementById('balanceStatus').innerHTML = balanceText(data.balance, data.leftMotor, data.rightMotor);
        document.getElementById('tauStatus').innerHTML = tauText(data.tauTest);
        syncBalanceInputs(data.balance);
        syncServoInputs(data.servo);
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
    bindRemoteJoystick(); bindHeightPresetSlider(); updateJoystickUi(0, 0); updateStatus(); updateAttitude(); setInterval(updateStatus, 500); setInterval(updateAttitude, 100);
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
  json += "\"tauTest\":" + tauTestJson() + ",";
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

void WiFiDebugServer::loadSavedServoConfig() {
  auto command = runtime_state::servoCommand();
  if (!loadSavedServoCommand(command)) return;

  command.has_height = false;
  command.updated_ms = millis();
  command.sequence = 0;
  runtime_state::updateServoCommand(command);
}
