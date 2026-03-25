#include "system/wifi_debug_server.h"

#include <Arduino.h>
#include <WiFi.h>

#include <cstring>

#include "config/wifi_debug_config.h"
#include "system/runtime_state.h"

namespace {

bool hasConfiguredStationCredentials() {
  // 仍为默认占位符时，视为用户尚未配置真实热点信息。
  return std::strlen(wifi_debug_config::kStaSsid) > 0 &&
         std::strcmp(wifi_debug_config::kStaSsid, "YOUR_PHONE_WIFI_SSID") != 0;
}

String wifiModeToString(const bool station_connected) {
  return station_connected ? "STA" : "AP";
}

String boolToLabel(const bool value, const char* true_label, const char* false_label) {
  return value ? String(true_label) : String(false_label);
}

}  // namespace

WiFiDebugServer& WiFiDebugServer::instance() {
  static WiFiDebugServer server;
  return server;
}

WiFiDebugServer::WiFiDebugServer()
    : server_(80), started_(false), stationConnected_(false) {}

void WiFiDebugServer::begin() {
  if (started_) {
    return;
  }

  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);

  // 启动前先注册网页与接口路由。
  configureRoutes();

  // 优先尝试连接手机热点或外部 Wi-Fi。
  stationConnected_ = connectToStation();
  if (!stationConnected_) {
    // 连接失败后自动切换为 AP 模式，方便现场调试。
    startFallbackAccessPoint();
  }

  server_.begin();
  started_ = true;

  Serial.println();
  Serial.println("[WiFi] Debug server started");
  Serial.print("[WiFi] Mode: ");
  Serial.println(wifiModeToString(stationConnected_));
  Serial.print("[WiFi] IP: ");
  Serial.println(currentIpString());
}

void WiFiDebugServer::loop() {
  if (!started_) {
    return;
  }

  server_.handleClient();
}

void WiFiDebugServer::configureRoutes() {
  server_.on("/", HTTP_GET, [this]() { handleRoot(); });
  server_.on("/api/status", HTTP_GET, [this]() { handleStatus(); });
  server_.on("/api/restart", HTTP_POST, [this]() { handleRestart(); });
  server_.onNotFound([this]() {
    server_.send(404, "text/plain", "Not Found");
  });
}

bool WiFiDebugServer::connectToStation() {
  if (!hasConfiguredStationCredentials()) {
    Serial.println("[WiFi] Station credentials are not configured, starting AP fallback");
    return false;
  }

  WiFi.mode(WIFI_MODE_STA);
  WiFi.setHostname(wifi_debug_config::kHostname);
  WiFi.begin(wifi_debug_config::kStaSsid, wifi_debug_config::kStaPassword);

  // 在设定超时时间内轮询连接状态。
  const uint32_t start_ms = millis();
  while (WiFi.status() != WL_CONNECTED &&
         millis() - start_ms < wifi_debug_config::kConnectTimeoutMs) {
    delay(250);
    Serial.print(".");
  }

  Serial.println();
  return WiFi.status() == WL_CONNECTED;
}

void WiFiDebugServer::startFallbackAccessPoint() {
  // 彻底断开 STA 状态后再切到 AP，避免模式残留。
  WiFi.disconnect(true, true);
  delay(100);
  WiFi.mode(WIFI_MODE_AP);
  WiFi.softAP(wifi_debug_config::kFallbackApSsid,
              wifi_debug_config::kFallbackApPassword);
}

void WiFiDebugServer::handleRoot() {
  server_.send(200, "text/html; charset=utf-8", buildDebugPage());
}

void WiFiDebugServer::handleStatus() {
  server_.send(200, "application/json; charset=utf-8", buildStatusJson());
}

void WiFiDebugServer::handleRestart() {
  server_.send(200, "application/json; charset=utf-8",
               "{\"ok\":true,\"message\":\"Device will restart\"}");
  delay(200);
  ESP.restart();
}

String WiFiDebugServer::buildDebugPage() const {
  // 页面保持轻量，重点展示双核任务框架与系统运行状态。
  String html = R"HTML(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Wheel-Leg Runtime Debug</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #f3f6fb;
      --panel: #ffffff;
      --line: #d8e1ef;
      --text: #1b2430;
      --muted: #5f6b7a;
      --accent: #0b7dda;
      --accent-2: #0f5fb5;
      --shadow: 0 16px 40px rgba(15, 35, 95, 0.10);
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: "Segoe UI", "PingFang SC", "Microsoft YaHei", sans-serif;
      background:
        radial-gradient(circle at top left, rgba(11,125,218,0.10), transparent 32%),
        linear-gradient(180deg, #f7f9fd 0%, var(--bg) 100%);
      color: var(--text);
    }
    .wrap {
      width: min(1100px, calc(100% - 32px));
      margin: 32px auto;
    }
    .hero {
      background: linear-gradient(135deg, #0b7dda 0%, #0f5fb5 100%);
      color: #fff;
      border-radius: 24px;
      padding: 28px;
      box-shadow: var(--shadow);
    }
    .hero h1 {
      margin: 0 0 10px;
      font-size: 30px;
    }
    .hero p {
      margin: 0;
      line-height: 1.6;
      opacity: 0.9;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
      gap: 16px;
      margin-top: 20px;
    }
    .card {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 20px;
      padding: 18px;
      box-shadow: var(--shadow);
    }
    .label {
      font-size: 13px;
      color: var(--muted);
      margin-bottom: 10px;
    }
    .value {
      font-size: 22px;
      font-weight: 700;
      word-break: break-word;
    }
    .actions {
      display: flex;
      gap: 12px;
      flex-wrap: wrap;
      margin-top: 20px;
    }
    button {
      border: 0;
      border-radius: 999px;
      padding: 12px 18px;
      background: var(--accent);
      color: #fff;
      font-size: 14px;
      cursor: pointer;
    }
    button.secondary {
      background: #eaf1fb;
      color: var(--accent-2);
    }
    .footer {
      margin-top: 18px;
      color: var(--muted);
      font-size: 13px;
      line-height: 1.6;
    }
    code {
      background: rgba(11,125,218,0.08);
      padding: 2px 6px;
      border-radius: 6px;
    }
  </style>
</head>
<body>
  <div class="wrap">
    <section class="hero">
      <h1>Wheel-Leg Runtime Debug</h1>
      <p>当前已切换为双核任务框架：控制链路与 Wi-Fi/调试链路分核运行。网页仅展示系统状态快照，不直接参与实时控制。</p>
    </section>

    <section class="grid">
      <article class="card"><div class="label">联网模式</div><div class="value" id="mode">--</div></article>
      <article class="card"><div class="label">Wi-Fi 名称</div><div class="value" id="ssid">--</div></article>
      <article class="card"><div class="label">设备 IP</div><div class="value" id="ip">--</div></article>
      <article class="card"><div class="label">信号强度</div><div class="value" id="rssi">--</div></article>
      <article class="card"><div class="label">运行时间</div><div class="value" id="uptime">--</div></article>
      <article class="card"><div class="label">空闲堆内存</div><div class="value" id="heap">--</div></article>
      <article class="card"><div class="label">控制任务核心</div><div class="value" id="controlCore">--</div></article>
      <article class="card"><div class="label">服务任务核心</div><div class="value" id="serviceCore">--</div></article>
      <article class="card"><div class="label">控制任务计数</div><div class="value" id="controlLoops">--</div></article>
      <article class="card"><div class="label">服务任务计数</div><div class="value" id="serviceLoops">--</div></article>
      <article class="card"><div class="label">电驱使能状态</div><div class="value" id="driveEnabled">--</div></article>
      <article class="card"><div class="label">电驱故障电平</div><div class="value" id="driveFault">--</div></article>
      <article class="card"><div class="label">控制周期</div><div class="value" id="controlPeriod">--</div></article>
      <article class="card"><div class="label">服务周期</div><div class="value" id="servicePeriod">--</div></article>
      <article class="card"><div class="label">芯片型号</div><div class="value" id="chip">--</div></article>
      <article class="card"><div class="label">SDK 版本</div><div class="value" id="sdk">--</div></article>
    </section>

    <div class="actions">
      <button onclick="refreshStatus()">刷新状态</button>
      <button class="secondary" onclick="restartDevice()">重启设备</button>
    </div>

    <div class="footer">
      首次使用请修改 <code>include/config/wifi_debug_config.h</code> 中的手机 Wi-Fi 名称和密码。<br>
      后续 FOC、编码器和 IMU 的实时逻辑应继续放在控制任务中，网页仅用于调试和遥测。
    </div>
  </div>

  <script>
    async function refreshStatus() {
      const response = await fetch('/api/status');
      const data = await response.json();
      document.getElementById('mode').textContent = data.mode;
      document.getElementById('ssid').textContent = data.ssid;
      document.getElementById('ip').textContent = data.ip;
      document.getElementById('rssi').textContent = data.rssi;
      document.getElementById('uptime').textContent = data.uptime;
      document.getElementById('heap').textContent = data.heap;
      document.getElementById('controlCore').textContent = data.controlCore;
      document.getElementById('serviceCore').textContent = data.serviceCore;
      document.getElementById('controlLoops').textContent = data.controlLoops;
      document.getElementById('serviceLoops').textContent = data.serviceLoops;
      document.getElementById('driveEnabled').textContent = data.driveEnabled;
      document.getElementById('driveFault').textContent = data.driveFault;
      document.getElementById('controlPeriod').textContent = data.controlPeriod;
      document.getElementById('servicePeriod').textContent = data.servicePeriod;
      document.getElementById('chip').textContent = data.chip;
      document.getElementById('sdk').textContent = data.sdk;
    }

    async function restartDevice() {
      await fetch('/api/restart', { method: 'POST' });
    }

    refreshStatus();
    setInterval(refreshStatus, __REFRESH_MS__);
  </script>
</body>
</html>
)HTML";

  html.replace("__REFRESH_MS__", String(wifi_debug_config::kStatusRefreshMs));
  return html;
}

String WiFiDebugServer::buildStatusJson() const {
  const runtime_state::SystemSnapshot system_snapshot = runtime_state::snapshot();

  // AP 模式下没有 RSSI，因此显示为 N/A。
  String ssid = stationConnected_ ? WiFi.SSID() : wifi_debug_config::kFallbackApSsid;
  String rssi = stationConnected_ ? String(WiFi.RSSI()) + " dBm" : "N/A";

  // 将运行时间转换为更适合网页展示的格式。
  const uint32_t seconds = millis() / 1000;
  const uint32_t hours = seconds / 3600;
  const uint32_t minutes = (seconds % 3600) / 60;
  const uint32_t remain_seconds = seconds % 60;

  String uptime = String(hours) + "h " + String(minutes) + "m " +
                  String(remain_seconds) + "s";

  String json = "{";
  json += "\"mode\":\"" + currentModeLabel() + "\",";
  json += "\"ssid\":\"" + ssid + "\",";
  json += "\"ip\":\"" + currentIpString() + "\",";
  json += "\"rssi\":\"" + rssi + "\",";
  json += "\"uptime\":\"" + uptime + "\",";
  json += "\"heap\":\"" + String(ESP.getFreeHeap()) + " bytes\",";
  json += "\"controlCore\":\"" + String(system_snapshot.control.core_id) + "\",";
  json += "\"serviceCore\":\"" + String(system_snapshot.service.core_id) + "\",";
  json += "\"controlLoops\":\"" + String(system_snapshot.control.loop_counter) + "\",";
  json += "\"serviceLoops\":\"" + String(system_snapshot.service.loop_counter) + "\",";
  json += "\"driveEnabled\":\"" +
          boolToLabel(system_snapshot.control.drive_enabled, "enabled", "disabled") + "\",";
  json += "\"driveFault\":\"" + String(system_snapshot.control.drive_fault_level) + "\",";
  json += "\"controlPeriod\":\"" + String(system_snapshot.control.task_period_ms) + " ms\",";
  json += "\"servicePeriod\":\"" + String(system_snapshot.service.task_period_ms) + " ms\",";
  json += "\"chip\":\"" + String(ESP.getChipModel()) + "\",";
  json += "\"sdk\":\"" + String(ESP.getSdkVersion()) + "\"";
  json += "}";
  return json;
}

String WiFiDebugServer::currentModeLabel() const {
  return stationConnected_ ? "STA connected" : "AP fallback";
}

String WiFiDebugServer::currentIpString() const {
  return stationConnected_ ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
}
