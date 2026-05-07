import re

with open("src/system/wifi_debug_server.cpp", "r", encoding="utf-8") as f:
    content = f.read()

new_build_page = r"""String WiFiDebugServer::buildDebugPage() const {
  // 极简开环电机测试页面
  String html = R"HTML(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>电机测试</title>
  <style>
    body { font-family: sans-serif; text-align: center; padding: 50px; background: #f4f4f9; }
    h1 { color: #333; }
    .card { background: white; padding: 30px; border-radius: 12px; box-shadow: 0 4px 10px rgba(0,0,0,0.1); max-width: 400px; margin: auto; }
    button { padding: 15px 30px; font-size: 18px; margin: 10px; cursor: pointer; border: none; border-radius: 8px; color: white; }
    .start { background: #28a745; }
    .stop { background: #dc3545; }
    .status { margin-top: 20px; font-size: 14px; color: #666; }
  </style>
</head>
<body>
  <div class="card">
    <h1>右轮单驱测试</h1>
    <div>
      <button class="start" onclick="sendCmd(5)">正转 (5rad/s)</button>
      <button class="start" style="background:#007bff;" onclick="sendCmd(-5)">反转 (-5rad/s)</button>
      <button class="stop" onclick="sendCmd(0)">停止 (0rad/s)</button>
    </div>
    <div class="status" id="status">正在连接...</div>
  </div>

  <script>
    async function sendCmd(v) {
      await fetch('/api/motor', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: 'v=' + v
      });
    }

    async function updateStatus() {
      try {
        const res = await fetch('/api/status');
        const data = await res.json();
        document.getElementById('status').innerText = "设备已连接 | 运行时间: " + data.uptime;
      } catch (e) {
        document.getElementById('status').innerText = "连接断开...";
      }
    }
    
    setInterval(updateStatus, __REFRESH_MS__);
    updateStatus();
  </script>
</body>
</html>
)HTML";

  html.replace("__REFRESH_MS__", String(wifi_debug_config::kStatusRefreshMs));
  return html;
}
"""

new_build_json = r"""String WiFiDebugServer::buildStatusJson() const {
  const uint32_t seconds = millis() / 1000;
  String json = "{";
  json += "\"uptime\":\"" + String(seconds) + " 秒\"";
  json += "}";
  return json;
}
"""

content = re.sub(r'String WiFiDebugServer::buildDebugPage\(\) const \{.*?return html;\n\}', new_build_page, content, flags=re.DOTALL)
content = re.sub(r'String WiFiDebugServer::buildStatusJson\(\) const \{.*?return json;\n\}', new_build_json, content, flags=re.DOTALL)

with open("src/system/wifi_debug_server.cpp", "w", encoding="utf-8") as f:
    f.write(content)
