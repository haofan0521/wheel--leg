import codecs

with codecs.open('src/system/wifi_debug_server.cpp', 'r', 'utf-8', errors='ignore') as f:
    content = f.read()

start_str = 'String WiFiDebugServer::buildDebugPage() const {'
end_str = 'String WiFiDebugServer::buildStatusJson() const {'

start_idx = content.find(start_str)
end_idx = content.find(end_str)

if start_idx != -1 and end_idx != -1:
    new_html = '''String WiFiDebugServer::buildDebugPage() const {
  String html = R"RAW(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Wheel-Leg Motor Test</title>
  <style>
    body { font-family: sans-serif; background: #f0f2f5; text-align: center; padding: 20px; }
    .card { background: white; padding: 30px; border-radius: 12px; box-shadow: 0 4px 12px rgba(0,0,0,0.1); max-width: 400px; margin: 0 auto; }
    button { padding: 15px 30px; font-size: 18px; border: none; border-radius: 8px; margin: 10px; cursor: pointer; color: white; display: inline-block; width: 40%;}
    .btn-fwd { background: #4caf50; }
    .btn-rev { background: #2196f3; }
    .btn-stop { background: #f44336; width: 85%; }
    .input-grp { margin: 20px 0; }
    .status { margin-top: 20px; font-size: 14px; color: #666; }
  </style>
</head>
<body>
  <div class="card">
    <h2>电机闭环调试</h2>
    
    <div class="input-grp">
      <label>目标速度 (rad/s): </label>
      <input type="number" id="speed" value="5.0" step="1.0" style="font-size: 18px; width: 80px; text-align: center;">
    </div>
    
    <button class="btn-fwd" onclick="setSpeed(document.getElementById('speed').value)">正转</button>
    <button class="btn-rev" onclick="setSpeed(-document.getElementById('speed').value)">反转</button>
    <button class="btn-stop" onclick="setSpeed(0)">停止</button>

    <hr style="margin: 20px 0; border: 0; border-top: 1px solid #ddd;">
    
    <div class="input-grp">
      <label>电压重设 (V): </label>
      <input type="number" id="voltage" value="3.0" step="0.5" style="font-size: 14px; width: 60px; text-align: center;">
      <button onclick="setVoltage()" style="width: auto; padding: 5px 10px; font-size: 14px; background: #607d8b;">应用</button>
    </div>

    <div class="status" id="uptime">连接中...</div>
  </div>

  <script>
    async function setSpeed(v) { await fetch('/api/motor?v=' + v, { method: 'POST' }); }
    async function setVoltage() { let l = document.getElementById('voltage').value; await fetch('/api/motor?l=' + l, { method: 'POST' }); }
    async function updateStatus() {
      try {
        let res = await fetch('/api/status');
        let data = await res.json();
        document.getElementById('uptime').innerText = "通信状态: 正常 | 运行: " + data.uptime;
      } catch (e) { document.getElementById('uptime').innerText = "通信状态: 断开"; }
    }
    setInterval(updateStatus, 1000);
  </script>
</body>
</html>
)RAW";
  return html;
}

'''
    
    new_content = content[:start_idx] + new_html + content[end_idx:]
    with codecs.open('src/system/wifi_debug_server.cpp', 'w', 'utf-8') as f:
        f.write(new_content)
    print('Fixed encoding.')
