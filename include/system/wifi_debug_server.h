#pragma once

#include <WebServer.h>

// Wi-Fi 调试服务：
// 1. 在服务核中运行，避免打断实时控制链路
// 2. 优先以 STA 模式连接手机热点
// 3. 连接失败后自动切换到 AP 模式
// 4. 通过共享命令状态向控制任务提交调试指令，不直接操作电机驱动
class WiFiDebugServer {
 public:
  static WiFiDebugServer& instance();

  // 启动 Wi-Fi 和 HTTP 服务。
  void begin();

  // 在服务任务中持续处理 HTTP 请求。
  void loop();

 private:
  WiFiDebugServer();

  // 注册网页路由与接口。
  void configureRoutes();

  // 尝试连接外部 Wi-Fi。
  bool connectToStation();

  // 启动设备自身的调试热点。
  void startFallbackAccessPoint();

  // 返回主调试页面。
  void handleRoot();

  // 返回设备状态 JSON。
  void handleStatus();

  // 返回 IMU 姿态 JSON。
  void handleAttitude();

  // 处理网页上的重启请求。
  void handleRestart();

  // 处理 IMU 平衡控制指令。
  void handleBalanceCommand();

  // 生成 HTML 调试页面。
  String buildDebugPage() const;

  // 生成状态接口的 JSON 响应。
  String buildStatusJson() const;

  // 生成 IMU 姿态接口的 JSON 响应。
  String buildAttitudeJson() const;

  // 处理接收到的电机测试指令。
  void handleMotorCommand();

  // 获取当前联网模式标签。
  String currentModeLabel() const;

  // 获取当前设备 IP 字符串。
  String currentIpString() const;

  WebServer server_;
  bool started_;
  bool stationConnected_;
};
