#pragma once

#include <stdint.h>

namespace wifi_debug_config {

// 手机热点或调试 Wi-Fi 名称。
constexpr char kStaSsid[] = "zmy";

// 手机热点或调试 Wi-Fi 密码。
constexpr char kStaPassword[] = "zmy060521";

// 设备在局域网中的主机名。
constexpr char kHostname[] = "wheel-leg-debug";

// 当 STA 模式连接失败时，设备会自动开启调试热点。
constexpr char kFallbackApSsid[] = "WheelLeg-Debug";
constexpr char kFallbackApPassword[] = "12345678";

// 连接外部 Wi-Fi 的超时时间，单位为毫秒。
constexpr uint32_t kConnectTimeoutMs = 15000;

// 调试页面自动刷新周期，单位为毫秒。
constexpr uint32_t kStatusRefreshMs = 1000;

}  // namespace wifi_debug_config
