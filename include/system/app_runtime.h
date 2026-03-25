#pragma once

namespace app_runtime {

// 创建并启动整机运行时任务框架。
void begin();

// 返回运行时框架是否已经启动。
bool started();

}  // namespace app_runtime
