#pragma once

#include <stdint.h>
#include <stdbool.h>

namespace servo {

// --- 宏定义 ---
#define CMD_MULT_SERVO_POS_READ 0x15
#define SERVO_FRAME_HEADER      0x55
#define SERVO_BROADCAST_ID      0xFE

// --- 数据结构 ---
struct BusServo_Move_Param {
    uint8_t  id;        // 舵机ID (0~253)
    uint16_t angle;     // 目标角度 (协议值0~1000，对应0~240度)
    uint16_t time;      // 移动时间 (ms, 0~30000)
    uint16_t feedback;  // 反馈值
};

struct IK_Result {
    float alpha;        // 关节1的角度 (度)
    float beta;         // 关节2的角度 (度)
    uint16_t servo_values[4]; // 1,2,3,4 号舵机的最终协议值
};

struct Status {
    bool initialized;
    uint8_t rx_flag;
    uint8_t expected_len;
    uint32_t available_bytes;
    uint8_t last_command;
    uint8_t last_rx_len;
    bool last_parse_ok;
    uint16_t battery_mv;
};

struct ReadResult {
    bool valid;
    uint8_t count;
    uint8_t id[16];
    uint16_t position[16];
};

// --- 核心接口 ---

// 初始化舵机模块（串口配置）
void begin();

// 运动控制
void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time);
void moveMulti(BusServo_Move_Param *param_list, uint8_t servo_num);

// 状态读取与解析
void readMulti(uint8_t *id_list, uint8_t num);
int8_t parseReadResponse(uint8_t *rx_buf, uint8_t *id_out, uint16_t *pos_out);
ReadResult lastReadResult();
void readBatteryVoltage();

// 逆解算法
bool solveIK(float x, float y, IK_Result *result);

// 模块状态
Status status();

// 周期性更新（处理串口接收）
void update();

}  // namespace servo

