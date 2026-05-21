#include "modules/servo/servo_module.h"

#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "modules/servo/servo_pins.h"

namespace {

// 串口实例定义
HardwareSerial ServoSerial(1);

// 缓存与状态
uint8_t g_ServoRxBuf[64];
uint8_t g_ServoRxFlag = 0;
uint8_t g_ExpectedLen = 0;
uint8_t g_LastCommand = 0;
uint8_t g_LastRxLen = 0;
bool g_LastParseOk = false;
uint16_t g_BatteryMv = 0;
bool g_initialized = false;
servo::ReadResult g_LastReadResult = {false, 0, {}, {}};

// 几何常量定义 (单位: cm)
constexpr float L1 = 10.0f;
constexpr float L2 = 20.0f;
constexpr float L3 = 25.0f;
constexpr float L4 = 15.0f;
constexpr float L5 = 5.0f;

// 辅助宏
#define GET_LOW_BYTE(A) ((uint8_t)(A))
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))

}  // namespace

namespace servo {

void begin() {
    // 根据用户要求，波特率修改为 9600
    ServoSerial.begin(9600, SERIAL_8N1, pins::kRx, pins::kTx);
    g_initialized = true;
}

void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time) {
    if (!g_initialized || servoID > 253) return;

    g_LastCommand = 3;
    uint8_t buf[10];
    buf[0] = buf[1] = 0x55;
    buf[2] = 8;     // 长度
    buf[3] = 3;     // 指令: 写入位置
    buf[4] = 1;     // 舵机数量
    buf[5] = GET_LOW_BYTE(Time);
    buf[6] = GET_HIGH_BYTE(Time);
    buf[7] = servoID;
    buf[8] = GET_LOW_BYTE(Position);
    buf[9] = GET_HIGH_BYTE(Position);

    ServoSerial.write(buf, 10);
}

void moveMulti(BusServo_Move_Param *param_list, uint8_t servo_num) {
    if (!g_initialized || servo_num == 0 || param_list == nullptr) return;

    g_LastCommand = 0x03;
    // 数据长度 = 舵机个数 * 3 + 5
    uint8_t data_len = servo_num * 3 + 5;
    uint8_t total_len = data_len + 2;

    static uint8_t tx_buf[128];
    if (total_len > sizeof(tx_buf)) return;
    
    tx_buf[0] = 0x55;
    tx_buf[1] = 0x55;
    tx_buf[2] = data_len;
    tx_buf[3] = 0x03; // 指令
    tx_buf[4] = servo_num; // 参数 1: 舵机个数

    uint16_t time = param_list[0].time;
    if (time > 30000) time = 30000;
    tx_buf[5] = GET_LOW_BYTE(time);  // 参数 2: 时间低八位
    tx_buf[6] = GET_HIGH_BYTE(time); // 参数 3: 时间高八位

    for (uint8_t i = 0; i < servo_num; i++) {
        uint8_t idx = 7 + i * 3;
        tx_buf[idx] = param_list[i].id;                     // 参数 4/7/...: 舵机 ID
        uint16_t angle = param_list[i].angle;
        if (angle > 1000) angle = 1000;
        tx_buf[idx + 1] = GET_LOW_BYTE(angle);              // 参数 5/8/...: 位置低八位
        tx_buf[idx + 2] = GET_HIGH_BYTE(angle);             // 参数 6/9/...: 位置高八位
    }

    ServoSerial.write(tx_buf, total_len);
}

void readMulti(uint8_t *id_list, uint8_t num) {
    if (!g_initialized || num == 0 || id_list == nullptr) return;

    g_LastCommand = CMD_MULT_SERVO_POS_READ;
    uint8_t data_len = num + 3;
    uint8_t total_len = data_len + 2;
    static uint8_t tx_buf[64];

    tx_buf[0] = 0x55;
    tx_buf[1] = 0x55;
    tx_buf[2] = data_len;
    tx_buf[3] = CMD_MULT_SERVO_POS_READ;
    tx_buf[4] = num;

    for (uint8_t i = 0; i < num; i++) {
        tx_buf[5 + i] = id_list[i];
    }

    g_ExpectedLen = num * 3 + 5;
    g_ServoRxFlag = 0;
    ServoSerial.write(tx_buf, total_len);
}

int8_t parseReadResponse(uint8_t *rx_buf, uint8_t *id_out, uint16_t *pos_out) {
    if (rx_buf[0] != 0x55 || rx_buf[1] != 0x55 || rx_buf[3] != CMD_MULT_SERVO_POS_READ) {
        return -1;
    }
    uint8_t num = rx_buf[4];
    for (uint8_t i = 0; i < num; i++) {
        uint8_t base_idx = 5 + i * 3;
        id_out[i] = rx_buf[base_idx];
        pos_out[i] = (uint16_t)(rx_buf[base_idx + 1] | (rx_buf[base_idx + 2] << 8));
    }
    return num;
}

ReadResult lastReadResult() {
    return g_LastReadResult;
}

void readBatteryVoltage() {
    // 占位实现，实际协议可能不同
    if (!g_initialized) return;
    g_LastCommand = 0x1B; // 假设 0x1B 是电压读取指令
    g_ExpectedLen = 6;
    g_ServoRxFlag = 0;
    // 此处应发送实际指令包
}

bool solveIK(float x, float y, IK_Result *result) {
    // --- 计算 Alpha (左连杆) ---
    float a = 2.0f * x * L1;
    float b = 2.0f * y * L1;
    float c = x * x + y * y + L1 * L1 - L2 * L2;

    float delta_alpha = a * a + b * b - c * c;
    if (delta_alpha < 0) return false;

    float t_alpha = (b + sqrtf(delta_alpha)) / (a + c);
    float alpha_rad = 2.0f * atanf(t_alpha);

    // --- 计算 Beta (右连杆) ---
    float d = 2.0f * (x - L5) * L4;
    float e = 2.0f * y * L4;
    float f = powf(x - L5, 2) + L4 * L4 + y * y - L3 * L3;

    float delta_beta = d * d + e * e - f * f;
    if (delta_beta < 0) return false;

    float t_beta = (e - sqrtf(delta_beta)) / (d + f);
    float beta_rad = 2.0f * atanf(t_beta);

    // --- 角度转换 ---
    result->alpha = alpha_rad * 180.0f / M_PI;
    result->beta = beta_rad * 180.0f / M_PI;

    // --- 4. 舵机映射 (基于 90度 参考值) ---
    // 映射公式: Value = Offset90 + (Angle - 90) * kDegToVal
    constexpr float kDegToVal = 1000.0f / 240.0f;

    // 1,2 号舵机 (第一侧): 发送值增大时角度减小 -> 反向映射
    float v1 = 500.0f - (result->alpha - 90.0f) * kDegToVal;
    float v2 = 500.0f - (result->beta - 90.0f) * kDegToVal;
    
    // 3,4 号舵机 (第二侧): 发送值增大时角度增大 -> 正向映射
    float v3 = 500.0f + (result->alpha - 90.0f) * kDegToVal;
    float v4 = 500.0f + (result->beta - 90.0f) * kDegToVal;

    // 限制范围在 0-1000 并赋值
    result->servo_values[0] = (uint16_t)constrain(v1, 0, 1000);
    result->servo_values[1] = (uint16_t)constrain(v2, 0, 1000);
    result->servo_values[2] = (uint16_t)constrain(v3, 0, 1000);
    result->servo_values[3] = (uint16_t)constrain(v4, 0, 1000);

    return true;
}

Status status() {
    Status s = {};
    s.initialized = g_initialized;
    s.rx_flag = g_ServoRxFlag;
    s.expected_len = g_ExpectedLen;
    s.available_bytes = ServoSerial.available();
    s.last_command = g_LastCommand;
    s.last_rx_len = g_LastRxLen;
    s.last_parse_ok = g_LastParseOk;
    s.battery_mv = g_BatteryMv;
    return s;
}

void update() {
    if (!g_initialized) return;

    // 简易串口接收逻辑，模拟 DMA 效果
    if (g_ExpectedLen > 0 && ServoSerial.available() >= g_ExpectedLen) {
        g_LastRxLen = ServoSerial.readBytes(g_ServoRxBuf, g_ExpectedLen);
        g_ServoRxFlag = 1;
        g_ExpectedLen = 0;

        if (g_LastCommand == CMD_MULT_SERVO_POS_READ) {
            int8_t count = parseReadResponse(g_ServoRxBuf, g_LastReadResult.id, g_LastReadResult.position);
            if (count >= 0) {
                g_LastReadResult.count = count;
                g_LastReadResult.valid = true;
                g_LastParseOk = true;
            } else {
                g_LastReadResult.valid = false;
                g_LastParseOk = false;
            }
        } else if (g_LastCommand == 0x1B) { // 假设的电压指令
            // 解析电压，例如 buf[4] | buf[5]<<8
            g_BatteryMv = 12000; // 模拟值
            g_LastParseOk = true;
        }
    }
}

}  // namespace servo

