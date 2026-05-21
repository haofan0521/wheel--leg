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
bool g_initialized = false;
servo::ReadResult g_last_read_result = {};
uint8_t g_last_command = 0;
uint8_t g_last_rx_len = 0;
bool g_last_parse_ok = false;
uint16_t g_battery_mv = 0;

// 几何常量定义 (单位: mm)
constexpr float L1 = 10.0f;
constexpr float L2 = 20.0f;
constexpr float L3 = 25.0f;
constexpr float L4 = 15.0f;
constexpr float L5 = 5.0f;

// 辅助宏
#define GET_LOW_BYTE(A) ((uint8_t)(A))
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))

void resetReceiveState(uint8_t command, uint8_t expected_len) {
    while (ServoSerial.available() > 0) {
        ServoSerial.read();
    }
    g_last_command = command;
    g_ExpectedLen = expected_len;
    g_ServoRxFlag = 0;
    g_last_rx_len = 0;
    g_last_parse_ok = false;
}

}  // namespace

namespace servo {

void begin() {
    // 初始化串口：幻尔舵机控制板常见默认波特率为 9600，指定 RX/TX 引脚。
    ServoSerial.begin(9600, SERIAL_8N1, pins::kRx, pins::kTx);
    g_initialized = true;
}

void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time) {
    if (!g_initialized || servoID > 253) return;

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

    g_last_command = buf[3];
    ServoSerial.write(buf, 10);
}

void moveMulti(BusServo_Move_Param *param_list, uint8_t servo_num) {
    if (!g_initialized || servo_num == 0 || param_list == nullptr) return;

    uint8_t data_len = servo_num * 3 + 5;
    uint8_t total_len = data_len + 2;

    // 使用静态缓冲区避免频繁 malloc
    static uint8_t tx_buf[128];
    if (total_len > sizeof(tx_buf)) return;

    tx_buf[0] = 0x55;
    tx_buf[1] = 0x55;
    tx_buf[2] = data_len;
    tx_buf[3] = 0x03; // 指令
    tx_buf[4] = servo_num;

    uint16_t time = param_list[0].time;
    if (time > 10000) time = 10000;
    tx_buf[5] = GET_LOW_BYTE(time);
    tx_buf[6] = GET_HIGH_BYTE(time);

    for (uint8_t i = 0; i < servo_num; i++) {
        uint8_t idx = 7 + i * 3;
        tx_buf[idx] = param_list[i].id;
        uint16_t angle = param_list[i].angle;
        if (angle > 1000) angle = 1000;
        tx_buf[idx + 1] = GET_LOW_BYTE(angle);
        tx_buf[idx + 2] = GET_HIGH_BYTE(angle);
    }

    g_last_command = tx_buf[3];
    ServoSerial.write(tx_buf, total_len);
    // 注意：原代码中有 HAL_Delay(300)，在 ESP32 任务中建议移除或改用非阻塞方式
}

void readMulti(uint8_t *id_list, uint8_t num) {
    if (!g_initialized || num == 0 || id_list == nullptr) return;

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

    resetReceiveState(CMD_MULT_SERVO_POS_READ, num * 3 + 5);
    ServoSerial.write(tx_buf, total_len);
}

void readBatteryVoltage() {
    if (!g_initialized) return;

    uint8_t tx_buf[4];
    tx_buf[0] = 0x55;
    tx_buf[1] = 0x55;
    tx_buf[2] = 0x02;
    tx_buf[3] = CMD_GET_BATTERY_VOLTAGE;

    resetReceiveState(CMD_GET_BATTERY_VOLTAGE, 6);
    ServoSerial.write(tx_buf, sizeof(tx_buf));
}

int8_t parseReadResponse(uint8_t *rx_buf, uint8_t *id_out, uint16_t *pos_out) {
    if (rx_buf == nullptr || id_out == nullptr || pos_out == nullptr) {
        return -1;
    }
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
    return g_last_read_result;
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

    // 映射：角度(0-240) -> 协议值(0-1000)
    result->alpha_val = (uint16_t)(result->alpha * 1000.0f / 240.0f);
    result->beta_val = (uint16_t)(result->beta * 1000.0f / 240.0f);

    return true;
}

Status status() {
    Status s = {};
    s.initialized = g_initialized;
    s.rx_flag = g_ServoRxFlag;
    s.expected_len = g_ExpectedLen;
    s.available_bytes = g_initialized ? ServoSerial.available() : 0;
    s.last_command = g_last_command;
    s.last_rx_len = g_last_rx_len;
    s.last_parse_ok = g_last_parse_ok;
    s.battery_mv = g_battery_mv;
    return s;
}

void update() {
    if (!g_initialized) return;

    // 简易串口接收逻辑，模拟 DMA 效果
    const int available = ServoSerial.available();
    if (g_ExpectedLen > 0 && available >= g_ExpectedLen) {
        const uint8_t expected_len = g_ExpectedLen;
        ServoSerial.readBytes(g_ServoRxBuf, expected_len);
        g_ServoRxFlag = 1;
        g_last_rx_len = expected_len;
        g_ExpectedLen = 0;

        if (g_last_command == CMD_GET_BATTERY_VOLTAGE) {
            g_last_parse_ok = g_ServoRxBuf[0] == 0x55 &&
                              g_ServoRxBuf[1] == 0x55 &&
                              g_ServoRxBuf[2] == 0x04 &&
                              g_ServoRxBuf[3] == CMD_GET_BATTERY_VOLTAGE;
            if (g_last_parse_ok) {
                g_battery_mv = static_cast<uint16_t>(g_ServoRxBuf[4] | (g_ServoRxBuf[5] << 8));
            }
        } else if (g_last_command == CMD_MULT_SERVO_POS_READ) {
            uint8_t id_out[8] = {};
            uint16_t pos_out[8] = {};
            const int8_t count = parseReadResponse(g_ServoRxBuf, id_out, pos_out);
            g_last_parse_ok = count > 0;
            if (count > 0) {
                g_last_read_result.valid = true;
                g_last_read_result.count = static_cast<uint8_t>(count > 8 ? 8 : count);
                for (uint8_t i = 0; i < g_last_read_result.count; ++i) {
                    g_last_read_result.id[i] = id_out[i];
                    g_last_read_result.position[i] = pos_out[i];
                }
            } else {
                g_last_read_result.valid = false;
                g_last_read_result.count = 0;
            }
        } else {
            g_last_parse_ok = false;
        }
    }
}

}  // namespace servo
