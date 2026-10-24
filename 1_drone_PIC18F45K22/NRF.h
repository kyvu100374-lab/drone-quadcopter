/*
 * File:   NRF.h
 * Author: congx
 *
 * Created on September 1, 2026, 9:53 PM
 */

#ifndef NRF_H
#define	NRF_H
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "SPI_.h"

// 1. Macro / hằng số
// --- Lệnh SPI cho nRF24L01 ---
#define NRF_CMD_R_REG        0x00  // Đọc thanh ghi (OR với địa chỉ)
#define NRF_CMD_W_REG        0x20  // Ghi thanh ghi (OR với địa chỉ)
#define NRF_CMD_R_PAYLOAD    0x61  // Đọc dữ liệu RX payload từ FIFO
#define NRF_CMD_W_PAYLOAD    0xA0  // Ghi dữ liệu TX payload vào FIFO
#define NRF_CMD_W_ACK_PL     0xA8  // Nạp ACK payload cho pipe 0 (phản hồi về tay cầm)
#define NRF_CMD_FLUSH_TX     0xE1  // Xóa bộ đệm FIFO phát
#define NRF_CMD_FLUSH_RX     0xE2  // Xóa bộ đệm FIFO nhận
#define NRF_CMD_NOP          0xFF  // Lệnh trống (dùng để đọc về thanh ghi STATUS)

// --- Địa chỉ các thanh ghi của nRF24L01 ---
#define NRF_REG_CONFIG       0x00
#define NRF_REG_EN_AA        0x01
#define NRF_REG_EN_RXADDR    0x02
#define NRF_REG_SETUP_AW     0x03
#define NRF_REG_RF_CH        0x05
#define NRF_REG_RF_SETUP     0x06
#define NRF_REG_STATUS       0x07
#define NRF_REG_RX_ADDR_P0   0x0A
#define NRF_REG_RX_PW_P0     0x11
#define NRF_REG_DYNPD        0x1C
#define NRF_REG_FEATURE      0x1D

// Bit 3:1 của thanh ghi STATUS là RX_P_NO, bằng 0b111 (0x0E) nghĩa là bộ đệm RX FIFO đang rỗng
#define NRF_STATUS_RX_P_NO   0x0E

// --- Độ dài gói dữ liệu (Payload) ---
#define PAY_LOAD_RX          13    // Số byte nhận từ tay cầm (lệnh điều khiển + PID)
#define PAY_LOAD_FB          11    // Số byte phản hồi về tay cầm (Telemetry qua ACK payload)

// 2. Cấu trúc dữ liệu (Union giúp truy cập linh hoạt theo tên trường hoặc mảng byte truyền qua SPI)
typedef union {
    struct {
        uint8_t speed;          // Giá trị ga cơ bản
        int16_t roll_ref;       // Góc đặt Roll mong muốn
        int16_t pitch_ref;      // Góc đặt Pitch mong muốn
        int8_t  Rate_ref;       // Tốc độ góc Yaw mong muốn
        uint8_t Turn;           // Byte trạng thái điều khiển: bit0 = chạy (arm/disarm), bit1 = reset PID/motor
        uint8_t kp_angle;       // Hệ số P của góc
        uint8_t ki_angle;       // Hệ số I của góc
        uint8_t kd_angle;       // Hệ số D của góc
        uint8_t kp_yaw;         // Hệ số P của Yaw
        uint8_t ki_yaw;         // Hệ số I của Yaw
        uint8_t kd_yaw;         // Hệ số D của Yaw
    };
    uint8_t buffer[PAY_LOAD_RX];    // Tổng cộng: 1 + 2 + 2 + 1 + 1 + 6 = 13 byte (trình biên dịch XC8 không chèn padding)
} Payload_t;

typedef union {
    struct {
        int16_t roll;           // Góc Roll thực tế đo được
        int16_t pitch;          // Góc Pitch thực tế đo được
        int16_t rate_x;         // Tốc độ góc trục X (Roll rate)
        int16_t rate_y;         // Tốc độ góc trục Y (Pitch rate)
        int16_t rate_z;         // Tốc độ góc trục Z (Yaw rate)
        uint8_t state;          // Trạng thái hệ thống gửi về tay cầm
    };
    uint8_t buffer[PAY_LOAD_FB];    // Tổng cộng: 5 * 2 + 1 = 11 byte
} featback;

// 3. Biến toàn cục: CHỈ khai báo extern ở đây, tránh lỗi multiple definition khi nhiều file .c include header này
extern Payload_t TX_data;               // Gói dữ liệu nhận từ tay cầm
extern featback  RX_data;               // Gói dữ liệu phản hồi về tay cầm (Telemetry)
extern volatile bool flag;              // Cờ yêu cầu reset PID / motor
extern volatile bool flag_run;          // Cờ cho phép quadcopter hoạt động (running state)
extern volatile bool last_flag_run;     // Lưu trạng thái flag_run của vòng lặp trước
extern volatile bool failsafe_latch;    // Cờ khóa khẩn cấp khi mất sóng hoặc lỗi, chờ tay cầm hạ cờ chạy rồi mới cho phép arm lại

// 4. Khai báo nguyên mẫu hàm
void NRF_WriteReg(unsigned char reg, unsigned char val);
void NRF_Init(void);
bool NRF_INT(void);         // Gọi trong hàm Low_ISR; trả về true nếu thực sự nhận được gói dữ liệu mới hợp lệ
void NRF_LoadAck(void);     // Nạp dữ liệu RX_data vào ACK payload (gọi từ hàm main, tự động chặn ngắt ưu tiên thấp)

#endif	/* NRF_H */