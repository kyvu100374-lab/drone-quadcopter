/*
 * File:   SPI_.h
 * Author: Thi
 *
 * Created on August 25, 2026, 10:51 PM
 */

#ifndef SPI__H
#define SPI__H

#include <xc.h>
#include <stdint.h>

/* ---------- 1. Macro / Constant ---------- */
/* _XTAL_FREQ phải khớp với cấu hình oscillator thực tế (dùng cho __delay_ms).
 * Nếu project đã định nghĩa ở nơi khác thì đoạn này sẽ không định nghĩa lại. */
#ifndef _XTAL_FREQ
#define _XTAL_FREQ      64000000UL
#endif

/* Cấu hình hướng chân (TRIS) cho giao tiếp SPI và điều khiển nRF24L01 */
#define TRIS_SCK    TRISDbits.RD0       // Chân xung clock SPI (SCK)
#define TRIS_SDI    TRISDbits.RD1       // Chân nhận dữ liệu SPI (Master In Slave Out - SDI/MISO)
#define TRIS_SDO    TRISDbits.RD4       // Chân phát dữ liệu SPI (Master Out Slave In - SDO/MOSI)
#define TRIS_CE     TRISCbits.RC5       // Chân Chip Enable của nRF24L01
#define TRIS_CSN    TRISDbits.RD3       // Chân Chip Select Not của nRF24L01
#define TRIS_CSN1   TRISBbits.RB3       // Chân chọn thiết bị SPI phụ thứ hai (nếu có)

/* Alias cũ (tên sai chính tả S-D-Zero thay vì S-D-O) để code cũ vẫn build được không báo lỗi */
#define TRIS_SD0    TRIS_SDO

/* Thanh ghi chốt đầu ra (LAT) để điều khiển mức logic các chân */
#define CE          LATCbits.LATC5
#define CSN         LATDbits.LATD3
#define CSN1        LATBbits.LATB3

/* ---------- 2. Prototype (Khai báo nguyên mẫu hàm) ---------- */
void    SPI_Init(void);                         // Khởi tạo module SPI phần cứng của vi điều khiển
uint8_t SPI_RW(uint8_t data);                   // Gửi và nhận đồng thời 1 byte dữ liệu qua SPI
uint8_t SPI_GetError(void);                     // Trạng thái lỗi: 1 = lần gọi SPI_RW gần nhất bị timeout, 0 = OK
void    SPI_Transfer(const uint8_t *tx, uint8_t *rx, uint8_t len); // Truyền nhận mảng dữ liệu (Buffer transfer) qua SPI

#endif /* SPI__H */