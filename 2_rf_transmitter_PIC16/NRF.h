/*
 * File:   NRF.h
 * Author: congx
 *
 * Created on October 22, 2026, 3:17 PM
 */

#ifndef NRF_H
#define NRF_H

#include <xc.h>
#include <stdint.h>
#include "SPI.h"

/* --- Lenh nRF24L01 --- */
#define R_REGISTER      0x00
#define W_REGISTER      0x20
#define R_RX_PL_WID     0x60
#define R_RX_PAYLOAD    0x61
#define W_TX_PAYLOAD    0xA0
#define FLUSH_TX        0xE1
#define FLUSH_RX        0xE2
#define NRF_NOP         0xFF

/* --- Thanh ghi --- */
#define REG_CONFIG      0x00
#define REG_EN_AA       0x01
#define REG_EN_RXADDR   0x02
#define REG_SETUP_RETR  0x04
#define REG_RF_CH       0x05
#define REG_RF_SETUP    0x06
#define REG_STATUS      0x07
#define REG_RX_ADDR_P0  0x0A
#define REG_TX_ADDR     0x10
#define REG_DYNPD       0x1C
#define REG_FEATURE     0x1D

/* --- Bit STATUS --- */
#define ST_RX_DR        0x40
#define ST_TX_DS        0x20
#define ST_MAX_RT       0x10
#define ST_MASK         0x70

/* --- Do dai goi du lieu --- */
#define PAY_LOAD_RX     13      /* byte nhan tu tay cam (lenh dieu khien + PID) */
#define PAY_LOAD_FB     11      /* byte phan hoi ve tay cam (telemetry, ACK payload) */

/* Union truy cap theo ten truong hoac mang byte (XC8 khong chen padding) */
typedef union {
    struct {
        uint8_t speed;          /* Ga co ban */
        int16_t roll_ref;       /* Goc dat Roll */
        int16_t pitch_ref;      /* Goc dat Pitch */
        int8_t  Rate_ref;       /* Toc do goc Yaw mong muon */
        uint8_t Turn;           /* bit0 = chay (arm/disarm), bit1 = reset PID/motor */
        uint8_t kp_angle;
        uint8_t ki_angle;
        uint8_t kd_angle;
        uint8_t kp_yaw;
        uint8_t ki_yaw;
        uint8_t kd_yaw;
    };
    uint8_t buffer[PAY_LOAD_RX];    /* 1+2+2+1+1+6 = 13 byte */
} Payload_t;

typedef union {
    struct {
        int16_t roll;
        int16_t pitch;
        int16_t rate_x;
        int16_t rate_y;
        int16_t rate_z;
        uint8_t state;
    };
    uint8_t buffer[PAY_LOAD_FB];    /* 5*2+1 = 11 byte */
} featback;

extern Payload_t TX_data;           /* Goi lenh nhan tu tay cam */
extern featback  RX_data;           /* Goi phan hoi ve tay cam */

/* --- Ham nRF24L01 --- */
void    NRF_WriteReg(uint8_t reg, uint8_t val);
void    NRF_WriteBuf(uint8_t reg, const uint8_t *buf, uint8_t len);
uint8_t NRF_ReadReg(uint8_t reg);
uint8_t NRF_Status(void);
uint8_t NRF_Init(void);                 /* tra 1 neu module phan hoi dung */
void    NRF_Send(const uint8_t *data, uint8_t len);

/* Gui 1 goi, cho ket qua, doc ACK payload vao fb.
 * Tra 1 neu nhan du telemetry, 0 neu mat link / khong co ACK payload. */
uint8_t NRF_Transfer(const uint8_t *tx, uint8_t len, uint8_t *fb, uint8_t fb_len);

#endif /* NRF_H */