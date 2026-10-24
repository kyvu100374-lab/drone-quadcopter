/*
 * File:   MPU6050.h
 * Author: congx
 *
 * Created on September 1, 2026, 9:13 PM
 */

#ifndef MPU6050_H
#define	MPU6050_H
#include <xc.h>
#include <stdint.h>
#include "I2C_.h"

// 1. Macro / hằng số
#define W_DATA         0xD0     // địa chỉ I2C (ghi)
#define R_DATA         0xD1     // địa chỉ I2C (đọc)
#define PWR_MGMT_1     0x6B
#define PWR_MGMT_2     0x6C
#define SMPRT_DIV      0x19
#define CONFIG_R       0x1A
#define GYRO_CONFIG    0x1B
#define ACCEL_CONFIG   0x1C
#define ACCEL_XOUT_H   0x3B
#define ACCEL_XOUT_L   0x3C
#define ACCEL_YOUT_H   0x3D
#define ACCEL_YOUT_L   0x3E
#define ACCEL_ZOUT_H   0x3F
#define ACCEL_ZOUT_L   0x40
#define TEMP_OUT_H     0x41
#define TEMP_OUT_L     0x42
#define GYRO_XOUT_H    0x43
#define GYRO_XOUT_L    0x44
#define GYRO_YOUT_H    0x45
#define GYRO_YOUT_L    0x46
#define GYRO_ZOUT_H    0x47
#define GYRO_ZOUT_L    0x48
#define INT_ENABLE     0x38
#define INT_PIN_CFG    0x37
#define WHO_AM_I       0x75

// FIFO
#define FIFO_EN_REG    0x23
#define USER_CTRL      0x6A
#define FIFO_COUNT_H   0x72
#define FIFO_R_W       0x74

#define _XTAL_FREQ  64000000

// 2. Hàm
uint8_t MPU6050_Init(void);                              // 1 = MPU phản hồi I2C, 0 = lỗi
int16_t MPU6050_Read(unsigned int add);
void    MPU6050_Write(unsigned int add, unsigned int data);
uint8_t MPU6050_int(uint8_t *buffer);                    // 1 = đọc đủ 14 byte, 0 = lỗi (buffer giữ nguyên)
uint8_t MPU6050_fifo4(uint8_t *buffer);                  // 1 = đã có mẫu trung bình 4 mẫu 1 kHz, 0 = chưa/lỗi

#endif	/* MPU6050_H */