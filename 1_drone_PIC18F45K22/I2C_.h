/*
 * File:   I2C_.h
 * Author: Thi
 *
 * Created on August 25, 2026, 9:37 PM
 */

#ifndef I2C__H
#define	I2C__H

#include <xc.h>
#include <stdint.h>

// 1. Macro / hằng số
#define I2C_TIMEOUT 500    // số vòng chờ SSPIF tối đa cho mỗi thao tác
#define _XTAL_FREQ  64000000
#define FOSC        _XTAL_FREQ

// 2. Cờ lỗi: được set khi một thao tác I2C bị timeout, người gọi tự xóa trước khi dùng
extern volatile uint8_t i2c_err;

// 3. Prototype
unsigned char i2c_check_timeout(void);
void i2c_master_init(void);
void i2c_slave_init(unsigned char addr);
void i2c_write(unsigned char data);
unsigned char i2c_read(short AnK);
void i2c_start(void);
void i2c_stop(void);
void i2c_restart(void);

#endif	/* I2C__H */