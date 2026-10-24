/*
 * File:   SPI.h
 * Author: congx
 *
 * Created on October 22, 2026, 3:12 PM
 */

#ifndef SPI_H
#define SPI_H

#include <xc.h>
#include <stdint.h>

/* Dinh nghia 1 cho duy nhat cho ca project (dung cho __delay_ms/__delay_us) */
#ifndef _XTAL_FREQ
#define _XTAL_FREQ 16000000UL
#endif

#define CE  PORTCbits.RC2
#define CS  PORTAbits.RA5

void    SPI_Init(void);
uint8_t SPI_RW(uint8_t data);

#endif /* SPI_H */