
#include <xc.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include "SPI.h"
#include "NRF.h"

// --- Cấu hình ---
#pragma config FOSC = HS, WDTE = OFF, PWRTE = OFF, CP = OFF, BOREN = OFF

Payload_t TX_data;                  /* Goi lenh nhan tu may tinh (ISR ghi) */
featback  RX_data;                  /* Goi telemetry tu drone */

static uint8_t rx_buffer[PAY_LOAD_RX + 2];
static volatile uint8_t rx_index = 0;
static volatile uint8_t flag_nhan_xong = 0;

/* ---------- UART ---------- */
static void UART_Init(void)
{
    TRISC7 = 1; TRISC6 = 0;
    SPBRG = 103;                    /* 9600 baud, 16 MHz, BRGH = 1 */
    TXSTA = 0x24;
    RCSTA = 0x90;
}

static void Uart_Send_Char(uint8_t data)
{
    while (!PIR1bits.TXIF);
    TXREG = data;
}

static void Uart_Send_Bytes(const uint8_t *data, uint8_t len)
{
    uint8_t i;
    for (i = 0; i < len; i++) Uart_Send_Char(data[i]);
}

/* Nhan frame: 0xCC 0xDD + PAY_LOAD_RX byte. Goi trong ISR. */
static void UART_Read_Binary_NonBlocking(void)
{
    uint8_t in_byte, i;

    if (RCSTAbits.OERR) {
        RCSTAbits.CREN = 0;
        RCSTAbits.CREN = 1;
        rx_index = 0;               /* bo frame do dang */
        return;
    }

    in_byte = RCREG;
    if (flag_nhan_xong) return;     /* main chua lay goi cu -> bo byte nay */

    rx_buffer[rx_index++] = in_byte;

    if (rx_index >= PAY_LOAD_RX + 2) {
        if (rx_buffer[0] == 0xCC && rx_buffer[1] == 0xDD) {
            for (i = 0; i < PAY_LOAD_RX; i++) TX_data.buffer[i] = rx_buffer[i + 2];
            flag_nhan_xong = 1;
            rx_index = 0;
        } else {
            /* Sai header: truot 1 byte de tim lai diem dong bo */
            for (i = 0; i < rx_index - 1; i++) rx_buffer[i] = rx_buffer[i + 1];
            rx_index--;
        }
    }
}

void __interrupt() my_isr0(void)
{
    if (RCIF) UART_Read_Binary_NonBlocking();
}

/* ---------- MAIN ---------- */
void main(void)
{
    uint8_t i;
    Payload_t cmd;

    ADCON1 = 0x06;                  /* PORTA digital */
    UART_Init();
    SPI_Init();

    if (!NRF_Init()) {              /* khong thay nRF: bao loi 0xEE len may tinh */
        while (1) {
            Uart_Send_Char(0xEE);
            __delay_ms(500);
        }
    }

    RCIE = 1; PEIE = 1; GIE = 1;

    while (1) {
        if (flag_nhan_xong) {
            for (i = 0; i < PAY_LOAD_RX; i++) cmd.buffer[i] = TX_data.buffer[i];
            flag_nhan_xong = 0;     /* mo khoa ngay: ISR nhan frame ke tiep trong luc main gui nRF */

            if (!NRF_Transfer(cmd.buffer, PAY_LOAD_RX, RX_data.buffer, PAY_LOAD_FB)) {
                for (i = 0; i < PAY_LOAD_FB; i++) RX_data.buffer[i] = 0;   /* khong gui telemetry cu */
            }

            if (NRF_Transfer(cmd.buffer, PAY_LOAD_RX, RX_data.buffer, PAY_LOAD_FB)) {
                Uart_Send_Char(0xAA);
                Uart_Send_Char(0xBB);
                Uart_Send_Bytes(RX_data.buffer, PAY_LOAD_FB);
            }
        }
    }
}