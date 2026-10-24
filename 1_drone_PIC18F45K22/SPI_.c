#include "SPI_.h"

#define SPI_TIMEOUT_LOOPS   500   /* chinh theo toc do chip */

static volatile uint8_t spi_error = 0;

void SPI_Init(void)
{
    /* 1 - Tat ADC tren cac chan dung cho SPI/CE/CSN */
    ANSELD = 0;
    ANSELC = 0;

    /* 2 - Dat muc an toan TRUOC khi chuyen sang output (tranh glitch CSN) */
    CSN1 = 1;
    CSN  = 1;       /* khong giao tiep */
    CE   = 0;

    /* 3 - Cau hinh chan */
    TRIS_SCK  = 0;
    TRIS_SDI  = 1;
    TRIS_SDO  = 0;
    TRIS_CE   = 0;
    TRIS_CSN  = 0;
    TRIS_CSN1 = 0;

    /* 4 - Thanh ghi MSSP2: dat ro rang, khong dua vao gia tri reset */
    SSP2CON1 = 0;
    SSP2STAT = 0;
    SSP2STATbits.SMP   = 0;         /* lay mau o giua */
    SSP2STATbits.CKE   = 1;         /* chuyen du lieu khi SCK xuong */
    SSP2CON1bits.CKP   = 0;         /* SCK idle = thap -> SPI mode 0 */
    SSP2CON1bits.SSPM  = 0b0001;    /* Master, Fosc/16 */
    SSP2CON1bits.SSPEN = 1;         /* bat MSSP2 */

    spi_error = 0;
}

uint8_t SPI_RW(uint8_t data)
{
    uint16_t timeout = SPI_TIMEOUT_LOOPS;

    SSP2CON1bits.WCOL = 0;          /* xoa co xung dot ghi neu co */
    (void)SSP2BUF;                  /* doc de xoa BF cu (neu con sot) */

    SSP2BUF = data;

    /* Cho BF duoc bat len hoac het thoi gian */
    while (!SSP2STATbits.BF) {
        if (--timeout == 0) {
            spi_error = 1;
            /* Neu can reset MSSP2:
             * SSP2CON1bits.SSPEN = 0; SSP2CON1bits.SSPEN = 1; */
            return 0xFF;
        }
    }

    spi_error = 0;
    return SSP2BUF;                 /* doc buffer se xoa BF */
}

uint8_t SPI_GetError(void)
{
    return spi_error;
}

/* Truyen nhieu byte. tx hoac rx co the la NULL.
 * Neu tx = NULL se gui 0xFF (NOP cua nRF24L01).
 * Tu dieu khien CSN/CSN1 ben ngoai ham nay (CSN = 0 ... CSN = 1). */
void SPI_Transfer(const uint8_t *tx, uint8_t *rx, uint8_t len)
{
    uint8_t i;
    uint8_t out, in;

    for (i = 0; i < len; i++) {
        out = (tx != 0) ? tx[i] : 0xFF;
        in  = SPI_RW(out);
        if (rx != 0) {
            rx[i] = in;
        }
    }
}