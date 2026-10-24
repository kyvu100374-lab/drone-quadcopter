#include "SPI.h"

void SPI_Init(void)
{
    /* Dat muc an toan truoc khi chuyen sang output (tranh glitch CS) */
    CS = 1;
    CE = 0;

    TRISC3 = 0;     /* SCK */
    TRISC4 = 1;     /* SDI */
    TRISC5 = 0;     /* SDO */
    TRISC2 = 0;     /* CE  */
    TRISA5 = 0;     /* CS  */

    SSPSTAT = 0x40; /* CKE=1, SMP=0 */
    SSPCON  = 0x21; /* SSPEN=1, CKP=0, Master Fosc/16 -> SPI mode 0, 1 MHz */
}

uint8_t SPI_RW(uint8_t data)
{
    uint16_t timeout = 5000;

    SSPCONbits.WCOL = 0;
    SSPBUF = data;
    while (!SSPSTATbits.BF) {
        if (--timeout == 0) return 0xFF;    /* chong treo */
    }
    return SSPBUF;                          /* doc buffer se xoa BF */
}