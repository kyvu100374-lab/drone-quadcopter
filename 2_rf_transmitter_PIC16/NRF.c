#include "NRF.h"

void NRF_WriteReg(uint8_t reg, uint8_t val)
{
    CS = 0; SPI_RW(W_REGISTER | reg); SPI_RW(val); CS = 1;
}

void NRF_WriteBuf(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    uint8_t i;
    CS = 0; SPI_RW(W_REGISTER | reg);
    for (i = 0; i < len; i++) SPI_RW(buf[i]);
    CS = 1;
}

uint8_t NRF_ReadReg(uint8_t reg)
{
    uint8_t val;
    CS = 0; SPI_RW(R_REGISTER | reg); val = SPI_RW(NRF_NOP); CS = 1;
    return val;
}

/* Byte dau tien nRF tra ve luon la STATUS, nen chi can 1 byte NOP */
uint8_t NRF_Status(void)
{
    uint8_t s;
    CS = 0; s = SPI_RW(NRF_NOP); CS = 1;
    return s;
}

uint8_t NRF_Init(void)
{
    static const uint8_t addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

    __delay_ms(100);

    NRF_WriteBuf(REG_TX_ADDR, addr, 5);
    NRF_WriteBuf(REG_RX_ADDR_P0, addr, 5);   /* de nhan ACK */
    NRF_WriteReg(REG_EN_AA, 0x01);
    NRF_WriteReg(REG_EN_RXADDR, 0x01);
    NRF_WriteReg(REG_SETUP_RETR, 0x1A);      /* ARD=500us, ARC=10 */
    NRF_WriteReg(REG_DYNPD, 0x01);           /* DPL pipe 0 */
    NRF_WriteReg(REG_FEATURE, 0x06);         /* EN_DPL | EN_ACK_PAY */
    NRF_WriteReg(REG_RF_CH, 0x40);
    NRF_WriteReg(REG_RF_SETUP, 0x0F);        /* 2 Mbps, 0 dBm */
    NRF_WriteReg(REG_CONFIG, 0x0E);          /* PWR_UP, CRC 2 byte, TX mode */
    __delay_ms(5);

    /* Doc lai de kiem tra module co mat va day SPI dung */
    return (NRF_ReadReg(REG_RF_CH) == 0x40) && (NRF_ReadReg(REG_CONFIG) == 0x0E);
}

void NRF_Send(const uint8_t *data, uint8_t len)
{
    uint8_t i;
    CE = 0;
    NRF_WriteReg(REG_STATUS, ST_MASK);       /* xoa co */
    CS = 0; SPI_RW(W_TX_PAYLOAD);
    for (i = 0; i < len; i++) SPI_RW(data[i]);
    CS = 1;
    CE = 1; __delay_us(15); CE = 0;
}

uint8_t NRF_Transfer(const uint8_t *tx, uint8_t len, uint8_t *fb, uint8_t fb_len)
{
    uint16_t timeout = 500;
    uint8_t  status = 0, got = 0, w, i, b;

    NRF_Send(tx, len);

    /* Cho gui xong (TX_DS) hoac that bai (MAX_RT), co timeout chong treo */
    while (timeout--) {
        status = NRF_Status();
        if (status & ST_MASK) break;
        __delay_us(10);
    }

    /* Doc ACK payload: lay do rong thuc te truoc, khong doc qua bo dem fb */
    if (status & ST_RX_DR) {
        CS = 0; SPI_RW(R_RX_PL_WID); w = SPI_RW(NRF_NOP); CS = 1;
        if (w > 32) {
            CS = 0; SPI_RW(FLUSH_RX); CS = 1;        /* payload loi */
        } else if (w > 0) {
            CS = 0; SPI_RW(R_RX_PAYLOAD);
            for (i = 0; i < w; i++) {
                b = SPI_RW(NRF_NOP);
                if (i < fb_len) fb[i] = b;
            }
            CS = 1;
            got = (w >= fb_len);
        }
    }

    /* Mat link hoac timeout: bo goi cu con ket trong TX FIFO */
    if (!(status & ST_TX_DS)) {
        CS = 0; SPI_RW(FLUSH_TX); CS = 1;
    }

    NRF_WriteReg(REG_STATUS, ST_MASK);               /* xoa moi co */
    return got;
}