#include "I2C_.h"

volatile uint8_t i2c_err = 0;

// 1. Khởi tạo I2C master (MSSP1: RC3 = SCL, RC4 = SDA)
void i2c_master_init(void)
{
    TRISCbits.TRISC3    = 1;                    // SCL
    TRISCbits.TRISC4    = 1;                    // SDA
    SSP1STATbits.SMP    = 0;                    // bật slew rate control (dùng cho 400 kHz)
    SSP1CON1bits.SSPM   = 0b1000;               // master, clock = FOSC / (4 * (SSP1ADD + 1))
    SSP1ADD             = 39;                   // 64MHz / (4 * 40) = 400 kHz
    SSP1CON1bits.SSPEN  = 1;                    // bật MSSP1
}

// 2. Khởi tạo I2C slave
void i2c_slave_init(unsigned char addr)
{
    TRISCbits.TRISC3    = 1;                    // SCL
    TRISCbits.TRISC4    = 1;                    // SDA

    SSP1CON1bits.SSPM   = 0b0110;               // I2C slave 7 bit
    SSP1CON3bits.AHEN   = 0;                    // tắt giữ địa chỉ
    SSP1CON2bits.SEN    = 1;                    // cho phép kéo dãn xung nhịp
    PIE1bits.SSPIE      = 1;                    // bật ngắt I2C
    PIR1bits.SSPIF      = 0;                    // xóa cờ ngắt
    INTCONbits.PEIE     = 1;                    // cho phép ngắt ngoại vi
    SSP1CON1bits.SSPEN  = 1;                    // bật MSSP1
}

// 3. Start
void i2c_start(void)
{
    SSP1CON2bits.SEN = 1;
    i2c_check_timeout();
}

// 4. Stop
void i2c_stop(void)
{
    SSP1CON2bits.PEN = 1;
    i2c_check_timeout();
}

// 5. Gửi 1 byte
void i2c_write(unsigned char data)
{
    SSP1BUF = data;
    i2c_check_timeout();
}

// 6. Đọc 1 byte (AnK = 0: gửi ACK, AnK = 1: gửi NACK)
unsigned char i2c_read(short AnK)
{
    // 1. Bật nhận dữ liệu
    SSP1CON2bits.RCEN = 1;

    // 2. Chờ nhận đủ 8 bit; timeout thì thoát ngay, không phát ACK/NACK
    if (i2c_check_timeout() != 0) {
        return 0;
    }
    // 3. Đọc dữ liệu
    unsigned char data = SSP1BUF;

    // 4. Phát ACK / NACK
    SSP1CON2bits.ACKDT = AnK;
    SSP1CON2bits.ACKEN = 1;

    // 5. Chờ phát xong
    i2c_check_timeout();

    return data;
}

// 7. Restart
void i2c_restart(void)
{
    SSP1CON2bits.RSEN = 1;
    i2c_check_timeout();
}

// 8. Chờ sự kiện I2C hoàn thành, có chống treo.
//    Trả về 0 nếu thành công, 1 nếu timeout (đồng thời set i2c_err).
unsigned char i2c_check_timeout(void)
{
    unsigned int timeout = I2C_TIMEOUT;

    // Chờ cờ SSPIF (byte hoặc điều kiện I2C đã hoàn thành)
    while (!PIR1bits.SSPIF) {
        timeout--;
        if (timeout == 0) {
            // Phát STOP để giải phóng bus
            SSP1CON2bits.PEN = 1;

            unsigned int stop_timeout = I2C_TIMEOUT;
            while (SSP1CON2bits.PEN) {
                if (--stop_timeout == 0) break;
            }

            PIR1bits.SSPIF = 0;         // xóa cờ ngắt
            i2c_err = 1;                // báo lỗi cho tầng trên
            return 1;
        }
    }

    PIR1bits.SSPIF = 0;                 // xóa cờ khi thành công
    return 0;
}