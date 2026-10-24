#include "MPU6050.h"

// 1. Khởi tạo. Trả về 1 nếu MPU phản hồi I2C trong suốt quá trình cấu hình, 0 nếu có lỗi.
uint8_t MPU6050_Init(void)
{
    i2c_err = 0;

    MPU6050_Write(PWR_MGMT_1, 0x80);     // Reset thiết bị
    __delay_ms(50);

    MPU6050_Write(PWR_MGMT_1, 0x03);     // Chọn clock gyro Z
    __delay_ms(10);

    MPU6050_Write(CONFIG_R, 0x03);       // DLPF 42 Hz (Fs = 1kHz)
    __delay_ms(10);

    MPU6050_Write(SMPRT_DIV, 0x00);      // 1000 Hz: chip lấy mẫu 1 kHz, MCU trung bình 4 mẫu -> 250 Hz
    __delay_ms(10);

    MPU6050_Write(GYRO_CONFIG, 0x08);    // FS_SEL = 1 (+-500 deg/s)
    __delay_ms(10);
    MPU6050_Write(ACCEL_CONFIG, 0x10);   // AFS_SEL = 2 (+-8g)
    __delay_ms(10);

    // --- FIFO: accel XYZ + gyro XYZ = 12 byte/mẫu ---
    MPU6050_Write(FIFO_EN_REG, 0x78);
    __delay_ms(10);
    MPU6050_Write(USER_CTRL, 0x44);      // FIFO_EN + FIFO_RESET
    __delay_ms(10);

    // --- Cấu hình ngắt INT ---
    MPU6050_Write(INT_ENABLE, 0x01);     // Bật ngắt DATA_RDY_EN (bắn 1 kHz, ISR chỉ đếm)
    __delay_ms(10);
    MPU6050_Write(INT_PIN_CFG, 0x90);    // Active-low, push-pull, xung 50us, xóa khi đọc bất kỳ
    __delay_ms(10);

    if (i2c_err) return 0;
    return ((uint8_t)MPU6050_Read(WHO_AM_I) == 0x68) ? 1 : 0;
}

// 2. Ghi 1 thanh ghi
void MPU6050_Write(unsigned int add, unsigned int data)
{
    i2c_start();
    i2c_write(W_DATA);
    i2c_write(add);
    i2c_write(data);
    i2c_stop();
}

// 3. Đọc 1 thanh ghi
int16_t MPU6050_Read(unsigned int add)
{
    int16_t retval;
    i2c_start();
    i2c_write(W_DATA);
    i2c_write(add);
    i2c_restart();
    i2c_write(R_DATA);
    retval = i2c_read(1);                // byte cuối cùng phải gửi NACK
    i2c_stop();
    return retval;
}

// 4. Bản đọc 14 byte cũ (giữ lại để quay về nếu cần)
uint8_t MPU6050_int(uint8_t *buffer)
{
    uint8_t tmp[14];

    i2c_err = 0;

    i2c_start();
    i2c_write(W_DATA);
    i2c_write(ACCEL_XOUT_H);
    i2c_restart();
    i2c_write(R_DATA);
    if (i2c_err) { i2c_stop(); return 0; }

    for (uint8_t i = 0; i < 14; i++) {
        tmp[i] = i2c_read(i == 13 ? 1 : 0);
        if (i2c_err) { i2c_stop(); return 0; }
    }
    i2c_stop();
    if (i2c_err) return 0;

    for (uint8_t i = 0; i < 14; i++) buffer[i] = tmp[i];
    return 1;
}

// 5. Đọc 4 mẫu cũ nhất trong FIFO (1 kHz), lấy trung bình -> 1 mẫu 250 Hz.
//    buffer[14] cùng layout MPU6050_int: accel ở 0-5, gyro ở 8-13 (6-7 nhiệt độ, không dùng).
uint8_t MPU6050_fifo4(uint8_t *buffer)
{
    uint8_t tmp[48];
    int32_t sum[6] = {0, 0, 0, 0, 0, 0};

    i2c_err = 0;

    // Số byte đang có trong FIFO
    i2c_start();
    i2c_write(W_DATA);
    i2c_write(FIFO_COUNT_H);
    i2c_restart();
    i2c_write(R_DATA);
    uint8_t ch = i2c_read(0);
    uint8_t cl = i2c_read(1);
    i2c_stop();
    if (i2c_err) return 0;

    uint16_t cnt = ((uint16_t)ch << 8) | cl;
    uint16_t ns  = cnt / 12;             // số mẫu nguyên
    if (ns < 4) return 0;                // chưa đủ 4 mẫu: bỏ lần này, KHÔNG reset
    if (ns > 8) {                        // đầy bất thường mới reset
        MPU6050_Write(USER_CTRL, 0x44);
        return 0;
    }

    // Đọc 48 byte từ FIFO_R_W
    i2c_start();
    i2c_write(W_DATA);
    i2c_write(FIFO_R_W);
    i2c_restart();
    i2c_write(R_DATA);
    if (i2c_err) { i2c_stop(); return 0; }
    for (uint8_t i = 0; i < 48; i++) {
        tmp[i] = i2c_read(i == 47 ? 1 : 0);
        if (i2c_err) { i2c_stop(); return 0; }
    }
    i2c_stop();
    if (i2c_err) return 0;

    for (uint8_t s = 0; s < 4; s++)
        for (uint8_t k = 0; k < 6; k++)
            sum[k] += (int16_t)(((uint16_t)tmp[s * 12 + k * 2] << 8) | tmp[s * 12 + k * 2 + 1]);

    for (uint8_t k = 0; k < 6; k++) {
        int16_t a = (int16_t)(sum[k] / 4);
        uint8_t pos = (k < 3) ? (uint8_t)(k * 2) : (uint8_t)(8 + (k - 3) * 2);
        buffer[pos]     = (uint8_t)((uint16_t)a >> 8);
        buffer[pos + 1] = (uint8_t)a;
    }
    return 1;
}