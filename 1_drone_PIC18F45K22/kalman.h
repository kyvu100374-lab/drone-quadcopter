// kalman.h
#ifndef KALMAN_H
#define KALMAN_H

#include <xc.h>
#include <stdint.h>

typedef struct {
    float Q_angle;     // phương sai quá trình của góc (deg^2/s)
    float Q_bias;      // phương sai quá trình của bias gyro
    float R_measure;   // R cơ sở của gia tốc kế (deg^2), lúc đứng yên
    float angle;
    float bias;
    float P[2][2];
    uint8_t initialized;   // 0 = chưa có mẫu đầu, lần gọi đầu lấy góc đo làm góc khởi tạo
} Kalman_t;

void initKalman(Kalman_t *K);

// R_dynamic: phương sai đo thích nghi, truyền vào mỗi lần cập nhật
float getKalman(Kalman_t *K, float newAngle, float newRate, float dt, float R_dynamic);

#endif