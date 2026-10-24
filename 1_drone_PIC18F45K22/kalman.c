// kalman.c
#include <math.h>
#include "kalman.h"

// Chỉ học bias khi máy gần đứng yên và gia tốc kế đáng tin
#define BIAS_RATE_MAX   5.0f     // deg/s
#define BIAS_R_MAX      5.0f     // deg^2 (so với R gốc, trước khi bị chặn sàn gain)

// Sàn hệ số kéo của gia tốc kế: dù R_dynamic lớn đến đâu, accel vẫn kéo góc về
// với hằng số thời gian tối đa ~ dt / K_MIN = 0.004 / 0.03 = ~0.13 s.
// Muốn về nhanh hơn thì tăng (0.05 -> ~0.08 s), muốn lọc rung mạnh hơn thì giảm.
#define K_MIN           0.03f

float getKalman(Kalman_t *K, float newAngle, float newRate, float dt, float R_dynamic) {
    // 0. Mẫu đầu tiên: lấy góc đo làm góc khởi tạo
    if (!K->initialized) {
        K->angle = newAngle;
        K->initialized = 1;
        return K->angle;
    }

    // 1. Dự đoán (Predict)
    float rate = newRate - K->bias;
    K->angle += dt * rate;
    K->P[0][0] += dt * (dt * K->P[1][1] - K->P[0][1] - K->P[1][0] + K->Q_angle);
    K->P[0][1] -= dt * K->P[1][1];
    K->P[1][0] -= dt * K->P[1][1];
    K->P[1][1] += K->Q_bias * dt;

    // 2. Cập nhật (Update) với R_dynamic biến thiên
    float R_in = R_dynamic;                          // giữ R gốc để xét điều kiện học bias

    // Chặn R để gain k0 = P00 / (P00 + R) không bao giờ nhỏ hơn K_MIN
    float R_cap = K->P[0][0] * (1.0f / K_MIN - 1.0f);
    if (R_dynamic > R_cap) R_dynamic = R_cap;

    float S = K->P[0][0] + R_dynamic;
    float k_gain[2];
    k_gain[0] = K->P[0][0] / S;
    k_gain[1] = K->P[1][0] / S;

    float y = newAngle - K->angle;
    K->angle += k_gain[0] * y;

    // Chặn nhiễm bias: lúc đang xoay hoặc gia tốc kế kém tin cậy thì không học bias
    if (fabsf(newRate) < BIAS_RATE_MAX && R_in < BIAS_R_MAX)
        K->bias += k_gain[1] * y;

    float P00_temp = K->P[0][0];
    float P01_temp = K->P[0][1];

    K->P[0][0] -= k_gain[0] * P00_temp;
    K->P[0][1] -= k_gain[0] * P01_temp;
    K->P[1][0] -= k_gain[1] * P00_temp;
    K->P[1][1] -= k_gain[1] * P01_temp;

    return K->angle;
}

void initKalman(Kalman_t *K) {
    K->Q_angle   = 0.25f;      // Q mỗi bước = 0.25 * 0.004 = 1e-3 (tăng để P00 không tụt quá thấp)
    K->Q_bias    = 0.0001f;    // gyro đã trừ offset tay, bias chỉ cần chỉnh rất chậm
    K->R_measure = 1.0f;       // std gia tốc kế ~1 deg
    K->angle = 0.0f;
    K->bias  = 0.0f;
    K->P[0][0] = 0.0f; K->P[0][1] = 0.0f;
    K->P[1][0] = 0.0f; K->P[1][1] = 0.0f;
    K->initialized = 0;
}