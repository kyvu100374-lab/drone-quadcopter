/*
 * File:   receptora.c
 * Author: Thi
 * Optimized for PIC18F45K22 @ 64MHz (Internal OSC + 4x PLL)
 *
 * Ban sua: telemetry co state that (run / latch / nguyen nhan ngat / nguyen nhan reset)
 */

#include <xc.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <stdbool.h>

#include "I2C_.h"
#include "SPI_.h"
#include "MPU6050.h"
#include "NRF.h"
#include "kalman.h"

// --- CONFIGURATION BITS ---
#pragma config FOSC = INTIO67       // Internal oscillator
#pragma config PLLCFG = ON          // 4X PLL Enabled (16MHz x 4 = 64MHz)
#pragma config PRICLKEN = ON
#pragma config FCMEN = OFF
#pragma config IESO = OFF

#pragma config PWRTEN = OFF
#pragma config BOREN = SBORDIS
#pragma config BORV = 285           // Reset khi Vdd < 2.85V
#pragma config WDTEN = ON           // Watchdog luôn bật
#pragma config WDTPS = 64           // 1:64 (~256ms)

#pragma config CCP2MX = PORTC1
#pragma config PBADEN = OFF         // PORTB<5:0> là Digital I/O
#pragma config CCP3MX = PORTB5
#pragma config HFOFST = ON
#pragma config T3CMX = PORTC0
#pragma config P2BMX = PORTD2
#pragma config MCLRE = INTMCLR

#pragma config STVREN = ON
#pragma config LVP = OFF
#pragma config XINST = OFF

#pragma config CP0 = OFF, CP1 = OFF, CP2 = OFF, CP3 = OFF
#pragma config CPB = OFF, CPD = OFF
#pragma config WRT0 = OFF, WRT1 = OFF, WRT2 = OFF, WRT3 = OFF
#pragma config WRTC = OFF, WRTB = OFF, WRTD = OFF
#pragma config EBTR0 = OFF, EBTR1 = OFF, EBTR2 = OFF, EBTR3 = OFF
#pragma config EBTRB = OFF

// --- DEFINES ---
#define _XTAL_FREQ 64000000

#define ACC_LPF_ALPHA   0.25f        // 1.0 = TẮT, chỉ dựa DLPF phần cứng
#define ACC_REF_ALPHA   0.2f        // mốc nhanh (~8 Hz) chỉ để đo rung
#define VIB_ALPHA       0.2f        // decay của rung (~16 ms), attack là tức thì
#define R_ACC_SCALE     10.0f       // đổi g^2 -> deg^2
#define R_MAX           5.0f       // trần R (deg^2)


#define DT_SEC          0.004f      // 250 Hz theo ngắt MPU
#define I_LIMIT         600.0f      // giới hạn tích phân (đơn vị sai số x giây)
#define PID_OUT_MAX     800.0f      // giới hạn đầu ra PID mỗi trục

// Xung motor tính theo tick 0.5us: 2000 = 1000us, 4000 = 2000us
#define MOTOR_OFF       2000        // mức tắt (đổi ở đây nếu ESC cần mức khác, vd 1800)
#define MOTOR_MIN       2200        // idle khi đang chạy
#define MOTOR_MAX       3900
#define THROTTLE_MAX    3600        // chừa headroom cho PID ở ga cao

// Timeout đếm bằng ngắt Timer3 (400 Hz = 2.5ms/tick), độc lập với ngắt MPU
#define RADIO_TIMEOUT_TICKS  200    // 200 x 2.5ms = 500ms mất sóng
#define MPU_TIMEOUT_TICKS    20     // 20 x 2.5ms  = 50ms không có mẫu MPU hợp lệ
#define MAX_TILT_DEG    60.0f

// Hệ số nhân khi gửi telemetry về tay cầm (int16: tối đa +-32767).
// Góc +-60 độ nhân tối đa 100; rate +-500 deg/s nhân tối đa 10 (nhân 100 sẽ tràn).
#define FB_SCALE_ANGLE  100.0f
#define FB_SCALE_RATE   10.0f

#define RAD2DEG         57.29578f

// Nguyên nhân ngắt khẩn cấp (gửi về PC trong state bit 2-3)
#define CAUSE_NONE      0
#define CAUSE_RADIO     1           // mất sóng NRF
#define CAUSE_MPU       2           // MPU ngừng gửi mẫu
#define CAUSE_TILT      3           // nghiêng quá MAX_TILT_DEG

// Nguyên nhân reset (gửi về PC trong state bit 4-5)
#define RST_NONE        0
#define RST_WDT         1           // watchdog reset
#define RST_BOR         2           // brown-out reset

///////////// PIN PWM (TRIS)
#define PIN0    TRISCbits.RC1
#define PIN1    TRISBbits.RB5
#define PIN3    TRISCbits.RC2
#define PIN2    TRISBbits.RB4

///////////// OUTPUT PWM (LAT)
#define pw0     LATCbits.LATC1
#define pw1     LATBbits.LATB5
#define pw3     LATCbits.LATC2
#define pw2     LATBbits.LATB4

// OUTPUT LED
#define Red     LATAbits.LA3
#define Green   LATAbits.LA1
#define Blue    LATAbits.LA2

// --- KHAI BÁO HÀM ---
void Timer3_init(void);
void ccp_init(void);
void OSC_INIT(void);
void INT_INIT(void);
void GPIO_INIT(void);

void CCP_INT(void);
void TIMER3_INT(void);
void INT_1(void);

void loc_goc(float dt_sec);
void pid(float dt_sec);

// --- BIẾN ---
Kalman_t kalmanRoll;
Kalman_t kalmanPitch;
// góc và tốc độ xoay
float roll, pitch, yaw;
float roll_rate, pitch_rate;
// dữ liệu MPU (ISR ghi, main đọc)
volatile uint8_t mpu_data[14];
volatile float gx = 0.0f, gy = 0.0f, gz = 0.0f;
volatile float axcc = 0.0f, aycc = 0.0f, azcc = 0.0f;
// hiệu chỉnh
float gxs = -0.62f, gys = -0.92f, gzs = 0.26f;
float roll_fix = 0.0f, pitch_fix = 0.0f;
// công suất động cơ
uint16_t value;
// Double buffer: ISR Timer3 đọc bộ motor_active, main ghi bộ còn lại rồi mới đổi chỉ số
volatile uint16_t motor_buf[2][4] = {
    {MOTOR_OFF, MOTOR_OFF, MOTOR_OFF, MOTOR_OFF},
    {MOTOR_OFF, MOTOR_OFF, MOTOR_OFF, MOTOR_OFF}
};
volatile uint8_t motor_active = 0;
// đầu ra PID
float roll_output, pitch_output, yaw_output;
// thành phần PID
float Eroll = 0.0f, Epitch = 0.0f, Eyaw = 0.0f;
float sum_Eroll = 0.0f, sum_Epitch = 0.0f, sum_Eyaw = 0.0f;
float last_Eyaw = 0.0f;
//--//
uint16_t count = 60536;               // Reload Timer3 (400Hz)
// Hai bộ đếm timeout: Timer3 ISR tăng (bão hòa 255), ISR nhận dữ liệu xóa về 0
volatile uint8_t failsafe_count = 0;    // số tick kể từ gói NRF hợp lệ cuối
volatile uint8_t mpu_timeout    = 0;    // số tick kể từ mẫu MPU hợp lệ cuối
volatile bool pid_flag = false;
volatile bool flag_nrf = false;
// Chẩn đoán: gửi về PC qua telemetry
volatile uint8_t fs_cause  = CAUSE_NONE;    // nguyên nhân ngắt khẩn cấp gần nhất
uint8_t          rst_cause = RST_NONE;      // nguyên nhân reset lần khởi động này

volatile uint8_t overrun = 0;

// --- ISR ---
void __interrupt(high_priority) High_ISR(void)  {
    TIMER3_INT();
    CCP_INT();
}

void __interrupt(low_priority) Low_ISR(void) {
    if(INTCON3bits.INT2IF) {
        INTCON3bits.INT2IF = 0;         // xóa trước để không mất cạnh xuống mới
        if(NRF_INT()) {                 // chỉ khi thật sự có gói mới
            failsafe_count = 0;
            flag_nrf = true;
        }
    }
    INT_1();
}

// --- TIỆN ÍCH ---
// Ghi 4 motor không cần chặn ngắt: ghi vào bộ đệm ISR không dùng, rồi đổi chỉ số (1 byte, atomic).
// Chỉ được gọi từ main (một nơi ghi duy nhất).
static void set_motors(uint16_t a, uint16_t b, uint16_t c, uint16_t d) {
    uint8_t n = motor_active ^ 1;
    motor_buf[n][0] = a; motor_buf[n][1] = b;
    motor_buf[n][2] = c; motor_buf[n][3] = d;
    motor_active = n;
}

static float clampf(float v, float lo, float hi) {
    if(v > hi) return hi;
    if(v < lo) return lo;
    return v;
}

// Chép lệnh từ tay cầm ra bản sao, tránh đọc dở lúc ISR đang ghi gói mới
static void get_cmd(Payload_t *c) {
    const volatile uint8_t *src = TX_data.buffer;
    unsigned char pe = INTCONbits.PEIE;
    INTCONbits.PEIE = 0;                // chặn ngắt ưu tiên thấp (NRF/MPU)
    for(uint8_t i = 0; i < PAY_LOAD_RX; i++) c->buffer[i] = src[i];
    INTCONbits.PEIE = pe;
}

static void reset_pid(void) {
    Eroll = 0.0f; Epitch = 0.0f; Eyaw = 0.0f;
    sum_Eroll = 0.0f; sum_Epitch = 0.0f; sum_Eyaw = 0.0f;
    last_Eyaw = 0.0f;
}

// Ngắt khẩn cấp: cắt motor và khóa (latch) cho tới khi tay cầm hạ cờ chạy rồi bật lại
static void trip_failsafe(uint8_t cause) {
    fs_cause = cause;                   // ghi nguyên nhân để gửi về PC
    failsafe_latch = true;              // đặt latch TRƯỚC khi hạ flag_run (tránh ISR bật lại)
    flag_run = false;
    set_motors(MOTOR_OFF, MOTOR_OFF, MOTOR_OFF, MOTOR_OFF);
    reset_pid();
}

// --- MAIN ---
void main(void) {
    // Đọc nguyên nhân reset TRƯỚC mọi thứ khác (CLRWDT sẽ set lại bit TO)
    if(!RCONbits.NOT_TO)                                rst_cause = RST_WDT;
    else if(!RCONbits.NOT_BOR && RCONbits.NOT_POR)      rst_cause = RST_BOR;
    RCONbits.NOT_POR = 1;
    RCONbits.NOT_BOR = 1;

    OSC_INIT();
    CLRWDT();
    GPIO_INIT();
    INT_INIT();                         // bật IPEN và cấu hình ưu tiên ngắt

    Red = 1; Green = 0; Blue = 0;       // đang khởi tạo

    initKalman(&kalmanRoll);
    initKalman(&kalmanPitch);

    i2c_master_init();
    if(!MPU6050_Init()) {              // MPU không phản hồi I2C
        Red = 1; Green = 0; Blue = 0;
        while(1);                       // không CLRWDT: watchdog reset rồi thử khởi tạo lại
    }
    CLRWDT();
    SPI_Init();
    NRF_Init();
    CLRWDT();
    ccp_init();
    Timer3_init();
    for(uint8_t i = 0; i < 20; i++) { __delay_ms(50); CLRWDT(); }

    // Bật ngắt toàn cục sau khi khởi tạo xong ngoại vi
    INTCONbits.PEIE = 1;                // GIEL (IPEN = 1)
    INTCONbits.GIE  = 1;                // GIEH

    Red = 0; Green = 1; Blue = 0;       // vào main thành công

    while(1) {
        CLRWDT();

        // I. Phản hồi qua ACK payload
        if(flag_nrf) {
            flag_nrf = false;           // xóa trước: gói mới đến lúc đang nạp sẽ không bị mất cờ
            RX_data.roll   = (int16_t)(roll * FB_SCALE_ANGLE);
            RX_data.pitch  = (int16_t)(pitch * FB_SCALE_ANGLE);
            RX_data.rate_x = (int16_t)(overrun * FB_SCALE_RATE);
            RX_data.rate_y = (int16_t)(pitch_rate * FB_SCALE_RATE);
            RX_data.rate_z = (int16_t)(yaw * FB_SCALE_RATE);
            // state: bit0 = đang chạy, bit1 = đang bị khóa khẩn cấp,
            //        bit2-3 = nguyên nhân ngắt, bit4-5 = nguyên nhân reset
            RX_data.state  = (uint8_t)((flag_run ? 0x01 : 0x00)
                           | (failsafe_latch ? 0x02 : 0x00)
                           | ((fs_cause  & 0x03) << 2)
                           | ((rst_cause & 0x03) << 4));
            NRF_LoadAck();
        }

        // II. Failsafe độc lập với ngắt MPU: tách riêng để biết nguyên nhân
        if(flag_run) {
            if(failsafe_count > RADIO_TIMEOUT_TICKS)  trip_failsafe(CAUSE_RADIO);
            else if(mpu_timeout > MPU_TIMEOUT_TICKS)  trip_failsafe(CAUSE_MPU);
        }

        // III. Reset / Standby
        if(flag) {
            set_motors(MOTOR_OFF, MOTOR_OFF, MOTOR_OFF, MOTOR_OFF);
            reset_pid();
            flag = false;
        }
        else if(!flag_run) {
            set_motors(MOTOR_OFF, MOTOR_OFF, MOTOR_OFF, MOTOR_OFF);
            Red = failsafe_latch; Green = !failsafe_latch; Blue = 0;   // đỏ = đã ngắt khẩn cấp
        }

        // IV. Vòng lặp góc + PID (250 Hz)
        if(pid_flag) {
            bool run = flag_run;                // chụp một lần, dùng nhất quán trong cả khối
            loc_goc(DT_SEC);
            if(run) {
                if(!last_flag_run) {            // vừa chuyển sang chạy: xóa tích lũy cũ
                    reset_pid();
                    fs_cause = CAUSE_NONE;      // bắt đầu lần chạy mới: xóa nguyên nhân cũ
                }
                Red = 0; Green = 0; Blue = 1;   // đang chạy
                pid(DT_SEC);
            }
            pid_flag = false;
            last_flag_run = run && flag_run;    // pid() có thể đã ngắt khẩn cấp
        }
    }
}

// --- CLOCK 64MHz ---
void OSC_INIT(void) {
    OSCTUNEbits.INTSRC = 1;
    OSCTUNEbits.PLLEN  = 1;               // PLLCFG = ON nên bit này không bắt buộc
    OSCTUNEbits.TUN    = 0;               // 0 = tần số hiệu chuẩn gốc (0b011111 là lệch lên tối đa)
    OSCCONbits.IRCF    = 0b111;           // 16 MHz
    OSCCONbits.SCS     = 0b00;
    while(!OSCCONbits.HFIOFS);          // HFINTOSC ổn định
    while(!OSCCON2bits.PLLRDY);          // PLL khóa tần số
}

void INT_INIT(void) {
    RCONbits.IPEN = 1;                  // bật ưu tiên ngắt trước khi cấu hình các bit IP

    INTCON3bits.INT1IP = 0;             // INT1 (MPU) và INT2 (NRF): ưu tiên thấp
    INTCON3bits.INT2IP = 0;

    TRISBbits.RB2       = 1;
    INTCON2bits.INTEDG2 = 0;            // cạnh xuống
    INTCON3bits.INT2IF  = 0;
    INTCON3bits.INT2IE  = 1;

    TRISBbits.RB1       = 1;
    INTCON2bits.INTEDG1 = 0;
    INTCON3bits.INT1F   = 0;
    INTCON3bits.INT1E   = 1;

    INTCON2bits.NOT_RBPU = 0;           // bật trở kéo lên PORTB
}

void GPIO_INIT(void) {
    ANSELA = 0x00;
    ANSELB = 0x00;
    ANSELC = 0x00;
    ANSELD = 0x00;
    ANSELE = 0x00;

    TRISAbits.RA1 = 0;
    TRISAbits.RA2 = 0;
    TRISAbits.RA3 = 0;

    PIN0 = 0; PIN1 = 0; PIN2 = 0; PIN3 = 0;

    Blue = 1; Green = 0; Red = 0;
    pw0 = 0; pw1 = 0; pw2 = 0; pw3 = 0;
}

void Timer3_init(void) {
    T3CONbits.TMR3CS = 0b00;
    T3CONbits.T3CKPS = 0b11;            // Prescaler 1:8 => 0.5us/tick
    TMR3 = count;
    IPR2bits.TMR3IP  = 1;
    PIR2bits.TMR3IF  = 0;
    PIE2bits.TMR3IE  = 1;
    T3CONbits.TMR3ON = 1;
}

void ccp_init(void) {
    TRISE = 0;
    CCPTMRS0bits.C2TSEL = 0b01;
    CCPTMRS0bits.C1TSEL = 0b01;
    CCPTMRS0bits.C3TSEL = 0b01;
    CCPTMRS1bits.C5TSEL = 0b01;

    CCP1CONbits.CCP1M = 0b1010;          // compare, chỉ báo ngắt
    CCP2CONbits.CCP2M = 0b1010;
    CCP3CONbits.CCP3M = 0b1010;
    CCP5CONbits.CCP5M = 0b1010;

    IPR1bits.CCP1IP = 1;
    IPR2bits.CCP2IP = 1;
    IPR4bits.CCP3IP = 1;
    IPR4bits.CCP5IP = 1;

    PIR1bits.CCP1IF = 0;
    PIR4bits.CCP3IF = 0;
    PIR4bits.CCP5IF = 0;
    PIR2bits.CCP2IF = 0;

    PIE1bits.CCP1IE = 1;
    PIE4bits.CCP3IE = 1;
    PIE2bits.CCP2IE = 1;
    PIE4bits.CCP5IE = 1;
}

void TIMER3_INT(void) {
    if(PIR2bits.TMR3IF) {
        TMR3 = count;
        uint8_t a = motor_active;       // đọc chỉ số một lần, dùng nhất quán cho cả 4 motor

        pw0 = 1; pw1 = 1; pw2 = 1; pw3 = 1;

        CCPR2 = motor_buf[a][0] + count;
        CCPR3 = motor_buf[a][1] + count;
        CCPR5 = motor_buf[a][2] + count;
        CCPR1 = motor_buf[a][3] + count;

        // Bộ đếm timeout (bão hòa ở 255 để không tràn về 0)
        if(failsafe_count < 255) failsafe_count++;
        if(mpu_timeout    < 255) mpu_timeout++;

        PIR2bits.TMR3IF = 0;
    }
}

void CCP_INT(void) {
    if(PIR2bits.CCP2IF) { pw0 = 0; PIR2bits.CCP2IF = 0; }
    if(PIR4bits.CCP3IF) { pw1 = 0; PIR4bits.CCP3IF = 0; }
    if(PIR4bits.CCP5IF) { pw2 = 0; PIR4bits.CCP5IF = 0; }
    if(PIR1bits.CCP1IF) { pw3 = 0; PIR1bits.CCP1IF = 0; }
}

void INT_1(void) {
   static uint8_t c = 0;
    if(INTCON3bits.INT1F) {
        INTCON3bits.INT1F = 0;
        if(++c < 4) return;              // MPU bắn 1 kHz, cứ 4 lần mới đọc FIFO
        c = 0;
        if(MPU6050_fifo4((uint8_t*)mpu_data)) {
            mpu_timeout = 0;
            pid_flag = true;
            overrun++;
        }
    }
}

// --- TÍNH GÓC ---
void loc_goc(float dt_sec) {
    static uint8_t acc_init = 0;
    static float axr = 0.0f, ayr = 0.0f, azr = 1.0f;   // mốc đo rung
    static float axa = 0.0f, aya = 0.0f, aza = 1.0f;   // gia tốc dùng tính góc
    static float vib = 0.0f;                           // năng lượng rung (g^2)

    uint8_t d[14];

    // Chép 14 byte ra bản sao (chặn ngắt thấp để ISR MPU không ghi đè giữa chừng)
    unsigned char pe = INTCONbits.PEIE;
    INTCONbits.PEIE = 0;
    for(uint8_t i = 0; i < 14; i++) d[i] = mpu_data[i];
    INTCONbits.PEIE = pe;

    // Đổi đơn vị: accel +-8g (4096 LSB/g), gyro +-500 deg/s (65.5 LSB/(deg/s))
    axcc = (float)((int16_t)((d[0] << 8) | d[1])) / 4096.0f;
    aycc = (float)((int16_t)((d[2] << 8) | d[3])) / 4096.0f;
    azcc = (float)((int16_t)((d[4] << 8) | d[5])) / 4096.0f;

    gx = (float)((int16_t)((d[8]  << 8) | d[9]))  / 65.5f;
    gy = (float)((int16_t)((d[10] << 8) | d[11])) / 65.5f;
    gz = (float)((int16_t)((d[12] << 8) | d[13])) / 65.5f;

    // Gyro đã trừ offset
    yaw        = gz - gzs;
    roll_rate  = gx - gxs;
    pitch_rate = gy - gys;

    // Lần đầu: khởi tạo các bộ lọc bằng mẫu hiện tại
    if(!acc_init) {
        axr = axa = axcc;
        ayr = aya = aycc;
        azr = aza = azcc;
        acc_init = 1;
    }

    // Mốc nhanh (chỉ để đo rung, KHÔNG dùng tính góc)
    axr += ACC_REF_ALPHA * (axcc - axr);
    ayr += ACC_REF_ALPHA * (aycc - ayr);
    azr += ACC_REF_ALPHA * (azcc - azr);

    // Gia tốc dùng cho góc (alpha = 1 nghĩa là dùng mẫu thô)
    axa += ACC_LPF_ALPHA * (axcc - axa);
    aya += ACC_LPF_ALPHA * (aycc - aya);
    aza += ACC_LPF_ALPHA * (azcc - aza);

    // --- Đo rung: attack tức thì, decay chậm ---
    float dx = axcc - axr;
    float dy = aycc - ayr;
    float dz = azcc - azr;
    float vib_now = dx * dx + dy * dy + dz * dz;
    if(vib_now > vib) vib = vib_now;                 // lên ngay mẫu đầu tiên
    else              vib += VIB_ALPHA * (vib_now - vib);

    // --- Gia tốc kéo dài ngoài trọng lực (vọt/lao, tay lắc) ---
    float acc_error = sqrtf(axr * axr + ayr * ayr + azr * azr) - 1.0f;

    // R thích nghi (deg^2): rung hoặc gia tốc ngoài trọng lực thì giảm tin gia tốc kế
    float R_dyn = kalmanRoll.R_measure + R_ACC_SCALE * (vib + acc_error * acc_error);
    if(R_dyn > R_MAX) R_dyn = R_MAX;

    // --- Góc từ gia tốc kế ---
    float ax2 = axa * axa;
    float ay2 = aya * aya;
    float az2 = aza * aza;
    float roll_acc  =  atan2f(aya, sqrtf(ax2 + az2)) * RAD2DEG;
    float pitch_acc = -atan2f(axa, sqrtf(ay2 + az2)) * RAD2DEG;

    // Kalman: gyro dẫn dắt (nhanh), gia tốc kế kéo về theo độ tin cậy
    roll  = getKalman(&kalmanRoll,  roll_acc,  -roll_rate,  dt_sec, R_dyn) - roll_fix;
    pitch = getKalman(&kalmanPitch, pitch_acc, -pitch_rate, dt_sec, R_dyn) - pitch_fix;
}
// --- PID ATTITUDE ---
void pid(float dt_sec) {
    Payload_t cmd;

    // 1. Nghiêng quá giới hạn -> ngắt khẩn cấp (mất sóng / mất MPU đã xử lý ở vòng main)
    if(fabsf(roll) > MAX_TILT_DEG || fabsf(pitch) > MAX_TILT_DEG) {
        trip_failsafe(CAUSE_TILT);
        return;
    }

    get_cmd(&cmd);

    value = (uint16_t)cmd.speed * 20 + 2000;
    if(value > THROTTLE_MAX) value = THROTTLE_MAX;

    Eroll  = (float)cmd.roll_ref  / 100.0f - roll;
    Epitch = (float)cmd.pitch_ref / 100.0f - pitch;
    Eyaw   = (float)cmd.Rate_ref - yaw;

    if(value > 2400) {
        sum_Eroll  += Eroll  * dt_sec;
        sum_Epitch += Epitch * dt_sec;
        sum_Eyaw   += Eyaw   * dt_sec;
    } else {
        sum_Eroll = 0.0f; sum_Epitch = 0.0f; sum_Eyaw = 0.0f;
    }

    // Anti-windup
    sum_Eroll  = clampf(sum_Eroll,  -I_LIMIT, I_LIMIT);
    sum_Epitch = clampf(sum_Epitch, -I_LIMIT, I_LIMIT);
    sum_Eyaw   = clampf(sum_Eyaw,   -I_LIMIT, I_LIMIT);

    roll_output  = ((cmd.kp_angle * Eroll)  + cmd.ki_angle * sum_Eroll  + cmd.kd_angle * roll_rate)  / 5.0f;
    pitch_output = ((cmd.kp_angle * Epitch) + cmd.ki_angle * sum_Epitch + cmd.kd_angle * pitch_rate) / 5.0f;
    yaw_output   = ((cmd.kp_yaw * Eyaw)    + cmd.ki_yaw * sum_Eyaw     + cmd.kd_yaw * (Eyaw - last_Eyaw) / dt_sec) / 10.0f;
    last_Eyaw = Eyaw;

    // Kẹp đầu ra trước khi trộn (tránh tràn khi ép kiểu)
    roll_output  = clampf(roll_output,  -PID_OUT_MAX, PID_OUT_MAX);
    pitch_output = clampf(pitch_output, -PID_OUT_MAX, PID_OUT_MAX);
    yaw_output   = clampf(yaw_output,   -PID_OUT_MAX, PID_OUT_MAX);

    // Mixer: dấu phải khớp vị trí/chiều quay thật, test khi chưa gắn cánh quạt
    float m0 = value + roll_output + pitch_output - yaw_output;
    float m1 = value - roll_output + pitch_output + yaw_output;
    float m2 = value - roll_output - pitch_output - yaw_output;
    float m3 = value + roll_output - pitch_output + yaw_output;

    set_motors((uint16_t)clampf(m0, MOTOR_MIN, MOTOR_MAX),
               (uint16_t)clampf(m1, MOTOR_MIN, MOTOR_MAX),
               (uint16_t)clampf(m2, MOTOR_MIN, MOTOR_MAX),
               (uint16_t)clampf(m3, MOTOR_MIN, MOTOR_MAX));
}