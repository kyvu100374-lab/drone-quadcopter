# Drone Quadcopter VVT

Hệ thống điều khiển quadcopter gồm 3 phần, đi theo chiều dữ liệu:

```
[WPF trên PC] --UART 9600--> [Bộ phát RF (PIC16 + nRF24L01)] --2.4GHz--> [Drone (PIC18F45K22 + nRF24L01)]
      ^                                                                           |
      +------------------------- telemetry (ACK payload) <------------------------+
```

## Cấu trúc thư mục

| Thư mục | Vai trò | Phần cứng / công nghệ |
|---|---|---|
| `1_drone_PIC18F45K22/` | Firmware drone: đọc MPU6050, lọc Kalman, PID attitude, xuất PWM 4 motor, failsafe | PIC18F45K22 @ 64 MHz (XC8), MPU6050 (I2C), nRF24L01 (SPI, chế độ RX) |
| `2_rf_transmitter_PIC16/` | Cầu nối UART <-> RF: nhận khung lệnh từ PC, gửi qua nRF, trả telemetry về PC | PIC16 (HS 16 MHz, XC8), nRF24L01 (chế độ TX) |
| `3_wpf_ground_station/` | Giao diện điều khiển và đồ thị thời gian thực trên PC | WPF (.NET), LiveCharts, SerialPort |

## Giao thức

Dùng chung kênh RF 0x40 (64), 2 Mbps, địa chỉ `E7 E7 E7 E7 E7`, Auto-ACK + Dynamic Payload + ACK payload.

**PC -> bộ phát (UART 9600, 15 byte):** `0xCC 0xDD` + 13 byte lệnh

| Byte | Trường |
|---|---|
| 0 | speed (ga, 0-100) |
| 1-2 | roll_ref (int16, độ x 100) |
| 3-4 | pitch_ref (int16, độ x 100) |
| 5 | Rate_ref (int8, tốc độ yaw mong muốn) |
| 6 | Turn: bit0 = chạy/dừng, bit1 = reset PID/motor (cũng dùng cho nút ZW SET) |
| 7-9 | kp / ki / kd góc |
| 10-12 | kp / ki / kd yaw |

**Drone -> PC (qua ACK payload, bộ phát gửi lên UART với header `0xAA 0xBB`, 11 byte):**

| Byte | Trường |
|---|---|
| 0-1 | roll (int16, độ x 100) |
| 2-3 | pitch (int16, độ x 100) |
| 4-5 | rate_x (int16, x 10) |
| 6-7 | rate_y (int16, x 10) |
| 8-9 | rate_z / yaw (int16, x 10) |
| 10 | state: bit0 = đang chạy, bit1 = khóa khẩn cấp (latch), bit2-3 = nguyên nhân ngắt (1 mất sóng, 2 mất MPU, 3 nghiêng quá 60 độ), bit4-5 = nguyên nhân reset (1 WDT, 2 BOR) |

Bộ phát gửi `0xEE` lên PC mỗi 500 ms nếu không thấy module nRF.

## Cách build

- **Drone / Bộ phát:** mở bằng MPLAB X, tạo project mới với đúng chip, thêm các file `.c` / `.h` trong thư mục tương ứng, biên dịch bằng XC8. Hai thư mục đều có `NRF.*` và `SPI.*` riêng (khác nhau), **không** thêm chung vào một project.
- **WPF:** tạo project WPF (namespace `drone_control`), thay `MainWindow.xaml` và `MainWindow.xaml.cs` bằng hai file trong thư mục này, cài NuGet `LiveCharts.Wpf`.

## Lưu ý an toàn

Khi test mixer và PID, luôn tháo cánh quạt. Kiểm tra dấu mixer khớp vị trí và chiều quay motor thực tế.
