#include "NRF.h"

// --- Định nghĩa biến toàn cục (khai báo extern trong NRF.h) ---
Payload_t TX_data;
featback  RX_data;
volatile bool flag             = true;
volatile bool flag_run         = false;
volatile bool last_flag_run    = false;
volatile bool failsafe_latch = false;

// 1. Ghi một giá trị vào thanh ghi của nRF24L01
void NRF_WriteReg(unsigned char reg, unsigned char val) {
    CSN = 0;                            // Kéo CSN xuống mức thấp để bắt đầu truyền thông SPI
    SPI_RW(NRF_CMD_W_REG | reg);        // Gửi lệnh ghi kèm theo địa chỉ thanh ghi
    SPI_RW(val);                        // Gửi giá trị cần ghi vào thanh ghi
    CSN = 1;                            // Kéo CSN lên mức cao để kết thúc giao dịch SPI
}

// 2. Khởi tạo nRF24L01+ ở chế độ PRX (Primary Receiver), bật ACK payload và dynamic payload
void NRF_Init(void) {
    unsigned char addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

    CE = 0;                             // Tắt CE trong quá trình cấu hình
    __delay_ms(100);                    // Chờ nRF ổn định nguồn điện sau khi khởi động

    NRF_WriteReg(NRF_REG_EN_AA,     0x01);            // Bật tính năng Auto-ACK cho pipe 0
    NRF_WriteReg(NRF_REG_EN_RXADDR, 0x01);            // Chỉ bật địa chỉ nhận ở pipe 0
    NRF_WriteReg(NRF_REG_SETUP_AW,  0x03);            // Độ rộng địa chỉ là 5 byte
    NRF_WriteReg(NRF_REG_RF_CH,     0x40);            // Chọn kênh truyền thông 64 (0x40), PHẢI trùng với bên phát
    NRF_WriteReg(NRF_REG_RF_SETUP,  0x0F);            // Tốc độ truyền 2 Mbps, công suất phát 0 dBm

    // Cài đặt địa chỉ nhận cho Pipe 0
    CSN = 0; SPI_RW(NRF_CMD_W_REG | NRF_REG_RX_ADDR_P0);
    for (int i = 0; i < 5; i++) SPI_RW(addr[i]);
    CSN = 1;

    NRF_WriteReg(NRF_REG_RX_PW_P0,  PAY_LOAD_RX);     // Cấu hình độ rộng payload tĩnh cho pipe 0 (sẽ bị bỏ qua khi bật DPL)
    NRF_WriteReg(NRF_REG_FEATURE,   0x06);            // Bật EN_DPL (Dynamic Payload) và EN_ACK_PAY (phải bật trước DYNPD)
    NRF_WriteReg(NRF_REG_DYNPD,     0x01);            // Bật tính năng Dynamic Payload cho pipe 0

    // Xóa bộ đệm FIFO TX và FIFO RX tránh rác dữ liệu cũ
    CSN = 0; SPI_RW(NRF_CMD_FLUSH_TX); CSN = 1;
    CSN = 0; SPI_RW(NRF_CMD_FLUSH_RX); CSN = 1;
    NRF_WriteReg(NRF_REG_STATUS, 0x70);               // Xóa toàn bộ các cờ ngắt cũ trong thanh ghi trạng thái

    // CONFIG = 0x3F: Bật PWR_UP, PRIM_RX, CRC 2 byte,
    // MASK_TX_DS = 1 và MASK_MAX_RT = 1 => Chân IRQ CHỈ kéo xuống mức thấp khi nhận được gói tin mới (RX_DR).
    // (Nếu dùng 0x0F, TX_DS cũng sẽ kéo IRQ xuống khi gói ACK được phát đi, gây ra ngắt thừa thứ hai.)
    NRF_WriteReg(NRF_REG_CONFIG, 0x3F);
    __delay_ms(5);

    // Nạp sẵn một gói ACK payload mặc định (toàn số 0) để sẵn sàng phản hồi
    CSN = 0; SPI_RW(NRF_CMD_W_ACK_PL);
    for (int i = 0; i < PAY_LOAD_FB; i++) SPI_RW(0x00);
    CSN = 1;

    CE = 1;                             // Bật chân CE để chuyển nRF sang trạng thái lắng nghe (Listening mode)
}

// 3. Xử lý ngắt nhận dữ liệu (được gọi bên trong Low_ISR). Trả về true nếu có gói tin hợp lệ mới.
bool NRF_INT(void)
{
    bool got = false;
    unsigned char status;

    // Xóa cờ ngắt TRƯỚC khi đọc dữ liệu: Nếu có gói mới đến đúng lúc đang đọc, cờ RX_DR sẽ được set lại
    // và sinh ra cạnh xuống mới ở chân IRQ, đảm bảo không bị mất gói tin.
    NRF_WriteReg(NRF_REG_STATUS, 0x70);

    // Rút sạch RX FIFO (độ sâu tối đa 3 gói), gói lấy ra cuối cùng sẽ là gói mới nhất.
    for (unsigned char n = 0; n < 3; n++) {
        CSN = 0; status = SPI_RW(NRF_CMD_NOP); CSN = 1;       // Gửi lệnh NOP để đọc về thanh ghi STATUS
        if ((status & NRF_STATUS_RX_P_NO) == NRF_STATUS_RX_P_NO) break;   // Kiểm tra nếu FIFO đã rỗng thì thoát vòng lặp

        // Đọc dữ liệu Payload từ RX FIFO
        CSN = 0;
        SPI_RW(NRF_CMD_R_PAYLOAD);
        for (unsigned char k = 0; k < PAY_LOAD_RX; k++) {
            TX_data.buffer[k] = SPI_RW(NRF_CMD_NOP);
        }
        CSN = 1;
        got = true; // Đánh dấu đã nhận được ít nhất một gói hợp lệ
    }

    // Nếu đọc thành công dữ liệu từ tay cầm
    if (got) {
        flag = (TX_data.Turn >> 1) & 1;                 // Giải mã cờ điều khiển phụ từ byte Turn
        bool want_run = TX_data.Turn & 1;               // Trạng thái yêu cầu chạy/mở motor từ tay cầm
        if (!want_run) failsafe_latch = false;          // Nếu tay cầm hạ cờ chạy thì cho phép mở khóa (arm) lại
        flag_run = want_run && !failsafe_latch;         // Nếu đang bị khóa (latch) thì không cho phép tự động chạy lại
    }
    return got;
}

// 4. Nạp dữ liệu ACK payload từ hàm main. Chặn ngắt ưu tiên thấp (INT1/INT2) trong lúc giao tiếp SPI
//    để tránh bị ngắt khác chen ngang làm lỗi khung truyền SPI. Ngắt PWM (ưu tiên cao) không bị ảnh hưởng.
void NRF_LoadAck(void)
{
    unsigned char pe = INTCONbits.PEIE;
    INTCONbits.PEIE = 0;                // Tạm thời vô hiệu hóa ngắt ngoại vi ưu tiên thấp (giữ an toàn giao tiếp SPI)

    CSN = 0;
    SPI_RW(NRF_CMD_W_ACK_PL);           // Gửi lệnh nạp ACK Payload vào bộ đệm của nRF
    for (unsigned char i = 0; i < PAY_LOAD_FB; i++) {
        SPI_RW(RX_data.buffer[i]);      // Gửi từng byte dữ liệu telemetry phản hồi về tay cầm
    }
    CSN = 1;

    INTCONbits.PEIE = pe;               // Khôi phục lại trạng thái ngắt ưu tiên thấp ban đầu
}