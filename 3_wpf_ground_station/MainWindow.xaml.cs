using LiveCharts;
using LiveCharts.Wpf;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO.Ports;
using System.Text.RegularExpressions;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Threading;

namespace drone_control
{
    public partial class MainWindow : Window
    {
        public Func<double, string> YFormatter { get; set; }
        public SeriesCollection DroneSeries { get; set; }
        SerialPort serial = new SerialPort();

        Button Tien, Lui, Trai, Phai, Rs;
        Button Up, Down, Set, Run;

        DispatcherTimer timer = new DispatcherTimer();

        int Speed = 0;
        float Roll_F = 0.0f, Pitch_F = 0.0f;
        float Roll_S = 0.0f, Pitch_S = 0.0f;
        int Yaw_F = 0;
        int flag_RW = 0, flag_up = 0;
        int dem = 0;

        bool[] current_state = new bool[4] { false, false, false, false };
        bool RUN = false, LAST_RUN = false;
        int OK = 0;

        float _Roll = 0, _Pitch = 0, _Rate = 0;
        float Roll_Rate = 0, Pitch_Rate = 0;
        int Kp = 0, Ki = 0, Kd = 0, Kpr = 0, Kir = 0, Kdr = 0;

        private List<byte> _byteBuffer = new List<byte>(1024);

        public MainWindow()
        {
            InitializeComponent();
            this.PreviewKeyDown += Window_KeyDown;
            this.Focusable = true;
            this.Focus();

            run.Content = "RUN";
            run.Background = System.Windows.Media.Brushes.Green;

            DroneSeries = new SeriesCollection
            {
                new LineSeries { Title = "Roll", Values = new ChartValues<double>(), PointGeometry = null, Stroke = System.Windows.Media.Brushes.Red, Fill = System.Windows.Media.Brushes.Transparent, LineSmoothness = 0 },
                new LineSeries { Title = "Pitch", Values = new ChartValues<double>(), PointGeometry = null, Stroke = System.Windows.Media.Brushes.Green, Fill = System.Windows.Media.Brushes.Transparent, LineSmoothness = 0 },
                new LineSeries { Title = "Yaw", Values = new ChartValues<double>(), PointGeometry = null, Stroke = System.Windows.Media.Brushes.Blue, Fill = System.Windows.Media.Brushes.Transparent, LineSmoothness = 0 }
            };

            YFormatter = value => value.ToString("F2");
            DataContext = this;

            cbPort.ItemsSource = SerialPort.GetPortNames();
            serial.DataReceived += SerialPort_DataReceived;

            timer.Interval = TimeSpan.FromMilliseconds(30);
            timer.Tick += Timer_Tick;
            timer.Start();

            FindControls();
            SetupInputBoxes();
        }

        private void FindControls()
        {
            Up = (Button)this.FindName("up"); Down = (Button)this.FindName("down");
            Tien = (Button)this.FindName("tien"); Lui = (Button)this.FindName("lui");
            Trai = (Button)this.FindName("trai"); Phai = (Button)this.FindName("phai");
            Rs = (Button)this.FindName("rs"); Set = (Button)this.FindName("set");
            Run = (Button)this.FindName("run");
        }

        // =====================================================================================
        // VALIDATE Ô NHẬP
        // =====================================================================================
        static readonly Regex IntRegex = new Regex(@"^\d{0,3}$");
        static readonly Regex FloatRegex = new Regex(@"^-?\d{0,2}([.,]\d{0,2})?$");

        static bool TryParseFloat(string s, out float v)
        {
            return float.TryParse((s ?? "").Replace(',', '.'), NumberStyles.Float,
                                  CultureInfo.InvariantCulture, out v);
        }

        static string F2(float v) { return v.ToString("0.00", CultureInfo.InvariantCulture); }

        static string Preview(TextBox tb, string input)
        {
            return tb.Text.Remove(tb.SelectionStart, tb.SelectionLength).Insert(tb.SelectionStart, input);
        }

        private void SetupInputBoxes()
        {
            foreach (var tb in new[] { kp, ki, kd, kpr, kir, kdr })
            {
                var t = tb;
                HookBox(t, IntRegex, () => NormalizeInt(t));
            }
            foreach (var tb in new[] { roll_z, pitch_z })
            {
                var t = tb;
                HookBox(t, FloatRegex, () => NormalizeFloat(t));
            }
        }

        private void HookBox(TextBox tb, Regex rx, Action normalize)
        {
            InputMethod.SetIsInputMethodEnabled(tb, false);          // tắt bộ gõ tiếng Việt

            tb.PreviewTextInput += (s, e) => e.Handled = !rx.IsMatch(Preview(tb, e.Text));

            DataObject.AddPastingHandler(tb, (s, e) =>               // chặn dán ký tự lỗi
            {
                var text = e.DataObject.GetData(DataFormats.UnicodeText) as string;
                if (text == null || !rx.IsMatch(Preview(tb, text))) e.CancelCommand();
            });

            tb.LostFocus += (s, e) => normalize();                   // rời ô -> chuẩn hóa
            tb.KeyDown += (s, e) =>
            {
                if (e.Key == Key.Enter) { normalize(); Keyboard.ClearFocus(); this.Focus(); }
            };
        }

        private void NormalizeInt(TextBox tb)
        {
            int v;
            if (!int.TryParse(tb.Text, out v)) v = 0;
            v = Math.Max(0, Math.Min(255, v));
            tb.Text = v.ToString();
        }

        private void NormalizeFloat(TextBox tb)
        {
            float v;
            if (!TryParseFloat(tb.Text, out v)) v = (tb == roll_z) ? Roll_S : Pitch_S;
            v = Math.Max(-20f, Math.Min(20f, v));
            tb.Text = F2(v);
        }

        private int StepInt(TextBox tb, int delta)
        {
            int v;
            if (!int.TryParse(tb.Text, out v)) v = 0;
            v = Math.Max(0, Math.Min(255, v + delta));
            tb.Text = v.ToString();
            return v;
        }

        private float StepFloat(TextBox tb, float current, float delta)
        {
            float v;
            if (!TryParseFloat(tb.Text, out v)) v = current;
            v = Math.Max(-20f, Math.Min(20f, v + delta));
            tb.Text = F2(v);
            return v;
        }

        // =====================================================================================
        // VÒNG LẶP CHÍNH
        // =====================================================================================
        private void Timer_Tick(object sender, EventArgs e)
        {
            HandleButtonInputs();
            SendDataToHardware();
        }

        private void HandleButtonInputs()
        {
            dem++;
            if (dem > 2)
            {
                dem = 0;

                current_state[0] = Tien?.IsPressed ?? false;
                current_state[1] = Lui?.IsPressed ?? false;
                current_state[2] = Trai?.IsPressed ?? false;
                current_state[3] = Phai?.IsPressed ?? false;

                if (current_state[0] && Roll_F < 5) Roll_F += 0.1f; else if (current_state[1] && Roll_F > -5) Roll_F -= 0.1f;
                if (current_state[2] && Pitch_F < 5) Pitch_F += 0.1f; else if (current_state[3] && Pitch_F > -5) Pitch_F -= 0.1f;

                if (roll_F != null) roll_F.Text = Roll_F.ToString("N2");
                if (pitch_F != null) pitch_F.Text = Pitch_F.ToString("N2");

                if (Rs?.IsPressed == true) RC();
                if (Up?.IsPressed == true && Speed < 100) Speed++; else if (Down?.IsPressed == true && Speed > 0) Speed--;
                if (speed != null) speed.Text = Speed.ToString();

                if (RUN != LAST_RUN)
                {
                    if (OK == 1)
                    {
                        run.Content = RUN ? "STOP" : "RUN";
                        run.Background = RUN ? System.Windows.Media.Brushes.Red : System.Windows.Media.Brushes.Green;
                        OK = 0; LAST_RUN = RUN;
                    }
                    else
                    {
                        run.Content = RUN ? "RUNNING" : "DISRUN";
                        run.Background = System.Windows.Media.Brushes.Yellow;
                    }
                }
            }
        }

        void RC() { Roll_F = 0; Pitch_F = 0; Yaw_F = 0; rota.Text = "0"; roll_F.Text = "0"; pitch_F.Text = "0"; }

        private void SendDataToHardware()
        {
            if (!serial.IsOpen) return;
            try
            {
                int commandByte = (flag_RW << 1) | (flag_up);
                byte[] packet = new byte[15];

                packet[0] = 0xCC; packet[1] = 0xDD; packet[2] = (byte)Speed;
                Array.Copy(BitConverter.GetBytes((int)((Roll_F + Roll_S) * 100)), 0, packet, 3, 2);
                Array.Copy(BitConverter.GetBytes((int)((Pitch_F + Pitch_S) * 100)), 0, packet, 5, 2);
                packet[7] = (byte)Yaw_F;
                packet[8] = (byte)commandByte;
                packet[9] = (byte)Kp; packet[10] = (byte)Ki; packet[11] = (byte)Kd;
                packet[12] = (byte)Kpr; packet[13] = (byte)Kir; packet[14] = (byte)Kdr;

                serial.Write(packet, 0, packet.Length);
                flag_RW = 0;
            }
            catch { }
        }

        private void SerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            if (!serial.IsOpen) return;
            try
            {
                int bytesToRead = serial.BytesToRead;
                byte[] tempBuffer = new byte[bytesToRead];
                serial.Read(tempBuffer, 0, bytesToRead);

                _byteBuffer.AddRange(tempBuffer);

                while (_byteBuffer.Count >= 13)
                {
                    if (_byteBuffer[0] == 0xAA && _byteBuffer[1] == 0xBB)
                    {
                        byte[] rawBytes = _byteBuffer.GetRange(2, 11).ToArray();
                        _byteBuffer.RemoveRange(0, 13);

                        Dispatcher.InvokeAsync(() => DecodeDroneData(rawBytes));
                    }
                    else
                    {
                        _byteBuffer.RemoveAt(0);
                    }
                }
            }
            catch { }
        }

        private int mismatch = 0;

        private void DecodeDroneData(byte[] b)      // b.Length == 11
        {
            _Roll = BitConverter.ToInt16(b, 0) / 100.0f;
            _Pitch = BitConverter.ToInt16(b, 2) / 100.0f;
            Roll_Rate = BitConverter.ToInt16(b, 4) / 10.0f;
            Pitch_Rate = BitConverter.ToInt16(b, 6) / 10.0f;
            _Rate = BitConverter.ToInt16(b, 8) / 10.0f;   // yaw rate

            byte st = b[10];
            bool droneRun = (st & 0x01) != 0;
            bool latch = (st & 0x02) != 0;
            int cause = (st >> 2) & 0x03;   // 1 = mất sóng, 2 = mất MPU, 3 = nghiêng quá
            int rst = (st >> 4) & 0x03;   // 1 = WDT reset, 2 = brown-out

            // Drone báo đã ngắt khẩn cấp: hạ cờ chạy để drone mở khóa (latch) lại
            if (latch && RUN)
            {
                RUN = false; flag_up = 0; LAST_RUN = false; OK = 0; mismatch = 0;
                run.Content = "RUN"; run.Background = System.Windows.Media.Brushes.Green;
                MessageBox.Show("Drone ngắt khẩn cấp, nguyên nhân = " + cause + ", reset = " + rst);
            }
            else
            {
                // OK chỉ bằng 1 khi trạng thái drone thật sự khớp với lệnh PC đã gửi
                OK = (droneRun == RUN) ? 1 : 0;
                // lệnh không được drone thực hiện quá ~0.5 s (VD drone vừa reset) thì bỏ lệnh
                mismatch = (OK == 0) ? mismatch + 1 : 0;
                if (mismatch > 15)
                {
                    RUN = false; flag_up = 0; LAST_RUN = false; mismatch = 0;
                    run.Content = "RUN"; run.Background = System.Windows.Media.Brushes.Green;
                }
            }

            roll.Text = _Roll.ToString("0.00"); pitch.Text = _Pitch.ToString("0.00");
            yaw.Text = _Rate.ToString("0.00"); x.Text = Roll_Rate.ToString("0.00");
            y.Text = Pitch_Rate.ToString("0.00"); h.Text = _Rate.ToString("0.00");
            UpdateChart(0, _Roll); UpdateChart(1, _Pitch); UpdateChart(2, Roll_Rate);
        }

        private void UpdateChart(int index, double value)
        {
            var series = DroneSeries[index].Values;
            if (series.Count > 100) series.RemoveAt(0);
            series.Add(value);
        }

        private void Connect_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                if (!serial.IsOpen)
                {
                    serial.PortName = cbPort.Text;
                    serial.BaudRate = 9600;
                    serial.Open();
                    _byteBuffer.Clear();

                    this.Focusable = true; this.Focus(); Keyboard.Focus(this);
                    MessageBox.Show("Đã kết nối thành công!");
                }
                else
                {
                    serial.Close();
                    MessageBox.Show("Đã ngắt kết nối.");
                }
            }
            catch (Exception ex) { MessageBox.Show("Lỗi: " + ex.Message); }
        }

        private void Window_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.OriginalSource is TextBox) return;   // đang gõ trong ô nhập thì bỏ qua phím tắt

            if (e.Key == Key.B) { RUN = !RUN; flag_up = RUN ? 1 : 0; OK = 0; e.Handled = true; }
            else if (e.Key == Key.Up) { if (Speed < 100) Speed++; if (speed != null) speed.Text = Speed.ToString(); e.Handled = true; }
            else if (e.Key == Key.Down) { if (Speed > 0) Speed--; if (speed != null) speed.Text = Speed.ToString(); e.Handled = true; }
        }

        // =====================================================================================
        // CÁC HÀM EVENT XAML
        // =====================================================================================
        private void BtnRefresh_Click(object sender, RoutedEventArgs e) { cbPort.ItemsSource = SerialPort.GetPortNames(); }

        private void CartesianChart_Loaded(object sender, RoutedEventArgs e) { }

        private void left_Click(object sender, RoutedEventArgs e)
        {
            if (Yaw_F > -50) Yaw_F -= 1;
            rota.Text = Yaw_F.ToString();
        }

        private void right_Click(object sender, RoutedEventArgs e)
        {
            if (Yaw_F < 50) Yaw_F += 1;
            rota.Text = Yaw_F.ToString();
        }

        private void tien_Click(object sender, RoutedEventArgs e) { }
        private void runc(object sender, RoutedEventArgs e) { RUN = !RUN; flag_up = RUN ? 1 : 0; OK = 0; }
        private void set_Click(object sender, RoutedEventArgs e) { flag_RW = 1; }

        // ---- set zero ----
        private void roll_z_TextChanged(object sender, TextChangedEventArgs e)
        {
            var tb = sender as TextBox; if (tb == null) return;
            float v; if (TryParseFloat(tb.Text, out v)) Roll_S = Math.Max(-20f, Math.Min(20f, v));
        }
        private void pitch_z_TextChanged(object sender, TextChangedEventArgs e)
        {
            var tb = sender as TextBox; if (tb == null) return;
            float v; if (TryParseFloat(tb.Text, out v)) Pitch_S = Math.Max(-20f, Math.Min(20f, v));
        }
        private void giam_roll_z(object s, RoutedEventArgs e) { Roll_S = StepFloat(roll_z, Roll_S, -0.1f); }
        private void tang_roll_z(object s, RoutedEventArgs e) { Roll_S = StepFloat(roll_z, Roll_S, 0.1f); }
        private void giam_pitch_z(object s, RoutedEventArgs e) { Pitch_S = StepFloat(pitch_z, Pitch_S, -0.1f); }
        private void tang_pitch_z(object s, RoutedEventArgs e) { Pitch_S = StepFloat(pitch_z, Pitch_S, 0.1f); }

        // ---- PID Angle ----
        private void tang_kp(object s, RoutedEventArgs e) { Kp = StepInt(kp, +1); }
        private void giam_kp(object s, RoutedEventArgs e) { Kp = StepInt(kp, -1); }
        private void tang_ki(object s, RoutedEventArgs e) { Ki = StepInt(ki, +1); }
        private void giam_ki(object s, RoutedEventArgs e) { Ki = StepInt(ki, -1); }
        private void tang_kd(object s, RoutedEventArgs e) { Kd = StepInt(kd, +1); }
        private void giam_kd(object s, RoutedEventArgs e) { Kd = StepInt(kd, -1); }

        // ---- PID Yaw ----
        private void tang_kpr(object s, RoutedEventArgs e) { Kpr = StepInt(kpr, +1); }
        private void giam_kpr(object s, RoutedEventArgs e) { Kpr = StepInt(kpr, -1); }
        private void tang_kir(object s, RoutedEventArgs e) { Kir = StepInt(kir, +1); }
        private void giam_kir(object s, RoutedEventArgs e) { Kir = StepInt(kir, -1); }
        private void tang_kdr(object s, RoutedEventArgs e) { Kdr = StepInt(kdr, +1); }
        private void giam_kdr(object s, RoutedEventArgs e) { Kdr = StepInt(kdr, -1); }

        // ---- TextChanged: chỉ cập nhật khi số hợp lệ, ô trống thì giữ giá trị cũ ----
        private void kp_TextChanged(object s, TextChangedEventArgs e) { int v; if (int.TryParse(((TextBox)s).Text, out v)) Kp = Math.Min(v, 255); }
        private void ki_TextChanged(object s, TextChangedEventArgs e) { int v; if (int.TryParse(((TextBox)s).Text, out v)) Ki = Math.Min(v, 255); }
        private void kd_TextChanged(object s, TextChangedEventArgs e) { int v; if (int.TryParse(((TextBox)s).Text, out v)) Kd = Math.Min(v, 255); }
        private void kpr_TextChanged(object s, TextChangedEventArgs e) { int v; if (int.TryParse(((TextBox)s).Text, out v)) Kpr = Math.Min(v, 255); }
        private void kir_TextChanged(object s, TextChangedEventArgs e) { int v; if (int.TryParse(((TextBox)s).Text, out v)) Kir = Math.Min(v, 255); }
        private void kdr_TextChanged(object s, TextChangedEventArgs e) { int v; if (int.TryParse(((TextBox)s).Text, out v)) Kdr = Math.Min(v, 255); }
    }
}