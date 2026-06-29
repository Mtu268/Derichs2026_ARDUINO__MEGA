/*
 * ============================================================
 * ARDUINO MEGA 2560 - SLAVE MECANUM & LINE SENSOR CONTROLLER
 * NQ Robotics Architecture 2026
 * ============================================================
 * Chức năng:
 * - Điều khiển 4 động cơ Mecanum bằng Driver BTS7960
 * - Đọc và phản hồi 4 bộ Encoder bánh xe
 * - Đọc hệ thống cảm biến dò line 8 mắt x 4 mặt (32 kênh) qua 2 mạch Analog Mux 16-ch
 * - Tích hợp hệ thống Calib 32 ngưỡng độc lập lưu trữ trong EEPROM chống sai số vật lý
 * - Chạy PID vòng kín điều khiển tốc độ từng bánh xe
 * - Giao tiếp đồng bộ UART với ESP32 Master qua Serial2
 * ============================================================
 */

#include <Arduino.h>
#include <EEPROM.h> // Thư viện EEPROM dùng để lưu trữ vĩnh viễn 32 ngưỡng độc lập

// ============================================================
//  ĐỊNH NGHĨA CHÂN ĐIỀU KHIỂN ĐỘNG CƠ (BTS7960)
// ============================================================
#define MOTOR_FL_A 9
#define MOTOR_FL_B 10

#define MOTOR_FR_A 5
#define MOTOR_FR_B 6

#define MOTOR_RL_A 7
#define MOTOR_RL_B 8

#define MOTOR_RR_A 3
#define MOTOR_RR_B 4

// ============================================================
//  ĐỊNH NGHĨA CHÂN ENCODER (HỖ TRỢ NGẮT CỨNG)
// ============================================================
#define ENC_FL_A 22
#define ENC_FL_B 18  // Ngắt INT.5

#define ENC_FR_A 23
#define ENC_FR_B 19  // Ngắt INT.4

#define ENC_RL_A 24
#define ENC_RL_B 20  // Ngắt INT.3

#define ENC_RR_A 25
#define ENC_RR_B 21  // Ngắt INT.2

// ============================================================
//  ĐỊNH NGHĨA CHÂN CẢM BIẾN DÒ LINE THEO SƠ ĐỒ MỚI (29/06/2026)
//  Loại bỏ chân DP44 để tránh lỗi chân khuyết trên shield của bạn
// ============================================================
// Chân Tín hiệu (SIG) kết nối về chân Analog của Mega
#define MUX1_SIG   A14 // Mux 1 (Mặt Trước FRONT & Mặt Sau BACK)
#define MUX2_SIG   A15 // Mux 2 (Mặt Trái LEFT & Mặt Phải RIGHT)

// Chân kích hoạt riêng biệt Mux (EN)
#define MUX1_EN    48  // Kích hoạt Mux 1 (Chân DP48)
#define MUX2_EN    49  // Kích hoạt Mux 2 (Chân DP49)

// Chân chọn địa chỉ kênh chung (S0, S1, S2, S3)
#define MUX_S0     42  // Chân DP42
#define MUX_S1     43  // Chân DP43
#define MUX_S2     45  // Chân DP45 (Sử dụng thay cho DP44)
#define MUX_S3     46  // Chân DP46

// ============================================================
//  CẤU HÌNH THỨ TỰ QUÉT MẮT ĐỌC LINE (SENSOR MAPPING CONFIG)
//  Mỗi phần tử trong mảng đại diện cho KÊNH thực tế trên mạch Multiplexer (0..15).
//  Thứ tự khai báo từ TRÁI qua PHẢI tương ứng với thứ tự mắt hiển thị F1..F8, B1..B8, L1..L8, R1..R8.
// ============================================================
// Mux 1 (A14): Mặt Trước (Front) và Mặt Sau (Back)
const uint8_t MUX1_FRONT_MAP[8] = {0, 1, 2, 3, 4, 5, 6, 7};         // Định danh từ F1 -> F8
const uint8_t MUX1_BACK_MAP[8]  = {8, 9, 10, 11, 12, 13, 14, 15};   // Định danh từ B1 -> B8

// Mux 2 (A15): Mặt Trái (Left) và Mặt Phải (Right)
const uint8_t MUX2_LEFT_MAP[8]  = {0, 1, 2, 3, 4, 5, 6, 7};         // Định danh từ L1 -> L8
const uint8_t MUX2_RIGHT_MAP[8] = {8, 9, 10, 11, 12, 13, 14, 15};   // Định danh từ R1 -> R8

// ============================================================
//  CẤU HÌNH ĐỊA CHỈ EEPROM ĐỂ LƯU 32 NGƯỠNG ĐỘC LẬP
// ============================================================
#define EEPROM_MAGIC 0x55 // Mã nhận dạng để kiểm tra xem EEPROM đã calib chưa
#define ADDR_MAGIC   0
#define ADDR_F       2
#define ADDR_B       18
#define ADDR_L       34
#define ADDR_R       50

// ============================================================
//  CÁC BIẾN TRẠNG THÁI (STATE VARIABLES)
// ============================================================
// Đếm xung Encoder
volatile long encFL = 0;
volatile long encFR = 0;
volatile long encRL = 0;
volatile long encRR = 0;

// Vận tốc mục tiêu (RPM)
float targetFL = 0;
float targetFR = 0;
float targetRL = 0;
float targetRR = 0;

// Vận tốc thực tế (RPM)
float rpmFL = 0;
float rpmFR = 0;
float rpmRL = 0;
float rpmRR = 0;

// Điều khiển PWM xuất ra Driver
int pwmFL = 0;
int pwmFR = 0;
int pwmRL = 0;
int pwmRR = 0;

// Biến lưu trữ nhị phân 8-bit của cảm biến dò line (Để gửi lên ESP32)
uint8_t lineDataF = 0;
uint8_t lineDataB = 0;
uint8_t lineDataL = 0;
uint8_t lineDataR = 0;

// Biến lưu trữ giá trị Analog thực tế để phục vụ in DEBUG lên máy tính
int rawAnalogF[8] = {0};
int rawAnalogB[8] = {0};
int rawAnalogL[8] = {0};
int rawAnalogR[8] = {0};

// MẢNG CHỨA 32 NGƯỠNG SO SÁNH ĐỘC LẬP (SẼ ĐƯỢC TỰ ĐỘNG CẬP NHẬT KHI CHẠY WIZARD)
int lineThreshF[8] = {500, 500, 500, 500, 500, 500, 500, 500};
int lineThreshB[8] = {500, 500, 500, 500, 500, 500, 500, 500};
int lineThreshL[8] = {500, 500, 500, 500, 500, 500, 500, 500};
int lineThreshR[8] = {500, 500, 500, 500, 500, 500, 500, 500};

// Mảng đệm lưu trữ tạm thời giá trị đo nền Trắng và vạch Đen khi Calib
int whiteCalibF[8] = {100}; int whiteCalibB[8] = {100}; int whiteCalibL[8] = {100}; int whiteCalibR[8] = {100};
int blackCalibF[8] = {900}; int blackCalibB[8] = {900}; int blackCalibL[8] = {900}; int blackCalibR[8] = {900};

// Cấu hình đảo logic vạch line (Mặc định: 1 = Vạch đen, 0 = Nền trắng)
bool lineInvert = false;

// Ngưỡng so sánh chung (Dùng làm tương thích ngược và fallback)
int lineThresh = 500; 

// ============================================================
//  THÔNG SỐ CẤU HÌNH (SETTINGS)
// ============================================================
// SỬA: Cập nhật PPR cho động cơ giảm tốc hành tinh tỉ lệ 1:19.2
// Tính toán: 11 xung đĩa từ thô * 2 sườn ngắt CHANGE * 19.2 tỉ số truyền = 422.4
const float PPR = 422.4; 
float Kp = 1.5;          // Hệ số PID bám tốc độ

// ============================================================
//  HÀM ĐỌC / GHI EEPROM BẢO VỆ NGƯỠNG ĐỘC LẬP KHI TẮT NGUỒN
// ============================================================
void loadCalibration() {
  if (EEPROM.read(ADDR_MAGIC) == EEPROM_MAGIC) {
    EEPROM.get(ADDR_F, lineThreshF);
    EEPROM.get(ADDR_B, lineThreshB);
    EEPROM.get(ADDR_L, lineThreshL);
    EEPROM.get(ADDR_R, lineThreshR);
    Serial.println(F("[CALIB] Da nap 32 nguong doc lap tu EEPROM."));
  } else {
    Serial.println(F("[CALIB] Khong tim thay EEPROM. Su dung nguong mac dinh 500."));
    for (int i = 0; i < 8; i++) {
      lineThreshF[i] = 500;
      lineThreshB[i] = 500;
      lineThreshL[i] = 500;
      lineThreshR[i] = 500;
    }
  }
}

void saveCalibration() {
  EEPROM.write(ADDR_MAGIC, EEPROM_MAGIC);
  EEPROM.put(ADDR_F, lineThreshF);
  EEPROM.put(ADDR_B, lineThreshB);
  EEPROM.put(ADDR_L, lineThreshL);
  EEPROM.put(ADDR_R, lineThreshR);
  Serial.println(F("[CALIB] Da luu 32 nguong doc lap vao EEPROM."));
}

// ============================================================
//  HÀM NGẮT ENCODER (XỬ LÝ ĐA CHIỀU CHUẨN XÁC)
// ============================================================
void ISR_FL() {
  if (digitalRead(ENC_FL_A) == digitalRead(ENC_FL_B)) encFL++;
  else encFL--;
}

void ISR_FR() {
  if (digitalRead(ENC_FR_A) == digitalRead(ENC_FR_B)) encFR++;
  else encFR--;
}

void ISR_RL() {
  if (digitalRead(ENC_RL_A) == digitalRead(ENC_RL_B)) encRL++;
  else encRL--;
}

void ISR_RR() {
  if (digitalRead(ENC_RR_A) == digitalRead(ENC_RR_B)) encRR++;
  else encRR--;
}

// ============================================================
//  HÀM ĐIỀU KHIỂN ĐỘNG CƠ (BTS7960)
// ============================================================
void setMotor(int pinA, int pinB, int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    analogWrite(pinA, pwm);
    analogWrite(pinB, 0);
  } else if (pwm < 0) {
    analogWrite(pinA, 0);
    analogWrite(pinB, -pwm);
  } else {
    analogWrite(pinA, 0);
    analogWrite(pinB, 0);
  }
}

// ============================================================
//  ĐỌC DỮ LIỆU CẢM BIẾN DÒ LINE QUA 2 MẠCH ANALOG MUX
// ============================================================
// Hàm thiết lập địa chỉ nhị phân cho các chân S0..S3
void setMuxAddress(uint8_t channel) {
  digitalWrite(MUX_S0, (channel >> 0) & 1);
  digitalWrite(MUX_S1, (channel >> 1) & 1);
  digitalWrite(MUX_S2, (channel >> 2) & 1);
  digitalWrite(MUX_S3, (channel >> 3) & 1);
}

void scanAllLineSensors() {
  // Đảm bảo cả hai IC Multiplexer đều được kích hoạt (EN kéo LOW)
  digitalWrite(MUX1_EN, LOW);
  digitalWrite(MUX2_EN, LOW);

  uint8_t tempF = 0;
  uint8_t tempB = 0;
  uint8_t tempL = 0;
  uint8_t tempR = 0;

  // Quét đồng bộ thứ tự 8 vị trí mắt (từ Trái qua Phải)
  for (uint8_t eye = 0; eye < 8; eye++) {
    // bitPos tương ứng với vị trí hiển thị trên màn hình LCD của ESP32 
    // Mắt ngoài cùng bên trái (eye = 0) tương ứng với bit cao nhất (bit 7) để in ra trước
    uint8_t bitPos = 7 - eye; 

    // --- ĐỌC MẶT TRƯỚC (MUX 1) ---
    setMuxAddress(MUX1_FRONT_MAP[eye]);
    delayMicroseconds(10); // Chờ chuyển kênh analog ổn định điện áp
    
    // Đọc nháp một lần để xả tụ nạp của bộ chuyển đổi ADC (Tránh nhiễu chéo kênh)
    analogRead(MUX1_SIG); 
    delayMicroseconds(2);
    int valF = analogRead(MUX1_SIG); // Đọc thật lấy kết quả chính xác
    rawAnalogF[eye] = valF; // Lưu giá trị Analog thô để Debug và Calib
    
    // So sánh với ngưỡng riêng biệt của chính mắt này
    bool stateF = (valF > lineThreshF[eye]);
    if (stateF) {
      tempF |= (1 << bitPos);
    }

    // --- ĐỌC MẶT SAU (MUX 1) ---
    setMuxAddress(MUX1_BACK_MAP[eye]);
    delayMicroseconds(10);
    
    analogRead(MUX1_SIG); // Đọc nháp xả tụ
    delayMicroseconds(2);
    int valB = analogRead(MUX1_SIG);
    rawAnalogB[eye] = valB; // Lưu giá trị Analog thô để Debug và Calib
    
    // So sánh với ngưỡng riêng biệt của chính mắt này
    bool stateB = (valB > lineThreshB[eye]);
    
    // SỬA LỖI VẬT LÝ: Mắt dò line cuối cùng (mắt số 8, eye == 7) của mạch BACK bị ngược phần cứng
    if (eye == 7) {
      stateB = !stateB;
    }
    
    if (stateB) {
      tempB |= (1 << bitPos);
    }

    // --- ĐỌC MẶT TRÁI (MUX 2) ---
    setMuxAddress(MUX2_LEFT_MAP[eye]);
    delayMicroseconds(10);
    
    analogRead(MUX2_SIG); // Đọc nháp xả tụ
    delayMicroseconds(2);
    int valL = analogRead(MUX2_SIG);
    rawAnalogL[eye] = valL; // Lưu giá trị Analog thô để Debug và Calib
    
    // So sánh với ngưỡng riêng biệt của chính mắt này
    bool stateL = (valL > lineThreshL[eye]);
    if (stateL) {
      tempL |= (1 << bitPos);
    }

    // --- ĐỌC MẶT PHẢI (MUX 2) ---
    setMuxAddress(MUX2_RIGHT_MAP[eye]);
    delayMicroseconds(10);
    
    analogRead(MUX2_SIG); // Đọc nháp xả tụ
    delayMicroseconds(2);
    int valR = analogRead(MUX2_SIG);
    rawAnalogR[eye] = valR; // Lưu giá trị Analog thô để Debug và Calib
    
    // So sánh với ngưỡng riêng biệt của chính mắt này
    bool stateR = (valR > lineThreshR[eye]);
    
    // SỬA LỖI VẬT LÝ: Mắt dò line cuối cùng (mắt số 8, eye == 7) của mạch RIGHT bị ngược phần cứng
    if (eye == 7) {
      stateR = !stateR;
    }
    
    if (stateR) {
      tempR |= (1 << bitPos);
    }
  }

  // Áp dụng bộ lọc đảo toàn bộ logic trạng thái nếu chạy vạch trắng nền đen
  if (lineInvert) {
    tempF = ~tempF;
    tempB = ~tempB;
    tempL = ~tempL;
    tempR = ~tempR;
  }

  // Ghi đè an toàn vào các biến trạng thái gửi lên ESP32
  lineDataF = tempF;
  lineDataB = tempB;
  lineDataL = tempL;
  lineDataR = tempR;
}

// ============================================================
//  HÀM IN KHỬ NHIỄU GIÁM SÁT TOÀN BỘ CẢM BIẾN LÊN TERMINAL MÁY TÍNH
//  Bổ sung in ra ngưỡng so sánh động [Th:xxx] của riêng mắt đó!
// ============================================================
void printSensorDebug() {
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint < 400) return; 
  lastPrint = millis();

  // Kết hợp xóa toàn màn hình và đưa con trỏ về gốc tọa độ (1;1) bằng mã tiêu chuẩn của VT100
  Serial.print(F("\033[2J\033[1;1H"));

  Serial.println(F("================= DATASHEET GIÁM SÁT 32 CẢM BIẾN DÒ LINE ĐỘC LẬP (115200 BAUD) ================="));
  
  // 1. In chi tiết Mặt Trước (Front - F1..F8)
  Serial.print(F("FRONT (Trái->Phải) | "));
  for (int i = 0; i < 8; i++) {
    Serial.print(F("F")); Serial.print(i + 1); Serial.print(F(":"));
    Serial.print(rawAnalogF[i]);
    // Kiểm tra xem bit thứ (7-i) có đang được kích hoạt hay không
    Serial.print((lineDataF & (1 << (7 - i))) ? F("(X)") : F("( )"));
    Serial.print(F("[Th:")); Serial.print(lineThreshF[i]); Serial.print(F("] "));
  }
  Serial.println(F("\033[K")); 

  // 2. In chi tiết Mặt Sau (Back - B1..B8)
  Serial.print(F("BACK  (Trái->Phải) | "));
  for (int i = 0; i < 8; i++) {
    Serial.print(F("B")); Serial.print(i + 1); Serial.print(F(":"));
    Serial.print(rawAnalogB[i]);
    Serial.print((lineDataB & (1 << (7 - i))) ? F("(X)") : F("( )"));
    Serial.print(F("[Th:")); Serial.print(lineThreshB[i]); Serial.print(F("] "));
  }
  Serial.println(F("\033[K"));

  // 3. In chi tiết Mặt Trái (Left - L1..L8)
  Serial.print(F("LEFT  (Trái->Phải) | "));
  for (int i = 0; i < 8; i++) {
    Serial.print(F("L")); Serial.print(i + 1); Serial.print(F(":"));
    Serial.print(rawAnalogL[i]);
    Serial.print((lineDataL & (1 << (7 - i))) ? F("(X)") : F("( )"));
    Serial.print(F("[Th:")); Serial.print(lineThreshL[i]); Serial.print(F("] "));
  }
  Serial.println(F("\033[K"));

  // 4. In chi tiết Mặt Phải (Right - R1..R8)
  Serial.print(F("RIGHT (Trái->Phải) | "));
  for (int i = 0; i < 8; i++) {
    Serial.print(F("R")); Serial.print(i + 1); Serial.print(F(":"));
    Serial.print(rawAnalogR[i]);
    Serial.print((lineDataR & (1 << (7 - i))) ? F("(X)") : F("( )"));
    Serial.print(F("[Th:")); Serial.print(lineThreshR[i]); Serial.print(F("] "));
  }
  Serial.println(F("\033[K"));
  
  // In thông tin bổ trợ
  Serial.print(F("-> Logic đảo (lineInvert): ")); Serial.print(lineInvert ? F("Bật (Vạch Trắng)") : F("Tắt (Vạch Đen)"));
  Serial.println(F("\033[K"));
  Serial.println(F("Ghi chú: (X)=Vạch màu, ( )=Nền, [Th:xxx]=Ngưỡng calib độc lập được áp dụng cho chính mắt đó.\033[K"));
  Serial.println(F("=========================================================================================="));
}

// ============================================================
//  TÍNH TOÁN VẬN TỐC THỰC TẾ (RPM CALCULATION)
// ============================================================
void calculateRPM() {
  static uint32_t last = 0;
  static long oldFL = 0, oldFR = 0, oldRL = 0, oldRR = 0;

  if (millis() - last < 100) return;

  long nowFL = encFL;
  long nowFR = encFR;
  long nowRL = encRL;
  long nowRR = encRR;

  // Tính RPM từ số lượng xung lệch trong chu kỳ 100ms
  rpmFL = (float)(nowFL - oldFL) * 600.0f / PPR;
  rpmFR = (float)(nowFR - oldFR) * 600.0f / PPR;
  rpmRL = (float)(nowRL - oldRL) * 600.0f / PPR;
  rpmRR = (float)(nowRR - oldRR) * 600.0f / PPR;

  oldFL = nowFL; oldFR = nowFR; oldRL = nowRL; oldRR = nowRR;
  last = millis();
}

// ============================================================
//  ĐIỀU KHIỂN TỐC ĐỘ VÒNG KÍN (PID PROCESS)
// ============================================================
void speedControl() {
  if (targetFL == 0 && targetFR == 0 && targetRL == 0 && targetRR == 0 &&
      abs(rpmFL) < 1.0 && abs(rpmFR) < 1.0 && abs(rpmRL) < 1.0 && abs(rpmRR) < 1.0) {
    pwmFL = 0; pwmFR = 0; pwmRL = 0; pwmRR = 0;
  } else {
    pwmFL += Kp * (targetFL - rpmFL);
    pwmFR += Kp * (targetFR - rpmFR);
    pwmRL += Kp * (targetRL - rpmRL);
    pwmRR += Kp * (targetRR - rpmRR);
  }

  pwmFL = constrain(pwmFL, -255, 255);
  pwmFR = constrain(pwmFR, -255, 255);
  pwmRL = constrain(pwmRL, -255, 255);
  pwmRR = constrain(pwmRR, -255, 255);

  setMotor(MOTOR_FL_A, MOTOR_FL_B, pwmFL);
  setMotor(MOTOR_FR_A, MOTOR_FR_B, pwmFR);
  setMotor(MOTOR_RL_A, MOTOR_RL_B, pwmRL);
  setMotor(MOTOR_RR_A, MOTOR_RR_B, pwmRR);
}

// ============================================================
//  HÀM TỰ ĐỘNG HIỆU CHUẨN 32 ĐƯỜNG NGƯỠNG ĐỘC LẬP
//  Tách biệt hoàn toàn tính toán trung vị cho từng bóng cảm biến
// ============================================================
void handleAutoCalib(bool isWhite) {
  scanAllLineSensors(); // Quét lấy dữ liệu Analog thô mới nhất của 32 mắt
  long sum = 0;

  for (int i = 0; i < 8; i++) {
    if (isWhite) {
      // Lưu lại giá trị nền trắng thực tế của từng mắt
      whiteCalibF[i] = rawAnalogF[i];
      whiteCalibB[i] = rawAnalogB[i];
      whiteCalibL[i] = rawAnalogL[i];
      whiteCalibR[i] = rawAnalogR[i];
    } else {
      // Lưu lại giá trị vạch đen thực tế của từng mắt
      blackCalibF[i] = rawAnalogF[i];
      blackCalibB[i] = rawAnalogB[i];
      blackCalibL[i] = rawAnalogL[i];
      blackCalibR[i] = rawAnalogR[i];
    }
    
    sum += rawAnalogF[i];
    sum += rawAnalogB[i];
    sum += rawAnalogL[i];
    sum += rawAnalogR[i];
  }

  // Nếu là bước đo vạch ĐEN (bước cuối của quá trình Calib), tiến hành tính toán và lưu EEPROM
  if (!isWhite) {
    for (int i = 0; i < 8; i++) {
      // Ngưỡng của từng mắt = (Trắng + Đen) / 2
      lineThreshF[i] = (whiteCalibF[i] + blackCalibF[i]) / 2;
      lineThreshB[i] = (whiteCalibB[i] + blackCalibB[i]) / 2;
      lineThreshL[i] = (whiteCalibL[i] + blackCalibL[i]) / 2;
      lineThreshR[i] = (whiteCalibR[i] + blackCalibR[i]) / 2;

      // Đặt giới hạn bảo vệ để chống nhiễu loạn giá trị do lọt sáng cực đoan
      lineThreshF[i] = constrain(lineThreshF[i], 150, 850);
      lineThreshB[i] = constrain(lineThreshB[i], 150, 850);
      lineThreshL[i] = constrain(lineThreshL[i], 150, 850);
      lineThreshR[i] = constrain(lineThreshR[i], 150, 850);
    }
    saveCalibration(); // Ghi vĩnh viễn 32 ngưỡng vào bộ nhớ EEPROM
  }

  int avg = sum / 32;
  if (isWhite) {
    Serial2.print("$CAL_WHITE,");
  } else {
    Serial2.print("$CAL_BLACK,");
  }
  Serial2.println(avg);
}

// ============================================================
//  HÀM XỬ LÝ LỆNH SAU KHI ĐƯỢC GIẢI MÃ HOÀN CHỈNH
// ============================================================
void executeCommand(String cmd) {
  // 1. Nhận lệnh di chuyển ($MOVE,vx,vy,w)
  if (cmd.startsWith("$MOVE,")) {
    int vx = 0, vy = 0, w = 0;
    sscanf(cmd.c_str(), "$MOVE,%d,%d,%d", &vx, &vy, &w);
    
    // Động học ngược Mecanum để quy đổi sang RPM từng bánh xe
    targetFL = (float)(vy + vx - w);
    targetFR = (float)(vy - vx + w);
    targetRL = (float)(vy - vx - w);
    targetRR = (float)(vy + vx + w);
  }
  // 2. Nhận lệnh dừng khẩn cấp ($STOP / $BRAKE)
  else if (cmd == "$STOP" || cmd == "$BRAKE") {
    targetFL = 0; targetFR = 0; targetRL = 0; targetRR = 0;
    pwmFL = 0; pwmFR = 0; pwmRL = 0; pwmRR = 0;
    setMotor(MOTOR_FL_A, MOTOR_FL_B, 0);
    setMotor(MOTOR_FR_A, MOTOR_FR_B, 0);
    setMotor(MOTOR_RL_A, MOTOR_RL_B, 0);
    setMotor(MOTOR_RR_A, MOTOR_RR_B, 0);
  }
  // 3. Nhận lệnh cấu hình PID bám động cơ ($SET_PID,kp,ki,kd)
  else if (cmd.startsWith("$SET_PID,")) {
    float p = 0.0, i = 0.0, d = 0.0;
    sscanf(cmd.c_str(), "$SET_PID,%f,%f,%f", &p, &i, &d);
    if (p > 0.0) Kp = p;
  }
  // 4. Nhận cấu hình ngưỡng bám vạch dò line ($SET_LINE,kp,kd,thresh)
  else if (cmd.startsWith("$SET_LINE,")) {
    float p = 0.0, d = 0.0;
    int t = 500;
    sscanf(cmd.c_str(), "$SET_LINE,%f,%f,%d", &p, &d, &t);
    lineThresh = t; 
    
    // Ghi đè toàn bộ mảng ngưỡng độc lập bằng giá trị mới để đảm bảo kiểm soát thủ công đồng bộ
    for (int i = 0; i < 8; i++) {
      lineThreshF[i] = t;
      lineThreshB[i] = t;
      lineThreshL[i] = t;
      lineThreshR[i] = t;
    }
    saveCalibration(); 
  }
  // 5. Nhận lệnh Calib vạch màu Đen từ ESP32 Master (Nhị phân cứng)
  else if (cmd == "$CALIB_BLACK") {
    lineInvert = false; // Logic chuẩn: Phát hiện vạch đen trên nền trắng
    Serial.println("[CALIB] Configured for BLACK line detection.");
  }
  // 6. Nhận lệnh Calib vạch màu Trắng từ ESP32 Master (Nhị phân cứng)
  else if (cmd == "$CALIB_WHITE") {
    lineInvert = true;  // Nghịch đảo logic: Phát hiện vạch trắng trên nền đen
    Serial.println("[CALIB] Configured for WHITE line detection.");
  }
  // 7. Nhận lệnh kích hoạt Calib nền TRẮNG tự động (Đo ADC và tính toán)
  else if (cmd == "$CAL_WHITE") {
    handleAutoCalib(true);
  }
  // 8. Nhận lệnh kích hoạt Calib vạch ĐEN tự động (Đo ADC và tính toán)
  else if (cmd == "$CAL_BLACK") {
    handleAutoCalib(false);
  }
}

// ============================================================
//  GIAO TIẾP UART: GIẢI MÃ LỆNH TỪ ESP32 (KHÔNG CHẶN)
// ============================================================
void receiveCommand() {
  static String cmdBuffer = "";
  
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    
    // Nếu gặp ký tự kết thúc gói tin
    if (c == '\n') {
      cmdBuffer.trim();
      if (cmdBuffer.length() > 0) {
        executeCommand(cmdBuffer); // Tiến hành thực thi lệnh
      }
      cmdBuffer = ""; // Reset bộ đệm nhận
    } 
    // Ghép ký tự thô vào bộ đệm (bỏ qua ký tự về đầu dòng '\r')
    else if (c != '\r') {
      cmdBuffer += c;
      
      // Giới hạn độ dài để chống tràn bộ nhớ SRAM khi có dòng nhiễu dội liên tục vào RX2
      if (cmdBuffer.length() > 64) {
        cmdBuffer = "";
      }
    }
  }
}

// ============================================================
//  GỬI PHẢN HỒI CHO ESP32 MASTER (TRÁNH XUNG ĐỘT BUFFER)
//  Sử dụng cơ chế Staggered Telemetry để chia nhỏ chu kỳ truyền nhận
// ============================================================
void sendTelemetry() {
  static uint32_t lastTx = 0;
  static uint8_t phase = 0;

  // Gửi thông tin luân phiên mỗi 33ms (Đảm bảo mỗi thông số cập nhật tần số ~10Hz)
  if (millis() - lastTx < 33) return;
  lastTx = millis();

  switch (phase) {
    case 0:
      // Gửi trạng thái vận tốc thực tế ($STAT,spdFL,spdFR,spdRL,spdRR)
      // Nhân hệ số 10 để tương thích với bộ giải mã parseMegaData của ESP32
      Serial2.print("$STAT,");
      Serial2.print((int)(rpmFL * 10.0f)); Serial2.print(",");
      Serial2.print((int)(rpmFR * 10.0f)); Serial2.print(",");
      Serial2.print((int)(rpmRL * 10.0f)); Serial2.print(",");
      Serial2.print((int)(rpmRR * 10.0f));
      Serial2.println();
      phase = 1;
      break;

    case 1:
      // Gửi dữ liệu nhị phân của cảm biến dò line ($LINE,lf,lb,ll,lr)
      Serial2.print("$LINE,");
      Serial2.print(lineDataF); Serial2.print(",");
      Serial2.print(lineDataB); Serial2.print(",");
      Serial2.print(lineDataL); Serial2.print(",");
      Serial2.print(lineDataR);
      Serial2.println();
      phase = 2;
      break;

    case 2:
      // Gửi số lượng xung thô của 4 bộ Encoders ($ENC,e1,e2,e3,e4)
      Serial2.print("$ENC,");
      Serial2.print(encFL); Serial2.print(",");
      Serial2.print(encFR); Serial2.print(",");
      Serial2.print(encRL); Serial2.print(",");
      Serial2.print(encRR);
      Serial2.println();
      phase = 0;
      break;
  }
}

// ============================================================
//  HÀM SETUP KHỞI TẠO MẠCH
// ============================================================
void setup() {
  // Serial giám sát qua cáp USB máy tính
  Serial.begin(115200);

  // Gửi chuỗi lệnh escape ANSI xóa sạch màn hình và đưa con trỏ về góc trái ngay khi khởi động
  Serial.print(F("\033[2J\033[H"));
  
  // Nạp 32 ngưỡng dò line độc lập đã được lưu từ EEPROM
  loadCalibration();

  // Serial2 kết nối truyền thông trực tiếp tới ESP32 Master
  Serial2.begin(115200);

  // Khởi tạo các chân Driver BTS7960
  pinMode(MOTOR_FL_A, OUTPUT); pinMode(MOTOR_FL_B, OUTPUT);
  pinMode(MOTOR_FR_A, OUTPUT); pinMode(MOTOR_FR_B, OUTPUT);
  pinMode(MOTOR_RL_A, OUTPUT); pinMode(MOTOR_RL_B, OUTPUT);
  pinMode(MOTOR_RR_A, OUTPUT); pinMode(MOTOR_RR_B, OUTPUT);

  // Tắt động cơ tạm thời để bảo vệ robot lúc vừa cắm điện
  setMotor(MOTOR_FL_A, MOTOR_FL_B, 0);
  setMotor(MOTOR_FR_A, MOTOR_FR_B, 0);
  setMotor(MOTOR_RL_A, MOTOR_RL_B, 0);
  setMotor(MOTOR_RR_A, MOTOR_RR_B, 0);

  // Khởi tạo các chân đọc Encoders (Trở kéo lên nội)
  pinMode(ENC_FL_A, INPUT_PULLUP); pinMode(ENC_FL_B, INPUT_PULLUP);
  pinMode(ENC_FR_A, INPUT_PULLUP); pinMode(ENC_FR_B, INPUT_PULLUP);
  pinMode(ENC_RL_A, INPUT_PULLUP); pinMode(ENC_RL_B, INPUT_PULLUP);
  pinMode(ENC_RR_A, INPUT_PULLUP); pinMode(ENC_RR_B, INPUT_PULLUP);

  // Thiết lập ngắt CHANGE cho kênh Phase B của cả 4 bộ Encoders
  attachInterrupt(digitalPinToInterrupt(ENC_FL_B), ISR_FL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_FR_B), ISR_FR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RL_B), ISR_RL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RR_B), ISR_RR, CHANGE);

  // Cấu hình các chân điều khiển và kích hoạt 2 bộ Analog Multiplexer CD74HC4067
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  
  pinMode(MUX1_EN, OUTPUT);
  pinMode(MUX2_EN, OUTPUT);

  // Đặt mặc định hai mạch Mux hoạt động liên tục (Active LOW)
  digitalWrite(MUX1_EN, LOW);
  digitalWrite(MUX2_EN, LOW);

  Serial.println("[MEGA SLAVE] Setup Complete. 32-ch Multiplexed Line active.");
}

// ============================================================
//  VÒNG LẶP CHÍNH (MAIN LOOP)
// ============================================================
void loop() {
  receiveCommand();       // Giải mã các tập lệnh nhận từ ESP32 Master
  scanAllLineSensors();   // Quét nhanh trạng thái 32 mắt dò line qua Analog Multiplexers
  calculateRPM();         // Tính toán tốc độ thực tế định kỳ 100ms
  speedControl();         // PID vòng kín ổn định tốc độ bánh xe
  sendTelemetry();        // Truyền luân phiên dữ liệu $STAT, $LINE, $ENC về ESP32
  printSensorDebug();     // In bảng giám sát 32 mắt định danh lên Serial Monitor
}