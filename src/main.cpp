/*
 * ============================================================
 * ARDUINO MEGA 2560 - SLAVE MECANUM & LINE SENSOR CONTROLLER
 * NQ Robotics Architecture 2026
 * ============================================================
 * Chức năng:
 *   - Điều khiển 4 động cơ Mecanum bằng Driver BTS7960
 *   - Đếm xung Encoder bánh xe thô (Sử dụng ngắt phần cứng tốc độ cao)
 *   - Đọc hệ thống cảm biến dò line 8 mắt x 4 mặt (32 kênh) qua 2 mạch Analog Mux 16-ch
 *   - Đóng gói dữ liệu Encoders + Line gửi liên tục lên ESP32 Master bằng gói tin $TELE mỗi 15ms
 *   - Nhận điện áp PWM trực tiếp $PWM từ ESP32 Master phát xuống để xuất thẳng ra các Driver
 *   - Tích hợp điều khiển hệ thống 16 cổng Relay mở rộng thô ($RELAY)
 * ============================================================
 */

#include <Arduino.h>
#include <EEPROM.h>

// ============================================================
//  ĐỊNH NGHĨA CHÂN ĐIỀU KHIỂN ĐỘNG CƠ (DRIVERS BTS7960)
// ============================================================
#define MOTOR_FL_A  3
#define MOTOR_FL_B  4

#define MOTOR_FR_A  6
#define MOTOR_FR_B  5

#define MOTOR_RL_A  7
#define MOTOR_RL_B  8

#define MOTOR_RR_A  10
#define MOTOR_RR_B  9

// ============================================================
//  ĐỊNH NGHĨA CHÂN ĐỌC ENCODER BÁNH XE
// ============================================================
#define ENC_M1_A    22
#define ENC_M1_B    18  // INT.5
#define ENC_M2_A    23
#define ENC_M2_B    19  // INT.4
#define ENC_M3_A    24
#define ENC_M3_B    20  // INT.3
#define ENC_M4_A    25
#define ENC_M4_B    21  // INT.2

// ============================================================
//  ĐỊNH NGHĨA CHÂN ANALOG MUX CHỌN KÊNH 32 MẮT DÒ LINE
// ============================================================
#define MUX1_SIG   A14
#define MUX2_SIG   A15
#define MUX1_EN    48
#define MUX2_EN    49

#define MUX_S0     42
#define MUX_S1     43
#define MUX_S2     45
#define MUX_S3     46

// ============================================================
//  ĐỊNH NGHĨA 16 CỔNG ĐẦU RA RELAY ĐIỀU KHIỂN THIẾT BỊ NGOẠI VI
// ============================================================
const uint8_t RELAY_PINS[16] = {
  26, 28, 30, 32, 34, 36, 38, 40, // Cụm 1: Chỉ số 0 -> 7 (chân Chẵn)
  27, 29, 31, 33, 35, 37, 39, 41  // Cụm 2: Chỉ số 8 -> 15 (chân Lẻ)
};

// Mảng ánh xạ kênh Mux (0 -> 15) cho từng thanh cảm biến 8 mắt dò line
const uint8_t MUX1_FRONT_MAP[8] = {0, 1, 2, 3, 4, 5, 6, 7};
const uint8_t MUX1_BACK_MAP[8]  = {8, 9, 10, 11, 12, 13, 14, 15};
const uint8_t MUX2_LEFT_MAP[8]  = {0, 1, 2, 3, 4, 5, 6, 7};
const uint8_t MUX2_RIGHT_MAP[8] = {8, 9, 10, 11, 12, 13, 14, 15};

// Cấu hình lưu trữ EEPROM cho 32 ngưỡng độc lập
#define EEPROM_MAGIC 0x55
#define ADDR_MAGIC   0
#define ADDR_F       2
#define ADDR_B       18
#define ADDR_L       34
#define ADDR_R       50

// ============================================================
//  CÁC BIẾN TRẠNG THÁI (STATE VARIABLES)
// ============================================================
volatile long encFL = 0;
volatile long encFR = 0;
volatile long encRL = 0;
volatile long encRR = 0;

// Biến lưu trữ nhị phân 8-bit của cảm biến dò line
uint8_t lineDataF = 0;
uint8_t lineDataB = 0;
uint8_t lineDataL = 0;
uint8_t lineDataR = 0;

// Biến lưu trữ giá trị Analog thực tế để phục vụ in DEBUG lên máy tính
int rawAnalogF[8] = {0};
int rawAnalogB[8] = {0};
int rawAnalogL[8] = {0};
int rawAnalogR[8] = {0};

// Mảng chứa 32 ngưỡng so sánh độc lập
int lineThreshF[8] = {500, 500, 500, 500, 500, 500, 500, 500};
int lineThreshB[8] = {500, 500, 500, 500, 500, 500, 500, 500};
int lineThreshL[8] = {500, 500, 500, 500, 500, 500, 500, 500};
int lineThreshR[8] = {500, 500, 500, 500, 500, 500, 500, 500};

int whiteCalibF[8] = {100}; int whiteCalibB[8] = {100}; int whiteCalibL[8] = {100}; int whiteCalibR[8] = {100};
int blackCalibF[8] = {900}; int blackCalibB[8] = {900}; int blackCalibL[8] = {900}; int blackCalibR[8] = {900};

bool lineInvert = false;

// Đọc giá trị Encoder an toàn chống vỡ bit dữ liệu trên MCU 8-bit (Atomic Read)
long safeReadEncFL() { long val; noInterrupts(); val = encFL; interrupts(); return val; }
long safeReadEncFR() { long val; noInterrupts(); val = encFR; interrupts(); return val; }
long safeReadEncRL() { long val; noInterrupts(); val = encRL; interrupts(); return val; }
long safeReadEncRR() { long val; noInterrupts(); val = encRR; interrupts(); return val; }

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
      lineThreshF[i] = 500; lineThreshB[i] = 500;
      lineThreshL[i] = 500; lineThreshR[i] = 500;
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
  if (digitalRead(ENC_M1_A) == digitalRead(ENC_M1_B)) encFL++;
  else encFL--;
}

void ISR_FR() {
  if (digitalRead(ENC_M2_A) == digitalRead(ENC_M2_B)) encFR++;
  else encFR--;
}

void ISR_RL() {
  if (digitalRead(ENC_M3_A) == digitalRead(ENC_M3_B)) encRL++;
  else encRL--;
}

void ISR_RR() {
  if (digitalRead(ENC_M4_A) == digitalRead(ENC_M4_B)) encRR++;
  else encRR--;
}

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

void setMuxAddress(uint8_t channel) {
  digitalWrite(MUX_S0, (channel >> 0) & 1);
  digitalWrite(MUX_S1, (channel >> 1) & 1);
  digitalWrite(MUX_S2, (channel >> 2) & 1);
  digitalWrite(MUX_S3, (channel >> 3) & 1);
}

void scanAllLineSensors() {
  digitalWrite(MUX1_EN, LOW);
  digitalWrite(MUX2_EN, LOW);

  uint8_t tempF = 0; uint8_t tempB = 0;
  uint8_t tempL = 0; uint8_t tempR = 0;

  for (uint8_t eye = 0; eye < 8; eye++) {
    uint8_t bitPos = 7 - eye;

    // --- ĐỌC MẶT TRƯỚC (MUX 1) ---
    setMuxAddress(MUX1_FRONT_MAP[eye]);
    delayMicroseconds(10); 
    analogRead(MUX1_SIG); // Đọc nháp xả tụ
    delayMicroseconds(2);
    int valF = analogRead(MUX1_SIG); 
    rawAnalogF[eye] = valF; 
    bool stateF = (valF > lineThreshF[eye]);
    if (stateF) tempF |= (1 << bitPos);

    // --- ĐỌC MẶT SAU (MUX 1) ---
    setMuxAddress(MUX1_BACK_MAP[eye]);
    delayMicroseconds(10);
    analogRead(MUX1_SIG); 
    delayMicroseconds(2);
    int valB = analogRead(MUX1_SIG);
    rawAnalogB[eye] = valB; 
    bool stateB = (valB > lineThreshB[eye]);
    if (eye == 7) stateB = !stateB; // Sửa lỗi ngược mắt số 8 mặt sau
    if (stateB) tempB |= (1 << bitPos);

    // --- ĐỌC MẶT TRÁI (MUX 2) ---
    setMuxAddress(MUX2_LEFT_MAP[eye]);
    delayMicroseconds(10);
    analogRead(MUX2_SIG); 
    delayMicroseconds(2);
    int valL = analogRead(MUX2_SIG);
    rawAnalogL[eye] = valL; 
    bool stateL = (valL > lineThreshL[eye]);
    if (stateL) tempL |= (1 << bitPos);

    // --- ĐỌC MẶT PHẢI (MUX 2) ---
    setMuxAddress(MUX2_RIGHT_MAP[eye]);
    delayMicroseconds(10);
    analogRead(MUX2_SIG); 
    delayMicroseconds(2);
    int valR = analogRead(MUX2_SIG);
    rawAnalogR[eye] = valR; 
    bool stateR = (valR > lineThreshR[eye]);
    if (eye == 7) stateR = !stateR; // Sửa lỗi ngược mắt số 8 mặt phải
    if (stateR) tempR |= (1 << bitPos);
  }

  if (lineInvert) {
    tempF = ~tempF; tempB = ~tempB;
    tempL = ~tempL; tempR = ~tempR;
  }

  lineDataF = tempF; lineDataB = tempB;
  lineDataL = tempL; lineDataR = tempR;
}

void printSensorDebug() {
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint < 400) return;
  lastPrint = millis();

  Serial.print(F("\033[2J\033[1;1H"));
  Serial.println(F("================= DATASHEET GIÁM SÁT 32 CẢM BIẾN DÒ LINE ĐỘC LẬP (115200 BAUD) ================="));

  Serial.print(F("FRONT | "));
  for (int i = 0; i < 8; i++) {
    Serial.print(F("F")); Serial.print(i + 1); Serial.print(F(":"));
    Serial.print(rawAnalogF[i]);
    Serial.print((lineDataF & (1 << (7 - i))) ? F("(X)") : F("( )"));
    Serial.print(F("[Th:")); Serial.print(lineThreshF[i]); Serial.print(F("] "));
  }
  Serial.println();

  Serial.print(F("BACK  | "));
  for (int i = 0; i < 8; i++) {
    Serial.print(F("B")); Serial.print(i + 1); Serial.print(F(":"));
    Serial.print(rawAnalogB[i]);
    Serial.print((lineDataB & (1 << (7 - i))) ? F("(X)") : F("( )"));
    Serial.print(F("[Th:")); Serial.print(lineThreshB[i]); Serial.print(F("] "));
  }
  Serial.println();

  Serial.print(F("LEFT  | "));
  for (int i = 0; i < 8; i++) {
    Serial.print(F("L")); Serial.print(i + 1); Serial.print(F(":"));
    Serial.print(rawAnalogL[i]);
    Serial.print((lineDataL & (1 << (7 - i))) ? F("(X)") : F("( )"));
    Serial.print(F("[Th:")); Serial.print(lineThreshL[i]); Serial.print(F("] "));
  }
  Serial.println();

  Serial.print(F("RIGHT | "));
  for (int i = 0; i < 8; i++) {
    Serial.print(F("R")); Serial.print(i + 1); Serial.print(F(":"));
    Serial.print(rawAnalogR[i]);
    Serial.print((lineDataR & (1 << (7 - i))) ? F("(X)") : F("( )"));
    Serial.print(F("[Th:")); Serial.print(lineThreshR[i]); Serial.print(F("] "));
  }
  Serial.println();

  Serial.print(F("-> Logic đảo (lineInvert): ")); Serial.println(lineInvert ? F("Vạch Trắng") : F("Vạch Đen"));
  Serial.println(F("=========================================================================================="));
}

void handleAutoCalib(bool isWhite) {
  scanAllLineSensors();
  long sum = 0;

  for (int i = 0; i < 8; i++) {
    if (isWhite) {
      whiteCalibF[i] = rawAnalogF[i]; whiteCalibB[i] = rawAnalogB[i];
      whiteCalibL[i] = rawAnalogL[i]; whiteCalibR[i] = rawAnalogR[i];
    } else {
      blackCalibF[i] = rawAnalogF[i]; blackCalibB[i] = rawAnalogB[i];
      blackCalibL[i] = rawAnalogL[i]; blackCalibR[i] = rawAnalogR[i];
    }
    sum += rawAnalogF[i]; sum += rawAnalogB[i];
    sum += rawAnalogL[i]; sum += rawAnalogR[i];
  }

  if (!isWhite) {
    for (int i = 0; i < 8; i++) {
      lineThreshF[i] = (whiteCalibF[i] + blackCalibF[i]) / 2;
      lineThreshB[i] = (whiteCalibB[i] + blackCalibB[i]) / 2;
      lineThreshL[i] = (whiteCalibL[i] + blackCalibL[i]) / 2;
      lineThreshR[i] = (whiteCalibR[i] + blackCalibR[i]) / 2;

      lineThreshF[i] = constrain(lineThreshF[i], 150, 850);
      lineThreshB[i] = constrain(lineThreshB[i], 150, 850);
      lineThreshL[i] = constrain(lineThreshL[i], 150, 850);
      lineThreshR[i] = constrain(lineThreshR[i], 150, 850);
    }
    saveCalibration(); 
  }

  int avg = sum / 32;
  if (isWhite) Serial2.print(F("$CAL_WHITE,"));
  else Serial2.print(F("$CAL_BLACK,"));
  Serial2.println(avg);
}

// ============================================================
//  HÀM XỬ LÝ LỆNH SAU KHI ĐƯỢC GIẢI MÃ HOÀN CHỈNH TỪ MASTER ESP32
// ============================================================
void executeCommand(String cmd) {
  // 1. Tiếp nhận lệnh phát điện áp trực tiếp xuống Driver BTS7960
  if (cmd.startsWith(F("$PWM,"))) {
    int fl = 0, fr = 0, rl = 0, rr = 0;
    if (sscanf(cmd.c_str(), "$PWM,%d,%d,%d,%d", &fl, &fr, &rl, &rr) == 4) {
      setMotor(MOTOR_FL_A, MOTOR_FL_B, fl);
      setMotor(MOTOR_FR_A, MOTOR_FR_B, fr);
      setMotor(MOTOR_RL_A, MOTOR_RL_B, rl);
      setMotor(MOTOR_RR_A, MOTOR_RR_B, rr);
    }
  }
  // 2. Tiếp nhận phanh dừng khẩn cấp
  else if (cmd == F("$STOP") || cmd == F("$BRAKE")) {
    setMotor(MOTOR_FL_A, MOTOR_FL_B, 0);
    setMotor(MOTOR_FR_A, MOTOR_FR_B, 0);
    setMotor(MOTOR_RL_A, MOTOR_RL_B, 0);
    setMotor(MOTOR_RR_A, MOTOR_RR_B, 0);
  }
  // 3. Nhận lệnh kích hoạt Relay ngoại vi thô
  else if (cmd.startsWith(F("$RELAY,"))) {
    int channel = 0;
    int state = 0;
    if (sscanf(cmd.c_str(), "$RELAY,%d,%d", &channel, &state) == 2) {
      if (channel >= 0 && channel < 16) {
        digitalWrite(RELAY_PINS[channel], state ? HIGH : LOW);
        Serial.print(F("[RELAY] Channel ")); Serial.print(channel);
        Serial.print(F(" -> pin ")); Serial.print(RELAY_PINS[channel]);
        Serial.println(state ? F(" (ON)") : F(" (OFF)"));
      }
    }
  }
  // 4. Nhận cấu hình ngưỡng dò line đồng bộ
  else if (cmd.startsWith(F("$SET_LINE,"))) {
    float dummy1 = 0, dummy2 = 0;
    int t = 500;
    if (sscanf(cmd.c_str(), "$SET_LINE,%f,%f,%d", &dummy1, &dummy2, &t) == 3) {
      for (int i = 0; i < 8; i++) {
        lineThreshF[i] = t; lineThreshB[i] = t;
        lineThreshL[i] = t; lineThreshR[i] = t;
      }
      saveCalibration();
    }
  }
  // 5. Nhận lệnh đảo logic màu vạch
  else if (cmd == F("$CALIB_BLACK")) {
    lineInvert = false;
  } else if (cmd == F("$CALIB_WHITE")) {
    lineInvert = true;
  }
  // 6. Kích hoạt hiệu chuẩn ADC
  else if (cmd == F("$CAL_WHITE")) {
    handleAutoCalib(true);
  } else if (cmd == F("$CAL_BLACK")) {
    handleAutoCalib(false);
  }
}

void receiveCommand() {
  static String cmdBuffer = "";
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    if (c == '\n') {
      cmdBuffer.trim();
      if (cmdBuffer.length() > 0) executeCommand(cmdBuffer);
      cmdBuffer = "";
    } else if (c != '\r') {
      cmdBuffer += c;
      if (cmdBuffer.length() > 64) cmdBuffer = "";
    }
  }
}

// Bắn dữ liệu telemetry thô nén lên ESP32 Master định kỳ 15ms cực nhạy
void sendTelemetry() {
  static uint32_t lastTx = 0;
  if (millis() - lastTx < 15) return;
  lastTx = millis();

  Serial2.print(F("$TELE,"));
  Serial2.print(safeReadEncFL()); Serial2.print(F(","));
  Serial2.print(safeReadEncFR()); Serial2.print(F(","));
  Serial2.print(safeReadEncRL()); Serial2.print(F(","));
  Serial2.print(safeReadEncRR()); Serial2.print(F(","));
  Serial2.print(lineDataF); Serial2.print(F(","));
  Serial2.print(lineDataB); Serial2.print(F(","));
  Serial2.print(lineDataL); Serial2.print(F(","));
  Serial2.print(lineDataR);
  Serial2.println();
}

// ============================================================
//  HÀM SETUP KHỞI TẠO MẠCH
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.print(F("\033[2J\033[H"));

  loadCalibration();
  Serial2.begin(115200);

  // Khởi động các cổng Relay đầu ra
  for (int i = 0; i < 16; i++) {
    pinMode(RELAY_PINS[i], OUTPUT);
    digitalWrite(RELAY_PINS[i], LOW); // Mặc định tắt an toàn relay
  }

  // Khởi tạo các chân Driver BTS7960
  pinMode(MOTOR_FL_A, OUTPUT); pinMode(MOTOR_FL_B, OUTPUT);
  pinMode(MOTOR_FR_A, OUTPUT); pinMode(MOTOR_FR_B, OUTPUT);
  pinMode(MOTOR_RL_A, OUTPUT); pinMode(MOTOR_RL_B, OUTPUT);
  pinMode(MOTOR_RR_A, OUTPUT); pinMode(MOTOR_RR_B, OUTPUT);

  setMotor(MOTOR_FL_A, MOTOR_FL_B, 0); setMotor(MOTOR_FR_A, MOTOR_FR_B, 0);
  setMotor(MOTOR_RL_A, MOTOR_RL_B, 0); setMotor(MOTOR_RR_A, MOTOR_RR_B, 0);

  // Khởi tạo các chân đọc Encoders thô bằng ngắt
  pinMode(ENC_M1_A, INPUT_PULLUP); pinMode(ENC_M1_B, INPUT_PULLUP);
  pinMode(ENC_M2_A, INPUT_PULLUP); pinMode(ENC_M2_B, INPUT_PULLUP);
  pinMode(ENC_M3_A, INPUT_PULLUP); pinMode(ENC_M3_B, INPUT_PULLUP);
  pinMode(ENC_M4_A, INPUT_PULLUP); pinMode(ENC_M4_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_M1_B), ISR_FL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_M2_B), ISR_FR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_M3_B), ISR_RL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_M4_B), ISR_RR, CHANGE);

  // Cấu hình mạch Analog Mux đọc quang cảm biến
  pinMode(MUX_S0, OUTPUT); pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT); pinMode(MUX_S3, OUTPUT);
  pinMode(MUX1_EN, OUTPUT); pinMode(MUX2_EN, OUTPUT);

  digitalWrite(MUX1_EN, LOW);
  digitalWrite(MUX2_EN, LOW);

  Serial.println(F("[MEGA SLAVE] Setup Complete. Dumb Slave mode initialized."));
}

// ============================================================
//  VÒNG LẶP CHÍNH (MAIN LOOP)
// ============================================================
void loop() {
  receiveCommand();
  scanAllLineSensors();
  sendTelemetry();
  printSensorDebug();
}
