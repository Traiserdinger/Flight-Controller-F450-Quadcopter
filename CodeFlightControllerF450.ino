#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_BMP5xx.h>

#define ESC1 39 // FR
#define ESC2 40 // RR 
#define ESC3 21 // RL 
#define ESC4 18 // FL 

#define SCK_PIN 3
#define MOSI_PIN 1
#define MISO_PIN 2
#define CS_PIN 14
#define IBUS_RX_PIN 5

uint16_t ibus[14];
uint8_t buffer[32];

// Đổi tên Object thành chuẩn của thư viện mới
Adafruit_BMP5xx bmp;

float roll = 0, pitch = 0;
float gx = 0, gy = 0, gz = 0;
float gx_prev = 0, gy_prev = 0;

float cal_gx = 0, cal_gy = 0, cal_gz = 0;
float cal_ax = 0, cal_ay = 0;
unsigned long lastT = 0;

// PID Cân bằng góc
float Kp_angle = 4.5;    
float Kp_rate = 1.1;     
float Ki_rate = 0.8;     
float Kd_rate = 0.015;   
float Kp_yaw = 3.5;      

float i_r = 0, i_p = 0;
float d_r_filtered = 0, d_p_filtered = 0;

// ==========================================
// PID GIỮ ĐỘ CAO (ALTITUDE HOLD)
// ==========================================
float Kp_alt = 2.0;
float Kd_alt = 1.5;
float current_alt = 0, target_alt = 0, prev_alt = 0;
bool is_alt_hold = false;
int hover_throttle = 1500; // Mức ga lơ lửng ước tính

uint32_t duty(int us) { return (us * 16383) / 4000; }

void writeM(int pin, int ch, int us) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty(us));
#else
  ledcWrite(ch, duty(us));
#endif
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, IBUS_RX_PIN, -1);

  pinMode(ESC1, OUTPUT); pinMode(ESC2, OUTPUT);
  pinMode(ESC3, OUTPUT); pinMode(ESC4, OUTPUT);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(ESC1, 250, 14); ledcAttach(ESC2, 250, 14);
  ledcAttach(ESC3, 250, 14); ledcAttach(ESC4, 250, 14);
#else
  ledcSetup(0, 250, 14); ledcAttachPin(ESC1, 0); 
  ledcSetup(1, 250, 14); ledcAttachPin(ESC2, 1);
  ledcSetup(2, 250, 14); ledcAttachPin(ESC3, 2); 
  ledcSetup(3, 250, 14); ledcAttachPin(ESC4, 3);
#endif

  writeM(ESC1, 0, 1000); writeM(ESC2, 1, 1000);
  writeM(ESC3, 2, 1000); writeM(ESC4, 3, 1000);
  Wire.begin(19, 20);
  
  if (!bmp.begin(0x47, &Wire) && !bmp.begin(0x46, &Wire)) {
    Serial.println("Canh bao: Khong tim thay BMP580!");
  } else {
    bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_128X);
    bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_127);
    bmp.setOutputDataRate(BMP5XX_ODR_50_HZ);
    Serial.println("BMP580 OK!");
  }

  // Khởi tạo ICM20948 
  pinMode(CS_PIN, OUTPUT); digitalWrite(CS_PIN, HIGH);
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(0x06); SPI.transfer(0x01); 
  digitalWrite(CS_PIN, HIGH);
  delay(50);

  digitalWrite(CS_PIN, LOW); SPI.transfer(0x7F); SPI.transfer(0x20); digitalWrite(CS_PIN, HIGH); delay(10);
  digitalWrite(CS_PIN, LOW); SPI.transfer(0x01); SPI.transfer(0x29); digitalWrite(CS_PIN, HIGH); delay(10);
  digitalWrite(CS_PIN, LOW); SPI.transfer(0x14); SPI.transfer(0x29); digitalWrite(CS_PIN, HIGH); delay(10);
  digitalWrite(CS_PIN, LOW); SPI.transfer(0x7F); SPI.transfer(0x00); digitalWrite(CS_PIN, HIGH); delay(10);
  SPI.endTransaction();

  // CALIB IMU (NHỚ ĐỂ MÁY BAY NẰM IM)
  long sum_gx = 0, sum_gy = 0, sum_gz = 0, sum_ax = 0, sum_ay = 0;
  for (int i = 0; i < 1000; i++) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(0x2D | 0x80);
    int16_t axR = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
    int16_t ayR = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
    int16_t azR = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
    int16_t gxR = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
    int16_t gyR = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
    int16_t gzR = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();

    sum_gx += gxR; sum_gy += gyR; sum_gz += gzR;
    sum_ax += axR; sum_ay += ayR;
    delay(2);
  }
  cal_gx = (float)sum_gx / 1000.0; cal_gy = (float)sum_gy / 1000.0; cal_gz = (float)sum_gz / 1000.0;
  cal_ax = (float)sum_ax / 1000.0; cal_ay = (float)sum_ay / 1000.0;
  
  delay(2000); 
  lastT = micros();
}

void loop() {
  while (Serial2.available()) {
    static uint8_t idx = 0;
    uint8_t c = Serial2.read();
    if (idx == 0 && c != 0x20) continue;
    if (idx == 1 && c != 0x40) { idx = 0; continue; }
    buffer[idx++] = c;
    if (idx == 32) {
      idx = 0;
      uint16_t chk = 0xFFFF;
      for (int i = 0; i < 30; i++) chk -= buffer[i];
      if (chk == (buffer[30] | (buffer[31] << 8))) {
        for (int i = 0; i < 14; i++) ibus[i] = buffer[2 + i*2] | (buffer[3 + i*2] << 8);
      }
    }
  }

  static unsigned long lastBaroT = 0;
  if (micros() - lastBaroT > 20000) {
    float new_alt = bmp.readAltitude(1013.25);
    //Low-Pass Filter
    current_alt = 0.8 * current_alt + 0.2 * new_alt;
    lastBaroT = micros();
  }

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(0x2D | 0x80);
  int16_t axR = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  int16_t ayR = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  int16_t azR = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  int16_t gxR = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  int16_t gyR = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  int16_t gzR = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();

  unsigned long curr = micros();
  float dt = (curr - lastT) / 1000000.0;
  lastT = curr;

  float ax_raw = (axR - cal_ax) / 16384.0; 
  float ay_raw = (ayR - cal_ay) / 16384.0; 
  float az_raw = azR / 16384.0; 
  float gx_raw = (gxR - cal_gx) / 131.0; 
  float gy_raw = (gyR - cal_gy) / 131.0;
  float gz_raw = (gzR - cal_gz) / 131.0;

  static float ax_f = 0, ay_f = 0, az_f = 1.0;
  ax_f = 0.9 * ax_f + 0.1 * ax_raw; ay_f = 0.9 * ay_f + 0.1 * ay_raw; az_f = 0.9 * az_f + 0.1 * az_raw;

  static float gx_f = 0, gy_f = 0, gz_f = 0;
  gx_f = 0.85 * gx_f + 0.15 * gx_raw; gy_f = 0.85 * gy_f + 0.15 * gy_raw; gz_f = 0.85 * gz_f + 0.15 * gz_raw;

  gx = gx_f; gy = gy_f; gz = gz_f; 

  float r_acc = atan2(ay_f, az_f) * 180.0 / PI;
  float p_acc = atan2(-ax_f, sqrt(ay_f * ay_f + az_f * az_f)) * 180.0 / PI;
  
  roll = 0.98 * (roll + gx * dt) + 0.02 * r_acc;
  pitch = 0.98 * (pitch + gy * dt) + 0.02 * p_acc;

  int thr = ibus[2];
  int ch1 = ibus[0]; int ch2 = ibus[1]; int ch4 = ibus[3]; 
  int ch5 = ibus[4]; // Công tắc Kênh 5 bật Altitude Hold
  
  if (ch1 > 1475 && ch1 < 1525) ch1 = 1500;
  if (ch2 > 1475 && ch2 < 1525) ch2 = 1500;
  if (ch4 > 1475 && ch4 < 1525) ch4 = 1500;

  float set_r_angle = (ch1 - 1500) * 0.06; 
  float set_p_angle = (ch2 - 1500) * 0.06;
  float set_y_rate = (ch4 - 1500) * 0.15; 

  float target_rate_r = Kp_angle * (set_r_angle - roll);
  float target_rate_p = Kp_angle * (set_p_angle - pitch);

  float err_rate_r = target_rate_r - gx;
  float err_rate_p = target_rate_p - gy;
  float err_rate_y = set_y_rate - gz;

  if (thr > 1200) { 
    i_r += Ki_rate * err_rate_r * dt; i_p += Ki_rate * err_rate_p * dt;
    if (i_r > 150) i_r = 150; if (i_r < -150) i_r = -150;
    if (i_p > 150) i_p = 150; if (i_p < -150) i_p = -150;
  } else { i_r *= 0.5; i_p *= 0.5; }

  float d_r_raw = Kd_rate * -(gx - gx_prev) / dt; float d_p_raw = Kd_rate * -(gy - gy_prev) / dt;
  gx_prev = gx; gy_prev = gy;

  float alpha_D = 0.15; 
  d_r_filtered = d_r_filtered + alpha_D * (d_r_raw - d_r_filtered);
  d_p_filtered = d_p_filtered + alpha_D * (d_p_raw - d_p_filtered);

  float tpa = 1.0;
  if (thr > 1400) {
    tpa = 1.0 - ((thr - 1400) / 600.0) * 0.35; 
    if (tpa < 0.5) tpa = 0.5; 
  }

  float pid_r = (Kp_rate * err_rate_r * tpa) + i_r + (d_r_filtered * tpa);
  float pid_p = (Kp_rate * err_rate_p * tpa) + i_p + (d_p_filtered * tpa);
  float pid_y = Kp_yaw * err_rate_y; 

  // =======================================================
  // LOGIC GIỮ ĐỘ CAO
  // =======================================================
  int base_throttle = thr;
  
  if (ch5 > 1500 && thr > 1300) { 
    if (!is_alt_hold) {
      target_alt = current_alt;
      hover_throttle = thr; 
      is_alt_hold = true;
    }
    if (thr > 1600) target_alt += 0.01; 
    if (thr < 1400) target_alt -= 0.01; 
    float alt_error = target_alt - current_alt;
    float alt_velocity = (current_alt - prev_alt) / dt;
    prev_alt = current_alt;
    float pid_alt = (Kp_alt * alt_error) - (Kd_alt * alt_velocity);
    if (pid_alt > 200) pid_alt = 200;
    if (pid_alt < -200) pid_alt = -200;

    base_throttle = hover_throttle + pid_alt;
  } else {
    is_alt_hold = false;
  }

  int m1 = 1000, m2 = 1000, m3 = 1000, m4 = 1000;
  
  if (base_throttle > 1050) {
    m1 = base_throttle + pid_r - pid_p - pid_y; 
    m2 = base_throttle + pid_r + pid_p + pid_y; 
    m3 = base_throttle - pid_r + pid_p - pid_y; 
    m4 = base_throttle - pid_r - pid_p + pid_y; 

    if (m1 < 1100) m1 = 1100; if (m1 > 2000) m1 = 2000;
    if (m2 < 1100) m2 = 1100; if (m2 > 2000) m2 = 2000;
    if (m3 < 1100) m3 = 1100; if (m3 > 2000) m3 = 2000;
    if (m4 < 1100) m4 = 1100; if (m4 > 2000) m4 = 2000;
  }

  writeM(ESC1, 0, m1); writeM(ESC2, 1, m2);
  writeM(ESC3, 2, m3); writeM(ESC4, 3, m4);

  while (micros() - curr < 4000);
}