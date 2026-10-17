/*
 * =====================================================================
 *  ROBOT TỰ CÂN BẰNG 2 BÁNH - ESP32 + MPU6500 + TB6612FNG
 *  PHIÊN BẢN: NONLINEAR PID + KALMAN 1D + THUẬT TOÁN 2 TRƯỜNG HỢP FALLBACK
 * =====================================================================
 */

#include "Arduino.h"
#include "Wire.h"
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
  #error "Bluetooth chua duoc bat trong SDK (chi ho tro ESP32 classic)."
#endif

// ==========================================
// 1. CẤU HÌNH GPIO VÀ PWM
// ==========================================
#define PIN_I2C_SDA     21
#define PIN_I2C_SCL     22

// Motor Trái (Kênh A)
#define PIN_MOTOR_A_PWM 32
#define PIN_MOTOR_A_IN1 25
#define PIN_MOTOR_A_IN2 33

// Motor Phải (Kênh B)
#define PIN_MOTOR_B_PWM 14
#define PIN_MOTOR_B_IN1 26
#define PIN_MOTOR_B_IN2 27

// Kích hoạt TB6612
#define PIN_TB6612_STBY 13

#define PWM_FREQ        10000   // 10 kHz
#define PWM_RES_BITS    8       // 8-bit (0 - 255)
#define PWM_MAX_DUTY    ((1 << PWM_RES_BITS) - 1)

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  #define LEDC_BY_PIN   1
#else
  #define LEDC_BY_PIN   0
  #define PWM_CHANNEL_A 0
  #define PWM_CHANNEL_B 1
#endif

// ==========================================
// 2. CLASS BỘ LỌC KALMAN 1D
// ==========================================
class Kalman1D {
private:
  float angle; 
  float bias;  
  float P[2][2];

  float Q_angle;   
  float Q_bias;    
  float R_measure;

public:
  Kalman1D() {
    Q_angle   = 0.001f;
    Q_bias    = 0.003f;
    R_measure = 0.06f;

    angle = 0.0f;
    bias  = 0.0f;

    P[0][0] = 0.0f; P[0][1] = 0.0f;
    P[1][0] = 0.0f; P[1][1] = 0.0f;
  }

  void init(float initial_angle) {
    angle = initial_angle;
    bias  = 0.0f;
  }

  float getAngle(float newAngle, float newRate, float dt, float r_scale = 1.0f) {
    float rate = newRate - bias;
    angle += dt * rate;

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    float S = P[0][0] + R_measure * r_scale;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    float y = newAngle - angle;
    angle += K[0] * y;
    bias  += K[1] * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
  }

  float getCleanRate(float rawRate) const {
    return rawRate - bias;
  }
};

// ==========================================
// 3. THÔNG SỐ ĐIỀU KHIỂN NONLINEAR PID (ĐÃ FIX LĂN LIÊN TỤC)
// ==========================================
// Cấu hình P phi tuyến: Đủ lực thắng ma sát tĩnh để bánh lăn từ từ ngay khi nghiêng nhẹ
float Kp_base       = 16.5f;  // Nâng từ 5.0 lên 16.5 để bánh xe lăn liên tục theo góc nghiêng, không đợi Gyro giật
float Kp_boost_gain = 4.5f;   // Tăng lực mượt mà khi xe nghiêng ra ngoài
float Kp_max        = 30.0f;  // Lực tối đa khi gặp góc nguy hiểm

float Ki            = 0.0f;   
#ifndef KD_VAL
#define KD_VAL .50f   // Kd khi lệch lớn
#endif
#ifndef KD_CALM
#define KD_CALM 0.250f  // Kd trong vùng cân bằng
#endif
float Kd            = KD_VAL;

// Lọc thông thấp cho Gyro: Chặn gai rung từ hộp số nhựa truyền vào đáy hộp làm ngắt quãng lệnh lăn
#ifndef GYRO_LPF_ALPHA
#define GYRO_LPF_ALPHA     0.4f   // 0.4 = mượt, xóa hiện tượng Kd băm xung làm bánh chạy-dừng-chạy-dừng
#endif

// Bù ma sát tĩnh sát với thực tế động cơ vàng 1:48 (~28-30 PWM) để tốc độ tăng từ từ, đều đặn
#ifndef MOTOR_DEADZONE
#define MOTOR_DEADZONE     30.0f   // Đặt sát ngưỡng lăn của motor vàng để vượt ma sát tĩnh mượt mà
#endif
#ifndef DZ_RAMP
#define DZ_RAMP            4.0f    // Vuốt mềm vùng chết trong khoảng +-4 PWM quanh điểm 0
#endif
#ifndef PWM_MAX_SLEW_RATE
#define PWM_MAX_SLEW_RATE  80.0f   // Giới hạn tốc độ tăng PWM mỗi 10ms để bánh tăng tốc từ từ, không giật thốc
#endif
#define MPU_ADDR           0x68
#define LOOP_TIME_MS       10      // 100 Hz
#define MAX_TILT_ANGLE     75.0f

// ---- Phục hồi khi nghiêng lớn ----
#ifndef SLEW_START_ERR
#define SLEW_START_ERR   2.0f   // độ: dưới 2 độ giữ tốc độ tăng PWM êm
#endif
#ifndef SLEW_FREE_ERR
#define SLEW_FREE_ERR    6.0f   // độ: trên 6 độ mới mở tối đa tốc độ đáp ứng
#endif
#ifndef ACC_TRUST_KNEE
#define ACC_TRUST_KNEE   80.0f
#endif
#ifndef ACC_TRUST_SPAN
#define ACC_TRUST_SPAN   40.0f
#endif

// ---- Bù Back-EMF êm (Tách khỏi vòng tự kích để không gây vọt tốc độ đột ngột) ----
#ifndef BEMF_COMP
#define BEMF_COMP        0.35f  // Hạ từ 0.95 xuống 0.35 để bù mượt, không tự kích thành xung giật nhanh
#endif
#ifndef BEMF_A
#define BEMF_A           0.04f  // Tích lũy từ từ theo góc nghiêng (hằng số thời gian ~250ms)
#endif
#ifndef BEMF_LIMIT
#define BEMF_LIMIT       80.0f
#endif

// ---- Vùng "êm" quanh điểm cân bằng ----
#ifndef CALM_ERR
#define CALM_ERR         1.0f
#endif
#ifndef CALM_FADE
#define CALM_FADE        2.0f
#endif
#ifndef CALM_ALPHA
#define CALM_ALPHA       0.60f  // Làm mịn đầu ra PWM
#endif
#ifndef REST_DB
#define REST_DB          0.3f   // Hạ từ 5.0 xuống 0.3: Không cắt mất lực lăn ở góc nghiêng 0.3° - 2.0°
#endif
#ifndef DEBUG_PLOT
#define DEBUG_PLOT 0
#endif
#ifndef REARM_ANGLE
#define REARM_ANGLE      10.0f
#endif

// ---- CẤU HÌNH THUẬT TOÁN 2 TRƯỜNG HỢP (CHỐNG TRÔI 1 PHÍA & FALLBACK) ----
#ifndef BALANCE_WINDOW_X
#define BALANCE_WINDOW_X        0.25f   // Khoảng +-x (độ) quanh setpoint coi là Trường hợp 1 (cân bằng)
#endif
#ifndef FALLBACK_TRIGGER_COUNT
#define FALLBACK_TRIGGER_COUNT  5       // 5-6 lần đọc liên tiếp không đổi dấu & không giảm -> nhảy vào Trường hợp 2
#endif
#ifndef FALLBACK_TOLERANCE
#define FALLBACK_TOLERANCE      0.02f   // Dung sai (độ) nhận diện góc lệch đang "giữ nguyên" hoặc "tăng dần"
#endif
#ifndef FALLBACK_STEP_PWM
#define FALLBACK_STEP_PWM       5.5f    // Điện áp (PWM) cộng thêm mỗi chu kỳ khi ở Trường hợp 2
#endif
#ifndef FALLBACK_GAIN_PWM
#define FALLBACK_GAIN_PWM       3.0f    // Điện áp (PWM) cộng thêm tỷ lệ với độ lớn góc lệch ở Trường hợp 2
#endif
#ifndef FALLBACK_MAX_PWM
#define FALLBACK_MAX_PWM        110.0f  // Trần áp tăng cường tối đa của Trường hợp 2
#endif
#ifndef FALLBACK_DECAY
#define FALLBACK_DECAY          0.55f   // Hệ số xả mượt áp tăng cường khi xe quay lại Trường hợp 1
#endif

// ---- HƯỚNG LẮP CẢM BIẾN (đã dời xuống đáy hộp) ----
#ifndef IMU_LEAN_SIGN
#define IMU_LEAN_SIGN    (1.0f)
#endif

// ĐIỂM CÂN BẰNG CƠ KHÍ:
float setpoint_pitch = -1.98f; 
float speed_offset   = 0.0f;
float turn_offset    = 0.0f;

// Lệnh Bluetooth
#define BT_DEVICE_NAME    "ESP32_Balancing_Bot"
#define CMD_SPEED_STEP    1.5f
#define CMD_TURN_STEP     30.0f
#define CMD_FORWARD_SIGN  (-1.0f)

// ==========================================
// 4. HẰNG SỐ CẢM BIẾN & BIẾN TOÀN CỤC
// ==========================================
#define MPU_REG_CONFIG        0x1A
#define MPU_REG_GYRO_CONFIG   0x1B
#define MPU_REG_ACCEL_CONFIG  0x1C
#define MPU_REG_ACCEL_XOUT_H  0x3B
#define MPU_REG_PWR_MGMT_1    0x6B
#define MPU_REG_WHO_AM_I      0x75

#define GYRO_SENS         131.0f
#define RAD2DEG           57.2957795f
#define CAL_SAMPLES       500
#define CAL_INTERVAL_MS   4
#define MAX_SENSOR_FAILS  5

BluetoothSerial SerialBT;
Kalman1D kalmanFilter;

int16_t raw_ax, raw_ay, raw_az;
int16_t raw_gx, raw_gy, raw_gz;

static bool  armed     = false;
float        wheel_est = 0;
static float out_filt  = 0;
static float calm_k    = 0;

// Biến trạng thái của Thuật toán 2 Trường hợp (Case 1 & Case 2)
static uint8_t control_case     = 1;     // 1 = Trường hợp 1 (Cân bằng), 2 = Trường hợp 2 (Tăng áp Fallback)
static uint8_t same_sign_count  = 0;     // Số lần đọc liên tiếp góc lệch giữ nguyên dấu & không giảm
static int8_t  prev_error_sign  = 0;     // Dấu của góc lệch ở chu kỳ trước (-1, 0, +1)
static float   prev_abs_error   = 0.0f;  // Độ lớn góc lệch ở chu kỳ trước
static float   fallback_boost   = 0.0f;  // Điện áp PWM tăng cường trong Trường hợp 2

float gyro_pitch_offset = 0;   // offset gyro quanh trục Y
float gyro_pitch_rate   = 0;
float clean_gyro_rate   = 0;   // tốc độ góc đã qua lọc thông thấp chống giật ngắt quãng
float angle_pitch       = 0;

float pid_error = 0, error_integral = 0;
float last_pwm_left = 0, last_pwm_right = 0;
uint32_t prev_time = 0;
uint8_t  sensor_fail_count = 0;

float    last_printed_angle = 0.0f;
uint32_t last_print_time    = 0;

// ==========================================
// 5. XỬ LÝ MPU6500 & KALMAN
// ==========================================
static void writeMPU(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

static uint8_t readMPU(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1, true);
  return Wire.available() ? Wire.read() : 0xFF;
}

static bool mpuResponds() {
  Wire.beginTransmission(MPU_ADDR);
  return Wire.endTransmission() == 0;
}

static int16_t readInt16() {
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

static bool readMPURaw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;

  Wire.requestFrom(MPU_ADDR, 14, true);
  if (Wire.available() < 14) return false;

  raw_ax = readInt16();
  raw_ay = readInt16();
  raw_az = readInt16();
  (void)readInt16();
  raw_gx = readInt16();
  raw_gy = readInt16();
  raw_gz = readInt16();
  return true;
}

// Góc nghiêng từ Accel. Z hướng xuống => az ≈ -1g khi thẳng đứng.
static float accelLeanDeg(float ax, float az) {
  return IMU_LEAN_SIGN * atan2f(-ax, -az) * RAD2DEG;
}

static bool initMPU6500() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  delay(100);

  if (!mpuResponds()) return false;

  writeMPU(MPU_REG_PWR_MGMT_1, 0x00);
  delay(50);

  writeMPU(MPU_REG_GYRO_CONFIG, 0x00);
  writeMPU(MPU_REG_ACCEL_CONFIG, 0x00);
  writeMPU(MPU_REG_CONFIG, 0x03); // DLPF ~42Hz
  delay(100);

  Serial.println(">>> DANG CAN CHINH GYRO (GIU YEN XE 2S) <<<");
  long sum_gy = 0, sum_ax = 0, sum_az = 0;
  int good = 0;
  for (int i = 0; i < CAL_SAMPLES; i++) {
    if (readMPURaw()) {
      sum_gy += raw_gy;
      sum_ax += raw_ax;
      sum_az += raw_az;
      good++;
    }
    delay(CAL_INTERVAL_MS);
  }
  if (good < CAL_SAMPLES / 2) return false;

  gyro_pitch_offset = (float)sum_gy / good;
  
  if ((float)sum_az / good > 0.0f)
    Serial.println("CANH BAO: az > 0 -> cam bien dang lap nguoc (Z phai huong xuong, az ~ -1g)");
  float initial_angle = accelLeanDeg((float)sum_ax / good, (float)sum_az / good);
  kalmanFilter.init(initial_angle);
  angle_pitch = initial_angle;
  clean_gyro_rate = 0.0f;

  Serial.println(">>> DA SAN SANG! <<<");
  return true;
}

static bool readSensor(float dt) {
  if (!readMPURaw()) return false;

  float acc_pitch = accelLeanDeg((float)raw_ax, (float)raw_az);
  gyro_pitch_rate = -IMU_LEAN_SIGN * ((float)raw_gy - gyro_pitch_offset) / GYRO_SENS;

  float u_abs = 0.5f * (fabsf(last_pwm_left) + fabsf(last_pwm_right));
  float over  = fmaxf(0.0f, u_abs - ACC_TRUST_KNEE) / ACC_TRUST_SPAN;
  angle_pitch = kalmanFilter.getAngle(acc_pitch, gyro_pitch_rate, dt, 1.0f + over * over);

  // Lọc thông thấp vận tốc góc với GYRO_LPF_ALPHA = 0.4f
  float raw_clean_rate = kalmanFilter.getCleanRate(gyro_pitch_rate);
  clean_gyro_rate += GYRO_LPF_ALPHA * (raw_clean_rate - clean_gyro_rate);

  return true;
}

// ==========================================
// 6. ĐIỀU KHIỂN ĐỘNG CƠ (SOFT DEADZONE LIÊN TỤC)
// ==========================================
static void setMotorPWM(uint8_t pin, uint32_t val) {
#if LEDC_BY_PIN
  ledcWrite(pin, val);
#else
  if (pin == PIN_MOTOR_A_PWM)      ledcWrite(PWM_CHANNEL_A, val);
  else if (pin == PIN_MOTOR_B_PWM) ledcWrite(PWM_CHANNEL_B, val);
#endif
}

static void initMotors() {
  pinMode(PIN_MOTOR_A_IN1, OUTPUT);
  pinMode(PIN_MOTOR_A_IN2, OUTPUT);
  pinMode(PIN_MOTOR_B_IN1, OUTPUT);
  pinMode(PIN_MOTOR_B_IN2, OUTPUT);
  digitalWrite(PIN_MOTOR_A_IN1, LOW);
  digitalWrite(PIN_MOTOR_A_IN2, LOW);
  digitalWrite(PIN_MOTOR_B_IN1, LOW);
  digitalWrite(PIN_MOTOR_B_IN2, LOW);

  pinMode(PIN_TB6612_STBY, OUTPUT);
  digitalWrite(PIN_TB6612_STBY, LOW);

#if LEDC_BY_PIN
  ledcAttach(PIN_MOTOR_A_PWM, PWM_FREQ, PWM_RES_BITS);
  ledcAttach(PIN_MOTOR_B_PWM, PWM_FREQ, PWM_RES_BITS);
#else
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RES_BITS);
  ledcAttachPin(PIN_MOTOR_A_PWM, PWM_CHANNEL_A);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RES_BITS);
  ledcAttachPin(PIN_MOTOR_B_PWM, PWM_CHANNEL_B);
#endif

  setMotorPWM(PIN_MOTOR_A_PWM, 0);
  setMotorPWM(PIN_MOTOR_B_PWM, 0);
}

static void stopMotors() {
  setMotorPWM(PIN_MOTOR_A_PWM, 0);
  setMotorPWM(PIN_MOTOR_B_PWM, 0);
  digitalWrite(PIN_MOTOR_A_IN1, LOW);
  digitalWrite(PIN_MOTOR_A_IN2, LOW);
  digitalWrite(PIN_MOTOR_B_IN1, LOW);
  digitalWrite(PIN_MOTOR_B_IN2, LOW);
  last_pwm_left   = 0;
  last_pwm_right  = 0;
  error_integral  = 0;
  wheel_est       = 0;
  out_filt        = 0;
  same_sign_count = 0;
  prev_error_sign = 0;
  prev_abs_error  = 0;
  fallback_boost  = 0;
  control_case    = 1;
}

// Bù Deadzone nội suy mượt mà
static float applyContinuousSoftDeadzone(float raw_pwm) {
  if (fabsf(raw_pwm) < 0.1f) return 0.0f;
  
  float boost = MOTOR_DEADZONE;
  if (fabsf(raw_pwm) < DZ_RAMP) {
    boost = MOTOR_DEADZONE * (fabsf(raw_pwm) / DZ_RAMP);
  }
  
  return (raw_pwm > 0) ? (raw_pwm + boost) : (raw_pwm - boost);
}

static void driveMotors(float pid_output, float turn) {
  if (fabsf(angle_pitch) > MAX_TILT_ANGLE) {
    stopMotors();
    return;
  }

  // Chỉ áp dụng REST_DB khi đang ở Trường hợp 1 (cân bằng tại tâm)
  if (REST_DB > 0.0f && calm_k > 0.0f && control_case == 1) {
    float a = fabsf(pid_output);
    float shrink = fminf(a, REST_DB * calm_k);
    pid_output = (pid_output > 0.0f ? 1.0f : -1.0f) * (a - shrink);
  }
  float target_pwm_left  = pid_output + turn;
  float target_pwm_right = pid_output - turn;

  target_pwm_left  = applyContinuousSoftDeadzone(target_pwm_left);
  target_pwm_right = applyContinuousSoftDeadzone(target_pwm_right);

  // Slew Rate Limiter: Giữ PWM_MAX_SLEW_RATE (80.0f) ở vùng cân bằng, nới dần khi nghiêng lớn hoặc ở TH2
  float free_k = constrain((fabsf(pid_error) - SLEW_START_ERR) / (SLEW_FREE_ERR - SLEW_START_ERR), 0.0f, 1.0f);
  if (control_case == 2) free_k = fmaxf(free_k, 0.5f);
  float slew = PWM_MAX_SLEW_RATE + (2.0f * PWM_MAX_DUTY - PWM_MAX_SLEW_RATE) * free_k;
  float delta_l = constrain(target_pwm_left - last_pwm_left, -slew, slew);
  float delta_r = constrain(target_pwm_right - last_pwm_right, -slew, slew);

  float pwm_left  = last_pwm_left + delta_l;
  float pwm_right = last_pwm_right + delta_r;

  last_pwm_left  = pwm_left;
  last_pwm_right = pwm_right;

  pwm_left  = constrain(pwm_left,  -(float)PWM_MAX_DUTY, (float)PWM_MAX_DUTY);
  pwm_right = constrain(pwm_right, -(float)PWM_MAX_DUTY, (float)PWM_MAX_DUTY);

  // Kênh A (Trái)
  if (pwm_left >= 0) {
    digitalWrite(PIN_MOTOR_A_IN1, LOW);
    digitalWrite(PIN_MOTOR_A_IN2, HIGH);
    setMotorPWM(PIN_MOTOR_A_PWM, (uint32_t)pwm_left);
  } else {
    digitalWrite(PIN_MOTOR_A_IN1, HIGH);
    digitalWrite(PIN_MOTOR_A_IN2, LOW);
    setMotorPWM(PIN_MOTOR_A_PWM, (uint32_t)(-pwm_left));
  }

  // Kênh B (Phải)
  if (pwm_right >= 0) {
    digitalWrite(PIN_MOTOR_B_IN1, HIGH);
    digitalWrite(PIN_MOTOR_B_IN2, LOW);
    setMotorPWM(PIN_MOTOR_B_PWM, (uint32_t)pwm_right);
  } else {
    digitalWrite(PIN_MOTOR_B_IN1, LOW);
    digitalWrite(PIN_MOTOR_B_IN2, HIGH);
    setMotorPWM(PIN_MOTOR_B_PWM, (uint32_t)(-pwm_right));
  }
}

// ==========================================
// 7. XỬ LÝ BLUETOOTH
// ==========================================
static void handleBluetooth() {
  if (!SerialBT.hasClient()) {
    speed_offset = 0.0f;
    turn_offset  = 0.0f;
    return;
  }

  while (SerialBT.available()) {
    char cmd = (char)SerialBT.read();
    switch (cmd) {
      case 'F': speed_offset =  CMD_FORWARD_SIGN * CMD_SPEED_STEP; turn_offset = 0.0f; break;
      case 'B': speed_offset = -CMD_FORWARD_SIGN * CMD_SPEED_STEP; turn_offset = 0.0f; break;
      case 'L': turn_offset = -CMD_TURN_STEP; break;
      case 'R': turn_offset =  CMD_TURN_STEP; break;
      case 'S': speed_offset = 0.0f; turn_offset = 0.0f; break;
      default: break;
    }
  }
}

// ==========================================
// 8. SETUP VÀ LOOP CHÍNH
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  if (!SerialBT.begin(BT_DEVICE_NAME)) {
    Serial.println("CANH BAO: khong khoi tao duoc Bluetooth");
  }

  initMotors();

  if (!initMPU6500()) {
    while (true) {
      Serial.println("!!! LOI: KHONG DOC DUOC MPU6500 !!!");
      delay(2000);
    }
  }

  digitalWrite(PIN_TB6612_STBY, HIGH);
  prev_time = micros();
  last_print_time = millis();
}

void loop() {
  uint32_t now = micros();
  uint32_t elapsed_us = now - prev_time;
  if (elapsed_us < (uint32_t)LOOP_TIME_MS * 1000UL) return;

  prev_time = now;
  float dt = (float)elapsed_us * 1.0e-6f;

  handleBluetooth();

  if (!readSensor(dt)) {
    if (sensor_fail_count < 255) sensor_fail_count++;
    if (sensor_fail_count >= MAX_SENSOR_FAILS) stopMotors();
    return;
  }
  sensor_fail_count = 0;

  // Khóa chạy lại: sau khi ngã (>75°) hoặc mới bật máy, chỉ cân bằng khi xe đã được dựng gần thẳng.
  if (fabsf(angle_pitch) > MAX_TILT_ANGLE) armed = false;
  if (!armed) {
    if (fabsf(angle_pitch) < REARM_ANGLE) armed = true;
    else { stopMotors(); return; }
  }

  // 1. Tính sai số so với setpoint_pitch (-1.98f)
  float target_angle = setpoint_pitch + speed_offset;
  pid_error = angle_pitch - target_angle;
  float abs_err = fabsf(pid_error);

  // -------------------------------------------------------------
  // 2. XÉT TRƯỜNG HỢP 1 (CÂN BẰNG) & TRƯỜNG HỢP 2 (TĂNG ÁP FALLBACK)
  // -------------------------------------------------------------
  int8_t current_sign = (pid_error > 0.02f) ? 1 : ((pid_error < -0.02f) ? -1 : 0);
  bool is_holding_or_increasing = (abs_err >= (prev_abs_error - FALLBACK_TOLERANCE));

  // Kiểm tra góc lệch vượt khoảng BALANCE_WINDOW_X và giữ nguyên dấu
  if (current_sign != 0 && current_sign == prev_error_sign && abs_err > BALANCE_WINDOW_X) {
    if (is_holding_or_increasing) {
      if (same_sign_count < 255) same_sign_count++;
    } else {
      // Góc lệch đang thu hẹp về phía setpoint -> trả về Trường hợp 1
      same_sign_count = 0;
    }
  } else {
    same_sign_count = (current_sign != 0 && abs_err > BALANCE_WINDOW_X) ? 1 : 0;
    if (current_sign != prev_error_sign) {
      fallback_boost = 0.0f; // Vừa đổi dấu qua setpoint -> xóa sạch áp cộng thêm
    }
  }

  // Chuyển đổi giữa Trường hợp 1 và Trường hợp 2
  if (same_sign_count >= FALLBACK_TRIGGER_COUNT && is_holding_or_increasing) {
    // ---> TRƯỜNG HỢP 2: Không đổi dấu trong 5-6 lần đọc & độ lệch giữ nguyên/tăng dần -> Tăng áp Fallback
    control_case = 2;
    float boost_step = FALLBACK_STEP_PWM + (FALLBACK_GAIN_PWM * abs_err);
    fallback_boost += (float)current_sign * boost_step;
    fallback_boost = constrain(fallback_boost, -FALLBACK_MAX_PWM, FALLBACK_MAX_PWM);
  } else {
    // ---> TRƯỜNG HỢP 1: Cân bằng quanh vị trí setpoint
    control_case = 1;
    fallback_boost *= FALLBACK_DECAY;
    if (fabsf(fallback_boost) < 0.5f) fallback_boost = 0.0f;
  }

  prev_abs_error = abs_err;
  if (current_sign != 0) prev_error_sign = current_sign;

  // -------------------------------------------------------------
  // 3. THUẬT TOÁN NONLINEAR PID (TRƯỜNG HỢP 1 + BÙ TRƯỜNG HỢP 2)
  // -------------------------------------------------------------
  float kp_effective = Kp_base;
  if (abs_err > 0.87f) {
    kp_effective += (abs_err - 0.87f) * Kp_boost_gain;
    if (kp_effective > Kp_max) kp_effective = Kp_max;
  }

  error_integral += pid_error * dt;
  error_integral = constrain(error_integral, -100.0f, 100.0f);

  calm_k = 1.0f - constrain((abs_err - CALM_ERR) / CALM_FADE, 0.0f, 1.0f);
  float kd_effective = KD_CALM + (Kd - KD_CALM) * (1.0f - calm_k);

  float p_term = kp_effective * pid_error;
  float d_term = kd_effective * clean_gyro_rate;

  // Chống khựng bánh xe (Anti-Stutter) khi đang nghiêng về 1 phía
  if (abs_err > 0.4f && (p_term * d_term < 0.0f)) {
    float max_oppose = 0.55f * fabsf(p_term);
    d_term = constrain(d_term, -max_oppose, max_oppose);
  }

  // Bù Back-EMF êm theo p_term
  wheel_est += BEMF_A * (p_term - wheel_est);
  float bemf_boost = constrain(BEMF_COMP * wheel_est, -BEMF_LIMIT, BEMF_LIMIT);

  // Tổng hợp lực: Cân bằng (TH1) + Tăng áp Fallback (TH2)
  float pid_output = p_term + (Ki * error_integral) + d_term + bemf_boost + fallback_boost;
  pid_output = constrain(pid_output, -255.0f, 255.0f);

  // Làm mịn đầu ra PWM ở Trường hợp 1; khi sang Trường hợp 2 thì xuất thẳng để fallback nhanh
  float alpha = (control_case == 2) ? 1.0f : (1.0f - calm_k * (1.0f - CALM_ALPHA));
  out_filt += alpha * (pid_output - out_filt);
  pid_output = out_filt;

  driveMotors(pid_output, turn_offset);

#if DEBUG_PLOT
  Serial.print(angle_pitch, 2);  Serial.print(',');
  Serial.print(fallback_boost, 1); Serial.print(',');
  Serial.println(0.5f * (last_pwm_left + last_pwm_right), 0);
  return;
#endif

  uint32_t ms = millis();
  if (fabsf(angle_pitch - last_printed_angle) >= 0.20f && (ms - last_print_time >= 100)) {
    last_printed_angle = angle_pitch;
    last_print_time = ms;

    Serial.print("TH");
    Serial.print(control_case);
    Serial.print(" (Cnt:");
    Serial.print(same_sign_count);
    Serial.print(") | Goc: ");
    Serial.print(angle_pitch, 2);
    Serial.print(" | Lech: ");
    Serial.print(pid_error, 2);
    Serial.print(" | Boost: ");
    Serial.print(fallback_boost, 1);
    Serial.print(" | PWM: ");
    Serial.println(0.5f * (last_pwm_left + last_pwm_right), 0);
  }
}