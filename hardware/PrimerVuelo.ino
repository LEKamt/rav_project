#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// --- Definición de pines ---
#define TRIG_PIN 6
#define ECHO_PIN 7
#define I2C_SDA 1
#define I2C_SCL 39
#define I2C1_SDA 4
#define I2C1_SCL 5

// --- Pines Motores ---
#define M1_PIN 8  
#define M2_PIN 17 
#define M3_PIN 21 
#define M4_PIN 18 

// --- Configuración PWM ---
#define PWM_FREQ 50
#define PWM_RES 14
const uint32_t DUTY_MAX = (1 << PWM_RES) - 1;

#define PULSO_MIN 1000
#define PULSO_MAX 1600 
#define PULSO_IDLE 1000

// --- PID (Ganancias bajas para no vibrar) ---
float Kp_roll = 0.05;  
float Kd_roll = 0.00;  
float Kp_pitch = 0.05; 
float Kd_pitch = 0.00;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
float ax, ay, az; 
float gx, gy, gz; 
float current_roll = 0, current_pitch = 0;
float roll_offset = 0, pitch_offset = 0;
float gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;
float accel_z_offset = 0;

// --- Filtro Media Móvil para el Giroscopio (Término D) ---
class MovingAverage {
  private:
    static const int N = 5; 
    float buf[N]; int idx; float sum;
  public:
    MovingAverage() : idx(0), sum(0) {
      for (int i = 0; i < N; i++) buf[i] = 0;
    }
    float process(float val) {
      sum -= buf[idx]; buf[idx] = val; sum += val;
      idx = (idx + 1) % N;
      return sum / N;
    }
};
MovingAverage filtroGx, filtroGy;

// --- Kalman Z ---
struct KalmanZ {
  float Q_alt = 0.0001, Q_vel = 0.0001;
  float P[2][2] = {{1, 0}, {0, 1}}; 
  float x[2] = {0, 0}; 
  void predict(float dt, float accel_z) {
    x[0] += x[1] * dt + 0.5 * accel_z * dt * dt;
    x[1] += accel_z * dt;
    float p00 = P[0][0] + dt * (P[1][0] + P[0][1]) + dt * dt * P[1][1] + Q_alt;
    float p01 = P[0][1] + dt * P[1][1]; float p10 = P[1][0] + dt * P[1][1];
    float p11 = P[1][1] + Q_vel;
    P[0][0] = p00; P[0][1] = p01; P[1][0] = p10; P[1][1] = p11;
  }
  void update(float measured_z, float R_dinamico) {
    float S = P[0][0] + R_dinamico;
    float K[2] = { P[0][0] / S, P[1][0] / S };
    float y = measured_z - x[0]; 
    x[0] += K[0] * y; x[1] += K[1] * y;
    float P00_temp = P[0][0]; float P01_temp = P[0][1];
    P[0][0] -= K[0] * P00_temp; P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp; P[1][1] -= K[1] * P01_temp;
  }
};
KalmanZ kfZ;
bool kalman_inicializado = false;
unsigned long last_time = 0;

void leerMPU6500() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true); 
  if (Wire.available() >= 14) {
    int16_t raw_ax = (Wire.read() << 8) | Wire.read();
    int16_t raw_ay = (Wire.read() << 8) | Wire.read();
    int16_t raw_az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // Ignorar temp
    int16_t raw_gx = (Wire.read() << 8) | Wire.read();
    int16_t raw_gy = (Wire.read() << 8) | Wire.read();
    int16_t raw_gz = (Wire.read() << 8) | Wire.read();
    
    // Orientación corregida (90 grados)
    // MPU en ±4g (divisor 8192) y ±500dps (divisor 65.5)
    ax = raw_ay / 8192.0; 
    ay = raw_ax / 8192.0; 
    az = raw_az / 8192.0;
    gx = raw_gy / 65.5;   
    gy = raw_gx / 65.5;   
    gz = raw_gz / 65.5;
  }
}

void setMotorPulse(int motor, int pulse_us) {
  if(pulse_us < PULSO_MIN) pulse_us = PULSO_MIN;
  if(pulse_us > PULSO_MAX) pulse_us = PULSO_MAX;
  uint32_t duty = ((uint32_t)pulse_us * DUTY_MAX) / 20000UL;
  switch(motor) {
    case 0: ledcWrite(M1_PIN, duty); break;
    case 1: ledcWrite(M2_PIN, duty); break;
    case 2: ledcWrite(M3_PIN, duty); break;
    case 3: ledcWrite(M4_PIN, duty); break;
  }
}

void setup() {
  Serial0.begin(115200);
  delay(1000);
  
  ledcAttach(M1_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(M2_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(M3_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(M4_PIN, PWM_FREQ, PWM_RES);

  Serial0.println("Armando ESCs...");
  for(int i=0; i<4; i++) setMotorPulse(i, PULSO_MAX);
  delay(3000);
  for(int i=0; i<4; i++) setMotorPulse(i, PULSO_MIN);
  delay(3000);
  for(int i=0; i<4; i++) setMotorPulse(i, PULSO_IDLE);

  Wire.begin(I2C_SDA, I2C_SCL);
  // Despertar MPU
  Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
  // Configurar Giroscopio a ±500 dps
  Wire.beginTransmission(0x68); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();
  // Configurar Acelerómetro a ±4g
  Wire.beginTransmission(0x68); Wire.write(0x1C); Wire.write(0x08); Wire.endTransmission();
  
  Wire1.begin(I2C1_SDA, I2C1_SCL);
  lox.begin(0x29, false, &Wire1);
  
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);

  // --- CALIBRACIÓN COMPLETA ---
  Serial0.println("Calibrando sensores (NO MOVER)...");
  float roll_sum = 0, pitch_sum = 0, az_sum = 0;
  float gx_sum = 0, gy_sum = 0, gz_sum = 0;
  
  for (int i = 0; i < 500; i++) {
    leerMPU6500();
    // Calcular ángulos crudos
    float accel_roll = atan2(ay, az) * RAD_TO_DEG;
    float accel_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
    
    roll_sum += accel_roll;
    pitch_sum += accel_pitch;
    az_sum += az;
    
    // Sumar giroscopio
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;
    
    delay(5);
  }
  
  roll_offset = roll_sum / 500.0;
  pitch_offset = pitch_sum / 500.0;
  accel_z_offset = ((az_sum / 500.0) * 9.81) - 9.81; 
  
  // Guardar offsets del giroscopio
  gyro_offset_x = gx_sum / 500.0;
  gyro_offset_y = gy_sum / 500.0;
  gyro_offset_z = gz_sum / 500.0;

  Serial0.print("Offset Giro X: "); Serial0.println(gyro_offset_x);
  Serial0.print("Offset Giro Y: "); Serial0.println(gyro_offset_y);
  Serial0.println("Listo. Envia 'a' para armar.");

  // Inicializar ángulos
  current_roll = 0;
  current_pitch = 0;
  last_time = micros();
}

bool vuelo_activo = false;
float throttle_base = 0.0; 
float target_throttle = 0.0; 
float deadband_deg = 2.0; 

void loop() {
  unsigned long current_time = micros();
  float dt = (current_time - last_time) / 1000000.0; 
  last_time = current_time;

  // Limitar dt para evitar saltos si el loop se bloquea
  if (dt > 0.02) dt = 0.02;
  if (dt < 0.001) dt = 0.001;

  leerMPU6500();

  // Aplicar calibración de giroscopio
  gx -= gyro_offset_x;
  gy -= gyro_offset_y;
  gz -= gyro_offset_z;

  // --- FILTRO COMPLEMENTARIO ---
  float accel_roll = atan2(ay, az) * RAD_TO_DEG;
  float accel_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

  // El factor 0.98 es válido para loops rápidos. 
  // Con dt limitado, la corrección del acelerómetro estabiliza el ángulo.
  current_roll = 0.98 * (current_roll + gx * dt) + 0.02 * accel_roll;
  current_pitch = 0.98 * (current_pitch + gy * dt) + 0.02 * accel_pitch;

  // Restar offset de nivel para que en reposo sea 0
  float angle_roll = current_roll - roll_offset;
  float angle_pitch = current_pitch - pitch_offset;

  // --- SUAVIZADO DEL ARRANQUE ---
  if (vuelo_activo) {
    throttle_base = throttle_base + (target_throttle - throttle_base) * 0.05; 
  } else {
    throttle_base = 0.0;
  }

  if (vuelo_activo && throttle_base > 0.01) { 
    float error_roll = 0.0 - (- angle_roll);
    float error_pitch = 0.0 - angle_pitch;

    if (abs(error_roll) < deadband_deg) error_roll = 0;
    if (abs(error_pitch) < deadband_deg) error_pitch = 0;

    // Filtrar giroscopio para término D
    float gx_filtrado = filtroGx.process(gx);
    float gy_filtrado = filtroGy.process(gy);

    float pid_roll = (Kp_roll * error_roll) - (Kd_roll * gx_filtrado);
    float pid_pitch = (Kp_pitch * error_pitch) - (Kd_pitch * gy_filtrado);

    float m1_speed = throttle_base - pid_roll + pid_pitch; 
    float m2_speed = throttle_base + pid_roll + pid_pitch; 
    float m3_speed = throttle_base + pid_roll - pid_pitch; 
    float m4_speed = throttle_base - pid_roll - pid_pitch; 

    int p_m1 = PULSO_IDLE + (max(0.0f, m1_speed) * (PULSO_MAX - PULSO_IDLE));
    int p_m2 = PULSO_IDLE + (max(0.0f, m2_speed) * (PULSO_MAX - PULSO_IDLE));
    int p_m3 = PULSO_IDLE + (max(0.0f, m3_speed) * (PULSO_MAX - PULSO_IDLE));
    int p_m4 = PULSO_IDLE + (max(0.0f, m4_speed) * (PULSO_MAX - PULSO_IDLE));

    setMotorPulse(0, p_m1);
    setMotorPulse(1, p_m2);
    setMotorPulse(2, p_m3);
    setMotorPulse(3, p_m4);
  } else {
    setMotorPulse(0, PULSO_IDLE); setMotorPulse(1, PULSO_IDLE);
    setMotorPulse(2, PULSO_IDLE); setMotorPulse(3, PULSO_IDLE);
  }

  // --- Comandos ---
  if (Serial0.available()) {
    char cmd = Serial0.read();
    if (cmd == 'a') {
      vuelo_activo = true;
      target_throttle = 0.05;
      Serial0.println("¡Vuelo activo! Throttle inicial 5%");
    } else if (cmd == 's') {
      vuelo_activo = false;
      target_throttle = 0.0;
      Serial0.println("Vuelo detenido. Motores en IDLE.");
    } else if (cmd == '+') {
      target_throttle += 0.05;
      if(target_throttle > 0.8) target_throttle = 0.8;
      Serial0.print("Target Throttle: "); Serial0.println(target_throttle * 100);
    } else if (cmd == '-') {
      target_throttle -= 0.05;
      if(target_throttle < 0.0) target_throttle = 0.0;
      Serial0.print("Target Throttle: "); Serial0.println(target_throttle * 100);
    }
  }

  static unsigned long print_timer = 0;
  if (millis() - print_timer > 100) {
    print_timer = millis();
    Serial0.print("Roll: "); Serial0.print(angle_roll); Serial0.print(" | ");
    Serial0.print("Pitch: "); Serial0.print(angle_pitch); Serial0.print(" | ");
    Serial0.print("Throttle: "); Serial0.print(throttle_base * 100); Serial0.println("%");
  }
}