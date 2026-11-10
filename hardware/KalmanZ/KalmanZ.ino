#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// --- Definición de pines ---
#define TRIG_PIN 6
#define ECHO_PIN 7
#define I2C_SDA 1
#define I2C_SCL 39
#define I2C1_SDA 4 // Ajusta si es necesario
#define I2C1_SCL 5 // Ajusta si es necesario

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

float ax, ay, az; 
float gx, gy, gz; 
float dist_tof_m = -1.0;
float dist_us_m = -1.0;
float current_roll = 0, current_pitch = 0;
float accel_z_offset = 0; 

struct KalmanZ {
  float Q_alt = 0.0001;  
  float Q_vel = 0.0001;  
  // R_meas fue eliminado de aquí porque ahora es dinámico
  
  float P[2][2] = {{1, 0}, {0, 1}}; 
  float x[2] = {0, 0}; 
  
  void predict(float dt, float accel_z) {
    x[0] += x[1] * dt + 0.5 * accel_z * dt * dt;
    x[1] += accel_z * dt;
    
    float p00 = P[0][0] + dt * (P[1][0] + P[0][1]) + dt * dt * P[1][1] + Q_alt;
    float p01 = P[0][1] + dt * P[1][1];
    float p10 = P[1][0] + dt * P[1][1];
    float p11 = P[1][1] + Q_vel;
    
    P[0][0] = p00; P[0][1] = p01;
    P[1][0] = p10; P[1][1] = p11;
  }

  // CAMBIO 1: La función ahora recibe R_dinamico
  void update(float measured_z, float R_dinamico) {
    float S = P[0][0] + R_dinamico;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;
    
    float y = measured_z - x[0]; 
    x[0] += K[0] * y;
    x[1] += K[1] * y;
    
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];
    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;
  }
};

KalmanZ kfZ;
unsigned long last_time = 0;
bool kalman_inicializado = false; 

void leerMPU6500() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true); 

  if (Wire.available() >= 14) {
    int16_t raw_ax = (Wire.read() << 8) | Wire.read();
    int16_t raw_ay = (Wire.read() << 8) | Wire.read();
    int16_t raw_az = (Wire.read() << 8) | Wire.read();
    int16_t raw_temp = (Wire.read() << 8) | Wire.read(); 
    int16_t raw_gx = (Wire.read() << 8) | Wire.read();
    int16_t raw_gy = (Wire.read() << 8) | Wire.read();
    int16_t raw_gz = (Wire.read() << 8) | Wire.read();

    ax = raw_ax / 16384.0; 
    ay = raw_ay / 16384.0;
    az = raw_az / 16384.0;
    gx = raw_gx / 131.0;
    gy = raw_gy / 131.0;
    gz = raw_gz / 131.0;
  }
}

void setup() {
  Serial0.begin(115200);
  delay(1000);
  Serial0.println("=== Iniciando Fusion de Sensores ===");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); 
  Wire.write(0x00); 
  if (Wire.endTransmission() == 0) {
    Serial0.println("MPU-6500 OK");
  }

  Wire1.begin(I2C1_SDA, I2C1_SCL);
  if (!lox.begin(0x29, false, &Wire1)) {
    Serial0.println("Fallo VL53L0X");
    while (1) delay(10);
  } else {
    Serial0.println("VL53L0X OK");
  }

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // CALIBRACIÓN DEL EJE Z (GRAVEDAD)
  Serial0.println("Calibrando eje Z (NO MOVER)...");
  float az_sum = 0;
  for (int i = 0; i < 500; i++) {
    leerMPU6500();
    az_sum += az;
    delay(2);
  }
  float az_promedio = az_sum / 500.0;
  accel_z_offset = (az_promedio * 9.81) - 9.81; 
  
  Serial0.print("Offset Z calculado: "); Serial0.println(accel_z_offset);
  Serial0.println("Iniciando...");

  last_time = micros();
}

void loop() {
  unsigned long current_time = micros();
  float dt = (current_time - last_time) / 1000000.0; 
  last_time = current_time;

  leerMPU6500();

  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  dist_tof_m = (measure.RangeStatus != 4) ? (measure.RangeMilliMeter / 1000.0) : -1.0;

  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 15000); 
  dist_us_m = (duration == 0) ? -1.0 : ((duration * 0.0343) / 2.0 / 100.0);

  // Orientación
  float accel_roll = atan2(ay, az) * RAD_TO_DEG;
  float accel_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
  current_roll = 0.98 * (current_roll + gx * dt) + 0.02 * accel_roll;
  current_pitch = 0.98 * (current_pitch + gy * dt) + 0.02 * accel_pitch;

  // FUSIÓN DE ALTITUD (Kalman)
  float accel_z_real = (az * 9.81) - 9.81 - accel_z_offset; 
  
  // CAMBIO 2: Nueva lógica de selección de sensores
  float measured_z = -1.0;
  float current_R = 0.02; // Valor por defecto

  // 1. Prioridad al Ultrasónico (US)
  if (dist_us_m > 0.05 && dist_us_m < 3.00) {
    measured_z = dist_us_m;
    current_R = 0.005;  // CONFIANZA ALTA: R bajo (le creemos mucho)
  } 
  // 2. Si el US falla, usamos el ToF
  else if (dist_tof_m > 0.03 && dist_tof_m < 1.50) {
    measured_z = dist_tof_m;
    current_R = 0.15;   // CONFIANZA BAJA: R alto (le creemos poco)
  }

  // INICIALIZACIÓN DEL KALMAN
  if (!kalman_inicializado && measured_z > 0) {
    kfZ.x[0] = measured_z; 
    kfZ.x[1] = 0;
    kalman_inicializado = true;
  }

  if (kalman_inicializado) {
    kfZ.predict(dt, accel_z_real);
    if (measured_z > 0) {
      // CAMBIO 3: Pasamos el valor dinámico de R
      kfZ.update(measured_z, current_R); 
    }
  }

  // Imprimir
  static unsigned long print_timer = 0;
  if (millis() - print_timer > 50) {
    print_timer = millis();
    Serial0.print(kfZ.x[0] * 100); Serial0.print(","); // Altura
    Serial0.print(kfZ.x[1] * 100); Serial0.print(","); // Velocidad
    Serial0.print(current_roll);   Serial0.print(","); 
    Serial0.println(current_pitch); 
  }
}