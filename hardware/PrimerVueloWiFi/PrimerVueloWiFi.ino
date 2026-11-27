// PrimerVuelo + WiFi
// -----------------------------------------------------------------------------
// Es hardware/PrimerVuelo.ino (última versión, con Kd, deadband suave y mezclador
// proporcional) SIN cambios en el control. Lo nuevo va marcado con [WiFi]:
//
//  - El ESP32-S3 crea su propia red WiFi (punto de acceso): "F450-RAV".
//  - Abrir http://192.168.4.1 en el celular o la laptop conectados a esa red.
//  - La página tiene ARMAR, PARAR, +/- y una barra de acelerador, y muestra
//    roll, pitch, acelerador y el estado del enlace.
//  - El servidor web corre en el núcleo 0; el lazo de control (loop) sigue en el
//    núcleo 1, así que la página no frena el control.
//  - Failsafe: la página pide el estado 5 veces por segundo (latido). Si con el
//    vuelo activo pasan 1 s sin latido (celular bloqueado, fuera de alcance),
//    el acelerador baja solo al 10 %/s hasta detenerse.
//  - Los comandos por Serial0 ('a', 's', '+', '-') siguen funcionando igual.
//
// Placa: "ESP32S3 Dev Module" (núcleo arduino-esp32 3.x). WiFi.h y WebServer.h
// vienen con el núcleo: no hay que instalar librerías nuevas.
// -----------------------------------------------------------------------------

#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <WiFi.h>        // [WiFi]
#include <WebServer.h>   // [WiFi]
#include "pagina.h"       // [WiFi]

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

// --- [WiFi] Red y failsafe ---
const char *WIFI_RED  = "F450-RAV";
const char *WIFI_CLAVE = "f450vuelo";       // mínimo 8 caracteres
#define WIFI_CANAL        6
#define LATIDO_TIMEOUT_MS 1000              // sin latido de la página -> bajada
#define BAJADA_POR_S      0.10f             // del acelerador por segundo
#define THROTTLE_MAX_WEB  0.80f             // mismo tope que el comando '+'

// --- PID ---
// Kp original mantenido porque reportaste que en positivo funciona bien.
// Kd añadido pequeño para amortiguar oscilaciones (antes 0 -> bang-bang).
float Kp_roll  = 0.01;
float Kd_roll  = 0.0006;
float Kp_pitch = 0.005;
float Kd_pitch = 0.0003;

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

// --- Deadband suave: sin escalón al cruzar el umbral ---
static inline float softDeadband(float e, float db) {
  if (fabsf(e) < db) return 0.0f;
  return (e > 0.0f) ? (e - db) : (e + db);
}

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

// --- Estado del vuelo (antes estaba declarado entre setup() y loop();
//     [WiFi] se sube aquí porque el servidor web también lo usa) ---
volatile bool vuelo_activo = false;
float throttle_base = 0.0;
volatile float target_throttle = 0.0;
float deadband_deg = 2.0;

// --- [WiFi] Estado compartido con la página ---
WebServer servidor(80);
volatile float tel_roll = 0, tel_pitch = 0, tel_throttle = 0;
volatile uint32_t ultimoLatido = 0;
volatile bool controlWeb = false;     // true desde el primer latido: activa el failsafe
volatile bool enBajada = false;

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

// ============================== [WiFi] PÁGINA ==============================
// La página está en pagina.h


// [WiFi] Cada petición de la página cuenta como latido
void latido() {
  ultimoLatido = millis();
  controlWeb = true;
}

void webInicio() {
  latido();
  servidor.send_P(200, "text/html", PAGINA);
}

void webEstado() {
  latido();
  char json[160];
  snprintf(json, sizeof(json),
           "{\"roll\":%.2f,\"pitch\":%.2f,\"thr\":%.3f,\"obj\":%.3f,\"activo\":%s,\"bajada\":%s}",
           tel_roll, tel_pitch, tel_throttle, target_throttle,
           vuelo_activo ? "true" : "false", enBajada ? "true" : "false");
  servidor.send(200, "application/json", json);
}

// Mismos comandos que por Serial0, más 't' (acelerador directo desde la barra)
void webComando() {
  latido();
  const String c = servidor.arg("c");
  if (c == "a") {
    enBajada = false;
    vuelo_activo = true;
    target_throttle = 0.15;
  } else if (c == "s") {
    enBajada = false;
    vuelo_activo = false;
    target_throttle = 0.0;
  } else if (c == "+" && !enBajada) {
    target_throttle = min(target_throttle + 0.05f, THROTTLE_MAX_WEB);
  } else if (c == "-" && !enBajada) {
    target_throttle = max(target_throttle - 0.05f, 0.0f);
  } else if (c == "t" && !enBajada) {
    target_throttle = constrain(servidor.arg("v").toFloat(), 0.0f, THROTTLE_MAX_WEB);
  }
  servidor.send(200, "text/plain", "ok");
}

// [WiFi] El servidor vive en el núcleo 0 para no frenar el lazo de control
void tareaWeb(void *) {
  for (;;) {
    servidor.handleClient();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void iniciarWiFi() {
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);                     // sin ahorro de energía: menos latencia
  WiFi.softAP(WIFI_RED, WIFI_CLAVE, WIFI_CANAL);
  servidor.on("/", webInicio);
  servidor.on("/estado", webEstado);
  servidor.on("/cmd", webComando);
  servidor.begin();
  xTaskCreatePinnedToCore(tareaWeb, "web", 8192, NULL, 1, NULL, 0);
  Serial0.print("WiFi: red "); Serial0.print(WIFI_RED);
  Serial0.print(", abrir http://"); Serial0.println(WiFi.softAPIP());
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
    float accel_roll = atan2(ay, az) * RAD_TO_DEG;
    float accel_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

    roll_sum += accel_roll;
    pitch_sum += accel_pitch;
    az_sum += az;

    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;

    delay(5);
  }

  roll_offset = roll_sum / 500.0;
  pitch_offset = pitch_sum / 500.0;
  accel_z_offset = ((az_sum / 500.0) * 9.81) - 9.81;

  gyro_offset_x = gx_sum / 500.0;
  gyro_offset_y = gy_sum / 500.0;
  gyro_offset_z = gz_sum / 500.0;

  Serial0.print("Offset Giro X: "); Serial0.println(gyro_offset_x);
  Serial0.print("Offset Giro Y: "); Serial0.println(gyro_offset_y);

  // [WiFi] Después de calibrar: el arranque del WiFi pide corriente y no debe
  // coincidir con la calibración (el dron tiene que estar quieto)
  iniciarWiFi();
  Serial0.println("Listo. Envia 'a' para armar (o ARMAR en la pagina).");

  current_roll = 0;
  current_pitch = 0;
  last_time = micros();
}

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

  current_roll  = 0.98 * (current_roll  + gx * dt) + 0.02 * accel_roll;
  current_pitch = 0.98 * (current_pitch + gy * dt) + 0.02 * accel_pitch;

  // Restar offset de nivel para que en reposo sea 0
  float angle_roll  = current_roll  - roll_offset;
  float angle_pitch = current_pitch - pitch_offset;

  // --- [WiFi] Failsafe: sin latido de la página, bajada gradual ---
  if (vuelo_activo && controlWeb && millis() - ultimoLatido > LATIDO_TIMEOUT_MS) {
    if (!enBajada) {
      enBajada = true;
      Serial0.println("SIN ENLACE WiFi: bajada automatica");
    }
    target_throttle = max(target_throttle - BAJADA_POR_S * dt, 0.0f);
    if (target_throttle <= 0.0f) {
      vuelo_activo = false;
      enBajada = false;
      Serial0.println("Bajada terminada. Motores en IDLE.");
    }
  }

  // --- SUAVIZADO DEL ARRANQUE ---
  if (vuelo_activo) {
    throttle_base = throttle_base + (target_throttle - throttle_base) * 0.05;
  } else {
    throttle_base = 0.0;
  }

  if (vuelo_activo && throttle_base > 0.01) {
    // Signos como los dejaste: funcionan en positivo.
    //   error_roll  = +angle_roll   (equivalente a tu 0.0 - (-angle_roll))
    //   error_pitch = -angle_pitch
    float raw_error_roll  = angle_roll;
    float raw_error_pitch = -angle_pitch;

    // Deadband suave (sin escalón al cruzar el umbral)
    float error_roll  = softDeadband(raw_error_roll,  deadband_deg);
    float error_pitch = softDeadband(raw_error_pitch, deadband_deg);

    // Filtrar giroscopio para término D
    float gx_filtrado = filtroGx.process(gx);
    float gy_filtrado = filtroGy.process(gy);

    float pid_roll  = (Kp_roll  * error_roll)  - (Kd_roll  * gx_filtrado);
    float pid_pitch = (Kp_pitch * error_pitch) - (Kd_pitch * gy_filtrado);

    float m1_speed = throttle_base - pid_roll + pid_pitch;
    float m2_speed = throttle_base + pid_roll + pid_pitch;
    float m3_speed = throttle_base + pid_roll - pid_pitch;
    float m4_speed = throttle_base - pid_roll - pid_pitch;

    // --- Mezclador proporcional ---
    // Si algún motor cae por debajo de 0, se atenúan TODAS las correcciones
    // en la misma proporción (mantiene el promedio en throttle_base) en vez
    // de recortar uno a uno (que producía el "parpadeo" que notabas).
    float m_min = min(m1_speed, min(m2_speed, min(m3_speed, m4_speed)));
    if (m_min < 0.0f) {
      float denom = throttle_base - m_min;
      float scale = (denom > 1e-6f) ? (throttle_base / denom) : 0.0f;
      m1_speed = throttle_base + (m1_speed - throttle_base) * scale;
      m2_speed = throttle_base + (m2_speed - throttle_base) * scale;
      m3_speed = throttle_base + (m3_speed - throttle_base) * scale;
      m4_speed = throttle_base + (m4_speed - throttle_base) * scale;
    }

    // Recorte superior global: mantiene las diferencias entre motores
    float m_max = max(m1_speed, max(m2_speed, max(m3_speed, m4_speed)));
    if (m_max > 1.0f) {
      float excess = m_max - 1.0f;
      m1_speed -= excess;
      m2_speed -= excess;
      m3_speed -= excess;
      m4_speed -= excess;
    }

    int p_m1 = PULSO_IDLE + (int)(m1_speed * (PULSO_MAX - PULSO_IDLE));
    int p_m2 = PULSO_IDLE + (int)(m2_speed * (PULSO_MAX - PULSO_IDLE));
    int p_m3 = PULSO_IDLE + (int)(m3_speed * (PULSO_MAX - PULSO_IDLE));
    int p_m4 = PULSO_IDLE + (int)(m4_speed * (PULSO_MAX - PULSO_IDLE));

    setMotorPulse(0, p_m1);
    setMotorPulse(1, p_m2);
    setMotorPulse(2, p_m3);
    setMotorPulse(3, p_m4);
  } else {
    setMotorPulse(0, PULSO_IDLE); setMotorPulse(1, PULSO_IDLE);
    setMotorPulse(2, PULSO_IDLE); setMotorPulse(3, PULSO_IDLE);
  }

  // [WiFi] Lo que muestra la página
  tel_roll = angle_roll;
  tel_pitch = angle_pitch;
  tel_throttle = vuelo_activo ? throttle_base : 0.0f;

  // --- Comandos ---
  if (Serial0.available()) {
    char cmd = Serial0.read();
    if (cmd == 'a') {
      enBajada = false;   // [WiFi]
      vuelo_activo = true;
      target_throttle = 0.15;   // antes 0.05 (muy bajo -> saturación del PID)
      Serial0.println("¡Vuelo activo! Throttle inicial 15%");
    } else if (cmd == 's') {
      enBajada = false;   // [WiFi]
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
