// =============================================================================
// Proyecto: Control de Dron F450 con ESP32-S3 y Calibracion de Balanza
// Archivo:  PruebaControlCalibrado.ino
// Placa:    ESP32S3 Dev Module
// =============================================================================
// Pines Hardware:
//  - MPU-6050/6500: Wire  (SDA = GPIO1, SCL = GPIO39)
//  - ToF VL53L0X:   Wire1 (SDA = GPIO4, SCL = GPIO5)
//  - HC-SR04:       TRIG = GPIO6, ECHO = GPIO7 (con divisor a 3.3V)
//  - Motores (PWM): M1 = GPIO8, M2 = GPIO17, M3 = GPIO21, M4 = GPIO18
//
// Curvas de Empuje medidas en Balanza (x = Gas / 5% = (pulso - 1000) / 50):
//  - M1 (Frente-Izq, CW):   F1 = 0.4286*x^2 + 1.2286*x - 1.8
//  - M2 (Frente-Der, CCW):  F2 = 8.9286*x^2 - 19.671*x + 33.2
//  - M3 (Atras-Der,  CW):   F3 = 0.8571*x^2 - 0.9429*x + 0.2
//  - M4 (Atras-Izq,  CCW):  F4 = 0.9286*x^2 - 0.4714*x - 0.2
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_VL53L0X.h>
#include "pagina.h"

// --- Definicion de pines ---
#define TRIG_PIN 6
#define ECHO_PIN 7
#define I2C_SDA 1
#define I2C_SCL 39
#define I2C1_SDA 4
#define I2C1_SCL 5

#define M1_PIN 8
#define M2_PIN 17
#define M3_PIN 21
#define M4_PIN 18

// --- Configuracion PWM Motores ---
#define PWM_FREQ 50          // 50 Hz compatible con todos los ESCs (o 200/400 si BLHeli)
#define PWM_RES 14
const uint32_t DUTY_MAX = (1 << PWM_RES) - 1;
const uint32_t PERIODO_US = 1000000UL / PWM_FREQ;

#define PULSO_MIN  1000
#define PULSO_MAX  1850
#define PULSO_IDLE 1000

// --- Curvas de Calibracion de Balanza ---
struct CurvaMotor {
  float a; float b; float c;
};

const CurvaMotor CURVAS[4] = {
  { 0.4286f,  1.2286f, -1.8f },  // Motor 1
  { 8.9286f, -19.671f,  33.2f }, // Motor 2
  { 0.8571f, -0.9429f,   0.2f }, // Motor 3
  { 0.9286f, -0.4714f,  -0.2f }  // Motor 4
};

// Resuelve la inversa cuadratica: F[g] -> pulso en microsegundos [1000..1850]
int fuerzaAPulso(int motor, float gramos) {
  if (gramos <= 0.0f) return PULSO_IDLE;
  const CurvaMotor &p = CURVAS[motor];
  float disc = p.b * p.b - 4.0f * p.a * (p.c - gramos);
  if (disc < 0.0f) disc = 0.0f;
  float x = (-p.b + sqrtf(disc)) / (2.0f * p.a);
  if (x < 0.0f) x = 0.0f;
  int pulso = 1000 + (int)(x * 50.0f);
  return constrain(pulso, PULSO_MIN, PULSO_MAX);
}

// Inversa: pulso_us -> gramos estimados
float pulsoAFuerza(int motor, int pulso_us) {
  if (pulso_us <= 1000) return 0.0f;
  float x = (pulso_us - 1000) / 50.0f;
  const CurvaMotor &p = CURVAS[motor];
  float f = p.a * x * x + p.b * x + p.c;
  return (f > 0.0f) ? f : 0.0f;
}

// --- Lazo de Control y Filtro Complementario ---
#define LAZO_HZ   500
#define LAZO_US   (1000000UL / LAZO_HZ)
#define TAU_COMP  0.50f

// --- Seguridad ---
#define ANGULO_CORTE   45.0f
#define ANGULO_ARMADO  10.0f

// --- Signos y Modo de Control ---
float signo_roll  = 1.0f;   // Roll negativo (inclinado izq) -> M1 y M4 suben, M2 y M3 bajan
float signo_pitch = 1.0f;
bool modo_linealizar_balanza = false; // FALSE = control directo proporcional (recomendado en balancin)
                                      // TRUE  = compensacion por curva inversa de balanza

// --- Ganancias PID de Actitud ---
float Kp_roll  = 0.0080f;
float Kd_roll  = 0.0015f;
float Ki_roll  = 0.0000f;   // Sin termino integral

float Kp_pitch = 0.0080f;
float Kd_pitch = 0.0015f;
float Ki_pitch = 0.0000f;

float Kp_yaw   = 0.0015f;
#define YAW_OUT_MAX 0.10f

float i_roll = 0.0f, i_pitch = 0.0f;
#define I_OUT_MAX 0.08f

// --- Sensores ---
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
bool tof_conectado = false;
uint16_t distancia_tof_mm = 0;
float distancia_us_cm = 0;

float ax, ay, az;
float rate_roll = 0, rate_pitch = 0, rate_yaw = 0;
float current_roll = 0, current_pitch = 0;
float roll_offset = 0, pitch_offset = 0;
float gyro_offset_r = 0, gyro_offset_p = 0, gyro_offset_y = 0;

class FiltroMA {
  private:
    static const int N = 5;
    float buf[N]; int idx; float sum;
  public:
    FiltroMA() : idx(0), sum(0) { for(int i=0; i<N; i++) buf[i] = 0; }
    float procesar(float val) {
      sum -= buf[idx]; buf[idx] = val; sum += val;
      idx = (idx + 1) % N;
      return sum / N;
    }
};
FiltroMA filtroGx, filtroGy;

static inline float softDeadband(float e, float db) {
  if (fabsf(e) < db) return 0.0f;
  return (e > 0.0f) ? (e - db) : (e + db);
}

// --- Estado del Dron ---
volatile bool vuelo_activo = false;
volatile bool modo_manual = false;  // modo manual para banco de prueba (m1, m2, etc)
float throttle_base = 0.0f;
volatile float target_throttle = 0.0f;
int tel_pulsos[4] = {PULSO_IDLE, PULSO_IDLE, PULSO_IDLE, PULSO_IDLE};
float tel_fuerzas[4] = {0, 0, 0, 0};
float tel_hz = LAZO_HZ;
char motivoCorte[32] = "";

// --- WiFi Web ---
WebServer servidor(80);
volatile uint32_t ultimoLatido = 0;
volatile bool controlWeb = false;

// --- FACTORES DE ESCALA POR MOTOR (COMPENSACION INDIVIDUAL) ---
// Multiplicador sobre la senal por encima de reposo (Delta = pulso - 1000)
// Permite equilibrar motores que entregan mas o menos empuje (ej: M2 al 35%)
float escala_motor[4] = {1.00f, 0.35f, 1.00f, 1.00f};

int aplicarEscalaMotor(int motor, int pulse_us) {
  if (pulse_us <= PULSO_IDLE) return PULSO_IDLE;
  int delta = pulse_us - PULSO_IDLE;
  if (delta <= 0) return PULSO_IDLE;

  int delta_escalado = (int)(delta * escala_motor[motor]);
  return constrain(PULSO_IDLE + delta_escalado, PULSO_MIN, PULSO_MAX);
}

void setMotorPulse(int motor, int pulse_us) {
  pulse_us = constrain(pulse_us, PULSO_MIN, PULSO_MAX);

  // Aplicar factor de escala individual (sin limites artificiales)
  pulse_us = aplicarEscalaMotor(motor, pulse_us);

  tel_pulsos[motor] = pulse_us;
  tel_fuerzas[motor] = pulsoAFuerza(motor, pulse_us);
  uint32_t duty = ((uint32_t)pulse_us * DUTY_MAX) / PERIODO_US;
  switch (motor) {
    case 0: ledcWrite(M1_PIN, duty); break;
    case 1: ledcWrite(M2_PIN, duty); break;
    case 2: ledcWrite(M3_PIN, duty); break;
    case 3: ledcWrite(M4_PIN, duty); break;
  }
}

void pararMotores() {
  vuelo_activo = false;
  modo_manual = false;
  target_throttle = 0.0f;
  throttle_base = 0.0f;
  for (int i = 0; i < 4; i++) setMotorPulse(i, PULSO_IDLE);
}

void cortar(const char *motivo) {
  pararMotores();
  strncpy((char *)motivoCorte, motivo, sizeof(motivoCorte) - 1);
  Serial0.printf("CORTE SEGURIDAD: %s\n", motivo);
}

// --- Lectura de MPU6050/6500 por I2C0 ---
bool leerMPU() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(0x68, 14, true) != 14) return false;

  int16_t raw_ax = (Wire.read() << 8) | Wire.read();
  int16_t raw_ay = (Wire.read() << 8) | Wire.read();
  int16_t raw_az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // temp
  int16_t raw_gx = (Wire.read() << 8) | Wire.read();
  int16_t raw_gy = (Wire.read() << 8) | Wire.read();
  int16_t raw_gz = (Wire.read() << 8) | Wire.read();

  ax = raw_ay / 8192.0f;
  ay = raw_ax / 8192.0f;
  az = raw_az / 8192.0f;

  rate_roll  = -raw_gy / 65.5f;
  rate_pitch = -raw_gx / 65.5f;
  rate_yaw   =  raw_gz / 65.5f;
  return true;
}

void calibrarIMU() {
  Serial0.println("Calibrando IMU (MANTENER QUIETO)...");
  float r_sum = 0, p_sum = 0, gr_sum = 0, gp_sum = 0, gy_sum = 0;
  int n = 0;
  for (int i = 0; i < 400; i++) {
    if (leerMPU()) {
      r_sum  += atan2f(ay, az) * RAD_TO_DEG;
      p_sum  += atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
      gr_sum += rate_roll;
      gp_sum += rate_pitch;
      gy_sum += rate_yaw;
      n++;
    }
    delay(4);
  }
  if (n > 200) {
    roll_offset   = r_sum / n;
    pitch_offset  = p_sum / n;
    gyro_offset_r = gr_sum / n;
    gyro_offset_p = gp_sum / n;
    gyro_offset_y = gy_sum / n;
    Serial0.printf("Offsets Giro: R=%.2f P=%.2f Y=%.2f | Roll/Pitch off: %.2f / %.2f\n",
                   gyro_offset_r, gyro_offset_p, gyro_offset_y, roll_offset, pitch_offset);
  } else {
    Serial0.println("ERROR: No se pudo calibrar la IMU. Revisar I2C (SDA=1, SCL=39)");
  }
}

// --- Ultrasonido HC-SR04 ---
float leerUltrasonido() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 25000); // max ~4.3m
  if (dur == 0) return -1.0f;
  return dur * 0.0343f / 2.0f;
}

// --- Web Handlers ---
void webEstado() {
  ultimoLatido = millis(); controlWeb = true;
  char buf[480];
  snprintf(buf, sizeof(buf),
    "{\"roll\":%.2f,\"pitch\":%.2f,\"thr\":%.2f,\"activo\":%s,\"hz\":%.0f,\"tof\":%u,"
    "\"m\":[%d,%d,%d,%d],\"f\":[%.1f,%.1f,%.1f,%.1f],"
    "\"esc\":[%.2f,%.2f,%.2f,%.2f],"
    "\"pid\":[%.4f,%.4f,%.4f,%.4f,%.4f]}",
    current_roll - roll_offset, current_pitch - pitch_offset, target_throttle,
    vuelo_activo ? "true" : "false", tel_hz, distancia_tof_mm,
    tel_pulsos[0], tel_pulsos[1], tel_pulsos[2], tel_pulsos[3],
    tel_fuerzas[0], tel_fuerzas[1], tel_fuerzas[2], tel_fuerzas[3],
    escala_motor[0], escala_motor[1], escala_motor[2], escala_motor[3],
    Kp_roll, Kd_roll, Kp_pitch, Kd_pitch, Kp_yaw);
  servidor.send(200, "application/json", buf);
}

void webCmd() {
  ultimoLatido = millis(); controlWeb = true;
  String c = servidor.arg("c");
  if (c == "a") {
    if (fabsf(current_roll - roll_offset) > ANGULO_ARMADO || fabsf(current_pitch - pitch_offset) > ANGULO_ARMADO) {
      servidor.send(200, "text/plain", "Inclinado: no arma"); return;
    }
    modo_manual = false;
    vuelo_activo = true;
    target_throttle = 0.15f;
    servidor.send(200, "text/plain", "ARMADO al 15%");
  } else if (c == "s") {
    pararMotores();
    servidor.send(200, "text/plain", "PARADO");
  } else if (c == "up" && vuelo_activo) {
    target_throttle = min(target_throttle + 0.05f, 1.00f);
    servidor.send(200, "text/plain", "Throttle aumentado");
  } else if (c == "down" && vuelo_activo) {
    target_throttle = max(target_throttle - 0.05f, 0.05f);
    servidor.send(200, "text/plain", "Throttle reducido");
  } else if (c == "tare") {
    calibrarIMU();
    servidor.send(200, "text/plain", "IMU Calibrada");
  } else if (c == "inv") {
    signo_roll = -signo_roll;
    servidor.send(200, "text/plain", String("Signo Roll: ") + signo_roll);
  } else if (c == "modo") {
    modo_linealizar_balanza = !modo_linealizar_balanza;
    servidor.send(200, "text/plain", modo_linealizar_balanza ? "Modo: Balanza" : "Modo: Directo");
  } else if (c == "esc") {
    if (servidor.hasArg("e1")) escala_motor[0] = constrain(servidor.arg("e1").toFloat(), 0.0f, 2.0f);
    if (servidor.hasArg("e2")) escala_motor[1] = constrain(servidor.arg("e2").toFloat(), 0.0f, 2.0f);
    if (servidor.hasArg("e3")) escala_motor[2] = constrain(servidor.arg("e3").toFloat(), 0.0f, 2.0f);
    if (servidor.hasArg("e4")) escala_motor[3] = constrain(servidor.arg("e4").toFloat(), 0.0f, 2.0f);
    Serial0.printf("Escalas actualizadas: M1=%.2f M2=%.2f M3=%.2f M4=%.2f\n",
                   escala_motor[0], escala_motor[1], escala_motor[2], escala_motor[3]);
    servidor.send(200, "text/plain", "Escalas actualizadas");
  } else {
    servidor.send(200, "text/plain", "ok");
  }
}

void webPid() {
  ultimoLatido = millis(); controlWeb = true;
  if (servidor.hasArg("kpr")) Kp_roll  = servidor.arg("kpr").toFloat();
  if (servidor.hasArg("kdr")) Kd_roll  = servidor.arg("kdr").toFloat();
  if (servidor.hasArg("kir")) Ki_roll  = servidor.arg("kir").toFloat();
  if (servidor.hasArg("kpp")) Kp_pitch = servidor.arg("kpp").toFloat();
  if (servidor.hasArg("kdp")) Kd_pitch = servidor.arg("kdp").toFloat();
  if (servidor.hasArg("kip")) Ki_pitch = servidor.arg("kip").toFloat();
  if (servidor.hasArg("kpy")) Kp_yaw   = servidor.arg("kpy").toFloat();
  i_roll = 0; i_pitch = 0;
  Serial0.printf("PID actualizado: Kp_r=%.5f Kd_r=%.5f Ki_r=%.5f | Kp_p=%.5f Kd_p=%.5f Ki_p=%.5f\n",
                 Kp_roll, Kd_roll, Ki_roll, Kp_pitch, Kd_pitch, Ki_pitch);
  servidor.send(200, "text/plain", "PID actualizado");
}

void iniciarWiFi() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP("F450-TEST", "f450vuelo", 6);
  servidor.on("/", []() { servidor.send_P(200, "text/html", PAGINA); });
  servidor.on("/estado", webEstado);
  servidor.on("/cmd", webCmd);
  servidor.on("/pid", webPid);
  servidor.begin();
  xTaskCreatePinnedToCore([](void *) { for (;;) { servidor.handleClient(); vTaskDelay(pdMS_TO_TICKS(2)); } },
                          "web", 8192, NULL, 1, NULL, 0);
  Serial0.print("WiFi AP listo: F450-TEST | Abrir http://");
  Serial0.println(WiFi.softAPIP());
}

// --- Comandos por Serial ---
void procesarSerial(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "inv" || cmd == "signo") {
    signo_roll = -signo_roll;
    Serial0.printf("SIGNO ROLL INVERTIDO: ahora es %.1f\n", signo_roll);
  } else if (cmd == "modo") {
    modo_linealizar_balanza = !modo_linealizar_balanza;
    Serial0.printf("MODO DE CONTROL: %s\n", modo_linealizar_balanza ? "LINEALIZACION POR BALANZA" : "PROPORCIONAL DIRECTO (BALANCIN)");
  } else if (cmd == "a") {
    modo_manual = false;
    vuelo_activo = true;  
    target_throttle = 0.15f;
    Serial0.println("ARMADO en modo PID estabilizado (15% throttle).");
  } else if (cmd == "s") {
    pararMotores();
    Serial0.println("PARADA: Motores en reposo (1000 us).");
  } else if (cmd == "+") {
    target_throttle = min(target_throttle + 0.05f, 1.00f);
    Serial0.printf("Throttle: %.0f%%\n", target_throttle * 100);
  } else if (cmd == "-") {
    target_throttle = max(target_throttle - 0.05f, 0.0f);
    Serial0.printf("Throttle: %.0f%%\n", target_throttle * 100);
  } else if (cmd.startsWith("ki ")) {
    float k = cmd.substring(3).toFloat();
    Ki_roll = k;
    Ki_pitch = k;
    i_roll = 0.0f; i_pitch = 0.0f;
    Serial0.printf("Ki configurado a %.5f (Roll y Pitch)\n", k);
  } else if (cmd.startsWith("esc ")) {
    // Formato: esc <1-4> <valor>  ej: esc 2 0.35
    int sp1 = cmd.indexOf(' ');
    int sp2 = cmd.indexOf(' ', sp1 + 1);
    if (sp2 > 0) {
      int m = cmd.substring(sp1 + 1, sp2).toInt() - 1;
      float val = cmd.substring(sp2 + 1).toFloat();
      if (m >= 0 && m < 4) {
        escala_motor[m] = constrain(val, 0.0f, 2.0f);
        Serial0.printf("Escala M%d ajustada a %.2f (%.0f%%)\n", m + 1, escala_motor[m], escala_motor[m] * 100);
      }
    }
  } else if (cmd.startsWith("m ")) {
    // Formato: m <1-4> <gramos>   ej: m 1 10
    int sp1 = cmd.indexOf(' ');
    int sp2 = cmd.indexOf(' ', sp1 + 1);
    if (sp2 > 0) {
      int m = cmd.substring(sp1 + 1, sp2).toInt() - 1;
      float g = cmd.substring(sp2 + 1).toFloat();
      if (m >= 0 && m < 4) {
        modo_manual = true;
        vuelo_activo = false;
        int p = fuerzaAPulso(m, g);
        setMotorPulse(m, p);
        Serial0.printf("MANUAL: Motor M%d -> %.1f g (Pulso: %d us)\n", m + 1, g, p);
      }
    }
  } else if (cmd.startsWith("todos ")) {
    // Formato: todos <gramos>   ej: todos 15
    float g = cmd.substring(6).toFloat();
    modo_manual = true;
    vuelo_activo = false;
    for (int i = 0; i < 4; i++) {
      int p = fuerzaAPulso(i, g);
      setMotorPulse(i, p);
      Serial0.printf("  M%d -> %.1f g (Pulso %d us)\n", i + 1, g, p);
    }
    Serial0.println("Los 4 motores ajustados por calibracion.");
  } else if (cmd == "diag") {
    Serial0.println("\n--- DIAGNOSTICO DE SENSORES ---");
    Serial0.printf("MPU Roll: %.2f | Pitch: %.2f | Rate Yaw: %.1f dps\n",
                   current_roll - roll_offset, current_pitch - pitch_offset, rate_yaw);
    if (tof_conectado) {
      VL53L0X_RangingMeasurementData_t measure;
      lox.rangingTest(&measure, false);
      Serial0.printf("ToF Distancia: %d mm\n", (measure.RangeStatus != 4) ? measure.RangeMilliMeter : -1);
    }
    Serial0.printf("Ultrasonido Distancia: %.1f cm\n", leerUltrasonido());
    Serial0.println("--------------------------------\n");
  } else if (cmd == "ayuda") {
    Serial0.println("\nComandos Serial:");
    Serial0.println("  a            -> Armar control PID estabilizado");
    Serial0.println("  s            -> Parar motores inmediatamente");
    Serial0.println("  + / -        -> Subir/bajar acelerador");
    Serial0.println("  esc <1-4> <val> -> Escala individual de motor (ej: esc 2 0.35)");
    Serial0.println("  m <1-4> <g>  -> Probar motor individual a cierta fuerza en gramos (ej: m 1 10)");
    Serial0.println("  todos <g>    -> Probar los 4 motores al mismo empuje en gramos (ej: todos 15)");
    Serial0.println("  diag         -> Imprimir lecturas de sensores (MPU, ToF, US)");
    Serial0.println("  ayuda        -> Mostrar este menu\n");
  }
}

void setup() {
  Serial0.begin(115200);
  delay(1000);

  // Inicializar PWM motores
  ledcAttach(M1_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(M2_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(M3_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(M4_PIN, PWM_FREQ, PWM_RES);
  pararMotores();

  // Pines de Ultrasonido
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Inicializar MPU (I2C0: SDA 1, SCL 39)
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(); // despertar
  delay(50);
  Wire.beginTransmission(0x68); Wire.write(0x1A); Wire.write(0x02); Wire.endTransmission(); // DLPF 92Hz
  Wire.beginTransmission(0x68); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission(); // +-500 dps
  Wire.beginTransmission(0x68); Wire.write(0x1C); Wire.write(0x08); Wire.endTransmission(); // +-4 g

  // Inicializar ToF (I2C1: SDA 4, SCL 5)
  Wire1.begin(I2C1_SDA, I2C1_SCL);
  if (lox.begin(0x29, false, &Wire1)) {
    tof_conectado = true;
    Serial0.println("ToF VL53L0X inicializado correctamente.");
  } else {
    Serial0.println("AVISO: ToF no detectado en I2C1 (GPIO4/5).");
  }

  calibrarIMU();
  iniciarWiFi();

  Serial0.println("\n=== Sistema Listo ===");
  Serial0.println("Escribe 'ayuda' en el monitor serial para ver comandos.");
}

unsigned long last_time = 0;
unsigned long telemetria_timer = 0;
unsigned long sensor_timer = 0;

void loop() {
  while (micros() - last_time < LAZO_US) { }
  unsigned long current_time = micros();
  float dt = (current_time - last_time) / 1000000.0f;
  last_time = current_time;
  if (dt > 0.01f) dt = 0.01f;

  tel_hz += 0.01f * (1.0f / dt - tel_hz);

  // Lectura IMU
  if (leerMPU()) {
    rate_roll  -= gyro_offset_r;
    rate_pitch -= gyro_offset_p;
    rate_yaw   -= gyro_offset_y;

    float acc_roll  = atan2f(ay, az) * RAD_TO_DEG;
    float acc_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
    const float alfa = TAU_COMP / (TAU_COMP + dt);
    current_roll  = alfa * (current_roll  + rate_roll  * dt) + (1.0f - alfa) * acc_roll;
    current_pitch = alfa * (current_pitch + rate_pitch * dt) + (1.0f - alfa) * acc_pitch;
  }

  float ang_r = current_roll - roll_offset;
  float ang_p = current_pitch - pitch_offset;

  // Corte por inclinacion excesiva en vuelo
  if (vuelo_activo && (fabsf(ang_r) > ANGULO_CORTE || fabsf(ang_p) > ANGULO_CORTE)) {
    cortar("Inclinacion > 45 deg");
  }

  // Failsafe WiFi: si se maneja por web y pierde latido por 1 s, baja suave
  if (vuelo_activo && controlWeb && millis() - ultimoLatido > 1000) {
    target_throttle = max(target_throttle - 0.10f * dt, 0.0f);
    if (target_throttle <= 0.02f) pararMotores();
  }

  // Suavizado del throttle
  if (vuelo_activo) {
    throttle_base += (target_throttle - throttle_base) * 0.02f;
  }

  // LAZO DE CONTROL PID (Soporta modo proporcional directo y modo balanza)
  if (vuelo_activo && throttle_base > 0.02f) {
    float err_r = softDeadband(ang_r, 0.5f);
    float err_p = softDeadband(-ang_p, 0.5f);

    float derr_r = filtroGx.procesar(rate_roll);
    float derr_p = -filtroGy.procesar(rate_pitch);

    // pid_r y pid_p con signos configurables (signo_roll = -1.0f invierte el signo)
    float pid_r = signo_roll  * ((Kp_roll  * err_r) + i_roll  + (Kd_roll  * derr_r));
    float pid_p = signo_pitch * ((Kp_pitch * err_p) + i_pitch + (Kd_pitch * derr_p));
    float pid_y = constrain(-Kp_yaw * rate_yaw, -YAW_OUT_MAX, YAW_OUT_MAX);

    // Distribucion en X:
    // M1 (frente-izq), M2 (frente-der), M3 (atras-der), M4 (atras-izq)
    float m1 = throttle_base - pid_r + pid_p + pid_y;
    float m2 = throttle_base + pid_r + pid_p - pid_y;
    float m3 = throttle_base + pid_r - pid_p + pid_y;
    float m4 = throttle_base - pid_r - pid_p - pid_y;

    // Desaturacion proporcional: preserva la accion de control diferencial
    float m_min = min(min(m1, m2), min(m3, m4));
    bool sat = (m_min < 0.0f);
    if (m_min < 0.0f) {
      float denom = throttle_base - m_min;
      float scale = (denom > 1e-6f) ? (throttle_base / denom) : 0.0f;
      m1 = throttle_base + (m1 - throttle_base) * scale;
      m2 = throttle_base + (m2 - throttle_base) * scale;
      m3 = throttle_base + (m3 - throttle_base) * scale;
      m4 = throttle_base + (m4 - throttle_base) * scale;
    }
    float m_max = max(max(m1, m2), max(m3, m4));
    if (m_max > 1.0f) {
      sat = true;
      float ex = m_max - 1.0f;
      m1 -= ex; m2 -= ex; m3 -= ex; m4 -= ex;
    }

    if (!sat) {
      i_roll  = constrain(i_roll  + Ki_roll  * ang_r    * dt, -I_OUT_MAX, I_OUT_MAX);
      i_pitch = constrain(i_pitch + Ki_pitch * (-ang_p) * dt, -I_OUT_MAX, I_OUT_MAX);
    }

    if (modo_linealizar_balanza) {
      const float F_MAX_G = 300.0f;
      setMotorPulse(0, fuerzaAPulso(0, m1 * F_MAX_G));
      setMotorPulse(1, fuerzaAPulso(1, m2 * F_MAX_G));
      setMotorPulse(2, fuerzaAPulso(2, m3 * F_MAX_G));
      setMotorPulse(3, fuerzaAPulso(3, m4 * F_MAX_G));
    } else {
      // Modo proporcional directo (el del balancin probado en PrimerVueloWiFi_PIDv2)
      setMotorPulse(0, PULSO_IDLE + (int)(constrain(m1, 0.0f, 1.0f) * (PULSO_MAX - PULSO_IDLE)));
      setMotorPulse(1, PULSO_IDLE + (int)(constrain(m2, 0.0f, 1.0f) * (PULSO_MAX - PULSO_IDLE)));
      setMotorPulse(2, PULSO_IDLE + (int)(constrain(m3, 0.0f, 1.0f) * (PULSO_MAX - PULSO_IDLE)));
      setMotorPulse(3, PULSO_IDLE + (int)(constrain(m4, 0.0f, 1.0f) * (PULSO_MAX - PULSO_IDLE)));
    }
  } else if (!modo_manual) {
    i_roll = 0; i_pitch = 0;
    for (int i = 0; i < 4; i++) setMotorPulse(i, PULSO_IDLE);
  }

  // Lectura periodica de sensores lentos (ToF y US) cada 100 ms
  if (millis() - sensor_timer > 100) {
    sensor_timer = millis();
    if (tof_conectado) {
      VL53L0X_RangingMeasurementData_t measure;
      lox.rangingTest(&measure, false);
      if (measure.RangeStatus != 4) distancia_tof_mm = measure.RangeMilliMeter;
    }
  }

  // Monitor Serial cada 150 ms
  if (millis() - telemetria_timer > 150) {
    telemetria_timer = millis();
    if (vuelo_activo || modo_manual) {
      Serial0.printf("R:%5.1f P:%5.1f | I_r:%+.3f | M1:%4d M2:%4d M3:%4d M4:%4d\n",
                     ang_r, ang_p, i_roll,
                     tel_pulsos[0], tel_pulsos[1], tel_pulsos[2], tel_pulsos[3]);
    }
  }

  // Leer comandos del monitor serial
  if (Serial0.available()) {
    procesarSerial(Serial0.readStringUntil('\n'));
  }
}
