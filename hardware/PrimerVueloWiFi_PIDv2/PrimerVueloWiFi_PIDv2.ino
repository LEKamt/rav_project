// PrimerVueloWiFi_PIDv2
// -----------------------------------------------------------------------------
// Parte de hardware/PrimerVueloWiFi_PID (PID + sintonia web). Los cambios van
// marcados con [V2]; lo demas queda como estaba.
//
// [V2] Correcciones antes de volar:
//  1. Signo del giroscopio. Antes se hacia ax=raw_ay, ay=raw_ax y lo mismo con el
//     giro. Eso es un ESPEJO, no una rotacion: con el acelerometro el angulo
//     salia +theta y el giro integraba -theta. El filtro complementario se peleaba
//     consigo mismo (angulo con mucho retraso) y la D de pitch EMPUJABA en vez de
//     amortiguar (por eso pitch necesitaba la mitad de ganancia que roll).
//     Ahora rate_roll y rate_pitch son la derivada exacta de angle_roll y
//     angle_pitch. El angulo del acelerometro no cambia, asi que el signo de la P
//     (el que ya probaron) se mantiene.
//  2. Rango de pulso de 1000 a 1850 us (antes el tope era 1600 us, y con el limite
//     del 80 % llegaba solo a 1480 us: con 1045 + A2212 a 3S el hover esta cerca
//     de 1500-1560 us). Las ganancias por defecto se reescalaron para que den los
//     mismos us por grado que antes.
//  3. I_OUT_MAX = 0.08 (antes 0.8 = 80 % del acelerador; el comentario decia 8 %).
//  4. Control de guinada (solo P sobre la velocidad de giro) y Kp_yaw en la pagina.
//  5. Lazo fijo a 500 Hz, I2C a 400 kHz, DLPF en la IMU y PWM a 400 Hz (PWM_FREQ).
//  6. Seguridad: ARMAR no hace nada si ya esta volando (antes bajaba el acelerador
//     al 15 % en pleno vuelo); corte por inclinacion > 50 grados; corte si la IMU
//     deja de responder; no arma si esta inclinado mas de 10 grados.
//  7. La pagina muestra los 4 pulsos de motor y la frecuencia del lazo: sirven
//     para verificar signos SIN helices.
//
// Placa: "ESP32S3 Dev Module" (arduino-esp32 3.x).
// -----------------------------------------------------------------------------

#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <WiFi.h>
#include <WebServer.h>
#include "pagina.h"

// --- Definicion de pines ---
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

// --- Configuracion PWM ---
// [V2] 400 Hz. Si algun ESC no lo acepta (motor que tartamudea o no arranca en la
// prueba sin helices), volver a 50.
#define PWM_FREQ 400
#define PWM_RES 14
const uint32_t DUTY_MAX = (1 << PWM_RES) - 1;
const uint32_t PERIODO_US = 1000000UL / PWM_FREQ;

#define PULSO_MIN 1000
#define PULSO_MAX 1850   // [V2] antes 1600
#define PULSO_IDLE 1000

// --- [V2] Lazo de control ---
#define LAZO_HZ   500
#define LAZO_US   (1000000UL / LAZO_HZ)
#define TAU_COMP  0.5f      // constante de tiempo del filtro complementario [s]

// --- [V2] Seguridad ---
#define ANGULO_CORTE   50.0f   // inclinacion que corta motores en vuelo [grados]
#define ANGULO_ARMADO  10.0f   // no arma si esta mas inclinado que esto
#define IMU_FALLAS_MAX 10      // lecturas fallidas seguidas que cortan motores

// --- [V2] Guinada ---
// M1 y M3 giran en sentido horario (vistos desde arriba), M2 y M4 antihorario.
// SIGNO_YAW = +1 supone el chip con la cara hacia arriba (az ~ +1 g en reposo).
// Se verifica sin helices (ver PRUEBAS.md).
#define SIGNO_YAW   1.0f
#define YAW_OUT_MAX 0.10f

// --- WiFi y failsafe ---
const char *WIFI_RED  = "F450-RAV";
const char *WIFI_CLAVE = "f450vuelo";
#define WIFI_CANAL        6
#define LATIDO_TIMEOUT_MS 1000
#define BAJADA_POR_S      0.10f
#define THROTTLE_MAX_WEB  0.85f   // [V2] 0.85 * 850 us = 1722 us

// --- PID ---
// [V2] Mismos us por grado que en la version anterior (0.01 * 600 us = 0.007 * 850 us).
// Pitch queda igual que roll: el F450 en X es simetrico y la D de pitch ya amortigua.
float Kp_roll  = 0.0070;
float Kd_roll  = 0.00040;
float Kp_pitch = 0.0070;
float Kd_pitch = 0.00040;
float Ki_roll  = 0.0;
float Ki_pitch = 0.0;
float Kp_yaw   = 0.0015;   // [V2] por grado/s
float i_roll = 0, i_pitch = 0;
volatile bool resetI = false;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
float ax, ay, az;
float rate_roll, rate_pitch, rate_yaw;   // [V2] grados/s, derivada de angle_roll / angle_pitch
float current_roll = 0, current_pitch = 0;
float roll_offset = 0, pitch_offset = 0;
float gyro_offset_r = 0, gyro_offset_p = 0, gyro_offset_y = 0;
float az_reposo = 1.0f;

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

static inline float softDeadband(float e, float db) {
  if (fabsf(e) < db) return 0.0f;
  return (e > 0.0f) ? (e - db) : (e + db);
}

unsigned long last_time = 0;

// --- Estado del vuelo ---
volatile bool vuelo_activo = false;
float throttle_base = 0.0;
volatile float target_throttle = 0.0;
float deadband_deg = 0.5;   // [V2] antes 2.0: dentro de la banda no hay P y el dron deriva

// --- Estado compartido con la pagina ---
WebServer servidor(80);
volatile float tel_roll = 0, tel_pitch = 0, tel_throttle = 0;
volatile int tel_m[4] = {PULSO_IDLE, PULSO_IDLE, PULSO_IDLE, PULSO_IDLE};  // [V2]
volatile float tel_hz = 0;                                                // [V2]
volatile uint32_t ultimoLatido = 0;
volatile bool controlWeb = false;
volatile bool enBajada = false;
volatile char motivoCorte[40] = "";   // [V2] ultimo corte automatico

// --- Limites duros de ganancias aceptadas desde la web ---
#define KP_MAX 0.050f    // [V2] antes 3.0: 0.05/grado ya es 42 us por grado
#define KD_MAX 0.005f    // [V2] antes 1.0
#define KI_MAX 0.050f
#define KY_MAX 0.010f    // [V2]
#define DB_MAX 15.0f
#define I_OUT_MAX 0.08f  // [V2] antes 0.8

#define LOG_N 128
volatile int16_t logR[LOG_N], logP[LOG_N];
volatile uint32_t logSeq = 0;

// [V2] Devuelve false si la lectura fallo.
bool leerMPU6500() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(0x68, 14, true) != 14) return false;
  int16_t raw_ax = (Wire.read() << 8) | Wire.read();
  int16_t raw_ay = (Wire.read() << 8) | Wire.read();
  int16_t raw_az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // temperatura
  int16_t raw_gx = (Wire.read() << 8) | Wire.read();
  int16_t raw_gy = (Wire.read() << 8) | Wire.read();
  int16_t raw_gz = (Wire.read() << 8) | Wire.read();

  // Acelerometro: igual que antes (ax = raw_ay, ay = raw_ax), asi que
  // angle_roll y angle_pitch tienen el mismo signo que en la version probada.
  ax = raw_ay / 8192.0;
  ay = raw_ax / 8192.0;
  az = raw_az / 8192.0;

  // [V2] Giroscopio coherente con esos angulos:
  //   roll  = atan2(raw_ax, raw_az)  -> d(roll)/dt  = -raw_gy
  //   pitch = atan2(-raw_ay, ...)    -> d(pitch)/dt = -raw_gx
  // (antes se usaba +raw_gy y +raw_gx: signo opuesto al del acelerometro)
  rate_roll  = -raw_gy / 65.5;
  rate_pitch = -raw_gx / 65.5;
  rate_yaw   = SIGNO_YAW * raw_gz / 65.5;   // + = antihorario visto desde arriba
  return true;
}

void setMotorPulse(int motor, int pulse_us) {
  if(pulse_us < PULSO_MIN) pulse_us = PULSO_MIN;
  if(pulse_us > PULSO_MAX) pulse_us = PULSO_MAX;
  tel_m[motor] = pulse_us;
  uint32_t duty = ((uint32_t)pulse_us * DUTY_MAX) / PERIODO_US;
  switch(motor) {
    case 0: ledcWrite(M1_PIN, duty); break;
    case 1: ledcWrite(M2_PIN, duty); break;
    case 2: ledcWrite(M3_PIN, duty); break;
    case 3: ledcWrite(M4_PIN, duty); break;
  }
}

// [V2] Corte inmediato con motivo (se ve en la pagina y por Serial)
void cortar(const char *motivo) {
  vuelo_activo = false;
  enBajada = false;
  target_throttle = 0.0f;
  strncpy((char *)motivoCorte, motivo, sizeof(motivoCorte) - 1);
  Serial0.print("CORTE: "); Serial0.println(motivo);
}

// [V2] Armar solo si esta detenido y nivelado. Devuelve el motivo si no arma.
const char *intentarArmar(float roll, float pitch) {
  if (vuelo_activo) return "ya esta activo";
  if (fabsf(roll) > ANGULO_ARMADO || fabsf(pitch) > ANGULO_ARMADO) return "inclinado: no arma";
  enBajada = false;
  motivoCorte[0] = '\0';
  target_throttle = 0.15;
  vuelo_activo = true;
  return "armado";
}

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
  uint32_t hasta = logSeq;
  uint32_t desde = (uint32_t)servidor.arg("s").toInt();
  if (desde > hasta) desde = hasta;
  if (hasta - desde > 60) desde = hasta - 60;
  String r = "", p = "";
  for (uint32_t i = desde; i < hasta; i++) {
    if (i > desde) { r += ','; p += ','; }
    r += (int)logR[i % LOG_N];
    p += (int)logP[i % LOG_N];
  }
  char cab[320];
  snprintf(cab, sizeof(cab),
           "{\"roll\":%.2f,\"pitch\":%.2f,\"thr\":%.3f,\"obj\":%.3f,\"activo\":%s,\"bajada\":%s,\"seq\":%lu,"
           "\"m\":[%d,%d,%d,%d],\"hz\":%.0f,\"corte\":\"%s\",",
           tel_roll, tel_pitch, tel_throttle, target_throttle,
           vuelo_activo ? "true" : "false", enBajada ? "true" : "false", (unsigned long)hasta,
           tel_m[0], tel_m[1], tel_m[2], tel_m[3], tel_hz, (const char *)motivoCorte);
  char k[180];
  // orden: kpr, kdr, kir, kpp, kdp, kip, db, kpy
  snprintf(k, sizeof(k), "\"k\":[%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.2f,%.5f],",
           Kp_roll, Kd_roll, Ki_roll, Kp_pitch, Kd_pitch, Ki_pitch, deadband_deg, Kp_yaw);
  String json = String(cab) + k + "\"r\":[" + r + "],\"p\":[" + p + "]}";
  servidor.send(200, "application/json", json);
}

void webComando() {
  latido();
  const String c = servidor.arg("c");
  if (c == "a") {
    servidor.send(200, "text/plain", intentarArmar(tel_roll, tel_pitch));
    return;
  } else if (c == "s") {
    enBajada = false;
    vuelo_activo = false;
    target_throttle = 0.0;
  } else if ((c == "up" || c == "+" || c == " ") && !enBajada && vuelo_activo) {
    target_throttle = min(target_throttle + 0.05f, THROTTLE_MAX_WEB);
  } else if ((c == "down" || c == "-") && !enBajada) {
    target_throttle = max(target_throttle - 0.05f, 0.0f);
  } else if (c == "t" && !enBajada && vuelo_activo) {
    target_throttle = constrain(servidor.arg("v").toFloat(), 0.0f, THROTTLE_MAX_WEB);
  }
  servidor.send(200, "text/plain", "ok");
}

void webPid() {
  latido();
  if (servidor.hasArg("kpr")) Kp_roll     = constrain(servidor.arg("kpr").toFloat(), 0.0f, KP_MAX);
  if (servidor.hasArg("kdr")) Kd_roll     = constrain(servidor.arg("kdr").toFloat(), 0.0f, KD_MAX);
  if (servidor.hasArg("kir")) Ki_roll     = constrain(servidor.arg("kir").toFloat(), 0.0f, KI_MAX);
  if (servidor.hasArg("kpp")) Kp_pitch    = constrain(servidor.arg("kpp").toFloat(), 0.0f, KP_MAX);
  if (servidor.hasArg("kdp")) Kd_pitch    = constrain(servidor.arg("kdp").toFloat(), 0.0f, KD_MAX);
  if (servidor.hasArg("kip")) Ki_pitch    = constrain(servidor.arg("kip").toFloat(), 0.0f, KI_MAX);
  if (servidor.hasArg("kpy")) Kp_yaw      = constrain(servidor.arg("kpy").toFloat(), 0.0f, KY_MAX);
  if (servidor.hasArg("db"))  deadband_deg = constrain(servidor.arg("db").toFloat(),  0.0f, DB_MAX);
  resetI = true;
  Serial0.printf("PID web: Kp_r=%.5f Kd_r=%.5f Ki_r=%.5f Kp_p=%.5f Kd_p=%.5f Ki_p=%.5f Kp_y=%.5f db=%.2f\n",
                 Kp_roll, Kd_roll, Ki_roll, Kp_pitch, Kd_pitch, Ki_pitch, Kp_yaw, deadband_deg);
  servidor.send(200, "text/plain", "ok");
}

void tareaWeb(void *) {
  for (;;) {
    servidor.handleClient();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void iniciarWiFi() {
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAP(WIFI_RED, WIFI_CLAVE, WIFI_CANAL);
  servidor.on("/", webInicio);
  servidor.on("/estado", webEstado);
  servidor.on("/cmd", webComando);
  servidor.on("/pid", webPid);
  servidor.begin();
  xTaskCreatePinnedToCore(tareaWeb, "web", 8192, NULL, 1, NULL, 0);
  Serial0.print("WiFi: red "); Serial0.print(WIFI_RED);
  Serial0.print(", abrir http://"); Serial0.println(WiFi.softAPIP());
}

void escribirIMU(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(0x68); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}

void setup() {
  Serial0.begin(115200);
  delay(1000);

  ledcAttach(M1_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(M2_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(M3_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(M4_PIN, PWM_FREQ, PWM_RES);

  // Solo pulso minimo para armar los ESC (nunca PULSO_MAX al arrancar)
  Serial0.println("Armando ESCs (solo pulso minimo)...");
  for(int i=0; i<4; i++) setMotorPulse(i, PULSO_MIN);
  delay(3000);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);              // [V2] lectura de ~0.4 ms en vez de ~1.5 ms
  escribirIMU(0x6B, 0x00);            // despertar
  delay(50);
  escribirIMU(0x1A, 0x02);            // [V2] DLPF giro 92 Hz (3.9 ms de retraso)
  escribirIMU(0x1B, 0x08);            // giro +-500 dps
  escribirIMU(0x1C, 0x08);            // acel +-4 g
  escribirIMU(0x1D, 0x05);            // [V2] DLPF acel 10 Hz (solo corrige la deriva)

  Wire1.begin(I2C1_SDA, I2C1_SCL);
  lox.begin(0x29, false, &Wire1);

  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);

  Serial0.println("Calibrando sensores (NO MOVER)...");
  float roll_sum = 0, pitch_sum = 0, az_sum = 0;
  float gr_sum = 0, gp_sum = 0, gy_sum = 0;
  int n = 0;
  for (int i = 0; i < 500; i++) {
    if (leerMPU6500()) {
      roll_sum  += atan2(ay, az) * RAD_TO_DEG;
      pitch_sum += atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
      az_sum += az;
      gr_sum += rate_roll; gp_sum += rate_pitch; gy_sum += rate_yaw;
      n++;
    }
    delay(5);
  }
  if (n < 400) {
    // [V2] Sin IMU no se vuela: se queda aqui con los motores en minimo
    Serial0.println("ERROR: la IMU no responde. Revisar cableado (SDA 1, SCL 39).");
    for (;;) { delay(1000); }
  }
  roll_offset  = roll_sum / n;
  pitch_offset = pitch_sum / n;
  az_reposo    = az_sum / n;
  gyro_offset_r = gr_sum / n;
  gyro_offset_p = gp_sum / n;
  gyro_offset_y = gy_sum / n;

  Serial0.printf("Offsets giro r/p/y: %.2f %.2f %.2f  az en reposo: %.3f g\n",
                 gyro_offset_r, gyro_offset_p, gyro_offset_y, az_reposo);
  if (az_reposo < 0.8f)
    Serial0.println("AVISO: az en reposo no es ~+1 g. Chip boca abajo? Revisar SIGNO_YAW y angulos.");

  iniciarWiFi();
  Serial0.println("Listo. Envia 'a' para armar (o ARMAR en la pagina).");

  current_roll = 0;
  current_pitch = 0;
  last_time = micros();
}

void loop() {
  // [V2] Lazo de frecuencia fija
  while (micros() - last_time < LAZO_US) { }
  unsigned long current_time = micros();
  float dt = (current_time - last_time) / 1000000.0;
  last_time = current_time;
  if (dt > 0.01) dt = 0.01;

  static float hz_filtrado = LAZO_HZ;
  hz_filtrado += 0.01f * (1.0f / dt - hz_filtrado);
  tel_hz = hz_filtrado;

  // [V2] IMU con deteccion de fallas
  static int fallasIMU = 0;
  if (!leerMPU6500()) {
    if (++fallasIMU >= IMU_FALLAS_MAX && vuelo_activo) cortar("IMU sin respuesta");
  } else {
    fallasIMU = 0;
  }

  rate_roll  -= gyro_offset_r;
  rate_pitch -= gyro_offset_p;
  rate_yaw   -= gyro_offset_y;

  // --- FILTRO COMPLEMENTARIO ---
  float accel_roll  = atan2(ay, az) * RAD_TO_DEG;
  float accel_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
  const float alfa = TAU_COMP / (TAU_COMP + dt);   // [V2] ~0.996 a 500 Hz
  current_roll  = alfa * (current_roll  + rate_roll  * dt) + (1 - alfa) * accel_roll;
  current_pitch = alfa * (current_pitch + rate_pitch * dt) + (1 - alfa) * accel_pitch;

  float angle_roll  = current_roll  - roll_offset;
  float angle_pitch = current_pitch - pitch_offset;

  // [V2] Corte por inclinacion excesiva
  if (vuelo_activo && (fabsf(angle_roll) > ANGULO_CORTE || fabsf(angle_pitch) > ANGULO_CORTE))
    cortar("inclinacion > 50 grados");

  // --- Failsafe: sin latido de la pagina, bajada gradual ---
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

  if (resetI) { i_roll = 0; i_pitch = 0; resetI = false; }

  // --- Suavizado del arranque ---
  // [V2] 0.02 por ciclo a 500 Hz = misma constante de tiempo (~50 ms) que antes
  if (vuelo_activo) {
    throttle_base = throttle_base + (target_throttle - throttle_base) * 0.02;
  } else {
    throttle_base = 0.0;
  }

  if (vuelo_activo && throttle_base > 0.01) {
    // Signos de la P como los dejaron (funcionan): error_roll = +angle_roll,
    // error_pitch = -angle_pitch.
    float raw_error_roll  = angle_roll;
    float raw_error_pitch = -angle_pitch;

    float error_roll  = softDeadband(raw_error_roll,  deadband_deg);
    float error_pitch = softDeadband(raw_error_pitch, deadband_deg);

    // [V2] D = Kd * d(error)/dt, con la derivada medida por el giro:
    //   d(error_roll)/dt  = +rate_roll
    //   d(error_pitch)/dt = -rate_pitch
    // Asi la D siempre tiene el mismo signo que la P y amortigua en ambos ejes.
    float derr_roll  = filtroGx.process(rate_roll);
    float derr_pitch = -filtroGy.process(rate_pitch);

    float pid_roll  = (Kp_roll  * error_roll)  + i_roll  + (Kd_roll  * derr_roll);
    float pid_pitch = (Kp_pitch * error_pitch) + i_pitch + (Kd_pitch * derr_pitch);

    // [V2] Guinada: frena el giro. rate_yaw > 0 (antihorario) -> acelerar M2 y M4
    // (helices antihorarias, su reaccion empuja el cuerpo en sentido horario).
    float pid_yaw = constrain(-Kp_yaw * rate_yaw, -YAW_OUT_MAX, YAW_OUT_MAX);

    float m1_speed = throttle_base - pid_roll + pid_pitch + pid_yaw;
    float m2_speed = throttle_base + pid_roll + pid_pitch - pid_yaw;
    float m3_speed = throttle_base + pid_roll - pid_pitch + pid_yaw;
    float m4_speed = throttle_base - pid_roll - pid_pitch - pid_yaw;

    // --- Mezclador proporcional ---
    float m_min = min(m1_speed, min(m2_speed, min(m3_speed, m4_speed)));
    bool saturado = (m_min < 0.0f);
    if (m_min < 0.0f) {
      float denom = throttle_base - m_min;
      float scale = (denom > 1e-6f) ? (throttle_base / denom) : 0.0f;
      m1_speed = throttle_base + (m1_speed - throttle_base) * scale;
      m2_speed = throttle_base + (m2_speed - throttle_base) * scale;
      m3_speed = throttle_base + (m3_speed - throttle_base) * scale;
      m4_speed = throttle_base + (m4_speed - throttle_base) * scale;
    }

    float m_max = max(m1_speed, max(m2_speed, max(m3_speed, m4_speed)));
    if (m_max > 1.0f) {
      saturado = true;
      float excess = m_max - 1.0f;
      m1_speed -= excess;
      m2_speed -= excess;
      m3_speed -= excess;
      m4_speed -= excess;
    }

    if (!saturado) {
      i_roll  = constrain(i_roll  + Ki_roll  * raw_error_roll  * dt, -I_OUT_MAX, I_OUT_MAX);
      i_pitch = constrain(i_pitch + Ki_pitch * raw_error_pitch * dt, -I_OUT_MAX, I_OUT_MAX);
    }

    setMotorPulse(0, PULSO_IDLE + (int)(m1_speed * (PULSO_MAX - PULSO_IDLE)));
    setMotorPulse(1, PULSO_IDLE + (int)(m2_speed * (PULSO_MAX - PULSO_IDLE)));
    setMotorPulse(2, PULSO_IDLE + (int)(m3_speed * (PULSO_MAX - PULSO_IDLE)));
    setMotorPulse(3, PULSO_IDLE + (int)(m4_speed * (PULSO_MAX - PULSO_IDLE)));
  } else {
    i_roll = 0; i_pitch = 0;
    setMotorPulse(0, PULSO_IDLE); setMotorPulse(1, PULSO_IDLE);
    setMotorPulse(2, PULSO_IDLE); setMotorPulse(3, PULSO_IDLE);
  }

  tel_roll = angle_roll;
  tel_pitch = angle_pitch;
  tel_throttle = vuelo_activo ? throttle_base : 0.0f;

  static unsigned long log_timer = 0;
  if (millis() - log_timer >= 20) {
    log_timer = millis();
    logR[logSeq % LOG_N] = (int16_t)(constrain(angle_roll,  -300.0f, 300.0f) * 100.0f);
    logP[logSeq % LOG_N] = (int16_t)(constrain(angle_pitch, -300.0f, 300.0f) * 100.0f);
    logSeq = logSeq + 1;
  }

  // --- Comandos por Serial0 ---
  if (Serial0.available()) {
    char cmd = Serial0.read();
    if (cmd == 'a') {
      Serial0.println(intentarArmar(angle_roll, angle_pitch));
    } else if (cmd == 's') {
      enBajada = false;
      vuelo_activo = false;
      target_throttle = 0.0;
      Serial0.println("Vuelo detenido. Motores en IDLE.");
    } else if (cmd == '+' && vuelo_activo) {
      target_throttle = min(target_throttle + 0.05f, THROTTLE_MAX_WEB);
      Serial0.print("Target Throttle: "); Serial0.println(target_throttle * 100);
    } else if (cmd == '-') {
      target_throttle = max(target_throttle - 0.05f, 0.0f);
      Serial0.print("Target Throttle: "); Serial0.println(target_throttle * 100);
    }
  }

  // [V2] Serial cada 100 ms, en una sola linea corta (no frena el lazo)
  static unsigned long print_timer = 0;
  if (millis() - print_timer > 100) {
    print_timer = millis();
    Serial0.printf("R %.1f P %.1f Y %.0f T %.0f%% M %d %d %d %d\n",
                   angle_roll, angle_pitch, rate_yaw, throttle_base * 100,
                   tel_m[0], tel_m[1], tel_m[2], tel_m[3]);
  }
}
