/*
 * F450 + ESP32-S3 : Prueba de lectura de sensores (Fase 0, sin motores)
 * ---------------------------------------------------------------------
 *  - MPU (MPU-6050 / MPU-6500 / MPU-9250) por I2C  -> Wire  (SDA=GPIO1, SCL=GPIO39)
 *  - ToF (VL53L0X o VL53L1X)              por I2C  -> Wire1 (SDA=GPIO4, SCL=GPIO5)
 *  - Ultrasonido HC-SR04                  TRIG=GPIO6, ECHO=GPIO7
 *
 * Librerías (Gestor de librerías): "VL53L0X" o "VL53L1X" de Pololu.
 * Si tu ToF es VL53L1X, comenta TOF_VL53L0X y descomenta TOF_VL53L1X.
 *
 * Placa: ESP32S3 Dev Module. Si usas el USB nativo: "USB CDC On Boot: Enabled".
 * Serial: 115200 baudios.
 *
 * !! El ECHO del HC-SR04 entrega 5 V. El GPIO del S3 tolera solo 3.3 V:
 *    usa divisor resistivo (p. ej. 1k + 2k) o un módulo HC-SR04 de 3.3 V.
 * !! NO conectar las baterías/ESC de los motores durante esta prueba.
 */

#include <Arduino.h>
#include <Wire.h>

// ====================== SELECCIÓN DE ToF ======================
#define TOF_VL53L0X
//#define TOF_VL53L1X

#if defined(TOF_VL53L0X)
  #include <VL53L0X.h>
  VL53L0X tof;
#elif defined(TOF_VL53L1X)
  #include <VL53L1X.h>
  VL53L1X tof;
#endif

// ========================== PINES =============================
constexpr int PIN_MPU_SDA = 1;
constexpr int PIN_MPU_SCL = 39;
constexpr int PIN_TOF_SDA = 4;
constexpr int PIN_TOF_SCL = 5;
constexpr int PIN_US_TRIG = 6;
constexpr int PIN_US_ECHO = 7;

// ======================= PARÁMETROS ===========================
constexpr uint32_t I2C_FREQ        = 400000;
constexpr uint32_t PERIODO_MS      = 50;      // 20 Hz de impresión
constexpr uint32_t US_TIMEOUT_US   = 25000;   // ~4.3 m máx.
constexpr int      N_CALIB_GIRO    = 500;     // muestras para offset de giroscopio

// MPU: ±4 g y ±500 °/s
constexpr float ACC_LSB_G   = 8192.0f;
constexpr float GYR_LSB_DPS = 65.5f;

// Registros MPU
constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
constexpr uint8_t REG_CONFIG       = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_CONF2  = 0x1D;   // solo 6500/9250
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t REG_WHO_AM_I     = 0x75;

// ========================= ESTADO =============================
uint8_t mpuAddr   = 0;
bool    mpuOK     = false;
bool    tofOK     = false;
float   gOffX = 0, gOffY = 0, gOffZ = 0;

struct MpuData { float ax, ay, az, gx, gy, gz, temp, roll, pitch; };

// ===================== UTILIDADES I2C =========================
void escanearBus(TwoWire &bus, const char *nombre) {
  Serial0.printf("[SCAN] %s: ", nombre);
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    bus.beginTransmission(a);
    if (bus.endTransmission() == 0) {
      Serial0.printf("0x%02X ", a);
      n++;
    }
  }
  if (n == 0) Serial0.print("(sin dispositivos: revisar cableado, pull-ups y alimentación)");
  Serial0.println();
}

bool escribirReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool leerRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)mpuAddr, (int)len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

// ========================= MPU ================================
bool mpuLeerCrudo(int16_t &ax, int16_t &ay, int16_t &az,
                  int16_t &t, int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t b[14];
  if (!leerRegs(REG_ACCEL_XOUT_H, b, 14)) return false;
  ax = (b[0]  << 8) | b[1];
  ay = (b[2]  << 8) | b[3];
  az = (b[4]  << 8) | b[5];
  t  = (b[6]  << 8) | b[7];
  gx = (b[8]  << 8) | b[9];
  gy = (b[10] << 8) | b[11];
  gz = (b[12] << 8) | b[13];
  return true;
}

bool mpuInit() {
  // Detectar dirección 0x68 / 0x69
  for (uint8_t a : {(uint8_t)0x68, (uint8_t)0x69}) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { mpuAddr = a; break; }
  }
  if (!mpuAddr) { Serial0.println("[MPU] No responde en 0x68/0x69"); return false; }

  uint8_t who = 0;
  if (!leerRegs(REG_WHO_AM_I, &who, 1)) { Serial0.println("[MPU] Error leyendo WHO_AM_I"); return false; }
  Serial0.printf("[MPU] Dir 0x%02X, WHO_AM_I = 0x%02X ", mpuAddr, who);
  bool es6500familia = false;
  switch (who) {
    case 0x68: Serial0.println("(MPU-6050)"); break;
    case 0x70: Serial0.println("(MPU-6500)"); es6500familia = true; break;
    case 0x71: Serial0.println("(MPU-9250)"); es6500familia = true; break;
    case 0x73: Serial0.println("(MPU-9255)"); es6500familia = true; break;
    default:   Serial0.println("(ID desconocido, se intenta continuar)"); break;
  }

  escribirReg(REG_PWR_MGMT_1, 0x80);   // reset
  delay(100);
  escribirReg(REG_PWR_MGMT_1, 0x01);   // despertar, reloj PLL giroscopio X
  delay(50);
  escribirReg(REG_SMPLRT_DIV, 0x00);   // 1 kHz con DLPF activo
  escribirReg(REG_CONFIG, 0x03);       // DLPF giro ~41 Hz
  escribirReg(REG_GYRO_CONFIG, 0x08);  // ±500 °/s
  escribirReg(REG_ACCEL_CONFIG, 0x08); // ±4 g
  if (es6500familia) escribirReg(REG_ACCEL_CONF2, 0x03); // DLPF accel ~44 Hz
  delay(50);
  return true;
}

void mpuCalibrarGiro() {
  Serial0.println("[MPU] Calibrando giroscopio: MANTENER LA PLATAFORMA QUIETA...");
  double sx = 0, sy = 0, sz = 0;
  int validas = 0;
  for (int i = 0; i < N_CALIB_GIRO; i++) {
    int16_t ax, ay, az, t, gx, gy, gz;
    if (mpuLeerCrudo(ax, ay, az, t, gx, gy, gz)) {
      sx += gx; sy += gy; sz += gz;
      validas++;
    }
    delay(3);
  }
  if (validas > 0) {
    gOffX = sx / validas; gOffY = sy / validas; gOffZ = sz / validas;
  }
  Serial0.printf("[MPU] Offset giro (LSB): %.1f, %.1f, %.1f  (%d muestras)\n",
                gOffX, gOffY, gOffZ, validas);
}

bool mpuLeer(MpuData &d) {
  int16_t ax, ay, az, t, gx, gy, gz;
  if (!mpuLeerCrudo(ax, ay, az, t, gx, gy, gz)) return false;
  d.ax = ax / ACC_LSB_G;
  d.ay = ay / ACC_LSB_G;
  d.az = az / ACC_LSB_G;
  d.gx = (gx - gOffX) / GYR_LSB_DPS;
  d.gy = (gy - gOffY) / GYR_LSB_DPS;
  d.gz = (gz - gOffZ) / GYR_LSB_DPS;
  d.temp  = t / 333.87f + 21.0f;   // MPU-9250/6500; en MPU-6050: t/340 + 36.53
  // Ángulos estáticos a partir del acelerómetro (solo referencia, sin fusión)
  d.roll  = atan2f(d.ay, d.az) * 180.0f / PI;
  d.pitch = atan2f(-d.ax, sqrtf(d.ay * d.ay + d.az * d.az)) * 180.0f / PI;
  return true;
}

// ========================= ToF ================================
bool tofInit() {
  tof.setBus(&Wire1);
  tof.setTimeout(200);
#if defined(TOF_VL53L0X)
  if (!tof.init()) { Serial0.println("[ToF] VL53L0X no inicializa"); return false; }
  tof.startContinuous();            // modo back-to-back
  Serial0.println("[ToF] VL53L0X OK");
#elif defined(TOF_VL53L1X)
  if (!tof.init()) { Serial0.println("[ToF] VL53L1X no inicializa"); return false; }
  tof.setDistanceMode(VL53L1X::Long);
  tof.setMeasurementTimingBudget(33000);
  tof.startContinuous(50);
  Serial0.println("[ToF] VL53L1X OK");
#endif
  return true;
}

// Devuelve mm, o -1 si hay timeout / fuera de rango
int tofLeerMM() {
#if defined(TOF_VL53L0X)
  uint16_t d = tof.readRangeContinuousMillimeters();
  if (tof.timeoutOccurred() || d >= 8190) return -1;
  return d;
#elif defined(TOF_VL53L1X)
  uint16_t d = tof.read();
  if (tof.timeoutOccurred()) return -1;
  return d;
#endif
}

// ===================== ULTRASONIDO ============================
// Devuelve cm, o -1 si no hay eco
float usLeerCM() {
  digitalWrite(PIN_US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_US_TRIG, LOW);
  unsigned long dur = pulseIn(PIN_US_ECHO, HIGH, US_TIMEOUT_US);
  if (dur == 0) return -1.0f;
  return dur * 0.0343f / 2.0f;     // 343 m/s a ~20 °C
}

// ========================= SETUP ==============================
void setup() {
  Serial0.begin(115200);
  delay(1500);
  Serial0.println();
  Serial0.println("=== F450 / ESP32-S3 : prueba de sensores (sin motores) ===");

  pinMode(PIN_US_TRIG, OUTPUT);
  pinMode(PIN_US_ECHO, INPUT);
  digitalWrite(PIN_US_TRIG, LOW);

  Wire.begin(PIN_MPU_SDA, PIN_MPU_SCL, I2C_FREQ);
  Wire1.begin(PIN_TOF_SDA, PIN_TOF_SCL, I2C_FREQ);

  escanearBus(Wire,  "Bus MPU (Wire)  ");
  escanearBus(Wire1, "Bus ToF (Wire1) ");

  mpuOK = mpuInit();
  if (mpuOK) mpuCalibrarGiro();
  tofOK = tofInit();

  Serial0.println();
  Serial0.println("t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,temp_C,roll_deg,pitch_deg,tof_mm,us_cm");
}

// ========================== LOOP ==============================
void loop() {
  static uint32_t tProx = 0;
  uint32_t ahora = millis();
  if (ahora < tProx) return;
  tProx = ahora + PERIODO_MS;

  MpuData m = {};
  bool mOK = mpuOK && mpuLeer(m);
  int   tofmm = tofOK ? tofLeerMM() : -1;
  float uscm  = usLeerCM();

  Serial0.print(ahora);
  Serial0.print(',');
  if (mOK) {
    Serial0.printf("%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,",
                  m.ax, m.ay, m.az, m.gx, m.gy, m.gz, m.temp, m.roll, m.pitch);
  } else {
    Serial0.print("nan,nan,nan,nan,nan,nan,nan,nan,nan,");
  }
  if (tofmm >= 0) Serial0.print(tofmm); else Serial0.print("nan");
  Serial0.print(',');
  if (uscm >= 0) Serial0.println(uscm, 1); else Serial0.println("nan");
}
