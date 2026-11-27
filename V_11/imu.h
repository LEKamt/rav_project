// =============================================================================
//  imu.h  -  MPU-6500 / MPU-6050 por I2C0
//
//  Entrega aceleracion en g y velocidad angular en deg/s YA EXPRESADAS en el
//  sistema del cuerpo FRD (x adelante, y derecha, z abajo) segun el mapeo de
//  config.h, con el sesgo del giroscopio restado.
// =============================================================================
#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

#define MPU_ADDR      0x68
#define MPU_WHO_AM_I  0x75
#define MPU_PWR_MGMT1 0x6B
#define MPU_CONFIG    0x1A
#define MPU_GYRO_CFG  0x1B
#define MPU_ACC_CFG   0x1C
#define MPU_SMPLRT    0x19
#define MPU_ACCEL_XO  0x3B

// +-4 g  -> 8192 LSB/g ;  +-500 dps -> 65.5 LSB/(deg/s)
#define LSB_POR_G    8192.0f
#define LSB_POR_DPS  65.5f

class IMU {
 public:
  uint8_t whoami = 0;
  bool    presente = false;
  uint32_t fallos = 0;

  // valores en ejes del CUERPO
  float ax = 0, ay = 0, az = 0;      // g
  float gx = 0, gy = 0, gz = 0;      // deg/s, sin sesgo
  float temp = 0;                    // C

  float bias[3] = {0, 0, 0};         // deg/s, en ejes del cuerpo

  bool iniciar() {
    Wire.begin(PIN_I2C0_SDA, PIN_I2C0_SCL, 400000);
    delay(10);
    whoami = leerReg(MPU_WHO_AM_I);
    // 0x70 = MPU-6500, 0x68 = MPU-6050, 0x73/0x75 = variantes MPU-9250/6555
    presente = (whoami != 0x00 && whoami != 0xFF);
    if (!presente) return false;

    escribirReg(MPU_PWR_MGMT1, 0x80); delay(100);   // reset
    escribirReg(MPU_PWR_MGMT1, 0x01); delay(20);    // reloj = giro X (mas estable)
    escribirReg(MPU_CONFIG,    0x03);               // DLPF 41 Hz (retardo ~5.9 ms)
    escribirReg(MPU_SMPLRT,    0x00);               // 1 kHz
    escribirReg(MPU_GYRO_CFG,  0x08);               // +-500 dps
    escribirReg(MPU_ACC_CFG,   0x08);               // +-4 g
    // [V_10] En el MPU-6500 el DLPF de 0x1A es SOLO del giroscopio. El del
    // acelerometro esta en 0x1D (ACCEL_CONFIG2) y por defecto vale 0, es decir
    // ~218 Hz de ancho de banda: la vibracion de las helices (giro ~140 Hz,
    // paso de pala ~280 Hz) entraba sin filtrar y, muestreada a 250 Hz, se
    // plegaba a frecuencias bajas. Eso ensuciaba justo la aceleracion vertical
    // con la que el Kalman calcula la velocidad de subida. 0x03 = 44.8 Hz.
    // El MPU-6050 (0x68) no tiene este registro: alli 0x1A ya filtra los dos.
    if (whoami != 0x68) escribirReg(0x1D, 0x03);
    delay(20);
    return true;
  }

  // Lee los 14 bytes y aplica el mapeo de ejes. ~0.45 ms a 400 kHz.
  bool leer() {
    uint8_t b[14];
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_ACCEL_XO);
    if (Wire.endTransmission(false) != 0) { fallos++; return false; }
    if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, true) != 14) { fallos++; return false; }
    for (int i = 0; i < 14; i++) b[i] = Wire.read();

    int16_t ra[3], rg[3];
    ra[0] = (int16_t)((b[0]  << 8) | b[1]);
    ra[1] = (int16_t)((b[2]  << 8) | b[3]);
    ra[2] = (int16_t)((b[4]  << 8) | b[5]);
    int16_t rt = (int16_t)((b[6] << 8) | b[7]);
    rg[0] = (int16_t)((b[8]  << 8) | b[9]);
    rg[1] = (int16_t)((b[10] << 8) | b[11]);
    rg[2] = (int16_t)((b[12] << 8) | b[13]);

    // sensor -> cuerpo
    float a[3], g[3];
    for (int k = 0; k < 3; k++) {
      a[k] = IMU_SIGNO[k] * (float)ra[IMU_EJE[k]] / LSB_POR_G;
      g[k] = IMU_SIGNO[k] * (float)rg[IMU_EJE[k]] / LSB_POR_DPS;
    }
    ax = a[0];  ay = a[1];  az = a[2];
    gx = g[0] - bias[0];
    gy = g[1] - bias[1];
    gz = g[2] - bias[2];
    temp = (float)rt / 333.87f + 21.0f;   // formula del MPU-6500
    return true;
  }

  // Promedia N lecturas con el dron QUIETO para estimar el sesgo del
  // giroscopio. Devuelve false si el dron se movio durante la medida.
  bool calibrarGiro(int n = 800) {
    float s[3] = {0, 0, 0};
    float mx[3] = {-1e9f, -1e9f, -1e9f}, mn[3] = {1e9f, 1e9f, 1e9f};
    float guardado[3] = {bias[0], bias[1], bias[2]};
    bias[0] = bias[1] = bias[2] = 0;

    for (int i = 0; i < n; i++) {
      if (!leer()) { delay(2); continue; }
      float v[3] = {gx, gy, gz};
      for (int k = 0; k < 3; k++) {
        s[k] += v[k];
        if (v[k] > mx[k]) mx[k] = v[k];
        if (v[k] < mn[k]) mn[k] = v[k];
      }
      // delay() en el ESP32 cede el procesador (es vTaskDelay por dentro): la
      // calibracion dura ~1.6 s y no puede dejar sin tiempo a la tarea idle del
      // nucleo 1, o salta el watchdog.
      delay(2);
    }
    // Si la diferencia entre el maximo y el minimo es grande, hubo movimiento
    // y el "sesgo" medido no serviria.
    for (int k = 0; k < 3; k++) {
      if (mx[k] - mn[k] > 8.0f) {
        bias[0] = guardado[0]; bias[1] = guardado[1]; bias[2] = guardado[2];
        return false;
      }
    }
    for (int k = 0; k < 3; k++) bias[k] = s[k] / n;
    return true;
  }

 private:
  void escribirReg(uint8_t r, uint8_t v) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(r); Wire.write(v);
    Wire.endTransmission();
  }
  uint8_t leerReg(uint8_t r) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(r);
    if (Wire.endTransmission(false) != 0) return 0;
    if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1, true) != 1) return 0;
    return Wire.read();
  }
};

#endif // IMU_H
