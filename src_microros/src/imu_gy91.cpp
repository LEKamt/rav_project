#include "imu_gy91.h"

// Registros (mapa de la MPU-9250, iguales en la MPU-6500)
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_ACCEL_CONFIG2 0x1D
#define REG_ACCEL_XOUT_H  0x3B
#define REG_PWR_MGMT_1    0x6B
#define REG_PWR_MGMT_2    0x6C
#define REG_WHO_AM_I      0x75

// Rangos: ±500 deg/s y ±8 g
#define GYRO_FS_SEL       0x08
#define ACCEL_FS_SEL      0x10
#define LSB_POR_DPS       65.5f
#define LSB_POR_G         4096.0f

static bool escribir(TwoWire &bus, uint8_t dir, uint8_t reg, uint8_t valor) {
    bus.beginTransmission(dir);
    bus.write(reg);
    bus.write(valor);
    return bus.endTransmission() == 0;
}

static bool leer(TwoWire &bus, uint8_t dir, uint8_t reg, uint8_t *buf, size_t n) {
    bus.beginTransmission(dir);
    bus.write(reg);
    if (bus.endTransmission(false) != 0) return false;
    if (bus.requestFrom(dir, (uint8_t)n) != n) return false;
    for (size_t i = 0; i < n; i++) buf[i] = bus.read();
    return true;
}

const char *nombreIMU(uint8_t quien) {
    switch (quien) {
        case 0x71: return "MPU-9250";
        case 0x73: return "MPU-9255";
        case 0x70: return "MPU-6500";
        default:   return nullptr;
    }
}

bool iniciarIMU(TwoWire &bus, uint8_t dir, uint8_t dlpfCfg, uint8_t *quien) {
    *quien = 0;
    if (!escribir(bus, dir, REG_PWR_MGMT_1, 0x80)) return false;   // reinicio
    delay(100);
    if (!leer(bus, dir, REG_WHO_AM_I, quien, 1) || nombreIMU(*quien) == nullptr) return false;

    bool ok = escribir(bus, dir, REG_PWR_MGMT_1, 0x01)          // reloj del PLL del giro X
           && escribir(bus, dir, REG_PWR_MGMT_2, 0x00)          // todos los ejes
           && escribir(bus, dir, REG_CONFIG, dlpfCfg & 0x07)    // DLPF del giro
           && escribir(bus, dir, REG_SMPLRT_DIV, 0x00)          // 1 kHz interno
           && escribir(bus, dir, REG_GYRO_CONFIG, GYRO_FS_SEL)
           && escribir(bus, dir, REG_ACCEL_CONFIG, ACCEL_FS_SEL)
           && escribir(bus, dir, REG_ACCEL_CONFIG2, 0x03);      // DLPF del acel. ~45 Hz
    delay(50);
    return ok;
}

bool leerIMU(TwoWire &bus, uint8_t dir, LecturaIMU *l) {
    uint8_t b[14];
    if (!leer(bus, dir, REG_ACCEL_XOUT_H, b, sizeof(b))) return false;
    for (int i = 0; i < 3; i++) {
        const int16_t a = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]);
        const int16_t g = (int16_t)((b[8 + 2 * i] << 8) | b[8 + 2 * i + 1]);   // 6-7 = temperatura
        l->acc[i] = a / LSB_POR_G;
        l->giro[i] = g / LSB_POR_DPS;
    }
    return true;
}
