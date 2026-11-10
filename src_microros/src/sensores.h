#ifndef SENSORES_H
#define SENSORES_H

#include <Wire.h>
#include "config.h"

// Estado estimado. Son escalares de 32 bits: la lectura/escritura es atómica en
// el Xtensa, así que se comparten entre la tarea de control y la de comunicaciones
// sin mutex. El bus I2C sí necesita mutexI2C.
extern volatile float angRoll, angPitch, angYaw;      // grados (AHRS Mahony, FLU)
extern volatile float velRoll, velPitch, velYaw;      // deg/s (giróscopo filtrado)
extern volatile float ax_g, ay_g, az_g;               // g, cuerpo, filtrado
extern volatile float alturaControl;                  // m, Kalman: la que usa el control
extern volatile float alturaLiDAR;                    // m, ToF proyectado (0 si no es válido)
extern volatile float alturaSonar;                    // m, ultrasonido proyectado (0 si no es válido)
extern volatile float alturaBaro;                     // m, barómetro relativo al arranque
extern volatile float sesgoBaro;                      // m, sesgo del baro estimado por el Kalman
extern volatile float velVertical;                    // m/s (+ hacia arriba)
extern volatile float aceleracionVertical;            // m/s^2 (+ hacia arriba, sin gravedad)
extern volatile bool  usarLiDAR;                      // la última medida del ToF fue válida
extern volatile bool  imuOK;                          // última lectura de IMU válida
extern volatile bool  giroCalibrado;                  // el sesgo del arranque se midió quieto
extern volatile uint32_t fallosIMUConsecutivos;       // ciclos seguidos sin poder leer la IMU
extern volatile uint32_t fallosIMUTotal;              // desde el arranque
extern volatile uint32_t tiempoIMUMaxUs;              // peor duración de actualizarIMU()

extern SemaphoreHandle_t mutexI2C;

// Qué sensores respondieron al arrancar (para el diagnóstico)
struct EstadoSensores {
    uint8_t whoAmI;
    bool imu, baro, tof, sonar;
};
extern EstadoSensores estadoSensores;

// La IMU es imprescindible; baro, ToF y sonar no (el Kalman sigue con lo que haya).
bool iniciarSensores();

// Tarea rápida (FREQ_CONTROL_HZ): IMU + filtros + AHRS
void actualizarIMU(float dt);

// Tarea lenta (FREQ_SENSORES_HZ): ToF + sonar + baro + Kalman de altura
void actualizarAltitud(float dt);

// Comprobaciones antes de armar. Devuelve nullptr si todo está bien, o el motivo.
const char *motivoNoArmar();

#endif
