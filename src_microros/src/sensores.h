#ifndef SENSORES_H
#define SENSORES_H

#include <Wire.h>
#include <MPU6500_WE.h>
#include <MS5611.h>
#include <Adafruit_VL53L0X.h>
#include "config.h"

// Estado estimado. Son escalares de 32 bits: la lectura/escritura es atómica en
// el Xtensa, así que se comparten entre la tarea de control y la de comunicaciones
// sin mutex. El bus I2C sí necesita mutexI2C.
extern volatile float angRoll, angPitch, angYaw;      // grados
extern volatile float velRoll, velPitch, velYaw;      // deg/s (giróscopo filtrado)
extern volatile float ax_g, ay_g, az_g;               // g
extern volatile float alturaControl, alturaLiDAR, alturaBaro;  // m
extern volatile float velVertical;                    // m/s (+ hacia arriba)
extern volatile float aceleracionVertical;            // m/s^2 (+ hacia arriba, sin gravedad)
extern volatile bool  usarLiDAR;
extern volatile bool  imuOK;                          // última lectura de IMU válida
extern volatile uint32_t fallosIMUConsecutivos;       // ciclos seguidos sin poder leer la IMU

extern SemaphoreHandle_t mutexI2C;

bool iniciarSensores();

// Tarea rápida (FREQ_CONTROL_HZ): IMU + filtro complementario
void actualizarIMU(float dt);

// Tarea lenta (FREQ_SENSORES_HZ): barómetro + LiDAR + fusión de altura
void actualizarAltitud(float dt);

#endif
