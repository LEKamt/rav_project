#ifndef CONTROL_VUELO_H
#define CONTROL_VUELO_H

#include "config.h"
#include "pid.h"

// Control de altura en cascada, como en el Lab_rav_03:
//
//   posición -> PID pos_z -> velocidad vertical deseada [m/s]
//   velocidad -> PID vel_z -> aceleración vertical deseada [m/s^2]
//   throttle = THROTTLE_HOVER * (1 + a_deseada / g)
//
// La última línea es el equivalente normalizado del `force = masa * (pid + g)`
// del plugin del laboratorio: con aceleración deseada nula sale exactamente el
// throttle de hover, y el término integral del PID de velocidad absorbe el error
// de estimación de THROTTLE_HOVER (peso real, batería, hélices).

extern volatile float alturaObjetivo;       // m, comandado por ROS 2
extern volatile bool  setpointManual;       // hay que fijar altura antes de armar
extern volatile float throttleBase;         // lo consume la mezcla

extern PID pidPosZ;
extern PID pidVelZ;

// Telemetría, común a las dos leyes de control
extern volatile float consignaAltura;       // m, consigna ya filtrada
extern volatile float vzDeseada;            // m/s
extern volatile float accDeseada;           // m/s^2

void initControlAltitud();
void controlAltitud(float dt);

#endif
