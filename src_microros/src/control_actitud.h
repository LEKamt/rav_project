#ifndef CONTROL_ACTITUD_H
#define CONTROL_ACTITUD_H

#include <Arduino.h>

// Control de actitud en cascada:
//   externo: error de ángulo (roll/pitch)  -> velocidad angular deseada [deg/s]
//   interno: error de velocidad angular    -> corrección de throttle
// La guiñada no tiene lazo externo: se controla la velocidad directamente, porque
// sin magnetómetro el ángulo de yaw deriva sin cota.

void initControlActitud();
void resetIntegradoresActitud();
void updateControlActitud(float dt);

// Salidas: fracción de throttle, saturadas a +-CORR_MAX
extern volatile float roll_corr, pitch_corr, yaw_corr;

// Setpoints comandados desde ROS 2
extern volatile float setpointRoll;      // grados
extern volatile float setpointPitch;     // grados
extern volatile float setpointYawRate;   // deg/s

// Velocidades angulares deseadas (publicadas para sintonizar el lazo interno)
extern volatile float rateSpRoll, rateSpPitch;

// Ajuste de ganancias en caliente (solo se aplica con el dron desarmado)
void setGananciasActitud(float kpAng, float kpRate, float kiRate, float kdRate,
                         float kpYaw, float kiYaw);
void getGananciasActitud(float *kpAng, float *kpRate, float *kiRate, float *kdRate,
                         float *kpYaw, float *kiYaw);

#endif
