#ifndef MOTORES_H
#define MOTORES_H

#include <Arduino.h>
#include "config.h"

extern volatile uint16_t pwmMotor1, pwmMotor2, pwmMotor3, pwmMotor4;
extern volatile float throttleAplicado;   // throttle común tras la desaturación

void iniciarMotores();
void mezclarMotores();
void aplicarPWM();
void detenerMotores();

#endif
