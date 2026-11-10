#ifndef SONAR_H
#define SONAR_H

// Ultrasonido tipo HC-SR04 sin bloqueo: el disparo es un pulso de 10 us y el eco
// se cronometra por interrupción, así la tarea de altura nunca espera los ~25 ms
// que tarda el eco a 4 m.
//
// Un HC-SR04 de 5 V saca el eco a 5 V: en el S3 hace falta un divisor (por
// ejemplo 1 k / 2 k) en PIN_SONAR_ECHO. El HC-SR04P o el RCWL-1601 funcionan a 3.3 V.

#include <Arduino.h>

void iniciarSonar(int pinTrig, int pinEcho);

// Lanza una medida. Entre disparos debe haber >= 60 ms (eco del anterior).
void dispararSonar();

// Si llegó un eco desde el último disparo, deja la distancia [m] y devuelve true.
// tempC corrige la velocidad del sonido (331.3 + 0.606 T m/s).
bool leerSonar(float tempC, float *distancia);

#endif
