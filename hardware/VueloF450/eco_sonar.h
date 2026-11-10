#ifndef ECO_SONAR_H
#define ECO_SONAR_H

// Ultrasonido tipo HC-SR04 sin bloquear: el eco se cronometra por interrupción.
// Va en una cabecera y no en el .ino porque Arduino genera prototipos de las funciones del .ino
// y el de la interrupción pierde IRAM_ATTR (el atributo de sección choca con el de la definición).

#include <Arduino.h>

static int ecoPin = -1, trigPin = -1;
static volatile uint32_t ecoInicio = 0, ecoDuracion = 0;
static volatile bool ecoNuevo = false;

static void IRAM_ATTR isrEco() {
    const uint32_t t = micros();
    if (digitalRead(ecoPin)) {
        ecoInicio = t;
    } else if (ecoInicio) {
        ecoDuracion = t - ecoInicio;
        ecoInicio = 0;
        ecoNuevo = true;
    }
}

static void iniciarSonar(int trig, int eco) {
    trigPin = trig;
    ecoPin = eco;
    pinMode(trigPin, OUTPUT);
    digitalWrite(trigPin, LOW);
    pinMode(ecoPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(ecoPin), isrEco, CHANGE);
}

static void dispararSonar() {
    ecoNuevo = false;
    ecoInicio = 0;
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
}

// Distancia [m] del último eco, o -1 si no hubo eco válido (sin obstáculo el HC-SR04 da ~38 ms)
static float leerSonar() {
    if (!ecoNuevo) return -1;
    ecoNuevo = false;
    const uint32_t us = ecoDuracion;
    if (us < 100 || us > 25000) return -1;
    return us * 1e-6f * 343.0f * 0.5f;
}

#endif
