#include "sonar.h"

static int trig = -1, echo = -1;
static volatile uint32_t tSubida = 0;
static volatile uint32_t duracionUs = 0;
static volatile bool hayEco = false;

static void IRAM_ATTR isrEco() {
    const uint32_t t = micros();
    if (digitalRead(echo)) {
        tSubida = t;
    } else if (tSubida != 0) {
        duracionUs = t - tSubida;
        tSubida = 0;
        hayEco = true;
    }
}

void iniciarSonar(int pinTrig, int pinEcho) {
    trig = pinTrig;
    echo = pinEcho;
    pinMode(trig, OUTPUT);
    digitalWrite(trig, LOW);
    pinMode(echo, INPUT);
    attachInterrupt(digitalPinToInterrupt(echo), isrEco, CHANGE);
}

void dispararSonar() {
    if (trig < 0) return;
    hayEco = false;
    tSubida = 0;
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
}

bool leerSonar(float tempC, float *distancia) {
    if (!hayEco) return false;
    hayEco = false;
    const uint32_t us = duracionUs;
    // Sin obstáculo el HC-SR04 devuelve un pulso de ~38 ms: se descarta todo lo
    // que pase de ~5 m de ida y vuelta.
    if (us < 100 || us > 30000) return false;
    const float c = 331.3f + 0.606f * tempC;
    *distancia = us * 1e-6f * c * 0.5f;
    return true;
}
