#ifndef PID_H
#define PID_H

#include <Arduino.h>

// PID con la misma semántica que el PIDController del Lab_rav_03 (sjtu_drone,
// plugin_drone.cpp + pid_controller.cpp):
//
//   consigna_filtrada = (dt*consigna + tau*consigna_filtrada) / (dt + tau)
//   p = consigna_filtrada - x
//   d = dconsigna - dx            <- la derivada MEDIDA, no la del error
//   i = i + dt*p
//   salida = kp*p + kd*d + ki*i
//
// Dos diferencias deliberadas con el original:
//   - `i_max` acota el integrador. El del laboratorio no tiene anti-windup, y en
//     un dron real con el throttle de hover mal estimado eso se carga sin freno.
//   - `limite` se aplica a la consigna de entrada, igual que en el laboratorio,
//     pero además `salida_max` acota la salida.
struct PID {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float tau = 0.0f;           // constante de tiempo del filtro de consigna
    float limite = -1.0f;       // acota |consigna| (<= 0: sin límite)
    float i_max = -1.0f;        // acota |integral| (<= 0: sin límite)
    float salida_max = -1.0f;   // acota |salida| (<= 0: sin límite)

    // Estado observable (se publica por telemetría para sintonizar)
    float consigna = 0.0f;      // consigna ya filtrada
    float dconsigna = 0.0f;
    float p = 0.0f, i = 0.0f, d = 0.0f;
    float salida = 0.0f;

    void configurar(float kp_, float ki_, float kd_, float tau_,
                    float limite_, float i_max_, float salida_max_) {
        kp = kp_; ki = ki_; kd = kd_; tau = tau_;
        limite = limite_; i_max = i_max_; salida_max = salida_max_;
        reset();
    }

    void reset() {
        consigna = dconsigna = 0.0f;
        p = i = d = salida = 0.0f;
    }

    // Arranca el filtro en un valor concreto (p. ej. la altura actual al armar),
    // para que la consigna no entre como un escalón desde cero.
    void inicializar_consigna(float valor) {
        consigna = valor;
        dconsigna = 0.0f;
    }

    float actualizar(float consigna_nueva, float x, float dx, float dt) {
        if (dt <= 0.0f) return salida;

        if (limite > 0.0f) {
            consigna_nueva = constrain(consigna_nueva, -limite, limite);
        }

        if (dt + tau > 0.0f) {
            float consigna_prev = consigna;
            consigna = (dt * consigna_nueva + tau * consigna) / (dt + tau);
            dconsigna = (consigna - consigna_prev) / dt;
        }

        p = consigna - x;
        d = dconsigna - dx;

        // Integración condicional: no se acumula más error hacia una saturación
        // de salida ya alcanzada.
        float candidato = i + dt * p;
        if (i_max > 0.0f) candidato = constrain(candidato, -i_max, i_max);
        float tentativa = kp * p + kd * d + ki * candidato;
        bool saturado = (salida_max > 0.0f) &&
                        ((tentativa > salida_max && p > 0.0f) ||
                         (tentativa < -salida_max && p < 0.0f));
        if (!saturado) i = candidato;

        salida = kp * p + kd * d + ki * i;
        if (salida_max > 0.0f) salida = constrain(salida, -salida_max, salida_max);
        return salida;
    }
};

#endif
