#ifndef SMC_H
#define SMC_H

#include <math.h>

// Control robusto por modo deslizante (SMC), según la sección 4.2 de las notas del
// curso (ec. 16), con saturación en lugar del signo para reducir el chattering:
//
//   s = de + lambda * e
//   u = tau_eq_estimado + sesgo - kd * s - rho * sat(s / phi)
//   sesgo_punto = -p * s     (solo dentro de la capa límite)
//
// El sesgo es la ley adaptativa del curso con regresor Y = 1. Sin él, un par
// constante deja s distinto de cero y el dron se va tumbando: ver robot_control/smc.py.
//
// Sin dependencias de Arduino: se compila también en el PC para comprobar que
// coincide con robot_control/smc.py (robot_tests/unit/test_smc_paridad.py). Si se
// cambia una ley aquí hay que cambiarla allí.

static inline float smcAcotar(float x, float minimo, float maximo) {
    return x < minimo ? minimo : (x > maximo ? maximo : x);
}

static inline float smcLey(float tauEq, float s, float kd, float rho, float phi) {
    return tauEq - kd * s - rho * smcAcotar(s / phi, -1.0f, 1.0f);
}

static inline float smcAdaptar(float sesgo, float s, float phi, float p, float maximo,
                               float dt) {
    if (fabsf(s) >= phi) return sesgo;
    return smcAcotar(sesgo - p * s * dt, -maximo, maximo);
}

// Un eje de actitud, en rad y N*m. La consigna llega como escalón, así que su
// velocidad y su aceleración son cero.
struct EjeSMC {
    float tau;
    float s;
    float velRef;
};

static inline EjeSMC smcEje(float angulo, float sp, float rate, float inercia,
                            float giroscopico, float lambda, float kd, float rho,
                            float phi, float p, float sesgoMax, float *sesgo, float dt) {
    EjeSMC r;
    r.velRef = -lambda * (angulo - sp);
    r.s = rate - r.velRef;
    *sesgo = smcAdaptar(*sesgo, r.s, phi, p, sesgoMax, dt);
    r.tau = smcLey(inercia * (-lambda * rate) + giroscopico + *sesgo, r.s, kd, rho, phi);
    return r;
}

// Trayectoria de altura: el SMC necesita posición, velocidad y aceleración deseadas
// y la consigna llega como escalón. Filtro de segundo orden críticamente amortiguado
// y acotado.
struct ReferenciaZ {
    float z = 0.0f;
    float vz = 0.0f;
    float az = 0.0f;

    void reset(float altura) {
        z = altura;
        vz = 0.0f;
        az = 0.0f;
    }

    void actualizar(float objetivo, float omega, float accMax, float velMax, float dt) {
        az = smcAcotar(omega * omega * (objetivo - z) - 2.0f * omega * vz, -accMax, accMax);
        vz = smcAcotar(vz + az * dt, -velMax, velMax);
        z += vz * dt;
    }
};

// Altura: devuelve la fuerza vertical pedida [N] y deja en velDeseada y s los
// valores que se publican por telemetría.
static inline float smcAltura(const ReferenciaZ &ref, float altura, float vz, float masa,
                              float g, float lambda, float kd, float rho, float phi,
                              float p, float sesgoMax, float *sesgo, float dt,
                              float *velDeseada, float *s) {
    float e = altura - ref.z;
    float de = vz - ref.vz;
    *velDeseada = ref.vz - lambda * e;
    *s = vz - *velDeseada;
    *sesgo = smcAdaptar(*sesgo, *s, phi, p, sesgoMax, dt);
    return smcLey(masa * (ref.az - lambda * de + g) + *sesgo, *s, kd, rho, phi);
}

#endif
