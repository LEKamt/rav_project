#ifndef AHRS_H
#define AHRS_H

// AHRS Mahony con cuaterniones. Reemplaza al filtro complementario sobre ángulos
// de Euler: no se degrada con inclinaciones grandes y estima el sesgo del
// giróscopo que quede después de la calibración del arranque.
//
// Convención del firmware (la de motores.cpp y control_actitud.cpp): cuerpo FLU,
// x adelante, y izquierda, z arriba. En reposo y nivelado el acelerómetro mide
// (0, 0, +1 g). Alabeo + = lado izquierdo arriba, cabeceo + = nariz abajo,
// guiñada + = antihorario visto desde arriba. q = [w, x, y, z], de cuerpo a mundo.
//
// Es el mismo algoritmo que blaze_ahrs de Arquitectura_Blaze, que trabaja en FRD.
// Sin dependencias de Arduino: se prueba en el PC.

#include <math.h>

struct Mahony {
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float kp = 1.0f;            // ganancia proporcional de la corrección [1/s]
    float ki = 0.0f;            // ganancia integral, estima el sesgo [1/s^2]
    float rechazoG = 0.15f;     // no corrige si | |a| - 1 g | supera esto [g]
    float sesgo[3] = {0.0f, 0.0f, 0.0f};   // corrección integral del giro [rad/s]
    bool accUsado = false;

    void configurar(float kp_, float ki_, float rechazo) {
        kp = kp_;
        ki = ki_;
        rechazoG = rechazo;
    }

    // Alabeo y cabeceo a partir de la gravedad medida [g]; guiñada 0
    void iniciarConAcc(const float acc[3]) {
        const float roll = atan2f(acc[1], acc[2]);
        const float pitch = atan2f(-acc[0], sqrtf(acc[1] * acc[1] + acc[2] * acc[2]));
        const float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
        const float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
        q[0] = cr * cp;
        q[1] = sr * cp;
        q[2] = cr * sp;
        q[3] = -sr * sp;
        sesgo[0] = sesgo[1] = sesgo[2] = 0.0f;
    }

    // giro en rad/s, acc en g, ambos en el cuerpo
    void actualizar(const float giro[3], const float acc[3], float dt) {
        const float w = q[0], x = q[1], y = q[2], z = q[3];
        float gx = giro[0], gy = giro[1], gz = giro[2];

        const float n = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
        accUsado = n > 0.0f && fabsf(n - 1.0f) < rechazoG;
        if (accUsado) {
            const float ax = acc[0] / n, ay = acc[1] / n, az = acc[2] / n;
            // "Arriba" estimado en el cuerpo: R^T e_z
            const float vx = 2.0f * (x * z - w * y);
            const float vy = 2.0f * (y * z + w * x);
            const float vz = w * w - x * x - y * y + z * z;
            // Error = medido x estimado
            const float ex = ay * vz - az * vy;
            const float ey = az * vx - ax * vz;
            const float ez = ax * vy - ay * vx;
            if (ki > 0.0f) {
                sesgo[0] += ki * ex * dt;
                sesgo[1] += ki * ey * dt;
                sesgo[2] += ki * ez * dt;
            }
            gx += kp * ex;
            gy += kp * ey;
            gz += kp * ez;
        }
        gx += sesgo[0];
        gy += sesgo[1];
        gz += sesgo[2];

        // q_punto = 1/2 q (x) (0, w)
        const float h = 0.5f * dt;
        float qw = w + h * (-x * gx - y * gy - z * gz);
        float qx = x + h * (w * gx + y * gz - z * gy);
        float qy = y + h * (w * gy - x * gz + z * gx);
        float qz = z + h * (w * gz + x * gy - y * gx);
        const float nq = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
        q[0] = qw / nq;
        q[1] = qx / nq;
        q[2] = qy / nq;
        q[3] = qz / nq;
    }

    // Euler ZYX [rad]
    void euler(float *roll, float *pitch, float *yaw) const {
        const float w = q[0], x = q[1], y = q[2], z = q[3];
        *roll = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
        float s = 2.0f * (w * y - z * x);
        s = s > 1.0f ? 1.0f : (s < -1.0f ? -1.0f : s);
        *pitch = asinf(s);
        *yaw = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
    }

    // Aceleración vertical en el mundo, sin gravedad [m/s^2, + arriba]: e_z^T R a - g
    float aceleracionVertical(const float acc[3], float g) const {
        const float w = q[0], x = q[1], y = q[2], z = q[3];
        const float r20 = 2.0f * (x * z - w * y);
        const float r21 = 2.0f * (y * z + w * x);
        const float r22 = w * w - x * x - y * y + z * z;
        return (r20 * acc[0] + r21 * acc[1] + r22 * acc[2] - 1.0f) * g;
    }

    // cos(alabeo) * cos(cabeceo): proyecta un sensor de distancia que mira hacia abajo
    float cosInclinacion() const {
        return q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
    }
};

#endif
