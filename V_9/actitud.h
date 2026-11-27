// =============================================================================
//  actitud.h  -  Estimador de orientacion (filtro complementario de Mahony)
//
//  Por que Mahony y no el complementario de un eje del codigo anterior:
//  aquel mezclaba, eje por eje, el angulo del acelerometro con la integral del
//  giroscopio. Eso funciona mientras el dron este casi nivelado y solo se mueva
//  en un eje; en cuanto roll y pitch se mezclan, la integral de p NO es la
//  derivada de roll (ver la ec. 26 de las notas: phi_punto = p + q sin(phi)
//  tan(theta) + r cos(phi) tan(theta)) y el estimador se desfasa. Ademas
//  obligaba a adivinar el signo entre el giroscopio y el angulo, que es
//  justamente lo que se rompio varias veces.
//
//  Mahony integra la orientacion COMPLETA en un cuaternion usando (p,q,r) tal
//  como salen del giroscopio, y corrige la deriva con el error entre la
//  gravedad medida y la gravedad predicha. Los signos salen solos del producto
//  cruz: no hay constantes de signo que ajustar. El termino Ki estima ademas
//  el sesgo del giroscopio en vuelo.
//
//  Convencion: cuaternion q = (q0, q1, q2, q3) que rota del CUERPO (FRD) al
//  sistema inercial NED. Angulos de Euler ZYX: yaw -> pitch -> roll.
//
//  AVISO: sin magnetometro el yaw solo sale de integrar r, asi que deriva
//  (tipicamente 1-3 deg/min despues de calibrar). Para estabilizar es
//  suficiente, porque lo que importa es mantener el rumbo a corto plazo; para
//  navegar hara falta brujula o vision.
// =============================================================================
#ifndef ACTITUD_H
#define ACTITUD_H

#include <Arduino.h>
#include <math.h>
#include "config.h"

class Actitud {
 public:
  float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
  float roll = 0, pitch = 0, yaw = 0;      // deg, ya con el trim aplicado
  float roll_s = 0, pitch_s = 0;           // deg, sin trim (los del cuaternion)
  float ibx = 0, iby = 0, ibz = 0;         // integral del error (rad/s)

  // Alinea el cuaternion con la gravedad medida: evita los primeros segundos
  // de convergencia y arranca con el angulo correcto aunque no este nivelado.
  void alinear(float ax, float ay, float az) {
    float n = sqrtf(ax * ax + ay * ay + az * az);
    if (n < 0.1f) return;
    ax /= n; ay /= n; az /= n;
    // En FRD y nivelado el acelerometro mide (0,0,-1).
    float r = atan2f(-ay, -az);
    float p = atan2f(ax, sqrtf(ay * ay + az * az));
    desdeEuler(r, p, 0.0f);
    ibx = iby = ibz = 0;
  }

  void desdeEuler(float r, float p, float y) {
    float cr = cosf(r * 0.5f), sr = sinf(r * 0.5f);
    float cp = cosf(p * 0.5f), sp = sinf(p * 0.5f);
    float cy = cosf(y * 0.5f), sy = sinf(y * 0.5f);
    q0 = cr * cp * cy + sr * sp * sy;
    q1 = sr * cp * cy - cr * sp * sy;
    q2 = cr * sp * cy + sr * cp * sy;
    q3 = cr * cp * sy - sr * sp * cy;
  }

  // gx,gy,gz en deg/s ; ax,ay,az en g (no hace falta normalizar)
  void actualizar(float gx, float gy, float gz,
                  float ax, float ay, float az,
                  float dt, float Kp, float Ki,
                  float trim_roll_deg, float trim_pitch_deg) {
    float wx = gx * DEG2RAD, wy = gy * DEG2RAD, wz = gz * DEG2RAD;

    float n = sqrtf(ax * ax + ay * ay + az * az);
    // Solo se corrige cuando el modulo del acelerometro se parece a 1 g. Si el
    // dron esta acelerando fuerte o vibrando, esa medida no apunta a la
    // gravedad y meterla solo ensucia la estimacion: en ese caso se integra el
    // giroscopio a secas durante esos milisegundos.
    if (n > 0.5f && n < 1.6f) {
      ax /= n; ay /= n; az /= n;

      // Direccion de la gravedad medida, segun el cuaternion actual:
      //   v = R^T * (0,0,-1)
      float vx =  2.0f * (q0 * q2 - q1 * q3);
      float vy = -2.0f * (q0 * q1 + q2 * q3);
      float vz = -(q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);

      // error = medido x estimado
      float ex = ay * vz - az * vy;
      float ey = az * vx - ax * vz;
      float ez = ax * vy - ay * vx;

      if (Ki > 0.0f) {
        ibx += Ki * ex * dt;  iby += Ki * ey * dt;  ibz += Ki * ez * dt;
        const float lim = 0.1f;               // rad/s, ~5.7 deg/s de sesgo
        ibx = limitar(ibx, -lim, lim);
        iby = limitar(iby, -lim, lim);
        ibz = limitar(ibz, -lim, lim);
        wx += ibx;  wy += iby;  wz += ibz;
      }
      wx += Kp * ex;  wy += Kp * ey;  wz += Kp * ez;
    }

    // q_punto = 0.5 * q (x) (0, w)
    float dq0 = 0.5f * (-q1 * wx - q2 * wy - q3 * wz);
    float dq1 = 0.5f * ( q0 * wx + q2 * wz - q3 * wy);
    float dq2 = 0.5f * ( q0 * wy - q1 * wz + q3 * wx);
    float dq3 = 0.5f * ( q0 * wz + q1 * wy - q2 * wx);

    q0 += dq0 * dt;  q1 += dq1 * dt;  q2 += dq2 * dt;  q3 += dq3 * dt;

    float nq = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (nq < 1e-6f) { q0 = 1; q1 = q2 = q3 = 0; nq = 1; }
    q0 /= nq; q1 /= nq; q2 /= nq; q3 /= nq;

    // Euler ZYX
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    sinp = limitar(sinp, -1.0f, 1.0f);
    roll_s  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD2DEG;
    pitch_s = asinf(sinp) * RAD2DEG;
    yaw     = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD2DEG;
    roll  = roll_s  - trim_roll_deg;
    pitch = pitch_s - trim_pitch_deg;
  }

  // Tercera fila de la matriz de rotacion cuerpo->inercial. r33 es el termino
  // que proyecta el empuje sobre la vertical (ec. 29 de las notas) y las tres
  // juntas pasan el acelerometro al sistema inercial para el Kalman vertical.
  void filaZ(float &r31, float &r32, float &r33) const {
    r31 = 2.0f * (q1 * q3 - q0 * q2);
    r32 = 2.0f * (q2 * q3 + q0 * q1);
    r33 = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
  }

  // Pone el yaw actual como 0 (referencia de rumbo al armar). Rehace el
  // cuaternion con los angulos SIN trim para no mover la actitud estimada.
  void ponerYawCero() {
    desdeEuler(roll_s * DEG2RAD, pitch_s * DEG2RAD, 0.0f);
    yaw = 0;
  }
};

#endif // ACTITUD_H
