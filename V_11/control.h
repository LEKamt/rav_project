// =============================================================================
//  control.h  -  Controlador de actitud en cascada (Seccion 5 de las notas)
//
//  LAZO EXTERNO (lento, sobre los angulos). Se impone una dinamica de primer
//  orden al error de cada angulo de Euler, igual que las ec. (40) y (44):
//        phi_punto_d   = kp_phi   (phi_d   - phi)
//        theta_punto_d = kp_theta (theta_d - theta)
//        psi_punto_d   = kp_psi   (psi_d   - psi)
//
//  Esas son velocidades de los ANGULOS DE EULER, y el giroscopio mide (p,q,r)
//  en el sistema del CUERPO. La ec. (26) de las notas relaciona ambas:
//        phi_punto   = p + q sin(phi) tan(theta) + r cos(phi) tan(theta)
//        theta_punto = q cos(phi) - r sin(phi)
//        psi_punto   = q sin(phi)/cos(theta) + r cos(phi)/cos(theta)
//  Invirtiendo esa relacion se obtiene la referencia del lazo interno:
//        p_d =  phi_punto_d - psi_punto_d sin(theta)
//        q_d =  theta_punto_d cos(phi) + psi_punto_d cos(theta) sin(phi)
//        r_d = -theta_punto_d sin(phi) + psi_punto_d cos(theta) cos(phi)
//  La tercera fila es la misma relacion que la ec. (43) de las notas resuelta
//  para r_d. Cerca del hover (phi, theta pequenos) las tres se reducen a
//  p_d ~ phi_punto_d, q_d ~ theta_punto_d, r_d ~ psi_punto_d, que es la
//  simplificacion que mencionan las notas para inclinaciones menores a 20 deg.
//  Aqui se usa la expresion completa: no cuesta nada y evita el acoplamiento
//  roll-pitch-yaw cuando el dron si esta inclinado.
//
//  LAZO INTERNO (rapido, sobre la velocidad angular), ec. (42):
//        v_p = -kp_p (p - p_d)      [+ integral, extension practica]
//  y la entrada real de torque sale de la linealizacion por realimentacion,
//  ec. (41), que cancela el acoplamiento giroscopico:
//        u_x = Ix v_p + (Iz - Iy) q r
//        u_y = Iy v_q + (Ix - Iz) p r
//        u_z = Iz v_r + (Iy - Ix) p q
//
//  [V_3] EXTENSION ROBUSTA: con `robusto` activo, el integrador del lazo
//  interno se sustituye por un super-twisting sobre el mismo error. Ver
//  robusto.h para el porque y la regla de sintonia. La estructura de las notas
//  no cambia: el termino entra en v_p, antes de la ec. (41).
//
//  EXTENSION: termino integral en el lazo de velocidad angular. Las notas usan
//  P puro, que deja error de regimen en cuanto hay una asimetria constante
//  (centro de masa descentrado, un ESC con otro recorrido, una helice gastada).
//  Ese error constante es exactamente lo que en el codigo anterior se corregia
//  a mano con un "trim". El integrador lo hace solo. Viene en 0 por defecto:
//  primero se sintoniza el P puro, como en las notas, y despues se sube.
// =============================================================================
#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "robusto.h"   // [V_3]

class Control {
 public:
  float pd = 0, qd = 0, rd = 0;      // deg/s, referencia del lazo interno
  float ux = 0, uy = 0, uz = 0;      // N*m
  float ip = 0, iq = 0, ir = 0;      // integrales, rad/s^2
  float vp_ult = 0, vq_ult = 0, vr_ult = 0;   // [V_5] entradas virtuales, para identificar
  float exc_pqr[3] = {0, 0, 0};               // [V_5] excitacion para identificar
  SuperTwisting stp, stq, str;       // [V_3] termino robusto por eje
  float ep_ant = 0, eq_ant = 0, er_ant = 0;   // para el termino D

  void reset() {
    stp.reset(); stq.reset(); str.reset();   // [V_3]
    ip = 0; iq = 0; ir = 0;
    ep_ant = eq_ant = er_ant = 0;
    pd = qd = rd = 0;
    ux = uy = uz = 0;
  }

  // Angulos en deg, velocidades angulares en deg/s.
  // sp_yaw: rumbo en deg si yaw_por_angulo, o velocidad en deg/s si no.
  // integrar: false mientras el mezclador este saturado (anti-windup).
  void actualizar(const Ajustes &a,
                  float roll, float pitch, float yaw,
                  float p, float q, float r,
                  float sp_roll, float sp_pitch, float sp_yaw,
                  float dt, bool integrar, bool robusto,
                  const float *kp_eff) {
    // [V_5] kp_eff: ganancias del lazo interno a usar de verdad. En modo
    // adaptativo vienen calculadas desde la autoridad medida; si es nullptr se
    // usan las de los ajustes, que es el comportamiento de siempre.
    const float kpr0 = kp_eff ? kp_eff[0] : a.kp_rate[0];
    const float kpr1 = kp_eff ? kp_eff[1] : a.kp_rate[1];
    const float kpr2 = kp_eff ? kp_eff[2] : a.kp_rate[2];
    // El lazo externo no depende de la autoridad del actuador, solo del tiempo
    // de respuesta que se quiera; en modo adaptativo se fija por ancho de banda.
    float kp_ang_ef[3];
    for (int i = 0; i < 3; i++)
      kp_ang_ef[i] = a.adaptar ? limitar(1.0f / a.tau_ang, 0.1f, LIM_KP_ANG)
                               : a.kp_ang[i];

    // ---------- lazo externo: angulo -> velocidad de Euler deseada ----------
    float dphi   = kp_ang_ef[0] * (sp_roll  - roll);     // deg/s
    float dtheta = kp_ang_ef[1] * (sp_pitch - pitch);
    float dpsi;
    if (a.yaw_por_angulo)
      dpsi = kp_ang_ef[2] * envolverDeg(sp_yaw - yaw);
    else
      dpsi = sp_yaw;                                    // mando directo de velocidad

    dphi   = limitar(dphi,   -a.rate_max, a.rate_max);
    dtheta = limitar(dtheta, -a.rate_max, a.rate_max);
    dpsi   = limitar(dpsi,   -RATE_MAX_YAW, RATE_MAX_YAW);

    // ---------- de velocidades de Euler a (p,q,r) deseadas (inv. ec. 26) ----
    float cf = cosf(roll * DEG2RAD),  sf = sinf(roll * DEG2RAD);
    float ct = cosf(pitch * DEG2RAD), st = sinf(pitch * DEG2RAD);

    pd =  dphi   - dpsi * st;
    qd =  dtheta * cf + dpsi * ct * sf;
    rd = -dtheta * sf + dpsi * ct * cf;

    pd = limitar(pd, -a.rate_max, a.rate_max);
    qd = limitar(qd, -a.rate_max, a.rate_max);
    rd = limitar(rd, -RATE_MAX_YAW, RATE_MAX_YAW);

    // [V_5] Excitacion de identificacion: se suma A LA REFERENCIA del lazo
    // interno, no a la salida. Asi el dron la sigue como seguiria cualquier
    // otra orden, y lo que se identifica es el lazo tal y como vuela.
    pd += exc_pqr[0];  qd += exc_pqr[1];  rd += exc_pqr[2];

    // ---------- lazo interno: velocidad angular (ec. 42) --------------------
    // Todo en rad para que los torques salgan en N*m directamente.
    float ep = (pd - p) * DEG2RAD;      // rad/s
    float eq = (qd - q) * DEG2RAD;
    float er = (rd - r) * DEG2RAD;

    float dp = (dt > 1e-6f) ? (ep - ep_ant) / dt : 0.0f;
    float dq = (dt > 1e-6f) ? (eq - eq_ant) / dt : 0.0f;
    float dr = (dt > 1e-6f) ? (er - er_ant) / dt : 0.0f;
    ep_ant = ep;  eq_ant = eq;  er_ant = er;

    float vp, vq, vr;                                        // rad/s^2
    if (robusto) {
      // [V_3] Modo robusto: el super-twisting sustituye al integrador clasico.
      // El error de velocidad angular ES la superficie deslizante, porque tras
      // la linealizacion la planta de este lazo es de primer orden:
      //       s_punto = v + d(t)
      // Tener los dos (integrador y super-twisting) corrigiendo lo mismo
      // produce una oscilacion lenta, asi que el integrador se pone a cero.
      ip = 0; iq = 0; ir = 0;
      vp = kpr0 * ep + stp.calcular(ep, a.st_k1[0], a.st_k2[0], a.st_phi[0], dt, integrar, I_MAX_V);
      vq = kpr1 * eq + stq.calcular(eq, a.st_k1[1], a.st_k2[1], a.st_phi[1], dt, integrar, I_MAX_V);
      vr = kpr2 * er + str.calcular(er, a.st_k1[2], a.st_k2[2], a.st_phi[2], dt, integrar, I_MAX_V);
    } else {
      if (integrar) {
        ip = limitar(ip + a.ki_rate[0] * ep * dt, -I_MAX_V, I_MAX_V);
        iq = limitar(iq + a.ki_rate[1] * eq * dt, -I_MAX_V, I_MAX_V);
        ir = limitar(ir + a.ki_rate[2] * er * dt, -I_MAX_V, I_MAX_V);
      }
      stp.reset(); stq.reset(); str.reset();
      vp = kpr0 * ep + ip + a.kd_rate[0] * dp;
      vq = kpr1 * eq + iq + a.kd_rate[1] * dq;
      vr = kpr2 * er + ir + a.kd_rate[2] * dr;
    }

    vp_ult = vp;  vq_ult = vq;  vr_ult = vr;   // [V_5] para la identificacion

    // ---------- torque por linealizacion por realimentacion (ec. 41) --------
    float pr = p * DEG2RAD, qr = q * DEG2RAD, rr = r * DEG2RAD;
    ux = a.Ix * vp + (a.Iz - a.Iy) * qr * rr;
    uy = a.Iy * vq + (a.Ix - a.Iz) * pr * rr;
    uz = a.Iz * vr + (a.Iy - a.Ix) * pr * qr;
  }
};

#endif // CONTROL_H
