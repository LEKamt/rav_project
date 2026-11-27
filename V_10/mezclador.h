// =============================================================================
//  mezclador.h  -  De (empuje total, torques) a microsegundos por ESC
//
//  Implementa la ec. (27) de las notas y su inversa (46). Con
//      f_i = kf w_i^2        tau_i = km w_i^2
//  y los rotores en r1=(l,-l), r2=(l,l), r3=(-l,l), r4=(-l,-l):
//
//      f_t   = kf   ( w1^2 + w2^2 + w3^2 + w4^2 )
//      tau_x = kf l ( w1^2 - w2^2 - w3^2 + w4^2 )
//      tau_y = kf l ( w1^2 + w2^2 - w3^2 - w4^2 )
//      tau_z = km   ( w1^2 - w2^2 + w3^2 - w4^2 )
//
//  Invirtiendo (ec. 46, con u1 = -f_t/m ya despejado como f_t):
//
//      w1^2 = 1/4 ( f_t/kf + tau_x/(kf l) + tau_y/(kf l) + tau_z/km )
//      w2^2 = 1/4 ( f_t/kf - tau_x/(kf l) + tau_y/(kf l) - tau_z/km )
//      w3^2 = 1/4 ( f_t/kf - tau_x/(kf l) - tau_y/(kf l) + tau_z/km )
//      w4^2 = 1/4 ( f_t/kf + tau_x/(kf l) - tau_y/(kf l) - tau_z/km )
//
//  [V_9] PRIORIDAD ENTRE EJES. Esto sale de un registro de vuelo real, no de la
//  teoria. Mirando la asignacion de los motores durante un intento de despegue:
//
//      rango de motor gastado en roll :  17 988
//      rango de motor gastado en pitch:   6 129
//      rango de motor gastado en YAW  : 302 539     <-- el 94 % del total
//      y el mezclador saturado el 93 % del tiempo
//
//  La causa no es un controlador mal sintonizado: es la fisica del reparto.
//  El coste en rango de motor de un N*m es 1/(4 kf l) para roll y pitch, y
//  1/(4 km) para yaw. Con los coeficientes de este dron:
//
//      (kf * l) / km  =  5.19e-7 / 5.09e-8  =  10.2
//
//  es decir, UN N*m DE YAW CUESTA 10 VECES MAS RANGO DE MOTOR QUE UN N*m DE
//  ROLL. Eso no es un defecto del dron: es asi en cualquier cuadricoptero,
//  porque el yaw sale del par aerodinamico de arrastre (km) y no del brazo de
//  palanca (kf * l).
//
//  La version anterior escalaba los TRES torques por el mismo factor para
//  conservar la direccion del vector de torque. Elegante, y equivocado: un
//  yaw saturado se llevaba por delante la autoridad de roll y pitch, que es
//  justo la que evita que el dron se caiga de lado. Perder rumbo es molesto;
//  perder roll es estrellarse.
//
//  Ahora el reparto va por prioridad: primero el empuje, despues roll y pitch
//  con todo el rango que necesiten, y el yaw con lo que sobre (y nunca mas de
//  YAW_FRAC_MAX del recorrido). Se publican las dos escalas por separado.
//
//  SATURACION. Las notas (Seccion 4.2) resuelven la infactibilidad proyectando
//  al politopo P = { w_min^2 <= w_i^2 <= w_max^2 }, es decir recortando
//  componente a componente. Ese recorte es la proyeccion euclidiana, pero
//  cambia la DIRECCION del torque: si solo se recorta un motor, el dron recibe
//  un torque distinto del pedido, y eso aparece como acoplamiento entre ejes
//  justo cuando mas autoridad se necesita.
//
//  Aqui se hace en dos pasos:
//    1) se recorta el empuje comun al rango factible;
//    2) se escala el torque COMPLETO por el mayor factor s en [0,1] que cabe
//       dentro de la caja. Asi el torque entregado es s*(tau pedido): tiene la
//       misma direccion, solo menos magnitud, y los tres ejes se degradan
//       juntos. Al final se recorta igual (paso 1 de las notas) como garantia
//       numerica; despues del escalado ese recorte ya no deberia actuar.
//  El factor s se publica como `escala_sat`: si baja mucho de 1, falta
//  autoridad y hay que bajar ganancias o empuje.
// =============================================================================
#ifndef MEZCLADOR_H
#define MEZCLADOR_H

#include <Arduino.h>
#include <math.h>
#include "config.h"

struct SalidaMezcla {
  float w[4];        // rad/s
  float w2[4];       // (rad/s)^2
  float mando[4];    // 0..1, fraccion del recorrido del ESC
  int   pulso[4];    // us
  float escala;      // factor aplicado al torque de ROLL y PITCH (1 = sin recorte)
  float escala_yaw;  // [V_9] factor aplicado al torque de YAW
  bool  saturado;
};

class Mezclador {
 public:
  // Empuje total maximo que pueden dar los cuatro rotores (N)
  static float empujeMax(const Ajustes &a) {
    return 4.0f * a.kf * a.omega_max * a.omega_max;
  }
  // Empuje necesario para sostener el dron (N)
  static float empujeHover(const Ajustes &a) {
    return a.masa * GRAVEDAD;
  }
  // Relacion empuje/peso
  static float empujePeso(const Ajustes &a) {
    return empujeMax(a) / empujeHover(a);
  }
  // Torque maximo disponible en roll/pitch con el dron en hover (N*m)
  static float torqueMaxHover(const Ajustes &a) {
    float w2h = empujeHover(a) / (4.0f * a.kf);
    float w2M = a.omega_max * a.omega_max;
    float dw2 = fminf(w2M - w2h, w2h - w2minDe(a));
    return 2.0f * a.kf * a.brazo_l * dw2;
  }
  static float w2minDe(const Ajustes &a) {
    float wmin = OMEGA_MIN_FRAC * a.omega_max;
    return wmin * wmin;
  }

  // ft en N, torques en N*m
  static SalidaMezcla calcular(const Ajustes &a, float ft,
                               float tx, float ty, float tz) {
    SalidaMezcla s;
    const float w2min = w2minDe(a);
    const float w2max = a.omega_max * a.omega_max;

    const float kfl = a.kf * a.brazo_l;
    float base = ft / (4.0f * a.kf);                  // w^2 comun
    float dx = tx / (4.0f * kfl);
    float dy = ty / (4.0f * kfl);
    float dz = tz / (4.0f * a.km);

    // [V_9] El reparto se hace POR PRIORIDAD, no todo junto. Ver la nota de
    // arriba: roll y pitch primero, yaw con lo que sobre.
    float dxy[4], dzz[4];
    dxy[0] =  dx + dy;   dzz[0] =  dz;
    dxy[1] = -dx + dy;   dzz[1] = -dz;
    dxy[2] = -dx - dy;   dzz[2] =  dz;
    dxy[3] =  dx - dy;   dzz[3] = -dz;

    // 1) empuje comun dentro de la caja
    base = limitar(base, w2min, w2max);

    // 2) roll y pitch: se les da todo el rango que haga falta
    float esc = 1.0f;
    for (int i = 0; i < 4; i++) {
      if (dxy[i] > 1e-9f) {
        float lim = (w2max - base) / dxy[i];
        if (lim < esc) esc = lim;
      } else if (dxy[i] < -1e-9f) {
        float lim = (w2min - base) / dxy[i];
        if (lim < esc) esc = lim;
      }
    }
    esc = limitar(esc, 0.0f, 1.0f);

    // 3) yaw: solo con lo que queda, y ademas nunca mas de una fraccion del
    //    recorrido total. Sin ese segundo tope, a media potencia hay sitio de
    //    sobra y el yaw se lo quedaria igual, dejando a los motores pegados a
    //    sus limites y sin margen para la siguiente rafaga en roll.
    float margen = YAW_FRAC_MAX * (w2max - w2min);
    float esc_z = 1.0f;
    for (int i = 0; i < 4; i++) {
      float v = base + esc * dxy[i];
      if (dzz[i] > 1e-9f) {
        float techo = fminf(w2max, v + margen);
        float lim = (techo - v) / dzz[i];
        if (lim < esc_z) esc_z = lim;
      } else if (dzz[i] < -1e-9f) {
        float suelo = fmaxf(w2min, v - margen);
        float lim = (suelo - v) / dzz[i];
        if (lim < esc_z) esc_z = lim;
      }
    }
    esc_z = limitar(esc_z, 0.0f, 1.0f);

    s.escala     = esc;      // la que importa para no caerse
    s.escala_yaw = esc_z;
    s.saturado   = (esc < 0.999f);

    for (int i = 0; i < 4; i++) {
      float w2 = limitar(base + esc * dxy[i] + esc_z * dzz[i], w2min, w2max);
      s.w2[i] = w2;
      s.w[i]  = sqrtf(w2);
      s.mando[i] = mandoDeOmega(a, s.w[i]);
      s.pulso[i] = pulsoDeMando(a, i, s.mando[i]);
    }
    return s;
  }

  // w -> fraccion del recorrido util del ESC. Se asume que la velocidad de la
  // helice crece de forma aproximadamente lineal con el ancho de pulso dentro
  // del recorrido calibrado (modelo de primer orden en regimen, ec. 3 de las
  // notas con w_i = km_motor * v_i).
  static float mandoDeOmega(const Ajustes &a, float w) {
    float wmin = OMEGA_MIN_FRAC * a.omega_max;
    float den  = a.omega_max - wmin;
    if (den < 1e-3f) return 0.0f;
    return limitar((w - wmin) / den, 0.0f, 1.0f);
  }

  // Cada motor con SU recorrido medido: el mismo mando significa la misma
  // fraccion de recorrido en los cuatro, aunque sus ESC topen distinto.
  static int pulsoDeMando(const Ajustes &a, int i, float m) {
    if (i < 0 || i > 3) return PULSO_MIN_US;
    m = limitar(m, 0.0f, 1.0f);
    int p = a.pulso_arr[i] + (int)(m * (float)(a.pulso_top[i] - a.pulso_arr[i]));
    return (int)limitar((float)p, (float)PULSO_MIN_US, (float)a.pulso_top[i]);
  }

  static float empujeDeOmega(const Ajustes &a, float w) {
    return a.kf * w * w;
  }
};

#endif // MEZCLADOR_H
