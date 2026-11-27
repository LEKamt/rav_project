// =============================================================================
//  identifica.h  -  Identificacion EN LINEA de la autoridad de control
//                   y ganancias que se adaptan al dron real
//
//  EL PROBLEMA
//  -----------
//  Todo el controlador de las notas se apoya en un modelo: Ix, Iy, Iz para
//  convertir aceleracion angular en torque (ec. 41), y kf, l, omega_max para
//  convertir torque en velocidad de helice (ec. 46). Si ese modelo esta
//  desviado, el torque que llega NO es el que se pidio, y el dron "intenta
//  controlar pero no lo consigue": las ganancias que sobre el papel dan un
//  ancho de banda de 15 rad/s dan en realidad 5, o 40.
//
//  De los parametros de este dron, omega_max NO esta medido: es una estimacion
//  a partir del KV y la tension de bateria. Un error del 30 % ahi es un error
//  del 70 % en la autoridad, porque el empuje va con omega al cuadrado.
//
//  QUE SE MIDE
//  -----------
//  Una sola cantidad por eje, adimensional:
//
//      g = (aceleracion angular REAL conseguida) / (la que se pidio)
//
//  Si el modelo fuera exacto, g = 1. Tras la linealizacion por realimentacion
//  la planta de cada eje es  p_punto = g * v_p, con v_p la entrada virtual en
//  rad/s^2 que ya calcula control.h. Entonces:
//
//      g < 1  el dron responde MENOS de lo pedido: falta autoridad
//      g > 1  responde de mas: las ganancias efectivas son mayores de lo
//             disenado y el lazo esta mas cerca de oscilar de lo que crees
//      g < 0  EL SIGNO ESTA INVERTIDO en ese eje: el control empuja hacia la
//             caida. Ninguna ganancia arregla eso.
//
//  Ese ultimo caso es el que vale su peso en oro: un signo invertido se
//  diagnostica en dos segundos en vez de con un dron roto.
//
//  COMO SE MIDE
//  ------------
//  Minimos cuadrados recursivos (RLS) escalar sobre y = g x, con
//      x = v_p pedida,   y = p_punto medida.
//  Dos detalles que hacen que funcione en un dron real:
//
//  1. p_punto sale de derivar el giroscopio, que es ruidoso. Se pasa por un
//     filtro de primer orden... y se pasa EL MISMO filtro a v_p. Al filtrar
//     las dos senales igual, el desfase que introduce el filtro se cancela en
//     el cociente y la estimacion no se sesga. Filtrar solo una es el error
//     clasico de este metodo.
//  2. Solo se aprende cuando hay excitacion: si |v_p| es pequena, el cociente
//     y/x es ruido dividido por ruido. Con poca excitacion el estimador se
//     queda quieto en vez de inventarse un numero.
//
//  El olvido (lambda < 1) hace que la estimacion siga al dron si cambia: la
//  bateria se descarga, se cambia una helice, se anade carga.
//
//  PARA QUE SIRVE
//  --------------
//  Con g medida, la ganancia que da el ancho de banda que TU pides sale de una
//  division. El lazo interno cerrado es
//      p_punto = g kp (p_d - p)      ->   constante de tiempo  tau = 1/(g kp)
//  asi que para un tau deseado:
//                          kp = 1 / (tau * g)
//  Eso es todo el "auto-tune": no hay que hacer oscilar el dron ni aplicar
//  reglas empiricas. Si g cambia en vuelo, kp se mueve con ella y el ancho de
//  banda se mantiene donde lo pusiste. Eso es el modo ADAPTATIVO.
// =============================================================================
#ifndef IDENTIFICA_H
#define IDENTIFICA_H

#include <Arduino.h>
#include <math.h>
#include "config.h"

class IdentEje {
 public:
  float g = 1.0f;        // autoridad real / autoridad del modelo
  float P = 1.0f;        // covarianza del estimador
  float calidad = 0;     // 0..1, cuanta excitacion util se ha visto
  bool  fiable = false;

  void reset() {
    g = 1.0f; P = 1.0f; calidad = 0; fiable = false;
    pf = 0; vf = 0; p_ant = 0; primera = true;
  }

  // p : velocidad angular medida (rad/s)
  // v : entrada virtual pedida por el control (rad/s^2)
  // activo : solo se aprende en vuelo y sin el mezclador saturado
  void actualizar(float p, float v, float dt, float lambda, bool activo) {
    if (dt < 1e-6f) return;
    if (primera) { p_ant = p; primera = false; return; }

    float pdot = (p - p_ant) / dt;
    p_ant = p;

    // MISMO filtro a las dos senales: el desfase se cancela en el cociente.
    float tau_f = 1.0f / (2.0f * PI * IDENT_FC_HZ);
    float a = dt / (dt + tau_f);
    pf += a * (pdot - pf);
    vf += a * (v    - vf);

    if (!activo) return;
    if (fabsf(vf) < IDENT_EXC_MIN) {        // sin excitacion no se aprende
      calidad *= 0.9995f;
      fiable = (calidad > 0.35f);
      return;
    }

    float x = vf, y = pf;
    float Px = P * x;
    float K  = Px / (lambda + x * Px);
    g += K * (y - g * x);
    P  = (P - K * x * P) / lambda;
    if (P > 1e4f)  P = 1e4f;
    if (P < 1e-6f) P = 1e-6f;
    // Se deja llegar a NEGATIVO a proposito: es el diagnostico de signo.
    g = limitar(g, -5.0f, 10.0f);

    calidad += 0.004f * (1.0f - calidad);
    fiable = (calidad > 0.35f);
  }

  // Ganancia que da la constante de tiempo pedida con la autoridad medida
  float kpRecomendada(float tau, float kp_max) const {
    float gg = fabsf(g);
    if (gg < 0.05f) gg = 0.05f;             // no dividir por casi cero
    if (tau < 0.01f) tau = 0.01f;
    return limitar(1.0f / (tau * gg), 0.1f, kp_max);
  }

 private:
  float pf = 0, vf = 0, p_ant = 0;
  bool  primera = true;
};

// --- Excitacion opcional ----------------------------------------------------
// Onda cuadrada pequena sobre la referencia de velocidad angular de UN eje,
// durante unos segundos. No hace falta para identificar -las perturbaciones
// normales del vuelo suelen bastar- pero la garantiza cuando el dron esta muy
// quieto. Es la unica parte del sistema que mueve el dron por su cuenta, asi
// que solo corre en vuelo y mientras la pagina la mantenga pedida.
class Excitador {
 public:
  int8_t   eje = -1;          // -1 = parado, 0=p, 1=q, 2=r
  uint32_t hasta = 0;
  float    amp = 0;           // deg/s

  float valor(int i, uint32_t ahora) const {
    if (eje != i || (int32_t)(ahora - hasta) > 0) return 0.0f;
    // medio periodo de IDENT_EXC_MS: cuadrada simetrica
    bool alto = ((ahora / IDENT_EXC_MS) & 1) != 0;
    return alto ? amp : -amp;
  }
  bool activo(uint32_t ahora) const { return eje >= 0 && (int32_t)(ahora - hasta) <= 0; }
};

#endif // IDENTIFICA_H
