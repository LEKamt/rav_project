// =============================================================================
//  robusto.h  -  Control robusto por modos deslizantes de segundo orden
//                (algoritmo super-twisting)
//
//  POR QUE HACE FALTA
//  ------------------
//  El controlador de las notas es linealizacion por realimentacion: cancela la
//  dinamica no lineal usando el MODELO. Las propias notas lo advierten al final
//  de la Seccion 5: "si el modelo tiene incertidumbre, con m o I no exactos, o
//  con kf, km que varian con la altura, desgaste de helices, densidad de aire,
//  etc., la cancelacion no es perfecta y en lugar de linealizar se introduce
//  error sistematico".
//
//  Y en este dron el modelo SI va a cambiar:
//    - la bateria pesa ~200 g y se descarga: m varia y el centro de masa se
//      mueve, asi que Ix, Iy cambian con ella;
//    - kf cae con el desgaste de las helices y con la tension de bateria;
//    - omega_max no esta medido, solo estimado a partir del KV;
//    - el ESC del rotor 2 es de otro modelo: su respuesta no es la de los otros;
//    - el efecto suelo cambia el empuje real por debajo de ~30 cm.
//
//  Cada una de esas cosas hace que el torque (o el empuje) que llega NO sea el
//  que el controlador pidio. Para el lazo cerrado eso es una PERTURBACION
//  ACOTADA, y de eso trata este archivo.
//
//  QUE SE ANADE
//  ------------
//  Sobre la ley nominal, sin tocarla, un termino de modos deslizantes. La planta
//  de cada lazo interno, tras la linealizacion imperfecta, es
//        s_punto = v + d(t)
//  donde s es el error (de velocidad angular, o la superficie de altura), v la
//  entrada virtual y d la perturbacion que junta TODO lo que el modelo no sabe.
//  Un deslizante clasico usaria  v = k sign(s), que es robusto pero conmuta a
//  tope en cada ciclo: con PWM a 50 Hz y ESCs de 30 A eso es castigar los
//  motores (chattering).
//
//  El super-twisting es de SEGUNDO orden y su salida es CONTINUA:
//        v = k1 |s|^(1/2) sign(s) + w
//        w_punto = k2 sign(s)
//  Converge en tiempo finito y compensa exactamente cualquier perturbacion con
//  derivada acotada, |d_punto| <= L, si
//        k2 > L        y       k1 > 1.5 sqrt(L)
//  sin necesidad de conocer d: el estado w CONVERGE a -d. Es decir, w es una
//  estimacion en linea de lo que el modelo esta equivocando, y por eso se
//  publica como telemetria: mirando w se ve cuanto se ha desviado el dron de su
//  modelo, en rad/s^2 o en m/s^2.
//
//  CAPA LIMITE
//  -----------
//  sign(s) se sustituye por sat(s/phi) dentro de una capa de grosor phi. Sin
//  eso, el ruido del giroscopio hace conmutar el termino en cada muestra. El
//  precio es que el error no converge a cero exacto sino a una banda del orden
//  de phi; con phi pequeno comparado con el error que importa, es irrelevante.
//
//  RELACION CON EL TERMINO INTEGRAL
//  --------------------------------
//  w hace el mismo trabajo que un integrador, pero mejor: un integrador lineal
//  tarda un tiempo que depende de la ganancia y puede pasarse; el super-twisting
//  converge en tiempo finito y su ganancia efectiva crece cuando el error es
//  grande. Por eso, con el modo robusto activo, el integrador clasico se
//  CONGELA: tener los dos compitiendo por la misma correccion es la receta de
//  una oscilacion lenta.
// =============================================================================
#ifndef ROBUSTO_H
#define ROBUSTO_H

#include <Arduino.h>
#include <math.h>
#include "config.h"

class SuperTwisting {
 public:
  float w = 0;       // estado integral: converge a menos la perturbacion
  float salida = 0;  // ultimo valor calculado

  void reset() { w = 0; salida = 0; }

  // Arranca w en un valor conocido (transferencia sin salto al activar el modo)
  void sembrar(float w0, float wmax) { w = limitar(w0, -wmax, wmax); }

  // s        : superficie deslizante (el error que se quiere llevar a cero)
  // k1, k2   : ganancias del super-twisting
  // phi      : grosor de la capa limite, en las mismas unidades que s
  // integrar : false congela w (anti-windup: actuador saturado)
  // wmax     : tope de w, en unidades de la entrada virtual
  float calcular(float s, float k1, float k2, float phi,
                 float dt, bool integrar, float wmax) {
    if (phi < 1e-4f) phi = 1e-4f;
    float sn = limitar(s / phi, -1.0f, 1.0f);      // sign(s) suavizado
    // |s|^(1/2) sign(s), continuo y con pendiente infinita en el origen: es lo
    // que da la convergencia en tiempo finito.
    float raiz = sqrtf(fabsf(s)) * sn;
    if (integrar) w = limitar(w + k2 * sn * dt, -wmax, wmax);
    salida = k1 * raiz + w;
    return salida;
  }

  // Perturbacion estimada (lo que el modelo no sabe), en unidades de la
  // entrada virtual. Es -w, por el signo con el que w entra en la ley.
  float perturbacion() const { return -w; }
};

#endif // ROBUSTO_H
