// =============================================================================
//  altura.h  -  Estimacion y control de altura  (Seccion 5.1 de las notas)
//
//  CONVENCION. Se trabaja en el sistema inercial de las notas: z hacia ABAJO.
//  Por tanto z es NEGATIVA cuando el dron esta por encima del suelo, y la
//  altura que ve el usuario es  h = -z.  La conversion se hace solo en los
//  bordes (medida y pagina); todo el lazo esta escrito tal como en las notas,
//  para que se pueda comparar linea a linea con las ec. (35) y (36).
//
//  -------------------------------------------------------------------------
//  1. ESTIMADOR
//  -------------------------------------------------------------------------
//  Los dos sensores de distancia dan 20 medidas por segundo, con ruido y, el
//  ultrasonido, con rebotes. Derivarlas para sacar la velocidad vertical seria
//  inservible. El acelerometro, al reves, da 250 muestras por segundo pero su
//  integral se va en segundos por culpa del sesgo.
//
//  El filtro de Kalman combina las dos con un estado de TRES componentes:
//      x = [ z , z_punto , b ]
//  donde b es el sesgo de la aceleracion vertical (m/s^2). Estimar el sesgo es
//  lo que evita que la velocidad derive: sin ese tercer estado, un error de
//  0.1 m/s^2 en el acelerometro se convierte en 0.1 m/s de error de velocidad
//  por cada segundo de vuelo.
//
//  Prediccion (modelo de doble integrador con la aceleracion medida como
//  entrada, no como estado):
//      z      += z_punto dt + 0.5 (a_med - b) dt^2
//      z_punto+= (a_med - b) dt
//      b      += 0
//
//  La aceleracion vertical inercial sale de proyectar el acelerometro con la
//  tercera fila de la matriz de rotacion y volver a sumar la gravedad:
//      a_z = r31 ax + r32 ay + r33 az + g
//  En reposo y nivelado el acelerometro mide (0,0,-g), r33 = 1, y queda
//  a_z = -g + g = 0, como debe ser.
//
//  Correccion: se usa la distancia medida cuando hay una, con una R distinta
//  segun venga del ToF (preciso) o del ultrasonido (ruidoso).
//
//  -------------------------------------------------------------------------
//  2. CONTROL  (ec. 35 y 36)
//  -------------------------------------------------------------------------
//      v_z = z_dd_ref - kd_z (z_punto - z_punto_ref) - kp_z (z - z_ref)
//      u_1 = (v_z - g) / r33                 ->      f_t = -m u_1
//
//  u_1 cancela la gravedad y divide por r33 = cos(phi)cos(theta), que es la
//  proyeccion del empuje sobre la vertical: inclinado hay que empujar mas para
//  sostener la misma altura. Sustituyendo en la dinamica z_dd = g + r33 u_1
//  queda z_dd = v_z, lineal y desacoplada, y el error cumple
//      e_z'' + kd_z e_z' + kp_z e_z = 0.
//
//  [V_3] EXTENSION ROBUSTA: con `robusto` activo, el integrador se sustituye
//  por un super-twisting sobre la superficie sigma = lambda (z_ref - z) - z_punto.
//  Es el mismo trabajo pero con convergencia en tiempo finito y sin necesidad
//  de conocer cuanto se ha desviado el modelo. Ver robusto.h.
//
//  EXTENSION: termino integral. La cancelacion de la gravedad usa la masa y el
//  coeficiente kf del modelo. Si cualquiera de los dos esta un 10 % desviado
//  -y lo estara: la bateria pesa distinto llena que vacia, y kf cambia con el
//  desgaste de las helices- el empuje de equilibrio no es el que calcula el
//  modelo y el dron se queda colgado con un error constante. El integrador
//  absorbe esa diferencia. Tiene anti-windup y se congela cuando el empuje
//  pedido topa con el maximo o el minimo.
// =============================================================================
#ifndef ALTURA_H
#define ALTURA_H

#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "robusto.h"   // [V_3]

// ----------------------------------------------------------------------------
//  Filtro de Kalman de 3 estados
// ----------------------------------------------------------------------------
class EstimadorZ {
 public:
  float x[3] = {0, 0, 0};        // z (m, abajo +), z_punto (m/s), sesgo (m/s^2)
  float P[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  bool  inicializado = false;
  uint16_t rechazos = 0;         // [V_10] medidas seguidas descartadas
  uint32_t rechazos_tot = 0;     // [V_10] para la pagina / diagnostico
  uint32_t reanclajes   = 0;     // [V_10b] veces que el filtro tuvo que rehacerse

  void reiniciar(float z0) {
    x[0] = z0;  x[1] = 0;  x[2] = 0;  rechazos = 0;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) P[i][j] = (i == j) ? ((i == 2) ? 0.5f : 0.1f) : 0.0f;
    inicializado = true;
  }

  // a_z: aceleracion vertical inercial medida (m/s^2, hacia abajo positiva)
  void predecir(float a_z, float dt, float q_acel) {
    float u = a_z - x[2];
    x[0] += x[1] * dt + 0.5f * u * dt * dt;
    x[1] += u * dt;
    // x[2] constante

    // F = [[1, dt, -dt^2/2], [0, 1, -dt], [0, 0, 1]]
    const float F[3][3] = {
      {1.0f, dt, -0.5f * dt * dt},
      {0.0f, 1.0f, -dt},
      {0.0f, 0.0f, 1.0f}
    };
    // P = F P F' + Q
    float FP[3][3], Pn[3][3];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) {
        float s = 0;
        for (int k = 0; k < 3; k++) s += F[i][k] * P[k][j];
        FP[i][j] = s;
      }
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) {
        float s = 0;
        for (int k = 0; k < 3; k++) s += FP[i][k] * F[j][k];
        Pn[i][j] = s;
      }
    // Ruido de proceso: la incertidumbre entra por la aceleracion (q_acel) y,
    // mucho mas despacio, por la deriva del sesgo.
    Pn[0][0] += q_acel * 0.25f * dt * dt * dt * dt;
    Pn[1][1] += q_acel * dt * dt;
    // [V_10b] Antes 1e-4: el sesgo quedaba practicamente fijo. El log del
    // 26-11 (vuelo_20261126_165034) muestra que en vuelo el acelerometro
    // vertical se desvia ~0.9 m/s^2 y cambia con el empuje (vibracion): con
    // el sesgo congelado, el filtro integraba esa deriva como caida.
    Pn[2][2] += KAL_Q_SESGO * dt;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) P[i][j] = Pn[i][j];
    // [V_10b] Un dron de 1 kg a menos de 1.5 m del suelo no baja ni sube a
    // 3 m/s. Si el filtro lo cree, esta integrando basura: velocidad a 0.
    if (fabsf(x[1]) > ALT_VZ_ABSURDA) { x[1] = 0.0f; P[1][1] = 0.25f; P[1][0] = P[0][1] = P[1][2] = P[2][1] = 0.0f; }
  }

  // z_med en metros, hacia abajo positivo (es decir, -altura medida)
  void corregir(float z_med, float R) {
    if (!inicializado) { reiniciar(z_med); return; }
    float S = P[0][0] + R;
    if (S < 1e-9f) return;
    float y = z_med - x[0];
    // [V_10] Puerta de innovacion. Un rebote del ultrasonido o un ToF que se
    // sale de rango es un salto de decenas de cm en una sola muestra; con la R
    // pequena que tiene este filtro, se lo tragaba entero y lo convertia en
    // velocidad vertical, y el lazo de altura respondia a un movimiento que no
    // existia. Ahora lo que se aparta mas de 3 sigma se descarta... salvo que
    // se repita: tras ALT_RECHAZO_MAX seguidas, el que esta mal es el filtro, y
    // se acepta inflando la incertidumbre para que se reenganche.
    if (y * y > 9.0f * S) {
      rechazos_tot++;
      if (++rechazos < ALT_RECHAZO_MAX) return;
      // [V_10b] CORREGIDO. Antes se aceptaba la medida inflando P, pero la
      // velocidad estimada (que llego a -12 m/s en el log) se quedaba como
      // estaba: la altura saltaba a la medida y en 0.2 s volvia a caer, en
      // diente de sierra hacia valores negativos. Si varias medidas seguidas
      // contradicen al filtro, el que esta mal es el filtro: se reancla a la
      // medida con velocidad 0 y se conserva el sesgo aprendido.
      x[0] = z_med;  x[1] = 0.0f;
      for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) if (i != j) P[i][j] = 0.0f;
      P[0][0] = R;  P[1][1] = 0.25f;  P[2][2] += 0.1f;
      rechazos = 0;  reanclajes++;
      return;
    }
    rechazos = 0;
    float K[3] = { P[0][0] / S, P[1][0] / S, P[2][0] / S };
    for (int i = 0; i < 3; i++) x[i] += K[i] * y;
    float fila0[3] = { P[0][0], P[0][1], P[0][2] };
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) P[i][j] -= K[i] * fila0[j];
  }

  float z()    const { return x[0]; }
  float vz()   const { return x[1]; }
  float sesgo()const { return x[2]; }
};

// ----------------------------------------------------------------------------
//  Controlador de altura
// ----------------------------------------------------------------------------
class ControlAltura {
 public:
  float iz = 0;          // termino integral, en m/s^2 (unidades de v_z)
  float vz_mando = 0;    // entrada virtual v_z resultante
  float u1 = 0;          // entrada u1 de las notas
  float z_ref = 0;       // referencia interna, con rampa (m, abajo +)
  float sigma = 0;       // [V_3] superficie deslizante
  SuperTwisting stz;     // [V_3] termino robusto
  // [V_10] "Gravedad efectiva": el empuje de hover REAL del dron, aprendido al
  // entrar en el modo (f_t * r33 / m en ese instante), en m/s^2. Con el modelo
  // perfecto vale g. Ver la nota en calcular().
  float g_ef = GRAVEDAD;

  void reset(float z_actual) {
    iz = 0; z_ref = z_actual; vz_mando = 0; u1 = 0; sigma = 0; stz.reset();
  }

  // Lleva z_ref hacia el objetivo del usuario sin pedir saltos imposibles.
  void rampa(float z_objetivo, float dt, float vz_max) {
    float d = z_objetivo - z_ref;
    float paso = vz_max * dt;
    if (d >  paso) d =  paso;
    if (d < -paso) d = -paso;
    z_ref += d;
  }

  // Devuelve el empuje total f_t en newtons.
  //   z, vz : estado estimado (abajo positivo)
  //   r33   : cos(phi) cos(theta)
  float calcular(const Ajustes &a, float z, float vz, float r33,
                 float dt, float ft_min, float ft_max, bool robusto) {
    // Superficie deslizante del lazo de altura. Con lambda = kp_z/kd_z se
    // cumple, termino a termino,
    //      kd_z * sigma  ==  -kd_z (z_punto - 0) - kp_z (z - z_ref)
    // que es exactamente la ec. (36) con z_dd_ref = 0. Es decir: la ley nominal
    // de las notas YA es proporcional a la superficie; el modo robusto solo le
    // suma el termino que domina la incertidumbre.
    float lam = (a.kd_z > 1e-3f) ? (a.kp_z / a.kd_z) : 1.0f;
    sigma = lam * (z_ref - z) - vz;
    float ez = z - z_ref;

    if (robusto) {
      // [V_3] El super-twisting absorbe el error de m y de kf. Su estado w
      // converge a la perturbacion, asi que tambien sirve para MEDIR cuanto se
      // ha desviado el dron de su modelo.
      iz = 0;
      vz_mando = a.kd_z * sigma +
                 stz.calcular(sigma, a.stz_k1, a.stz_k2, a.stz_phi, dt, true, IZ_MAX);
    } else {
      stz.reset();
      vz_mando = a.kd_z * sigma + iz;
    }

    // ec. (35), con g sustituida por la gravedad efectiva aprendida.
    // [V_10] El termino -g de la ec. (35) es la prealimentacion del hover:
    // m*g/r33 es lo que el MODELO cree que sostiene al dron. Con la curva de
    // empuje medida (Excel, hoja Kf) y los ESC en 1040-1800 us, el modelo se
    // equivoca un ~9 % justo en el hover, y mas cuanto mas baja la bateria.
    // Antes eso lo tenia que absorber el integral, cuyo tope es 2 m/s^2 (20 %):
    // entraba ya medio cargado y, al bajar la bateria, topaba y el dron se iba
    // hundiendo. Ahora la prealimentacion es el hover MEDIDO al entrar, y el
    // integral solo corrige lo que cambie despues.
    if (r33 < 0.5f) r33 = 0.5f;       // mas de 60 grados: no tiene sentido
    u1 = (vz_mando - g_ef) / r33;
    float ft = -a.masa * u1;

    // Anti-windup: solo se integra si el empuje pedido cabe en lo que pueden
    // dar los motores. Si no, seguir integrando solo cargaria el termino.
    // [V_6] El anti-windup mira contra los topes REALES que le pasa el lazo,
    // que ahora son relativos al hover. Si el sensor miente, el empuje topa
    // enseguida y el integral deja de crecer en vez de seguir cargandose.
    bool dentro = (ft > ft_min) && (ft < ft_max);
    if (dentro && !robusto) iz = limitar(iz - a.ki_z * ez * dt, -IZ_MAX, IZ_MAX);
    if (!dentro && robusto) stz.w = limitar(stz.w, -IZ_MAX, IZ_MAX);

    return limitar(ft, ft_min, ft_max);
  }
};

#endif // ALTURA_H
