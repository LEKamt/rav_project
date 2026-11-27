// =============================================================================
//  config.h  -  F450 / ESP32-S3  -  V_9
//  Pines, parametros fisicos del dron, limites de seguridad y ajustes por
//  defecto. TODO lo que se puede tocar sin recompilar vive en `struct Ajustes`
//  (se edita desde la pagina web y se guarda en memoria NVS).
//
//  CONVENCION DE EJES (la de las notas de clase, Seccion 4.2):
//    Sistema del cuerpo {B}:  x hacia ADELANTE,  y hacia la DERECHA,
//                             z hacia ABAJO  (convencion aeronautica, FRD).
//    Angulos de Euler ZYX:    roll (phi) sobre x, pitch (theta) sobre y,
//                             yaw (psi) sobre z.
//    Velocidad angular en {B}: (p, q, r).
//
//  POSICION DE LOS ROTORES (notas, ec. 27):
//      r1 = ( l, -l, 0)   adelante-izquierda
//      r2 = ( l,  l, 0)   adelante-derecha
//      r3 = (-l,  l, 0)   atras-derecha
//      r4 = (-l, -l, 0)   atras-izquierda
//  Sentido de giro: 1 y 3 en un sentido, 2 y 4 en el contrario, de modo que
//      tau_z = km (w1^2 - w2^2 + w3^2 - w4^2)
// =============================================================================
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ----------------------------------------------------------------------------
// 1. PINES
// ----------------------------------------------------------------------------
// Motores: el indice es el NUMERO DE ROTOR de las notas (0 -> rotor 1, etc.),
// NO el orden en que esten etiquetados los cables. Verificalo con la prueba
// individual de motores de la pagina (pestana CALIBRAR) antes de volar.
//                                   rotor1  rotor2  rotor3  rotor4
//                                   ad-izq  ad-der  at-der  at-izq
static const uint8_t PIN_MOTOR[4] = {   8,     17,     21,     18 };

#define PIN_I2C0_SDA   1      // MPU-6500 / MPU-6050
#define PIN_I2C0_SCL  39
#define PIN_I2C1_SDA   4      // VL53L0X (ToF)
#define PIN_I2C1_SCL   5
#define PIN_US_TRIG    6      // HC-SR04
#define PIN_US_ECHO    7
#define PIN_VBAT      -1      // ADC con divisor resistivo; -1 = no instalado

#define DIVISOR_VBAT  11.0f   // (R1+R2)/R2 del divisor, si PIN_VBAT >= 0

// ----------------------------------------------------------------------------
// 2. ORIENTACION DE LA IMU  (sensor -> cuerpo FRD)
// ----------------------------------------------------------------------------
// IMU_EJE[k]   = que eje del SENSOR alimenta el eje k del CUERPO (0=x,1=y,2=z)
// IMU_SIGNO[k] = signo con el que entra.
//
// El valor por defecto {1,0,2} / {+1,+1,-1} corresponde a: la IMU girada 90
// grados respecto del frente del dron y con su eje z apuntando HACIA ARRIBA
// (es el montaje del codigo anterior). Ese codigo usaba {1,0,2} con los tres
// signos positivos, lo que intercambia dos ejes sin invertir ninguno: eso
// produce un sistema IZQUIERDO y es la causa de que el giroscopio y el angulo
// se contradijeran. Aqui el z se invierte y el sistema vuelve a ser derecho.
//
// COMO VERIFICARLO (pestana SENSORES, dron desarmado, en la mano):
//   - inclinar el dron hacia la DERECHA  -> roll  debe subir (+)
//   - levantar la NARIZ                  -> pitch debe subir (+)
//   - girar la nariz a la DERECHA        -> yaw   debe subir (+)
//   - y mientras se mueve, p/q/r deben tener el MISMO signo que el angulo
//     que esta cambiando. Si alguno sale al reves, cambia aqui su signo.
static const uint8_t IMU_EJE[3]   = {  1,   0,   2  };
// MEDIDO EN BANCO 2026-11-25: con {+1,+1,-1} el dron daba roll NEGATIVO al
// inclinarlo a la derecha y pitch POSITIVO con la nariz abajo, es decir roll y
// pitch invertidos los dos a la vez, con el yaw correcto. Eso es exactamente
// una IMU montada girada 180 grados sobre z: invertir x e y lo corrige y el
// sistema sigue siendo derecho (determinante +1). Si solo uno de los dos
// hubiera salido invertido, el sistema seria IZQUIERDO, que no puede salir de
// ningun montaje fisico: seria un error de mapeo, no de montaje.
static const float   IMU_SIGNO[3] = { -1., -1., -1. };

// ----------------------------------------------------------------------------
// 3. PWM A LOS ESC
// ----------------------------------------------------------------------------
// 50 Hz es lo que acepta cualquier ESC de 30 A y es la frecuencia con la que
// estan calibrados los de este dron. El precio es un retardo de actuacion de
// hasta 20 ms, que es lo que limita la ganancia util del lazo de velocidad
// angular. Si los ESC son BLHeli/SimonK se puede subir a 250 o 400 Hz y el
// lazo gana mucho margen: cambia PWM_FREQ, recalibra el recorrido y vuelve a
// sintonizar desde ganancias bajas.
#define PWM_FREQ        50
#define PWM_RES         14
#define PWM_PERIODO_US  (1000000UL / PWM_FREQ)

#define PULSO_MIN_US   1000   // pulso de "parado" / armado del ESC
#define PULSO_MAX_US   2000   // tope fisico del protocolo

// Recorrido util MEDIDO de cada ESC (us). El ESC del rotor 2 es de otro
// modelo y no acepta la calibracion de recorrido: topa en 1380 us mientras
// los otros topan en 1780-1880. Por eso cada motor tiene SU propio mapeo:
// asi una misma velocidad angular pedida da el mismo empuje en los cuatro.
// Volver a medirlos con las helices FUERA si se cambia un ESC o la bateria.
//                                           r1    r2    r3    r4
// MEDIDO 2026-11-25: los cuatro ESC quedaron con el MISMO recorrido, 1040-1800.
// Antes el del rotor 2 topaba en 1380 y hacia falta compensarlo. El mapeo por
// motor se mantiene igual de util: si vuelve a cambiarse un ESC, basta editar
// su fila aqui o desde la pagina, sin tocar el control.
static const int PULSO_ARRANQUE_DEF[4] = { 1040, 1040, 1040, 1040 };
static const int PULSO_TOPE_DEF[4]     = { 1800, 1800, 1800, 1800 };

// ----------------------------------------------------------------------------
// 4. PARAMETROS FISICOS DEL DRON  (identificacion, hoja de parametros)
// ----------------------------------------------------------------------------
#define MASA_DEF        1.0022f     // kg
#define BRAZO_L_DEF     0.1616f     // m   (brazo efectivo en X: L_real/sqrt(2))
#define IX_DEF          0.0097f     // kg*m^2
#define IY_DEF          0.0106f     // kg*m^2
#define IZ_DEF          0.0201f     // kg*m^2
#define KF_DEF          3.2107e-6f  // N / (rad/s)^2     f_i = kf * w_i^2
#define KM_DEF          5.0879e-8f  // N*m / (rad/s)^2   tau_i = km * w_i^2
#define GRAVEDAD        9.81f

// Velocidad angular maxima de la helice, con carga, a bateria cargada.
// Estimacion:  w_max ~= KV * V_bat * (2*pi/60) * eficiencia_bajo_carga
//   1000 KV * 11.1 V * 0.10472 * 0.90 ~= 1046 rad/s
// Es el unico parametro del mapeo que NO viene medido: se ajusta desde la
// pagina (pestana MODELO) comparando el empuje de hover real con el teorico.
#define OMEGA_MAX_DEF   1046.0f     // rad/s
#define OMEGA_MIN_FRAC  0.15f       // w al pulso de arranque, como fraccion

// ----------------------------------------------------------------------------
// 5. LAZO DE CONTROL
// ----------------------------------------------------------------------------
#define F_CONTROL_HZ    250                      // nucleo 1
#define DT_CONTROL      (1.0f / F_CONTROL_HZ)
#define MS_CONTROL      (1000 / F_CONTROL_HZ)    // 4 ms -> 4 ticks de FreeRTOS
#define F_SENSORES_HZ   20                       // ToF + ultrasonido, nucleo 0
#define F_LOG_HZ        50                       // muestras para las graficas

// Ganancias por defecto. Estructura exacta de las notas (Seccion 5):
//   lazo externo (angulo -> velocidad angular deseada), ec. (40) y (44):
//       p_d = kp_ang_roll  * (phi_d  - phi)
//   lazo interno (velocidad angular), ec. (42):
//       v_p = -kp_rate_p * (p - p_d)      [+ termino integral, extension]
//   torque, ec. (41):
//       u_x = Ix*v_p + (Iz - Iy) q r
#define KP_ANG_ROLL_DEF    6.0f     // 1/s
#define KP_ANG_PITCH_DEF   6.0f
#define KP_ANG_YAW_DEF     2.5f    // [V_9] bajada: el yaw es caro en rango de motor

#define KP_RATE_P_DEF     15.0f     // 1/s
#define KP_RATE_Q_DEF     15.0f
#define KP_RATE_R_DEF      5.0f    // [V_9] idem
#define KI_RATE_P_DEF      0.0f     // 1/s^2  (subir a ~1.0 desde la pagina)
#define KI_RATE_Q_DEF      0.0f
#define KI_RATE_R_DEF      0.0f
#define KD_RATE_P_DEF      0.0f     // s^0    (derivada de la velocidad = ruido)
#define KD_RATE_Q_DEF      0.0f
#define KD_RATE_R_DEF      0.0f

#define I_MAX_V           15.0f     // tope del termino integral, en rad/s^2

// --- [V_2] Lazo de altura, ec. (35)-(36) ---------------------------------
// e_z'' + kd_z e_z' + kp_z e_z = 0.  Con kp_z = 4 y kd_z = 3.6 sale
// omega_n = 2 rad/s y zeta = 0.9: sube a una altura nueva en ~2 s sin pasarse.
// Subir kp_z mas alla de ~8 no ayuda: lo limita el ruido del sensor de
// distancia, no la ganancia.
#define KP_Z_DEF           4.0f     // 1/s^2
#define KD_Z_DEF           3.6f     // 1/s
// El integral NO es opcional aqui, al contrario que en el lazo de actitud: la
// cancelacion de la gravedad usa m y kf del modelo, y si uno esta desviado un
// 10 % el dron se cuelga con error constante. Ver la nota de altura.h.
#define KI_Z_DEF           1.0f     // 1/s^3
// [V_6] Bajado de 4.0 a 2.0. Cuatro m/s^2 son el 40 % de g: si el sensor de
// altura miente, el integral solo ya puede pedir media vez el peso de mas.
#define IZ_MAX             2.0f     // tope del integral, en m/s^2 (~0.2 g)

// --- [V_6] Topes duros del lazo de altura --------------------------------
// El empuje que puede pedir el lazo de altura se limita CONTRA EL HOVER, no
// contra el maximo de los motores. Aunque el sensor mienta, el dron nunca
// recibe mas de un 35 % por encima de lo que necesita para sostenerse: sube,
// pero despacio y controlable, en vez de irse al techo a toda potencia.
#define ALT_EMPUJE_MAX_K   1.35f    // veces el empuje de hover
#define ALT_EMPUJE_MIN_K   0.45f    // y nunca por debajo de esto: no se apaga

// Techo: por encima de la referencia maxima mas este margen se sale del modo.
#define ALT_TECHO_MARGEN   0.30f    // m

// Detector de SENSOR ATASCADO. Si el lazo lleva este tiempo pidiendo mas
// empuje del que hace falta para sostenerse y la altura MEDIDA no ha subido,
// la medida no esta siguiendo a la realidad: o el ultrasonido esta cegado por
// las helices, o el ToF esta fuera de rango devolviendo basura. Es exactamente
// el fallo que manda el dron al techo, porque el lazo "ve" que no sube y pide
// mas. Al detectarlo se sale del modo y el empuje vuelve al de hover.
#define ALT_ATASCO_MS      2500     // ms pidiendo subir sin subir
#define ALT_ATASCO_K       1.12f    // empuje/hover a partir del cual deberia subir
#define ALT_ATASCO_DH      0.05f    // m que como minimo deberia haber subido

#define VZ_MAX_DEF         0.5f     // m/s, rapidez con que se mueve la referencia
#define ALT_MAX_DEF        1.0f     // m: hasta donde llega bien el ultrasonido
#define ALT_MIN_HOLD       0.15f    // m, por debajo de esto no se mantiene altura

// Ruidos del filtro de Kalman vertical
// --- [V_3] Control robusto: super-twisting -------------------------------
// Regla de sintonia: si |d_punto| <= L (lo rapido que puede cambiar lo que el
// modelo ignora), basta con  k2 > L  y  k1 > 1.5 sqrt(L).
//
// Lazo de velocidad angular, con s en rad/s y salida en rad/s^2. Si Ix esta
// desviado un 20 % y la entrada virtual anda por 20 rad/s^2, la perturbacion
// vale ~4 rad/s^2 y puede moverse a ~20 rad/s^3: k2 = 8 y k1 = 3 dan margen
// de sobra sin pedirle al ESC mas de lo que puede seguir a 50 Hz.
#define ST_K1_PQ_DEF       3.0f
#define ST_K2_PQ_DEF       8.0f
#define ST_K1_R_DEF        2.0f     // yaw: menos autoridad, ganancias menores
#define ST_K2_R_DEF        5.0f
// Capa limite, en rad/s. 0.10 rad/s = 5.7 deg/s, por encima del ruido del
// giroscopio ya filtrado: por debajo de eso el termino deja de conmutar.
#define ST_PHI_PQ_DEF      0.10f
#define ST_PHI_R_DEF       0.15f

// Lazo de altura: s en m/s, salida en m/s^2. Lo que tiene que absorber es el
// error de m y de kf, del orden de 1-2 m/s^2.
#define STZ_K1_DEF         1.5f
#define STZ_K2_DEF         2.0f
#define STZ_PHI_DEF        0.10f    // m/s

#define ROBUSTO_ATT_DEF    0        // se arranca en nominal y se activa volando
#define ROBUSTO_ALT_DEF    0

// --- [V_5] Identificacion en linea y ganancias adaptativas ----------------
// Se mide, por eje, g = aceleracion angular real / la pedida. Ver identifica.h.
#define IDENT_FC_HZ        8.0f     // filtro comun a p_punto y a v_p
#define IDENT_EXC_MIN      1.0f     // rad/s^2: por debajo no hay que aprender
#define IDENT_LAMBDA_DEF   0.9990f  // olvido: sigue al dron si cambia
#define IDENT_EXC_MS       250      // medio periodo de la excitacion opcional
#define IDENT_EXC_AMP      40.0f    // deg/s de la excitacion opcional
#define IDENT_EXC_MAX_MS   4000     // dura como mucho 4 s

// En modo ADAPTATIVO no se fija la ganancia sino el ANCHO DE BANDA: la
// constante de tiempo que quieres en cada lazo. La ganancia sale de
// kp = 1/(tau*g) con la g medida, asi que el comportamiento se mantiene aunque
// el dron cambie. tau = 0.067 s equivale a kp = 15 cuando g = 1, que es lo que
// traian las versiones anteriores.
#define TAU_RATE_DEF       0.067f   // s, lazo de velocidad angular
#define TAU_ANG_DEF        0.170f   // s, lazo de angulo
#define ADAPTAR_DEF        0        // se arranca en manual

#define KAL_Q_DEF          0.6f     // (m/s^2)^2, ruido de la aceleracion
// [V_5] MEDIDO: en este dron el VL53L0X solo es creible por debajo de ~20 cm
// (luz ambiente y reflectividad del suelo lo ciegan enseguida), mientras que el
// HC-SR04 mide bien hasta ~1 m. Asi que se invierte la prioridad respecto de
// las versiones anteriores: manda el ULTRASONIDO, y el ToF solo cubre el tramo
// bajo, donde el ultrasonido tiene su zona muerta.
#define TOF_MAX_FIABLE     0.25f    // m, por encima el ToF se ignora
#define ALT_CAMBIO         0.20f    // m, frontera entre los dos
#define ALT_HISTERESIS     0.05f    // m, para que no salte en la frontera
#define US_MIN_FIABLE      0.08f    // m, zona muerta del HC-SR04
#define US_MAX_FIABLE      1.60f    // m

#define KAL_R_TOF          1e-4f    // m^2, ToF de cerca (sigma ~ 1 cm)
#define KAL_R_US           6e-4f    // m^2, ultrasonido (sigma ~ 2.5 cm)
#define ALT_TIMEOUT_MS     1000     // sin medida valida -> se sale de mantener altura
// [V_7] Si pasa este tiempo sin NINGUNA medida, el filtro vertical se apaga en
// vez de seguir integrando el acelerometro. Sin una medida que lo ancle, un
// sesgo de 0.1 m/s^2 son 0.05 m de error al segundo, 5 m al minuto y kilometros
// en un rato: es exactamente como se llegaba a leer -15880 m estando parado.
#define ALT_RESET_MS       3000
#define ALT_LIMITE_ABS     60.0f    // m, cualquier cosa fuera de esto es basura

// Filtro de actitud (Mahony). Kp fija la rapidez con la que el acelerometro
// corrige la deriva del giroscopio; Ki estima el sesgo del giroscopio.
#define MAHONY_KP_DEF      2.0f
#define MAHONY_KI_DEF      0.05f

// ----------------------------------------------------------------------------
// 6. LIMITES Y SEGURIDAD
// ----------------------------------------------------------------------------
#define ANG_MAX_DEF       25.0f    // tope de la referencia de roll/pitch (deg)
#define RATE_MAX_DEF     220.0f    // tope de p_d, q_d (deg/s)
#define RATE_MAX_YAW     120.0f    // tope de r_d (deg/s)
#define ANG_CORTE        60.0f     // mas inclinacion que esto -> desarme
#define EMPUJE_MAX_FRAC   0.85f    // tope del acelerador de la pagina
#define EMPUJE_ARMADO     0.00f    // empuje al armar: SIEMPRE 0
#define LATIDO_TIMEOUT_MS 800      // sin latido de la pagina -> bajada
#define BAJADA_POR_S      0.20f    // fraccion de empuje que baja por segundo
#define PRUEBA_MOTOR_MS   1500     // la prueba individual se corta sola
// [V_8] Paso del ajuste fino de nivel. 0.2 grados por toque: fino para afinar
// un hover y grande para notarse en pocos toques.
#define TRIM_PASO          0.2f

// [V_9] Fraccion maxima del recorrido de los motores que puede gastarse en
// yaw. Con 0.30, aunque el lazo de yaw pida un torque enorme, siempre quedan
// dos tercios del recorrido para que roll y pitch sostengan el dron.
#define YAW_FRAC_MAX       0.30f

// ----------------------------------------------------------------------------
// 7. RED
// ----------------------------------------------------------------------------
#define WIFI_RED     "F450-RAV"
#define WIFI_CLAVE   "f450vuelo"   // minimo 8 caracteres
#define WIFI_CANAL   6

// ----------------------------------------------------------------------------
// 8. AJUSTES EDITABLES EN CALIENTE (se guardan en NVS)
// ----------------------------------------------------------------------------
struct Ajustes {
  // control
  float kp_ang[3];      // roll, pitch, yaw   (lazo externo)
  float kp_rate[3];     // p, q, r            (lazo interno)
  float ki_rate[3];
  float kd_rate[3];
  float kp_z, kd_z, ki_z;   // [V_2] lazo de altura
  float vz_max, alt_max;    // [V_2] rampa y tope de la referencia
  float kal_q;              // [V_2] ruido de aceleracion del Kalman vertical
  // [V_3] control robusto (super-twisting)
  uint8_t robusto_att, robusto_alt;
  float st_k1[3], st_k2[3], st_phi[3];   // p, q, r
  float stz_k1, stz_k2, stz_phi;
  // [V_5] adaptacion
  uint8_t adaptar;          // 1 = las ganancias salen de la autoridad medida
  float tau_rate, tau_ang;  // anchos de banda pedidos (s)
  float ident_lambda;       // olvido del estimador
  float ang_max;        // deg
  float rate_max;       // deg/s
  uint8_t yaw_por_angulo;  // 1 = mantiene rumbo, 0 = solo velocidad de yaw

  // estimador
  float mahony_kp, mahony_ki;
  float trim_roll, trim_pitch;   // deg, correccion de nivel del montaje

  // modelo fisico
  float masa, brazo_l, Ix, Iy, Iz, kf, km, omega_max;

  // actuadores
  int pulso_arr[4], pulso_top[4];

  uint32_t magia;       // sello de version del bloque guardado
};

#define AJUSTES_MAGIA 0x46343509u   // "F45" + version 09 (V_9: prioridad roll/pitch sobre yaw)

inline void ajustesPorDefecto(Ajustes &a) {
  a.kp_ang[0] = KP_ANG_ROLL_DEF;  a.kp_ang[1] = KP_ANG_PITCH_DEF;  a.kp_ang[2] = KP_ANG_YAW_DEF;
  a.kp_rate[0] = KP_RATE_P_DEF;   a.kp_rate[1] = KP_RATE_Q_DEF;    a.kp_rate[2] = KP_RATE_R_DEF;
  a.ki_rate[0] = KI_RATE_P_DEF;   a.ki_rate[1] = KI_RATE_Q_DEF;    a.ki_rate[2] = KI_RATE_R_DEF;
  a.kd_rate[0] = KD_RATE_P_DEF;   a.kd_rate[1] = KD_RATE_Q_DEF;    a.kd_rate[2] = KD_RATE_R_DEF;
  a.kp_z = KP_Z_DEF;  a.kd_z = KD_Z_DEF;  a.ki_z = KI_Z_DEF;
  a.vz_max = VZ_MAX_DEF;  a.alt_max = ALT_MAX_DEF;  a.kal_q = KAL_Q_DEF;
  a.robusto_att = ROBUSTO_ATT_DEF;  a.robusto_alt = ROBUSTO_ALT_DEF;
  a.st_k1[0] = ST_K1_PQ_DEF;  a.st_k1[1] = ST_K1_PQ_DEF;  a.st_k1[2] = ST_K1_R_DEF;
  a.st_k2[0] = ST_K2_PQ_DEF;  a.st_k2[1] = ST_K2_PQ_DEF;  a.st_k2[2] = ST_K2_R_DEF;
  a.st_phi[0] = ST_PHI_PQ_DEF; a.st_phi[1] = ST_PHI_PQ_DEF; a.st_phi[2] = ST_PHI_R_DEF;
  a.stz_k1 = STZ_K1_DEF;  a.stz_k2 = STZ_K2_DEF;  a.stz_phi = STZ_PHI_DEF;
  a.adaptar = ADAPTAR_DEF;
  a.tau_rate = TAU_RATE_DEF;  a.tau_ang = TAU_ANG_DEF;
  a.ident_lambda = IDENT_LAMBDA_DEF;
  a.ang_max  = ANG_MAX_DEF;
  a.rate_max = RATE_MAX_DEF;
  a.yaw_por_angulo = 1;
  a.mahony_kp = MAHONY_KP_DEF;  a.mahony_ki = MAHONY_KI_DEF;
  a.trim_roll = 0.0f;           a.trim_pitch = 0.0f;
  a.masa = MASA_DEF;  a.brazo_l = BRAZO_L_DEF;
  a.Ix = IX_DEF;      a.Iy = IY_DEF;    a.Iz = IZ_DEF;
  a.kf = KF_DEF;      a.km = KM_DEF;    a.omega_max = OMEGA_MAX_DEF;
  for (int i = 0; i < 4; i++) {
    a.pulso_arr[i] = PULSO_ARRANQUE_DEF[i];
    a.pulso_top[i] = PULSO_TOPE_DEF[i];
  }
  a.magia = AJUSTES_MAGIA;
}

// Topes duros: ninguna ganancia que llegue por la web puede salir de aqui.
#define LIM_KP_ANG    30.0f
#define LIM_KP_RATE   80.0f
#define LIM_KI_RATE   40.0f
#define LIM_KD_RATE    2.0f
#define LIM_KP_Z      20.0f
#define LIM_KD_Z      20.0f
#define LIM_KI_Z      10.0f
#define LIM_ST_K1     20.0f
#define LIM_ST_K2     50.0f
#define LIM_ST_PHI     2.0f
#define LIM_TAU_MIN    0.02f
#define LIM_TAU_MAX    0.60f

// ----------------------------------------------------------------------------
// 9. UTILIDADES
// ----------------------------------------------------------------------------
#define RAD2DEG  57.2957795f
#define DEG2RAD  0.0174532925f

inline float limitar(float v, float lo, float hi) {
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

// Lleva un angulo en grados al rango (-180, 180]
inline float envolverDeg(float a) {
  while (a >  180.0f) a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
  return a;
}

#endif // CONFIG_H
