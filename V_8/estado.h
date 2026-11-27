// =============================================================================
//  estado.h  -  Estado compartido entre los tres hilos
//
//  nucleo 1 : tareaControl   (250 Hz)  ESCRIBE actitud, control y motores
//  nucleo 0 : tareaSensores  ( 20 Hz)  ESCRIBE ToF, ultrasonido, bateria
//  nucleo 0 : tareaWeb       (demanda) LEE todo y escribe las referencias
//
//  No hay semaforos: cada campo es de 32 bits y en el ESP32 una escritura
//  alineada de 32 bits es atomica, asi que un lector nunca ve medio numero.
//  Lo unico que podria pasar es leer dos campos de dos ciclos distintos, que
//  para telemetria no importa. Para el buffer de la grafica si importa el
//  orden: se escribe la muestra y DESPUES se sube logSeq.
// =============================================================================
#ifndef ESTADO_H
#define ESTADO_H

#include <Arduino.h>
#include "config.h"

// --- Modos de vuelo -----------------------------------------------------
enum Modo : uint8_t {
  MODO_DESARMADO = 0,
  MODO_ARMADO    = 1,   // motores al ralenti, control activo, empuje 0
  MODO_VUELO     = 2,   // con empuje
  MODO_BAJADA    = 3,   // failsafe: perdio el enlace, bajando solo
  MODO_PRUEBA    = 4    // prueba individual de motores (helices fuera)
};

// --- [V_2] Que decide el empuje total -----------------------------------
enum ModoAltura : uint8_t {
  ALT_MANUAL = 0,       // la barra manda empuje directamente
  ALT_MANTENER = 1      // lazo de altura cerrado (ec. 35-36)
};

// --- Causa del ultimo desarme automatico --------------------------------
enum Fallo : uint8_t {
  FALLO_NINGUNO = 0,
  FALLO_INCLINACION,     // paso de ANG_CORTE
  FALLO_IMU,             // la IMU dejo de responder
  FALLO_ENLACE,          // bajada por failsafe terminada
  FALLO_USUARIO,         // boton PARAR
  FALLO_ALTURA           // [V_2] se perdio la medida de altura manteniendo
};

struct Estado {
  // ---- actitud y velocidad angular (nucleo 1) ----
  float roll, pitch, yaw;          // deg
  float p, q, r;                   // deg/s  (cuerpo)
  float q0, q1, q2, q3;            // cuaternion cuerpo->inercial

  // ---- IMU cruda, ya en ejes del cuerpo (nucleo 1) ----
  float ax, ay, az;                // g
  float gx, gy, gz;                // deg/s, sin sesgo
  float bias_gx, bias_gy, bias_gz; // deg/s, sesgo estimado
  float temp_imu;                  // C
  bool  imu_ok;

  // ---- referencias ----
  float sp_roll, sp_pitch, sp_yaw; // deg
  float sp_empuje;                 // 0..1 fraccion del empuje maximo
  float empuje_suave;              // tras el filtro de rampa

  // ---- salida del controlador ----
  float pd, qd, rd;                // deg/s, referencia del lazo interno
  float ux, uy, uz;                // N*m
  float ft;                        // N, empuje total pedido
  float i_p, i_q, i_r;             // terminos integrales (rad/s^2)

  // ---- motores ----
  float omega[4];                  // rad/s pedidos
  float empuje_motor[4];           // N
  int   pulso[4];                  // us enviados
  float mando[4];                  // 0..1 fraccion del recorrido
  bool  saturado;
  float escala_sat;                // 1.0 = sin recorte de torque

  // ---- [V_5] identificacion en linea ----
  float g_est[3];        // autoridad real / autoridad del modelo, por eje
  float g_cal[3];        // calidad de la estimacion, 0..1
  float kp_ef[3];        // ganancia del lazo interno realmente usada
  uint8_t ident_ok;      // bits 0..2: la estimacion de ese eje es fiable
  uint8_t signo_malo;    // bits 0..2: g negativa -> signo invertido
  uint8_t excitando;     // eje que se esta excitando + 1, o 0

  // ---- [V_3] control robusto ----
  float pert[3];       // perturbacion estimada en p, q, r (rad/s^2)
  float pert_z;        // perturbacion estimada en altura (m/s^2)
  float sigma_z;       // superficie deslizante del lazo de altura
  float err_modelo;    // % del peso que el modelo esta equivocando

  // ---- [V_2] altura ----
  uint8_t modo_altura;             // ALT_MANUAL / ALT_MANTENER
  float z_est, vz_est, bias_az;    // estado del Kalman (z abajo +)
  float alt_est;                   // -z_est, la altura que ve el usuario
  float alt_ref;                   // objetivo del usuario (m)
  float alt_ref_rampa;             // referencia interna tras la rampa (m)
  float vz_virtual;                // entrada virtual v_z (m/s^2)
  float i_z;                       // integral del lazo de altura
  bool  alt_valida;                // hay medida de distancia utilizable
  uint8_t fuente_alt;              // 0=ninguna, 1=ToF, 2=ultrasonido

  // ---- sensores lentos (nucleo 0) ----
  // [V_7] diagnostico de los sensores de distancia
  bool     tof_presente;
  uint16_t tof_mm_crudo;
  uint8_t  tof_estado;
  uint32_t tof_lecturas, tof_fallos;
  uint32_t us_ecos, us_ancho_us, us_fallos;
  float    us_m_crudo;

  float tof_m;      bool tof_ok;   // VL53L0X
  float us_m;       bool us_ok;    // HC-SR04
  float altura_m;                  // fusion simple de los dos
  float vbat;       bool vbat_ok;

  // ---- sistema ----
  uint8_t  modo;
  uint8_t  fallo;
  uint32_t t_armado_s;
  float    f_lazo;                 // Hz reales del lazo de control
  uint32_t lazo_us_max;            // peor caso del ciclo, us
  int32_t  rssi;
  uint8_t  clientes;
  uint32_t heap;
  uint32_t uptime_s;
  bool     enlace_ok;
};

extern volatile Estado G;

// --- Buffer circular para las graficas (50 Hz) --------------------------
#define LOG_N 150
// [V_6] La muestra lleva ahora TODO lo que hace falta para reconstruir un
// vuelo sin estar mirando la pagina: el empuje pedido, lo que daba cada sensor
// de distancia por separado, y en que modo estaba el dron. Con los dos sensores
// por separado se ve cual de los dos minitio, que es justo lo que no se puede
// deducir mirando solo la altura fusionada.
struct Muestra {
  int16_t roll, pitch, yaw;   // deg x10
  int16_t p, q, r;            // deg/s x10
  int16_t alt, altref;        // m x100
  int16_t vz;                 // m/s x100 (hacia arriba positiva)
  int16_t ft;                 // [V_6] empuje total pedido, N x100
  int16_t tof, us;            // [V_6] cada sensor por separado, cm (-1 = no vale)
  uint8_t thr;                // [V_6] empuje suavizado, 0..255
  uint8_t estado;             // [V_6] modo | modo_altura<<3 | alt_valida<<5 | sat<<6
  uint8_t m[4];               // mando de cada motor, 0..255
};
extern volatile Muestra logBuf[LOG_N];
extern volatile uint32_t logSeq;

// Contador de medidas de distancia: el nucleo 0 lo sube cuando hay una nueva,
// el lazo de control compara con el suyo para corregir el Kalman solo entonces.
extern volatile uint32_t altSeq;

// --- Ajustes vivos ------------------------------------------------------
extern Ajustes AJ;

#endif // ESTADO_H
