// =============================================================================
//  F450 - RAV   |   Control de estabilizacion ROLL / PITCH / YAW
//  ESP32-S3  +  MPU-6500  +  VL53L0X  +  HC-SR04  +  4x ESC 30 A / A2212 1000KV
//
//  VERSION V_9   (2026-11-26)
//  Reescritura completa siguiendo las notas del curso "Robots Aereos" (UTEC,
//  O. Ramos). Esta version resuelve SOLO la actitud; el control de altura
//  entra en la siguiente, y la estructura ya esta preparada para recibirlo
//  (ver `altura_m` en sensores.h y la nota al final de este archivo).
//
//  QUE HACE
//    - [V_4] Pagina rehecha: tres pestanas en vez de seis, ganancias con
//      deslizadores que se aplican al momento y confirman el valor que el dron
//      acepta de vuelta. Signos de la IMU corregidos tras medirlos en banco
//      (roll y pitch salian invertidos: la IMU esta girada 180 grados).
//    - [V_3] Control ROBUSTO opcional en los dos lazos (actitud y altura):
//      modos deslizantes de segundo orden (super-twisting) sobre la ley
//      nominal de las notas. Tolera que m, I, kf y el empuje real cambien
//      -bateria que se descarga, helices gastadas, efecto suelo- sin volver a
//      identificar el modelo. Se activa y desactiva en vuelo desde la pagina,
//      asi que se puede comparar contra la ley nominal en el mismo vuelo.
//      Ver robusto.h.
//    - [V_2] Estima altura y velocidad vertical con un Kalman de 3 estados
//      (z, z_punto, sesgo del acelerometro) que fusiona el ToF / ultrasonido
//      con la aceleracion vertical, y cierra el lazo de altura de las
//      ec. (35)-(36): u1 = (v_z - g)/r33,  f_t = -m u1.
//    - Estima roll/pitch/yaw con un filtro de Mahony sobre cuaternion.
//    - Controla la actitud en cascada: angulo -> velocidad angular -> torque,
//      con la linealizacion por realimentacion de la ec. (41) de las notas.
//    - Reparte (empuje, torques) entre los cuatro rotores con la inversa del
//      mezclador, ec. (46), proyectando al conjunto factible cuando satura.
//    - Punto de acceso WiFi "F450-RAV" + pagina de control y telemetria en
//      http://192.168.4.1 con todos los sensores, sintonia en caliente,
//      graficas y prueba individual de motores.
//
//  REPARTO DE NUCLEOS
//    nucleo 1 : tareaControl   250 Hz, prioridad alta. Nada mas vive aqui.
//    nucleo 0 : tareaSensores   20 Hz  (ToF, ultrasonido, bateria)
//    nucleo 0 : tareaWeb        servidor HTTP
//
//  MODOS DE EMPUJE
//    MANUAL    la barra de la pagina manda empuje total, en newtons.
//    MANTENER  el lazo de altura decide el empuje. Solo se entra EN VUELO,
//              con medida valida y por encima de 15 cm; al entrar se carga el
//              integrador para que no haya salto, y si se pierde la medida se
//              vuelve a manual con el mismo empuje.
//
//  SEGURIDAD (leer antes de poner helices)
//    - Arma solo con el acelerador en 0 y el dron casi nivelado.
//    - Se desarma solo si pasa de 60 grados de inclinacion o si la IMU calla.
//    - Failsafe de enlace: sin latido de la pagina durante 0.8 s baja el
//      acelerador solo y termina desarmando.
//    - La prueba individual de motores exige confirmar "helices fuera" y se
//      corta sola a los 1.5 s.
//
//  Placa: "ESP32S3 Dev Module", nucleo arduino-esp32 3.x
//  Libreria externa: Adafruit_VL53L0X
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

#include "config.h"
#include "estado.h"
#include "imu.h"
#include "actitud.h"
#include "control.h"
#include "mezclador.h"
#include "sensores.h"
#include "altura.h"       // [V_2]
#include "robusto.h"      // [V_3]
#include "identifica.h"   // [V_5]

// ---------------------------------------------------------------- globales
volatile Estado   G;
volatile Muestra  logBuf[LOG_N];
volatile uint32_t logSeq = 0;
volatile uint32_t altSeq = 0;      // [V_2] medidas de distancia nuevas
Ajustes           AJ;

IMU           imu;
Actitud       est;
Control       ctrl;
EstimadorZ    kz;      // [V_2] Kalman vertical
ControlAltura calt;    // [V_2] lazo de altura
IdentEje      ident[3];// [V_5] autoridad medida en p, q, r
Excitador     exc;     // [V_5] excitacion opcional
Sensores    sens;
Preferences nvs;
WebServer   servidor(80);

// --- ordenes que la web deja para el lazo de control ---
volatile bool  cmd_armar     = false;
volatile bool  cmd_desarmar  = false;
volatile bool  cmd_cal_giro  = false;
volatile bool  cmd_nivelar   = false;
volatile bool  cmd_yaw_cero  = false;
volatile bool  cmd_reset_i   = false;
volatile float sp_roll_web   = 0, sp_pitch_web = 0, sp_yaw_web = 0;
volatile float sp_empuje_web = 0;
// Mando de yaw: siempre llega como VELOCIDAD (deg/s). Si el modo de yaw es por
// angulo, el lazo lo integra sobre el rumbo de referencia; si no, pasa directo.
volatile float yaw_rate_cmd  = 0;
// [V_2] altura
volatile bool    cmd_nivel_vuelo = false;  // [V_5] aprender el nivel en el aire
volatile float   cmd_trim_r = 0, cmd_trim_p = 0;   // [V_8] ajuste fino de nivel
volatile bool    cmd_reset_ident = false;  // [V_5] olvidar lo identificado
volatile int8_t  cmd_excitar     = -1;     // [V_5] eje a excitar
volatile uint8_t cmd_modo_alt = 255;   // 255 = sin orden; 0 manual, 1 mantener
volatile float   alt_ref_web  = 0.0f;  // objetivo del usuario, en metros

// --- prueba individual de motores ---
volatile int      prueba_motor = -1;      // -1 = ninguno
volatile float    prueba_mando = 0;
volatile uint32_t prueba_hasta = 0;

// --- enlace ---
volatile uint32_t ultimoLatido = 0;
volatile bool     huboCliente  = false;

char mensaje[80] = "listo";
void msg(const char *m) { strncpy(mensaje, m, sizeof(mensaje) - 1); mensaje[sizeof(mensaje) - 1] = 0; }

#include "web.h"     // usa los globales de arriba

// ---------------------------------------------------------------- NVS
void guardarAjustes() {
  nvs.begin("f450", false);
  nvs.putBytes("aj", &AJ, sizeof(AJ));
  nvs.end();
}
void cargarAjustes() {
  ajustesPorDefecto(AJ);
  nvs.begin("f450", true);
  Ajustes tmp;
  size_t n = nvs.getBytesLength("aj");
  if (n == sizeof(Ajustes)) {
    nvs.getBytes("aj", &tmp, sizeof(tmp));
    if (tmp.magia == AJUSTES_MAGIA) AJ = tmp;
  }
  nvs.end();
}

// ---------------------------------------------------------------- motores
void escribirPulso(int i, int us) {
  if (i < 0 || i > 3) return;
  us = (int)limitar((float)us, (float)PULSO_MIN_US, (float)PULSO_MAX_US);
  uint32_t duty = ((uint32_t)us * ((1UL << PWM_RES) - 1)) / PWM_PERIODO_US;
  ledcWrite(PIN_MOTOR[i], duty);
}
void motoresParados() {
  for (int i = 0; i < 4; i++) { escribirPulso(i, PULSO_MIN_US); G.pulso[i] = PULSO_MIN_US; }
}

// [V_6] Salir de MANTENER ALTURA sin dejar al dron subiendo.
// Las versiones anteriores conservaban el ultimo empuje al salir, y si se salia
// por un fallo del sensor ese ultimo empuje era precisamente el de subida: el
// dron seguia trepando igual, solo que ya sin lazo. Aqui el empuje se deja en
// el de HOVER, que es la unica cifra que se conoce sin depender del sensor que
// acaba de fallar.
void salirDeAltura(float r33, const char *porque) {
  float r = (r33 < 0.5f) ? 0.5f : r33;
  float ft_hover = AJ.masa * GRAVEDAD / r;
  float tope = Mezclador::empujeMax(AJ);
  float frac = (tope > 0.01f) ? limitar(ft_hover / tope, 0.0f, EMPUJE_MAX_FRAC) : 0.0f;
  G.modo_altura  = ALT_MANUAL;
  sp_empuje_web  = frac;
  G.empuje_suave = frac;
  G.fallo = FALLO_ALTURA;
  msg(porque);
}

void desarmar(uint8_t causa) {
  G.modo  = MODO_DESARMADO;
  G.modo_altura = ALT_MANUAL;     // [V_2]
  G.fallo = causa;
  sp_empuje_web = 0;
  yaw_rate_cmd  = 0;
  G.sp_empuje = 0;
  G.empuje_suave = 0;
  ctrl.reset();
  motoresParados();
}

// ---------------------------------------------------------------- telemetria
void guardarMuestra() {
  uint32_t i = logSeq % LOG_N;
  logBuf[i].roll  = (int16_t)limitar(G.roll  * 10.0f, -1800, 1800);
  logBuf[i].pitch = (int16_t)limitar(G.pitch * 10.0f, -1800, 1800);
  logBuf[i].yaw   = (int16_t)limitar(G.yaw   * 10.0f, -1800, 1800);
  logBuf[i].p     = (int16_t)limitar(G.p * 10.0f, -5000, 5000);
  logBuf[i].q     = (int16_t)limitar(G.q * 10.0f, -5000, 5000);
  logBuf[i].r     = (int16_t)limitar(G.r * 10.0f, -5000, 5000);
  logBuf[i].alt    = (int16_t)limitar(G.alt_est      * 100.0f, -3000, 3000);
  logBuf[i].altref = (int16_t)limitar(G.alt_ref_rampa * 100.0f, -3000, 3000);
  logBuf[i].vz     = (int16_t)limitar(-G.vz_est       * 100.0f, -3000, 3000);
  // [V_6] lo que hace falta para reconstruir el vuelo despues
  logBuf[i].ft  = (int16_t)limitar(G.ft * 100.0f, -30000, 30000);
  logBuf[i].tof = G.tof_ok ? (int16_t)limitar(G.tof_m * 100.0f, 0, 30000) : (int16_t)-1;
  logBuf[i].us  = G.us_ok  ? (int16_t)limitar(G.us_m  * 100.0f, 0, 30000) : (int16_t)-1;
  logBuf[i].thr = (uint8_t)limitar(G.empuje_suave * 255.0f, 0, 255);
  logBuf[i].estado = (uint8_t)((G.modo & 0x07)
                     | ((G.modo_altura & 0x03) << 3)
                     | (G.alt_valida ? 0x20 : 0)
                     | (G.saturado   ? 0x40 : 0));
  for (int k = 0; k < 4; k++)
    logBuf[i].m[k] = (uint8_t)limitar(G.mando[k] * 255.0f, 0, 255);
  logSeq = logSeq + 1;          // despues del dato: la web nunca lee a medias
}

// =============================================================================
//  TAREA DE CONTROL  -  nucleo 1, 250 Hz
// =============================================================================
void tareaControl(void *) {
  TickType_t tUltimo = xTaskGetTickCount();
  uint32_t t_prev = micros();
  uint32_t fallos_imu = 0;
  uint32_t t_log = 0;
  uint32_t alt_visto = 0;          // [V_2] ultima medida de distancia usada
  uint32_t t_alt_ok = 0;           // [V_2] cuando llego la ultima valida
  uint32_t t_atasco = 0;           // [V_6] detector de sensor que no sigue
  float    alt_atasco = 0;
  float    f_filt = F_CONTROL_HZ;

  for (;;) {
    vTaskDelayUntil(&tUltimo, pdMS_TO_TICKS(MS_CONTROL));
    uint32_t t0 = micros();
    float dt = (t0 - t_prev) * 1e-6f;
    t_prev = t0;
    if (dt < 0.0005f) dt = 0.0005f;
    if (dt > 0.050f)  dt = 0.050f;
    f_filt = 0.98f * f_filt + 0.02f * (1.0f / dt);

    // ---------------- 1. IMU ----------------
    bool ok = imu.leer();
    if (ok) { fallos_imu = 0; }
    else if (++fallos_imu > 25) {                 // 100 ms sin respuesta
      G.imu_ok = false;
      if (G.modo != MODO_DESARMADO) { desarmar(FALLO_IMU); msg("IMU sin respuesta: desarmado"); }
    }
    // Un fallo suelto de I2C no es motivo para marcar la IMU como caida: se
    // avisa a partir de 5 seguidos, y se desarma a partir de 25 (100 ms).
    G.imu_ok = (fallos_imu < 5);

    // ---------------- 2. actitud ----------------
    est.actualizar(imu.gx, imu.gy, imu.gz, imu.ax, imu.ay, imu.az,
                   dt, AJ.mahony_kp, AJ.mahony_ki, AJ.trim_roll, AJ.trim_pitch);

    G.roll = est.roll;  G.pitch = est.pitch;  G.yaw = est.yaw;
    G.p = imu.gx;       G.q = imu.gy;         G.r = imu.gz;
    G.q0 = est.q0; G.q1 = est.q1; G.q2 = est.q2; G.q3 = est.q3;
    G.ax = imu.ax; G.ay = imu.ay; G.az = imu.az;
    G.gx = imu.gx; G.gy = imu.gy; G.gz = imu.gz;
    G.bias_gx = imu.bias[0]; G.bias_gy = imu.bias[1]; G.bias_gz = imu.bias[2];
    G.temp_imu = imu.temp;

    // ---------------- 2b. [V_2] estimacion de altura ----------------
    // Corre siempre, armado o no: asi al armar el filtro ya esta asentado y el
    // sesgo del acelerometro ya esta estimado.
    float r31, r32, r33;
    est.filaZ(r31, r32, r33);
    // Acelerometro (en g) proyectado al sistema inercial, mas la gravedad.
    // En reposo y nivelado: r33 = 1, az = -1 g  ->  -9.81 + 9.81 = 0.
    float az_i = (r31 * imu.ax + r32 * imu.ay + r33 * imu.az) * GRAVEDAD + GRAVEDAD;
    // [V_7] NO se integra el acelerometro hasta que haya habido una medida que
    // ancle el filtro. Antes se predecia siempre, tambien desarmado y sin
    // ningun sensor contestando: el resultado era un estimador en caida libre
    // que llegaba a marcar decenas de kilometros. Un filtro sin correccion no
    // es una estimacion, es una integral de ruido.
    if (kz.inicializado) kz.predecir(az_i, dt, AJ.kal_q);

    uint32_t s_alt = altSeq;
    if (s_alt != alt_visto) {
      alt_visto = s_alt;
      uint8_t fu = G.fuente_alt;
      if (fu != 0) {
        kz.corregir(-G.altura_m, (fu == 1) ? KAL_R_TOF : KAL_R_US);
        t_alt_ok = millis();
      }
    }
    // [V_7] Sin medidas durante un rato, el filtro se apaga del todo en vez de
    // seguir derivando. Vuelve a arrancar solo en cuanto llegue una medida.
    if (kz.inicializado && (millis() - t_alt_ok > ALT_RESET_MS)) {
      kz.inicializado = false;
      kz.x[0] = 0; kz.x[1] = 0; kz.x[2] = 0;
    }
    // Y por si acaso: cualquier valor absurdo tambien reinicia el filtro.
    if (kz.inicializado && fabsf(kz.z()) > ALT_LIMITE_ABS) {
      kz.inicializado = false;
      kz.x[0] = 0; kz.x[1] = 0; kz.x[2] = 0;
      msg("estimador de altura reiniciado: valor absurdo");
    }
    G.alt_valida = kz.inicializado && (millis() - t_alt_ok < ALT_TIMEOUT_MS);
    G.z_est   = kz.z();
    G.vz_est  = kz.vz();
    G.bias_az = kz.sesgo();
    G.alt_est = kz.inicializado ? -kz.z() : 0.0f;

    // ---------------- 3. ordenes de la pagina ----------------
    if (cmd_cal_giro) {
      cmd_cal_giro = false;
      if (G.modo == MODO_DESARMADO) {
        motoresParados();
        bool bien = imu.calibrarGiro(800);
        est.alinear(imu.ax, imu.ay, imu.az);
        msg(bien ? "giroscopio calibrado" : "ERROR: el dron se movio, repetir quieto");
        t_prev  = micros();                 // la calibracion tardo ~2 s
        tUltimo = xTaskGetTickCount();      // no intentar recuperar esos ciclos
      } else msg("no se puede calibrar armado");
    }
    if (cmd_nivelar) {
      cmd_nivelar = false;
      if (G.modo == MODO_DESARMADO) {
        AJ.trim_roll  += est.roll;     // lleva la lectura actual a cero
        AJ.trim_pitch += est.pitch;
        AJ.trim_roll  = limitar(AJ.trim_roll,  -15.0f, 15.0f);
        AJ.trim_pitch = limitar(AJ.trim_pitch, -15.0f, 15.0f);
        msg("nivel ajustado (guardar para conservarlo)");
      } else msg("no se puede nivelar armado");
    }
    if (cmd_yaw_cero)  { cmd_yaw_cero = false; est.ponerYawCero(); sp_yaw_web = 0; }

    // [V_5] Aprender el nivel EN VUELO. El giroscopio se calibra con el dron en
    // el suelo, pero el suelo no esta a nivel, el centro de masa no esta
    // centrado y los cuatro motores no dan lo mismo: la actitud a la que el
    // dron se queda quieto en el aire NO es la que marcaba 0 en el suelo. Aqui
    // se vuela sujetando la referencia que haga falta para no desplazarse, se
    // pulsa, y esa inclinacion pasa a ser el nuevo cero.
    //   roll = roll_sensor - trim   ->   para que la actitud actual lea 0:
    //   trim_nuevo = trim_viejo + referencia que estabas sujetando
    // y la referencia vuelve al centro, asi que el dron no se mueve al pulsar.
    if (cmd_nivel_vuelo) {
      cmd_nivel_vuelo = false;
      if (G.modo == MODO_VUELO) {
        AJ.trim_roll  = limitar(AJ.trim_roll  + sp_roll_web,  -15.0f, 15.0f);
        AJ.trim_pitch = limitar(AJ.trim_pitch + sp_pitch_web, -15.0f, 15.0f);
        sp_roll_web = 0;  sp_pitch_web = 0;
        msg("nivel aprendido en vuelo");
      } else msg("NO: esto se hace volando, sujetando la correccion");
    }
    // [V_8] Ajuste fino del nivel, un toque cada vez, SIN tocar la referencia.
    // Por que no se puede hacer automatico: el dron no tiene ningun sensor que
    // mida desplazamiento. Sabe su actitud, pero no sabe si esa actitud lo esta
    // dejando quieto o lo esta llevando hacia la pared. Solo el que mira sabe
    // eso, asi que el trim lo pone el piloto. Y no se puede "aprender" de la
    // barra sujetada: si se sube el trim mientras la barra sigue pedida, la
    // actitud fisica resultante es la suma de las dos y el dron se va girando.
    if (cmd_trim_r != 0.0f || cmd_trim_p != 0.0f) {
      AJ.trim_roll  = limitar(AJ.trim_roll  + cmd_trim_r, -15.0f, 15.0f);
      AJ.trim_pitch = limitar(AJ.trim_pitch + cmd_trim_p, -15.0f, 15.0f);
      cmd_trim_r = 0;  cmd_trim_p = 0;
    }
    if (cmd_reset_ident) {
      cmd_reset_ident = false;
      for (int i = 0; i < 3; i++) ident[i].reset();
      msg("identificacion reiniciada");
    }
    if (cmd_excitar >= 0) {
      int e = cmd_excitar;  cmd_excitar = -1;
      if (G.modo == MODO_VUELO) {
        exc.eje = e;  exc.amp = IDENT_EXC_AMP;
        exc.hasta = millis() + IDENT_EXC_MAX_MS;
        msg("excitando para identificar");
      } else msg("NO: la excitacion solo se hace en vuelo");
    }
    if (cmd_reset_i)   { cmd_reset_i = false;  ctrl.ip = ctrl.iq = ctrl.ir = 0; }

    // [V_2] entrar o salir de mantener altura
    if (cmd_modo_alt != 255) {
      uint8_t pedido = cmd_modo_alt;
      cmd_modo_alt = 255;
      if (pedido == ALT_MANUAL) {
        G.modo_altura = ALT_MANUAL;
        msg("acelerador manual");
      } else if (G.modo != MODO_VUELO) {
        msg("NO: primero despega a mano");
      } else if (!G.alt_valida) {
        msg("NO: no hay medida de altura");
      } else if (G.alt_est < ALT_MIN_HOLD) {
        msg("NO: demasiado bajo para mantener altura");
      } else if (G.alt_est > AJ.alt_max) {
        msg("NO: por encima del alcance del sensor");
      } else {
        // Transferencia sin salto: se carga el integrador con el valor que
        // hace que el empuje calculado salga EXACTAMENTE igual al que ya se
        // esta dando. Sin esto el dron pega un tiron al activar el modo.
        float ft_ahora = G.empuje_suave * Mezclador::empujeMax(AJ);
        float r33s = (r33 < 0.5f) ? 0.5f : r33;
        float vz_equiv = GRAVEDAD - ft_ahora * r33s / AJ.masa;
        calt.reset(kz.z());
        // El mismo valor sirve para los dos modos: en nominal es el integrador
        // y en robusto es el estado w del super-twisting. En robusto queda un
        // residuo del termino k1|s|^(1/2), despreciable si se activa en hover.
        float semilla = limitar(vz_equiv + AJ.kd_z * kz.vz(), -IZ_MAX, IZ_MAX);
        calt.iz = semilla;
        calt.stz.sembrar(semilla, IZ_MAX);                   // [V_3]
        alt_ref_web   = G.alt_est;
        t_atasco      = 0;
        G.modo_altura = ALT_MANTENER;
        msg("manteniendo altura");
      }
    }

    if (cmd_desarmar) { cmd_desarmar = false; desarmar(FALLO_USUARIO); msg("desarmado"); }

    if (cmd_armar) {
      cmd_armar = false;
      if (G.modo != MODO_DESARMADO)                 msg("ya estaba armado");
      else if (!G.imu_ok)                           msg("NO: la IMU no responde");
      else if (sp_empuje_web > 0.001f)              msg("NO: baja el acelerador a 0");
      else if (fabsf(est.roll) > 15.0f ||
               fabsf(est.pitch) > 15.0f)            msg("NO: nivela el dron (menos de 15 deg)");
      else {
        ctrl.reset();
        est.ponerYawCero();
        sp_roll_web = 0;  sp_pitch_web = 0;  sp_yaw_web = 0;
        yaw_rate_cmd = 0;
        G.empuje_suave = 0;
        G.fallo = FALLO_NINGUNO;
        G.modo  = MODO_ARMADO;
        G.t_armado_s = millis() / 1000;
        G.lazo_us_max = 0;
        G.modo_altura = ALT_MANUAL;      // [V_2] siempre se despega a mano
        // [V_7] Solo se arranca el filtro si hay una medida de verdad. Antes se
        // arrancaba siempre, aunque los dos sensores estuvieran mudos, y a
        // partir de ahi el estimador se iba solo.
        if (G.fuente_alt != 0) kz.reiniciar(-G.altura_m);
        else { kz.inicializado = false; kz.x[0] = 0; kz.x[1] = 0; kz.x[2] = 0; }
        calt.reset(kz.z());
        alt_ref_web = G.altura_m;
        msg("ARMADO: motores al ralenti");
      }
    }

    // ---------------- 4. prueba individual de motores ----------------
    if (prueba_motor >= 0) {
      if (G.modo == MODO_DESARMADO || G.modo == MODO_PRUEBA) {
        G.modo = MODO_PRUEBA;
        if ((int32_t)(millis() - prueba_hasta) > 0) {
          prueba_motor = -1;
          G.modo = MODO_DESARMADO;
          motoresParados();
        } else {
          for (int i = 0; i < 4; i++) {
            int us = (i == prueba_motor)
                     ? Mezclador::pulsoDeMando(AJ, i, prueba_mando)
                     : PULSO_MIN_US;
            escribirPulso(i, us);
            G.pulso[i] = us;
            G.mando[i] = (i == prueba_motor) ? prueba_mando : 0.0f;
          }
        }
      } else prueba_motor = -1;
    }

    // ---------------- 5. failsafe de enlace ----------------
    uint32_t ahora = millis();
    G.enlace_ok = (!huboCliente) || (ahora - ultimoLatido < LATIDO_TIMEOUT_MS);
    if (!G.enlace_ok && (G.modo == MODO_ARMADO || G.modo == MODO_VUELO)) {
      G.modo = MODO_BAJADA;
      msg("SIN ENLACE: bajada automatica");
    }
    if (G.modo == MODO_BAJADA) {
      float e = sp_empuje_web - BAJADA_POR_S * dt;
      sp_empuje_web = (e > 0.0f) ? e : 0.0f;
      if (sp_empuje_web <= 0.0f) { desarmar(FALLO_ENLACE); msg("bajada terminada, desarmado"); }
      else if (G.enlace_ok) { G.modo = MODO_VUELO; msg("enlace recuperado"); }
    }

    // ---------------- 6. corte por inclinacion ----------------
    if ((G.modo == MODO_ARMADO || G.modo == MODO_VUELO || G.modo == MODO_BAJADA) &&
        (fabsf(est.roll) > ANG_CORTE || fabsf(est.pitch) > ANG_CORTE)) {
      desarmar(FALLO_INCLINACION);
      msg("CORTE: paso de 60 grados de inclinacion");
    }

    // ---------------- 7. referencias ----------------
    G.sp_roll  = limitar(sp_roll_web,  -AJ.ang_max, AJ.ang_max);
    G.sp_pitch = limitar(sp_pitch_web, -AJ.ang_max, AJ.ang_max);
    if (AJ.yaw_por_angulo) {
      sp_yaw_web = envolverDeg(sp_yaw_web + yaw_rate_cmd * dt);
      G.sp_yaw   = sp_yaw_web;
    } else {
      G.sp_yaw   = limitar(yaw_rate_cmd, -RATE_MAX_YAW, RATE_MAX_YAW);
    }
    G.sp_empuje = limitar(sp_empuje_web, 0.0f, EMPUJE_MAX_FRAC);

    // rampa de empuje: el salto de un dedo en la barra no llega entero a los ESC
    float k = 1.0f - expf(-dt / 0.15f);          // constante de tiempo 150 ms
    G.empuje_suave += (G.sp_empuje - G.empuje_suave) * k;
    if (G.modo == MODO_DESARMADO || G.modo == MODO_PRUEBA) G.empuje_suave = 0;

    if (G.modo == MODO_ARMADO && G.sp_empuje > 0.02f) G.modo = MODO_VUELO;
    if (G.modo == MODO_VUELO  && G.sp_empuje <= 0.0f && G.empuje_suave < 0.01f) G.modo = MODO_ARMADO;

    // ---------------- 8. control + mezclador ----------------
    if (G.modo == MODO_ARMADO || G.modo == MODO_VUELO || G.modo == MODO_BAJADA) {
      // Con empuje casi nulo no hay autoridad para corregir nada (el mezclador
      // no puede bajar de la velocidad minima): se congela el integrador para
      // que no se cargue en el suelo.
      bool integrar = (G.empuje_suave > 0.15f) && (!G.saturado);
      if (G.empuje_suave < 0.05f) { ctrl.ip = ctrl.iq = ctrl.ir = 0; }

      // [V_5] Ganancia del lazo interno. En modo adaptativo sale de la
      // autoridad medida: kp = 1/(tau*g). Se mueve despacio (1 s) para que un
      // salto en la estimacion no sea un salto en el mando.
      static float kp_suave[3] = {KP_RATE_P_DEF, KP_RATE_Q_DEF, KP_RATE_R_DEF};
      float kp_ef[3];
      for (int i = 0; i < 3; i++) {
        float obj = AJ.adaptar ? ident[i].kpRecomendada(AJ.tau_rate, LIM_KP_RATE)
                               : AJ.kp_rate[i];
        kp_suave[i] += (obj - kp_suave[i]) * (dt / (dt + 1.0f));
        kp_ef[i] = kp_suave[i];
        G.kp_ef[i] = kp_ef[i];
      }

      // [V_5] excitacion opcional para garantizar que hay que aprender
      uint32_t t_exc = millis();
      for (int i = 0; i < 3; i++) ctrl.exc_pqr[i] = exc.valor(i, t_exc);
      G.excitando = exc.activo(t_exc) ? (uint8_t)(exc.eje + 1) : 0;

      ctrl.actualizar(AJ, est.roll, est.pitch, est.yaw,
                      imu.gx, imu.gy, imu.gz,
                      G.sp_roll, G.sp_pitch, G.sp_yaw,
                      dt, integrar, AJ.robusto_att != 0, kp_ef);   // [V_3][V_5]

      // [V_5] Identificacion: solo mientras se vuela de verdad y el mezclador
      // no esta recortando. Con el mezclador saturado, el torque que llega no
      // es el pedido por una razon CONOCIDA, y meter esas muestras sesgaria la
      // estimacion hacia abajo.
      bool ident_activa = (G.modo == MODO_VUELO) && (G.empuje_suave > 0.15f)
                          && (G.escala_sat > 0.98f);
      ident[0].actualizar(imu.gx * DEG2RAD, ctrl.vp_ult, dt, AJ.ident_lambda, ident_activa);
      ident[1].actualizar(imu.gy * DEG2RAD, ctrl.vq_ult, dt, AJ.ident_lambda, ident_activa);
      ident[2].actualizar(imu.gz * DEG2RAD, ctrl.vr_ult, dt, AJ.ident_lambda, ident_activa);
      G.ident_ok = 0;  G.signo_malo = 0;
      for (int i = 0; i < 3; i++) {
        G.g_est[i] = ident[i].g;
        G.g_cal[i] = ident[i].calidad;
        if (ident[i].fiable)                  G.ident_ok   |= (1 << i);
        if (ident[i].fiable && ident[i].g < 0) G.signo_malo |= (1 << i);
      }

      // ---- [V_2] empuje total: a mano o con el lazo de altura ----
      const float ft_tope = Mezclador::empujeMax(AJ);
      float ft;

      if (G.modo_altura == ALT_MANTENER) {
        float r33s = (r33 < 0.5f) ? 0.5f : r33;
        float ft_hover = AJ.masa * GRAVEDAD / r33s;

        if (G.modo != MODO_VUELO) {
          G.modo_altura = ALT_MANUAL;
          ft = G.empuje_suave * ft_tope;
        } else if (!G.alt_valida) {
          salirDeAltura(r33, "sin medida de altura: empuje a hover");
          ft = G.empuje_suave * ft_tope;
        } else if (G.alt_est > AJ.alt_max + ALT_TECHO_MARGEN) {
          // [V_6] Techo duro. Si de verdad esta ahi arriba, hay que bajar; y si
          // no lo esta, la medida es basura y tampoco hay que seguir subiendo.
          salirDeAltura(r33, "techo alcanzado: empuje a hover");
          ft = G.empuje_suave * ft_tope;
        } else {
          float obj = -limitar(alt_ref_web, ALT_MIN_HOLD, AJ.alt_max);   // z = -altura
          calt.rampa(obj, dt, AJ.vz_max);

          // [V_6] Topes RELATIVOS AL HOVER, no al maximo de los motores. Aunque
          // el sensor mienta, el lazo nunca puede pedir mas de un 35 % por
          // encima de lo que hace falta para sostenerse.
          float ft_max_alt = fminf(ft_tope * 0.95f, ALT_EMPUJE_MAX_K * ft_hover);
          float ft_min_alt = ALT_EMPUJE_MIN_K * ft_hover;

          ft = calt.calcular(AJ, kz.z(), kz.vz(), r33, dt, ft_min_alt, ft_max_alt,
                             AJ.robusto_alt != 0);
          G.empuje_suave = ft / ft_tope;
          sp_empuje_web  = G.empuje_suave;

          // [V_6] Detector de sensor atascado: lleva un rato pidiendo subir y
          // la altura MEDIDA no sube. O el ultrasonido esta cegado por las
          // helices, o el ToF devuelve basura fuera de rango. Es el fallo que
          // manda el dron al techo, porque el lazo "ve" que no sube y pide mas.
          if (ft > ALT_ATASCO_K * ft_hover) {
            if (t_atasco == 0) { t_atasco = ahora; alt_atasco = G.alt_est; }
            else if (ahora - t_atasco > ALT_ATASCO_MS) {
              if (G.alt_est - alt_atasco < ALT_ATASCO_DH)
                salirDeAltura(r33, "la altura medida no sigue al empuje: sensor dudoso");
              t_atasco = 0;
            }
          } else t_atasco = 0;
        }
      } else {
        ft = G.empuje_suave * ft_tope;
        calt.reset(kz.z());          // referencia pegada al estado actual
      }
      // [V_8] CORREGIDO. Antes los dos campos valian lo mismo: la rampa. La
      // pagina sincronizaba su barra con ese valor, asi que mientras el dron
      // subia la barra se movia sola y, si el usuario la tocaba despues,
      // mandaba como objetivo nuevo la altura por la que iba pasando. El
      // objetivo "fijo" acababa persiguiendo al dron.
      //   alt_ref       = lo que pidio el usuario, y solo cambia si el lo cambia
      //   alt_ref_rampa = por donde va la rampa interna, solo para la grafica
      G.alt_ref        = alt_ref_web;
      G.alt_ref_rampa  = -calt.z_ref;
      G.vz_virtual     = calt.vz_mando;
      G.i_z            = calt.iz;
      // [V_3] Lo que el modelo esta equivocando, estimado en linea por el
      // termino robusto. En altura se traduce a % del peso: si marca +8 %, el
      // dron necesita un 8 % mas de empuje del que predice m*g, o sea que la
      // masa real o kf estan desviados esa cantidad.
      G.pert[0]   = ctrl.stp.perturbacion();
      G.pert[1]   = ctrl.stq.perturbacion();
      G.pert[2]   = ctrl.str.perturbacion();
      G.pert_z    = calt.stz.perturbacion();
      G.sigma_z   = calt.sigma;
      G.err_modelo = 100.0f * G.pert_z / GRAVEDAD;

      SalidaMezcla sm = Mezclador::calcular(AJ, ft, ctrl.ux, ctrl.uy, ctrl.uz);

      for (int i = 0; i < 4; i++) {
        escribirPulso(i, sm.pulso[i]);
        G.pulso[i]  = sm.pulso[i];
        G.mando[i]  = sm.mando[i];
        G.omega[i]  = sm.w[i];
        G.empuje_motor[i] = Mezclador::empujeDeOmega(AJ, sm.w[i]);
      }
      G.ft = ft;  G.ux = ctrl.ux;  G.uy = ctrl.uy;  G.uz = ctrl.uz;
      G.pd = ctrl.pd; G.qd = ctrl.qd; G.rd = ctrl.rd;
      G.i_p = ctrl.ip; G.i_q = ctrl.iq; G.i_r = ctrl.ir;
      G.saturado = sm.saturado;
      G.escala_sat = sm.escala;
      G.escala_yaw = sm.escala_yaw;   // [V_9]

    } else if (G.modo == MODO_DESARMADO) {
      motoresParados();
      for (int i = 0; i < 4; i++) { G.mando[i] = 0; G.omega[i] = 0; G.empuje_motor[i] = 0; }
      G.ft = 0;  G.ux = 0;  G.uy = 0;  G.uz = 0;
      G.pd = 0;  G.qd = 0;  G.rd = 0;
      G.saturado = false;  G.escala_sat = 1.0f;  G.escala_yaw = 1.0f;
      calt.reset(kz.z());                 // [V_2]
      G.alt_ref = G.alt_est;  G.alt_ref_rampa = G.alt_est;
      G.vz_virtual = 0;  G.i_z = 0;
    }

    // ---------------- 9. registro para las graficas ----------------
    if (ahora - t_log >= (1000 / F_LOG_HZ)) { t_log = ahora; guardarMuestra(); }

    G.f_lazo = f_filt;
    uint32_t dur = micros() - t0;
    if (dur > G.lazo_us_max) G.lazo_us_max = dur;
    G.uptime_s = millis() / 1000;
  }
}

// =============================================================================
//  TAREA DE SENSORES  -  nucleo 0, 20 Hz
// =============================================================================
void tareaSensores(void *) {
  sens.iniciar();
  TickType_t t = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&t, pdMS_TO_TICKS(1000 / F_SENSORES_HZ));
    sens.actualizar(G.roll, G.pitch);
    G.tof_m = sens.tof_m;   G.tof_ok = sens.tof_ok;
    G.us_m  = sens.us_m;    G.us_ok  = sens.us_ok;
    // [V_7] diagnostico por sensor
    G.tof_presente = sens.tof_presente;
    G.tof_mm_crudo = sens.tof_mm_crudo;
    G.tof_estado   = sens.tof_estado;
    G.tof_lecturas = sens.tof_lecturas;
    G.tof_fallos   = sens.tof_fallos;
    G.us_ecos      = sens.us_ecos;
    G.us_ancho_us  = sens.us_ancho_us;
    G.us_fallos    = sens.us_fallos;
    G.us_m_crudo   = sens.us_m_crudo;
    G.vbat  = sens.vbat;    G.vbat_ok = sens.vbat_ok;
    G.altura_m   = sens.altura_m;
    G.fuente_alt = sens.fuente;                        // [V_2]
    if (sens.hay_nueva) altSeq = altSeq + 1;           // [V_2] dato y luego contador
    G.heap = ESP.getFreeHeap();
    G.clientes = WiFi.softAPgetStationNum();
  }
}

// =============================================================================
//  TAREA WEB  -  nucleo 0
// =============================================================================
void tareaWeb(void *) {
  for (;;) {
    servidor.handleClient();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
  Serial0.begin(115200);
  delay(800);
  Serial0.println("\n=== F450-RAV  V_1  (control de actitud) ===");

  // --- estado inicial limpio ---
  memset((void *)&G, 0, sizeof(G));
  G.modo = MODO_DESARMADO;
  G.escala_sat = 1.0f;
  G.escala_yaw = 1.0f;
  G.imu_ok = false;

  // --- PWM y armado de los ESC: solo pulso minimo ---
  // Nunca se manda el pulso maximo al arrancar: algunos ESC lo entienden como
  // orden de girar al maximo en lugar de como entrada a calibracion.
  for (int i = 0; i < 4; i++) {
    ledcAttach(PIN_MOTOR[i], PWM_FREQ, PWM_RES);
    escribirPulso(i, PULSO_MIN_US);
  }
  Serial0.println("Armando ESCs (pulso minimo, 3 s)...");
  delay(3000);

  cargarAjustes();

  // --- IMU ---
  if (!imu.iniciar()) {
    Serial0.println("!! IMU NO DETECTADA: revisa SDA/SCL y alimentacion");
  } else {
    Serial0.printf("IMU WHO_AM_I = 0x%02X\n", imu.whoami);
    Serial0.println("Calibrando giroscopio (NO MOVER, 2 s)...");
    bool ok = imu.calibrarGiro(800);
    Serial0.printf("  sesgo = %.3f  %.3f  %.3f deg/s  %s\n",
                   imu.bias[0], imu.bias[1], imu.bias[2], ok ? "OK" : "(se movio!)");
    imu.leer();
    est.alinear(imu.ax, imu.ay, imu.az);
    Serial0.printf("  actitud inicial: roll %.1f  pitch %.1f\n", est.roll_s, est.pitch_s);
  }

  // --- modelo: resumen util antes de volar ---
  Serial0.printf("Empuje max %.2f N | hover %.2f N (%.0f %% de la barra) | T/W %.2f\n",
                 Mezclador::empujeMax(AJ), Mezclador::empujeHover(AJ),
                 100.0f * Mezclador::empujeHover(AJ) / Mezclador::empujeMax(AJ),
                 Mezclador::empujePeso(AJ));

  // --- WiFi + web ---
  iniciarWeb();

  // --- tareas ---
  xTaskCreatePinnedToCore(tareaControl,  "control",  8192, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(tareaSensores, "sensores", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(tareaWeb,      "web",      8192, NULL, 1, NULL, 0);

  Serial0.println("Listo. Abrir http://192.168.4.1 en la red F450-RAV.");
  Serial0.println("Serial: a=armar  s=parar  +/-=acelerador  h=mantener altura");
  Serial0.println("        r=robusto  d=adaptativo  l=aprender nivel en vuelo");
  Serial0.println("        c=calibrar giro  n=nivelar  g=guardar  i=info");
}

// =============================================================================
//  LOOP  -  solo consola. El control NO vive aqui.
// =============================================================================
void loop() {
  if (Serial0.available()) {
    char c = Serial0.read();
    switch (c) {
      case 'a': cmd_armar = true; break;
      case 's': cmd_desarmar = true; break;
      case '+': sp_empuje_web = limitar(sp_empuje_web + 0.05f, 0, EMPUJE_MAX_FRAC); break;
      case '-': sp_empuje_web = limitar(sp_empuje_web - 0.05f, 0, EMPUJE_MAX_FRAC); break;
      case 'd': AJ.adaptar = !AJ.adaptar;
                Serial0.printf("Adaptativo: %s", AJ.adaptar ? "SI" : "NO");
                break;
      case 'l': cmd_nivel_vuelo = true; break;
      case 'r': AJ.robusto_att = !AJ.robusto_att; AJ.robusto_alt = AJ.robusto_att;
                Serial0.printf("Modo robusto: %s", AJ.robusto_att ? "SI" : "NO");
                break;
                
      case 'h': cmd_modo_alt = (G.modo_altura == ALT_MANTENER) ? ALT_MANUAL : ALT_MANTENER; break;
      case 'c': cmd_cal_giro = true; break;
      case 'n': cmd_nivelar = true; break;
      case 'g': guardarAjustes(); msg("ajustes guardados"); break;
      case 'i':
        Serial0.printf("modo=%d roll=%.1f pitch=%.1f yaw=%.1f thr=%.2f lazo=%.0fHz heap=%u\n",
                       G.modo, G.roll, G.pitch, G.yaw, G.sp_empuje, G.f_lazo, (unsigned)G.heap);
        break;
    }
  }

  static uint32_t t = 0;
  if (millis() - t > 500) {
    t = millis();
    if (G.modo != MODO_DESARMADO)
      Serial0.printf("[%s] r%+6.1f p%+6.1f y%+6.1f | thr %3.0f%% | m %4d %4d %4d %4d | sat %.2f\n",
                     G.modo == MODO_VUELO ? "VUELO" : (G.modo == MODO_BAJADA ? "BAJADA" : "ARMADO"),
                     G.roll, G.pitch, G.yaw, G.empuje_suave * 100.0f,
                     G.pulso[0], G.pulso[1], G.pulso[2], G.pulso[3], G.escala_sat);
  }
  vTaskDelay(pdMS_TO_TICKS(20));
}

// -----------------------------------------------------------------------------
//  SIGUIENTE VERSION (V_3): posicion lateral (x, y)
//  Es el ultimo lazo que falta de la Seccion 5 de las notas, y el unico que
//  este dron todavia no puede cerrar: no hay sensor que mida x e y. Haria
//  falta flujo optico, vision o GPS. Cuando lo haya, la estructura ya encaja:
//    - ec. (38): v_x, v_y a partir del error de posicion
//    - ec. (37): r13_ref = v_x/u1 , r23_ref = v_y/u1
//    - ec. (39)-(40): de r13/r23 a p_d, q_d, que es justo lo que el lazo de
//      actitud de `control.h` ya acepta como entrada.
//  Es decir: el lazo lateral se enchufa ENCIMA del que ya existe, sustituyendo
//  las referencias de roll y pitch que hoy vienen de la pagina.
// -----------------------------------------------------------------------------
