// =============================================================================
//  web.h  -  Punto de acceso WiFi, servidor HTTP y API de la pagina  (V_9)
//
//  Rutas
//    GET /            la pagina (pagina.h, en PROGMEM)
//    GET /t?s=<seq>   telemetria completa + las muestras nuevas para la grafica
//    GET /cmd?c=...   ordenes de vuelo y de calibracion
//    GET /pid?...     ganancias y limites del controlador
//    GET /modelo?...  parametros fisicos y recorrido de cada ESC
//    GET /motor?...   prueba individual de un motor (helices fuera)
//
//  Toda peticion cuenta como latido del enlace. Si el movil se bloquea o se
//  aleja, a los LATIDO_TIMEOUT_MS el lazo de control empieza la bajada.
// =============================================================================
#ifndef WEB_H
#define WEB_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "config.h"
#include "estado.h"
#include "mezclador.h"
#include "pagina.h"

extern WebServer servidor;
extern Ajustes   AJ;
extern volatile Estado G;
extern volatile Muestra  logBuf[LOG_N];
extern volatile uint32_t logSeq;
extern char mensaje[80];

extern volatile bool  cmd_armar, cmd_desarmar, cmd_cal_giro, cmd_nivelar,
                      cmd_yaw_cero, cmd_reset_i;
extern volatile float sp_roll_web, sp_pitch_web, sp_yaw_web, sp_empuje_web;
extern volatile float yaw_rate_cmd;
extern volatile uint8_t cmd_modo_alt;
extern volatile bool    cmd_nivel_vuelo;
extern volatile float   cmd_trim_r, cmd_trim_p;
extern volatile bool    cmd_reset_ident;
extern volatile int8_t  cmd_excitar;
extern volatile float   alt_ref_web;
extern volatile int      prueba_motor;
extern volatile float    prueba_mando;
extern volatile uint32_t prueba_hasta;
extern volatile uint32_t ultimoLatido;
extern volatile bool     huboCliente;

void guardarAjustes();
void cargarAjustes();
void msg(const char *m);

static inline void latido() { ultimoLatido = millis(); huboCliente = true; }

// --------------------------------------------------------------- /
static void webRaiz() {
  latido();
  servidor.send_P(200, "text/html", PAGINA);
}

// --------------------------------------------------------------- /t
static void webTelemetria() {
  latido();

  uint32_t hasta = logSeq;
  uint32_t desde = (uint32_t)servidor.arg("s").toInt();
  if (desde > hasta) desde = hasta;                 // la pagina venia de un reinicio
  if (hasta - desde > 40) desde = hasta - 40;       // tope por respuesta

  String ar, ap, ay, bp, bq, br, m0, m1, m2, m3, ah, aref, avz;
  String aft, atof, aus, athr, aest;   // [V_6] registro de vuelo
  String aaz, ausc;                    // [V_11]
  for (uint32_t i = desde; i < hasta; i++) {
    uint32_t k = i % LOG_N;
    if (i > desde) { ar += ','; ap += ','; ay += ','; bp += ','; bq += ','; br += ',';
                     m0 += ','; m1 += ','; m2 += ','; m3 += ',';
                     ah += ','; aref += ','; avz += ',';
                     aft += ','; atof += ','; aus += ','; athr += ','; aest += ',';
                     aaz += ','; ausc += ','; }
    ar += logBuf[k].roll;   ap += logBuf[k].pitch;  ay += logBuf[k].yaw;
    bp += logBuf[k].p;      bq += logBuf[k].q;      br += logBuf[k].r;
    m0 += (int)logBuf[k].m[0]; m1 += (int)logBuf[k].m[1];
    m2 += (int)logBuf[k].m[2]; m3 += (int)logBuf[k].m[3];
    ah += logBuf[k].alt;  aref += logBuf[k].altref;  avz += logBuf[k].vz;
    aft += logBuf[k].ft;  atof += logBuf[k].tof;  aus += logBuf[k].us;
    athr += (int)logBuf[k].thr;  aest += (int)logBuf[k].estado;
    aaz += logBuf[k].az;  ausc += logBuf[k].usc;
  }

  char b[4300];
  int n = 0;
  n += snprintf(b + n, sizeof(b) - n,
    "{\"modo\":%u,\"fallo\":%u,\"enl\":%s,\"msg\":\"%s\",\"seq\":%lu,"
    "\"att\":[%.2f,%.2f,%.2f],\"rate\":[%.1f,%.1f,%.1f],"
    "\"rated\":[%.1f,%.1f,%.1f],"
    "\"sp\":[%.1f,%.1f,%.1f,%.3f],\"thr\":%.3f,",
    G.modo, G.fallo, G.enlace_ok ? "true" : "false", mensaje, (unsigned long)hasta,
    G.roll, G.pitch, G.yaw, G.p, G.q, G.r,
    G.pd, G.qd, G.rd,
    G.sp_roll, G.sp_pitch, G.sp_yaw, G.sp_empuje, G.empuje_suave);
  if (n >= (int)sizeof(b)) n = sizeof(b) - 1;

  n += snprintf(b + n, sizeof(b) - n,
    "\"imu\":{\"ok\":%s,\"a\":[%.3f,%.3f,%.3f],\"g\":[%.2f,%.2f,%.2f],"
    "\"bias\":[%.3f,%.3f,%.3f],\"temp\":%.1f},"
    "\"tof\":{\"ok\":%s,\"m\":%.3f},\"us\":{\"ok\":%s,\"m\":%.3f},\"alt\":%.3f,"
    "\"bat\":{\"ok\":%s,\"v\":%.2f},",
    G.imu_ok ? "true" : "false", G.ax, G.ay, G.az, G.gx, G.gy, G.gz,
    G.bias_gx, G.bias_gy, G.bias_gz, G.temp_imu,
    G.tof_ok ? "true" : "false", G.tof_m,
    G.us_ok ? "true" : "false", G.us_m, G.altura_m,
    G.vbat_ok ? "true" : "false", G.vbat);
  if (n >= (int)sizeof(b)) n = sizeof(b) - 1;

  // [V_2] bloque de altura. vz se publica HACIA ARRIBA positiva, que es lo que
  // espera leer una persona; dentro del lazo es hacia abajo, como en las notas.
  n += snprintf(b + n, sizeof(b) - n,
    "\"alt2\":{\"modo\":%u,\"val\":%s,\"fuente\":%u,\"h\":%.3f,\"ref\":%.3f,"
    "\"vz\":%.3f,\"bias\":%.3f,\"vzv\":%.3f,\"iz\":%.3f,\"obj\":%.3f},",
    G.modo_altura, G.alt_valida ? "true" : "false", G.fuente_alt,
    G.alt_est, G.alt_ref_rampa, -G.vz_est, G.bias_az, G.vz_virtual, G.i_z, G.alt_ref);
  if (n >= (int)sizeof(b)) n = sizeof(b) - 1;

  // [V_3] control robusto: modo activo y perturbacion estimada por el
  // super-twisting, que es la medida en linea de cuanto se ha desviado el dron
  // de su modelo.
  n += snprintf(b + n, sizeof(b) - n,
    "\"rob\":{\"att\":%u,\"alt\":%u,\"p\":[%.3f,%.3f,%.3f],"
    "\"pz\":%.3f,\"sig\":%.3f,\"err\":%.1f},",
    AJ.robusto_att, AJ.robusto_alt,
    G.pert[0], G.pert[1], G.pert[2], G.pert_z, G.sigma_z, G.err_modelo);
  if (n >= (int)sizeof(b)) n = sizeof(b) - 1;

  // [V_5] identificacion en linea: autoridad medida por eje, su calidad, los
  // ejes con el signo invertido y la ganancia que se esta usando de verdad.
  n += snprintf(b + n, sizeof(b) - n,
    "\"id\":{\"g\":[%.3f,%.3f,%.3f],\"cal\":[%.2f,%.2f,%.2f],"
    "\"ok\":%u,\"sig\":%u,\"exc\":%u,\"kpef\":[%.2f,%.2f,%.2f]},",
    G.g_est[0], G.g_est[1], G.g_est[2],
    G.g_cal[0], G.g_cal[1], G.g_cal[2],
    G.ident_ok, G.signo_malo, G.excitando,
    G.kp_ef[0], G.kp_ef[1], G.kp_ef[2]);
  if (n >= (int)sizeof(b)) n = sizeof(b) - 1;

  // [V_7] diagnostico de los dos sensores de distancia, en crudo
  n += snprintf(b + n, sizeof(b) - n,
    "\"dg\":{\"tp\":%s,\"tmm\":%u,\"tst\":%u,\"tlec\":%lu,\"tfal\":%lu,"
    "\"ue\":%lu,\"uus\":%lu,\"ufal\":%lu,\"um\":%.3f},",
    G.tof_presente ? "true" : "false", G.tof_mm_crudo, G.tof_estado,
    (unsigned long)G.tof_lecturas, (unsigned long)G.tof_fallos,
    (unsigned long)G.us_ecos, (unsigned long)G.us_ancho_us,
    (unsigned long)G.us_fallos, G.us_m_crudo);
  if (n >= (int)sizeof(b)) n = sizeof(b) - 1;

  n += snprintf(b + n, sizeof(b) - n,
    "\"u\":[%.4f,%.4f,%.4f,%.3f],\"i\":[%.3f,%.3f,%.3f],"
    "\"mot\":{\"us\":[%d,%d,%d,%d],\"w\":[%.0f,%.0f,%.0f,%.0f],"
    "\"f\":[%.2f,%.2f,%.2f,%.2f],\"n\":[%.3f,%.3f,%.3f,%.3f],\"sat\":%.3f,\"saty\":%.3f},",
    G.ux, G.uy, G.uz, G.ft, G.i_p, G.i_q, G.i_r,
    G.pulso[0], G.pulso[1], G.pulso[2], G.pulso[3],
    G.omega[0], G.omega[1], G.omega[2], G.omega[3],
    G.empuje_motor[0], G.empuje_motor[1], G.empuje_motor[2], G.empuje_motor[3],
    G.mando[0], G.mando[1], G.mando[2], G.mando[3], G.escala_sat, G.escala_yaw);
  if (n >= (int)sizeof(b)) n = sizeof(b) - 1;

  n += snprintf(b + n, sizeof(b) - n,
    "\"sis\":{\"hz\":%.0f,\"umax\":%lu,\"heap\":%lu,\"up\":%lu,\"cli\":%u},"
    "\"aj\":{\"kpa\":[%.3f,%.3f,%.3f],\"kpr\":[%.3f,%.3f,%.3f],"
    "\"kir\":[%.3f,%.3f,%.3f],\"kdr\":[%.4f,%.4f,%.4f],"
    "\"adap\":%u,\"taur\":%.3f,\"taua\":%.3f,\"lam\":%.4f,"
    "\"stk1\":[%.2f,%.2f,%.2f],\"stk2\":[%.2f,%.2f,%.2f],\"stphi\":[%.3f,%.3f,%.3f],"
    "\"stz\":[%.2f,%.2f,%.3f],"
    "\"kz\":[%.3f,%.3f,%.3f],\"vzmax\":%.2f,\"altmax\":%.2f,\"kalq\":%.3f,"
    "\"angmax\":%.1f,\"ratemax\":%.0f,\"yawang\":%u,"
    "\"mk\":[%.2f,%.3f],\"trim\":[%.2f,%.2f],"
    "\"fis\":[%.4f,%.4f,%.5f,%.5f,%.5f,%.4e,%.4e,%.0f],"
    "\"parr\":[%d,%d,%d,%d],\"ptop\":[%d,%d,%d,%d]},",
    G.f_lazo, (unsigned long)G.lazo_us_max, (unsigned long)G.heap,
    (unsigned long)G.uptime_s, G.clientes,
    AJ.kp_ang[0], AJ.kp_ang[1], AJ.kp_ang[2],
    AJ.kp_rate[0], AJ.kp_rate[1], AJ.kp_rate[2],
    AJ.ki_rate[0], AJ.ki_rate[1], AJ.ki_rate[2],
    AJ.kd_rate[0], AJ.kd_rate[1], AJ.kd_rate[2],
    AJ.adaptar, AJ.tau_rate, AJ.tau_ang, AJ.ident_lambda,
    AJ.st_k1[0], AJ.st_k1[1], AJ.st_k1[2],
    AJ.st_k2[0], AJ.st_k2[1], AJ.st_k2[2],
    AJ.st_phi[0], AJ.st_phi[1], AJ.st_phi[2],
    AJ.stz_k1, AJ.stz_k2, AJ.stz_phi,
    AJ.kp_z, AJ.kd_z, AJ.ki_z, AJ.vz_max, AJ.alt_max, AJ.kal_q,
    AJ.ang_max, AJ.rate_max, AJ.yaw_por_angulo,
    AJ.mahony_kp, AJ.mahony_ki, AJ.trim_roll, AJ.trim_pitch,
    AJ.masa, AJ.brazo_l, AJ.Ix, AJ.Iy, AJ.Iz, AJ.kf, AJ.km, AJ.omega_max,
    AJ.pulso_arr[0], AJ.pulso_arr[1], AJ.pulso_arr[2], AJ.pulso_arr[3],
    AJ.pulso_top[0], AJ.pulso_top[1], AJ.pulso_top[2], AJ.pulso_top[3]);
  if (n >= (int)sizeof(b)) n = sizeof(b) - 1;

  n += snprintf(b + n, sizeof(b) - n,
    "\"mod\":{\"fmax\":%.2f,\"fhov\":%.2f,\"tw\":%.2f,\"thov\":%.3f,\"taumax\":%.3f},",
    Mezclador::empujeMax(AJ), Mezclador::empujeHover(AJ), Mezclador::empujePeso(AJ),
    Mezclador::empujeHover(AJ) / Mezclador::empujeMax(AJ),
    Mezclador::torqueMaxHover(AJ));
  if (n >= (int)sizeof(b)) n = sizeof(b) - 1;

  String json = String(b);
  json += "\"g\":{\"r\":[" + ar + "],\"p\":[" + ap + "],\"y\":[" + ay +
          "],\"wp\":[" + bp + "],\"wq\":[" + bq + "],\"wr\":[" + br +
          "],\"m\":[[" + m0 + "],[" + m1 + "],[" + m2 + "],[" + m3 + "]]," +
          "\"h\":[" + ah + "],\"href\":[" + aref + "],\"vz\":[" + avz + "]," +
          "\"ft\":[" + aft + "],\"tof\":[" + atof + "],\"us\":[" + aus + "]," +
          "\"thr\":[" + athr + "],\"est\":[" + aest + "]," +
          "\"az\":[" + aaz + "],\"usc\":[" + ausc + "]}}";

  servidor.send(200, "application/json", json);
}

// --------------------------------------------------------------- /cmd
static void webComando() {
  latido();
  String c = servidor.arg("c");

  if      (c == "armar")    cmd_armar = true;
  else if (c == "parar")    { cmd_desarmar = true; sp_empuje_web = 0; }
  // Mientras el lazo de altura manda, la barra no toca el empuje: si no, los
  // dos escribirian sobre lo mismo y la barra ganaria un ciclo de cada dos.
  else if (c == "thr")      { if (G.modo_altura == ALT_MANUAL)
                                sp_empuje_web = limitar(servidor.arg("v").toFloat(), 0.0f, EMPUJE_MAX_FRAC); }
  // OJO: en una URL el '+' se decodifica como ESPACIO, asi que "thr+" llegaria
  // como "thr ". Por eso los botones usan nombres sin signos y aqui se aceptan
  // las dos formas.
  else if (c == "thrmas" || c == "thr+" || c == "thr ")
                            sp_empuje_web = limitar(sp_empuje_web + 0.02f, 0.0f, EMPUJE_MAX_FRAC);
  else if (c == "thrmenos" || c == "thr-")
                            sp_empuje_web = limitar(sp_empuje_web - 0.02f, 0.0f, EMPUJE_MAX_FRAC);
  else if (c == "ang")      { sp_roll_web  = limitar(servidor.arg("r").toFloat(), -AJ.ang_max, AJ.ang_max);
                              sp_pitch_web = limitar(servidor.arg("p").toFloat(), -AJ.ang_max, AJ.ang_max); }
  else if (c == "yaw")      yaw_rate_cmd = limitar(servidor.arg("v").toFloat(), -RATE_MAX_YAW, RATE_MAX_YAW);
  else if (c == "yawcero")  { cmd_yaw_cero = true; yaw_rate_cmd = 0; }
  // [V_2] altura
  else if (c == "altmodo")  cmd_modo_alt = servidor.arg("v").toInt() ? ALT_MANTENER : ALT_MANUAL;
  else if (c == "altref")   alt_ref_web  = limitar(servidor.arg("v").toFloat(), 0.0f, AJ.alt_max);
  // [V_3] conmutar control robusto, en vuelo
  // [V_5]
  else if (c == "nivelvuelo") cmd_nivel_vuelo = true;
  // [V_8] ajuste fino de nivel, un paso por toque
  else if (c == "trimrd")   cmd_trim_r =  TRIM_PASO;   // inclina a la derecha
  else if (c == "trimri")   cmd_trim_r = -TRIM_PASO;   // a la izquierda
  else if (c == "trimpa")   cmd_trim_p =  TRIM_PASO;   // nariz arriba
  else if (c == "trimpb")   cmd_trim_p = -TRIM_PASO;   // nariz abajo
  else if (c == "resetident") cmd_reset_ident = true;
  else if (c == "excitar")    cmd_excitar = (int8_t)limitar(servidor.arg("v").toFloat(), 0, 2);
  else if (c == "adaptar")    AJ.adaptar = servidor.arg("v").toInt() ? 1 : 0;
  else if (c == "robatt")   AJ.robusto_att = servidor.arg("v").toInt() ? 1 : 0;
  else if (c == "robalt")   AJ.robusto_alt = servidor.arg("v").toInt() ? 1 : 0;
  else if (c == "altsube")  alt_ref_web  = limitar(alt_ref_web + 0.10f, 0.0f, AJ.alt_max);
  else if (c == "altbaja")  alt_ref_web  = limitar(alt_ref_web - 0.10f, 0.0f, AJ.alt_max);
  else if (c == "reseti")   cmd_reset_i  = true;
  else if (c == "calgiro")  cmd_cal_giro = true;
  else if (c == "nivelar")  cmd_nivelar  = true;
  else if (c == "guardar")  { guardarAjustes(); msg("ajustes guardados en memoria"); }
  else if (c == "defecto")  { ajustesPorDefecto(AJ); cmd_reset_i = true; msg("ajustes de fabrica (sin guardar)"); }

  servidor.send(200, "text/plain", "ok");
}

// --------------------------------------------------------------- /pid
static void webPid() {
  latido();
  // Las ganancias SI se pueden cambiar en vuelo: es justo para lo que sirve
  // esta pagina. Lo que no se puede tocar en vuelo es el modelo fisico (/modelo).
  const char *kpa[3] = {"kpar", "kpap", "kpay"};
  const char *kpr[3] = {"kprp", "kprq", "kprr"};
  const char *kir[3] = {"kirp", "kirq", "kirr"};
  const char *kdr[3] = {"kdrp", "kdrq", "kdrr"};
  for (int i = 0; i < 3; i++) {
    if (servidor.hasArg(kpa[i])) AJ.kp_ang[i]  = limitar(servidor.arg(kpa[i]).toFloat(), 0.0f, LIM_KP_ANG);
    if (servidor.hasArg(kpr[i])) AJ.kp_rate[i] = limitar(servidor.arg(kpr[i]).toFloat(), 0.0f, LIM_KP_RATE);
    if (servidor.hasArg(kir[i])) AJ.ki_rate[i] = limitar(servidor.arg(kir[i]).toFloat(), 0.0f, LIM_KI_RATE);
    if (servidor.hasArg(kdr[i])) AJ.kd_rate[i] = limitar(servidor.arg(kdr[i]).toFloat(), 0.0f, LIM_KD_RATE);
  }
  // [V_5] anchos de banda del modo adaptativo
  if (servidor.hasArg("taur")) AJ.tau_rate = limitar(servidor.arg("taur").toFloat(), LIM_TAU_MIN, LIM_TAU_MAX);
  if (servidor.hasArg("taua")) AJ.tau_ang  = limitar(servidor.arg("taua").toFloat(), LIM_TAU_MIN, LIM_TAU_MAX);
  if (servidor.hasArg("lam"))  AJ.ident_lambda = limitar(servidor.arg("lam").toFloat(), 0.990f, 1.0f);

  // [V_3] ganancias del super-twisting
  const char *k1n[3] = {"stk1p", "stk1q", "stk1r"};
  const char *k2n[3] = {"stk2p", "stk2q", "stk2r"};
  const char *phn[3] = {"stphip", "stphiq", "stphir"};
  for (int i = 0; i < 3; i++) {
    if (servidor.hasArg(k1n[i])) AJ.st_k1[i]  = limitar(servidor.arg(k1n[i]).toFloat(), 0.0f, LIM_ST_K1);
    if (servidor.hasArg(k2n[i])) AJ.st_k2[i]  = limitar(servidor.arg(k2n[i]).toFloat(), 0.0f, LIM_ST_K2);
    if (servidor.hasArg(phn[i])) AJ.st_phi[i] = limitar(servidor.arg(phn[i]).toFloat(), 0.001f, LIM_ST_PHI);
  }
  if (servidor.hasArg("stzk1"))  AJ.stz_k1  = limitar(servidor.arg("stzk1").toFloat(), 0.0f, LIM_ST_K1);
  if (servidor.hasArg("stzk2"))  AJ.stz_k2  = limitar(servidor.arg("stzk2").toFloat(), 0.0f, LIM_ST_K2);
  if (servidor.hasArg("stzphi")) AJ.stz_phi = limitar(servidor.arg("stzphi").toFloat(), 0.001f, LIM_ST_PHI);

  // [V_2] lazo de altura
  if (servidor.hasArg("kpz"))   AJ.kp_z   = limitar(servidor.arg("kpz").toFloat(), 0.0f, LIM_KP_Z);
  if (servidor.hasArg("kdz"))   AJ.kd_z   = limitar(servidor.arg("kdz").toFloat(), 0.0f, LIM_KD_Z);
  if (servidor.hasArg("kiz"))   AJ.ki_z   = limitar(servidor.arg("kiz").toFloat(), 0.0f, LIM_KI_Z);
  if (servidor.hasArg("vzmax")) AJ.vz_max = limitar(servidor.arg("vzmax").toFloat(), 0.05f, 2.0f);
  if (servidor.hasArg("altmax"))AJ.alt_max= limitar(servidor.arg("altmax").toFloat(), 0.3f, 3.5f);
  if (servidor.hasArg("kalq"))  AJ.kal_q  = limitar(servidor.arg("kalq").toFloat(), 0.01f, 10.0f);
  if (servidor.hasArg("angmax"))  AJ.ang_max  = limitar(servidor.arg("angmax").toFloat(), 2.0f, 45.0f);
  if (servidor.hasArg("ratemax")) AJ.rate_max = limitar(servidor.arg("ratemax").toFloat(), 20.0f, 400.0f);
  if (servidor.hasArg("yawang"))  AJ.yaw_por_angulo = servidor.arg("yawang").toInt() ? 1 : 0;
  if (servidor.hasArg("mkp"))     AJ.mahony_kp = limitar(servidor.arg("mkp").toFloat(), 0.1f, 20.0f);
  if (servidor.hasArg("mki"))     AJ.mahony_ki = limitar(servidor.arg("mki").toFloat(), 0.0f, 1.0f);
  if (servidor.hasArg("trimr"))   AJ.trim_roll  = limitar(servidor.arg("trimr").toFloat(), -15.0f, 15.0f);
  if (servidor.hasArg("trimp"))   AJ.trim_pitch = limitar(servidor.arg("trimp").toFloat(), -15.0f, 15.0f);

  // [V_4] Ya NO se reinician los integradores en cada peticion. La pagina
  // nueva aplica cada deslizador AL MOMENTO, asi que arrastrar una barra manda
  // decenas de peticiones: reiniciar en cada una dejaria al dron sin la
  // correccion de trim justo mientras se sintoniza. Se reinicia solo cuando
  // cambia una ganancia que es DUENA de un estado integral (las ki, o la k2
  // del super-twisting, que es la que integra). Para lo demas esta el boton
  // explicito de la pestana AJUSTAR.
  bool toca_integral = false;
  const char *claves_i[] = {"kirp", "kirq", "kirr", "kiz",
                          "stk2p", "stk2q", "stk2r", "stzk2"};
  for (unsigned i = 0; i < sizeof(claves_i) / sizeof(claves_i[0]); i++)
    if (servidor.hasArg(claves_i[i])) toca_integral = true;
  if (toca_integral) cmd_reset_i = true;
  msg("ganancias aplicadas");
  servidor.send(200, "text/plain", "ok");
}

// --------------------------------------------------------------- /modelo
static void webModelo() {
  latido();
  if (G.modo != MODO_DESARMADO) {
    servidor.send(200, "text/plain", "solo desarmado");
    return;
  }
  if (servidor.hasArg("m"))    AJ.masa     = limitar(servidor.arg("m").toFloat(), 0.2f, 5.0f);
  if (servidor.hasArg("l"))    AJ.brazo_l  = limitar(servidor.arg("l").toFloat(), 0.05f, 0.5f);
  if (servidor.hasArg("ix"))   AJ.Ix       = limitar(servidor.arg("ix").toFloat(), 1e-4f, 0.2f);
  if (servidor.hasArg("iy"))   AJ.Iy       = limitar(servidor.arg("iy").toFloat(), 1e-4f, 0.2f);
  if (servidor.hasArg("iz"))   AJ.Iz       = limitar(servidor.arg("iz").toFloat(), 1e-4f, 0.4f);
  if (servidor.hasArg("kf"))   AJ.kf       = limitar(servidor.arg("kf").toFloat(), 1e-8f, 1e-3f);
  if (servidor.hasArg("km"))   AJ.km       = limitar(servidor.arg("km").toFloat(), 1e-10f, 1e-4f);
  if (servidor.hasArg("wmax")) AJ.omega_max = limitar(servidor.arg("wmax").toFloat(), 200.0f, 3000.0f);
  for (int i = 0; i < 4; i++) {
    char k1[8], k2[8];
    snprintf(k1, sizeof(k1), "pa%d", i);
    snprintf(k2, sizeof(k2), "pt%d", i);
    if (servidor.hasArg(k1)) AJ.pulso_arr[i] = (int)limitar(servidor.arg(k1).toFloat(), 1000, 1500);
    if (servidor.hasArg(k2)) AJ.pulso_top[i] = (int)limitar(servidor.arg(k2).toFloat(), 1100, 2000);
    if (AJ.pulso_top[i] <= AJ.pulso_arr[i] + 50) AJ.pulso_top[i] = AJ.pulso_arr[i] + 50;
  }
  msg("modelo actualizado");
  servidor.send(200, "text/plain", "ok");
}

// --------------------------------------------------------------- /motor
// Prueba individual. Exige el dron desarmado y la confirmacion ok=1 que la
// pagina solo manda si esta marcada la casilla "helices fuera". El motor se
// para solo a los PRUEBA_MOTOR_MS si la pagina deja de refrescar la orden.
static void webMotor() {
  latido();
  if (G.modo != MODO_DESARMADO && G.modo != MODO_PRUEBA) {
    servidor.send(200, "text/plain", "solo desarmado");
    return;
  }
  if (servidor.arg("ok") != "1") {
    prueba_motor = -1;
    servidor.send(200, "text/plain", "falta confirmacion");
    return;
  }
  int   m = servidor.arg("m").toInt();
  float v = limitar(servidor.arg("v").toFloat(), 0.0f, 0.45f);   // tope de la prueba
  if (m < 0 || m > 3 || v <= 0.0f) {
    prueba_motor = -1;
  } else {
    prueba_mando = v;
    prueba_hasta = millis() + PRUEBA_MOTOR_MS;
    prueba_motor = m;
  }
  servidor.send(200, "text/plain", "ok");
}

static void webNoEncontrado() {
  latido();
  // Cualquier otra ruta devuelve la pagina: asi el "portal cautivo" del movil
  // no deja al usuario en una pantalla en blanco.
  servidor.sendHeader("Location", "/", true);
  servidor.send(302, "text/plain", "");
}

// --------------------------------------------------------------- arranque
void iniciarWeb() {
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);                    // sin ahorro de energia: menos latencia
  WiFi.softAP(WIFI_RED, WIFI_CLAVE, WIFI_CANAL, 0, 4);

  servidor.on("/",       webRaiz);
  servidor.on("/t",      webTelemetria);
  servidor.on("/cmd",    webComando);
  servidor.on("/pid",    webPid);
  servidor.on("/modelo", webModelo);
  servidor.on("/motor",  webMotor);
  servidor.onNotFound(webNoEncontrado);
  servidor.begin();

  Serial0.print("WiFi: red "); Serial0.print(WIFI_RED);
  Serial0.print("  clave "); Serial0.print(WIFI_CLAVE);
  Serial0.print("  -> http://"); Serial0.println(WiFi.softAPIP());
}

#endif // WEB_H
