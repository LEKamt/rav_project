/*
 * VueloF450: firmware de vuelo de prueba del F450 con ESP32-S3 (Arduino, sin micro-ROS)
 * ------------------------------------------------------------------------------------
 * Sucesor de hardware/PrimerVuelo.ino, con los mismos pines y sensores. Qué cambia y por qué:
 *
 *  - Ejes de la IMU: PrimerVuelo hacía ax = raw_ay, ay = raw_ax (un ESPEJO, no una rotación): el
 *    giroscopio y el acelerómetro empujaban el ángulo en sentidos opuestos. Aquí el cambio de ejes
 *    es una rotación configurable (IMU_CUERPO_*) y el comando 'c' comprueba que gire y acelerómetro
 *    coincidan. No arma hasta que EJES_VERIFICADOS sea 1.
 *  - Sin recalibrar los ESC al arrancar: PrimerVuelo mandaba 1600 us durante 3 s en setup(). Si el
 *    ESP32 se reinicia con los ESC encendidos, eso es 60 % de empuje con hélices. Aquí los ESC
 *    reciben paro desde el primer instante y la calibración es un comando aparte ('k').
 *  - Control en cascada (ángulo -> velocidad angular) con PID o SMC, guiñada incluida, a 500 Hz
 *    fijos. PrimerVuelo tenía solo P sobre el ángulo con zona muerta y sin guiñada.
 *  - PWM a 400 Hz (PrimerVuelo: 50 Hz, 20 ms de retardo) con mezcla desaturada (src/mezcla.h).
 *  - Filtros del giro (notch + Butterworth), AHRS Mahony y Kalman de altura (ToF + ultrasonido):
 *    las mismas cabeceras probadas del firmware micro-ROS (src_microros/src).
 *  - Failsafes: sin enlace 1 s -> bajada gradual; inclinación > 60 deg o IMU caída -> paro.
 *  - Telemetría CSV para comparar PID contra SMC (escalones con 'e').
 *
 * Placa: "ESP32S3 Dev Module" (núcleo arduino-esp32 3.x). Librería: Adafruit_VL53L0X.
 * Serial0 (puerto UART/CH340) a 921600 baudios. Operar con hardware/VueloF450/estacion.py, que
 * manda el latido del enlace y guarda la telemetría. Comandos: escribir 'ayuda'.
 *
 * Cabeceras en src/: copias de src_microros/src (robot_tests/unit/test_vuelo_arduino.py
 * comprueba que sean idénticas). Si se cambia una, cambiar la otra.
 */

#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include "src/filtros.h"
#include "src/ahrs.h"
#include "src/smc.h"
#include "src/kalman_altura.h"
#include "src/mezcla.h"
#include "eco_sonar.h"

// =============================== PINES (hardware/CABLEADO_S3.md) ===============================
const int PIN_M[4] = {8, 17, 21, 18};   // M1 frente-izq (horario), M2 frente-der (antihorario),
                                        // M3 tras-der (horario), M4 tras-izq (antihorario)
#define IMU_SDA 1
#define IMU_SCL 39
#define TOF_SDA 4
#define TOF_SCL 5
#define US_TRIG 6
#define US_ECHO 7          // HC-SR04 de 5 V: divisor 1 k / 2 k antes del pin

// ===================================== EJES DE LA IMU =====================================
// Cuerpo FLU (x adelante, y izquierda, z arriba) a partir del sensor s[0..2]. El MISMO cambio se
// aplica al acelerómetro y al giro, y debe ser una rotación. Partida: PrimerVuelo indica la IMU
// girada 90 deg sobre z; con el signo que la vuelve rotación queda x = s[1], y = -s[0].
// Comprobar con 'c' y 'v' (tabla en CABLEADO_S3.md) y después poner EJES_VERIFICADOS en 1.
#define IMU_CUERPO_X(s) ((s)[1])
#define IMU_CUERPO_Y(s) (-(s)[0])
#define IMU_CUERPO_Z(s) ((s)[2])
#define EJES_VERIFICADOS 0

// ==================================== PWM DE LOS ESC ====================================
#define PWM_HZ        400     // SimonK/BLHeli lo aceptan. Si un ESC no arranca o tartamudea, 50
#define PWM_BITS      14
#define US_PARO       1000    // motor detenido
#define US_MAX        2000    // tope del rango calibrado de los ESC (comando 'k')
#define MOTOR_IDLE    0.08f   // armado, por debajo de esto un ESC puede perder sincronía
#define MANDO_MAX     0.80f   // tope de cada motor en las primeras pruebas (0..1)
#define MANDO_BANCO   0.25f   // tope del empuje en modo banco (sin hélices)

// ==================================== LAZO Y FILTROS ====================================
#define LAZO_HZ       500
#define DLPF_MPU      2       // registro CONFIG: 2 = 92 Hz
#define GIRO_LPF_HZ   60.0f
#define NOTCH_ON      false   // encenderlo después de ver el espectro con hélices
#define NOTCH_HZ      90.0f
#define NOTCH_BW      30.0f
#define ACC_LPF_HZ    20.0f
#define AHRS_KP       1.0f
#define AHRS_KI       0.02f

// ================================= CONTROL (de config.h) =================================
// PID en cascada: ángulo [deg] -> velocidad angular [deg/s] -> corrección de mando
float kpAng = 4.5f, rateMax = 150.0f;
float kpRate = 0.0020f, kiRate = 0.0040f, kdRate = 0.00006f;
float kpYaw = 0.0040f, kiYaw = 0.0060f;
#define CORR_MAX      0.35f
#define I_MAX         0.10f
#define EMPUJE_INTEGRA 0.20f  // no integrar en el suelo (por debajo de este empuje)
// SMC (smc.h, ec. 16 de las notas del curso)
#define SMC_IXX 0.0115f
#define SMC_IYY 0.0115f
#define SMC_IZZ 0.0218f
#define SMC_BRAZO 0.1589f
#define SMC_EMPUJE_MAX 8.3f
#define SMC_K_YAW 0.016f
float smcLambda = 4.5f, smcKd = 0.25f, smcRho = 0.15f, smcPhi = 0.50f, smcP = 1.0f;
#define SMC_SESGO_MAX 0.30f
#define SMC_KD_YAW 0.10f
#define SMC_RHO_YAW 0.03f
#define SMC_PHI_YAW 0.50f
#define SMC_P_YAW 0.2f
#define SMC_SESGO_MAX_YAW 0.10f

#define SP_ANG_MAX    15.0f   // deg
#define SP_YAW_MAX    90.0f   // deg/s

// ======================================= SEGURIDAD =======================================
#define ENLACE_TIMEOUT_MS 1000    // sin bytes del PC -> bajada gradual
#define BAJADA_POR_S      0.05f   // del empuje por segundo (la misma rampa de Blaze)
#define INCLINACION_CORTE 60.0f   // deg: más que esto es un choque -> paro
#define INCLINACION_ARMAR 10.0f
#define FALLOS_IMU_CORTE  25      // ciclos seguidos (50 ms)
#define RAMPA_EMPUJE_POR_S 0.5f   // el empuje real sigue a la consigna a esta velocidad

// ======================================= ALTURA =======================================
#define TOF_MAX_M     1.00f
#define SONAR_MAX_M   4.00f
#define SONAR_PERIODO_MS 60

// ========================================= ESTADO =========================================
enum Modo { DESARMADO, ARMADO, BAJADA, MOTOR_PRUEBA, CALIBRA_ESC };
const char *NOMBRE_MODO[] = {"DESARMADO", "ARMADO", "BAJADA", "MOTOR", "CALIB_ESC"};
volatile Modo modo = DESARMADO;
bool leySMC = false;
bool banco = false;            // sin hélices: tope MANDO_BANCO y sin failsafe de enlace

Adafruit_VL53L0X tof;
bool tofOK = false;
FiltroEjes filtroGiro, filtroAcc;
Mahony ahrs;
KalmanAltura kalman;

float sesgoGiro[3] = {0, 0, 0};
bool giroCalibrado = false;
float giro[3] = {0, 0, 0}, giroF[3] = {0, 0, 0}, accF[3] = {0, 0, 1};   // deg/s y g, cuerpo
float roll = 0, pitch = 0, yaw = 0;   // deg
uint32_t fallosIMU = 0;

float empujeConsigna = 0, empuje = 0;
float spRoll = 0, spPitch = 0, spYaw = 0;
float corr[3] = {0, 0, 0};
float mando[4] = {0, 0, 0, 0};
uint16_t pulso[4] = {US_PARO, US_PARO, US_PARO, US_PARO};
float iRoll = 0, iPitch = 0, iYaw = 0, rPrev = 0, pPrev = 0;
float sesgoR = 0, sesgoP = 0, sesgoY = 0;

uint32_t ultimoRx = 0;
uint32_t lazoMaxUs = 0, atrasos = 0;
uint32_t telPeriodoMs = 20;     // 50 Hz
int motorPrueba = -1;
uint32_t finMotorPrueba = 0;
float escalonValor = 0;
char escalonEje = 0;
uint32_t finEscalon = 0;

// Altura
float accZSuma = 0;
uint32_t accZN = 0;
float alturaTof = 0, alturaSonar = 0, offsetTof = 0, offsetSonar = 0;
const float D2R = 0.017453293f;   // DEG_TO_RAD de Arduino es double

// ========================================= PWM =========================================
void escribirPulso(int i, uint16_t us) {
    pulso[i] = us;
    const uint32_t periodoUs = 1000000UL / PWM_HZ;
    ledcWrite(PIN_M[i], (uint32_t)((uint64_t)us * ((1UL << PWM_BITS) - 1) / periodoUs));
}

void pararMotores() {
    for (int i = 0; i < 4; i++) escribirPulso(i, US_PARO);
}

uint16_t mandoAPulso(float m) {
    return (uint16_t)(US_PARO + m * (US_MAX - US_PARO));
}

// ========================================= IMU =========================================
bool mpuEscribir(uint8_t reg, uint8_t v) {
    Wire.beginTransmission(0x68);
    Wire.write(reg);
    Wire.write(v);
    return Wire.endTransmission() == 0;
}

bool mpuLeer(uint8_t reg, uint8_t *b, uint8_t n) {
    Wire.beginTransmission(0x68);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)0x68, n) != n) return false;
    for (uint8_t i = 0; i < n; i++) b[i] = Wire.read();
    return true;
}

bool mpuIniciar() {
    uint8_t quien = 0;
    if (!mpuEscribir(0x6B, 0x80)) return false;      // reinicio
    delay(100);
    mpuLeer(0x75, &quien, 1);
    Serial0.printf("# IMU WHO_AM_I = 0x%02X %s\n", quien,
                   quien == 0x71 ? "(MPU-9250)" : quien == 0x73 ? "(MPU-9255)" :
                   quien == 0x70 ? "(MPU-6500)" : quien == 0x68 ? "(MPU-6050)" : "(desconocida)");
    bool ok = mpuEscribir(0x6B, 0x01)        // reloj del PLL
           && mpuEscribir(0x19, 0x00)        // 1 kHz interno
           && mpuEscribir(0x1A, DLPF_MPU)
           && mpuEscribir(0x1B, 0x08)        // ±500 deg/s
           && mpuEscribir(0x1C, 0x10);       // ±8 g (las vibraciones saturan ±4 g)
    if (quien != 0x68) mpuEscribir(0x1D, 0x03);   // DLPF del acelerómetro (no existe en la 6050)
    delay(50);
    return ok;
}

// Acelerómetro [g] y giro [deg/s] en el cuerpo FLU
bool leerCuerpo(float acc[3], float g[3]) {
    uint8_t b[14];
    if (!mpuLeer(0x3B, b, 14)) return false;
    float a[3], w[3];
    for (int i = 0; i < 3; i++) {
        a[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]) / 4096.0f;
        w[i] = (int16_t)((b[8 + 2 * i] << 8) | b[9 + 2 * i]) / 65.5f;
    }
    acc[0] = IMU_CUERPO_X(a); acc[1] = IMU_CUERPO_Y(a); acc[2] = IMU_CUERPO_Z(a);
    g[0] = IMU_CUERPO_X(w);   g[1] = IMU_CUERPO_Y(w);   g[2] = IMU_CUERPO_Z(w);
    return true;
}

bool calibrarGiro() {
    for (int intento = 0; intento < 5; intento++) {
        double s[3] = {0, 0, 0}, s2[3] = {0, 0, 0};
        int n = 0;
        for (int i = 0; i < 1000; i++) {
            float a[3], g[3];
            if (leerCuerpo(a, g)) {
                for (int k = 0; k < 3; k++) { s[k] += g[k]; s2[k] += (double)g[k] * g[k]; }
                n++;
            }
            delay(2);
        }
        if (n < 500) continue;
        bool quieto = true;
        for (int k = 0; k < 3; k++) {
            sesgoGiro[k] = s[k] / n;
            if (s2[k] / n - (double)sesgoGiro[k] * sesgoGiro[k] > 4.0) quieto = false;   // sigma > 2 deg/s
        }
        if (quieto) return true;
        Serial0.println("# Se movio durante la calibracion del giro: repitiendo");
    }
    return false;
}

// ======================================== CONTROL ========================================
float pidVelocidad(float sp, float med, float medPrev, float kp, float ki, float kd,
                   float *integral, float dt, bool integrar) {
    const float e = sp - med;
    const float sinI = kp * e + kd * (medPrev - med) / dt;    // D sobre la medida
    if (integrar) {
        float cand = constrain(*integral + ki * e * dt, -I_MAX, I_MAX);
        const float tent = sinI + cand;
        // Anti-windup: no integrar hacia una saturación ya alcanzada
        if (!((tent > CORR_MAX && e > 0) || (tent < -CORR_MAX && e < 0))) *integral = cand;
    }
    return constrain(sinI + *integral, -CORR_MAX, CORR_MAX);
}

void controlActitud(float dt) {
    const bool integrar = empuje > EMPUJE_INTEGRA;
    float sr = constrain(spRoll, -SP_ANG_MAX, SP_ANG_MAX);
    float sp = constrain(spPitch, -SP_ANG_MAX, SP_ANG_MAX);
    if (millis() < finEscalon) {
        if (escalonEje == 'r') sr = escalonValor;
        if (escalonEje == 'p') sp = escalonValor;
    }
    const float sy = constrain(spYaw, -SP_YAW_MAX, SP_YAW_MAX);

    if (!leySMC) {
        const float rr = constrain(kpAng * (sr - roll), -rateMax, rateMax);
        const float rp = constrain(kpAng * (sp - pitch), -rateMax, rateMax);
        corr[0] = pidVelocidad(rr, giroF[0], rPrev, kpRate, kiRate, kdRate, &iRoll, dt, integrar);
        corr[1] = pidVelocidad(rp, giroF[1], pPrev, kpRate, kiRate, kdRate, &iPitch, dt, integrar);
        corr[2] = pidVelocidad(sy, giroF[2], giroF[2], kpYaw, kiYaw, 0.0f, &iYaw, dt, integrar);
    } else {
        const float D = D2R;
        const float p = giroF[0] * D, q = giroF[1] * D, r = giroF[2] * D;
        float sR = sesgoR, sP = sesgoP, sY = sesgoY;
        EjeSMC eR = smcEje(roll * D, sr * D, p, SMC_IXX, q * r * (SMC_IZZ - SMC_IYY), smcLambda,
                           smcKd, smcRho, smcPhi, smcP, SMC_SESGO_MAX, &sR, dt);
        EjeSMC eP = smcEje(pitch * D, sp * D, q, SMC_IYY, p * r * (SMC_IXX - SMC_IZZ), smcLambda,
                           smcKd, smcRho, smcPhi, smcP, SMC_SESGO_MAX, &sP, dt);
        const float sYaw = r - sy * D;
        sY = smcAdaptar(sY, sYaw, SMC_PHI_YAW, SMC_P_YAW, SMC_SESGO_MAX_YAW, dt);
        const float tauZ = smcLey(p * q * (SMC_IYY - SMC_IXX) + sY, sYaw, SMC_KD_YAW, SMC_RHO_YAW,
                                  SMC_PHI_YAW);
        if (integrar) { sesgoR = sR; sesgoP = sP; sesgoY = sY; }    // misma regla que el PID
        const float porCorr = 4.0f * SMC_EMPUJE_MAX;
        corr[0] = constrain(eR.tau / (porCorr * SMC_BRAZO), -CORR_MAX, CORR_MAX);
        corr[1] = constrain(eP.tau / (porCorr * SMC_BRAZO), -CORR_MAX, CORR_MAX);
        corr[2] = constrain(tauZ / (porCorr * SMC_K_YAW), -CORR_MAX, CORR_MAX);
    }
    rPrev = giroF[0];
    pPrev = giroF[1];
}

void resetControl() {
    iRoll = iPitch = iYaw = 0;
    sesgoR = sesgoP = sesgoY = 0;
    rPrev = giroF[0];
    pPrev = giroF[1];
    corr[0] = corr[1] = corr[2] = 0;
}

// ======================================== MODOS ========================================
void desarmar(const char *motivo) {
    if (modo != DESARMADO) Serial0.printf("# DESARMADO: %s\n", motivo);
    modo = DESARMADO;
    empuje = empujeConsigna = 0;
    spRoll = spPitch = spYaw = 0;
    finEscalon = 0;
    pararMotores();
}

const char *motivoNoArmar() {
    if (!EJES_VERIFICADOS) return "ejes sin verificar: probar con 'c' y 'v' y poner EJES_VERIFICADOS 1";
    if (!giroCalibrado) return "giro sin calibrar (se movio al arrancar): reiniciar quieto";
    if (fallosIMU) return "IMU sin lecturas";
    const float ex = accF[0], ey = accF[1], ez = accF[2] - 1.0f;
    if (sqrtf(ex * ex + ey * ey + ez * ez) > 0.2f) return "acelerometro no marca (0,0,1) g: revisar ejes";
    if (fabsf(roll) > INCLINACION_ARMAR || fabsf(pitch) > INCLINACION_ARMAR) return "dron inclinado";
    if (empujeConsigna > 0) return "empuje distinto de 0";
    return nullptr;
}

void armar() {
    if (modo != DESARMADO) { Serial0.println("# Ya esta armado o en otro modo"); return; }
    const char *m = motivoNoArmar();
    if (m) { Serial0.printf("# NO ARMA: %s\n", m); return; }
    resetControl();
    empuje = empujeConsigna = 0;
    modo = ARMADO;
    Serial0.printf("# ARMADO (%s%s). Subir el empuje con + o 't 0.xx'\n", leySMC ? "SMC" : "PID",
                   banco ? ", BANCO" : "");
}

// ====================================== TELEMETRÍA ======================================
void encabezado() {
    Serial0.println("t_ms,modo,ley,empuje,roll,pitch,yaw,p,q,r,sp_roll,sp_pitch,sp_r,"
                    "c_roll,c_pitch,c_yaw,m1,m2,m3,m4,z,vz,tof,sonar,lazo_max_us,atrasos");
}

void telemetria() {
    float sr = spRoll, sp = spPitch;
    if (millis() < finEscalon) { if (escalonEje == 'r') sr = escalonValor; if (escalonEje == 'p') sp = escalonValor; }
    Serial0.printf("%lu,%d,%d,%.3f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,"
                   "%.4f,%.4f,%.4f,%u,%u,%u,%u,%.3f,%.3f,%.3f,%.3f,%lu,%lu\n",
                   (unsigned long)millis(), (int)modo, (int)leySMC, empuje, roll, pitch, yaw,
                   giroF[0], giroF[1], giroF[2], sr, sp, spYaw, corr[0], corr[1], corr[2],
                   pulso[0], pulso[1], pulso[2], pulso[3], kalman.z, kalman.vz, alturaTof, alturaSonar,
                   (unsigned long)lazoMaxUs, (unsigned long)atrasos);
    lazoMaxUs = 0;
}

// ============================= COMPROBACIÓN DE EJES ('c') =============================
// Durante 8 s el operador inclina el dron a mano (alabeo y cabeceo, sin hélices). Si el giro y
// el acelerómetro describen el mismo movimiento, la suma de (giro * cambio del ángulo del
// acelerómetro) es positiva. Negativa = espejo o signo invertido en ese eje.
void comprobarEjes() {
    Serial0.println("# CHEQUEO DE EJES: inclina el dron de lado a lado y de adelante atras durante 8 s");
    double cr = 0, cp = 0, mr = 0, mp = 0;
    float rAnt = NAN, pAnt = NAN;
    const uint32_t fin = millis() + 8000;
    while (millis() < fin) {
        float a[3], g[3];
        if (leerCuerpo(a, g)) {
            for (int k = 0; k < 3; k++) g[k] -= sesgoGiro[k];
            const float ra = atan2f(a[1], a[2]) * RAD_TO_DEG;
            const float pa = atan2f(-a[0], sqrtf(a[1] * a[1] + a[2] * a[2])) * RAD_TO_DEG;
            if (!isnan(rAnt)) { cr += g[0] * (ra - rAnt); cp += g[1] * (pa - pAnt); }
            mr = fmax(mr, fabs(g[0])); mp = fmax(mp, fabs(g[1]));
            rAnt = ra; pAnt = pa;
        }
        delay(5);
    }
    if (mr < 20 || mp < 20) {
        Serial0.println("# Movimiento insuficiente: repetir inclinando mas (> 20 deg/s en cada eje)");
        return;
    }
    Serial0.printf("# Alabeo:  %s (%.0f)\n", cr > 0 ? "OK" : "INVERTIDO: revisar IMU_CUERPO_Y", cr);
    Serial0.printf("# Cabeceo: %s (%.0f)\n", cp > 0 ? "OK" : "INVERTIDO: revisar IMU_CUERPO_X", cp);
    Serial0.println("# Falta el sentido: con 'v', nariz abajo => pitch POSITIVO; lado izq arriba => roll POSITIVO;");
    Serial0.println("# girar a la izquierda (antihorario visto desde arriba) => r POSITIVO.");
}

// ======================================== COMANDOS ========================================
void ayuda() {
    Serial0.println(
        "# a armar | d desarmar | x PARO | s bajada | + / - empuje 0.02 | t 0.40 empuje\n"
        "# r 5 / p -5 / y 30 consigna de alabeo, cabeceo [deg] y guinada [deg/s] | 0 consignas a cero\n"
        "# e r 5 1 escalon en alabeo de 5 deg durante 1 s (e p ... cabeceo)\n"
        "# ley pid | ley smc (desarmado) | g kpAng kpRate kiRate kdRate kpYaw kiYaw | gs lambda kd rho phi p\n"
        "# banco 1|0 (sin helices: tope 0.25 y sin failsafe de enlace) | tel 50 (Hz, 0 apaga)\n"
        "# c chequeo de ejes | v ver IMU | m 1..4 girar un motor 2 s (SIN HELICES) | k calibrar ESC\n"
        "# h encabezado CSV | ayuda");
}

bool verIMU = false;

void comando(char *l) {
    while (*l == ' ') l++;
    if (!*l) return;                                     // línea vacía = latido
    char c0[8] = {0};
    float v[6] = {0};
    int n = sscanf(l, "%7s %f %f %f %f %f %f", c0, &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) - 1;
    String c(c0);
    verIMU = false;

    if (c == "x") { desarmar("PARO pedido"); return; }
    if (c == "d") { desarmar("pedido"); return; }
    if (c == "a") { armar(); return; }
    if (c == "s") { if (modo == ARMADO) { modo = BAJADA; Serial0.println("# BAJADA"); } return; }
    if (c == "+" || c == "-" || c == "t") {
        if (modo != ARMADO) { Serial0.println("# Armar primero"); return; }
        float t = c == "t" ? v[0] : empujeConsigna + (c == "+" ? 0.02f : -0.02f);
        empujeConsigna = constrain(t, 0.0f, banco ? MANDO_BANCO : MANDO_MAX);
        Serial0.printf("# empuje -> %.2f\n", empujeConsigna);
        return;
    }
    if (c == "r") { spRoll = v[0]; return; }
    if (c == "p") { spPitch = v[0]; return; }
    if (c == "y") { spYaw = v[0]; return; }
    if (c == "0") { spRoll = spPitch = spYaw = 0; return; }
    if (c == "e") {
        char eje = 0;
        float amp = 0, dur = 0;
        if (sscanf(l, "e %c %f %f", &eje, &amp, &dur) == 3 && (eje == 'r' || eje == 'p')) {
            escalonEje = eje;
            escalonValor = constrain(amp, -SP_ANG_MAX, SP_ANG_MAX);
            finEscalon = millis() + (uint32_t)(constrain(dur, 0.1f, 5.0f) * 1000);
            Serial0.printf("# ESCALON %c %.1f deg %.1f s\n", eje, escalonValor, dur);
        }
        return;
    }
    if (c == "ley") {
        if (modo != DESARMADO) { Serial0.println("# Cambiar la ley solo desarmado"); return; }
        leySMC = strstr(l, "smc") != nullptr;
        Serial0.printf("# Ley: %s\n", leySMC ? "SMC" : "PID");
        return;
    }
    if (c == "g" && n == 6) {
        kpAng = v[0]; kpRate = v[1]; kiRate = v[2]; kdRate = v[3]; kpYaw = v[4]; kiYaw = v[5];
        resetControl();
        Serial0.printf("# PID: kpAng %.3f kpRate %.5f kiRate %.5f kdRate %.6f kpYaw %.5f kiYaw %.5f\n",
                       kpAng, kpRate, kiRate, kdRate, kpYaw, kiYaw);
        return;
    }
    if (c == "gs" && n == 5) {
        smcLambda = v[0]; smcKd = v[1]; smcRho = v[2]; smcPhi = v[3]; smcP = v[4];
        resetControl();
        Serial0.printf("# SMC: lambda %.2f kd %.3f rho %.3f phi %.2f p %.2f\n", smcLambda, smcKd, smcRho,
                       smcPhi, smcP);
        return;
    }
    if (c == "banco") {
        if (modo != DESARMADO) { Serial0.println("# Cambiar banco solo desarmado"); return; }
        banco = v[0] != 0;
        Serial0.printf("# Banco %s\n", banco ? "SI (tope 0.25, sin failsafe de enlace)" : "NO");
        return;
    }
    if (c == "tel") { telPeriodoMs = v[0] > 0 ? (uint32_t)(1000 / v[0]) : 0; return; }
    if (c == "h") { encabezado(); return; }
    if (c == "c") { if (modo == DESARMADO) comprobarEjes(); return; }
    if (c == "v") { verIMU = true; return; }
    if (c == "m") {
        if (modo != DESARMADO || v[0] < 1 || v[0] > 4) { Serial0.println("# m 1..4, desarmado"); return; }
        Serial0.printf("# MOTOR %d durante 2 s (SIN HELICES). Debe girar el de su posicion y sentido\n", (int)v[0]);
        motorPrueba = (int)v[0] - 1;
        finMotorPrueba = millis() + 2000;
        modo = MOTOR_PRUEBA;
        return;
    }
    if (c == "k") {
        if (modo == DESARMADO && strstr(l, "ok")) {
            modo = CALIBRA_ESC;
            for (int i = 0; i < 4; i++) escribirPulso(i, US_MAX);
            Serial0.println("# Salida en MAXIMO. Conecta la bateria de los ESC; tras los pitidos escribe 'k fin'");
        } else if (modo == CALIBRA_ESC && strstr(l, "fin")) {
            pararMotores();
            modo = DESARMADO;
            Serial0.println("# Salida en MINIMO. Al terminar los pitidos los ESC quedan en 1000-2000 us");
        } else {
            Serial0.println("# Calibrar ESC: QUITA LAS HELICES y desconecta la bateria; luego 'k ok'");
        }
        return;
    }
    if (c == "ayuda") { ayuda(); return; }
    Serial0.printf("# Comando desconocido: %s (ayuda)\n", l);
}

void leerComandos() {
    static char linea[64];
    static uint8_t n = 0;
    while (Serial0.available()) {
        const char ch = Serial0.read();
        ultimoRx = millis();
        if (ch == '\n' || ch == '\r') {
            linea[n] = 0;
            if (n) comando(linea);
            n = 0;
        } else if (n < sizeof(linea) - 1) {
            linea[n++] = ch;
        }
    }
}

// ========================================= SETUP =========================================
void setup() {
    // Paro a los ESC antes que nada (sin secuencia de máximo: ver el encabezado)
    for (int i = 0; i < 4; i++) ledcAttach(PIN_M[i], PWM_HZ, PWM_BITS);
    pararMotores();

    Serial0.setTxBufferSize(4096);        // la telemetría no debe bloquear el lazo
    Serial0.begin(921600);
    delay(500);
    Serial0.println("\n# VueloF450: NO MOVER, calibrando");

    Wire.begin(IMU_SDA, IMU_SCL, 400000);
    Wire1.begin(TOF_SDA, TOF_SCL, 400000);
    if (!mpuIniciar()) {
        Serial0.println("# FATAL: la IMU no responde. Motores en paro");
        for (;;) { pararMotores(); delay(500); }
    }
    giroCalibrado = calibrarGiro();
    Serial0.printf("# Sesgo del giro %.2f %.2f %.2f deg/s %s\n", sesgoGiro[0], sesgoGiro[1], sesgoGiro[2],
                   giroCalibrado ? "" : "(SIN CALIBRAR: no arma)");

    filtroGiro.configurar(LAZO_HZ, GIRO_LPF_HZ, NOTCH_ON, NOTCH_HZ, NOTCH_BW);
    filtroAcc.configurar(LAZO_HZ, ACC_LPF_HZ, false, 0, 0);
    ahrs.configurar(AHRS_KP, AHRS_KI, 0.15f);
    float a[3], g[3];
    if (leerCuerpo(a, g)) {
        const float cero[3] = {0, 0, 0};
        filtroAcc.reset(a);
        filtroGiro.reset(cero);
        ahrs.iniciarConAcc(a);
        for (int k = 0; k < 3; k++) accF[k] = a[k];
    }

    tofOK = tof.begin(0x29, false, &Wire1);
    if (tofOK) {
        tof.startRangeContinuous(20);
        delay(100);
        float s = 0; int n = 0;
        for (int i = 0; i < 20; i++) {
            delay(25);
            if (tof.isRangeComplete()) { uint16_t mm = tof.readRangeResult(); if (mm > 10 && mm < 2000) { s += mm / 1000.0f; n++; } }
        }
        offsetTof = n ? s / n : 0;
    }
    iniciarSonar(US_TRIG, US_ECHO);
    {
        float s = 0; int n = 0;
        for (int i = 0; i < 8; i++) { dispararSonar(); delay(SONAR_PERIODO_MS); float d = leerSonar(); if (d > 0 && d < 1) { s += d; n++; } }
        offsetSonar = n ? s / n : 0;
        Serial0.printf("# ToF %s (suelo %.3f m), ultrasonido %s (suelo %.3f m)\n", tofOK ? "OK" : "NO",
                       offsetTof, n >= 4 ? "OK" : "SIN ECO", offsetSonar);
    }
    kalman.reset(0, 0);

    if (!EJES_VERIFICADOS) Serial0.println("# AVISO: EJES_VERIFICADOS = 0, no arma. Usar 'c' y 'v'");
    ayuda();
    encabezado();
    ultimoRx = millis();
}

// ========================================= LAZO =========================================
void loop() {
    static uint32_t tAnterior = micros();
    const uint32_t periodo = 1000000UL / LAZO_HZ;
    leerComandos();
    uint32_t ahora = micros();
    if (ahora - tAnterior < periodo) return;
    if (ahora - tAnterior > periodo + periodo / 2) atrasos++;
    const float dt = periodo * 1e-6f;          // nominal: el lazo tiene periodo fijo
    tAnterior += periodo;
    if (ahora - tAnterior > 10 * periodo) tAnterior = ahora;   // tras un bloqueo largo, no recuperar

    // ---------------- IMU, filtros y AHRS ----------------
    float a[3], g[3];
    if (leerCuerpo(a, g)) {
        fallosIMU = 0;
        for (int k = 0; k < 3; k++) giro[k] = g[k] - sesgoGiro[k];
        filtroGiro.paso(giro, giroF);
        filtroAcc.paso(a, accF);
        float gr[3] = {giro[0] * D2R, giro[1] * D2R, giro[2] * D2R};
        ahrs.actualizar(gr, accF, dt);
        accZSuma += ahrs.aceleracionVertical(accF, 9.80665f);
        accZN++;
    } else {
        fallosIMU++;
        float gr[3] = {giro[0] * D2R, giro[1] * D2R, giro[2] * D2R};
        const float sinAcc[3] = {0, 0, 0};
        ahrs.actualizar(gr, sinAcc, dt);
    }
    float rr, pp, yy;
    ahrs.euler(&rr, &pp, &yy);
    roll = rr * RAD_TO_DEG; pitch = pp * RAD_TO_DEG; yaw = yy * RAD_TO_DEG;

    // ---------------- Seguridad ----------------
    const bool volando = modo == ARMADO || modo == BAJADA;
    if (volando && (fabsf(roll) > INCLINACION_CORTE || fabsf(pitch) > INCLINACION_CORTE))
        desarmar("inclinacion > 60 deg");
    if (volando && fallosIMU > FALLOS_IMU_CORTE) desarmar("IMU sin lecturas");
    if (modo == ARMADO && !banco && millis() - ultimoRx > ENLACE_TIMEOUT_MS) {
        modo = BAJADA;
        Serial0.println("# SIN ENLACE: bajada gradual");
    }

    // ---------------- Empuje ----------------
    if (modo == BAJADA) {
        spRoll = spPitch = spYaw = 0;
        finEscalon = 0;
        empujeConsigna = fmaxf(0, empujeConsigna - BAJADA_POR_S * dt);
        empuje = fminf(empuje, empujeConsigna);
        const bool enSuelo = tofOK && alturaTof > 0 && alturaTof < 0.08f;
        if (empuje <= MOTOR_IDLE + 0.01f || enSuelo) desarmar("bajada terminada");
    } else if (modo == ARMADO) {
        const float paso = RAMPA_EMPUJE_POR_S * dt;
        empuje += constrain(empujeConsigna - empuje, -paso, paso);
    }

    // ---------------- Control y motores ----------------
    if (modo == ARMADO || modo == BAJADA) {
        controlActitud(dt);
        const float tope = banco ? MANDO_BANCO : MANDO_MAX;
        mezclarX(fmaxf(empuje, MOTOR_IDLE), corr[0], corr[1], corr[2], MOTOR_IDLE, tope, mando);
        for (int i = 0; i < 4; i++) escribirPulso(i, mandoAPulso(mando[i]));
    } else if (modo == MOTOR_PRUEBA) {
        for (int i = 0; i < 4; i++) escribirPulso(i, i == motorPrueba ? mandoAPulso(0.12f) : US_PARO);
        if (millis() > finMotorPrueba) { pararMotores(); modo = DESARMADO; Serial0.println("# Fin de la prueba de motor"); }
    } else if (modo == DESARMADO) {
        pararMotores();
    }   // CALIBRA_ESC: la salida la fija el comando 'k'

    // ---------------- Altura (cada 20 ms, sin bloquear) ----------------
    static uint32_t tAltura = 0, tSonar = 0;
    if (millis() - tAltura >= 20) {
        const float dtz = (millis() - tAltura) * 1e-3f;
        tAltura = millis();
        kalman.predecir(accZN ? accZSuma / accZN : 0, dtz > 0.1f ? 0.02f : dtz);
        accZSuma = 0; accZN = 0;
        const float c = cosf(roll * DEG_TO_RAD) * cosf(pitch * DEG_TO_RAD);
        if (tofOK && tof.isRangeComplete()) {
            const float r = tof.readRangeResult() / 1000.0f;
            if (c > 0.94f && r > 0.02f && r < TOF_MAX_M) { alturaTof = r * c - offsetTof; kalman.corregirDistancia(0, alturaTof, 0.02f); }
            else alturaTof = 0;
        }
        if (millis() - tSonar >= SONAR_PERIODO_MS) {
            tSonar = millis();
            const float d = leerSonar();
            if (c > 0.94f && d > 0.05f && d < SONAR_MAX_M) { alturaSonar = d * c - offsetSonar; kalman.corregirDistancia(1, alturaSonar, 0.05f); }
            else alturaSonar = 0;
            dispararSonar();
        }
    }

    // ---------------- Telemetría ----------------
    const uint32_t us = micros() - ahora;
    if (us > lazoMaxUs) lazoMaxUs = us;
    static uint32_t tTel = 0, tVer = 0;
    if (verIMU && millis() - tVer >= 200) {
        tVer = millis();
        Serial0.printf("# acc %.2f %.2f %.2f g | roll %.1f pitch %.1f yaw %.1f | giro p %.1f q %.1f r %.1f\n",
                       accF[0], accF[1], accF[2], roll, pitch, yaw, giroF[0], giroF[1], giroF[2]);
    } else if (!verIMU && telPeriodoMs && millis() - tTel >= telPeriodoMs) {
        tTel = millis();
        telemetria();
    }
}
