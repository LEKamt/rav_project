#include "sensores.h"
#include <Adafruit_BMP280.h>
#include <Adafruit_VL53L0X.h>
#include "imu_gy91.h"
#include "sonar.h"
#include "filtros.h"
#include "ahrs.h"
#include "kalman_altura.h"

static Adafruit_BMP280 bmp(&Wire);
static Adafruit_VL53L0X vl53;

volatile float angRoll = 0, angPitch = 0, angYaw = 0;
volatile float velRoll = 0, velPitch = 0, velYaw = 0;
volatile float ax_g = 0, ay_g = 0, az_g = 1.0f;
volatile float alturaControl = 0, alturaLiDAR = 0, alturaSonar = 0, alturaBaro = 0;
volatile float sesgoBaro = 0;
volatile float velVertical = 0;
volatile float aceleracionVertical = 0;
volatile bool  usarLiDAR = false;
volatile bool  imuOK = false;
volatile bool  giroCalibrado = false;
volatile uint32_t fallosIMUConsecutivos = 0;
volatile uint32_t fallosIMUTotal = 0;
volatile uint32_t tiempoIMUMaxUs = 0;

SemaphoreHandle_t mutexI2C = NULL;
EstadoSensores estadoSensores = {0, false, false, false, false};

static float sesgoGiro[3] = {0, 0, 0};      // deg/s, ejes del cuerpo
static FiltroEjes filtroGiro, filtroAcc;
static Mahony ahrs;
static KalmanAltura kalman;
static float giroUltimo[3] = {0, 0, 0};    // rad/s, para predecir si falla una lectura

// Aceleración vertical acumulada por la tarea rápida y consumida por la lenta:
// el Kalman predice con el promedio del periodo, no con una muestra suelta.
static portMUX_TYPE muxAcc = portMUX_INITIALIZER_UNLOCKED;
static float sumaAccZ = 0.0f;
static uint32_t nAccZ = 0;

static float presionCero = 0.0f;    // Pa, al arrancar
static float offsetTof = 0.0f;      // m, lo que mide el ToF con el dron en el suelo
static float offsetSonar = 0.0f;
static float temperaturaC = 20.0f;

// ==========================================
// Lectura en ejes del cuerpo, con la calibración del acelerómetro aplicada
static bool leerCuerpo(float acc[3], float giro[3]) {
    LecturaIMU l;
    if (!leerIMU(Wire, MPU_ADDR, &l)) return false;
    acc[0] = (IMU_CUERPO_X(l.acc) - ACC_OFFSET_X) * ACC_ESCALA_X;
    acc[1] = (IMU_CUERPO_Y(l.acc) - ACC_OFFSET_Y) * ACC_ESCALA_Y;
    acc[2] = (IMU_CUERPO_Z(l.acc) - ACC_OFFSET_Z) * ACC_ESCALA_Z;
    giro[0] = IMU_CUERPO_X(l.giro);
    giro[1] = IMU_CUERPO_Y(l.giro);
    giro[2] = IMU_CUERPO_Z(l.giro);
    return true;
}

// Promedia el giro con el dron quieto. Si se movió, lo repite.
static bool calibrarGiro() {
    const int n = GIRO_CALIB_MS / PERIODO_CONTROL_MS;
    for (int intento = 0; intento < GIRO_CALIB_INTENTOS; intento++) {
        double suma[3] = {0, 0, 0}, suma2[3] = {0, 0, 0};
        int validas = 0;
        for (int i = 0; i < n; i++) {
            float acc[3], giro[3];
            if (leerCuerpo(acc, giro)) {
                for (int k = 0; k < 3; k++) {
                    suma[k] += giro[k];
                    suma2[k] += (double)giro[k] * giro[k];
                }
                validas++;
            }
            delay(PERIODO_CONTROL_MS);
        }
        if (validas < n / 2) continue;
        bool quieto = true;
        for (int k = 0; k < 3; k++) {
            const double media = suma[k] / validas;
            const double var = suma2[k] / validas - media * media;
            if (var > GIRO_CALIB_QUIETO * GIRO_CALIB_QUIETO) quieto = false;
            sesgoGiro[k] = (float)media;
        }
        if (quieto) return true;
        LOG("[IMU] Se movió durante la calibración del giro, repitiendo\n");
    }
    return false;   // queda el último promedio, pero no se deja armar
}

static float alturaPresion(float pa) {
    // Fórmula barométrica internacional, relativa a la presión del arranque
    return 44330.0f * (1.0f - powf(pa / presionCero, 0.1903f));
}

// ==========================================
bool iniciarSensores() {
    mutexI2C = xSemaphoreCreateMutex();
    if (mutexI2C == NULL) return false;

    // --- IMU (imprescindible) ---
    uint8_t quien = 0;
    estadoSensores.imu = iniciarIMU(Wire, MPU_ADDR, MPU_DLPF_CFG, &quien);
    estadoSensores.whoAmI = quien;
    if (!estadoSensores.imu) {
        LOG("[ERROR] IMU no detectada (WHO_AM_I = 0x%02X)\n", quien);
        return false;
    }
    LOG("[OK] %s (0x%02X)\n", nombreIMU(quien), quien);

    giroCalibrado = calibrarGiro();
    LOG("[IMU] Sesgo del giro %.2f %.2f %.2f deg/s%s\n", sesgoGiro[0], sesgoGiro[1],
        sesgoGiro[2], giroCalibrado ? "" : " (SIN CALIBRAR: no se podrá armar)");

    const float fs = (float)FREQ_CONTROL_HZ;
    filtroGiro.configurar(fs, GYRO_LPF_HZ, GYRO_NOTCH_ON, GYRO_NOTCH_HZ, GYRO_NOTCH_BW);
    filtroAcc.configurar(fs, ACC_LPF_HZ, false, 0.0f, 0.0f);
    ahrs.configurar(AHRS_KP, AHRS_KI, AHRS_RECHAZO_G);

    float acc[3], giro[3];
    if (leerCuerpo(acc, giro)) {
        const float cero[3] = {0, 0, 0};
        filtroAcc.reset(acc);
        filtroGiro.reset(cero);
        ahrs.iniciarConAcc(acc);
    }

    // --- Barómetro (opcional) ---
    estadoSensores.baro = bmp.begin(BMP280_ADDR_1, BMP280_CHIPID) ||
                          bmp.begin(BMP280_ADDR_2, BMP280_CHIPID);
    if (estadoSensores.baro) {
        // Sin el IIR fuerte del sensor: el Kalman ya filtra y el retardo le estorba
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2,
                        Adafruit_BMP280::SAMPLING_X8, Adafruit_BMP280::FILTER_X4,
                        Adafruit_BMP280::STANDBY_MS_1);
        delay(100);
        double suma = 0;
        const int nb = 50;
        for (int i = 0; i < nb; i++) {
            suma += bmp.readPressure();
            delay(20);
        }
        presionCero = (float)(suma / nb);
        temperaturaC = bmp.readTemperature();
        LOG("[OK] BMP280: %.1f Pa, %.1f C\n", presionCero, temperaturaC);
    } else {
        LOG("[AVISO] BMP280 no detectado: la altura queda solo con ToF y sonar\n");
    }

    // --- ToF (opcional) ---
    estadoSensores.tof = vl53.begin(VL53L0X_ADDR, false, &Wire);
    if (estadoSensores.tof) {
        vl53.startRangeContinuous(PERIODO_SENSORES_MS);
        // Lo que mide en el suelo es la altura del sensor: se resta para que el
        // suelo sea z = 0 (el aterrizaje corta a ALTURA_CORTE_MOTORES).
        float suma = 0;
        int n = 0;
        for (int i = 0; i < 20; i++) {
            delay(PERIODO_SENSORES_MS + 5);
            if (vl53.isRangeComplete()) {
                const uint16_t mm = vl53.readRangeResult();
                if (mm > 10 && mm < 2000) { suma += mm / 1000.0f; n++; }
            }
        }
        offsetTof = n ? suma / n : 0.0f;
        LOG("[OK] VL53L0X en modo continuo, en el suelo mide %.3f m\n", offsetTof);
    } else {
        LOG("[AVISO] VL53L0X no detectado\n");
    }

    // --- Ultrasonido (opcional: se sabe si responde al medir) ---
    iniciarSonar(PIN_SONAR_TRIG, PIN_SONAR_ECHO);
    {
        float suma = 0, d;
        int n = 0;
        for (int i = 0; i < 10; i++) {
            dispararSonar();
            delay(SONAR_PERIODO_MS);
            if (leerSonar(temperaturaC, &d) && d < 1.0f) { suma += d; n++; }
        }
        estadoSensores.sonar = n >= 5;
        offsetSonar = n ? suma / n : 0.0f;
        LOG(estadoSensores.sonar ? "[OK] Ultrasonido, en el suelo mide %.3f m\n"
                                 : "[AVISO] Ultrasonido sin eco (%.3f)\n", offsetSonar);
    }

    kalman.sigmaAcc = KF_SIGMA_ACC;
    kalman.sigmaSesgo = KF_SIGMA_SESGO;
    kalman.compuerta = KF_COMPUERTA;
    kalman.maxRechazos = KF_MAX_RECHAZOS;
    kalman.reset(0.0f, 0.0f);
    return true;
}

// ==========================================
// Tarea rápida: IMU + filtros + AHRS.
// Si el bus I2C está ocupado no espera: el AHRS se propaga con el último giro y
// el lazo conserva su cadencia, que es lo que el control necesita.
void actualizarIMU(float dt) {
    const uint32_t t0 = micros();
    float acc[3], giro[3];
    bool leido = false;

    if (xSemaphoreTake(mutexI2C, pdMS_TO_TICKS(1)) == pdTRUE) {
        leido = leerCuerpo(acc, giro);
        xSemaphoreGive(mutexI2C);
    }

    imuOK = leido;
    if (!leido) {
        if (fallosIMUConsecutivos < 0xFFFFFFFF) fallosIMUConsecutivos++;
        fallosIMUTotal++;
        const float sinAcc[3] = {0, 0, 0};     // |a| = 0: el AHRS no corrige
        ahrs.actualizar(giroUltimo, sinAcc, dt);
    } else {
        fallosIMUConsecutivos = 0;

        for (int k = 0; k < 3; k++) giro[k] -= sesgoGiro[k];

        // Giro filtrado para el lazo de velocidad angular (P y D)
        float giroF[3];
        filtroGiro.paso(giro, giroF);
        velRoll = giroF[0];
        velPitch = giroF[1];
        velYaw = giroF[2];

        float accF[3];
        filtroAcc.paso(acc, accF);
        ax_g = accF[0];
        ay_g = accF[1];
        az_g = accF[2];

        // El AHRS integra el giro sin filtrar: la integral ya es un pasabajos y
        // así no hereda el retardo del biquad.
        for (int k = 0; k < 3; k++) giroUltimo[k] = giro[k] * DEG_TO_RAD;
        ahrs.actualizar(giroUltimo, accF, dt);

        const float az = ahrs.aceleracionVertical(accF, 9.80665f);
        // El PID de velocidad vertical la usa como término D: va más filtrada
        const float alphaA = dt / (dt + 1.0f / (2.0f * PI * ACC_Z_LPF_HZ));
        aceleracionVertical = aceleracionVertical + alphaA * (az - aceleracionVertical);
        portENTER_CRITICAL(&muxAcc);
        sumaAccZ += az;
        nAccZ++;
        portEXIT_CRITICAL(&muxAcc);
    }

    float r, p, y;
    ahrs.euler(&r, &p, &y);
    angRoll = r * RAD_TO_DEG;
    angPitch = p * RAD_TO_DEG;
    angYaw = y * RAD_TO_DEG;

    const uint32_t us = micros() - t0;
    if (us > tiempoIMUMaxUs) tiempoIMUMaxUs = us;
}

// ==========================================
// Tarea lenta: ToF + sonar + baro + Kalman
void actualizarAltitud(float dt) {
    // --- Predicción con la aceleración media del periodo ---
    float acc = 0.0f;
    portENTER_CRITICAL(&muxAcc);
    if (nAccZ) acc = sumaAccZ / nAccZ;
    sumaAccZ = 0.0f;
    nAccZ = 0;
    portEXIT_CRITICAL(&muxAcc);
    kalman.predecir(acc, dt);

    // Inclinado, el ToF y el sonar miden la hipotenusa. Se calcula con los ángulos
    // publicados y no con ahrs.q, que la otra tarea escribe a la vez.
    const float c = cosf(angRoll * DEG_TO_RAD) * cosf(angPitch * DEG_TO_RAD);
    const bool proyectable = c > COS_INCL_DISTANCIA;

    // --- ToF (no bloqueante: solo lee si hay medida lista) ---
    if (estadoSensores.tof && xSemaphoreTake(mutexI2C, pdMS_TO_TICKS(5)) == pdTRUE) {
        const bool listo = vl53.isRangeComplete();
        const uint16_t mm = listo ? vl53.readRangeResult() : 0;
        xSemaphoreGive(mutexI2C);
        if (listo) {
            const float r = mm / 1000.0f;      // fuera de rango devuelve 8190/8191
            if (proyectable && r > TOF_MIN_M && r < TOF_MAX_M) {
                alturaLiDAR = r * c - offsetTof;
                usarLiDAR = true;
                kalman.corregirDistancia(0, alturaLiDAR, SIGMA_TOF);
            } else {
                alturaLiDAR = 0.0f;
                usarLiDAR = false;
            }
        }
    }

    // --- Ultrasonido: lee el eco del disparo anterior y lanza el siguiente ---
    static uint32_t tSonar = 0;
    if (millis() - tSonar >= SONAR_PERIODO_MS) {
        tSonar = millis();
        float d;
        if (leerSonar(temperaturaC, &d) && proyectable && d > SONAR_MIN_M && d < SONAR_MAX_M) {
            alturaSonar = d * c - offsetSonar;
            kalman.corregirDistancia(1, alturaSonar, SIGMA_SONAR);
        } else {
            alturaSonar = 0.0f;
        }
        dispararSonar();
    }

    // --- Barómetro ---
    static uint32_t tBaro = 0;
    if (estadoSensores.baro && millis() - tBaro >= BARO_PERIODO_MS) {
        tBaro = millis();
        if (xSemaphoreTake(mutexI2C, pdMS_TO_TICKS(5)) == pdTRUE) {
            const float pa = bmp.readPressure();
            xSemaphoreGive(mutexI2C);
            if (pa > 30000.0f && pa < 120000.0f) {
                alturaBaro = alturaPresion(pa);
                kalman.corregirBaro(alturaBaro, SIGMA_BARO);
            }
        }
    }

    alturaControl = kalman.z;
    velVertical = kalman.vz;
    sesgoBaro = kalman.b;
}

// ==========================================
const char *motivoNoArmar() {
    if (!imuOK) return "IMU sin lecturas";
    if (!giroCalibrado) return "giro sin calibrar (se movio al arrancar)";
    // Si los ejes del cuerpo están mal configurados, la gravedad no cae en +z y
    // el control empujaría hacia el lado equivocado.
    const float ex = ax_g, ey = ay_g, ez = az_g - 1.0f;
    if (sqrtf(ex * ex + ey * ey + ez * ez) > ACC_ARMADO_ERR_G)
        return "acelerometro no marca (0,0,1) g: revisar IMU_CUERPO_* o nivelar";
    if (fabsf(angRoll) > INCLINACION_ARMADO || fabsf(angPitch) > INCLINACION_ARMADO)
        return "dron inclinado";
    return nullptr;
}
