#include "sensores.h"

MPU6500_WE mpu(MPU_ADDR);
MS5611 ms5611(MS5611_ADDR);
Adafruit_VL53L0X vl53 = Adafruit_VL53L0X();

volatile float angRoll = 0, angPitch = 0, angYaw = 0;
volatile float velRoll = 0, velPitch = 0, velYaw = 0;
volatile float ax_g = 0, ay_g = 0, az_g = 0;
volatile float alturaControl = 0, alturaLiDAR = 0, alturaBaro = 0;
volatile float velVertical = 0;
volatile float aceleracionVertical = 0;
volatile bool  usarLiDAR = false;
volatile bool  imuOK = false;
volatile uint32_t fallosIMUConsecutivos = 0;

SemaphoreHandle_t mutexI2C = NULL;

static float baroOffset = 0;

// ==========================================
bool iniciarSensores() {
    mutexI2C = xSemaphoreCreateMutex();
    if (mutexI2C == NULL) return false;

    // --- MPU-6500 ---
    if (!mpu.init()) {
        LOG("[ERROR] MPU-6500 no detectado\n");
        return false;
    }
    mpu.autoOffsets();                              // el dron debe estar quieto y nivelado
    mpu.setAccRange(MPU6500_ACC_RANGE_8G);
    mpu.setGyrRange(MPU6500_GYRO_RANGE_500);
    mpu.setGyrDLPF(MPU6500_DLPF_3);                 // ~41 Hz en hardware
    mpu.setSampleRateDivider(0);                    // 1 kHz interno: no limita el lazo
    LOG("[OK] MPU-6500 inicializado\n");

    // --- MS5611 ---
    if (!ms5611.begin()) {
        LOG("[ERROR] MS5611 no detectado\n");
        return false;
    }
    // OSR bajo (~2 ms por conversión): el barómetro comparte el bus I2C con la IMU
    // y una conversión larga retrasaría el lazo de actitud.
    ms5611.setOversampling(OSR_LOW);

    const int nCalib = 50;
    float sumBaro = 0;
    for (int i = 0; i < nCalib; i++) {
        ms5611.read();
        sumBaro += ms5611.getAltitude();
        delay(10);
    }
    baroOffset = sumBaro / nCalib;
    LOG("[OK] MS5611 calibrado. Offset: %.2f m\n", baroOffset);

    // --- VL53L0X en modo continuo: readRange() bloqueaba ~30 ms por llamada ---
    if (!vl53.begin(VL53L0X_ADDR, false, &Wire)) {
        LOG("[ERROR] VL53L0X no detectado\n");
        return false;
    }
    vl53.startRangeContinuous(PERIODO_SENSORES_MS);
    LOG("[OK] VL53L0X en modo continuo\n");

    return true;
}

// ==========================================
// Tarea rápida: IMU + filtro complementario.
// Si el bus I2C está ocupado no espera: propaga el estado con la última velocidad
// angular conocida y conserva la cadencia del lazo, que es lo que el control
// necesita. Bloquear aquí metería jitter directamente en el PWM.
void actualizarIMU(float dt) {
    bool leido = false;
    xyzFloat acc, gyr;

    if (xSemaphoreTake(mutexI2C, pdMS_TO_TICKS(2)) == pdTRUE) {
        acc = mpu.getGValues();
        gyr = mpu.getGyrValues();
        xSemaphoreGive(mutexI2C);
        leido = true;
    }

    imuOK = leido;
    if (!leido) {
        if (fallosIMUConsecutivos < 0xFFFFFFFF) fallosIMUConsecutivos++;
        // Predicción pura: se integra la última velocidad angular filtrada
        angRoll  += velRoll * dt;
        angPitch += velPitch * dt;
        angYaw   += velYaw * dt;
        return;
    }
    fallosIMUConsecutivos = 0;

    ax_g = acc.x;
    ay_g = acc.y;
    az_g = acc.z;

    // Filtro pasa-bajos del giróscopo (el término D del lazo interno lo usa)
    const float alphaG = dt / (dt + 1.0f / (2.0f * PI * GYRO_LPF_HZ));
    velRoll  += alphaG * (gyr.x - velRoll);
    velPitch += alphaG * (gyr.y - velPitch);
    velYaw   += alphaG * (gyr.z - velYaw);

    float accRoll  = atan2(acc.y, acc.z) * 180.0f / PI;
    float accPitch = atan2(-acc.x, sqrt(acc.y * acc.y + acc.z * acc.z)) * 180.0f / PI;

    // alpha derivado de tau: así cambiar FREQ_CONTROL_HZ no altera el filtro
    const float alphaC = FILTRO_COMP_TAU / (FILTRO_COMP_TAU + dt);
    angRoll  = alphaC * (angRoll  + gyr.x * dt) + (1.0f - alphaC) * accRoll;
    angPitch = alphaC * (angPitch + gyr.y * dt) + (1.0f - alphaC) * accPitch;

    // Aceleración vertical para el término D del PID de velocidad: se proyecta la
    // medida del acelerómetro sobre la vertical y se le quita la gravedad.
    float azMundo = (az_g * cosf(angRoll * DEG_TO_RAD) * cosf(angPitch * DEG_TO_RAD) - 1.0f)
                    * 9.80665f;
    const float alphaA = dt / (dt + 1.0f / (2.0f * PI * ACC_Z_LPF_HZ));
    aceleracionVertical += alphaA * (azMundo - aceleracionVertical);

    // Sin magnetómetro (el MPU-6500 no lo tiene) el yaw deriva sin cota. Se publica
    // como referencia, pero el control de guiñada trabaja sobre velocidad, no ángulo.
    angYaw += gyr.z * dt;
    if (angYaw > 180.0f)  angYaw -= 360.0f;
    if (angYaw < -180.0f) angYaw += 360.0f;
}

// ==========================================
// Tarea lenta: barómetro + LiDAR + fusión + velocidad vertical
void actualizarAltitud(float dt) {
    // --- LiDAR (no bloqueante: solo lee si hay medida lista) ---
    if (xSemaphoreTake(mutexI2C, pdMS_TO_TICKS(5)) == pdTRUE) {
        bool listo = vl53.isRangeComplete();
        uint16_t mm = listo ? vl53.readRangeResult() : 0;
        xSemaphoreGive(mutexI2C);

        if (listo) {
            if (mm > 20 && mm < 3500) {      // fuera de rango devuelve 8190/8191
                alturaLiDAR = mm / 1000.0f;
                usarLiDAR = true;
            } else {
                usarLiDAR = false;
            }
        }
    }

    // --- Barómetro a 10 Hz ---
    // Cada read() son dos conversiones ADC: mantener el mutex todo ese tiempo a
    // 50 Hz dejaba al lazo de actitud sin lecturas de IMU en buena parte de los
    // ciclos. A 10 Hz sobra, porque la altura va filtrada y por debajo de
    // ALTURA_LIDAR_MAX manda el LiDAR.
    static uint8_t divisorBaro = (FREQ_SENSORES_HZ / 10);   // la primera llamada ya lee
    static bool primeraMuestra = true;
    static float alturaBaroFiltrada = 0;

    if (++divisorBaro >= (FREQ_SENSORES_HZ / 10)) {
        divisorBaro = 0;
        if (xSemaphoreTake(mutexI2C, pdMS_TO_TICKS(15)) == pdTRUE) {
            ms5611.read();
            float baroCrudo = ms5611.getAltitude() - baroOffset;
            xSemaphoreGive(mutexI2C);

            if (primeraMuestra) {
                alturaBaroFiltrada = baroCrudo;
                primeraMuestra = false;
            }
            alturaBaroFiltrada += 0.3f * (baroCrudo - alturaBaroFiltrada);
            alturaBaro = alturaBaroFiltrada;
        }
    }

    // --- Anclaje del barómetro al LiDAR ---
    // Mientras el LiDAR es la referencia buena se estima la deriva del barómetro.
    // Sin esto, al salir del alcance del LiDAR la altura salta de golpe a la del
    // baro (decenas de cm de deriva) y ese escalón entra como un pico en la
    // derivada que alimenta el PID de velocidad.
    static float correccionBaro = 0.0f;
    if (usarLiDAR && alturaLiDAR < ALTURA_MEZCLA_MIN && !primeraMuestra) {
        correccionBaro += 0.05f * ((alturaLiDAR - alturaBaro) - correccionBaro);
    }
    float baroCorregido = alturaBaro + correccionBaro;

    // --- Fusión: LiDAR cerca del suelo, baro arriba, mezcla lineal en medio ---
    float altura;
    if (usarLiDAR && alturaLiDAR < ALTURA_LIDAR_MAX) {
        if (alturaLiDAR > ALTURA_MEZCLA_MIN) {
            float k = (alturaLiDAR - ALTURA_MEZCLA_MIN) / (ALTURA_LIDAR_MAX - ALTURA_MEZCLA_MIN);
            altura = (1.0f - k) * alturaLiDAR + k * baroCorregido;
        } else {
            altura = alturaLiDAR;
        }
    } else {
        altura = baroCorregido;
    }

    // --- Velocidad vertical: filtro complementario IMU + altura ---
    // La aceleración vertical de la IMU predice y la derivada de la altura
    // corrige. Mismo principio que el filtro de actitud y por el mismo motivo:
    // la derivada de la altura es ruidosa a corto plazo (el barómetro tiene
    // ~8 cm de ruido) y la integral de la aceleración deriva a largo.
    static float alturaPrev = 0;
    static bool  vzInit = false;
    if (!vzInit) {
        alturaPrev = altura;
        vzInit = true;
    }
    if (dt > 0.0f) {
        float vzMedida = constrain((altura - alturaPrev) / dt,
                                   -VZ_MAX_CRUDA, VZ_MAX_CRUDA);
        velVertical += aceleracionVertical * dt;
        velVertical += K_VZ * (vzMedida - velVertical) * dt;
        velVertical = constrain(velVertical, -VZ_MAX_CRUDA, VZ_MAX_CRUDA);
    }
    alturaPrev = altura;

    alturaControl = altura;
}
