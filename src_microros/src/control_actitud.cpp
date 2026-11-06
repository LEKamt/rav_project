#include "control_actitud.h"
#include "sensores.h"
#include "config.h"
#include "smc.h"

volatile float setpointRoll = 0.0f;
volatile float setpointPitch = 0.0f;
volatile float setpointYawRate = 0.0f;

volatile float roll_corr = 0.0f;
volatile float pitch_corr = 0.0f;
volatile float yaw_corr = 0.0f;

volatile float rateSpRoll = 0.0f, rateSpPitch = 0.0f;

// Ganancias (ajustables en caliente por /dron/cmd_ganancias)
static float kpAng   = KP_ANG_ROLL;
static float kpRate  = KP_RATE_ROLL;
static float kiRate  = KI_RATE_ROLL;
static float kdRate  = KD_RATE_ROLL;
static float kpYaw   = KP_RATE_YAW;
static float kiYaw   = KI_RATE_YAW;

// Estado interno de los PID de velocidad angular
static float iRoll = 0, iPitch = 0, iYaw = 0;
static float velRollPrev = 0, velPitchPrev = 0;

// Sesgos estimados por el SMC [N*m]
static float sesgoRoll = 0, sesgoPitch = 0, sesgoYaw = 0;

void initControlActitud() {
    resetIntegradoresActitud();
    roll_corr = pitch_corr = yaw_corr = 0.0f;
}

void resetIntegradoresActitud() {
    iRoll = iPitch = iYaw = 0.0f;
    sesgoRoll = sesgoPitch = sesgoYaw = 0.0f;
    velRollPrev = velRoll;
    velPitchPrev = velPitch;
}

// Un PID de velocidad angular. El término D se calcula sobre la medida (no sobre
// el error) para que un escalón en el setpoint no produzca un pico de derivada.
static float pidVelocidad(float rateSp, float rate, float ratePrev,
                          float kp, float ki, float kd,
                          float *integral, float dt) {
    float error = rateSp - rate;

    float p = kp * error;
    float d = kd * (ratePrev - rate) / dt;

    // Anti-windup condicional: no se integra si la salida ya está saturada
    // y el error empujaría más hacia la saturación.
    float sinI = p + d;
    float candidato = *integral + ki * error * dt;
    candidato = constrain(candidato, -I_MAX_ACTITUD, I_MAX_ACTITUD);
    float salida = sinI + candidato;
    if (salida > CORR_MAX && error > 0) {
        // mantener integral
    } else if (salida < -CORR_MAX && error < 0) {
        // mantener integral
    } else {
        *integral = candidato;
    }

    salida = sinI + *integral;
    return constrain(salida, -CORR_MAX, CORR_MAX);
}

#if LEY_CONTROL == LEY_SMC
// Misma salida que la cascada: correcciones de throttle para la mezcla. Una
// corrección c produce 4 * c * SMC_EMPUJE_MAX de empuje diferencial.
static void actitudSMC(float dt) {
    const float roll = angRoll * DEG_TO_RAD;
    const float pitch = angPitch * DEG_TO_RAD;
    const float p = velRoll * DEG_TO_RAD;
    const float q = velPitch * DEG_TO_RAD;
    const float r = velYaw * DEG_TO_RAD;
    const float spRoll  = constrain(setpointRoll,  -ANG_SP_MAX, ANG_SP_MAX) * DEG_TO_RAD;
    const float spPitch = constrain(setpointPitch, -ANG_SP_MAX, ANG_SP_MAX) * DEG_TO_RAD;
    const float spYaw   = constrain(setpointYawRate, -YAW_RATE_SP_MAX, YAW_RATE_SP_MAX) * DEG_TO_RAD;

    EjeSMC ejeRoll = smcEje(roll, spRoll, p, SMC_IXX, q * r * (SMC_IZZ - SMC_IYY),
                            SMC_LAMBDA_ANG, SMC_KD_ANG, SMC_RHO_ANG, SMC_PHI_ANG,
                            SMC_P_ANG, SMC_SESGO_MAX_ANG, &sesgoRoll, dt);
    EjeSMC ejePitch = smcEje(pitch, spPitch, q, SMC_IYY, p * r * (SMC_IXX - SMC_IZZ),
                             SMC_LAMBDA_ANG, SMC_KD_ANG, SMC_RHO_ANG, SMC_PHI_ANG,
                             SMC_P_ANG, SMC_SESGO_MAX_ANG, &sesgoPitch, dt);
    const float sYaw = r - spYaw;
    sesgoYaw = smcAdaptar(sesgoYaw, sYaw, SMC_PHI_YAW, SMC_P_YAW, SMC_SESGO_MAX_YAW, dt);
    float tauZ = smcLey(p * q * (SMC_IYY - SMC_IXX) + sesgoYaw, sYaw,
                        SMC_KD_YAW, SMC_RHO_YAW, SMC_PHI_YAW);

    const float porCorr = 4.0f * SMC_EMPUJE_MAX;
    roll_corr  = constrain(ejeRoll.tau / (porCorr * SMC_BRAZO), -CORR_MAX, CORR_MAX);
    pitch_corr = constrain(ejePitch.tau / (porCorr * SMC_BRAZO), -CORR_MAX, CORR_MAX);
    yaw_corr   = constrain(tauZ / (porCorr * SMC_K_YAW), -CORR_MAX, CORR_MAX);

    rateSpRoll  = ejeRoll.velRef * RAD_TO_DEG;
    rateSpPitch = ejePitch.velRef * RAD_TO_DEG;
}
#endif

void updateControlActitud(float dt) {
    if (dt <= 0.0f) dt = 1.0f / FREQ_CONTROL_HZ;

#if LEY_CONTROL == LEY_SMC
    actitudSMC(dt);
    return;
#endif

    // --- Lazo externo: ángulo -> velocidad angular deseada ---
    float spRoll  = constrain(setpointRoll,  -ANG_SP_MAX, ANG_SP_MAX);
    float spPitch = constrain(setpointPitch, -ANG_SP_MAX, ANG_SP_MAX);

    rateSpRoll  = constrain(kpAng * (spRoll  - angRoll),  -RATE_SP_MAX, RATE_SP_MAX);
    rateSpPitch = constrain(kpAng * (spPitch - angPitch), -RATE_SP_MAX, RATE_SP_MAX);

    // --- Lazo interno: velocidad angular medida por el giróscopo ---
    roll_corr = pidVelocidad(rateSpRoll, velRoll, velRollPrev,
                             kpRate, kiRate, kdRate, &iRoll, dt);
    pitch_corr = pidVelocidad(rateSpPitch, velPitch, velPitchPrev,
                              kpRate, kiRate, kdRate, &iPitch, dt);

    // Guiñada: control directo de velocidad, sin lazo de ángulo
    float spYawRate = constrain(setpointYawRate, -YAW_RATE_SP_MAX, YAW_RATE_SP_MAX);
    yaw_corr = pidVelocidad(spYawRate, velYaw, velYaw,
                            kpYaw, kiYaw, KD_RATE_YAW, &iYaw, dt);

    velRollPrev = velRoll;
    velPitchPrev = velPitch;
}

void setGananciasActitud(float kp_ang, float kp_rate, float ki_rate, float kd_rate,
                         float kp_yaw, float ki_yaw) {
    kpAng  = kp_ang;
    kpRate = kp_rate;
    kiRate = ki_rate;
    kdRate = kd_rate;
    kpYaw  = kp_yaw;
    kiYaw  = ki_yaw;
    resetIntegradoresActitud();
}

void getGananciasActitud(float *kp_ang, float *kp_rate, float *ki_rate, float *kd_rate,
                         float *kp_yaw, float *ki_yaw) {
    *kp_ang  = kpAng;
    *kp_rate = kpRate;
    *ki_rate = kiRate;
    *kd_rate = kdRate;
    *kp_yaw  = kpYaw;
    *ki_yaw  = kiYaw;
}
