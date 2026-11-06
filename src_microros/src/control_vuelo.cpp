#include "control_vuelo.h"
#include "sensores.h"
#include "emergencia.h"
#include "smc.h"

volatile float alturaObjetivo = ALTURA_OBJETIVO_DEF;
volatile bool  setpointManual = false;
volatile float throttleBase = THROTTLE_HOVER;

PID pidPosZ;
PID pidVelZ;

volatile float consignaAltura = 0.0f;
volatile float vzDeseada = 0.0f;
volatile float accDeseada = 0.0f;

static ReferenciaZ referenciaZ;
static float sesgoZ = 0.0f;     // N, estimado por el SMC

void initControlAltitud() {
    // pos_z: el límite de la consigna es la altura máxima aceptada; la salida se
    // acota a la velocidad de ascenso/descenso permitida.
    pidPosZ.configurar(KP_POS_Z, KI_POS_Z, KD_POS_Z, TAU_POS_Z,
                       ALTURA_SP_MAX, -1.0f, VEL_Z_MAX);
    // vel_z: la salida es una aceleración, acotada para no pedir más empuje del
    // que hay. El integrador sí está acotado (ver pid.h).
    pidVelZ.configurar(KP_VEL_Z, KI_VEL_Z, KD_VEL_Z, TAU_VEL_Z,
                       VEL_Z_MAX, I_MAX_VEL_Z, ACC_Z_MAX);

    // El filtro de consigna arranca en la altura actual: así armar no mete un
    // escalón de un metro.
    pidPosZ.inicializar_consigna(alturaControl);
    referenciaZ.reset(alturaControl);
    sesgoZ = 0.0f;
    consignaAltura = alturaControl;
    vzDeseada = accDeseada = 0.0f;
    throttleBase = THROTTLE_HOVER;
}

void controlAltitud(float dt) {
    if (dt <= 0.0f) dt = 1.0f / FREQ_SENSORES_HZ;

#if LEY_CONTROL == LEY_SMC
    referenciaZ.actualizar(constrain(alturaObjetivo, -ALTURA_SP_MAX, ALTURA_SP_MAX),
                           SMC_OMEGA_REF_Z, SMC_ACC_REF_MAX, VEL_Z_MAX, dt);
    consignaAltura = referenciaZ.z;

    float velDeseada, s;
    float fuerza = smcAltura(referenciaZ, alturaControl, velVertical, SMC_MASA, 9.80665f,
                             SMC_LAMBDA_Z, SMC_KD_Z, SMC_RHO_Z, SMC_PHI_Z,
                             SMC_P_Z, SMC_SESGO_MAX_Z, &sesgoZ, dt, &velDeseada, &s);
    float acc = constrain(fuerza / SMC_MASA - 9.80665f, -ACC_Z_MAX, ACC_Z_MAX);
    vzDeseada = velDeseada;
    accDeseada = acc;

    // Inclinado, solo la componente vertical del empuje sostiene al dron
    float c = cosf(angRoll * DEG_TO_RAD) * cosf(angPitch * DEG_TO_RAD);
    if (c < 0.5f) c = 0.5f;
    throttleBase = constrain(THROTTLE_HOVER * (1.0f + acc / 9.80665f) / c, 0.0f, 1.0f);
#else
    // Lazo externo: error de altura -> velocidad vertical deseada
    float vz = pidPosZ.actualizar(alturaObjetivo, alturaControl, velVertical, dt);

    // Lazo interno: error de velocidad -> aceleración vertical deseada
    float acc = pidVelZ.actualizar(vz, velVertical, aceleracionVertical, dt);

    consignaAltura = pidPosZ.consigna;
    vzDeseada = vz;
    accDeseada = acc;

    // Equivalente normalizado de force = masa * (pid + g)
    throttleBase = constrain(THROTTLE_HOVER * (1.0f + acc / 9.80665f), 0.0f, 1.0f);
#endif
}
