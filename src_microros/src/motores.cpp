#include "motores.h"
#include "control_vuelo.h"
#include "control_actitud.h"
#include "emergencia.h"
#include "config.h"

volatile uint16_t pwmMotor1 = 0, pwmMotor2 = 0, pwmMotor3 = 0, pwmMotor4 = 0;
volatile float throttleAplicado = 0.0f;

static bool rampaActiva = false;
static unsigned long tiempoInicioRampa = 0;

// ==========================================
void iniciarMotores() {
    for (int ch = 0; ch < 4; ch++) {
        ledcSetup(ch, PWM_FREQ, PWM_RESOLUTION);
    }
    ledcAttachPin(MOTOR1_PIN, 0);
    ledcAttachPin(MOTOR2_PIN, 1);
    ledcAttachPin(MOTOR3_PIN, 2);
    ledcAttachPin(MOTOR4_PIN, 3);

    LOG("[Motores] PWM %d Hz, %d bits (paro=%u, max=%u)\n",
        PWM_FREQ, PWM_RESOLUTION, (unsigned)PWM_MIN, (unsigned)PWM_MAX);
}

// ==========================================
void mezclarMotores() {
    // Rampa de arranque al armar
    static bool estabaArmado = false;
    bool armado = (modoVuelo == MODO_ARMADO);
    if (armado && !estabaArmado) {
        rampaActiva = true;
        tiempoInicioRampa = millis();
    }
    estabaArmado = armado;

    float throttle = throttleBase;
    if (rampaActiva) {
        unsigned long transcurrido = millis() - tiempoInicioRampa;
        if (transcurrido >= RAMP_TIME_MS) {
            rampaActiva = false;
        } else {
            throttle *= (float)transcurrido / RAMP_TIME_MS;
        }
    }

    // Mezcla en configuración X
    // Motor 1 (frente izquierdo): +roll -pitch -yaw
    // Motor 2 (frente derecho):   -roll -pitch +yaw
    // Motor 3 (trasero derecho):  -roll +pitch -yaw
    // Motor 4 (trasero izquierdo):+roll +pitch +yaw
    float m[4];
    m[0] = throttle + roll_corr - pitch_corr - yaw_corr;
    m[1] = throttle - roll_corr - pitch_corr + yaw_corr;
    m[2] = throttle - roll_corr + pitch_corr - yaw_corr;
    m[3] = throttle + roll_corr + pitch_corr + yaw_corr;

    // Desaturación: si algún motor se sale del rango se desplaza el throttle común
    // de los cuatro en vez de recortar uno solo. Recortar destruye el par
    // diferencial justo cuando el control más lo necesita.
    float mx = m[0], mn = m[0];
    for (int i = 1; i < 4; i++) {
        if (m[i] > mx) mx = m[i];
        if (m[i] < mn) mn = m[i];
    }
    float desplazamiento = 0.0f;
    if (mx > 1.0f)            desplazamiento = 1.0f - mx;
    else if (mn < MOTOR_IDLE) desplazamiento = MOTOR_IDLE - mn;

    float suma = 0.0f;
    for (int i = 0; i < 4; i++) {
        m[i] = constrain(m[i] + desplazamiento, MOTOR_IDLE, 1.0f);
        suma += m[i];
    }
    throttleAplicado = suma / 4.0f;

    const uint16_t span = PWM_MAX - PWM_MIN;
    pwmMotor1 = PWM_MIN + (uint16_t)(m[0] * span);
    pwmMotor2 = PWM_MIN + (uint16_t)(m[1] * span);
    pwmMotor3 = PWM_MIN + (uint16_t)(m[2] * span);
    pwmMotor4 = PWM_MIN + (uint16_t)(m[3] * span);
}

// ==========================================
void aplicarPWM() {
    ledcWrite(0, pwmMotor1);
    ledcWrite(1, pwmMotor2);
    ledcWrite(2, pwmMotor3);
    ledcWrite(3, pwmMotor4);
}

// ==========================================
void detenerMotores() {
    rampaActiva = false;
    throttleAplicado = 0.0f;
    pwmMotor1 = pwmMotor2 = pwmMotor3 = pwmMotor4 = PWM_MIN;
    aplicarPWM();
}
