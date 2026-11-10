#include "emergencia.h"
#include "sensores.h"
#include "control_vuelo.h"
#include "control_actitud.h"

volatile ModoVuelo modoVuelo = MODO_DESARMADO;
volatile MotivoFailsafe motivoFailsafe = FAILSAFE_NINGUNO;
volatile unsigned long tUltimoComando = 0;

static bool botonPresionadoAntes = false;

void iniciarEmergencia() {
    pinMode(PIN_BOTON_EMERGENCIA, INPUT_PULLUP);
    modoVuelo = MODO_DESARMADO;
    motivoFailsafe = FAILSAFE_NINGUNO;
    tUltimoComando = millis();
}

// ==========================================
bool armarDron() {
    if (modoVuelo != MODO_DESARMADO && modoVuelo != MODO_EMERGENCIA_DETENIDO) return false;
    if (!setpointManual) {
        LOG("[ARMAR] Rechazado: primero fija una altura objetivo\n");
        return false;
    }
    if (digitalRead(PIN_BOTON_EMERGENCIA) == LOW) {
        LOG("[ARMAR] Rechazado: el boton de emergencia esta pulsado\n");
        return false;
    }
    const char *motivo = motivoNoArmar();
    if (motivo != nullptr) {
        LOG("[ARMAR] Rechazado: %s\n", motivo);
        return false;
    }
    resetIntegradoresActitud();
    initControlAltitud();
    motivoFailsafe = FAILSAFE_NINGUNO;
    tUltimoComando = millis();
    modoVuelo = MODO_ARMADO;
    LOG("[ARMAR] Dron armado\n");
    return true;
}

void desarmarDron() {
    modoVuelo = MODO_DESARMADO;
    setpointManual = false;      // obliga a fijar setpoint otra vez antes de rearmar
    motivoFailsafe = FAILSAFE_NINGUNO;
    LOG("[DESARMAR] Dron desarmado\n");
}

void pedirAterrizaje(MotivoFailsafe motivo) {
    // El aterrizaje se hace siempre nivelado: si el enlace cayó con una consigna
    // de inclinación activa, mantenerla haría que el dron se fuera de lado.
    setpointRoll = 0.0f;
    setpointPitch = 0.0f;
    setpointYawRate = 0.0f;

    if (modoVuelo == MODO_ARMADO) {
        motivoFailsafe = motivo;
        modoVuelo = MODO_ATERRIZAJE_EMERGENCIA;
    } else if (modoVuelo != MODO_ATERRIZAJE_EMERGENCIA) {
        // En el suelo cualquier emergencia es simplemente desarmar
        modoVuelo = MODO_EMERGENCIA_DETENIDO;
        motivoFailsafe = motivo;
    }
}

// ==========================================
void verificarBotonEmergencia() {
    bool presionado = (digitalRead(PIN_BOTON_EMERGENCIA) == LOW);
    if (presionado && !botonPresionadoAntes) {
        LOG("[EMERGENCIA] Boton fisico pulsado\n");
        pedirAterrizaje(FAILSAFE_BOTON);
    }
    botonPresionadoAntes = presionado;
}

// ==========================================
// Dos vigilantes independientes: el flujo de comandos (equivale a perder el mando)
// y la sesión con el agente micro-ROS (equivale a perder el enlace).
void verificarFailsafe(bool agenteConectado) {
    if (modoVuelo != MODO_ARMADO) return;

    // Sin IMU no hay estimación de actitud: el control volaría a ciegas
    if (fallosIMUConsecutivos > MAX_FALLOS_IMU) {
        LOG("[FAILSAFE] IMU sin lecturas (%lu ciclos)\n", (unsigned long)fallosIMUConsecutivos);
        pedirAterrizaje(FAILSAFE_IMU);
        return;
    }

    if (!agenteConectado) {
        LOG("[FAILSAFE] Agente micro-ROS perdido\n");
        pedirAterrizaje(FAILSAFE_AGENTE);
        return;
    }
    if (millis() - tUltimoComando > TIMEOUT_CMD_MS) {
        LOG("[FAILSAFE] Sin comandos durante %lu ms\n", millis() - tUltimoComando);
        pedirAterrizaje(FAILSAFE_TIMEOUT_CMD);
    }
}

// ==========================================
// Aterrizaje: baja el setpoint de altura a VELOCIDAD_DESCENSO y corta motores al
// detectar suelo. El control de actitud sigue activo durante todo el descenso.
void ejecutarAterrizajeEmergencia(float dt) {
    static bool inicializado = false;
    static float alturaObjetivoDescenso = 0;

    if (!inicializado) {
        alturaObjetivoDescenso = alturaControl;
        inicializado = true;
    }

    if (usarLiDAR && alturaLiDAR < ALTURA_CORTE_MOTORES) {
        LOG("[ATERRIZAJE] Suelo a %.2f m. Motores cortados\n", alturaLiDAR);
        modoVuelo = MODO_EMERGENCIA_DETENIDO;
        setpointManual = false;   // obliga a fijar altura otra vez antes de rearmar
        inicializado = false;
        return;
    }

    alturaObjetivoDescenso -= VELOCIDAD_DESCENSO * dt;
    if (alturaObjetivoDescenso < 0.0f) alturaObjetivoDescenso = 0.0f;
    alturaObjetivo = alturaObjetivoDescenso;

    // Si el objetivo llega al suelo y el LiDAR no lo ve (p. ej. superficie negra),
    // se corta por tiempo igualmente: el descenso no puede quedarse colgado.
    static unsigned long tEnCero = 0;
    if (alturaObjetivoDescenso <= 0.0f) {
        if (tEnCero == 0) tEnCero = millis();
        if (millis() - tEnCero > 2000) {
            LOG("[ATERRIZAJE] Objetivo en 0 sin deteccion de suelo. Motores cortados\n");
            modoVuelo = MODO_EMERGENCIA_DETENIDO;
            setpointManual = false;
            inicializado = false;
            tEnCero = 0;
        }
    } else {
        tEnCero = 0;
    }

    if (modoVuelo != MODO_ATERRIZAJE_EMERGENCIA) inicializado = false;
}
