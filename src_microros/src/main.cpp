#include "config.h"
#include "sensores.h"
#include "motores.h"
#include "control_actitud.h"
#include "control_vuelo.h"
#include "emergencia.h"
#include "microros_comm.h"

// Dos tareas con tasa fija, una por núcleo:
//
//   core 1 - tareaControl (FREQ_CONTROL_HZ): IMU, actitud en cascada, mezcla, PWM.
//            Nada bloqueante aquí: el dt del control es el del planificador.
//   core 0 - tareaComunicaciones (FREQ_SENSORES_HZ): barómetro, LiDAR, altura,
//            cascada de altura, failsafe y toda la pila micro-ROS.
//
// Antes todo compartía un único loop() y la lectura bloqueante del LiDAR (~30 ms)
// marcaba el ritmo del lazo de actitud.

static void tareaControl(void *arg) {
    TickType_t ultimoDespertar = xTaskGetTickCount();
    const TickType_t periodo = pdMS_TO_TICKS(PERIODO_CONTROL_MS);
    const float dt = PERIODO_CONTROL_MS / 1000.0f;

    for (;;) {
        actualizarIMU(dt);
        verificarBotonEmergencia();

        switch (modoVuelo) {
            case MODO_ARMADO:
            case MODO_ATERRIZAJE_EMERGENCIA:
                updateControlActitud(dt);
                mezclarMotores();
                aplicarPWM();
                break;

            case MODO_DESARMADO:
            case MODO_EMERGENCIA_DETENIDO:
            default:
                detenerMotores();
                resetIntegradoresActitud();
                break;
        }

        vTaskDelayUntil(&ultimoDespertar, periodo);
    }
}

static void tareaComunicaciones(void *arg) {
    TickType_t ultimoDespertar = xTaskGetTickCount();
    const TickType_t periodo = pdMS_TO_TICKS(PERIODO_SENSORES_MS);
    const float dt = PERIODO_SENSORES_MS / 1000.0f;

    for (;;) {
        actualizarAltitud(dt);

        if (modoVuelo == MODO_ATERRIZAJE_EMERGENCIA) {
            ejecutarAterrizajeEmergencia(dt);
        }
        if (modoVuelo == MODO_ARMADO || modoVuelo == MODO_ATERRIZAJE_EMERGENCIA) {
            controlAltitud(dt);
        }

        verificarFailsafe(agenteConectado());
        tickComunicaciones();

        vTaskDelayUntil(&ultimoDespertar, periodo);
    }
}

void setup() {
    // Configura el transporte antes que nada: con la variante serie, micro-ROS
    // toma Serial y ya no se puede imprimir por ahí.
    iniciarComunicaciones();
    delay(500);

    // Motores primero: los ESC reciben pulso de paro desde el primer instante
    iniciarMotores();
    detenerMotores();

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);

    initControlActitud();
    initControlAltitud();
    iniciarEmergencia();

    if (!iniciarSensores()) {
        // Sin estimación de actitud no se vuela. Motores en paro y sin crear las
        // tareas: el dron no puede armarse ni por error.
        LOG("[FATAL] Fallo en sensores\n");
        for (;;) {
            detenerMotores();
            delay(500);
        }
    }

    delay(3000);   // los ESC confirman con su pitido mientras reciben el pulso de paro

    xTaskCreatePinnedToCore(tareaControl, "control", 8192, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(tareaComunicaciones, "comms", 20480, NULL, 3, NULL, 0);

    LOG("[OK] Tareas creadas. Esperando agente micro-ROS...\n");
}

void loop() {
    // Todo el trabajo vive en las dos tareas. El loopTask de Arduino no hace nada.
    vTaskDelay(pdMS_TO_TICKS(1000));
}
