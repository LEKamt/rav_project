#ifndef EMERGENCIA_H
#define EMERGENCIA_H

#include "config.h"

enum ModoVuelo {
    MODO_DESARMADO,
    MODO_ARMADO,
    MODO_ATERRIZAJE_EMERGENCIA,
    MODO_EMERGENCIA_DETENIDO
};

// Motivo por el que se abandonó el vuelo normal (se publica en /dron/estado)
enum MotivoFailsafe {
    FAILSAFE_NINGUNO = 0,
    FAILSAFE_BOTON,
    FAILSAFE_TIMEOUT_CMD,
    FAILSAFE_AGENTE,
    FAILSAFE_ROS,
    FAILSAFE_IMU
};

extern volatile ModoVuelo modoVuelo;
extern volatile MotivoFailsafe motivoFailsafe;

// Marca de tiempo del último comando recibido por ROS 2 (la actualiza microros_comm)
extern volatile unsigned long tUltimoComando;

void iniciarEmergencia();
void verificarBotonEmergencia();
void verificarFailsafe(bool agenteConectado);
void ejecutarAterrizajeEmergencia(float dt);

// Transiciones pedidas desde ROS 2
bool armarDron();
void desarmarDron();
void pedirAterrizaje(MotivoFailsafe motivo);

#endif
