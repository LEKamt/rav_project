#ifndef MICROROS_COMM_H
#define MICROROS_COMM_H

#include <Arduino.h>

// Capa micro-ROS. Todas las llamadas rcl/rclc ocurren dentro de la tarea de
// comunicaciones (micro-ROS no es seguro entre hilos): la tarea de control nunca
// toca estas funciones, solo comparte escalares volátiles.

void iniciarComunicaciones();   // configura el transporte (serie o wifi)
void tickComunicaciones();      // máquina de estados del agente + spin + publicación
bool agenteConectado();

#endif
