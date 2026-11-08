#ifndef IMU_GY91_H
#define IMU_GY91_H

// MPU-9250 / MPU-9255 / MPU-6500 de la GY-91 por I2C, con registros directos.
//
// La librería MPU6500_WE exigía el identificador de la MPU-6500 (0x70) y la GY-91
// de Blaze trae una MPU-9250 (0x71). Muchos GY-91 traen un clon, así que se
// aceptan los tres: el magnetómetro no se usa.

#include <Arduino.h>
#include <Wire.h>

struct LecturaIMU {
    float acc[3];    // g, ejes del sensor
    float giro[3];   // deg/s, ejes del sensor
};

// Configura la IMU. Devuelve el WHO_AM_I leído en *quien (0 si no responde).
bool iniciarIMU(TwoWire &bus, uint8_t direccion, uint8_t dlpfCfg, uint8_t *quien);

// Lee aceleración y giro en una sola transacción de 14 bytes. No toma el mutex:
// lo hace quien la llama.
bool leerIMU(TwoWire &bus, uint8_t direccion, LecturaIMU *l);

const char *nombreIMU(uint8_t quien);

#endif
