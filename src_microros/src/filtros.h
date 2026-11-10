#ifndef FILTROS_H
#define FILTROS_H

// Filtros del giróscopo: biquads de 2.º orden en forma directa II transpuesta.
// Mismo diseño que blaze_filter de Arquitectura_Blaze (Audio EQ Cookbook, R.
// Bristow-Johnson), para que el F450 y Blaze filtren igual.
//
// Sin dependencias de Arduino: se prueba en el PC
// (robot_tests/unit/test_firmware_estimacion.py). Una frecuencia fuera de
// (0, fs/2) deja el filtro en paso directo: es preferible no filtrar a dejar un
// filtro inestable dentro del lazo.

#include <math.h>

struct Biquad {
    float b0 = 1.0f, b1 = 0.0f, b2 = 0.0f, a1 = 0.0f, a2 = 0.0f;
    float z1 = 0.0f, z2 = 0.0f;

    void pasoDirecto() {
        b0 = 1.0f;
        b1 = b2 = a1 = a2 = 0.0f;
        z1 = z2 = 0.0f;
    }

    static bool frecuenciaValida(float fs, float hz) {
        return hz > 0.0f && hz < 0.5f * fs;      // la comparación también descarta NaN
    }

    // Pasabajos Butterworth (Q = 0.707) con corte fc [Hz]
    void pasabajos(float fs, float fc) {
        if (!frecuenciaValida(fs, fc)) { pasoDirecto(); return; }
        const float w = 2.0f * 3.14159265f * fc / fs;
        const float cw = cosf(w);
        const float alpha = sinf(w) / (2.0f * 0.70710678f);
        const float a0 = 1.0f + alpha;
        b0 = 0.5f * (1.0f - cw) / a0;
        b1 = (1.0f - cw) / a0;
        b2 = b0;
        a1 = -2.0f * cw / a0;
        a2 = (1.0f - alpha) / a0;
        z1 = z2 = 0.0f;
    }

    // Notch centrado en f0 con ancho de banda bw a -3 dB [Hz]
    void notch(float fs, float f0, float bw) {
        if (!frecuenciaValida(fs, f0) || !(bw > 0.0f)) { pasoDirecto(); return; }
        const float w = 2.0f * 3.14159265f * f0 / fs;
        const float cw = cosf(w);
        const float alpha = sinf(w) * bw / (2.0f * f0);   // Q = f0 / bw
        const float a0 = 1.0f + alpha;
        b0 = 1.0f / a0;
        b1 = -2.0f * cw / a0;
        b2 = b0;
        a1 = b1;
        a2 = (1.0f - alpha) / a0;
        z1 = z2 = 0.0f;
    }

    // Estado de régimen para la entrada x: la salida arranca en x sin transitorio
    void reset(float x) {
        const float y = x * (b0 + b1 + b2) / (1.0f + a1 + a2);
        z1 = y - b0 * x;
        z2 = b2 * x - a2 * y;
    }

    float paso(float x) {
        const float y = b0 * x + z1;
        z1 = b1 * x - a1 * y + z2;
        z2 = b2 * x - a2 * y;
        return y;
    }
};

// Cadena de tres ejes: notch opcional y después pasabajos
struct FiltroEjes {
    Biquad notchEje[3];
    Biquad lpf[3];
    bool notchActivo = false;

    void configurar(float fs, float lpfHz, bool conNotch, float notchHz, float notchBw) {
        notchActivo = conNotch;
        for (int i = 0; i < 3; i++) {
            notchEje[i].notch(fs, notchHz, notchBw);
            lpf[i].pasabajos(fs, lpfHz);
        }
    }

    void reset(const float v[3]) {
        for (int i = 0; i < 3; i++) {
            notchEje[i].reset(v[i]);
            lpf[i].reset(v[i]);
        }
    }

    void paso(const float in[3], float out[3]) {
        for (int i = 0; i < 3; i++) {
            const float x = notchActivo ? notchEje[i].paso(in[i]) : in[i];
            out[i] = lpf[i].paso(x);
        }
    }
};

#endif
