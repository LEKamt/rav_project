#ifndef KALMAN_ALTURA_H
#define KALMAN_ALTURA_H

// Filtro de Kalman de la altura. Fusiona la aceleración vertical de la IMU con
// tres sensores que se complementan:
//
//   ToF VL53L0X      preciso (~1 cm), pero solo hasta ~1 m
//   ultrasonido      hasta ~4 m, más ruidoso y con ecos falsos
//   barómetro        sin límite de altura, pero con deriva (~decenas de cm)
//
// Estado x = [z, vz, b]: altura [m], velocidad vertical [m/s] y sesgo del
// barómetro [m]. El barómetro mide z + b; los sensores de distancia miden z.
// Mientras hay un sensor de distancia válido el sesgo es observable, y al salir
// de su alcance el barómetro sigue sin el salto que daba la mezcla lineal.
//
// Cada medida pasa por una compuerta de innovación (|nu| < k * sqrt(S)): un eco
// falso del ultrasonido no entra al estado. Si un sensor de distancia acumula
// demasiados rechazos seguidos, el filtro asume que el que se perdió es él y se
// re-ancla a esa medida (por ejemplo, después de mucho tiempo solo con el baro).
//
// Sin dependencias de Arduino: se prueba en el PC.

#include <math.h>

struct KalmanAltura {
    // Estado
    float z = 0.0f, vz = 0.0f, b = 0.0f;
    // Covarianza (simétrica): P[i][j]
    float P[3][3] = {{0.01f, 0, 0}, {0, 0.01f, 0}, {0, 0, 1.0f}};

    // Ruidos
    float sigmaAcc = 0.5f;      // m/s^2, ruido de la aceleración de entrada
    float sigmaSesgo = 0.02f;   // m/sqrt(s), paseo aleatorio del sesgo del baro
    float compuerta = 3.0f;     // en desviaciones estándar
    int maxRechazos = 15;       // rechazos seguidos de un sensor antes de re-anclar

    // Diagnóstico: rechazos seguidos por sensor [0 ToF, 1 sonar, 2 baro]
    int rechazos[3] = {0, 0, 0};

    void reset(float altura, float baro) {
        z = altura;
        vz = 0.0f;
        b = baro - altura;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) P[i][j] = 0.0f;
        P[0][0] = 0.01f;
        P[1][1] = 0.01f;
        P[2][2] = 0.04f;
        rechazos[0] = rechazos[1] = rechazos[2] = 0;
    }

    // Predicción con la aceleración vertical sin gravedad [m/s^2, + arriba]
    void predecir(float acc, float dt) {
        if (dt <= 0.0f) return;
        z += vz * dt + 0.5f * acc * dt * dt;
        vz += acc * dt;

        // P = F P F^T + Q, con F = [[1, dt, 0], [0, 1, 0], [0, 0, 1]]
        const float p00 = P[0][0], p01 = P[0][1], p02 = P[0][2];
        const float p11 = P[1][1], p12 = P[1][2], p22 = P[2][2];
        float n00 = p00 + 2.0f * dt * p01 + dt * dt * p11;
        float n01 = p01 + dt * p11;
        float n02 = p02 + dt * p12;
        float n11 = p11;
        float n12 = p12;
        float n22 = p22;

        // Q de una aceleración blanca: G = [dt^2/2, dt]
        const float qa = sigmaAcc * sigmaAcc;
        n00 += 0.25f * dt * dt * dt * dt * qa;
        n01 += 0.5f * dt * dt * dt * qa;
        n11 += dt * dt * qa;
        n22 += sigmaSesgo * sigmaSesgo * dt;

        P[0][0] = n00; P[0][1] = P[1][0] = n01; P[0][2] = P[2][0] = n02;
        P[1][1] = n11; P[1][2] = P[2][1] = n12; P[2][2] = n22;
    }

    // Medida de distancia al suelo ya proyectada en la vertical (H = [1, 0, 0]).
    // sensor: 0 ToF, 1 sonar. Devuelve true si la medida entró al estado.
    bool corregirDistancia(int sensor, float medida, float sigma) {
        const float nu = medida - z;
        const float S = P[0][0] + sigma * sigma;
        if (nu * nu > compuerta * compuerta * S) {
            if (++rechazos[sensor] < maxRechazos) return false;
            // El sensor insiste: el que se perdió es el filtro
            reanclar(medida);
            rechazos[sensor] = 0;
            return true;
        }
        rechazos[sensor] = 0;
        const float k0 = P[0][0] / S, k1 = P[1][0] / S, k2 = P[2][0] / S;
        aplicar(nu, k0, k1, k2, 0);
        return true;
    }

    // Barómetro relativo al arranque (H = [1, 0, 1])
    bool corregirBaro(float medida, float sigma) {
        const float nu = medida - (z + b);
        const float S = P[0][0] + 2.0f * P[0][2] + P[2][2] + sigma * sigma;
        if (nu * nu > compuerta * compuerta * S) {
            // El baro no puede re-anclar la altura: solo se cuenta el rechazo.
            // Si se repite, la deriva la absorbe el sesgo con la siguiente medida
            // de distancia o, sin ella, se reinicia el sesgo.
            if (++rechazos[2] >= maxRechazos) {
                b = medida - z;
                P[2][2] = sigma * sigma;
                P[0][2] = P[2][0] = P[1][2] = P[2][1] = 0.0f;
                rechazos[2] = 0;
            }
            return false;
        }
        rechazos[2] = 0;
        const float k0 = (P[0][0] + P[0][2]) / S;
        const float k1 = (P[1][0] + P[1][2]) / S;
        const float k2 = (P[2][0] + P[2][2]) / S;
        aplicar(nu, k0, k1, k2, 1);
        return true;
    }

private:
    // x += K nu;  P = (I - K H) P, con H = [1, 0, h2]
    void aplicar(float nu, float k0, float k1, float k2, int h2) {
        z += k0 * nu;
        vz += k1 * nu;
        b += k2 * nu;

        // Fila H P
        float hp[3];
        for (int j = 0; j < 3; j++) hp[j] = P[0][j] + (h2 ? P[2][j] : 0.0f);
        const float k[3] = {k0, k1, k2};
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) P[i][j] -= k[i] * hp[j];
        // Simetría numérica
        for (int i = 0; i < 3; i++)
            for (int j = i + 1; j < 3; j++) {
                const float m = 0.5f * (P[i][j] + P[j][i]);
                P[i][j] = P[j][i] = m;
            }
    }

    void reanclar(float altura) {
        const float baro = z + b;     // conserva la lectura de baro estimada
        z = altura;
        b = baro - altura;
        P[0][0] = 0.01f;
        P[0][1] = P[1][0] = 0.0f;
        P[0][2] = P[2][0] = 0.0f;
        P[2][2] = P[2][2] + 0.04f;
    }
};

#endif
