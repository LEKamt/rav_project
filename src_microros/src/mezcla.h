#ifndef MEZCLA_H
#define MEZCLA_H

// Mezcla en X con desaturación por desplazamiento común. Misma ley que
// robot_control/control.py::mezclar (test_mezcla_paridad.py lo comprueba).
//
// Hélices (hardware/CABLEADO_S3.md): M1 frente-izq y M3 tras-der horarias, M2
// frente-der y M4 tras-izq antihorarias. Cuerpo FLU: alabeo + = lado izquierdo
// arriba, cabeceo + = nariz abajo, guiñada + = antihorario visto desde arriba.
// Una hélice horaria hace par de reacción +z: la guiñada + acelera M1 y M3.
//
// Si un motor se sale de [minimo, maximo] se desplaza el empuje común de los
// cuatro en vez de recortar uno solo: recortar destruye el par diferencial justo
// cuando el control más lo necesita. Sin dependencias de Arduino.

static inline void mezclarX(float empuje, float roll, float pitch, float yaw,
                            float minimo, float maximo, float m[4]) {
    m[0] = empuje + roll - pitch + yaw;
    m[1] = empuje - roll - pitch - yaw;
    m[2] = empuje - roll + pitch + yaw;
    m[3] = empuje + roll + pitch - yaw;

    float mx = m[0], mn = m[0];
    for (int i = 1; i < 4; i++) {
        if (m[i] > mx) mx = m[i];
        if (m[i] < mn) mn = m[i];
    }
    float desplazamiento = 0.0f;
    if (mx > maximo)      desplazamiento = maximo - mx;
    else if (mn < minimo) desplazamiento = minimo - mn;

    for (int i = 0; i < 4; i++) {
        float v = m[i] + desplazamiento;
        m[i] = v < minimo ? minimo : (v > maximo ? maximo : v);
    }
}

#endif
