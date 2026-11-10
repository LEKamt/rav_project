# VueloF450: firmware de prueba del F450 (Arduino, sin micro-ROS)

Sucesor de [`PrimerVuelo.ino`](../PrimerVuelo.ino), con los mismos pines y sensores. Sirve para las
pruebas F1–F5 de `Arquitectura_Blaze/docs/24`: control de actitud PID o SMC a 500 Hz, filtros del
giro, AHRS Mahony, Kalman de altura (ToF + ultrasonido) y failsafes. El encabezado de
`VueloF450.ino` explica qué cambia respecto de `PrimerVuelo.ino` y por qué.

## Cargar

- Arduino IDE con el núcleo **arduino-esp32 3.x**, placa "ESP32S3 Dev Module".
- Librería **Adafruit_VL53L0X**.
- Abrir `VueloF450.ino`; la carpeta `src/` se compila sola. Son copias de `src_microros/src`
  (`robot_tests/unit/test_vuelo_arduino.py` comprueba que no se separen).
- Operar con la estación (necesita `pip install pyserial`):

  ```bash
  python hardware/VueloF450/estacion.py COM5        # el puerto del CH340
  ```

  La terminal serie de Arduino sirve para el banco sin hélices (`banco 1`): **sin la estación no
  hay latido**, y armado fuera de banco el dron entra en bajada al segundo.

## Antes de volar, en este orden

| Paso | Hélices | Comando | Qué debe pasar |
| --- | --- | --- | --- |
| 1. Ejes | No | `c`, luego `v` | `c` da OK en los dos ejes. Con `v`: nariz abajo → pitch +, lado izquierdo arriba → roll +, girar a la izquierda → r +. Si algo falla, corregir `IMU_CUERPO_*`. Después poner `EJES_VERIFICADOS 1` y volver a cargar |
| 2. ESC | No | `k`, `k ok`, conectar la batería, `k fin` | Los ESC quedan calibrados en 1000–2000 µs (`PrimerVuelo` los dejaba en 1000–1600 en cada arranque) |
| 3. Motores | No | `m 1` … `m 4` | Gira el motor de esa posición y en el sentido de la tabla de `CABLEADO_S3.md` |
| 4. Mezcla | No | `banco 1`, `a`, `t 0.20` | Al inclinar a mano aceleran los motores del lado que baja; al girarlo a la izquierda aceleran M2 y M4 |
| 5. Balancín | Sí | `a`, subir con `+`, `e r 5 1` | En un eje fijo (1 grado de libertad): respuesta al escalón con `ley pid` y con `ley smc` |
| 6. Hover con cuerda | Sí | `a`, `+` hasta despegar | 60 s estable a ~1 m |

`d` desarma, `x` es PARO, `s` inicia la bajada gradual. Escribir `ayuda` para ver todos los comandos.

## Seguridad

- **Sin enlace 1 s** (fuera de banco): bajada gradual al 5 %/s con la actitud activa.
- **Inclinación > 60°** o **IMU caída 50 ms**: paro de motores.
- **No arma** si: ejes sin verificar, giro sin calibrar (se movió al arrancar), acelerómetro fuera
  de (0, 0, 1) g, dron inclinado > 10° o empuje distinto de 0.
- Tope de cada motor: 0.80 (`MANDO_MAX`); en banco, 0.25.
- PWM a 400 Hz. Si algún ESC no arranca o tartamudea, poner `PWM_HZ 50`.

## Telemetría

Cada vuelo queda en `vuelos/vuelo_<fecha>.csv` (50 Hz) y los mensajes en el `.txt` del mismo nombre.
Columnas: tiempo, modo, ley, empuje, ángulos, velocidades angulares, consignas, correcciones, pulsos
de los 4 motores, altura y velocidad del Kalman, ToF, ultrasonido, peor tiempo del lazo y atrasos.
