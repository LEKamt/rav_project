# Cableado del ESP32-S3 a la PCB FirePigeon v1.0

La PCB se diseñó para una ESP32-DevKitC-32E y se reutiliza sin rediseñar: la
ESP32-S3-DevKitC-1 no encaja en su zócalo (44 pines frente a 38, en otro orden), así
que se monta aparte y se une a la PCB con cables. Los puntos de la PCB son los pads
del zócalo U1, para conservar las resistencias serie de 33 Ω.

**Ignorar del diseño original:** Alpha Data Link (J2), RPM de los ESC, UART hacia la
Raspberry Pi (J7) y las líneas SDO, NCS y CSB de J6.

## Conexiones

Pines del montaje real (10-nov-2026), los de `hardware/VueloF450/VueloF450.ino` y de
`src_microros/src/config.h`.

| Señal | Pin del S3 | Pad de U1 (nombre en el esquemático) | Llega a |
|---|---|---|---|
| PWM M1 | GPIO8 | J3_11 (IO17) | R4 33 Ω → L1.2 |
| PWM M2 | GPIO17 | J3_13 (IO4) | R3 33 Ω → L1.6 |
| PWM M3 | GPIO21 | J2_10 (IO26) | R1 33 Ω → L2.2 |
| PWM M4 | GPIO18 | J2_12 (IO14) | R2 33 Ω → L2.6 |
| MPU SDA (bus `Wire`) | GPIO1 | — | IMU |
| MPU SCL (bus `Wire`) | GPIO39 | — | IMU |
| ToF SDA (bus `Wire1`) | GPIO4 | — | VL53L0X |
| ToF SCL (bus `Wire1`) | GPIO5 | — | VL53L0X |
| Ultrasonido TRIG | GPIO6 | — | HC-SR04 |
| Ultrasonido ECHO | GPIO7 | — | HC-SR04 (divisor 1 k / 2 k si es de 5 V) |
| 3V3 | 3V3 | J2_1 (3V3) | J6.2, L1.4, L2.4 |
| 5 V | 5V | J2_19 (EXT_5V) | +5V_PROTECTED |
| GND | GND | J3_1 (GND) | GND |

> **Pines de los motores.** El commit 4d07395 de `main` decía M1–M4 = GPIO37, 35,
> 17 y 8; `PrimerVuelo.ino` (el que movió los motores) usa 8, 17, 21 y 18. Se tomó
> este último: en un S3 con PSRAM octal (módulos N8R8/N16R8) los GPIO35–37 son de la
> PSRAM y no sirven como salida. **Confirmar con el comando `m` de VueloF450** (gira
> un motor a la vez, sin hélices) que cada salida mueve el motor de su posición.

Pulsador de emergencia (opcional): GPIO10 a GND, activo bajo con pull-up interno.

El ECHO de un HC-SR04 de 5 V sale a 5 V y el S3 no lo tolera: divisor 1 kΩ / 2 kΩ
(1 k del sensor al pin, 2 k del pin a GND). El HC-SR04P y el RCWL-1601 funcionan a
3.3 V y no lo necesitan. Mira hacia abajo, junto al VL53L0X.

## Lo que debe estar montado

- **Módulos conversores de nivel en L1↔H1 y L2↔H2.** Sin ellos el PWM no llega a los
  ESC: `PWM_M*` (3.3 V) y `PWM_ESC*` (5 V) solo se unen a través de esos módulos.
- **Buck de 5 V** en IN+/IN- y out1+.
- **IMU en su propio bus** (`Wire`, 0x68): MPU-9250, MPU-9255 o el clon MPU-6500 de
  la GY-91. Si la GY-91 trae BMP280, queda en el mismo bus (0x76 o 0x77).
- **VL53L0X en el bus `Wire1`** (0x29), mirando hacia abajo. Mide hasta ~1 m.

## Ejes de la IMU

Los firmwares trabajan en FLU (x adelante, y izquierda, z arriba). Cómo quedan los
ejes del sensor respecto del dron depende del montaje, y se corrige con
`IMU_CUERPO_X/Y/Z` (en `VueloF450.ino` y en `config.h`).

**El mismo cambio de ejes va al acelerómetro y al giroscopio, y tiene que ser una
rotación.** Intercambiar x e y sin cambiar un signo (`ax = raw_ay; ay = raw_ax`) es
un espejo: el giroscopio y el acelerómetro empujan el ángulo en sentidos opuestos y
el filtro no converge. Una rotación de 90° sobre z es `x = raw_y, y = -raw_x` (o
`x = -raw_y, y = raw_x`).

Con el dron quieto, sin hélices, el comando `c` de VueloF450 revisa los ejes:

| Posición | Acelerómetro esperado | Ángulo esperado |
|---|---|---|
| Nivelado | z ≈ +1 g, x ≈ 0, y ≈ 0 | alabeo y cabeceo ≈ 0 |
| Nariz abajo | x negativo | cabeceo positivo |
| Lado izquierdo arriba | y positivo | alabeo positivo |
| Girar a la izquierda (antihorario visto desde arriba) | giro z positivo | — |

**Ningún firmware arma** si en reposo el acelerómetro no marca (0, 0, 1) g, ni si el
dron se movió durante la calibración del giro al arrancar.

## Motores

| Motor | Posición | Conector | Giro visto desde arriba |
|---|---|---|---|
| M1 | Frente izquierda | J4 | Horario |
| M2 | Frente derecha | J5 | Antihorario |
| M3 | Atrás derecha | J8 | Horario |
| M4 | Atrás izquierda | J9 | Antihorario |

Mezcla en X que corresponde a estos sentidos (FLU, `r` alabeo, `p` cabeceo,
`y` guiñada, `T` empuje común):

```text
M1 = T + r - p + y      M2 = T - r - p - y
M3 = T - r + p + y      M4 = T + r + p - y
```

Una hélice horaria hace par de reacción antihorario sobre el cuerpo: para girar a la
izquierda (guiñada +) se aceleran M1 y M3.

## Alimentación

- No alimentar el S3 a la vez por USB y por el pin 5V. En banco con USB, desconectar
  el cable de 5 V.
- La salida de 5 V pasa por F2 (PTC de 500 mA) y alimenta el S3, los conversores y
  los sensores.
- Los ESC se alimentan directamente de la batería, no a través de esta PCB.

## Antes de volar

1. Sin hélices: encender **quieto y nivelado** (calibra el giro) y confirmar que los
   cuatro ESC terminan su secuencia de pitidos.
2. Sin hélices: comando `c` (ejes) y comando `m` (cada salida mueve su motor y en el
   sentido de la tabla).
3. Sin hélices, armado con poco empuje: inclinar el dron a mano y comprobar que
   aceleran los motores del lado que baja; girarlo a la izquierda y comprobar que
   aceleran M2 y M4 (los que lo devuelven a la derecha).
4. Medir el mando de hover real y ajustarlo.
5. Primer vuelo sujeto, a poca altura.
