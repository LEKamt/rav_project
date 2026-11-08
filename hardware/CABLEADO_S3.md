# Cableado del ESP32-S3 a la PCB FirePigeon v1.0

La PCB se diseñó para una ESP32-DevKitC-32E y se reutiliza sin rediseñar: la
ESP32-S3-DevKitC-1 no encaja en su zócalo (44 pines frente a 38, en otro orden), así
que se monta aparte y se une a la PCB con cables. Los pines del S3 son los de
`src_microros/src/config.h`. Los puntos de la PCB son los pads del zócalo U1, para
conservar las resistencias serie de 33 Ω.

**Ignorar del diseño original:** Alpha Data Link (J2), RPM de los ESC, UART hacia la
Raspberry Pi (J7) y las líneas SDO, NCS y CSB de J6.

## Conexiones

| Señal | Pin del S3 | Pad de U1 (nombre en el esquemático) | Llega a |
|---|---|---|---|
| PWM M1 | GPIO4 | J3_11 (IO17) | R4 33 Ω → L1.2 |
| PWM M2 | GPIO5 | J3_13 (IO4) | R3 33 Ω → L1.6 |
| PWM M3 | GPIO6 | J2_10 (IO26) | R1 33 Ω → L2.2 |
| PWM M4 | GPIO7 | J2_12 (IO14) | R2 33 Ω → L2.6 |
| I2C SDA | GPIO8 | J3_2 (IO23) | J6.5 |
| I2C SCL | GPIO9 | J3_9 (IO18) | J6.4 |
| 3V3 | 3V3 | J2_1 (3V3) | J6.2, L1.4, L2.4 |
| 5 V | 5V | J2_19 (EXT_5V) | +5V_PROTECTED |
| GND | GND | J3_1 (GND) | GND |

Pulsador de emergencia: **GPIO10 a GND**, directo, porque la PCB no tiene entrada
para él. Es activo bajo con pull-up interno; sin pulsador el firmware funciona, pero
no hay corte físico.

Ultrasonido (HC-SR04 o similar), también directo al S3 porque la PCB no tiene
entrada: **TRIG a GPIO11 y ECHO a GPIO12** (`PIN_SONAR_*` en `config.h`). Un HC-SR04
de 5 V saca el eco a 5 V y el S3 no lo tolera: poner un **divisor 1 kΩ / 2 kΩ** en
ECHO (1 k del sensor al pin, 2 k del pin a GND). El HC-SR04P y el RCWL-1601
funcionan a 3.3 V y no lo necesitan. Debe mirar hacia abajo, junto al VL53L0X.

## Lo que debe estar montado

- **Módulos conversores de nivel en L1↔H1 y L2↔H2.** Sin ellos el PWM no llega a los
  ESC: `PWM_M*` (3.3 V) y `PWM_ESC*` (5 V) solo se unen a través de esos módulos.
- **Buck de 5 V** en IN+/IN- y out1+.
- **GY-91 en J6, por I2C:** MPU-9250 en 0x68 (se aceptan también la MPU-9255 y el
  clon MPU-6500) y BMP280 en 0x76 o 0x77. El firmware usa SDA/SCL (J6.5/J6.4) e
  ignora SDO, NCS y CSB: en I2C, CSB y NCS deben quedar en alto (el módulo ya los
  sube) y SDO fija la dirección.
- **VL53L0X en paralelo en el mismo bus** (0x29), mirando hacia abajo. En este dron
  mide hasta ~1 m (`TOF_MAX_M`).

Solo la IMU es imprescindible: sin ella el firmware se detiene en `[FATAL] Fallo
en sensores` con los motores parados. Sin baro, ToF o ultrasonido arranca igual y
el Kalman de altura sigue con lo que haya. Lo que respondió al arrancar sale en
`/dron/motores[6]` (bits: 0 IMU, 1 baro, 2 ToF, 3 sonar, 4 giro calibrado) y el
`WHO_AM_I` en `/dron/motores[7]`.

## Ejes de la IMU

El firmware trabaja en FLU (x adelante, y izquierda, z arriba). Cómo quedan los
ejes de la GY-91 respecto del dron depende de cómo se montó, y se corrige con
`IMU_CUERPO_X/Y/Z` en `config.h`. Con el dron encendido, sin hélices, mirar
`ros2 topic echo /dron/imu`:

| Posición | `linear_acceleration` esperada | Ángulo esperado |
|---|---|---|
| Nivelado | z ≈ +9.8, x ≈ 0, y ≈ 0 | alabeo y cabeceo ≈ 0 |
| Nariz abajo | x negativo | cabeceo positivo |
| Lado izquierdo arriba | y positivo | alabeo positivo |
| Girar a la izquierda (antihorario visto desde arriba) | `angular_velocity.z` positivo | — |

Si un eje sale cambiado o con el signo al revés, corregir la macro (por ejemplo,
`#define IMU_CUERPO_X(s) (-(s)[1])`). **El firmware no arma** si en reposo el
acelerómetro no marca (0, 0, 1) g con 0.2 g de tolerancia, ni si el dron se movió
durante la calibración del giro al arrancar (2 s quieto).

## Motores

| Motor | Posición | Conector | Giro visto desde arriba |
|---|---|---|---|
| M1 | Frente izquierda | J4 | Antihorario |
| M2 | Frente derecha | J5 | Horario |
| M3 | Atrás derecha | J8 | Antihorario |
| M4 | Atrás izquierda | J9 | Horario |

En J4, J5, J8 y J9: pin 3 = señal del ESC, pin 2 = GND, pin 1 sin usar.

El sentido de giro sale del signo de la guiñada en
`rav_ws/src/robot_control/src/robot_control/controlador_sim.py`. Comprobar la mezcla
**sin hélices** antes del primer vuelo.

## Alimentación

- No alimentar el S3 a la vez por USB y por el pin 5V. En banco con USB, desconectar
  el cable de 5 V.
- La salida de 5 V pasa por F2 (PTC de 500 mA) y alimenta el S3, los conversores y
  los sensores.
- Los ESC se alimentan directamente de la batería, no a través de esta PCB.

## Antes de volar

1. Sin hélices: encender **quieto y nivelado** (calibra el giro) y confirmar que los
   cuatro ESC terminan su secuencia de pitidos.
2. Sin hélices: revisar la tabla de [Ejes de la IMU](#ejes-de-la-imu).
3. Sin hélices: inclinar el dron a mano y comprobar que aceleran los motores del lado
   que baja.
4. Medir el mando de hover real en banco y ajustar `THROTTLE_HOVER` en `config.h` y en
   `robot_control/control.py`: su error se convierte en error de altura en régimen.
5. Primer vuelo sujeto, a poca altura.
