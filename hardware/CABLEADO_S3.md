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

## Lo que debe estar montado

- **Módulos conversores de nivel en L1↔H1 y L2↔H2.** Sin ellos el PWM no llega a los
  ESC: `PWM_M*` (3.3 V) y `PWM_ESC*` (5 V) solo se unen a través de esos módulos.
- **Buck de 5 V** en IN+/IN- y out1+.
- **Sensores I2C en J6**, en el mismo bus: MPU-6500 (0x68), MS5611 (0x77) y VL53L0X
  (0x29). La PCB solo tiene un conector, así que el VL53L0X se conecta en paralelo.

> **Atención con la GY-91.** J6 está pensado para una GY-91, y el firmware actual no
> funciona con ella. `mpu.init()` exige el identificador de la MPU-6500, y el MS5611
> tampoco está en ese módulo. El firmware se detiene en `[FATAL] Fallo en sensores`
> con los motores parados.

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

1. Sin hélices: encender y confirmar que los cuatro ESC terminan su secuencia de pitidos.
2. Sin hélices: inclinar el dron a mano y comprobar que aceleran los motores del lado
   que baja.
3. Medir el mando de hover real en banco y ajustar `THROTTLE_HOVER` en `config.h` y en
   `robot_control/control.py`: su error se convierte en error de altura en régimen.
4. Primer vuelo sujeto, a poca altura.
