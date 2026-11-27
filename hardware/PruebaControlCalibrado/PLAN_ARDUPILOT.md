# Plan: ArduPilot (ArduCopter) en el ESP32-S3 DevKit del F450

> Fecha: 21-nov-2026 · Entrega: 28-nov (quedan 7 días)
> Estado: **propuesta**. Nada de esto se ha probado todavía en el hardware.

## 0. Respuesta corta

**Sí se puede, con reservas.** ArduPilot tiene un HAL para ESP32 (`libraries/AP_HAL_ESP32`)
que incluye placas ESP32-S3 (`esp32s3devkit`, `esp32s3empty` y la M5Stack StampFly, que es un
quad con S3 que sí vuela). Pero:

| | |
|---|---|
| Soporte | **Experimental.** No hay binarios oficiales en firmware.ardupilot.org: hay que compilarlo uno mismo (Linux/WSL + ESP-IDF). |
| Vehículo | Solo ArduCopter (y Rover) son razonables. |
| Telemetría | MAVLink por **WiFi** (AP propio, UDP 14550) → Mission Planner o QGroundControl en la laptop. |
| Riesgo | Alto para 7 días. Puede fallar al detectar la IMU por I2C, tener jitter en el lazo o no armar por checks. |

**Recomendación:** que ArduPilot sea una **rama paralela con puntos de corte**, no el camino
crítico. `PruebaControlCalibrado` / `PrimerVueloWiFi_PIDv2` siguen siendo el plan A para la
entrega. Si ArduPilot vuela, sirve como **línea base** ("controlador de referencia") frente al
PID/SMC propio. En la tesis, el aporte es el control propio: ArduPilot no lo reemplaza.

---

## 1. Lo que ArduPilot resuelve (y lo que no)

**Resuelve:**
- **El problema de M2.** La curva de balanza de M2 (`8.93x² − 19.7x + 33.2`) da unos 730 g a
  x = 10, mientras que M1 da unos 53 g. Eso no es un motor distinto: son **ESC con extremos
  sin calibrar**. ArduPilot calibra los 4 ESC a la vez (`ESC_CALIBRATION = 3`) y quita la
  necesidad del parche `escala_motor[1] = 0.35`.
- Un EKF3, un PID de rate y de ángulo ya sintonizables (AutoTune), failsafes, logs `.bin` y
  una GCS gráfica.

**No resuelve:**
- ArduPilot **supone motores iguales**: no tiene escala por motor. Todos los motores usan el
  mismo `MOT_PWM_MIN/MAX`; `SERVOn_MIN/MAX` no se aplica a las salidas de motor.
- El SMC y la identificación de planta del balancín: eso sigue en el firmware propio.

### 1.1 M2 con un ESC distinto: qué hacer

El integrador del PID de rate **no basta** para compensar M2. Con el mismo pulso, M2 empuja
unas 10 veces lo que M1. En el suelo, ArduPilot mantiene los integradores casi en cero (estado
*landed*). Al despegar, M2 (frente-derecha) levanta antes que los demás y el dron se voltea
hacia atrás-izquierda antes de que el I se cargue. Además, M2 es CCW: su exceso de par también
satura la guiñada.

En orden de preferencia:

1. **Cambiar el ESC de M2 por uno del mismo modelo que los otros tres.** Es lo más barato y lo
   arregla para los dos firmwares.
2. **Calibrar los 4 ESC** con el mismo rango (1000–2000 µs) y **medir de nuevo en la balanza
   desde Mission Planner → Motor Test**: mismo % de gas en cada motor, anotar los gramos. Si
   las 4 curvas quedan dentro de ±15 %, los valores por defecto + el I + AutoTune lo absorben.
3. **Situación real (21-nov): el ESC de M2 no acepta la calibración.** Siempre queda con un
   rango más corto: con pulsos bajos ya da mucho más empuje que los otros. Por eso un factor
   fijo (`escala_motor = 0.35`) no alcanza: corrige un punto de la curva y deja mal los demás.
   Hace falta una **tabla de mapeo por motor**: para el pulso "nominal" que pide el control,
   enviar a M2 el pulso que da **el mismo empuje** que los otros motores con ese pulso.

   **Medir la tabla (balanza, sin ArduPilot, con el sketch actual):**
   - Para M1, M3 y M4: empuje en g con 1000, 1100, …, 1850 µs → curva nominal = promedio.
   - Para M2: barrido fino (cada 10–20 µs) en su zona útil, hasta que el empuje deje de subir.
   - Por cada pulso nominal p, buscar el pulso de M2 con el mismo empuje (interpolando).
     Resultado: unos 8–10 pares (p_nominal → p_M2). Usar los puntos medidos, no el ajuste
     cuadrático: el de M2 (`8.93x² − 19.7x + 33.2`) tiene su mínimo dentro del rango y no
     sirve para invertirlo.
   - **Comprobar el empuje máximo de M2.** Si a tope empuja menos que los otros, el mapeo se
     satura y el gas total se limita a lo que dé M2.

   **Parche en ArduPilot** (se compila a mano de todos modos): interpolación lineal en la tabla,
   justo antes de escribir el PWM de la salida 1 (M2):
   ```cpp
   // en AP_MotorsMatrix::output_to_motors(), donde se hace rc_write(i, output_to_pwm(...))
   // tabla medida en balanza: pulso nominal -> pulso de M2 (valores de ejemplo, MEDIR)
   static const int16_t P_NOM[] = {1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1850};
   static const int16_t P_M2[]  = {1000, 1030, 1060, 1090, 1120, 1150, 1180, 1210, 1250};
   int16_t pwm = output_to_pwm(_actuator[i]);
   if (i == 0) {                      // salida AP 1 = M2 (frente-der)
       pwm = interp_tabla(pwm, P_NOM, P_M2, 9);   // lineal por tramos, con saturación
   }
   rc_write(i, pwm);
   ```
   (Verificar los nombres de las funciones en el commit clonado; `interp_tabla` es una función
   de unas 10 líneas que hay que agregar.) El mismo mapeo conviene llevarlo al firmware propio,
   en lugar de `aplicarEscalaMotor()` y `fuerzaAPulso()`.
   - Mientras tanto, si se puede: identificar el modelo del ESC de M2. Si es BLHeli/BLHeli_S, el
     rango de gas se fija en BLHeliSuite (con un Arduino como passthrough) y se arregla de raíz.
     Si no, **cambiarlo por uno igual a los otros** sigue siendo lo mejor.
4. Prueba de aceptación antes de poner hélices de vuelo: en el balancín, en Stabilize con gas
   cerca del hover, el dron no debe tender a caer hacia un lado y el I de roll/pitch del log
   (`PIDR.I`, `PIDP.I`) debe quedar por debajo de ~0.1.

**Hacer ya, sin importar ArduPilot:** calibrar los ESC (`hardware/CalibrarESC`) y repetir la
curva de balanza de M2. Si al calibrar las 4 curvas quedan parecidas, el firmware propio
también mejora.

---

## 2. Hardware actual → mapeo en ArduPilot

| Elemento | Hoy | En ArduPilot | Nota |
|---|---|---|---|
| IMU (MPU-6500/9250 del GY-91) | I2C0 SDA 1 / SCL 39, 0x68 | Driver `Invensense` por I2C | Por I2C funciona, pero ArduPilot prefiere SPI. Opción: recablear el GY-91 a SPI (SCL→SCK, SDA→MOSI, SDO→MISO, NCS→CS). Probar I2C primero. |
| Baro BMP280 (GY-91) | mismo I2C0 | `BARO_PROBE` BMP280 0x76/0x77 | |
| Brújula AK8963 | (no se usa) | `COMPASS_ENABLE = 0` | Adentro, cerca del cable de potencia, la brújula estorba. Se vuela en Stabilize/AltHold sin ella. |
| ToF VL53L0X | I2C1 SDA 4 / SCL 5 | `RNGFND1_TYPE = 16` (VL53L0X), `RNGFND1_ADDR = 41` | ~1 m de alcance: útil solo para despegue y aterrizaje. |
| HC-SR04 | TRIG 6 / ECHO 7 | `RNGFND2_TYPE = 30` (HC-SR04) | **Verificar** que el driver compila en el HAL de ESP32 (usa una interrupción de GPIO). Si no, dejarlo fuera. |
| Motores | GPIO 8/17/21/18 | `RCOUT` en el hwdef | Ver el orden abajo. |
| RC | no hay (mando por WiFi) | `RC_CHANNELS_OVERRIDE` / `MANUAL_CONTROL` desde la GCS con un joystick | ArduPilot puede volar sin radio, pero el failsafe de GCS es obligatorio. |
| Batería | cable (fuente 12 V 50 A) | `BATT_MONITOR = 0` | Sin sensor de voltaje. |

### Orden de motores (crítico)

ArduCopter, Quad X (`FRAME_CLASS = 1`, `FRAME_TYPE = 1`):

| Salida AP | Posición | Giro AP | Motor actual | GPIO |
|---|---|---|---|---|
| Motor 1 | Frente-Der | CCW | M2 (CCW) ✔ | **17** |
| Motor 2 | Atrás-Izq | CCW | M4 (CCW) ✔ | **18** |
| Motor 3 | Frente-Izq | CW | M1 (CW) ✔ | **8** |
| Motor 4 | Atrás-Der | CW | M3 (CW) ✔ | **21** |

Los sentidos de las hélices ya coinciden con ArduPilot (M1 y M3 horarias). Solo cambia la
numeración: en el hwdef, poner `RCOUT = {17, 18, 8, 21}`, o dejar el orden {8, 17, 21, 18} y
remapear con `SERVO1..4_FUNCTION = 35, 33, 36, 34`. Se confirma con **Motor Test** en Mission
Planner (sin hélices): A = frente-der, B = atrás-der, C = atrás-izq, D = frente-izq (en el
sentido de las agujas del reloj).

---

## 3. Fases con criterio de corte

### Fase 0 — Preparar el entorno (día 1, ~3 h)

En **WSL Ubuntu** (ya existe el que se usa para PlatformIO):

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y   # pide sudo; si no hay sudo, ver nota
. ~/.profile
Tools/scripts/esp32_get_idf.sh        # descarga la versión de ESP-IDF que exige este commit
. modules/esp_idf/export.sh           # (la ruta exacta la imprime el script)
./waf configure --board=esp32s3devkit
./waf copter
```

- Nota: en este WSL no había sudo (ver la memoria del venv de PlatformIO). Si
  `install-prereqs` falla, instalar a mano solo `python3-pip`, `empy==3.3.4`, `pexpect`,
  `future` en un venv, más los requisitos de ESP-IDF (`cmake`, `ninja`), que trae el script del IDF.
- Revisar en `libraries/AP_HAL_ESP32/` **cómo define este commit las placas**: `boards/*.h`
  (formato antiguo, con macros `HAL_ESP32_*`) o `hwdef/<placa>/hwdef.dat` (formato nuevo).
  La fase 2 depende de esto.
- Leer `libraries/AP_HAL_ESP32/README.md`: ahí están el pinout por defecto del `esp32s3devkit`,
  el SSID/clave de WiFi y el comando de flasheo.
- Confirmar la flash y la PSRAM del módulo (`esptool.py flash_id`). ArduCopter necesita ≥ 4 MB;
  si es N8R8 o N16R8, mejor.

**Corte:** si `./waf copter` no compila el `esp32s3devkit` original en ≤ 4 h → **abortar
ArduPilot**.

### Fase 1 — Flashear la placa original, sin nada conectado (día 1)

```bash
./waf copter --upload          # o esptool.py write_flash con los .bin de build/esp32s3devkit/
```

- Conectarse al AP WiFi que crea la placa (por defecto `ardupilot` / `ardupilot1`; confirmar
  en el README) y abrir Mission Planner con UDP 14550 (o TCP 192.168.4.1:5760).
- Esperado: llega el HEARTBEAT, se pueden leer parámetros y aparecen mensajes "No IMU" o "Baro"
  (normal: los pines no coinciden).

**Corte:** si no hay MAVLink estable por WiFi → abortar.

### Fase 2 — Placa propia `f450s3` (día 2)

Copiar la definición del `esp32s3devkit` a una placa nueva `f450s3` y cambiar solo esto (los
nombres de las macros o de las claves se toman del archivo original, no de este plan):

- I2C bus 0: SDA 1, SCL 39, 400 kHz → IMU + baro.
- I2C bus 1: SDA 4, SCL 5 → VL53L0X.
- Lista de IMU: Invensense en I2C bus 0, dirección 0x68, con la rotación según cómo va montado
  el GY-91. **Recordar** el hallazgo del 10/19-nov: en este montaje los ejes van en espejo
  (x = raw_y). Ese espejo se corrige con la rotación del sensor (`ROTATION_*` en el hwdef o
  `AHRS_ORIENTATION`), no a mano.
- Baro: BMP280 en I2C bus 0.
- RCOUT: 17, 18, 8, 21.
- RCIN: deshabilitado (o un pin libre si después se agrega un receptor PPM/SBUS).
- **No** usar los GPIO 26–32 (flash) ni los 35–37 (PSRAM octal). GPIO 39 sirve, pero es
  MTCK del JTAG.
- WiFi: modo AP, SSID `F450-AP`.

```bash
./waf configure --board=f450s3 && ./waf copter --upload
```

**Corte:** si la IMU no aparece (`INS_ACC_ID` / `INS_GYR_ID` = 0) después de 1 día → probar el
GY-91 por SPI. Si tampoco aparece → abortar.

### Fase 3 — Configuración en banco, SIN hélices (día 3)

Parámetros base (cargar como `f450s3.param`):

```
FRAME_CLASS     1      # Quad
FRAME_TYPE      1      # X
COMPASS_ENABLE  0
COMPASS_USE     0
BATT_MONITOR    0
ARMING_CHECK    <ajustar: quitar GPS, brújula, RC y batería; dejar INS y baro>
MOT_PWM_TYPE    0      # PWM normal
MOT_PWM_MIN     1000
MOT_PWM_MAX     2000   # después de calibrar los ESC
MOT_SPIN_ARM    0.06
MOT_SPIN_MIN    0.10
MOT_THST_HOVER  0.45   # a estimar; ajustar con MOT_HOVER_LEARN
SERVO_RATE      400    # bajar a 50 si un ESC no acepta 400 Hz
FS_GCS_ENABLE   1      # aterriza si se pierde la GCS (no hay radio)
FS_THR_ENABLE   0      # no hay radio
FLTMODE1        0      # Stabilize
RNGFND1_TYPE    16
RNGFND1_MAX_CM  120
EK3_SRC1_YAW    0      # sin brújula (la guiñada deriva; aceptable en cautivo)
SCHED_LOOP_RATE 400    # bajar a 200 si el log muestra pérdida de lazo
```

Lista de chequeo:
1. Accel Calibration (6 posiciones) y Level.
2. En la vista HUD, inclinar el dron con la mano: roll y pitch con el signo correcto. Si sale
   en espejo → cambiar la rotación (fase 2) y volver a compilar.
3. **Calibrar los ESC** (`ESC_CALIBRATION = 3`, se hace al reiniciar, sin hélices).
4. Motor Test A–D: confirmar posición y sentido de giro.
5. Revisar el log: `PM.NLon` (lazos lentos) y la vibración (`VIBE`).

**Corte:** si hay lazos perdidos sostenidos o la actitud tiene saltos a 200 Hz → abortar.

### Fase 4 — Balancín y cautivo (días 4–5)

Seguir la misma escalera de `PrimerVueloWiFi_PIDv2/PRUEBAS.md`:
1. Balancín de 1 GDL (tubo, roll): Stabilize, gas bajo. Ajustar `ATC_RAT_RLL_P/I/D` (los
   valores por defecto suelen ser altos para un F450 con 1045; bajar P a ~0.08).
2. Hover cautivo con cuerda, joystick por WiFi.
3. Si se sostiene: **AutoTune** solo en roll/pitch, siempre amarrado.
4. AltHold con baro + VL53L0X, solo si sobra tiempo.

**Seguridad:** siempre con alguien en el corte de la fuente de 12 V; probar el failsafe de GCS
(apagar el WiFi de la laptop → el dron debe aterrizar o desarmar) **antes** de cada vuelo.

### Fase 5 — Comparación para la tesis (día 6, opcional)

- Descargar el `.bin` y extraer `ATT` (Roll/DesRoll) y `RATE`.
- Comparar contra el PID propio, con el mismo escalón en el balancín: sobrepaso, tiempo de
  establecimiento e ITAE. Usar `hardware/balancin/identificar.py` con los dos registros.
- Presentarlo como "controlador de referencia de código abierto", no como el aporte.

### Día 7 — Congelar

No se toca firmware el 27–28-nov. Se vuela con el que haya pasado la fase 4.

---

## 4. Riesgos

| Riesgo | Probabilidad | Mitigación |
|---|---|---|
| El HAL de ESP32 en master no compila o no arranca con el S3 | Media | Probar una etiqueta estable `Copter-4.x` además de master. |
| IMU por I2C lenta o ruidosa | Media | GY-91 por SPI (4 cables + CS). |
| El WiFi del S3 roba CPU y el lazo pierde ciclos | Media | `SCHED_LOOP_RATE 200`, MAVLink a una tasa baja (`SR0_*`). |
| Armado bloqueado por los checks | Alta | `ARMING_CHECK` mínimo, leer los mensajes de prearmado. |
| M2 sigue distinto después de calibrar | Media | Cambiar el ESC o el motor de M2; ArduPilot no lo compensa. |
| Se come el tiempo del plan A | **Alta** | Cortes estrictos; máximo 1 persona en esto (Luis o Kenyi), la otra sigue con el plan A. |

## 5. Qué entra al repo

- `hardware/ardupilot_f450/` con `README.md`, la definición de la placa `f450s3` (copia para
  referencia), `f450s3.param` y los logs.
- El fork de ArduPilot queda fuera de `rav_project`: solo se anota el commit usado.
