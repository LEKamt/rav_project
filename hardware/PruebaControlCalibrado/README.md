# Guía de Pruebas: PruebaControlCalibrado

Este proyecto implementa el control de actitud y pruebas de empuje para el dron **F450** con el **ESP32-S3 Dev Module**, integrando las curvas de calibración medidas en la balanza y la lectura de los sensores.

## 1. Conexión de Pines

| Función | Pin ESP32-S3 | Conexión |
|---|---|---|
| **PWM M1** (Frente Izq, CW) | **GPIO 8** | Señal ESC 1 |
| **PWM M2** (Frente Der, CCW) | **GPIO 17** | Señal ESC 2 |
| **PWM M3** (Atrás Der, CW) | **GPIO 21** | Señal ESC 3 |
| **PWM M4** (Atrás Izq, CCW) | **GPIO 18** | Señal ESC 4 |
| **MPU SDA** | **GPIO 1** | Bus I2C0 MPU-6050/6500 |
| **MPU SCL** | **GPIO 39** | Bus I2C0 MPU-6050/6500 |
| **ToF SDA** | **GPIO 4** | Bus I2C1 VL53L0X |
| **ToF SCL** | **GPIO 5** | Bus I2C1 VL53L0X |
| **US TRIG** | **GPIO 6** | HC-SR04 Trig |
| **US ECHO** | **GPIO 7** | HC-SR04 Echo (¡con divisor 5V->3.3V!) |

---

## 2. Configuración en Arduino IDE

1. Abrir `hardware/PruebaControlCalibrado/PruebaControlCalibrado.ino`.
2. Placa: **ESP32S3 Dev Module**.
3. Parámetros recomendados:
   - **USB CDC On Boot:** *Enabled*
   - **Flash Size:** *8MB (o 4MB según tu módulo)*
   - **Upload Speed:** *921600*
4. Librerías necesarias:
   - `Adafruit_VL53L0X` (si tienes el sensor ToF conectado).

---

## 3. Comandos Disponibles por Monitor Serial (115200 baudios)

Al conectar por USB y abrir el Monitor Serial con "Nueva línea", puedes enviar:

### Pruebas de Banco / Motores Individuales:
* `m 1 10`  $\to$ Enciende el **Motor 1** para generar exactamente **10 gramos** de empuje.
* `m 2 25`  $\to$ Enciende el **Motor 2** a **25 gramos**.
* `m 3 15`  $\to$ Enciende el **Motor 3** a **15 gramos**.
* `m 4 20`  $\to$ Enciende el **Motor 4** a **20 gramos**.
* `todos 15` $\to$ Enciende los 4 motores para generar **15 gramos** cada uno. El dron calcula el pulso microsegundos específico de cada motor según su curva.
* `s`        $\to$ **PARADA INMEDIATA** (manda 1000 µs a todos).

### Modo de Vuelo Estabilizado (PID + MPU):
* `a`        $\to$ **Armar**. Arranca el lazo PID estabilizado con 15% de acelerador base.
* `+`        $\to$ Sube el acelerador un 5%.
* `-`        $\to$ Baja el acelerador un 5%.
* `s`        $\to$ Corta motores de inmediato.
* `esc <1-4> <val>` $\to$ Ajusta la escala de empuje de un motor individual (ej: `esc 2 0.35` para 35% en M2).

### Sensores:
* `diag`     $\to$ Muestra las lecturas de Roll, Pitch, Rate Yaw de la MPU, ToF (mm) y Ultrasonido (cm).

---

## 4. Control Web Inalámbrico (WiFi)

El ESP32-S3 genera una red WiFi propia:
* **SSID:** `F450-TEST`
* **Contraseña:** `f450vuelo`
* **Dirección IP:** `http://192.168.4.1`

Desde cualquier teléfono o laptop conectada a esa red, puedes ver los ángulos de Roll y Pitch en tiempo real, los pulsos y gramos estimados de cada motor, y botones para armar, parar y ajustar ganancias PID.
