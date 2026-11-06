#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ===================== PINES =====================
#define I2C_SDA             8
#define I2C_SCL             9

#define MOTOR1_PIN          4
#define MOTOR2_PIN          5
#define MOTOR3_PIN          6
#define MOTOR4_PIN          7

#define PIN_BOTON_EMERGENCIA 10  // Pulsador externo (activo bajo)

// ===================== DIRECCIONES I2C =====================
#define MPU_ADDR            0x68
#define MS5611_ADDR         0x77
#define VL53L0X_ADDR        0x29

// ===================== micro-ROS =====================
#define MICROROS_NODO           "dron_f450"
#define MICROROS_NAMESPACE      ""
#define MICROROS_SERIAL_BAUD    921600  // debe coincidir con el -b del agente

// Solo para transporte wifi: el ESP32 entra como ESTACIÓN en la red donde
// corre el agente (router o hotspot del PC). Ya no crea un AP propio.
#define MICROROS_WIFI_SSID      "Dron_F450"
#define MICROROS_WIFI_PASS      "dron123456789"
#define MICROROS_AGENTE_IP      "192.168.1.100"  // IP del PC que corre el agente
#define MICROROS_AGENTE_PUERTO  8888

// ===================== TASAS DE LOS LAZOS =====================
#define FREQ_CONTROL_HZ     250     // tarea rápida: IMU + actitud + mezcla + PWM (core 1)
#define FREQ_SENSORES_HZ    50      // tarea lenta: baro/LiDAR + cascada de altura (core 0)
#define PERIODO_CONTROL_MS  (1000 / FREQ_CONTROL_HZ)
#define PERIODO_SENSORES_MS (1000 / FREQ_SENSORES_HZ)

#define PUB_RAPIDO_HZ       50      // alturas + señales de control
#define PUB_IMU_HZ          20      // sensor_msgs/Imu (mensaje grande)
#define PUB_LENTO_HZ        5       // estado + motores

// ===================== MOTORES / ESC =====================
#define PWM_FREQ            400     // Hz. Los ESC 30A (SimonK/BLHeli) lo aceptan;
                                    // a 50 Hz el retardo de actuación llega a 20 ms.
#define PWM_RESOLUTION      12      // bits
#define PULSO_MIN_US        1000    // ancho de pulso de paro
#define PULSO_MAX_US        2000    // ancho de pulso de máximo
// En 64 bits: con 12 bits y 400 Hz el producto ya roza el límite de uint32_t, y
// subir la resolución lo desbordaría en silencio.
#define US_A_DUTY(us)       ((uint32_t)(((uint64_t)(us) * (1ULL << PWM_RESOLUTION) * PWM_FREQ) / 1000000ULL))
#define PWM_MIN             US_A_DUTY(PULSO_MIN_US)
#define PWM_MAX             US_A_DUTY(PULSO_MAX_US)

#define MOTOR_IDLE          0.08f   // ralentí mínimo con el dron armado: por debajo de
                                    // esto un ESC puede desincronizarse y no re-arrancar
#define THROTTLE_HOVER      0.36f   // estimado: 1150 g / (4 x ~800 g de empuje). Ajustar en banco
#define RAMP_TIME_MS        800     // rampa de arranque al armar

// ===================== ESTIMACIÓN DE ACTITUD =====================
#define FILTRO_COMP_TAU     0.50f   // s. Constante de tiempo del filtro complementario
                                    // (independiente de la tasa del lazo)
#define GYRO_LPF_HZ         40.0f   // filtro del giróscopo para el término D

// ===================== CONTROL DE ACTITUD (cascada) =====================
// Lazo externo: error de ángulo -> velocidad angular deseada [deg/s por deg]
#define KP_ANG_ROLL         4.5f
#define KP_ANG_PITCH        4.5f
#define RATE_SP_MAX         150.0f  // deg/s máximos que puede pedir el lazo externo

// Lazo interno: error de velocidad angular -> corrección de throttle [por deg/s]
#define KP_RATE_ROLL        0.0020f
#define KI_RATE_ROLL        0.0040f
#define KD_RATE_ROLL        0.00006f
#define KP_RATE_PITCH       0.0020f
#define KI_RATE_PITCH       0.0040f
#define KD_RATE_PITCH       0.00006f
#define KP_RATE_YAW         0.0040f
#define KI_RATE_YAW         0.0060f
#define KD_RATE_YAW         0.0f

#define CORR_MAX            0.35f   // saturación de cada corrección (fracción de throttle)
#define I_MAX_ACTITUD       0.10f   // anti-windup del integrador de velocidad angular

// Límites de los setpoints que se aceptan por ROS 2
#define ANG_SP_MAX          15.0f   // grados de inclinación comandada
#define YAW_RATE_SP_MAX     90.0f   // deg/s de guiñada comandada

// ===================== LEY DE CONTROL =====================
#define LEY_PID             0
#define LEY_SMC             1
#define LEY_CONTROL         LEY_SMC

// ===================== SMC (ver smc.h) =====================
// Modelo estimado: robot_description/config/f450.yaml. F450, A2212 1000 KV,
// hélice 1045, LiPo 3S 2200 mAh.
#define SMC_MASA            1.15f   // kg, con batería
#define SMC_IXX             0.0115f // kg*m^2
#define SMC_IYY             0.0115f
#define SMC_IZZ             0.0218f
#define SMC_BRAZO           0.1589f // m, brazo de palanca de cada motor en X y en Y
#define SMC_EMPUJE_MAX      8.3f    // N por motor
#define SMC_K_YAW           0.016f  // m, par de reacción / empuje

// Roll y pitch. rho debe superar la perturbación más el error de modelo; la
// pendiente kd + rho/phi la limita el retardo del motor.
#define SMC_LAMBDA_ANG      4.5f    // 1/s
#define SMC_KD_ANG          0.25f   // N*m por rad/s
#define SMC_RHO_ANG         0.15f   // N*m
#define SMC_PHI_ANG         0.50f   // rad/s
#define SMC_P_ANG           1.0f    // N*m por rad, ganancia de adaptación del sesgo
#define SMC_SESGO_MAX_ANG   0.30f   // N*m

// Guiñada: solo velocidad, s = r - r_deseada
#define SMC_KD_YAW          0.10f   // N*m por rad/s
#define SMC_RHO_YAW         0.03f   // N*m
#define SMC_PHI_YAW         0.50f   // rad/s
#define SMC_P_YAW           0.2f    // N*m por rad
#define SMC_SESGO_MAX_YAW   0.10f   // N*m

// Altura
#define SMC_LAMBDA_Z        1.2f    // 1/s
#define SMC_KD_Z            3.0f    // N por m/s
#define SMC_RHO_Z           2.5f    // N
#define SMC_PHI_Z           0.50f   // m/s
#define SMC_P_Z             3.0f    // N por m
#define SMC_SESGO_MAX_Z     3.0f    // N

// Generador de referencia de altura
#define SMC_OMEGA_REF_Z     2.0f    // rad/s
#define SMC_ACC_REF_MAX     2.0f    // m/s^2

// ===================== CONTROL DE ALTURA =====================
#define ALTURA_LIDAR_MAX    1.5f    // m: hasta aquí el LiDAR es la fuente principal
#define ALTURA_MEZCLA_MIN   1.0f    // m: desde aquí se mezcla progresivamente con el baro
#define ALTURA_OBJETIVO_DEF 1.0f    // m: hover por defecto
#define ALTURA_SP_MAX       3.0f    // m: setpoint máximo aceptado por ROS 2

// Velocidad vertical por filtro complementario: la aceleracion de la IMU predice
// y la derivada de la altura corrige. Derivar solo el barometro (sigma ~8 cm)
// metia ~0.4 m/s de ruido en vz, y con KP_VEL_Z=5 eso son 2 m/s^2 de basura en el
// mando: medido en simulacion, el dron oscilaba +-0.2 m sobre el alcance del LiDAR.
#define K_VZ                1.5f    // 1/s, ganancia de correccion
#define VZ_MAX_CRUDA        3.0f    // m/s, acota la derivada de la altura
#define ACC_Z_LPF_HZ        5.0f    // filtro de la aceleración vertical estimada

// Cascada de altura (misma estructura y ganancias de partida que el Lab_rav_03:
// positionZ kp=1.0 kd=0.2, velocityZ kp=5.0 kd=1.0).
//
// pos_z: error de altura [m] -> velocidad vertical deseada [m/s]
#define KP_POS_Z            1.0f
#define KI_POS_Z            0.0f
#define KD_POS_Z            0.2f
#define TAU_POS_Z           0.20f   // s, filtro de la consigna de altura
#define VEL_Z_MAX           0.50f   // m/s, acota la velocidad vertical pedida

// vel_z: error de velocidad [m/s] -> aceleración vertical deseada [m/s^2]
#define KP_VEL_Z            5.0f
#define KI_VEL_Z            1.0f    // el laboratorio usa 0, pero aquí absorbe el
                                    // error de estimación de THROTTLE_HOVER
#define KD_VEL_Z            1.0f
#define TAU_VEL_Z           0.05f
#define I_MAX_VEL_Z         3.0f    // m/s^2 que puede aportar el integrador
#define ACC_Z_MAX           4.0f    // m/s^2 (~0.4 g) de autoridad vertical

// ===================== EMERGENCIA / FAILSAFE =====================
#define TIMEOUT_CMD_MS      500     // ms sin /dron/cmd_actitud -> aterrizaje
#define TIMEOUT_AGENTE_MS   1000    // ms sin agente micro-ROS -> aterrizaje
#define MAX_FALLOS_IMU      (FREQ_CONTROL_HZ / 5)   // ~0.2 s sin IMU -> aterrizaje
#define VELOCIDAD_DESCENSO  0.4f    // m/s
#define ALTURA_CORTE_MOTORES 0.10f  // m: a esta altura se cortan los motores

// ===================== DEPURACIÓN =====================
// Con transporte serie, micro-ROS es dueño de Serial: imprimir ahí corrompe
// la trama XRCE-DDS. Por eso LOG() solo existe en la variante wifi.
#if defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
  #define LOG(...)   do {} while (0)
#else
  #define LOG(...)   Serial.printf(__VA_ARGS__)
#endif

#endif
