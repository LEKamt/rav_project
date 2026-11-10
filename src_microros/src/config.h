#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ===================== PINES =====================
// Montaje real del 10-nov (hardware/CABLEADO_S3.md, el mismo de VueloF450.ino)
#define I2C_SDA             1       // bus Wire: IMU (y BMP280 si la GY-91 lo trae)
#define I2C_SCL             39
#define I2C1_SDA            4       // bus Wire1: VL53L0X
#define I2C1_SCL            5

#define MOTOR1_PIN          8       // frente izquierdo, horario
#define MOTOR2_PIN          17      // frente derecho, antihorario
#define MOTOR3_PIN          21      // trasero derecho, horario
#define MOTOR4_PIN          18      // trasero izquierdo, antihorario

#define PIN_BOTON_EMERGENCIA 10  // Pulsador externo opcional (activo bajo)

// Ultrasonido (HC-SR04 o similar), directo al S3: la PCB no tiene entrada para
// él. Con un HC-SR04 de 5 V, divisor 1 k / 2 k en el eco.
#define PIN_SONAR_TRIG      6
#define PIN_SONAR_ECHO      7

// ===================== DIRECCIONES I2C =====================
// GY-91 en J6: MPU-9250 (o clon MPU-6500) + BMP280. El BMP280 está en 0x76 o en
// 0x77 según el módulo: se prueban las dos.
#define MPU_ADDR            0x68
#define BMP280_ADDR_1       0x76
#define BMP280_ADDR_2       0x77
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
#define FREQ_CONTROL_HZ     500     // tarea rápida: IMU + actitud + mezcla + PWM (core 1)
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

// ===================== IMU: EJES Y CALIBRACIÓN =====================
// Ejes del cuerpo (FLU: x adelante, y izquierda, z arriba) a partir de los del
// sensor s[0..2]. Dependen de cómo quedó montada la GY-91: comprobar en banco con
// la telemetría /dron/imu antes de armar (ver hardware/CABLEADO_S3.md):
//   nivelado          -> acc = (0, 0, +1 g)
//   nariz abajo       -> acc.x negativo
//   lado izq. arriba  -> acc.y positivo
// El firmware no arma si en reposo el acelerómetro no apunta a +z del cuerpo.
#define IMU_CUERPO_X(s)     ((s)[0])
#define IMU_CUERPO_Y(s)     ((s)[1])
#define IMU_CUERPO_Z(s)     ((s)[2])

// Corrección del acelerómetro en ejes del cuerpo, de la calibración de 6
// posiciones: a = (medida - offset) * escala [g]
#define ACC_OFFSET_X        0.0f
#define ACC_OFFSET_Y        0.0f
#define ACC_OFFSET_Z        0.0f
#define ACC_ESCALA_X        1.0f
#define ACC_ESCALA_Y        1.0f
#define ACC_ESCALA_Z        1.0f

// Sesgo del giróscopo: se promedia al arrancar, con el dron quieto
#define GIRO_CALIB_MS       2000
#define GIRO_CALIB_QUIETO   2.0f    // deg/s: desviación máxima para aceptar la calibración
#define GIRO_CALIB_INTENTOS 5

// ===================== FILTROS DE LA IMU =====================
// DLPF de la MPU (registro CONFIG): 1 = 184 Hz, 2 = 92 Hz, 3 = 41 Hz. Con 92 Hz
// el filtro de hardware deja pasar la banda del control y el biquad hace el resto.
#define MPU_DLPF_CFG        2
#define GYRO_LPF_HZ         60.0f   // pasabajos Butterworth del giro (P y D del lazo)
// Notch de la vibración de las hélices. Apagado hasta medir el espectro con
// hélices (la frecuencia de paso de pala del F450 en hover cae cerca de 80-100 Hz).
#define GYRO_NOTCH_ON       0
#define GYRO_NOTCH_HZ       90.0f
#define GYRO_NOTCH_BW       30.0f
#define ACC_LPF_HZ          20.0f   // acelerómetro hacia el AHRS y el Kalman
#define ACC_Z_LPF_HZ        5.0f    // aceleración vertical para el término D del PID de altura

// ===================== ESTIMACIÓN DE ACTITUD (Mahony) =====================
#define AHRS_KP             1.0f    // 1/s: tiempo de corrección por gravedad ~1 s
#define AHRS_KI             0.02f   // 1/s^2: sesgo residual del giro
#define AHRS_RECHAZO_G      0.15f   // no corrige con el acelerómetro si | |a| - 1 g | > esto
#define INCLINACION_ARMADO  10.0f   // grados: no arma más inclinado que esto
#define ACC_ARMADO_ERR_G    0.20f   // g: tolerancia de acc = (0, 0, 1) al armar

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

// ===================== ESTIMACIÓN DE ALTURA (Kalman) =====================
// Estado [z, vz, sesgo del baro]; ver kalman_altura.h. Las sigmas son de partida:
// ajustarlas con los registros reales de /dron/alturas.
#define TOF_MIN_M           0.02f
#define TOF_MAX_M           1.00f   // el VL53L0X de este dron no pasa de ~1 m
#define SONAR_MIN_M         0.05f
#define SONAR_MAX_M         4.00f
#define SONAR_PERIODO_MS    60      // entre disparos: deja morir el eco anterior
#define BARO_PERIODO_MS     40      // 25 Hz
#define SIGMA_TOF           0.02f   // m
#define SIGMA_SONAR         0.05f   // m
#define SIGMA_BARO          0.35f   // m, el chorro de las hélices lo empeora
#define KF_SIGMA_ACC        0.50f   // m/s^2
#define KF_SIGMA_SESGO      0.02f   // m/sqrt(s)
#define KF_COMPUERTA        3.0f    // desviaciones estándar
#define KF_MAX_RECHAZOS     15
#define COS_INCL_DISTANCIA  0.94f   // cos(20 deg): más inclinado, el ToF y el sonar no miran al suelo

// ===================== CONTROL DE ALTURA =====================
#define ALTURA_OBJETIVO_DEF 1.0f    // m: hover por defecto
#define ALTURA_SP_MAX       3.0f    // m: setpoint máximo aceptado por ROS 2

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
