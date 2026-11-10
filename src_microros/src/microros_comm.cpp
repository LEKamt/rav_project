#include "microros_comm.h"

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/vector3.h>

#include "config.h"
#include "sensores.h"
#include "control_actitud.h"
#include "control_vuelo.h"
#include "motores.h"
#include "emergencia.h"

#if defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
  #include <WiFi.h>
#endif

// ===================== ESTADO DE LA SESIÓN =====================
enum EstadoAgente {
    ESPERANDO_AGENTE,
    AGENTE_DISPONIBLE,
    AGENTE_CONECTADO,
    AGENTE_DESCONECTADO
};
static EstadoAgente estadoAgente = ESPERANDO_AGENTE;

#define EJECUTAR_CADA(MS, X) do {                       \
    static unsigned long t_prev = 0;                    \
    if (millis() - t_prev >= (unsigned long)(MS)) {     \
        X;                                              \
        t_prev = millis();                              \
    }                                                   \
} while (0)

#define RCCHECK(fn) do { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) return false; } while (0)
#define RCSOFT(fn)  do { rcl_ret_t rc = fn; (void)rc; } while (0)

// ===================== ENTIDADES =====================
static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t nodo;
static rclc_executor_t executor;

static rcl_publisher_t pub_imu;
static rcl_publisher_t pub_alturas;
static rcl_publisher_t pub_control;
static rcl_publisher_t pub_motores;
static rcl_publisher_t pub_estado;

static rcl_subscription_t sub_cmd_actitud;
static rcl_subscription_t sub_cmd_altura;
static rcl_subscription_t sub_armar;
static rcl_subscription_t sub_emergencia;
static rcl_subscription_t sub_ganancias;

static sensor_msgs__msg__Imu msg_imu;
static std_msgs__msg__Float32MultiArray msg_alturas;
static std_msgs__msg__Float32MultiArray msg_control;
static std_msgs__msg__Float32MultiArray msg_motores;
static std_msgs__msg__Int32 msg_estado;

static geometry_msgs__msg__Vector3 msg_cmd_actitud;
static std_msgs__msg__Float32 msg_cmd_altura;
static std_msgs__msg__Bool msg_armar;
static std_msgs__msg__Bool msg_emergencia;
static std_msgs__msg__Float32MultiArray msg_ganancias;

// Buffers estáticos: micro-ROS no debe asignar memoria dinámica en vuelo
static float buf_alturas[8];
static float buf_control[8];
static float buf_motores[8];
static float buf_ganancias[8];
static std_msgs__msg__MultiArrayDimension dim_ganancias[1];
static char label_ganancias[24];
static char imu_frame_id[] = "base_link";

static bool horaSincronizada = false;
static bool soporteOK = false;   // rclc_support_init llegó a completarse

// ===================== UTILIDADES =====================
static void prepararArrayPub(std_msgs__msg__Float32MultiArray *msg, float *buf, size_t n) {
    msg->data.data = buf;
    msg->data.capacity = n;
    msg->data.size = n;
    msg->layout.dim.data = NULL;
    msg->layout.dim.capacity = 0;
    msg->layout.dim.size = 0;
    msg->layout.data_offset = 0;
}

static void sellarTiempo(builtin_interfaces__msg__Time *stamp) {
    if (horaSincronizada) {
        int64_t ns = rmw_uros_epoch_nanos();
        stamp->sec = (int32_t)(ns / 1000000000LL);
        stamp->nanosec = (uint32_t)(ns % 1000000000LL);
    } else {
        unsigned long ms = millis();
        stamp->sec = (int32_t)(ms / 1000UL);
        stamp->nanosec = (uint32_t)((ms % 1000UL) * 1000000UL);
    }
}

// ===================== CALLBACKS =====================
// Flujo continuo de comandos de actitud: hace también de latido del enlace.
static void cb_cmd_actitud(const void *msgin) {
    const geometry_msgs__msg__Vector3 *m = (const geometry_msgs__msg__Vector3 *)msgin;
    tUltimoComando = millis();

    // Durante el aterrizaje de emergencia el descenso es nivelado y no se aceptan
    // consignas de inclinación, aunque el enlace siga vivo.
    if (modoVuelo == MODO_ATERRIZAJE_EMERGENCIA) return;

    setpointRoll    = constrain((float)m->x, -ANG_SP_MAX, ANG_SP_MAX);
    setpointPitch   = constrain((float)m->y, -ANG_SP_MAX, ANG_SP_MAX);
    setpointYawRate = constrain((float)m->z, -YAW_RATE_SP_MAX, YAW_RATE_SP_MAX);
}

static void cb_cmd_altura(const void *msgin) {
    const std_msgs__msg__Float32 *m = (const std_msgs__msg__Float32 *)msgin;
    tUltimoComando = millis();
    // Durante el aterrizaje el setpoint lo manda la rutina de descenso
    if (modoVuelo == MODO_ATERRIZAJE_EMERGENCIA) return;
    if (m->data > 0.05f && m->data <= ALTURA_SP_MAX) {
        alturaObjetivo = m->data;
        setpointManual = true;
    }
}

static void cb_armar(const void *msgin) {
    const std_msgs__msg__Bool *m = (const std_msgs__msg__Bool *)msgin;
    tUltimoComando = millis();
    if (m->data) armarDron();
    else         desarmarDron();
}

static void cb_emergencia(const void *msgin) {
    const std_msgs__msg__Bool *m = (const std_msgs__msg__Bool *)msgin;
    tUltimoComando = millis();
    if (m->data) pedirAterrizaje(FAILSAFE_ROS);
}

// Sintonización sin reflashear. Solo con el dron desarmado: cambiar ganancias en
// vuelo es la forma más rápida de romper el dron.
static void cb_ganancias(const void *msgin) {
    const std_msgs__msg__Float32MultiArray *m = (const std_msgs__msg__Float32MultiArray *)msgin;
    tUltimoComando = millis();
    if (modoVuelo != MODO_DESARMADO) {
        LOG("[GANANCIAS] Ignoradas: el dron no esta desarmado\n");
        return;
    }
    if (m->data.size < 6) return;
    setGananciasActitud(m->data.data[0], m->data.data[1], m->data.data[2],
                        m->data.data[3], m->data.data[4], m->data.data[5]);
    LOG("[GANANCIAS] kpAng=%.3f kpRate=%.5f kiRate=%.5f kdRate=%.6f kpYaw=%.5f kiYaw=%.5f\n",
        m->data.data[0], m->data.data[1], m->data.data[2],
        m->data.data[3], m->data.data[4], m->data.data[5]);
}

// ===================== CREACIÓN / DESTRUCCIÓN =====================
static bool crearEntidades() {
    allocator = rcl_get_default_allocator();
    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
    soporteOK = true;   // a partir de aquí el contexto es válido y se puede destruir

    RCCHECK(rclc_node_init_default(&nodo, MICROROS_NODO, MICROROS_NAMESPACE, &support));

    // Telemetría de alta frecuencia: best effort (si se pierde una muestra, da igual)
    RCCHECK(rclc_publisher_init_best_effort(&pub_imu, &nodo,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "dron/imu"));
    RCCHECK(rclc_publisher_init_best_effort(&pub_alturas, &nodo,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "dron/alturas"));
    RCCHECK(rclc_publisher_init_best_effort(&pub_control, &nodo,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "dron/control"));
    RCCHECK(rclc_publisher_init_best_effort(&pub_motores, &nodo,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "dron/motores"));
    // El estado sí es fiable: un cambio de modo no se puede perder
    RCCHECK(rclc_publisher_init_default(&pub_estado, &nodo,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "dron/estado"));

    RCCHECK(rclc_subscription_init_best_effort(&sub_cmd_actitud, &nodo,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "dron/cmd_actitud"));
    RCCHECK(rclc_subscription_init_default(&sub_cmd_altura, &nodo,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "dron/cmd_altura"));
    RCCHECK(rclc_subscription_init_default(&sub_armar, &nodo,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "dron/armar"));
    RCCHECK(rclc_subscription_init_default(&sub_emergencia, &nodo,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "dron/emergencia"));
    RCCHECK(rclc_subscription_init_default(&sub_ganancias, &nodo,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "dron/cmd_ganancias"));

    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd_actitud, &msg_cmd_actitud,
                                           &cb_cmd_actitud, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd_altura, &msg_cmd_altura,
                                           &cb_cmd_altura, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_armar, &msg_armar,
                                           &cb_armar, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_emergencia, &msg_emergencia,
                                           &cb_emergencia, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_ganancias, &msg_ganancias,
                                           &cb_ganancias, ON_NEW_DATA));

    // Sincronización de reloj para las marcas de tiempo de /dron/imu
    horaSincronizada = (RMW_RET_OK == rmw_uros_sync_session(1000));

    LOG("[micro-ROS] Entidades creadas (hora %s)\n", horaSincronizada ? "sincronizada" : "local");
    return true;
}

static void destruirEntidades() {
    // Sin soporte inicializado, el contexto no tiene implementación y pedirle el
    // rmw_context reventaría.
    if (!soporteOK) return;

    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    RCSOFT(rcl_publisher_fini(&pub_imu, &nodo));
    RCSOFT(rcl_publisher_fini(&pub_alturas, &nodo));
    RCSOFT(rcl_publisher_fini(&pub_control, &nodo));
    RCSOFT(rcl_publisher_fini(&pub_motores, &nodo));
    RCSOFT(rcl_publisher_fini(&pub_estado, &nodo));
    RCSOFT(rcl_subscription_fini(&sub_cmd_actitud, &nodo));
    RCSOFT(rcl_subscription_fini(&sub_cmd_altura, &nodo));
    RCSOFT(rcl_subscription_fini(&sub_armar, &nodo));
    RCSOFT(rcl_subscription_fini(&sub_emergencia, &nodo));
    RCSOFT(rcl_subscription_fini(&sub_ganancias, &nodo));
    RCSOFT(rclc_executor_fini(&executor));
    RCSOFT(rcl_node_fini(&nodo));
    RCSOFT(rclc_support_fini(&support));

    horaSincronizada = false;
    soporteOK = false;
}

// ===================== PUBLICACIÓN =====================
static void publicarImu() {
    sellarTiempo(&msg_imu.header.stamp);

    // Cuaternión a partir de los ángulos del AHRS (ZYX)
    float cr = cosf(angRoll * DEG_TO_RAD * 0.5f), sr = sinf(angRoll * DEG_TO_RAD * 0.5f);
    float cp = cosf(angPitch * DEG_TO_RAD * 0.5f), sp = sinf(angPitch * DEG_TO_RAD * 0.5f);
    float cy = cosf(angYaw * DEG_TO_RAD * 0.5f), sy = sinf(angYaw * DEG_TO_RAD * 0.5f);
    msg_imu.orientation.w = cr * cp * cy + sr * sp * sy;
    msg_imu.orientation.x = sr * cp * cy - cr * sp * sy;
    msg_imu.orientation.y = cr * sp * cy + sr * cp * sy;
    msg_imu.orientation.z = cr * cp * sy - sr * sp * cy;

    msg_imu.angular_velocity.x = velRoll * DEG_TO_RAD;
    msg_imu.angular_velocity.y = velPitch * DEG_TO_RAD;
    msg_imu.angular_velocity.z = velYaw * DEG_TO_RAD;

    msg_imu.linear_acceleration.x = ax_g * 9.80665f;
    msg_imu.linear_acceleration.y = ay_g * 9.80665f;
    msg_imu.linear_acceleration.z = az_g * 9.80665f;

    RCSOFT(rcl_publish(&pub_imu, &msg_imu, NULL));
}

static void publicarRapido() {
    buf_alturas[0] = alturaControl;
    buf_alturas[1] = alturaLiDAR;
    buf_alturas[2] = alturaBaro;
    buf_alturas[3] = consignaAltura;        // consigna de altura ya filtrada
    buf_alturas[4] = velVertical;
    // El comandado se publica aparte para que la estación pueda confirmar que el
    // setpoint llegó: si se envía antes de que exista la sesión, se pierde.
    buf_alturas[5] = alturaObjetivo;
    buf_alturas[6] = alturaSonar;           // 0 si la medida no es válida
    buf_alturas[7] = sesgoBaro;             // sesgo del baro estimado por el Kalman
    RCSOFT(rcl_publish(&pub_alturas, &msg_alturas, NULL));

    buf_control[0] = throttleAplicado;
    buf_control[1] = roll_corr;
    buf_control[2] = pitch_corr;
    buf_control[3] = yaw_corr;
    buf_control[4] = vzDeseada;             // velocidad vertical deseada [m/s]
    buf_control[5] = accDeseada;            // aceleración vertical deseada [m/s^2]
    buf_control[6] = rateSpRoll;
    buf_control[7] = rateSpPitch;
    RCSOFT(rcl_publish(&pub_control, &msg_control, NULL));
}

static void publicarLento() {
    buf_motores[0] = pwmMotor1;
    buf_motores[1] = pwmMotor2;
    buf_motores[2] = pwmMotor3;
    buf_motores[3] = pwmMotor4;
    // Diagnóstico del lazo rápido y de los sensores (sin crear otro tópico:
    // micro-ROS tiene un número fijo de entidades)
    buf_motores[4] = (float)tiempoIMUMaxUs;
    buf_motores[5] = (float)fallosIMUTotal;
    buf_motores[6] = (float)(estadoSensores.imu | estadoSensores.baro << 1 |
                             estadoSensores.tof << 2 | estadoSensores.sonar << 3 |
                             giroCalibrado << 4);
    buf_motores[7] = (float)estadoSensores.whoAmI;
    RCSOFT(rcl_publish(&pub_motores, &msg_motores, NULL));
}

// estado = modo + 10 * motivo_failsafe
static void publicarEstado() {
    msg_estado.data = (int32_t)modoVuelo + 10 * (int32_t)motivoFailsafe;
    RCSOFT(rcl_publish(&pub_estado, &msg_estado, NULL));
}

// ===================== API =====================
void iniciarComunicaciones() {
#if defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
    IPAddress ipAgente;
    ipAgente.fromString(MICROROS_AGENTE_IP);
    static char ssid[] = MICROROS_WIFI_SSID;
    static char pass[] = MICROROS_WIFI_PASS;
    set_microros_wifi_transports(ssid, pass, ipAgente, MICROROS_AGENTE_PUERTO);
    Serial.begin(115200);   // con transporte wifi, Serial queda libre para depurar
    LOG("\n[micro-ROS] Transporte WiFi -> %s:%d\n", MICROROS_AGENTE_IP, MICROROS_AGENTE_PUERTO);
#else
    // A 921600 baudios los 256 B por defecto del buffer de recepción se desbordan
    // con las ráfagas del agente (sesión, ACK de los tópicos fiables).
    Serial.setRxBufferSize(2048);
    Serial.begin(MICROROS_SERIAL_BAUD);
    set_microros_serial_transports(Serial);
#endif

    prepararArrayPub(&msg_alturas, buf_alturas, 8);
    prepararArrayPub(&msg_control, buf_control, 8);
    prepararArrayPub(&msg_motores, buf_motores, 8);

    // El mensaje suscrito sí necesita capacidad para lo que llegue del PC
    msg_ganancias.data.data = buf_ganancias;
    msg_ganancias.data.capacity = 8;
    msg_ganancias.data.size = 0;
    msg_ganancias.layout.dim.data = dim_ganancias;
    msg_ganancias.layout.dim.capacity = 1;
    msg_ganancias.layout.dim.size = 0;
    dim_ganancias[0].label.data = label_ganancias;
    dim_ganancias[0].label.capacity = sizeof(label_ganancias);
    dim_ganancias[0].label.size = 0;

    msg_imu.header.frame_id.data = imu_frame_id;
    msg_imu.header.frame_id.capacity = sizeof(imu_frame_id);
    msg_imu.header.frame_id.size = strlen(imu_frame_id);

    estadoAgente = ESPERANDO_AGENTE;
}

bool agenteConectado() {
    return estadoAgente == AGENTE_CONECTADO;
}

void tickComunicaciones() {
    switch (estadoAgente) {
        case ESPERANDO_AGENTE:
            EJECUTAR_CADA(500, estadoAgente =
                (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENTE_DISPONIBLE : ESPERANDO_AGENTE);
            break;

        case AGENTE_DISPONIBLE:
            estadoAgente = crearEntidades() ? AGENTE_CONECTADO : AGENTE_DESCONECTADO;
            break;

        case AGENTE_CONECTADO: {
            // El ping bloquea hasta 100 ms y esta tarea también lleva la altura.
            // Si acaban de llegar comandos, el enlace ya está demostrado vivo y no
            // hace falta pingear: en vuelo normal esto no bloquea nunca.
            EJECUTAR_CADA(TIMEOUT_AGENTE_MS, {
                if (millis() - tUltimoComando > 300 &&
                    RMW_RET_OK != rmw_uros_ping_agent(50, 2)) {
                    LOG("[micro-ROS] Agente perdido\n");
                    estadoAgente = AGENTE_DESCONECTADO;
                }
            });
            if (estadoAgente != AGENTE_CONECTADO) break;

            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2));

            EJECUTAR_CADA(1000 / PUB_IMU_HZ,    publicarImu());
            EJECUTAR_CADA(1000 / PUB_RAPIDO_HZ, publicarRapido());
            EJECUTAR_CADA(1000 / PUB_LENTO_HZ,  publicarLento());

            static ModoVuelo modoPrev = MODO_DESARMADO;
            static MotivoFailsafe motivoPrev = FAILSAFE_NINGUNO;
            if (modoVuelo != modoPrev || motivoFailsafe != motivoPrev) {
                modoPrev = modoVuelo;
                motivoPrev = motivoFailsafe;
                publicarEstado();
            } else {
                EJECUTAR_CADA(1000 / PUB_LENTO_HZ, publicarEstado());
            }
            break;
        }

        case AGENTE_DESCONECTADO:
            destruirEntidades();
            estadoAgente = ESPERANDO_AGENTE;
            break;
    }
}
