# Señales internas del control, las que hacen falta para sintonizar. Sustituye al
# Float32MultiArray de /dron/control.

std_msgs/Header header

float32 throttle                 # mando común aplicado, 0-1
float32 correccion_roll          # fracción de throttle
float32 correccion_pitch
float32 correccion_yaw

float32 velocidad_vertical_deseada   # m/s, salida del PID de posición
float32 aceleracion_vertical_deseada # m/s^2, salida del PID de velocidad

float32 rate_deseado_roll        # deg/s, salida del lazo externo de actitud
float32 rate_deseado_pitch

float32[4] motores               # cuentas de PWM de M1..M4
