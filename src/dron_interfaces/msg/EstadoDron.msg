# Estado de la máquina de vuelo. Equivale al entero de /dron/estado que publica el
# firmware (modo + 10*motivo), pero con los campos nombrados.

# Modos
uint8 DESARMADO = 0
uint8 ARMADO = 1
uint8 ATERRIZAJE = 2
uint8 DETENIDO = 3

# Motivos de failsafe
uint8 FS_NINGUNO = 0
uint8 FS_BOTON = 1
uint8 FS_TIMEOUT_CMD = 2
uint8 FS_AGENTE = 3
uint8 FS_ROS = 4
uint8 FS_IMU = 5

std_msgs/Header header
uint8 modo
uint8 motivo_failsafe
bool setpoint_fijado      # el firmware exige fijar altura antes de armar
string descripcion        # texto legible, para registro y paneles
