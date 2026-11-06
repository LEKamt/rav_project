# Estimación de altura y sus fuentes. Sustituye al Float32MultiArray de
# /dron/alturas, cuyos índices había que documentar en el README.

std_msgs/Header header
float32 fusionada         # m, la que usa el control
float32 lidar             # m, 0 si la medida no es válida
float32 baro              # m, relativa al cero del arranque
float32 consigna          # m, consigna ya filtrada que persigue el PID de posición
float32 velocidad_vertical  # m/s, + hacia arriba
float32 objetivo          # m, lo último que aceptó por /dron/cmd_altura
bool lidar_valido
