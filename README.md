# FirePigeon: dron F450 con ESP32-S3

Proyecto de Robótica Avanzada (UTEC, 2026-2). Workspace ROS 2 Humble + Gazebo
Classic con el dron simulado; el control de vuelo real corre en un ESP32-S3 con
micro-ROS. La ley de control es SMC (control por modo deslizante) y el PID se
conserva para comparar.

Hardware: chasis F450, motores A2212 1000 KV, hélices 1045, LiPo 3S 2200 mAh. La PCB
se conecta al S3 por cables: ver [hardware/CABLEADO_S3.md](hardware/CABLEADO_S3.md).

## Estructura

```
rav_project/
├── docs/curso/      diapositivas, notas y notebooks del curso
├── hardware/        esquemático, capas de la PCB y cableado al S3
├── src_microros/    firmware del ESP32-S3 (PlatformIO)
└── rav_ws/src/
    ├── robot_description/    URDF, mallas y parámetros físicos del F450
    ├── robot_simulation/     mundos, modelos y lanzador de Gazebo
    ├── robot_bringup/        lanzadores completos, estación de tierra, puente del firmware
    ├── robot_control/        leyes de control y controlador simulado (hace de ESP32-S3)
    ├── robot_sensors/        diagnóstico de sensores
    ├── robot_perception/     reservado: el dron no lleva cámara
    ├── robot_estimation/     estimación de actitud y altura, TF
    ├── robot_navigation/     misiones de altura
    ├── robot_interfaces/     mensajes, servicio y acción propios
    └── robot_tests/          pruebas unitarias, de integración y escenarios
```

Los paquetes Python usan layout `src/<paquete>/`, con `package_dir` en `setup.py`.

## Compilar

```bash
cd rav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Simular

```bash
ros2 launch robot_simulation simulacion.launch.py            # SMC
ros2 launch robot_simulation simulacion.launch.py ley:=pid    # PID
ros2 run robot_bringup estacion_tierra      # en otra terminal
```

Las ganancias se ajustan en `rav_ws/src/robot_control/config/ganancias.yaml`. Las que
funcionen hay que pasarlas a `src_microros/src/config.h`.

En el firmware, la ley se elige con `LEY_CONTROL` en `config.h`.

## Probar

```bash
cd rav_ws
python3 -m pytest src/robot_tests/unit -q
python3 -m pytest src/robot_tests/integration -q -s   # arranca Gazebo, tarda minutos
```
