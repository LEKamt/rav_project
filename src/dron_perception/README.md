# dron_perception

**Vacío a propósito.** El F450 de este proyecto no lleva cámara ni sensor de
flujo óptico, así que no hay nada que percibir más allá de lo que ya cubren
`dron_sensors` (IMU, LiDAR de altura, barómetro) y `dron_localization`
(estimación de actitud y altura).

El paquete existe para que el esquema del workspace esté completo y para que
quede escrito qué iría aquí el día que se añada hardware:

| Si se añade                        | Qué iría en este paquete |
| ---------------------------------- | ------------------------ |
| Cámara mirando al suelo            | Flujo óptico para estimar velocidad horizontal, que es lo que hoy falta para que el dron mantenga posición y no solo altura |
| Cámara + marcador (ArUco/AprilTag) | Detección del marcador para aterrizaje preciso sobre la plataforma |
| Cámara frontal                     | Detección de obstáculos |

Con cualquiera de esas, lo estimado se publicaría para que `dron_localization`
lo fusione y `dron_navigation` lo use; el control del ESP32-S3 no cambiaría:
seguiría recibiendo consignas de actitud y de altura.

Las carpetas `config/`, `launch/`, `models/` y `dron_perception/` están creadas
con `.gitkeep` para no romper la estructura del workspace.
