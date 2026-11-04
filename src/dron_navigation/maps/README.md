# maps/

Vacío a propósito.

El esquema de paquetes reserva `maps/` para los mapas de navegación (SLAM,
costmaps de Nav2). Este dron no los usa: no tiene estimación de posición
horizontal, así que no hay nada que mapear ni sobre lo que planificar rutas.
Lo que sí hay son **perfiles de altura**, y esos viven en `param/`.

Si algún día se le añade flujo óptico o GPS, aquí irían los mapas y en `param/`
la configuración de Nav2.
