# PrimerVueloWiFi_PIDv2: pruebas y sintonía del F450

F450 con A2212 1000 KV, hélices 1045, LiPo 3S y ESP32-S3. Placa en Arduino IDE: **ESP32S3 Dev Module**.
Página de control: red `F450-RAV` (clave `f450vuelo`), abrir `http://192.168.4.1`.

Unidades de las ganancias: fracción del acelerador. 1.0 equivale a 850 µs (de 1000 a 1850 µs).
- `Kp = 0.007`: 6 µs por grado de error.
- `Kd = 0.0004`: 0.34 µs por grado/s.
- `Kp_yaw = 0.0015`: 1.3 µs por grado/s de giro.

## 0. Sin hélices: arranque

1. Subir el sketch y abrir el monitor serie a 115200 con el dron quieto y nivelado.
2. Debe mostrar `az en reposo` ≈ **+1.00 g**. Si sale negativo, la IMU está boca abajo: avisar antes de seguir.
3. En la página, **Lazo Hz** debe marcar ≈ 500.

## 1. Sin hélices: ESC a 400 Hz

Pulsar ARMAR (15 %) y subir al 30 %. Los 4 motores deben girar parejo, sin tartamudear, y arrancar juntos.
Si alguno falla, cambiar `PWM_FREQ` a 50, volver a subir el sketch y repetir. PARAR.

## 2. Sin hélices: signos (lo más importante)

Con el dron armado al 15 %, mirar los pulsos **M1–M4** en la página.

| Prueba | Qué hacer | Resultado correcto |
|---|---|---|
| P de roll | Inclinar ~15° hacia el lado de M1 y M4 y sostener | M1 y M4 **suben**, M2 y M3 bajan |
| P de pitch | Inclinar ~15° hacia el lado de M3 y M4 y sostener | M3 y M4 **suben**, M1 y M2 bajan |
| Ángulo sin rebote | Inclinar rápido y soltar a mano | En la gráfica el ángulo se mueve **directo** al lado correcto, sin irse primero al lado contrario (ese era el error de la versión anterior) |
| Guiñada | Girar el dron a mano en sentido antihorario visto desde arriba | M2 y M4 **suben** mientras gira |
| Hélices | Mirar el giro de cada motor | M1 y M3 en sentido horario, M2 y M4 antihorario (vistos desde arriba) |

- Si la guiñada sale al revés, poner `SIGNO_YAW -1.0f`.
- Si sale al revés la P de roll o de pitch, **no volar**: avisar, porque es un problema del mezclador o del orden de los motores.

## 3. Balancín de 1 GDL (con hélices, atado)

Pasar una varilla o cuerda tensa por el centro del chasis, de modo que el dron **solo pueda rotar en roll**.
Hacer lo mismo después para pitch. Mantener a todos detrás del plano de las hélices y tener PARAR a la mano.

Para cada eje:
1. Poner Ki = 0, Kd = 0 y Kp = 0.003. Subir el acelerador hasta casi el hover (~45–55 %).
2. Empujar con un palo y **subir Kp** de a 0.001 hasta que empiece a oscilar lento (1–3 Hz). Anotar ese valor.
3. Subir **Kd** de a 0.0001 hasta que el empujón se amortigüe en 1–2 rebotes.
   Si aparece un zumbido rápido o los motores "chirrían", Kd está alto: bajarla un 30 %.
4. Con la D puesta, se puede subir Kp un poco más (~20 %). Si vuelve la oscilación lenta, bajarla.
5. Si queda inclinado fijo hacia un lado (centro de gravedad corrido): **Ki** de 0.002 a 0.005.
6. Anotar los valores finales. La página no los guarda: al reiniciar vuelven los del código. Para fijarlos, copiarlos en el `.ino`.

Valores de partida esperados: Kp de 0.005 a 0.012 y Kd de 0.0003 a 0.0008.

## 4. Hover cautivo

1. Atar el dron con 4 cuerdas cortas a pesos en el piso, con holgura de ~30 cm.
2. Probar el failsafe: con el motor girando bajo, cerrar la pestaña del navegador. Debe hacer una bajada automática en ~1 s.
3. Armar y subir **rápido** hasta despegar, para no quedarse en el efecto suelo, que lo desestabiliza.
4. Anotar el % de hover y, si gira sobre sí mismo, subir Kp_yaw de a 0.0005.
5. Si se inclina más de 50° corta solo (aparece "inclinacion > 50 grados" en la página).

## 5. Vuelo libre

Solo con hover cautivo estable durante más de 30 s.
