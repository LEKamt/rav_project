# F450 · RAV — Control de actitud y altura

ESP32-S3 + MPU-6500 + VL53L0X + HC-SR04 + 4 × ESC 30 A / A2212 1000 KV, marco F450.

Reescritura completa siguiendo las *Notas de Clase — Robots Aéreos* (UTEC, O. Ramos).

| Versión | Qué resuelve |
|---|---|
| **V_1** | estabilización de actitud: roll, pitch y yaw |
| **V_2** | lo anterior **+ control de altura** (ec. 35-36) |
| **V_3** | lo anterior **+ control robusto** por modos deslizantes, conmutable en vuelo |
| **V_4** | lo anterior, con los signos de la IMU ya medidos y la página rehecha |
| **V_5** | lo anterior **+ identificación en línea y ganancias adaptativas**, fusión de altura corregida |
| **V_6** | lo anterior **+ topes duros del lazo de altura y caja negra descargable en CSV** |
| **V_7** | lo anterior, con el estimador vertical acotado y diagnóstico por sensor |
| **V_8** | lo anterior, con la referencia de altura fija, nivel fino y página reducida |
| **V_9** | lo anterior, con roll/pitch con prioridad sobre yaw en el mezclador |
| **V_10** | lo anterior, con la altura corregida: filtro del acelerómetro, solo medidas nuevas, rechazo de rebotes y hover aprendido; V_10b: estimador que no diverge (log del 26-11) |
| **V_11** | lo anterior, con el Kalman de altura sintonizado con el ruido medido del ultrasonido, mediana de ecos, IMU vertical filtrada y CSV sin duplicados |

Lo único que falta de la Sección 5 de las notas es la posición lateral `(x, y)`, y no
por el código: este dron no tiene ningún sensor que la mida.

---

## 1. Estructura

```
Dron_F450_Control/
├── README.md          este archivo
├── VERSIONES.md       qué cambió en cada V_x
├── V_1/            solo actitud
├── V_2/            actitud + altura
├── V_3/            + control robusto
├── V_4/            + página nueva
├── V_5/            + adaptación al dron real
├── V_6/            + topes de altura y caja negra
├── V_7/            + estimador acotado y diagnóstico
└── V_8/            + nivel fino y página reducida  ← la versión a usar
    ├── V_2.ino        tareas, armado, failsafe, modos de empuje, consola
    ├── config.h       pines, parámetros físicos, límites, ajustes por defecto
    ├── estado.h       estado compartido entre los tres hilos + buffer de gráficas
    ├── imu.h          MPU-6500/6050 y mapeo de ejes sensor → cuerpo
    ├── actitud.h      filtro de Mahony sobre cuaternión
    ├── control.h      cascada ángulo → velocidad angular → torque (ec. 40-44 y 41)
    ├── mezclador.h    inversa del mezclador (ec. 46) + proyección al politopo
    ├── altura.h       [V_2] Kalman vertical + lazo de altura (ec. 35-36)
    ├── robusto.h      [V_3] super-twisting: por qué y cómo se sintoniza
    ├── identifica.h   [V_5] autoridad medida en vuelo y ganancias adaptativas
    ├── sensores.h     ToF, ultrasonido y batería, sin bloquear el lazo
    ├── web.h          punto de acceso, API HTTP
    └── pagina.h       interfaz web (PROGMEM)
```

Cada versión nueva se guarda como carpeta `V_2`, `V_3`, … al lado de `V_1`.
El nombre del `.ino` tiene que coincidir con el de la carpeta.

---

## 2. Conexiones

| Señal | Pin ESP32-S3 |
|---|---|
| Motor 1 (adelante-izq) | 8 |
| Motor 2 (adelante-der) | 17 |
| Motor 3 (atrás-der) | 21 |
| Motor 4 (atrás-izq) | 18 |
| I2C0 SDA / SCL (MPU) | 1 / 39 |
| I2C1 SDA / SCL (VL53L0X) | 4 / 5 |
| HC-SR04 TRIG / ECHO | 6 / 7 |
| Batería (opcional) | `PIN_VBAT` en `config.h`, −1 = no instalada |

> El HC-SR04 entrega el eco a 5 V. Si no hay un divisor resistivo en ECHO, el pin 7
> del ESP32-S3 está fuera de especificación. Divisor 10 k / 20 k y listo.

**Software**: Arduino IDE con el núcleo `esp32` 3.x, placa *ESP32S3 Dev Module*.
Única librería externa: **Adafruit_VL53L0X**. `WiFi.h`, `WebServer.h` y
`Preferences.h` vienen con el núcleo.

Compilado con `esp32` 3.3.0 y `--warnings all`, sin errores ni avisos:
**1 050 219 bytes de flash (80 %)** y
50 880 bytes de RAM (15 %). Con el esquema de particiones por defecto quedan
unos 260 kB libres; si la V_2 se queda sin sitio, cambiar a *Huge APP (3 MB No
OTA)* en el menú Herramientas.

---

## 3. Convención de ejes y numeración de rotores

La del curso (Sección 4.2): **x adelante, y a la derecha, z hacia abajo** (FRD),
ángulos de Euler ZYX, velocidad angular `(p, q, r)` en el cuerpo.

```
            adelante (+x)
                 ▲
        1 ◯             ◯ 2
          ╲             ╱
            ╲         ╱
              ╲  ×  ╱            →  +y (derecha)
              ╱     ╲
            ╱         ╲
        4 ◯             ◯ 3
```

- `r1 = ( l, −l, 0)`, `r2 = ( l, l, 0)`, `r3 = (−l, l, 0)`, `r4 = (−l, −l, 0)`
- Rotores **1 y 3** giran en un sentido; **2 y 4** en el contrario, para que
  `τ_z = k_m (ω₁² − ω₂² + ω₃² − ω₄²)`.
- `l = 0.1616 m` es el brazo **efectivo en X**, es decir `L_real/√2`, no la
  longitud del brazo del marco.

El orden de `PIN_MOTOR[4]` en `config.h` es por **número de rotor**, no por la
etiqueta del cable. Comprobarlo con la prueba individual de la pestaña CALIBRAR
antes de poner hélices.

### Orientación de la IMU

`IMU_EJE = {1,0,2}` y `IMU_SIGNO = {−1,−1,−1}` (**medido en banco el 25-11-2026**).

Con `{+1,+1,−1}` el dron daba roll negativo al inclinarlo a la derecha y pitch
positivo con la nariz abajo: los dos invertidos a la vez, con el yaw correcto.
Eso es exactamente una IMU montada girada 180° sobre z. El diagnóstico se
verifica solo: si **solo** roll estuviera invertido, el sistema sería izquierdo,
y eso no sale de ningún montaje físico. La corrección invierte x e y y deja z,
con determinante +1. Los motores no se tocan: su numeración va con el frente
físico del dron.

El código anterior usaba `{1,0,2}` con los tres signos positivos. Intercambiar dos
ejes sin invertir ninguno produce un sistema **izquierdo**, y de ahí venían las
contradicciones entre el signo del giroscopio y el del ángulo. Aquí se invierte z
y el sistema vuelve a ser derecho, así que no hay constantes de signo que ajustar
a mano en ninguna parte del control.

Verificación (pestaña SENSORES, desarmado, el dron en la mano):

| Movimiento | Debe pasar |
|---|---|
| inclinar a la derecha | roll sube (+) |
| levantar la nariz | pitch sube (+) |
| girar la nariz a la derecha | yaw sube (+) |

Y mientras se mueve, `p/q/r` deben tener el mismo signo que el ángulo que cambia.
Si alguno sale al revés, se corrige su signo en `IMU_SIGNO`.

---

## 4. Dónde está cada ecuación de las notas

| Notas | Qué es | Archivo |
|---|---|---|
| (1), (2) | `f = k_f ω²`, `τ = k_m ω²` | `mezclador.h` |
| (26) | `(φ̇, θ̇, ψ̇)` ↔ `(p, q, r)` | `control.h`, invertida |
| (27), (45) | mezclador directo | `mezclador.h` (comentario) |
| (32) | dinámica rotacional con acoplamiento giroscópico | `control.h`, ec. (41) lo cancela |
| (40), (44) | lazo externo proporcional sobre los ángulos | `control.h` |
| (41) | `u_x = I_x v_p + (I_z − I_y) q r` | `control.h` |
| (42) | lazo interno sobre `(p, q, r)` | `control.h` |
| (46) | inversa del mezclador | `mezclador.h::calcular` |
| § 4.2 | proyección al politopo factible | `mezclador.h`, ver abajo |
| (29) | `r₃₃` proyecta el empuje sobre la vertical | `altura.h`, `actitud.h::filaZ` |
| (35), (36) | control de altura | `altura.h::ControlAltura` **[V_2]** |
| (37)-(40) | posición lateral | no implementable: faltan sensores |
| § 5, final | el aviso sobre la incertidumbre del modelo | `robusto.h` **[V_3]** |

### Dos desviaciones conscientes respecto de las notas

**1. Saturación.** Las notas proyectan al politopo recortando componente a
componente. Ese recorte cambia la *dirección* del torque: si solo satura un motor,
el dron recibe un torque distinto del pedido y aparece acoplamiento entre ejes
justo cuando más autoridad hace falta. Aquí se recorta primero el empuje común y
después se escala el torque **completo** por el mayor factor `s ∈ [0,1]` que cabe
en la caja, de modo que el torque entregado es `s·τ_pedido`: misma dirección,
menos magnitud. El recorte de las notas se aplica igual al final como garantía
numérica. El factor `s` se publica en la página como *escala por saturación*: si
baja mucho de 1, falta autoridad.

**2. Término integral.** Las notas usan P puro en el lazo de velocidad angular,
que deja error de régimen en cuanto hay una asimetría constante (centro de masa
descentrado, un ESC con otro recorrido, una hélice gastada). Eso es exactamente lo
que antes se corregía a mano con un *trim*. El integrador lo hace solo, con
anti-windup y congelado mientras el mezclador satura. Viene en **0** por defecto:
primero se sintoniza el P puro, como en las notas, y después se sube.

### Estimador de actitud

Mahony sobre cuaternión, no el complementario de un eje. El complementario mezcla,
eje por eje, el ángulo del acelerómetro con la integral del giroscopio; eso vale
mientras el dron esté casi nivelado y se mueva en un solo eje, pero en cuanto roll
y pitch se mezclan, la integral de `p` **no** es la derivada de roll (ec. 26) y el
estimador se desfasa. Mahony integra la orientación completa con `(p, q, r)` tal
como salen del giroscopio y corrige la deriva con el error entre la gravedad medida
y la predicha; los signos salen del producto cruz. El término `Ki` estima además el
sesgo del giroscopio en vuelo.

La corrección por acelerómetro solo se aplica si `|a|` está entre 0.5 y 1.6 g: si
el dron acelera fuerte o vibra, esa medida no apunta a la gravedad.

**Sin magnetómetro el yaw solo sale de integrar `r`, así que deriva** (del orden de
1-3 °/min después de calibrar). Para estabilizar basta; para navegar hará falta
brújula o visión.

### Control de altura (V_2)

**Estimador.** Los sensores de distancia dan 20 medidas por segundo, con ruido y, el
ultrasónico, con rebotes: derivarlas para sacar velocidad vertical no sirve de nada.
El acelerómetro da 250 por segundo pero su integral se va en segundos por el sesgo.
El Kalman los fusiona con **tres** estados, `[z, ż, b]`, donde `b` es el sesgo de la
aceleración vertical. Ese tercer estado es el que importa: sin él, 0.1 m/s² de sesgo
se convierten en 0.1 m/s de error de velocidad por **cada segundo** de vuelo.

La aceleración vertical inercial sale de proyectar el acelerómetro con la tercera fila
de la matriz de rotación y volver a sumar la gravedad:

```
a_z = r₃₁·ax + r₃₂·ay + r₃₃·az + g
```

En reposo y nivelado el acelerómetro mide `(0,0,−g)` y `r₃₃ = 1`, así que `a_z = 0`.
La corrección usa una `R` distinta según la medida venga del ToF (σ ≈ 2 cm) o del
ultrasónico (σ ≈ 5 cm).

**Control**, ec. (35)-(36) tal cual:

```
v_z = −kd_z (ż − ż_ref) − kp_z (z − z_ref)
u₁  = (v_z − g) / r₃₃          →      f_t = −m·u₁
```

`u₁` cancela la gravedad y divide por `r₃₃ = cos φ cos θ`, que es la proyección del
empuje sobre la vertical: inclinado hay que empujar más para sostener la misma altura.
Sustituyendo en `z̈ = g + r₃₃·u₁` queda `z̈ = v_z`, y el error cumple
`ë_z + kd_z ė_z + kp_z e_z = 0`.

Todo el lazo está escrito con **z hacia abajo**, como en las notas; la conversión a
«altura» se hace solo en los bordes (medida y página).

**El término integral aquí no es opcional**, al revés que en actitud. La cancelación
de la gravedad usa `m` y `kf` del modelo; si cualquiera de los dos está un 10 %
desviado —y lo estará: la batería pesa distinto llena que vacía— el empuje de
equilibrio no es el que calcula el modelo y el dron se queda colgado con error
constante. Por eso `ki_z = 1.0` viene activo, con anti-windup y congelado cuando el
empuje pedido topa.

**Modos de empuje**

| Modo | Quién decide `f_t` |
|---|---|
| MANUAL | la barra de la página, en newtons |
| MANTENER | el lazo de altura |

Se entra solo **en vuelo**, con medida válida y por encima de 15 cm: se despega a
mano, siempre. Al entrar se carga el integrador con

```
iz = g − f_t·r₃₃/m + kd_z·ż
```

que hace que el empuje calculado salga **exactamente igual** al que ya se está dando.
Sin eso el dron pega un tirón al activar el modo. Al salir no hay salto tampoco,
porque mientras el lazo manda, la barra va siguiendo al empuje real.

Si se pierde la medida más de 1 s, vuelve solo a manual con el mismo empuje y avisa.
Mientras el lazo manda, la barra **no** escribe empuje: si no, los dos pelearían por
la misma variable.

**Límites.** El ToF se queda ciego pasados ~1.2 m y el ultrasónico ~4 m, así que el
tope de referencia está en 2.5 m. Además miden distancia **al suelo que haya debajo**,
no altura sobre el punto de despegue: si el dron pasa sobre una mesa, la referencia se
mueve con ella.

### Control robusto (V_3)

Las notas terminan la Sección 5 avisando de su propio límite: la linealización por
realimentación cancela la dinámica usando el **modelo**, y si el modelo tiene
incertidumbre, con `m` o `I` no exactos, o con `kf`, `km` que varían con el desgaste
de las hélices o la densidad del aire, la cancelación no es perfecta y en lugar de
linealizar se introduce error sistemático.

En este dron el modelo **va** a cambiar: la batería pesa ~200 g y al descargarse mueve
`m` y el centro de masa —y con él `Ix`, `Iy`—; `kf` cae con el desgaste de las hélices
y con la tensión; `omega_max` nunca se midió, solo se estimó del KV; el ESC del rotor
2 es de otro modelo; y el efecto suelo altera el empuje por debajo de ~30 cm.

Para el lazo cerrado todo eso es una **perturbación acotada**. La V_3 le suma a la ley
nominal un término de modos deslizantes de segundo orden (super-twisting):

```
v = k1 |s|^(1/2) sign(s) + w,      w_punto = k2 sign(s)
```

Elegido sobre el deslizante clásico `k·sign(s)` porque su salida es **continua**: con
PWM a 50 Hz y ESCs de 30 A, conmutar a tope en cada ciclo es castigar los motores.
Converge en tiempo finito y compensa cualquier perturbación con derivada acotada
`|d_punto| <= L` si `k2 > L` y `k1 > 1.5√L`, **sin necesidad de conocer `d`**.

| Lazo | Superficie `s` | Unidades de `s` → salida |
|---|---|---|
| velocidad angular | el propio error `p_d − p` | rad/s → rad/s² |
| altura | `sigma = lambda(z_ref − z) − z_punto`, con `lambda = kp_z/kd_z` | m/s → m/s² |

Esa lambda no es arbitraria: con ella se cumple
`kd_z·sigma = −kd_z e_z' − kp_z e_z`, es decir que **la ley nominal de las notas ya es
proporcional a la superficie**. El modo robusto no la sustituye, solo le suma el
término que domina la incertidumbre.

Con el modo robusto activo, el integrador clásico se congela en ambos lazos: dos
términos corrigiendo lo mismo dan una oscilación lenta.

**El dron dice cuánto se equivoca su modelo.** El estado `w` converge a `−d`, así que
es una estimación en línea del error del modelo. La página lo muestra en rad/s² para
actitud y, para altura, traducido a **% del peso**: si marca +8 % de forma estable, la
masa real o `kf` están desviados esa cantidad y conviene corregirlos en MODELO.

Viene en **NOMINAL** por defecto. Se despega con la ley de las notas y se conmuta en
vuelo (botón en VUELO, o `r` por serie), que es lo que permite comparar las dos leyes
en el mismo vuelo y con la misma batería.

### Identificación y adaptación (V_5)

Todo el controlador se apoya en un modelo, y de los parámetros de este dron
**`omega_max` nunca se midió**. Un error del 30 % ahí son un 70 % de error en la
autoridad, porque el empuje va con `omega` al cuadrado. El síntoma es
inconfundible: el dron *intenta* controlar y no lo consigue.

La V_5 mide en vuelo una sola cantidad por eje, adimensional:

```
g = aceleración angular conseguida / aceleración angular pedida
```

| `g` | Significa |
|---|---|
| `≈ 1` | el modelo acierta |
| `< 1` | falta autoridad: responde menos de lo pedido |
| `> 1` | responde de más; el lazo está más cerca de oscilar de lo que crees |
| **`< 0`** | **el signo está invertido en ese eje**; ninguna ganancia lo arregla |

Esa última fila es la más útil de la tabla: un signo invertido se diagnostica
en dos segundos en vez de con un dron roto.

Con `g` medida, la ganancia sale de una división. El lazo interno cerrado es
`p' = g·kp·(p_d − p)`, de constante de tiempo `tau = 1/(g·kp)`, luego:

```
kp = 1 / (tau · g)
```

En modo **ADAPTATIVO** no fijas la ganancia sino el **tiempo de respuesta**, y la
ganancia se mueve con `g` para mantenerlo aunque cambie la batería o las
hélices. `tau = 0.067 s` equivale a `kp = 15` con `g = 1`, que es lo que tenían
las versiones anteriores.

El botón **EXC** inyecta ±40 °/s durante 4 s en un eje para garantizar que hay
qué aprender cuando el dron vuela muy quieto. Se suma a la *referencia*, no a la
salida, así que lo que se identifica es el lazo tal como vuela. Solo en vuelo y
con espacio alrededor.

### El nivel del suelo no es el nivel del aire (V_5)

El giroscopio se calibra con el dron en el suelo, pero el suelo no está a nivel,
el centro de masa no está centrado y los cuatro motores no dan lo mismo: **la
actitud a la que el dron se queda quieto en el aire no es la que marcaba 0
abajo**. Por eso se desplaza del punto de partida al subir.

Botón **APRENDER ESTE NIVEL** (pestaña VOLAR): se vuela sujetando roll y pitch
donde el dron no se desplace y se pulsa. Entonces

```
trim_nuevo = trim_viejo + referencia que estabas sujetando
```

y la referencia vuelve al centro, así que **el dron no se mueve al pulsar**: lo
único que cambia es dónde está el cero. Después, GUARDAR EN MEMORIA.

### Por qué el lazo de altura no puede fiarse del sensor (V_6)

El lazo de las ec. (35)-(36) es correcto, pero su entrada es una medida. Si la
medida **miente diciendo que no sube**, el lazo pide más empuje, sigue sin ver
que sube, y pide más. Con el único tope en el 95 % del empuje de los motores,
ese bucle acaba a plena potencia contra el techo. Pasa de verdad: el HC-SR04 se
ciega con el ruido de las hélices, y el ToF fuera de rango devuelve basura.

La V_6 corta ese bucle en cuatro sitios:

| Protección | Qué hace |
|---|---|
| Topes contra el hover | el empuje queda en `[0.45, 1.35] x m·g/r33`, que no depende de ningún sensor |
| Techo duro | por encima de `alt_max + 0.30 m` se sale del modo |
| Sensor atascado | 2.5 s pidiendo subir sin que la altura medida suba 5 cm → fuera |
| Salida segura | al salir, el empuje va al de **hover**, no al último que había |

Esa última fila importa más de lo que parece: conservar el último empuje al
salir por fallo del sensor significa seguir subiendo igual, solo que ya sin
lazo.

### Caja negra (V_6)

El dron ya guardaba muestras a 50 Hz para las gráficas. Ahora esa muestra lleva
también el empuje pedido, **lo que da cada sensor de distancia por separado** y
el modo, y la página lo descarga como CSV de 22 columnas. Los dos sensores por
separado son la clave: con la altura fusionada no se puede saber cuál mintió.

Graba sola al armar y para al desarmar, así que un vuelo que acaba mal queda
registrado sin acordarse de pulsar nada. Pestaña DATOS → DESCARGAR CSV.

---

## 5. Parámetros del dron

| Parámetro | Valor | Unidad |
|---|---|---|
| masa `m` | 1.0022 | kg |
| brazo efectivo `l` | 0.1616 | m |
| `I_x` | 0.0097 | kg·m² |
| `I_y` | 0.0106 | kg·m² |
| `I_z` | 0.0201 | kg·m² |
| `k_f` | 3.2107 × 10⁻⁶ | N/(rad/s)² |
| `k_m` | 5.0879 × 10⁻⁸ | N·m/(rad/s)² |
| `ω_max` | 1046 (estimada) | rad/s |
| `kp_z`, `kd_z`, `ki_z` | 4.0, 3.6, 1.0 | ωn = 2 rad/s, ζ = 0.9 |

`ω_max` es el **único** parámetro del mapeo que no viene medido. Sale de
`KV × V_bat × 2π/60 × η ≈ 1000 × 11.1 × 0.10472 × 0.90`. Con esos números:

- empuje máximo ≈ 14.0 N, peso ≈ 9.83 N → **empuje/peso ≈ 1.43**
- acelerador de hover ≈ **70 %** de la barra

Un empuje/peso de 1.43 es bajo: deja poco margen para corregir. Si el dron sube con
menos barra de la que dice la página, `ω_max` real es mayor; se ajusta desde la
pestaña MODELO hasta que el hover coincida. La barra de acelerador manda **empuje
total en newtons**, no un porcentaje de PWM, así que esa cifra es física y se puede
comprobar.

El tope de la barra está en 85 % y el acelerador sube con una rampa de 150 ms.

---

## 6. Puesta en marcha

**Siempre sin hélices hasta el paso 5.**

1. **Cargar y abrir el monitor serie** a 115200. Al arrancar manda solo pulso
   mínimo durante 3 s (nunca el pulso máximo: algunos ESC lo toman como orden de
   girar al máximo). Después calibra el giroscopio: no mover durante esos 2 s.
2. **Conectarse** a la red WiFi `F450-RAV`, clave `f450vuelo`, y abrir
   `http://192.168.4.1`.
3. **SENSORES**: comprobar que la IMU dice OK, que `|a| ≈ 1.000` en reposo, que el
   sesgo del giroscopio es pequeño y que los signos de la tabla del apartado 3 son
   correctos. Comprobar también ToF y ultrasonido.
4. **CALIBRAR → prueba individual**: marcar «hélices fuera» y pulsar M1…M4. Cada
   botón debe mover el motor que dice el diagrama, y 1/3 deben girar al revés que
   2/4. Si no coinciden, se reordena `PIN_MOTOR[]` o se cambian dos cables del
   motor para invertir el sentido.
5. **CALIBRAR → nivel**: con el dron apoyado y nivelado, *poner nivel actual como
   cero*, y después *guardar en memoria*.
6. **Banco de un eje** (varilla, hélices puestas, solo roll): armar, subir
   acelerador hasta que el eje se sostenga y sintonizar como en el apartado 7.
7. **Vuelo libre** solo después de que el banco esté limpio en los tres ejes.
8. **Altura (V_2)**: con el dron en la mano, comprobar en SENSORES que la altura
   estimada sigue a la medida y que `vz` se queda en cero cuando no se mueve. Después
   despegar a mano, estabilizar a ~50 cm y pulsar **MANTENER ALTURA**. Solo cuando
   eso aguante quieto, tocar el slider de objetivo.

---

## 7. Sintonía

El orden importa: de dentro hacia fuera.

1. Sujetar el dron en una varilla de un solo eje. Subir **kp p** hasta que oscile
   y bajar a la mitad.
2. Repetir con **kp q** (pitch) y después con **kp r** (yaw, bastante más bajo).
3. Subir **kp roll/pitch** del lazo externo hasta que siga la referencia sin
   rebotar. Son `1/s`: `kp = 6` es una constante de tiempo de 170 ms.
4. Solo al final, subir **ki** (empezar en 1.0) para quitar el error de régimen.
5. **kd** casi nunca hace falta: deriva la velocidad angular y amplifica el ruido.

Para la **altura** (V_2), en este orden y siempre con la actitud ya limpia:

6. Subir **kp_z** hasta que responda sin rebotar. Pasado ~8 no ayuda: lo que limita no
   es la ganancia, es el ruido del sensor de distancia.
7. Ajustar **kd_z** para que no sobrepase. La pareja por defecto (4 y 3.6) da
   ωn = 2 rad/s y ζ = 0.9: sube a una altura nueva en unos 2 s sin pasarse.
8. **ki_z** solo si se queda colgado por debajo o por encima del objetivo de forma
   constante; eso significa que `m` o `kf` están desviados.
9. Si la altura estimada tiembla, subir `Kalman q acel`; si va con retraso, bajarlo.

Para el **modo robusto** (V_3), una vez que el nominal vuela bien:

10. Activarlo en vuelo estacionario y mirar la perturbación estimada. Debe
    asentarse en un valor casi constante en unos segundos.
11. Si tarda en asentarse, subir `k2`. La regla es `k2 > L` y `k1 > 1.5√L`, donde
    `L` es lo rápido que puede cambiar lo que el modelo ignora.
12. **Si los motores zumban, subir la capa límite antes que bajar `k2`.** El
    zumbido es el término conmutando con el ruido del giroscopio; ensanchar la
    capa lo quita a cambio de dejar un error residual de ese tamaño.
13. No subir `k1` y `k2` por si acaso: ganancias de más convierten el ruido del
    sensor en mando, que es justo lo que el PWM a 50 Hz no puede seguir.

Las ganancias se pueden cambiar **en vuelo**; cada vez que se aplican, los
integradores se ponen a cero. *Guardar en memoria* las escribe en NVS y sobreviven
al reinicio. Los parámetros del modelo físico solo se pueden cambiar desarmado.

**El techo de ganancia no lo pone el control, lo pone el PWM.** Con `PWM_FREQ = 50`
el retardo de actuación llega a 20 ms y eso limita el lazo de velocidad angular por
mucho que se suba `kp`. Si los ESC son BLHeli/SimonK se puede subir a 250 o 400 Hz
en `config.h`; hay que recalibrar el recorrido y volver a sintonizar desde ganancias
bajas.

---

## 8. La página

| Pestaña | Qué hay |
|---|---|
| **VOLAR** | conmutador NOMINAL/ROBUSTO con el error de modelo en %, altura y su modo, roll/pitch/yaw grandes, horizonte artificial, barra de acelerador con marca de hover, referencias de actitud que vuelven solas al centro, mando y µs de cada motor, aviso de torque recortado |
| **DATOS** | actitud, `p/q/r` medidos y de referencia, acelerómetro y giroscopio crudos, `|a|`, sesgo estimado, temperatura, ToF, ultrasonido, altura corregida por inclinación, `f_t` y torques, integrales, pulsos, ω y empuje por rotor, Hz del lazo, peor ciclo, batería, memoria, clientes, uptime |
| **AJUSTAR** | las ganancias como deslizadores que se aplican al momento, agrupadas y numeradas en el orden en que se sintonizan; calibración; prueba de motores; modelo físico y ESC (con botón, solo desarmado) |
| (en AJUSTAR) | `m, l, I, k_f, k_m, ω_max`, recorrido de cada ESC, y el resumen que recalcula el dron: empuje máximo, de hover, empuje/peso, acelerador de hover y torque disponible |
| (en DATOS) | ángulos, velocidades angulares, mando de los cuatro motores y altura (medida, referencia y `vz`), con escala y pausa |
| (en AJUSTAR) | giroscopio, nivel, rumbo y prueba individual de motores |

Refresca 10 veces por segundo; las gráficas se alimentan de un registro a 50 Hz que
el lazo de control escribe en un buffer circular, así que no se pierden muestras
entre refrescos.

**Al cargar una versión nueva**, el sello `AJUSTES_MAGIA` cambia y los ajustes
guardados en memoria por la versión anterior se descartan: entran los valores de
fábrica de esa versión. Es deliberado, porque entre versiones cambian constantes
como los topes de los ESC. Si vienes de V_1–V_3 con ajustes guardados y quieres
los nuevos, pulsa VALORES DE FABRICA y después GUARDAR.

---

## 9. Seguridad

- **Arma solo** con el acelerador en 0, la IMU respondiendo y menos de 15° de
  inclinación. Al armar, los motores quedan al ralentí con empuje 0.
- **Corte por inclinación**: más de 60° en roll o pitch → desarme inmediato.
- **Corte por IMU**: 100 ms sin respuesta del MPU → desarme.
- **Failsafe de enlace**: la página pide estado 10 veces por segundo. Sin latido
  durante 0.8 s (móvil bloqueado, fuera de alcance), el acelerador baja solo al
  20 %/s y termina desarmando. Si el enlace vuelve antes, el vuelo sigue.
- **Prueba de motores**: exige confirmar «hélices fuera», topa en 45 % y se corta
  sola a 1.5 s si la página deja de refrescar la orden.
- Con empuje por debajo del 15 % el integrador se congela y por debajo del 5 % se
  pone a cero, para que no se cargue mientras el dron está en el suelo.
- El botón **PARAR** desarma en el acto. Por serie: `s`.

Comandos por serie: `a` armar, `s` parar, `+`/`−` acelerador, `c` calibrar
giroscopio, `n` nivelar, `g` guardar, `i` info.

---

## 10. Reparto de núcleos

| Núcleo | Tarea | Frecuencia |
|---|---|---|
| 1 | `tareaControl` — IMU, actitud, control, mezclador, motores | 250 Hz |
| 0 | `tareaSensores` — ToF, ultrasonido, batería | 20 Hz |
| 0 | `tareaWeb` — servidor HTTP | a demanda |

El lazo de control no comparte núcleo con la red, así que la página no lo frena.
Ningún sensor lento bloquea: el ToF va en modo continuo y el ultrasonido mide el
eco por interrupción, no con `pulseIn()`, que bloquearía hasta 30 ms —siete ciclos
del lazo.

No hay semáforos: cada campo del estado compartido es de 32 bits y en el ESP32 una
escritura alineada de 32 bits es atómica. En el buffer de gráficas sí importa el
orden, así que se escribe la muestra y **después** se sube el contador.

---

## 11. Lo que falta (V_2): control de altura

Ya está todo lo que necesita:

- `G.altura_m`, fusión de ToF y ultrasonido corregida por `cos(φ)cos(θ) = r₃₃`.
- El mezclador acepta empuje total en **newtons**, no un «acelerador».

Falta cerrar el lazo de las ec. (35)-(36):

```
v_z = z̈_ref − k_d,z (ż − ż_ref) − k_p,z (z − z_ref)
u₁  = (v_z − g) / r₃₃          →      f_t = −m u₁
```

con `ż` estimado por un Kalman entre la altura medida y la aceleración vertical del
acelerómetro proyectada al sistema inercial.
