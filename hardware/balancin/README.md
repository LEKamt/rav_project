# Identificación y diseño del PD en el balancín

Sirve para un eje a la vez (roll o pitch) con el F450 montado en el tubo. Usa el firmware
`hardware/PrimerVueloWiFi_PIDv2` (sección **Identificación** de la página) y este script.
Solo se necesitan numpy y matplotlib.

## Modelo que se identifica

```
θ'' = K·uf(t − d) − b·θ' − k·θ + c        uf' = (u − uf)/τ
```

| Símbolo | Qué es | Unidad |
|---|---|---|
| θ | ángulo del eje | grados |
| u | mando diferencial que sale del mezclador | fracción del acelerador |
| K | aceleración por unidad de mando (sale negativo si el signo de la P es correcto) | grados/s² |
| τ | constante de tiempo de motor + ESC | s |
| d | retardo puro (ESC, PWM, DLPF de la IMU) | muestras |
| b | amortiguamiento (roce del tubo, aire) | 1/s |
| k | rigidez del tubo: > 0 si el pivote está sobre el CG, < 0 si está debajo | 1/s² |
| c | desbalance (CG corrido) | grados/s² |

El método es una regresión lineal de la aceleración angular sobre [uf, θ', θ, 1], probando una
grilla de (τ, d). Con eso se valida el modelo y se buscan Kp y Kd que cumplan márgenes mínimos:
ganancia ×2, retardo de 15 ms y fase de 40°.

## Ensayo 1: identificación

1. El dron en el tubo, con el eje libre. Cargar unas ganancias que lo **sostengan** aunque sea
   oscilando un poco. Por ejemplo, Kp 0.003 y Kd 0.0006.
2. ARMAR y subir el acelerador al de hover (~45–55 %). K depende del acelerador, así que hay que
   identificar donde se va a volar.
3. Identificación: eje roll, amplitud 0.03, de 0.5 a 10 Hz, 20 s → **INICIAR ENSAYO**.
   - Si el ángulo pasa de ±15°, DETENER y repetir con amplitud 0.02. A los 30° se aborta solo.
   - Durante el ensayo, la banda muerta es 0 y el otro eje y la guiñada quedan en 0.
4. Cuando diga "listo, 10000 muestras (completo)": **PARAR** y luego **DESCARGAR CSV**.
5. Correr el script:
   ```
   python identificar.py ident1.csv
   ```
   Imprime el modelo, los márgenes de las ganancias actuales y el PD recomendado. Guarda tres
   archivos: `ident1_identificacion.png`, `ident1_diseno.png` y `ident1_resultado.json`.

Qué revisar en la salida:
- R² de la aceleración > 0.8.
- Ajuste en lazo cerrado > 70 %.
- Menos de 5 % del tiempo saturado.

Si no se cumplen, el ensayo no sirve: repetir con otra amplitud.

## Ensayo 2: validación (esto va a la tesis)

1. Cargar en la página el Kp y el Kd recomendados y repetir el mismo ensayo.
2. Correr:
   ```
   python identificar.py ident1.csv --validar ident2.csv
   ```
   Compara el ángulo medido con lo que predice el modelo del ensayo 1 para las ganancias nuevas.
3. Prueba cualitativa: inclinar el dron ~10° con la mano, soltarlo y compararlo con `ident1_diseno.png`.
4. Después, Ki pequeño (0.002–0.005) para quitar el offset del desbalance.

Repetir todo para pitch.

## Verificación del script

`simular_ensayo.py` genera un CSV con una planta conocida que incluye ruido, DLPF y fricción seca.

Planta usada: K = −30 000, τ = 60 ms y 8 ms de retardo, más ~4 ms del DLPF.

Lo que recuperó el script:
- K = −33 000
- τ = 70 ms
- d = 10 ms
- ajuste en lazo cerrado del 95 %

También predijo bien qué ganancias eran inestables.
