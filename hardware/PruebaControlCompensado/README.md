# PruebaControlCompensado

Es igual a `PruebaControlCalibrado` (pines, PID, WiFi `F450-TEST`), con un cambio: **M2 se
compensa con una tabla medida en la balanza** en lugar de con `escala_motor = 0.35`.

## Cómo funciona

1. El control calcula el **pulso nominal** de cada motor, en la misma escala de antes
   (1000–1850 µs), así que las ganancias PID conservan su significado.
2. Con ese pulso se busca en la tabla el empuje de referencia `Fref`: el promedio de M1, M3 y M4.
3. A M2 se le manda el pulso que, según **su** tabla, da `Fref`.

Con los datos actuales (1000–1250 µs):

| Nominal [µs] | Fref [g] | M2 real [µs] |
|---|---|---|
| 1050 | 0.0 | 1000 |
| 1100 | 2.3 | 1006 |
| 1150 | 6.0 | 1014 |
| 1200 | 10.7 | 1025 |
| 1250 | 17.7 | 1042 |

**Por encima de la tabla (más de 1250 µs, ≈ 29 % de gas):** el gas sigue hasta el **100 %**.
M2 sigue una recta desde el último punto de la tabla (1042 µs) hasta lo que daba la escala
antigua de 0.35 a tope (1298 µs). En el serie aparece `[SOBRE TABLA]`, solo como aviso. Ese
tramo es una suposición: cuando la tabla llegue a 1850 µs, se usa la tabla en todo el rango.

| Gas | 30 % | 50 % | 70 % | 100 % |
|---|---|---|---|---|
| M2 real [µs] | 1044 | 1117 | 1189 | 1298 |

## Completar la tabla (balanza, sin hélices de vuelo en el dron)

1. `comp` → apaga la compensación (los pulsos quedan crudos).
2. Por cada motor: `raw <motor> <us>` cada 50 µs, de 1000 a 1850 (en M2, cada 10–20 µs en la
   zona útil), y anotar los gramos. `s` para parar.
3. Editar `TAB_PULSO`, `TAB_EMPUJE` y `N_PTS` en el `.ino`. Al arrancar, el sketch verifica que
   todo sea creciente y, si la tabla está mal, apaga la compensación.
4. `tabla` → imprime la tabla, el techo y el mapeo nominal → real.

## Verificar

- `nom 1150`, `nom 1200`, …: los 4 motores reciben el mismo pulso nominal compensado. **En la
  balanza, M2 debe dar lo mismo que los otros** (±15 %).
- En el balancín: con gas constante y sin control, no debe irse hacia M2.

## Comandos nuevos

| Comando | Qué hace |
|---|---|
| `raw <0-4> <us>` | Pulso crudo sin compensar (0 = los 4) |
| `nom <us>` | Los 4 con el mismo pulso nominal, compensados |
| `tabla` | Tabla, techo y mapeo |
| `comp` | Compensación ON/OFF (con motores parados) |

Se quitó `esc`/`modo`. `m <motor> <g>` y `todos <g>` usan ahora la tabla. En el sketch
original, `m` aplicaba la curva **y además** la escala de 0.35, así que M2 quedaba compensado
dos veces.
