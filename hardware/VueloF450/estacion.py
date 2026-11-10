#!/usr/bin/env python3
"""Estación para operar VueloF450 por el puerto serie (USB/CH340), sin ROS.

- Manda un latido (línea vacía) cada 100 ms: si el firmware no recibe nada en 1 s, baja solo.
- Muestra un resumen del estado dos veces por segundo y los mensajes del firmware ('# ...').
- Guarda toda la telemetría CSV en vuelos/vuelo_AAAAmmdd_HHMMSS.csv (para graficar PID contra SMC).
- Lo que se escribe + Enter se manda tal cual (escribir 'ayuda' para ver los comandos).
- Ctrl+C manda PARO ('x') antes de salir.

  pip install pyserial
  python hardware/VueloF450/estacion.py COM5          # Windows
  python3 hardware/VueloF450/estacion.py /dev/ttyUSB0 # Linux
"""
import argparse
from datetime import datetime
from pathlib import Path
import sys
import threading
import time

ENCABEZADO = 't_ms,'
CAMPOS_RESUMEN = ('modo', 'ley', 'empuje', 'roll', 'pitch', 'r', 'z', 'lazo_max_us', 'atrasos')
MODOS = {0: 'DESARMADO', 1: 'ARMADO', 2: 'BAJADA', 3: 'MOTOR', 4: 'CALIB_ESC'}


def clasificar(linea, columnas):
    """('mensaje', texto) | ('encabezado', columnas) | ('dato', dict) | ('otro', texto)."""
    if linea.startswith('#'):
        return 'mensaje', linea
    if linea.startswith(ENCABEZADO):
        return 'encabezado', linea.split(',')
    if columnas and linea.count(',') == len(columnas) - 1:
        try:
            return 'dato', dict(zip(columnas, (float(v) for v in linea.split(','))))
        except ValueError:
            pass
    return 'otro', linea


def resumen(d):
    return (f"{MODOS.get(int(d['modo']), d['modo']):>9} {'SMC' if d['ley'] else 'PID'} "
            f"empuje {d['empuje']:.2f} | roll {d['roll']:+6.1f} pitch {d['pitch']:+6.1f} "
            f"r {d['r']:+6.1f} | z {d['z']:.2f} m | lazo {d['lazo_max_us']:.0f} us "
            f"atrasos {d['atrasos']:.0f}")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('puerto')
    p.add_argument('--baudios', type=int, default=921600)
    p.add_argument('--carpeta', type=Path, default=Path('vuelos'))
    args = p.parse_args()

    import serial

    sp = serial.Serial(args.puerto, args.baudios, timeout=0.1)
    args.carpeta.mkdir(parents=True, exist_ok=True)
    ruta = args.carpeta / f'vuelo_{datetime.now():%Y%m%d_%H%M%S}.csv'
    log = open(ruta, 'w', encoding='utf-8', newline='\n')
    mensajes = open(ruta.with_suffix('.txt'), 'w', encoding='utf-8', newline='\n')
    print(f'Registro: {ruta}')
    vivo = threading.Event()
    vivo.set()
    candado = threading.Lock()

    def enviar(texto):
        with candado:
            sp.write((texto + '\n').encode())

    def latido():
        while vivo.is_set():
            enviar('')
            time.sleep(0.1)

    def lector():
        columnas, ultimo = None, 0.0
        while vivo.is_set():
            crudo = sp.readline()
            if not crudo:
                continue
            linea = crudo.decode(errors='replace').strip()
            tipo, valor = clasificar(linea, columnas)
            if tipo == 'encabezado':
                columnas = valor
                log.write(linea + '\n')
            elif tipo == 'dato':
                log.write(linea + '\n')
                if time.time() - ultimo > 0.5:
                    ultimo = time.time()
                    print('  ' + resumen(valor))
            elif linea:
                print(linea)
                mensajes.write(f'{time.time():.3f} {linea}\n')

    for f in (latido, lector):
        threading.Thread(target=f, daemon=True).start()
    enviar('h')     # pide el encabezado del CSV
    try:
        while True:
            enviar(input())
    except (KeyboardInterrupt, EOFError):
        print('\nPARO y salida')
        enviar('x')
        time.sleep(0.2)
    finally:
        vivo.clear()
        time.sleep(0.2)
        sp.close()
        log.close()
        mensajes.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
