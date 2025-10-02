#!/usr/bin/env python3
# serial_arduino_comm.py
# Requiere pyserial

import sys
import time
import serial
import serial.tools.list_ports
from realtime_plot import plotter


BAUD = 115200
HANDSHAKE_SEND = "[HELP]"
HANDSHAKE_EXPECT = "[HELP] - WELCOME to the HAMALGAMH Testing System"
TIMEOUT = 2  # segundos

def list_serial_ports():
    """Devuelve lista de (device, desc)"""
    ports = serial.tools.list_ports.comports()
    return [(p.device, p.description) for p in ports]

def auto_find_arduino():
    """Intenta encontrar un puerto probable de Arduino.
    Simple heurística: busca nombres típicos en distintos OS.
    Devuelve device o None."""
    candidates = list_serial_ports()
    if not candidates:
        return None
    # preferencias por substrings típicas
    substrings = ['usbmodem', 'usbserial', 'usb', 'ACM', 'COM']  # COM para Windows
    for dev, desc in candidates:
        low = dev.lower() + " " + desc.lower()
        if any(s.lower() in low for s in substrings):
            return dev
    # si nada, devolver el primero
    return candidates[0][0]

def open_serial(device, baud=BAUD, timeout=TIMEOUT):
    try:
        ser = serial.Serial(device, baudrate=baud, timeout=timeout)
        # pequeño retraso para que se estabilice la conexión
        time.sleep(0.2)
        # flush input/output
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        return ser
    except serial.SerialException as e:
        print(f"Error abriendo {device}: {e}")
        return None

def handshake(ser, send=HANDSHAKE_SEND, expect=HANDSHAKE_EXPECT, tries=3):
    for attempt in range(1, tries+1):
        ser.write((send + '\n').encode('utf-8'))
        ser.flush()
        # esperar respuesta
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print(f"Respuesta recibida: {line}")
            if line.__contains__(expect):
                while True:
                    line = ser.readline().decode('utf-8', errors='ignore')
                    if not line:
                        break
                    print(f"< {line.strip()}")
                return True
        else:
            print(f"No hay respuesta (intento {attempt}/{tries}).")
    return False

def interactive_loop(ser):
    print("Entrando en modo interactivo. Escribe texto y se enviará al Arduino (CTRL-C para salir).")
    try:
        while True:
            to_send = input("> ")
            ser.write((to_send + '\n').encode('utf-8'))
            ser.flush()
            # lee respuestas hasta timeout
            start = time.time()
            while True:
                line = ser.readline().decode('utf-8', errors='ignore')
                if not line:
                    break
                print(f"< {line.strip()}")
                # evitar bucle infinito: si tardó > 0.1s entre lecturas, salimos de lectura inmediata
                # if time.time() - start > 0.1:
                #     break
    except KeyboardInterrupt:
        print("\nSaliendo...")
    finally:
        ser.close()

def main():
    print("Puertos serie disponibles:")
    for dev, desc in list_serial_ports():
        print(f"  {dev}  --  {desc}")
    device = auto_find_arduino()
    if device is None:
        print("No se detectaron puertos serie. Especifica el puerto como argumento.")
        sys.exit(1)
    print(f"Intentando usar puerto: {device}")

    ser = open_serial(device)
    if ser is None:
        sys.exit(1)

    print("Realizando handshake...")
    if handshake(ser):
        print("Handshake OK.")
    else:
        print("Handshake fallido. Aún puedes usar el modo interactivo.")

    plotter.on_point = lambda x, y, source: print(f"[PLOTTER] x{x}, y{y}, source{source}")  # aquí tu lógica
    plotter.start_in_thread()         # abre la ventana (no bloquea)
    plotter.put_point(0.0, 0.0)       # envía puntos cuando quieras
    plotter.put_point(1.0, 1.2)
    interactive_loop(ser)

if __name__ == "__main__":
    main()
