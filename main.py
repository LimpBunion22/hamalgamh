#!/usr/bin/env python3
# serial_arduino_comm.py
# Requiere pyserial

import sys
import time
import serial
import serial.tools.list_ports
from realtime_plot import plotter
from datetime import datetime

PAR_COEF = 1.0

BAUD = 115200
HANDSHAKE_SEND = "[HELP]"
HANDSHAKE_EXPECT = "[HELP] - WELCOME to the HAMALGAMH Testing System"
TIMEOUT = 2  # segundos


KNOWN_VID_PID = {
    (0x2341, 0x804E), # Arduino MKR1010
    (0x2341, 0x0043), # Arduino Uno (Arduino LLC)
    (0x2341, 0x0001), # Arduino Uno (antiguo)
    (0x2A03, 0x0043), # Genuino/Arduino
    (0x1A86, 0x7523), # CH340/CH341 (clones)
    (0x10C4, 0xEA60), # CP210x
    (0x0403, 0x6001), # FTDI FT232
}

def list_serial_ports():
    """Devuelve lista de (device, desc)"""
    ports = serial.tools.list_ports.comports()
    return [(p.device, p.description) for p in ports]

def auto_find_arduino():
    ports = serial.tools.list_ports.comports()
    # 1) Coincidencia por VID/PID conocidos
    for p in ports:
        if p.vid is not None and p.pid is not None:
            if (p.vid, p.pid) in KNOWN_VID_PID:
                return p.device
    # 2) Fallback por manufacturer/product si existen
    for p in ports:
        info = f"{p.manufacturer or ''} {p.product or ''}".lower()
        if any(k in info for k in ["arduino", "genuino", "ftdi", "wch", "silicon labs", "cp210", "ch340", "ch341"]):
            return p.device
    # 3) Último recurso: el primero “USB/ACM/COM”
    for p in ports:
        if any(s in p.device.lower() for s in ["usb", "acm", "com"]):
            return p.device
    return None

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
        print(f"<[SYSTEM] Error abriendo {device}: {e}")
        return None

def handshake(ser, send=HANDSHAKE_SEND, expect=HANDSHAKE_EXPECT, tries=3):
    for attempt in range(1, tries+1):
        ser.write((send + '\n').encode('utf-8'))
        ser.flush()
        # esperar respuesta
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print(f"<[SYSTEM] Respuesta recibida: {line}")
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
    print("<[SYSTEM] Entrando en modo interactivo.")
    print("<[SYSTEM] Esperando comando (CTRL-C para salir).")
    try:
        while True:
            # to_send = input("> ")

            while True:
                time.sleep(0.1)
                if plotter.event == True:
                    plotter.event = False
                    break

            to_send = plotter.eventCmd  
            ser.write((to_send + '\n').encode('utf-8'))
            ser.flush()
            # lee respuestas hasta timeout
            while True:
                line = ser.readline().decode('utf-8', errors='ignore')
                if not line:
                    break
                print(f"<[ARDUINO]   {line.strip()}")

            if(to_send.__contains__("[EXIT]")):
                raise KeyboardInterrupt

            if(to_send.__contains__("[RUN]")):
                plotter._arduino_x.clear()
                plotter._arduino_y.clear()
                now = datetime.now()
                testName = now.strftime("HamalgamhTest_%Y_%m_%d__%H_%M_%S")
                testData = testName + ";" + str(len(plotter._points_x)) + ";"
                # testData += str(plotter._points_y[1]) + "," + str(0) + ";"
                for i in range(len(plotter._points_x)):
                    print(f"<[SYSTEM] Datos y{plotter._points_y[i]} x{plotter._points_x[i]}")

                for i in range(len(plotter._points_x)):
                    testData += str(plotter._points_y[i]) + "," + str(1000*(plotter._points_x[i])) + ";"
                
                print(f"<[SYSTEM] Ejecutando operacion \n\t{testData}")
                ser.write((testData + '\n').encode('utf-8'))
                ser.flush()

                while True:
                    line = ser.readline().decode('utf-8', errors='ignore')
                    if not line:
                        break
                    print(f"<[ARDUINO]   {line.strip()}")
                    if line.__contains__("Reading complete"):
                        break
                
                start = time.time()
                xTime = 0.0
                positions = []
                parOrPos = True
                while True:
                    line = ser.readline().decode('utf-8', errors='ignore')
                    if not line:
                        if time.time() - start > 2:
                            break
                        else:
                            continue
                    print(f"<[ARDUINO]   {line.strip()}")
                    if line.__contains__("COMPLETED"):
                        plotter.set_second_graph(positions, plotter._arduino_y)
                        break
                    y = 0.0
                    try:
                        y = float(line)
                    except:
                        y = -1
                    if parOrPos:
                        plotter.put_point_arduino(xTime,y*PAR_COEF)
                        xTime += 0.1   
                    else:
                        positions.append(y)
                    parOrPos = not(parOrPos)

            
    except KeyboardInterrupt:
        print("\n<[SYSTEM] Saliendo...")
    finally:
        ser.close()
        print("\n<[SYSTEM] Fin de programa")

def main():
    print("<[SYSTEM] Puertos serie disponibles:")
    for dev, desc in list_serial_ports():
        print(f"  {dev}  --  {desc}")
    device = auto_find_arduino()
    if device is None:
        print("<[SYSTEM] No se detectaron puertos serie. Especifique el puerto como argumento.")
        sys.exit(1)
    print(f"<[SYSTEM] Intentando usar puerto: {device}")

    ser = open_serial(device)
    if ser is None:
        sys.exit(1)

    print("<[SYSTEM] Realizando handshake...")
    if handshake(ser):
        print("<[SYSTEM] Handshake OK.")
    else:
        print("<[SYSTEM] Handshake fallido. Aún puedes usar el modo interactivo.")

    plotter.on_point = lambda x, y, source: print(f"<[SYSTEM][PLOTTER] x{x}, y{y}, source{source}")  # aquí tu lógica
    plotter.start_in_thread()         # abre la ventana (no bloquea)
    plotter.put_point(0.0, 0.0)       # envía puntos cuando quieras
    plotter.put_point(1.0, 1.2)
    interactive_loop(ser)

if __name__ == "__main__":
    main()
