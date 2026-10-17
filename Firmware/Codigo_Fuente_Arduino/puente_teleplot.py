
"""Puente serial entre Arduino Uno y las graficas UDP de Teleplot.

Requiere pyserial: py -m pip install pyserial
El puerto COM3 debe estar libre (Monitor Serial y Serial de Teleplot cerrados).
"""

import serial
import socket
import threading
import time

# ----------
# CONFIGURACION

PUERTO = "COM3"
BAUDIOS = 9600
TELEPLOT = ("127.0.0.1", 47269)

# Variables mostradas durante las pruebas de velocidad.
VARIABLES = {
    "RPM_Objetivo": "RPM",
    "RPM_Real": "RPM",
    "RPM_Filtrada": "RPM",
    "PWM_Aplicado": "PWM",
    "Aceleracion_RPM_s": "RPM/s",
}

# Estas variables aparecen solo si DIAGNOSTICO_ENCODER = 1 en el .ino.
# No hace falta cambiar este archivo para activar el diagnostico.
DIAGNOSTICOS = {
    "D": (
        ("Pulsos_Flancos", "pulsos"),
        ("Pulsos_Netos", "pulsos"),
        ("Intervalo_RPM_ms", "ms"),
        ("RPM_Flancos", "RPM"),
    ),
    "Q": (
        ("Quad_Adelante", "transiciones"),
        ("Quad_Atras", "transiciones"),
        ("Quad_Invalidas", "transiciones"),
        ("Quad_Repetidas", "transiciones"),
    ),
}

detener = threading.Event()

# ----------
# ENVIO DE DATOS A TELEPLOT

def graficar(udp, nombre, valor, unidad):
    mensaje = f"{nombre}:{valor}§{unidad}|g"
    udp.sendto(mensaje.encode("utf-8"), TELEPLOT)


def procesar_telemetria(linea, udp):
    """Procesa un mensaje del Arduino, devolviendo True si es telemetria."""
    if not linea.startswith(">") or ":" not in linea:
        return False

    nombre, valor = linea[1:].split(":", 1)

    # Paquetes compactos opcionales del diagnostico de cuadratura.
    if nombre in DIAGNOSTICOS:
        campos = DIAGNOSTICOS[nombre]
        valores = valor.split(",")
        if len(valores) == len(campos):
            for (variable, unidad), dato in zip(campos, valores):
                graficar(udp, variable, dato, unidad)
        return True

    # Envio habitual: solo RPM, PWM y aceleracion.
    if nombre in VARIABLES:
        graficar(udp, nombre, valor, VARIABLES[nombre])
    return True


# ----------
# LECTURA DEL PUERTO SERIAL

def recibir(arduino, udp):
    while not detener.is_set():
        try:
            linea = arduino.readline().decode("utf-8", errors="replace").strip()
            if linea and not procesar_telemetria(linea, udp):
                print("\n" + linea, flush=True)
        except (serial.SerialException, OSError):
            detener.set()
            print("\nERROR: Conexion con Arduino perdida.")
            break


# ----------
# EJECUCION Y COMANDOS

def main():
    detener.clear()
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        with serial.Serial(PUERTO, BAUDIOS, timeout=0.2, write_timeout=1) as arduino:
            print("----------")
            print("ARDUINO Y TELEPLOT")
            print(f"Puerto: {PUERTO} | Baudios: {BAUDIOS}")
            print("Escribe SALIR para cerrar")

            hilo = threading.Thread(target=recibir, args=(arduino, udp), daemon=True)
            hilo.start()

            while not detener.is_set():
                try:
                    comando = input("\nComando > ").strip()
                    if comando.upper() == "SALIR":
                        break
                    if comando:
                        arduino.write((comando + "\n").encode("ascii", errors="ignore"))
                except (KeyboardInterrupt, EOFError):
                    break
                except (serial.SerialException, OSError):
                    print("No se pudo enviar el comando.")
                    break

            # Intentar detener el motor antes de cerrar el puerto.
            try:
                arduino.write(b"P\n")
                arduino.flush()
                time.sleep(0.15)
            except (serial.SerialException, OSError):
                pass

            detener.set()
            hilo.join(timeout=0.5)

    except serial.SerialException as error:
        print(f"ERROR AL ABRIR {PUERTO}")
        print(error)
        print(f"Cierra {PUERTO} en Arduino IDE y Teleplot.")
    finally:
        detener.set()
        udp.close()


if __name__ == "__main__":
    main()