
import serial
import socket
import threading
import time

# =====================================
# CONEXION CON ARDUINO
# =====================================

PUERTO = "COM7"
BAUDIOS = 9600

# =====================================
# CONEXION CON TELEPLOT
# =====================================

TELEPLOT = ("127.0.0.1", 47269)

# Variables que queremos graficar
# Las demas no se envian a Teleplot.

VARIABLES = {
    "RPM_Objetivo": "RPM",
    "RPM_Real": "RPM",
    "RPM_Filtrada": "RPM",
    "Aceleracion_RPM_s": "RPM/s",
    "Grados_Objetivo": "grados",
    "Grados_Estimados": "grados",
}

detener = threading.Event()


# =====================================
# LEER DATOS DEL ARDUINO
# =====================================

def recibir(arduino, udp):

    while not detener.is_set():

        try:
            linea = arduino.readline().decode(
                "utf-8",
                errors="replace"
            ).strip()

            if not linea:
                continue

            # =================================
            # DATOS PARA LAS GRAFICAS
            # =================================

            if linea.startswith(">") and ":" in linea:

                nombre, valor = linea[1:].split(":", 1)

                # Solo enviar variables permitidas
                if nombre in VARIABLES:

                    unidad = VARIABLES[nombre]

                    mensaje = (
                        f"{nombre}:{valor}"
                        f"§{unidad}|g"
                    )

                    udp.sendto(
                        mensaje.encode("utf-8"),
                        TELEPLOT
                    )

                # PWM y otras variables no aparecen
                # ni crean graficas adicionales.

            # =================================
            # MENSAJES DEL MENU
            # =================================

            else:
                print("\n" + linea, flush=True)

        except (serial.SerialException, OSError):

            detener.set()

            print(
                "\nERROR: Conexion con Arduino perdida."
            )

            break


# =====================================
# PROGRAMA PRINCIPAL
# =====================================

def main():

    udp = socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM
    )

    detener.clear()

    try:

        with serial.Serial(
            PUERTO,
            BAUDIOS,
            timeout=0.2,
            write_timeout=1
        ) as arduino:

            print("==========================")
            print("   ARDUINO Y TELEPLOT")
            print("==========================")
            print("Arduino conectado en COM7")
            print("Graficas habilitadas")
            print("Escribe SALIR para cerrar")
            print("==========================")

            hilo = threading.Thread(
                target=recibir,
                args=(arduino, udp),
                daemon=True
            )

            hilo.start()

            # =================================
            # ENVIAR COMANDOS AL MOTOR
            # =================================

            while not detener.is_set():

                try:

                    comando = input(
                        "\nComando > "
                    ).strip()

                    if comando.upper() == "SALIR":
                        break

                    if comando:

                        arduino.write(
                            (comando + "\n").encode(
                                "ascii",
                                errors="ignore"
                            )
                        )

                except (KeyboardInterrupt, EOFError):
                    break

                except (serial.SerialException, OSError):
                    print("No se pudo enviar el comando.")
                    break

            # =================================
            # SALIDA SEGURA
            # =================================

            try:
                arduino.write(b"P\n")
                arduino.flush()
                time.sleep(0.15)

            except (serial.SerialException, OSError):
                pass

            detener.set()

            hilo.join(timeout=0.5)

    except serial.SerialException as error:

        print("ERROR AL ABRIR COM7")
        print(error)
        print(
            "Cierra COM7 en Arduino IDE y Teleplot."
        )

    finally:
        udp.close()


if __name__ == "__main__":
    main()
