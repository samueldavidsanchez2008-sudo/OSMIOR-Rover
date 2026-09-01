"""
Control del rover por teclado (Windows).

Requisitos (una sola vez):
    pip install pygame pyserial

Uso:
    1. Conecta la placa transmisora por USB a la PC.
    2. Averigua el puerto COM en el Administrador de Dispositivos
       de Windows (ej. "COM5") y ponlo abajo en PUERTO_SERIAL.
    3. Ejecuta: python control_teclado.py
    4. Con la ventana que se abre en foco, usa:
         W = adelante   S = atrás
         A = izquierda  D = derecha
       (sin tecla presionada = detener)
    5. Cierra la ventana o presiona ESC para salir.
"""

import sys
import serial
import pygame

# --- Configuración ---
PUERTO_SERIAL = "COM5"   # <-- cambia esto por tu puerto real
BAUDRATE = 115200

def main():
    try:
        ser = serial.Serial(PUERTO_SERIAL, BAUDRATE, timeout=1)
    except serial.SerialException as e:
        print(f"No se pudo abrir el puerto {PUERTO_SERIAL}: {e}")
        print("Revisa el Administrador de Dispositivos de Windows para el nombre correcto (ej. COM5).")
        sys.exit(1)

    pygame.init()
    pantalla = pygame.display.set_mode((420, 200))
    pygame.display.set_caption("Control del Rover - WASD")
    fuente = pygame.font.SysFont(None, 28)

    ultimo_comando = None
    reloj = pygame.time.Clock()
    corriendo = True

    while corriendo:
        for evento in pygame.event.get():
            if evento.type == pygame.QUIT:
                corriendo = False
            if evento.type == pygame.KEYDOWN and evento.key == pygame.K_ESCAPE:
                corriendo = False

        teclas = pygame.key.get_pressed()

        # Prioriza adelante/atrás si se presionan varias teclas a la vez
        if teclas[pygame.K_w]:
            comando = "F"
        elif teclas[pygame.K_s]:
            comando = "B"
        elif teclas[pygame.K_a]:
            comando = "L"
        elif teclas[pygame.K_d]:
            comando = "R"
        else:
            comando = "S"

        if comando != ultimo_comando:
            ser.write(comando.encode("ascii"))
            ultimo_comando = comando
            print(f"Comando enviado: {comando}")

        # Dibuja la ventana con instrucciones y el comando actual
        pantalla.fill((30, 30, 30))
        lineas = [
            "W = adelante   S = atrás",
            "A = izquierda  D = derecha",
            "ESC o cerrar ventana = salir",
            f"Comando actual: {comando}",
        ]
        for i, linea in enumerate(lineas):
            texto = fuente.render(linea, True, (255, 255, 255))
            pantalla.blit(texto, (20, 20 + i * 35))
        pygame.display.flip()

        reloj.tick(20)  # 20 actualizaciones por segundo es suficiente

    # Al salir, detener el rover por seguridad
    try:
        ser.write(b"S")
    except serial.SerialException:
        pass
    ser.close()
    pygame.quit()


if __name__ == "__main__":
    main()
