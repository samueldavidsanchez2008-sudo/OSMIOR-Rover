# Rover Marciano — Control por LoRa

Rover de 6 ruedas con suspensión rocker-bogie, controlado de forma inalámbrica
mediante comunicación LoRa. El control se hace desde una PC con Windows,
usando el teclado.

## Arquitectura

```
[Teclado]  --USB/Serial-->  [Placa Transmisora]  --LoRa-->  [Placa Receptora]  -->  [6 Motores]
(control_teclado.py)        (rover_transmisor_serial.ino)  (rover_receptor_6motores.ino)
```

- La **placa transmisora** no tiene motores ni sensores propios: solo recibe
  comandos por cable USB desde la PC y los retransmite por LoRa.
- La **placa receptora** está montada en el rover, junto a los motores. Recibe
  los comandos por LoRa y mueve las 6 ruedas según corresponda.

## Archivos

| Archivo | Dónde va | Qué hace |
|---|---|---|
| `rover_receptor_6motores.ino` | Placa dentro del rover (con los motores) | Recibe comandos por LoRa y controla los 6 motores (3 por lado) |
| `rover_transmisor_serial.ino` | Placa conectada a la PC por USB | Recibe comandos por Serial desde la PC y los reenvía por LoRa |
| `control_teclado.py` | Corre en la PC con Windows | Lee las teclas WASD y las envía por Serial a la placa transmisora |

## Protocolo de comandos

Un solo carácter define la acción:

| Carácter | Acción |
|---|---|
| `F` | Adelante |
| `B` | Atrás |
| `L` | Girar a la izquierda (sobre su propio eje) |
| `R` | Girar a la derecha (sobre su propio eje) |
| `S` | Detener |

Por USB (PC → placa transmisora) se envía el carácter solo (ej. `F`).
Por LoRa (transmisora → receptora) se envía con el prefijo `CMD_` (ej. `CMD_F`).

## Motores

6 motores agrupados en 2 lados (izquierda y derecha), 3 por lado. Los motores
de un mismo lado siempre giran igual (mismo sentido y velocidad) — no hay
control independiente por rueda. Para girar, un lado avanza mientras el otro
retrocede (como un tanque).

> Si los 3 motores de un lado van en paralelo a un solo driver, ese driver
> debe soportar la corriente combinada de los 3 (ej. BTS7960); un L298N no
> alcanza.

## Cómo correr el control por teclado

1. Sube `rover_transmisor_serial.ino` a la placa conectada por USB a la PC.
2. Sube `rover_receptor_6motores.ino` a la placa del rover.
3. En la PC:
   ```
   pip install pygame pyserial
   ```
4. Edita `PUERTO_SERIAL` en `control_teclado.py` con el puerto COM real
   (se ve en el Administrador de Dispositivos de Windows).
5. Ejecuta:
   ```
   python control_teclado.py
   ```
6. Con la ventana en foco: `W` adelante, `S` atrás, `A` izquierda, `D` derecha.
   `ESC` o cerrar la ventana para salir.

## Seguridad

Si la PC deja de enviar datos (se cierra el script, se desconecta el cable),
la placa transmisora detiene el rover automáticamente después de 500 ms sin
recibir nada por Serial.

## Pendiente

- Control por mando (Xbox/PS4) vía Bluetooth — se dejó pausado, se puede
  retomar más adelante con la librería Bluepad32.
