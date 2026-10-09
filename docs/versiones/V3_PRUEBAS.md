# OSMIOR V3 - Comunicaciones robustas

## Orden de flasheo
1. Flashea `OSMIOR_UNO_V3.ino` en el UNO.
2. Reinicia el UNO y confirma en Monitor Serie a 115200: `OSMIOR UNO V3 - comms optimized` y `[DF] init OK` (si el DFPlayer funciona).
3. Carga `OSMIOR_NICLA_V3.py` en la NICLA y ejecuta. Debe seguir mostrando `MODEL OK: True` y la memoria del modelo.
4. Ejecuta `OSMIOR_HMI_V3.py` en Windows.

## Protocolo NICLA -> UNO
- Movimiento: `F`, `B`, `L`, `R` o `S` (un byte; el UNO ejecuta inmediatamente).
- Audio: `A1` ... `A5` (dos bytes).
- Telemetría: `?` (el UNO responde una trama compacta).

## Protocolo UNO -> NICLA
- ACK movimiento: `K,M,F\n` (análogos B/L/R/S).
- ACK audio: `K,A,5\n`.
- Telemetría: `T,DL_cm,DR_cm,DF,STATE,RX_BAD,SW_OVERFLOW,LINE_BAD\n`.

La NICLA lee el UART como bytes; no llama a `decode()` para el tráfico del UNO.
El UNO no emite telemetría periódica: responde cuando la NICLA pregunta cada 500 ms,
siempre que no haya comandos o ACK pendientes. La NICLA aplaza el siguiente envío
mientras espera el turno de transmisión del UNO.

## HMI: latencia
La HMI añade un número de secuencia a CMD/AUD sobre TCP. La NICLA lo conserva para
correlacionar el ACK del UNO. El `ACK RTT` visible en la interfaz es el tiempo de ida
y vuelta HMI -> NICLA -> UNO -> NICLA -> HMI (no es solo el tiempo de aplicación del motor).
`UART_MS` en el evento es la medición de UART desde escritura de la NICLA hasta el ACK.

## Diagnóstico
- `URXBAD`: bytes no imprimibles descartados por la NICLA en la UART.
- `NURXBAD`: bytes no imprimibles descartados por el UNO.
- `UOVF`: overflow reportado por `SoftwareSerial` en el UNO.
- `ULINE`: líneas desconocidas/overflow de buffer, contados por el UNO.
- ACK sin respuesta expira a los 300 ms y no bloquea para siempre el sondeo.

## Cambios
- Motores intactos: D3-D9, PWM 220 y las funciones originales.
- `pulseIn` pasó de 25 ms a 15 ms de timeout; máxima distancia teórica ~2.57 m.
- Sensores se actualizan a 10 Hz, pero la telemetría UART sale a demanda a 2 Hz.
- Línea de telemetría más corta en UART.
- Una sola instancia SoftwareSerial escucha recepción del NICLA.
- Google person_detect.tflite, cámara, IMU, HMI plots y modelo físico se conservan.

## Seguridad al probar
Levanta el rover o deja las ruedas sin tocar el suelo en la primera prueba. Pulsa una
sola dirección durante 0.5-1 s y confirma el ACK; verifica que STOP se reconozca al soltar.
No dejes el rover en el suelo sin supervisión durante las primeras pruebas.
