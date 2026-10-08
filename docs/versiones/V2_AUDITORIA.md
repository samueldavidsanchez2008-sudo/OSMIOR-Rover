# OSMIOR V2 - AUDITORIA Y OPTIMIZACION

## 1. ERROR QUE EXPLICA LOS MOTORES SIN RESPUESTA

El fallo principal estaba en el UNO, no en el HMI ni en el HC-SR04.

Se usaban dos objetos `SoftwareSerial`:

- `niclaSerial(10,11)` para recibir comandos desde la NICLA.
- `dfPlayerSerial(13,12)` para el DFPlayer.

En Arduino AVR, `SoftwareSerial` permite varias instancias, pero solo UNA puede estar escuchando RX a la vez. El `begin()` del segundo puerto puede dejar al DFPlayer como la instancia activa de recepcion. Por eso el UNO podia seguir transmitiendo telemetria del ultrasonico a la NICLA, pero `niclaSerial.available()` no veia los comandos entrantes.

La documentacion oficial de Arduino explicita esta limitacion.

### Correccion

1. El DFPlayer se usa solo en TX.
2. El TX del DFPlayer NO se conecta al UNO.
3. D13 queda como RX dummy del objeto `SoftwareSerial`.
4. Despues de inicializar/enviar al DFPlayer se llama `niclaSerial.listen()`.
5. Antes de leer del NICLA tambien se fuerza `niclaSerial.listen()`.

Esto conserva la comunicacion de entrada NICLA -> UNO.

## 2. SEGUNDO CAMBIO: PROTOCOLO CON ACK

Antes, si un comando se perdia, el HMI solo parecia silencioso.

Ahora:

HMI -> NICLA:
`CMD,F`

NICLA -> UNO:
`CMD,F`

UNO -> NICLA:
`ACK,CMD,F`

NICLA -> HMI:
`EVT;UNO_ACK=CMD,F`

Para audio:

`AUD,3` -> `ACK,AUD,3`

Esto permite saber exactamente si el comando llego al UNO.

## 3. RAM: MODELO GOOGLE PERSON DETECTION

La placa Nicla Vision usa un STM32H747; el firmware OpenMV corre en el Cortex-M7 y la placa dispone de 1 MB de SRAM interna y 16 MB de flash QSPI externa.

El modelo Google/TFLite Micro `person_detect.tflite` es un modelo cuantizado de aproximadamente 300 KB, con entrada 96x96 grayscale y dos clases. El benchmark de TFLite Micro reporta un tensor arena de alrededor de 135 KB para ese modelo; el valor exacto que usa OpenMV se consulta en `model.ram` en tiempo de ejecucion.

### Punto clave

En OpenMV 5.x, `ml.Model(..., load_to_fb=True)` ya NO es valido: el argumento fue eliminado y la gestion de memoria paso al `unified allocator`.

Por eso el codigo nuevo NO utiliza `load_to_fb=True`.

La estrategia correcta para el modelo Google actual es:

1. Abrir Open Model Zoo del OpenMV IDE.
2. Seleccionar Google -> person_detection.
3. Copiarlo/compilarlo hacia ROMFS de la Nicla.
4. El script busca automaticamente cualquier `.tflite` que contenga `person_detection` o `person_detect` dentro de `/rom`.
5. Carga con `ml.Model(path)`.
6. Imprime `model.len`, `model.ram`, `model.input_shape`, `model.output_shape` y memoria libre.

ROMFS es importante porque los modelos alojados ahi se pueden ejecutar directamente desde flash sin copiar los pesos completos a RAM. El tensor arena y los buffers temporales de inferencia siguen usando RAM.

## 4. ESTRATEGIA DE MEMORIA IMPLEMENTADA

Antes del modelo:
- camara en grayscale
- QQVGA 160x120
- stream del host desactivado
- no hay sockets abiertos todavia
- `gc.collect()`

Despues del modelo:
- se inicializa UART
- se inicializa Wi-Fi
- se crea el servidor TCP

Durante ejecucion:
- inferencia cada 500 ms (2 Hz)
- telemetria cada 200 ms (5 Hz)
- IMU cada 50 ms (20 Hz)
- `gc.collect()` cada 1 s
- no se guardan frames en una lista
- no se hace `String` dinamico en el UNO

## 5. PERSON DETECTION

El modelo Google usado es de PRESENCIA DE PERSONA, no es un detector tipo YOLO.

Se interpreta la salida como:

`[no_person, person]`

El script usa el segundo score por defecto y umbral configurable de 0.60.

Tambien agrega histeresis temporal:

- 2 detecciones consecutivas -> PERSON=1
- 2 ausencias consecutivas -> PERSON=0

Audio 005 se dispara en flanco de subida, con cooldown.

## 6. ERRORES FISICOS CORREGIDOS

### Torque

No existe sensor de corriente ni encoder, y la ficha técnica disponible del TT 1:90 no publica torque de stall.

Por tanto el sistema NO presenta torque de motor como una medicion.

Muestra:

`TAUREQ`: torque requerido por motor debido a pendiente + resistencia a rodadura.

`TAUTOTAL`: torque requerido equivalente de ambos motores.

`FREQ`: fuerza tractiva requerida.

La velocidad teorica usa:

- salida TT: 60 rpm a 3 V, 90 rpm a 4.5 V, 120 rpm a 6 V
- 5 V de alimentacion del driver
- PWM 220/255
- rueda de 70 mm

Se obtiene aproximadamente:

`Veq = 4.31 V`

`RPM teoricas = 86.3`

`v teorica = 0.316 m/s = 1.14 km/h`

La velocidad no es medida porque no hay encoder.

### Angulo critico

Con 0.35 m de base y h=0.25 m conservador:

`theta_crit = atan(0.175/0.25) ~= 35 deg`

Rojo comienza 5 deg antes: 30 deg.

## 7. HMI V2

La estructura conserva la idea visual tipo terminal/bash:

IZQUIERDA:
- gauges Pitch/Roll
- estado Green/Yellow/Red
- angulo critico
- margen
- cargas normales

CENTRO:
- velocidad vs tiempo
- torque requerido vs tiempo
- pitch/roll/critico vs tiempo
- cargas normales vs tiempo
- HC-SR04 vs tiempo
- fuerza de traccion vs tiempo
- acelerometro vs tiempo
- giroscopio vs tiempo

DERECHA:
- W/A/S/D/X
- pregunta por persona
- test de audios 1-5
- persona + score
- HC-SR04
- ACK del UNO
- DFPlayer
- memoria/modelo

ABAJO:
- autonomia estimada
- eventos/ACK

## 8. WATCHDOG

HMI envia `HB` cada 500 ms.

NICLA fuerza STOP si deja de recibir mensajes durante 2.5 s y cierra la conexion.

Al reconectar, HMI nunca reutiliza comandos viejos de movimiento.

## 9. LIMITACIONES DECLARADAS

- Sin encoder: no hay velocidad real de rueda.
- Sin sensor de corriente: no hay corriente de motor medida.
- Sin torque de stall del fabricante: no hay torque motor absoluto medido/modelado de forma unica.
- La velocidad IMU es una estimacion con deriva y se fuerza a cero en STOP para visualizacion operativa.
- Las cargas normales son un modelo equivalente por eje, no mediciones por rueda.
- La autonomia es estimada a partir del tiempo de marcha y `FULL_RUNTIME_MIN`.

## 10. ORDEN DE PRUEBA RECOMENDADO

1. UNO solo + motores por USB: F/B/L/R/S.
2. UNO + DFPlayer: reproducir 1-5 por USB enviando `AUD,n` desde NICLA o un puente.
3. UNO + NICLA: verificar en monitor que aparece `ACK,CMD,F`.
4. NICLA sin ML: verificar HMI/IMU/ultrasonicos.
5. Cargar Google person_detection en ROMFS y verificar `MODEL OK`, `MODEL RAM` y `MODEL INPUT`.
6. HMI completo.

