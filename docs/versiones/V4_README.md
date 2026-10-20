# OSMIOR V4

Archivos: `OSMIOR_UNO_V4.ino` · `OSMIOR_NICLA_V4.py` · `OSMIOR_HMI_V4.py`

Qué se probó y qué no: el sketch del UNO solo pasó una revisión de sintaxis (sin el toolchain AVR ni las librerías reales), la lógica de la NICLA se probó con simuladores y la HMI se ejecutó sin pantalla con telemetría falsa. Nada se ha probado todavía en el robot.

## 1. Pinout del UNO (16 pines útiles, los 16 usados)

| Pin | Función | Cambio vs V3 |
|---|---|---|
| D2 | DHT11 DATA | nuevo |
| D3 | PWM tracción: **PWMA del driver 1 + PWMA y PWMB del driver 2, los tres unidos aquí** | PWMB (antes D6) pasa a D3 |
| D4 / D5 | Driver 1 canal A, lado izq (3 motores): AIN1 / AIN2 | igual |
| D6 | PWM recolección: driver 1 canal B (PWMB) | antes PWM der |
| D7 / D8 | Lado der (3 motores) en el driver 2: **AIN1 y BIN1 unidos a D7, AIN2 y BIN2 unidos a D8** | igual en el UNO; cambia el cableado del driver |
| D9 | Recolección IN1: driver 1 canal B (BIN1) | antes STBY |
| D10 / D11 | UART NICLA: RX / TX | igual |
| D12 | UNO → DFPlayer RX (con 1 kΩ en serie) | igual |
| D13 | Recolección IN2: driver 1 canal B (BIN2) | antes RX dummy del DFPlayer |
| A0 | Servo del radar (señal) | nuevo |
| A1 / A2 / A3 | HC-SR04 izq / der / radar, **un solo pin cada uno** (NewPing) | antes A0–A3 con TRIG y ECHO separados |
| A4 / A5 | I2C: SDA / SCL del BMP280 (dirección **0x76**) | nuevo |
| STBY (ambos drivers) | **a 5 V fijo** | libera D9 |

Las tres decisiones que liberan pines (PWM unido, STBY fijo, TRIG+ECHO en un pin) son lo que permite que todo quepa. Con TRIG y ECHO separados en los tres ultrasónicos faltarían 3 pines.

### Ultrasónicos con NewPing (1 pin)
Une TRIG y ECHO de cada HC-SR04 al mismo pin del UNO. **Pon una resistencia de ~1 kΩ en serie en el ECHO** (ECHO → 1 kΩ → pin; TRIG directo al pin). Sin ella el ECHO (que queda en bajo) pelea con el pin cuando el UNO lo pone en alto. No todos los módulos HC-SR04 aguantan este modo: prueba cada uno por separado antes de armar todo.

### DFPlayer
Se desconecta el **TX del DFPlayer** (el UNO nunca lo escucha). Solo van: 5 V, GND, D12 → 1 kΩ → RX del DFPlayer, y el parlante. Consecuencia: sin ese cable de retorno el UNO no puede saber si la SD falló, así que `DF=1` solo significa "inicializado". El audio 6 (`006`) debe existir en la SD en la misma posición que usabas para 1–5 (`play(n)` usa el orden de copia).
El segundo `SoftwareSerial` ahora comparte D10 como RX dummy, y el UNO vuelve a llamar `niclaSerial.listen()` después de cada envío, así que no se repite el conflicto de V2.

## 2. Distribución de los motores en los dos drivers (revisada)

| Driver | Canal | Carga | Pines del UNO |
|---|---|---|---|
| 1 | A | 3 motores del lado izquierdo, en paralelo | PWMA → D3, AIN1 → D4, AIN2 → D5 |
| 1 | B | motor de recolección (dedicado) | PWMB → D6, BIN1 → D9, BIN2 → D13 |
| 2 | A | 2 motores del lado derecho | PWMA → D3, AIN1 → D7, AIN2 → D8 |
| 2 | B | 1 motor del lado derecho | PWMB → D3, BIN1 → D7, BIN2 → D8 |

Los dos canales del driver 2 reciben las mismas señales (los pines de entrada van unidos), así que juntos son el lado derecho con sus 3 motores. El UNO no cambia respecto a la versión anterior, solo el cableado de los drivers. STBY de ambos drivers a 5 V fijo.

Qué cambia en la práctica: el lado derecho reparte su corriente en dos canales, y el izquierdo carga 3 motores en un solo canal. Ese canal es el punto débil: un TB6612 da 1.2 A continuos (3.2 A de pico) y un L298N 2 A, y un TT en bloqueo puede pedir del orden de 1 A (no se cuenta con la ficha del motor usado). **Pon como driver 1 el de más corriente por canal.** Mide la corriente real en serie antes de confiar. Añade un condensador de 470–1000 µF cerca de cada driver. El servo del radar va con su propio regulador de 5 V.

## 3. Arquitectura sin bloqueos

- Cada ciclo del loop hace como máximo **un** ping bloqueante (≤ 12 ms con `MAX_CM = 200`). Antes eran 2×15 ms + 4 ms seguidos.
- No uso `ping_timer` de NewPing: dispara una interrupción cada 24 µs y alteraría el SoftwareSerial a 9600.
- Radar: máquina de estados con `millis()`; un paso del servo cada 70 ms, un ping por paso. Con el radar apagado el servo se hace `detach()` (sin pulsos, sin consumo ni jitter).
- DHT11: lectura en dos fases, sin el `delay(20)`. La captura (~5 ms, interrupciones apagadas) solo ocurre cuando la NICLA no está transmitiendo.
- I2C con timeout (`Wire.setWireTimeout`) para que un BMP280 desconectado no cuelgue nada. Watchdog de 2 s (`USE_WDT`; pon 0 si tu UNO es clon con bootloader viejo).
- **Deadman**: si el UNO no recibe ningún byte de la NICLA en 1.5 s y hay motores o recolección encendidos, los apaga. El poll de telemetría (cada 0.5 s) hace de latido. Por USB, cada comando dura 1.5 s.
- **Reintentos**: si el ACK del último comando de movimiento, recolección o radar no llega en 300 ms, la NICLA lo reenvía (máx. 3). Así un STOP que caiga en la ventana en que el DHT11 deja sordo al UART no se pierde.
- Al perder la HMI, la NICLA manda STOP, recolección apagada y radar apagado.

## 4. Protocolo nuevo

NICLA → UNO: `F B L R S` · `C` recolectar / `E` reversa / `O` off · `A1..A6` · `Z1` / `Z0` radar · `?` telemetría · `P` puntos de radar.
UNO → NICLA: `K,<tipo>,<código>` (ACK) · `T,DL,DR,DF,MOV,COL,RXBAD,OVF,LINE,TEXT10,HUM,TCAB10,PRES10,RAD,ENVST` · `R,ang:cm,...` (hasta 4 puntos).
HMI → NICLA: `CMD,x` · `AUD,1..6` · `COL,C|E|O` · `RAD,1|0`.
NICLA → HMI: `TEL;...` (ahora con `TEXT HUM TCAB PRESS COL RADAR PRAW PIDX FMEAN VMS ENVST TAGE`) y `RAD;PTS=ang:cm,...`.

## 5. Radar
Botón **RADAR** en la HMI. Al activarlo suena el audio 6 y el servo barre de 15° a 165° (paso de 6°, un barrido completo en ~2 s). Se dibuja el semicírculo con anillos a 50/100/150/200 cm, el contorno del espacio libre y los puntos de eco; lo viejo se desvanece en 8 s. Si el dibujo sale invertido izquierda/derecha, cambia `RADAR_MIRROR` en la HMI.

## 6. Tiempo de misión
Contador en la HMI (columna izquierda): arranca la primera vez que la HMI se conecta al AP y se detiene solo con **FIN MISION**. Si el enlace se cae sigue contando. Después de terminar, el botón pasa a **NUEVA MISION**.

## 7. Temperatura, humedad, presión
DHT11 (D2) → temperatura y humedad **externas**. BMP280 (I2C, 3.3 V) → **presión** y temperatura de **cabina**. El BMP280 usa la dirección **0x76** (la que funcionó en tu prueba con `bmp.begin(0x76)`); el sketch ya no intenta 0x77 y deja la configuración por defecto de la librería, igual que en la prueba. Esa prueba se hizo con `Wire.begin(21, 22)`, que es de ESP32; en el UNO el I2C es fijo: SDA → A4, SCL → A5, VCC → 3.3 V, GND común. Panel ENVIRONMENT + gráfica. El DHT11 mide de a 1 °C y ±5 % HR; el BMP280 cerca de motores o reguladores marca de más. Librerías a instalar: NewPing, Adafruit BMP280 (con Adafruit Unified Sensor y BusIO), DFRobotDFPlayerMini. El DHT11 no usa librería.

## 8. CO₂: no se puede calcular con estos sensores
Con temperatura, humedad y presión no hay ningún filtro ni fórmula que dé el CO₂: no hay relación física. Cualquier número que mostrara sería inventado, así que la HMI muestra **"CO2: SIN SENSOR"** y la telemetría lleva `CO2=-1`. Para medirlo de verdad: **SCD40/SCD41** (CO₂ real, I2C, comparte A4/A5 con el BMP280, sin gastar pines) o un MH-Z19B (UART, ocuparía pines que no quedan). Un MQ-135 es analógico, cruzado con otros gases y poco fiable para CO₂. Si necesitas un valor para una demo, puedo hacer un modelo de balance de masa de una cabina sellada con ocupantes, marcado como SIMULADO.

## 9. Modelo de personas: hallazgos

**Bug confirmado en el código (V3):** para elegir la clase "persona" se buscaba el texto `"person"` dentro de las etiquetas, pero `"no_person"` también lo contiene. Con etiquetas `['no_person','person']` el código tomaba la probabilidad de **no persona** como si fuera persona: resultado invertido, y por eso "no reconoce a las personas". Ahora se busca coincidencia exacta y se descartan las etiquetas negativas; además las salidas se normalizan a probabilidades (por si el modelo entrega logits o enteros).

**Pendiente de verificar con la cámara y el modelo:** lo demás son hipótesis que se pueden comprobar en la HMI (línea `VISION raw[...] idx luz ms`) y en la consola de OpenMV:
- `raw` con la persona de frente debe dar la clase persona alta. Si sale alta la otra, revisa `MODEL LABELS` en la consola.
- `luz` < 25 es imagen oscura y > 230 es saturada. Interior con poca luz degrada mucho este modelo.
- Imagen de cabeza o espejada según cómo montaste la Nicla: ajusta `CAMERA_VFLIP` / `CAMERA_HMIRROR`. Un modelo de personas no reconoce a alguien de cabeza.
- Es un modelo diminuto (~300 KB, 96×96 en grises) hecho para "hay/no hay persona" con la persona ocupando buena parte del cuadro. A 3 m o más, o de cuerpo entero lejos, falla. Prueba a 1–2 m, torso visible.
- Movimiento del rover: con motores en marcha hay borrosidad y vibración.
- Umbral actual 0.60 con 2 lecturas seguidas (~1 s).

## 10. Orden de prueba
1. Un solo ultrasónico con 1 pin + 1 kΩ, en un sketch aparte.
2. UNO con motores levantados: F/B/L/R/S, luego C/E/O; suelta y confirma el STOP.
3. DFPlayer 1–6 por USB.
4. BMP280 y DHT11 (mira `[BMP] OK` y `T,...` en el monitor).
5. Radar con el servo sin carga.
6. NICLA + HMI completo; revisa `VISION raw` con una persona a 1.5 m.

## 11. Si en la HMI no aparecen los sensores de ambiente

Que los audios funcionen no prueba nada sobre la telemetría de ambiente: los audios sirven con cualquier versión, pero los datos nuevos solo llegan si **los tres archivos son V4** (UNO, NICLA y HMI). Una NICLA o una HMI vieja descartan los campos nuevos y se ve "--".

Dónde mirar:

| Qué ves | Qué significa | Qué revisar |
|---|---|---|
| HMI: `UNO SIN RESPUESTA POR UART` | No llega ninguna telemetría del UNO a la NICLA (tampoco distancias ni ACK) | Cable D11 (UNO TX) → RX de la NICLA, GND común, NICLA con el V4 cargado |
| HMI: `BMP280: NO INICIA` y USB `[BMP] no responde en 0x76` | El UNO no ve el BMP280 | SDA → A4, SCL → A5, VCC 3.3 V, GND. Un escáner I2C debe mostrar 0x76 |
| HMI: `BMP280: SIN LECTURA` | Inició pero las lecturas fallan | Cableado intermitente, alimentación |
| HMI: `DHT11: SIN LECTURA` y USB `pulsos=0` | El DHT11 no responde | DATA en D2, VCC y GND, resistencia de 4.7–10 kΩ de DATA a 5 V si es el sensor suelto (los módulos de 3 pines ya la traen) |
| `DHT11: SIN LECTURA` con `pulsos` entre 1 y 39 | Responde a medias | Cable largo o ruido (cerca de motores o del servo) |
| `pulsos=40` o `41` y `fail` sube | Llega la trama pero falla el checksum o los tiempos | Avísame y mando el valor |
| HMI: `BMP280: OK | DHT11: OK` pero `--` en los valores | No debería pasar | Revisa que la HMI sea la V4 |

Por USB (115200) el UNO imprime cada 2 s una línea así:
`[ENV] DHT ok=3 fail=0 pulsos=40 T10=310 HR=65 | BMP init-ok fail=0 T10=305 P10=10132`
(`T10` es la temperatura x10, `P10` la presión en hPa x10). Si una lectura nueva deja de llegar, el dato se invalida a los 15 s (DHT11) o 5 s (BMP280) y la HMI muestra "--" en lugar de dejar un valor viejo.
