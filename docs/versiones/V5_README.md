# OSMIOR V5 — HMI, Nicla Vision y Arduino UNO

## Cambios incluidos

- La pista 001 de inicialización solo se solicita al pulsar **INICIAR MISIÓN**. Encender el UNO o conectar la HMI no inicia la misión ni solicita esa pista.
- El contador empieza con el botón, no al conectar la HMI. **FIN MISIÓN** detiene el rover y congela el contador; **NUEVA MISIÓN** inicia una sesión nueva.
- La barra representa el uso de una misión de 50 minutos. Al alcanzar 35 minutos transcurridos (15 minutos restantes), la Nicla solicita una sola vez la pista 002 de batería/tiempo bajo.
- El desplazamiento IMU se reinicia al iniciar misión, solo se acumula con órdenes `F`/`B`, y se pone a cero la velocidad estimada al detener o girar. Incluye filtro pasa-bajos, compensación aproximada de gravedad a partir del pitch, estimación de sesgo mientras está detenido, zona muerta y fuga de velocidad para limitar deriva.
- Se muestra el desplazamiento firmado (`+` hacia adelante, `−` hacia atrás) como **estimación IMU**, no odometría exacta. Sin encoders el acelerómetro no permite conocer la velocidad constante; la doble integración puede derivar y subestimar distancias.
- Se elimina la función de recolección del HMI, la Nicla y el UNO.
- Se elimina el recolector y se conserva el control izquierda/derecha con las entradas A/B puenteadas en cada TB6612FNG, como en el cableado real confirmado.
- Se conservan el servo del radar en A0 y el HC-SR04 superior del radar en A3. El radar sigue siendo opcional y activa la pista 006.
- Los gauges de pitch y roll tienen arco de límites y una silueta simplificada del rover: perfil lateral para pitch y frontal para roll.

## Pinout UNO V5 — cableado corregido según el cableado real verificado

| Pin UNO | Función |
|---|---|
| D2 | DHT11 DATA (temperatura y humedad exterior) |
| D3 | PWM común del driver 1: PWMA y PWMB puenteados (tracción izquierda) |
| D4 | Dirección IN1 común del driver 1: AIN1 y BIN1 puenteados |
| D5 | Dirección IN2 común del driver 1: AIN2 y BIN2 puenteados |
| D6 | PWM común del driver 2: PWMA y PWMB puenteados (tracción derecha) |
| D7 | Dirección IN1 común del driver 2: AIN1 y BIN1 puenteados |
| D8 | Dirección IN2 común del driver 2: AIN2 y BIN2 puenteados |
| D9 | STBY compartido de ambos TB6612FNG |
| D10 | RX SoftwareSerial del UNO desde TX de la Nicla |
| D11 | TX SoftwareSerial del UNO hacia RX de la Nicla |
| D12 | TX SoftwareSerial hacia RX del DFPlayer, con resistencia serie de aproximadamente 1 kΩ |
| D13 | RX dummy del objeto SoftwareSerial del DFPlayer; no conectar cable de retorno |
| A0 | Señal del servo del radar |
| A1 / A2 | HC-SR04 izquierdo y derecho, modo de un solo pin mediante NewPing |
| A3 | HC-SR04 superior del radar, modo de un solo pin mediante NewPing |
| A4 / A5 | SDA / SCL del BMP280, dirección 0x76 |

### Cableado de los TB6612FNG

- **Driver 1 (lado izquierdo):** PWMA y PWMB unidos a D3; AIN1 y BIN1 unidos a D4; AIN2 y BIN2 unidos a D5.
- **Driver 2 (lado derecho):** PWMA y PWMB unidos a D6; AIN1 y BIN1 unidos a D7; AIN2 y BIN2 unidos a D8.
- **STBY de ambos drivers:** unidos a D9. El firmware configura D9 como salida HIGH.
- **VM:** alimentación de motor de 6 V desde el buck de motores. **VCC:** lógica de 5 V desde el buck de lógica. GND de ambos drivers, UNO, Nicla y fuentes unido en tierra común. No unir las salidas positivas de dos buck distintos entre sí.
- Las entradas lógicas A/B de cada driver están puenteadas para que ambos canales respondan a la misma dirección y PWM. **No puentear las salidas de potencia AO1/AO2 con BO1/BO2.** Mantén los motores distribuidos en sus canales de salida correspondientes.
- Los seis motores de tracción se agrupan en lado izquierdo y lado derecho: un TB6612FNG por lado, con sus dos canales A/B sincronizados. Si un canal alimenta dos motores en paralelo, comprobar la corriente conjunta, especialmente al arrancar o bloquearse.

### Comunicación y sensores

- Mantener Nicla TX → UNO D10 y UNO D11 → RX de la Nicla (GND común).
- DFPlayer: UNO D12 → resistencia serie ~1 kΩ → RX del DFPlayer; TX del DFPlayer sin conectar. La pista 001 solo se solicita al iniciar misión desde la HMI.
- Cada HC-SR04 usa una entrada del UNO para TRIG y ECHO unidos. La práctica con NewPing requiere revisar el módulo y usar aproximadamente 1 kΩ en serie en la línea ECHO para evitar contención.
- El radar conserva servo A0 y sensor superior A3; el DHT11 conserva D2 y el BMP280 I²C en A4/A5.

## Protocolo

- HMI → Nicla: `CMD,F/B/L/R/S`, `AUD,2..6`, `RAD,1/0`, `MISSION,START`, `MISSION,STOP`.
- Nicla → UNO: movimiento de un byte, audio `A1..A6`, radar `Z1/Z0`, consulta `?` y puntos radar `P`.
- UNO → Nicla: ACK `K,...`, telemetría `T,DL,DR,DF,MOV,RXBAD,OVF,LINE,TEXT10,HUM,TCAB10,PRES10,RAD,ENVST` y puntos `R,...`.

## Filtro de desplazamiento IMU

El cálculo compensa aproximadamente la gravedad con el pitch, suaviza la aceleración longitudinal, aprende sesgo mientras el rover está detenido, rechaza aceleraciones pequeñas y reinicia la velocidad cuando el rover para o gira. `DIST` es desplazamiento neto con signo y se reinicia al iniciar una misión.

El filtrado reduce ruido, pero no elimina la deriva ni la limitación fundamental de un acelerómetro sin referencia externa. Debe calibrarse `FORWARD_AXIS_SIGN` y comprobar el signo del eje en una prueba corta. Para distancia fiable, instalar encoders es la mejora recomendada.

## Prueba recomendada

1. Revisa continuidad de GND, entradas de dirección compartidas y STBY antes de conectar motores al suelo.
2. Con las ruedas levantadas, prueba F/B/L/R/S y confirma la parada de seguridad.
3. Confirma que el DFPlayer no reproduce la pista 001 al encender, pero que **INICIAR MISIÓN** la reproduce.
4. Prueba radar ON/OFF: el servo no debe barrer al estar desactivado; al activarlo debe sonar la pista 006.
5. Compara el desplazamiento IMU en una distancia corta medida con cinta. Prueba avanzar, retroceder, girar y detenerse por separado.
6. Verifica DHT11/BMP280 y telemetría de la HMI.

## Estado de validación

Los archivos Python pasaron la comprobación de sintaxis. El pinout UNO fue corregido para respetar Driver 1 en D3-D5, Driver 2 en D6-D8 y STBY compartido en D9. Todavía no se hizo una prueba visual de la HMI. El sketch de Arduino requiere compilación en Arduino IDE con NewPing, DFRobotDFPlayerMini y Adafruit BMP280, seguida de pruebas físicas; no debe considerarse validado en hardware todavía.
