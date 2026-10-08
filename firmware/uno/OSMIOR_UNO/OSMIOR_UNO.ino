#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>

// ============================================================
// OSMIOR - ARDUINO UNO
// ------------------------------------------------------------
// Motores: se conserva la logica original y los pines D3-D9.
// UART NICLA: D10 RX / D11 TX @ 9600.
// DFPlayer: D12 TX -> RX del DFPlayer. D13 queda sin conectar.
// HC-SR04 izquierda: TRIG A0 / ECHO A1.
// HC-SR04 derecha : TRIG A2 / ECHO A3.
//
// NOTA IMPORTANTE DE ESTRUCTURA:
// Arduino SoftwareSerial solo permite UNA instancia escuchando RX
// a la vez. El DFPlayer se usa solo para TX, por lo que despues
// de inicializarlo se fuerza niclaSerial.listen().
// ============================================================

// ---------------- UART NICLA ----------------
SoftwareSerial niclaSerial(10, 11);  // RX, TX

// ---------------- DFPlayer ----------------
// RX dummy en D13: NO conectar el TX del DFPlayer.
// TX real del UNO en D12 -> RX del DFPlayer.
SoftwareSerial dfPlayerSerial(13, 12); // RX dummy, TX
DFRobotDFPlayerMini dfPlayer;
bool dfPlayerOK = false;

// ---------------- TB6612 izquierdo ----------------
const uint8_t PWM_L  = 3;
const uint8_t IN1_L  = 4;
const uint8_t IN2_L  = 5;

// ---------------- TB6612 derecho ----------------
const uint8_t PWM_R  = 6;
const uint8_t IN1_R  = 7;
const uint8_t IN2_R  = 8;

// ---------------- Standby ----------------
const uint8_t STBY = 9;

// ---------------- Velocidad fija original ----------------
const uint8_t VELOCIDAD = 220;

// ---------------- HC-SR04 ----------------
const uint8_t TRIG_L = A0;
const uint8_t ECHO_L = A1;
const uint8_t TRIG_R = A2;
const uint8_t ECHO_R = A3;

const unsigned long ECHO_TIMEOUT_US = 25000UL;
const unsigned long SENSOR_PERIOD_MS = 100UL;
const unsigned long TELEMETRY_PERIOD_MS = 200UL;

// ---------------- Buffers fijos ----------------
// Evitamos String para no fragmentar el heap del ATmega328P.
char niclaBuffer[48];
uint8_t niclaIndex = 0;

char usbBuffer[16];
uint8_t usbIndex = 0;

// ---------------- Estado ----------------
char currentMotion = 'S';
float distL = -1.0f;
float distR = -1.0f;

unsigned long lastSensorMs = 0;
unsigned long lastTelemetryMs = 0;

// ============================================================
// PROTOTIPOS
// ============================================================

void processNiclaLine(char *line);
void processCommand(char cmd, const char *origin);
void sendNicla(const char *line);
void sendAckCommand(char cmd);
void sendAckAudio(uint8_t track);

void adelante();
void atras();
void izquierda();
void derecha();
void detener();

float medirSensor(uint8_t trigPin, uint8_t echoPin);
void updateSensors();
void sendTelemetry();
void reproducirAudio(uint8_t track);

void readNiclaSerial();
void readUsbSerial();

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);

  // IMPORTANTE: cada begin() de SoftwareSerial puede cambiar
  // cual instancia queda escuchando. Por eso al final de setup()
  // hacemos niclaSerial.listen().
  niclaSerial.begin(9600);
  dfPlayerSerial.begin(9600);

  // Motores
  pinMode(PWM_L, OUTPUT);
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);

  pinMode(PWM_R, OUTPUT);
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  detener();

  // HC-SR04
  pinMode(TRIG_L, OUTPUT);
  pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT);
  pinMode(ECHO_R, INPUT);
  digitalWrite(TRIG_L, LOW);
  digitalWrite(TRIG_R, LOW);

  Serial.println(F("=========================================="));
  Serial.println(F("OSMIOR UNO - INIT"));
  Serial.println(F("Motors: D3-D9"));
  Serial.println(F("NICLA UART: D10/D11"));
  Serial.println(F("DFPlayer TX: D12 -> RX"));
  Serial.println(F("HC-SR04: A0/A1 + A2/A3"));
  Serial.println(F("=========================================="));

  delay(500);

  // El TX del DFPlayer no esta conectado, asi que no pedimos ACK.
  if (dfPlayer.begin(dfPlayerSerial, false, true)) {
    dfPlayerOK = true;
    dfPlayer.volume(25);
    delay(250);
    reproducirAudio(1); // AUDIO 001: inicio
  } else {
    dfPlayerOK = false;
    Serial.println(F("[DF] init warning"));
  }

  // CRITICO: dejar al NICLA como unica instancia RX activa.
  niclaSerial.listen();

  detener();
  sendNicla(dfPlayerOK ? "READY,UNO,1,DF,1" : "READY,UNO,1,DF,0");
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  readNiclaSerial();
  readUsbSerial();

  const unsigned long now = millis();

  if (now - lastSensorMs >= SENSOR_PERIOD_MS) {
    lastSensorMs = now;
    updateSensors();
  }

  if (now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = now;
    sendTelemetry();
  }
}

// ============================================================
// SERIAL RX
// ============================================================

void readNiclaSerial() {
  // Por si alguna operacion del DFPlayer lo cambio, recuperamos
  // explicitamente el receptor del NICLA antes de leer.
  niclaSerial.listen();

  while (niclaSerial.available() > 0) {
    const char c = (char)niclaSerial.read();

    if (c == '\r') continue;

    if (c == '\n') {
      niclaBuffer[niclaIndex] = '\0';

      if (niclaIndex > 0) {
        processNiclaLine(niclaBuffer);
      }

      niclaIndex = 0;
      continue;
    }

    if (niclaIndex < sizeof(niclaBuffer) - 1) {
      niclaBuffer[niclaIndex++] = c;
    } else {
      // Descartar linea corrupta/overflow.
      niclaIndex = 0;
    }
  }
}

void readUsbSerial() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();

    if (c == '\r' || c == '\n') {
      if (usbIndex == 1) {
        processCommand((char)usbBuffer[0], "USB");
      }
      usbIndex = 0;
      continue;
    }

    if (usbIndex < sizeof(usbBuffer) - 1) {
      usbBuffer[usbIndex++] = c;
    } else {
      usbIndex = 0;
    }
  }
}

// ============================================================
// PROTOCOLO NICLA -> UNO
// Acepta el protocolo nuevo y comandos legacy simples.
// ============================================================

void processNiclaLine(char *line) {
  if (strncmp(line, "CMD,", 4) == 0 && line[5] == '\0') {
    processCommand(line[4], "NICLA");
    return;
  }

  // Compatibilidad: F/B/L/R/S
  if (line[0] != '\0' && line[1] == '\0') {
    processCommand(line[0], "NICLA");
    return;
  }

  if (strncmp(line, "AUD,", 4) == 0) {
    const int track = atoi(line + 4);
    if (track >= 1 && track <= 5) {
      reproducirAudio((uint8_t)track);
    }
    return;
  }

  // Compatibilidad: AUD:3
  if (strncmp(line, "AUD:", 4) == 0) {
    const int track = atoi(line + 4);
    if (track >= 1 && track <= 5) {
      reproducirAudio((uint8_t)track);
    }
    return;
  }

  if (strcmp(line, "PING") == 0) {
    sendNicla("PONG");
    return;
  }

  if (strcmp(line, "STOP") == 0) {
    processCommand('S', "NICLA");
    return;
  }

  Serial.print(F("[UNO] Unknown line: "));
  Serial.println(line);
  sendNicla("ERR,UNKNOWN");
}

void processCommand(char cmd, const char *origin) {
  cmd = (char)toupper((unsigned char)cmd);

  switch (cmd) {
    case 'F':
      adelante();
      currentMotion = 'F';
      break;

    case 'B':
      atras();
      currentMotion = 'B';
      break;

    case 'L':
      izquierda();
      currentMotion = 'L';
      break;

    case 'R':
      derecha();
      currentMotion = 'R';
      break;

    case 'S':
      detener();
      currentMotion = 'S';
      break;

    default:
      Serial.print(F("[UNO] Bad command: "));
      Serial.println(cmd);
      sendNicla("ERR,BAD_CMD");
      return;
  }

  Serial.print(F("[UNO] "));
  Serial.print(origin);
  Serial.print(F(" -> "));
  Serial.println(cmd);

  sendAckCommand(cmd);
}

// ============================================================
// MOTORES - NO CAMBIAR LOGICA
// ============================================================

void adelante() {
  digitalWrite(IN1_L, HIGH);
  digitalWrite(IN2_L, LOW);
  analogWrite(PWM_L, VELOCIDAD);

  digitalWrite(IN1_R, HIGH);
  digitalWrite(IN2_R, LOW);
  analogWrite(PWM_R, VELOCIDAD);
}

void atras() {
  digitalWrite(IN1_L, LOW);
  digitalWrite(IN2_L, HIGH);
  analogWrite(PWM_L, VELOCIDAD);

  digitalWrite(IN1_R, LOW);
  digitalWrite(IN2_R, HIGH);
  analogWrite(PWM_R, VELOCIDAD);
}

void izquierda() {
  digitalWrite(IN1_L, LOW);
  digitalWrite(IN2_L, HIGH);
  analogWrite(PWM_L, VELOCIDAD);

  digitalWrite(IN1_R, HIGH);
  digitalWrite(IN2_R, LOW);
  analogWrite(PWM_R, VELOCIDAD);
}

void derecha() {
  digitalWrite(IN1_L, HIGH);
  digitalWrite(IN2_L, LOW);
  analogWrite(PWM_L, VELOCIDAD);

  digitalWrite(IN1_R, LOW);
  digitalWrite(IN2_R, HIGH);
  analogWrite(PWM_R, VELOCIDAD);
}

void detener() {
  digitalWrite(IN1_L, LOW);
  digitalWrite(IN2_L, LOW);
  analogWrite(PWM_L, 0);

  digitalWrite(IN1_R, LOW);
  digitalWrite(IN2_R, LOW);
  analogWrite(PWM_R, 0);
}

// ============================================================
// DFPLAYER
// ============================================================

void reproducirAudio(uint8_t track) {
  if (!dfPlayerOK) {
    Serial.print(F("[DF] unavailable track="));
    Serial.println(track);
    sendNicla("ACK,AUD,0");
    // MUY IMPORTANTE: recuperar RX del NICLA igualmente.
    niclaSerial.listen();
    return;
  }

  dfPlayer.play(track);

  Serial.print(F("[DF] track="));
  Serial.println(track);

  sendAckAudio(track);

  // dfPlayer usa otra instancia SoftwareSerial.
  // Recuperar escucha del NICLA al terminar.
  niclaSerial.listen();
}

// ============================================================
// HC-SR04
// ============================================================

float medirSensor(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  const unsigned long duration = pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);

  if (duration == 0) {
    return -1.0f;
  }

  return (duration * 0.0343f) / 2.0f;
}

void updateSensors() {
  distL = medirSensor(TRIG_L, ECHO_L);

  // Evita que el mismo pulso acustico se interprete por el segundo sensor.
  delay(6);

  distR = medirSensor(TRIG_R, ECHO_R);
}

// ============================================================
// TELEMETRIA UNO -> NICLA
// ============================================================

void sendTelemetry() {
  niclaSerial.print(F("TEL,DL="));
  niclaSerial.print(distL, 1);

  niclaSerial.print(F(",DR="));
  niclaSerial.print(distR, 1);

  niclaSerial.print(F(",DF="));
  niclaSerial.print(dfPlayerOK ? 1 : 0);

  niclaSerial.print(F(",PWM="));
  niclaSerial.print(VELOCIDAD);

  niclaSerial.print(F(",STATE="));
  niclaSerial.println(currentMotion);
}

void sendNicla(const char *line) {
  niclaSerial.println(line);
}

void sendAckCommand(char cmd) {
  niclaSerial.print(F("ACK,CMD,"));
  niclaSerial.println(cmd);
}

void sendAckAudio(uint8_t track) {
  niclaSerial.print(F("ACK,AUD,"));
  niclaSerial.println(track);
}
