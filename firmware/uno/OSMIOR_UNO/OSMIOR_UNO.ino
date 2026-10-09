#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <ctype.h>
#include <string.h>

// OSMIOR V3 - UNO
// Motores intactos en D3-D9. UART NICLA D10 RX / D11 TX.
// DFPlayer TX-only: D12 -> RX del DFPlayer; no conectar su TX.
// HC-SR04 izq A0/A1, der A2/A3.
//
// Protocolo NICLA -> UNO (minimo):
//   F/B/L/R/S = movimiento inmediato, un byte
//   A1..A5    = audio, dos bytes
//   ?         = solicitud de telemetria
// UNO -> NICLA:
//   K,M,F\n / K,A,5\n = ACK
//   T,DL,DR,DF,STATE,RX_BAD,OVF\n = telemetria compacta

SoftwareSerial niclaSerial(10, 11);     // RX, TX
SoftwareSerial dfPlayerSerial(13, 12);  // RX dummy, TX real en D12
DFRobotDFPlayerMini dfPlayer;
bool dfPlayerOK = false;

// TB6612 - NO CAMBIAR
const uint8_t PWM_L = 3;
const uint8_t IN1_L = 4;
const uint8_t IN2_L = 5;
const uint8_t PWM_R = 6;
const uint8_t IN1_R = 7;
const uint8_t IN2_R = 8;
const uint8_t STBY = 9;
const uint8_t VELOCIDAD = 220;

// HC-SR04
const uint8_t TRIG_L = A0;
const uint8_t ECHO_L = A1;
const uint8_t TRIG_R = A2;
const uint8_t ECHO_R = A3;
const unsigned long ECHO_TIMEOUT_US = 15000UL;
const unsigned long SENSOR_PERIOD_MS = 100UL;

// Buffers fijos: evita String y fragmentacion del heap del UNO.
char legacyBuffer[24];
uint8_t legacyIndex = 0;
char usbBuffer[8];
uint8_t usbIndex = 0;

// Estado
char currentMotion = 'S';
float distL = -1.0f;
float distR = -1.0f;
bool audioPrefixSeen = false;
unsigned long lastSensorMs = 0;
unsigned long rxGoodCommands = 0;
unsigned long rxBadBytes = 0;
unsigned long rxLineOverflows = 0;
unsigned long softwareRxOverflows = 0;

void processCommand(char cmd, const char *origin);
void processNiclaByte(uint8_t raw);
void processLegacyLine(char *line);
void readNiclaSerial();
void readUsbSerial();
void sendNicla(const char *line);
void sendAckCommand(char cmd);
void sendAckAudio(uint8_t track);
void sendTelemetry();
void reproducirAudio(uint8_t track);
float medirSensor(uint8_t trigPin, uint8_t echoPin);
void updateSensors();
void adelante();
void atras();
void izquierda();
void derecha();
void detener();

void setup() {
  Serial.begin(115200);
  niclaSerial.begin(9600);
  dfPlayerSerial.begin(9600);

  pinMode(PWM_L, OUTPUT); pinMode(IN1_L, OUTPUT); pinMode(IN2_L, OUTPUT);
  pinMode(PWM_R, OUTPUT); pinMode(IN1_R, OUTPUT); pinMode(IN2_R, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  detener();

  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
  digitalWrite(TRIG_L, LOW);
  digitalWrite(TRIG_R, LOW);

  Serial.println(F("OSMIOR UNO V3 - comms optimized"));
  Serial.println(F("Motors D3-D9 unchanged | NICLA D10/D11"));
  Serial.println(F("DFPlayer D12 TX-only | HC-SR04 A0/A1 A2/A3"));

  delay(350);
  if (dfPlayer.begin(dfPlayerSerial, false, true)) {
    dfPlayerOK = true;
    dfPlayer.volume(25);
    delay(200);
    dfPlayer.play(1); // track 001: startup
    Serial.println(F("[DF] init OK; track 1 requested"));
  } else {
    dfPlayerOK = false;
    Serial.println(F("[DF] init failed"));
  }

  // Restore NICLA as the only active SoftwareSerial receiver.
  niclaSerial.listen();
  detener();
  sendNicla(dfPlayerOK ? "READY,UNO,1,DF,1" : "READY,UNO,1,DF,0");
}

void loop() {
  readNiclaSerial();
  readUsbSerial();

  const unsigned long now = millis();
  if (now - lastSensorMs >= SENSOR_PERIOD_MS) {
    lastSensorMs = now;
    updateSensors();
  }
}

// -------------------- NICLA RX: byte parser --------------------
void readNiclaSerial() {
  // Do not call listen() continuously: only restore it after DFPlayer TX.
  while (niclaSerial.available() > 0) {
    processNiclaByte((uint8_t)niclaSerial.read());
  }

  if (niclaSerial.overflow()) {
    softwareRxOverflows++;
    Serial.println(F("[UART] SoftwareSerial RX overflow"));
  }
}

void processNiclaByte(uint8_t raw) {
  if (raw == '\r' || raw == '\n') {
    if (legacyIndex > 0) {
      legacyBuffer[legacyIndex] = '\0';
      processLegacyLine(legacyBuffer);
      legacyIndex = 0;
    }
    audioPrefixSeen = false;
    return;
  }

  // Discard control/non-ASCII bytes. They must never enter string parsing.
  if (raw < 0x20 || raw > 0x7E) {
    rxBadBytes++;
    return;
  }

  const char c = (char)toupper(raw);

  // Motion commands are executed immediately: no need to wait for LF.
  if (c == 'F' || c == 'B' || c == 'L' || c == 'R' || c == 'S') {
    legacyIndex = 0;
    audioPrefixSeen = false;
    processCommand(c, "NICLA");
    return;
  }

  // Audio consists of exactly A followed by digit 1..5.
  if (audioPrefixSeen) {
    audioPrefixSeen = false;
    if (c >= '1' && c <= '5') {
      legacyIndex = 0;
      reproducirAudio((uint8_t)(c - '0'));
      return;
    }
  }

  if (c == 'A') {
    audioPrefixSeen = true;
    legacyIndex = 0;
    return;
  }

  // '?' is a scheduled telemetry poll from the NICLA. No unsolicited
  // telemetry is sent, keeping the UART half-duplex traffic coordinated.
  if (c == '?') {
    legacyIndex = 0;
    sendTelemetry();
    return;
  }

  // Optional old textual diagnostics. A motion letter within a legacy
  // CMD,F line is already executed above when the F byte arrives.
  if (legacyIndex < sizeof(legacyBuffer) - 1) {
    legacyBuffer[legacyIndex++] = c;
  } else {
    legacyIndex = 0;
    rxLineOverflows++;
  }
}

void processLegacyLine(char *line) {
  if (strcmp(line, "PING") == 0) {
    sendNicla("PONG");
    return;
  }
  if (strcmp(line, "STOP") == 0) {
    processCommand('S', "NICLA");
    return;
  }
  // Do not flood the monitor with random noise; count malformed frames.
  rxLineOverflows++;
}

void readUsbSerial() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (usbIndex == 1) processCommand((char)toupper(usbBuffer[0]), "USB");
      usbIndex = 0;
      continue;
    }
    if (usbIndex < sizeof(usbBuffer) - 1) usbBuffer[usbIndex++] = c;
    else usbIndex = 0;
  }
}

void processCommand(char cmd, const char *origin) {
  cmd = (char)toupper((unsigned char)cmd);
  switch (cmd) {
    case 'F': adelante(); currentMotion = 'F'; break;
    case 'B': atras(); currentMotion = 'B'; break;
    case 'L': izquierda(); currentMotion = 'L'; break;
    case 'R': derecha(); currentMotion = 'R'; break;
    case 'S': detener(); currentMotion = 'S'; break;
    default: return;
  }

  rxGoodCommands++;
  Serial.print(F("[UNO] NICLA -> "));
  Serial.println(cmd);
  sendAckCommand(cmd);
}

// -------------------- MOTORES: LOGICA ORIGINAL INTACTA --------------------
void adelante() {
  digitalWrite(IN1_L, HIGH); digitalWrite(IN2_L, LOW); analogWrite(PWM_L, VELOCIDAD);
  digitalWrite(IN1_R, HIGH); digitalWrite(IN2_R, LOW); analogWrite(PWM_R, VELOCIDAD);
}

void atras() {
  digitalWrite(IN1_L, LOW); digitalWrite(IN2_L, HIGH); analogWrite(PWM_L, VELOCIDAD);
  digitalWrite(IN1_R, LOW); digitalWrite(IN2_R, HIGH); analogWrite(PWM_R, VELOCIDAD);
}

void izquierda() {
  digitalWrite(IN1_L, LOW); digitalWrite(IN2_L, HIGH); analogWrite(PWM_L, VELOCIDAD);
  digitalWrite(IN1_R, HIGH); digitalWrite(IN2_R, LOW); analogWrite(PWM_R, VELOCIDAD);
}

void derecha() {
  digitalWrite(IN1_L, HIGH); digitalWrite(IN2_L, LOW); analogWrite(PWM_L, VELOCIDAD);
  digitalWrite(IN1_R, LOW); digitalWrite(IN2_R, HIGH); analogWrite(PWM_R, VELOCIDAD);
}

void detener() {
  digitalWrite(IN1_L, LOW); digitalWrite(IN2_L, LOW); analogWrite(PWM_L, 0);
  digitalWrite(IN1_R, LOW); digitalWrite(IN2_R, LOW); analogWrite(PWM_R, 0);
}

// -------------------- DFPLAYER --------------------
void reproducirAudio(uint8_t track) {
  if (track < 1 || track > 5) return;

  if (!dfPlayerOK) {
    Serial.print(F("[DF] unavailable track=")); Serial.println(track);
    sendNicla("K,A,0");
    niclaSerial.listen();
    return;
  }

  dfPlayer.play(track);
  // SoftwareSerial TX to DFPlayer may temporarily own the listener.
  niclaSerial.listen();

  Serial.print(F("[DF] track=")); Serial.println(track);
  sendAckAudio(track);
}

// -------------------- HC-SR04 --------------------
float medirSensor(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  const unsigned long duration = pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return -1.0f;
  return (duration * 0.0343f) / 2.0f;
}

void updateSensors() {
  distL = medirSensor(TRIG_L, ECHO_L);
  delay(4);
  distR = medirSensor(TRIG_R, ECHO_R);
}

// -------------------- NICLA TX --------------------
void sendNicla(const char *line) {
  niclaSerial.print(line);
  niclaSerial.write('\n');
  niclaSerial.listen();
}

void sendAckCommand(char cmd) {
  niclaSerial.print(F("K,M,"));
  niclaSerial.write(cmd);
  niclaSerial.write('\n');
  niclaSerial.listen();
}

void sendAckAudio(uint8_t track) {
  niclaSerial.print(F("K,A,"));
  niclaSerial.print(track);
  niclaSerial.write('\n');
  niclaSerial.listen();
}

void sendTelemetry() {
  // Compact integer-centimeter values reduce the SoftwareSerial TX window.
  const int leftCm = (distL < 0.0f) ? -1 : (int)(distL + 0.5f);
  const int rightCm = (distR < 0.0f) ? -1 : (int)(distR + 0.5f);

  niclaSerial.print(F("T,"));
  niclaSerial.print(leftCm); niclaSerial.write(',');
  niclaSerial.print(rightCm); niclaSerial.write(',');
  niclaSerial.print(dfPlayerOK ? 1 : 0); niclaSerial.write(',');
  niclaSerial.write(currentMotion); niclaSerial.write(',');
  niclaSerial.print((unsigned int)(rxBadBytes > 65535UL ? 65535UL : rxBadBytes)); niclaSerial.write(',');
  niclaSerial.print((unsigned int)(softwareRxOverflows > 65535UL ? 65535UL : softwareRxOverflows)); niclaSerial.write(',');
  niclaSerial.println((unsigned int)(rxLineOverflows > 65535UL ? 65535UL : rxLineOverflows));
  niclaSerial.listen();

  Serial.print(F("[UART] poll -> T; bad="));
  Serial.print(rxBadBytes);
  Serial.print(F(" ovf="));
  Serial.print(softwareRxOverflows);
  Serial.print(F(" line="));
  Serial.println(rxLineOverflows);
}
