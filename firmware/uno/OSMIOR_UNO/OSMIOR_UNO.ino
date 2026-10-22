#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <NewPing.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <avr/wdt.h>
#include <ctype.h>
#include <math.h>

// ============================================================
// OSMIOR V5 - UNO
// Distribucion segun el cableado real verificado.
// Driver 1 = traccion izquierda; Driver 2 = traccion derecha.
// En CADA driver se comparten las entradas logicas de los dos canales:
//   PWMA + PWMB unidos; AIN1 + BIN1 unidos; AIN2 + BIN2 unidos.
// Driver 1: PWM comun D3, IN1 comun D4, IN2 comun D5.
// Driver 2: PWM comun D6, IN1 comun D7, IN2 comun D8.
// STBY de ambos drivers a D9.
//   D2      DHT11 DATA
//   D3      PWM traccion izquierda (PWMA+PWMB Driver 1)
//   D4/D5   IN1/IN2 izquierda (AIN1+BIN1 / AIN2+BIN2 Driver 1)
//   D6      PWM traccion derecha (PWMA+PWMB Driver 2)
//   D7/D8   IN1/IN2 derecha (AIN1+BIN1 / AIN2+BIN2 Driver 2)
//   D9      STBY compartido de ambos TB6612FNG
//   D10/D11 UART por SoftwareSerial: RX / TX del UNO hacia Nicla
//   D12     TX SoftwareSerial hacia RX del DFPlayer (resistencia ~1 kOhm)
//   D13     RX dummy del SoftwareSerial del DFPlayer; sin cable externo
//   A0      señal del servo radar
//   A1/A2   HC-SR04 izquierdo/derecho con NewPing (TRIG+ECHO en un pin)
//   A3      HC-SR04 superior del radar con NewPing (TRIG+ECHO en un pin)
//   A4/A5   BMP280 SDA/SCL (direccion 0x76)
//
// Importante: las entradas de ambos puentes H en cada driver reciben la misma
// señal, pero NO se deben unir entre si las salidas de potencia A y B.
// NICLA -> UNO: F B L R S, A1..A6, Z1/Z0, ?, P
// UNO -> NICLA: ACK, telemetria T y puntos radar R. Sin recolector.
// ============================================================

#define USE_WDT 1          // pon 0 si tu UNO es clon con bootloader viejo

// ---------------- pines ----------------
const uint8_t PIN_DHT      = 2;
const uint8_t PWM_L        = 3;
const uint8_t PWM_R        = 6;
const uint8_t IN1_L        = 4;
const uint8_t IN2_L        = 5;
const uint8_t IN1_R        = 7;
const uint8_t IN2_R        = 8;
const uint8_t PIN_STBY     = 9;
const uint8_t PIN_SERVO    = A0;
const uint8_t PIN_US_L     = A1;
const uint8_t PIN_US_R     = A2;
const uint8_t PIN_US_RADAR = A3;
const uint8_t VELOCIDAD    = 180;

// UNO recibe de Nicla por D10 y transmite a Nicla por D11.
// DFPlayer es de solo envio: UNO D12 -> RX del DFPlayer; TX del DFPlayer sin conectar.
SoftwareSerial niclaSerial(10, 11);     // RX, TX
SoftwareSerial dfPlayerSerial(13, 12);  // RX dummy, TX real D12
DFRobotDFPlayerMini dfPlayer;
bool dfPlayerOK = false;

// ---------------- ultrasónicos (NewPing, 1 pin cada uno) ----------------
const uint16_t MAX_CM = 200;
NewPing sonarL(PIN_US_L, PIN_US_L, MAX_CM);
NewPing sonarR(PIN_US_R, PIN_US_R, MAX_CM);
NewPing sonarRadar(PIN_US_RADAR, PIN_US_RADAR, MAX_CM);
const unsigned long SONAR_PERIOD_MS = 50UL;   // alterna L/R: cada uno a 10 Hz
const unsigned long PING_GUARD_MS   = 30UL;   // separación mínima entre pings

// ---------------- radar ----------------
Servo radarServo;
const uint8_t RADAR_MIN_DEG = 15;
const uint8_t RADAR_MAX_DEG = 165;
const uint8_t RADAR_STEP    = 6;
const unsigned long RADAR_STEP_MS = 70UL;     // asentamiento del servo + ping
struct RadarPt { uint8_t ang; int16_t cm; };
const uint8_t RADAR_BUF = 16;
RadarPt radarBuf[RADAR_BUF];
uint8_t rHead = 0, rTail = 0, rCount = 0;
bool radarActive = false;
uint8_t radarAngle = RADAR_MIN_DEG;
int8_t radarDir = 1;
unsigned long radarNextMs = 0;

// ---------------- ambiente ----------------
// Nuestro BMP280 (clon) responde en 0x76 y se probó con begin(0x76) y la
// configuración por defecto de la librería Adafruit.
const uint8_t BMP_ADDR = 0x76;
Adafruit_BMP280 bmp;
bool bmpOK = false;
const int16_t INVALID_T = -999;
int16_t tExt10 = INVALID_T;   // DHT11, °C x10
int16_t rhExt  = -1;          // DHT11, %HR
int16_t tCab10 = INVALID_T;   // BMP280, °C x10
int16_t press10 = -1;         // BMP280, hPa x10
const unsigned long BMP_PERIOD_MS = 1000UL;
const unsigned long DHT_PERIOD_MS = 3000UL;
unsigned long lastBmpMs = 0, lastDhtMs = 0, dhtStartMs = 0;
bool dhtWaiting = false;
bool dhtLastFailed = false;
unsigned long dhtLastOkMs = 0, bmpLastOkMs = 0, lastEnvDbgMs = 0;
unsigned int dhtOkCount = 0, dhtFailCount = 0, bmpFailCount = 0;
uint8_t dhtLastN = 0;                       // pulsos capturados en el último intento
const unsigned long DHT_RETRY_MS = 1500UL;  // reintento rápido si la lectura falla
const unsigned long ENV_STALE_MS = 15000UL; // sin lecturas nuevas => dato invalido ("--")
#define DEBUG_ENV 1                         // imprime [ENV] por USB cada 2 s

// ---------------- estado ----------------
char currentMotion = 'S';
int16_t distL = -1, distR = -1;
char prefixByte = 0;
unsigned long lastSonarMs = 0, lastPingEndMs = 0, lastRxMs = 0, lastLinkMs = 0;
uint8_t sonarTurn = 0;
unsigned long rxBadBytes = 0, rxLineOverflows = 0, softwareRxOverflows = 0;
char usbBuffer[4];
uint8_t usbIndex = 0;
const unsigned long LINK_TIMEOUT_MS = 1500UL;
const unsigned long RADAR_LINK_TIMEOUT_MS = 4000UL;

// ---------------- prototipos ----------------
void processNiclaByte(uint8_t raw);
void processMotion(char cmd);
void setRadar(bool on);
void reproducirAudio(uint8_t track);
void sendNicla(const char *line);
void sendAck(char kind, char code);
void sendTelemetry();
void sendRadar();
void adelante(); void atras(); void izquierda(); void derecha(); void detener();

// ============================================================
void setup() {
  Serial.begin(115200);

  pinMode(PWM_L, OUTPUT); pinMode(PWM_R, OUTPUT);
  pinMode(IN1_L, OUTPUT); pinMode(IN2_L, OUTPUT);
  pinMode(IN1_R, OUTPUT); pinMode(IN2_R, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);
  detener();

  niclaSerial.begin(9600);
  dfPlayerSerial.begin(9600);   // begin() deja a este objeto escuchando...
  niclaSerial.listen();         // ...así que devolvemos la escucha a la NICLA

  Serial.println(F("OSMIOR UNO V5 - PINOUT DRIVER1 D3-D5, DRIVER2 D6-D8, STBY D9"));

  // I2C con timeout para que un BMP280 desconectado no cuelgue el loop.
  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif
  bmpOK = bmp.begin(BMP_ADDR);   // sin setSampling: misma config que funcionó en la prueba
  Serial.println(bmpOK ? F("[BMP] OK 0x76") : F("[BMP] no responde en 0x76"));

  pinMode(PIN_DHT, OUTPUT);
  digitalWrite(PIN_DHT, HIGH);

  // El DFPlayer tarda ~1.5 s en arrancar. No sonar al encender:
  // la pista 1 se solicita al recibir MISSION,START desde la HMI.
  while (millis() < 1800UL) { }
  dfPlayer.begin(dfPlayerSerial, false, false);  // sin retorno TX conectado
  dfPlayerOK = true;  // no se verifica la SD sin el retorno del reproductor
  dfPlayer.volume(25);
  dfPlayer.stop();               // evita continuar una pista previa al encender
  niclaSerial.listen();

  lastLinkMs = millis();
#if USE_WDT
  wdt_enable(WDTO_2S);
#endif
  sendNicla("READY,UNO,5");
}

// ============================================================
void readNiclaSerial() {
  while (niclaSerial.available() > 0) {
    processNiclaByte((uint8_t)niclaSerial.read());
  }
  if (niclaSerial.overflow()) {
    softwareRxOverflows++;
  }
}

void readUsbSerial() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (usbIndex == 1) {
        const char u = (char)toupper(usbBuffer[0]);
        lastLinkMs = millis();   // por USB cada comando dura 1.5 s (deadman)
        if (u == 'F' || u == 'B' || u == 'L' || u == 'R' || u == 'S') processMotion(u);
      }
      usbIndex = 0;
      continue;
    }
    if (usbIndex < sizeof(usbBuffer) - 1) usbBuffer[usbIndex++] = c;
    else usbIndex = 0;
  }
}

void processNiclaByte(uint8_t raw) {
  lastRxMs = millis();
  if (raw == '\r' || raw == '\n') { prefixByte = 0; return; }
  if (raw < 0x20 || raw > 0x7E) { rxBadBytes++; prefixByte = 0; return; }

  lastLinkMs = lastRxMs;            // byte válido => enlace vivo
  const char c = (char)toupper(raw);

  if (prefixByte) {
    const char p = prefixByte;
    prefixByte = 0;
    if (p == 'A' && c >= '1' && c <= '6') { reproducirAudio((uint8_t)(c - '0')); return; }
    if (p == 'Z' && (c == '0' || c == '1')) { setRadar(c == '1'); return; }
    rxLineOverflows++;              // comando de 2 bytes mal formado; sigue con c
  }

  switch (c) {
    case 'F': case 'B': case 'L': case 'R': case 'S': processMotion(c); break;
    case '?': sendTelemetry(); break;
    case 'P': sendRadar(); break;
    case 'A': case 'Z': prefixByte = c; break;
    default: rxLineOverflows++; break;
  }
}

void processMotion(char cmd) {
  switch (cmd) {
    case 'F': adelante(); break;
    case 'B': atras(); break;
    case 'L': izquierda(); break;
    case 'R': derecha(); break;
    default:  detener(); cmd = 'S'; break;
  }
  currentMotion = cmd;
  sendAck('M', cmd);
}

// ---------------- motores de traccion: distribucion original ----------------
void adelante() {
  digitalWrite(IN1_L, HIGH); digitalWrite(IN2_L, LOW);
  digitalWrite(IN1_R, HIGH); digitalWrite(IN2_R, LOW);
  analogWrite(PWM_L, VELOCIDAD); analogWrite(PWM_R, VELOCIDAD);
}
void atras() {
  digitalWrite(IN1_L, LOW); digitalWrite(IN2_L, HIGH);
  digitalWrite(IN1_R, LOW); digitalWrite(IN2_R, HIGH);
  analogWrite(PWM_L, VELOCIDAD); analogWrite(PWM_R, VELOCIDAD);
}
void izquierda() {
  digitalWrite(IN1_L, LOW); digitalWrite(IN2_L, HIGH);
  digitalWrite(IN1_R, HIGH); digitalWrite(IN2_R, LOW);
  analogWrite(PWM_L, VELOCIDAD); analogWrite(PWM_R, VELOCIDAD);
}
void derecha() {
  digitalWrite(IN1_L, HIGH); digitalWrite(IN2_L, LOW);
  digitalWrite(IN1_R, LOW); digitalWrite(IN2_R, HIGH);
  analogWrite(PWM_L, VELOCIDAD); analogWrite(PWM_R, VELOCIDAD);
}
void detener() {
  digitalWrite(IN1_L, LOW); digitalWrite(IN2_L, LOW);
  digitalWrite(IN1_R, LOW); digitalWrite(IN2_R, LOW);
  analogWrite(PWM_L, 0); analogWrite(PWM_R, 0);
}

// ---------------- DFPlayer ----------------
void reproducirAudio(uint8_t track) {
  if (track < 1 || track > 6) return;
  if (!dfPlayerOK) { sendAck('A', '0'); return; }
  dfPlayer.play(track);        // 10 bytes por SoftwareSerial TX (~10 ms)
  niclaSerial.listen();
  sendAck('A', (char)('0' + track));
}

// ---------------- radar ----------------
void radarPush(uint8_t ang, int16_t cm) {
  radarBuf[rHead].ang = ang;
  radarBuf[rHead].cm = cm;
  rHead = (rHead + 1) % RADAR_BUF;
  if (rCount < RADAR_BUF) rCount++;
  else rTail = (rTail + 1) % RADAR_BUF;   // buffer lleno: descarta el más viejo
}

void setRadar(bool on) {
  if (on && !radarActive) {
    rHead = rTail = rCount = 0;
    radarAngle = RADAR_MIN_DEG;
    radarDir = 1;
    radarServo.attach(PIN_SERVO);
    radarServo.write(radarAngle);
    radarNextMs = millis() + 400UL;       // tiempo para llegar al extremo
    radarActive = true;
  } else if (!on && radarActive) {
    radarActive = false;
    radarServo.detach();                  // sin pulsos => sin consumo ni jitter
  }
  sendAck('Z', radarActive ? '1' : '0');
}

// Devuelve true si hizo un ping (para no encadenar dos pings en un mismo ciclo).
bool radarTick(unsigned long now) {
  if (!radarActive) return false;
  if ((long)(now - radarNextMs) < 0) return false;
  if (now - lastPingEndMs < PING_GUARD_MS) return false;

  const unsigned int cm = sonarRadar.ping_cm();   // bloquea <= ~12 ms
  radarPush(radarAngle, cm == 0 ? -1 : (int16_t)cm);

  int next = (int)radarAngle + (int)radarDir * RADAR_STEP;
  if (next > RADAR_MAX_DEG) { radarDir = -1; next = (int)radarAngle - RADAR_STEP; }
  if (next < RADAR_MIN_DEG) { radarDir = 1;  next = (int)radarAngle + RADAR_STEP; }
  radarAngle = (uint8_t)next;
  radarServo.write(radarAngle);

  lastPingEndMs = millis();
  radarNextMs = lastPingEndMs + RADAR_STEP_MS;
  return true;
}

// ---------------- ultrasónicos laterales ----------------
void sonarTick(unsigned long now) {
  if (now - lastSonarMs < SONAR_PERIOD_MS) return;
  if (now - lastPingEndMs < PING_GUARD_MS) return;
  lastSonarMs = now;
  if (sonarTurn == 0) {
    const unsigned int cm = sonarL.ping_cm();
    distL = cm == 0 ? -1 : (int16_t)cm;
  } else {
    const unsigned int cm = sonarR.ping_cm();
    distR = cm == 0 ? -1 : (int16_t)cm;
  }
  sonarTurn ^= 1;
  lastPingEndMs = millis();
}

// ---------------- BMP280 (temperatura cabina + presión) ----------------
void bmpTick(unsigned long now) {
  if (!bmpOK) return;
  if (now - lastBmpMs >= BMP_PERIOD_MS) {
    lastBmpMs = now;
    const float t = bmp.readTemperature();
    const float p = bmp.readPressure();           // Pa
    if (!isnan(t) && !isnan(p) && p > 30000.0f) {
      tCab10 = (int16_t)lroundf(t * 10.0f);
      press10 = (int16_t)lroundf(p / 10.0f);      // hPa x10
      bmpLastOkMs = now;
    } else {
      bmpFailCount++;
    }
  }
  if (tCab10 != INVALID_T && (long)(now - bmpLastOkMs) > 5000L) { tCab10 = INVALID_T; press10 = -1; }
}

// ---------------- DHT11 (temperatura y humedad externas) ----------------
// Lectura en 2 fases para no bloquear 20 ms:
//   fase 1: baja la línea y sigue con el loop
//   fase 2 (>=20 ms después): captura de ~5 ms con interrupciones apagadas,
//           solo si la NICLA no está transmitiendo en ese momento.
bool dhtCapture() {
  uint16_t w[41];
  uint8_t n = 0;
  noInterrupts();
  pinMode(PIN_DHT, INPUT_PULLUP);              // suelta la línea y escucha ya
  // El timeout de pulseIn se reparte entre esperar, el HIGH de respuesta y el
  // pulso medido (hasta ~200 us juntos): 400 us deja margen.
  if (pulseIn(PIN_DHT, LOW, 400) != 0) {       // respuesta del sensor (~80 us)
    for (n = 0; n < 41; n++) {
      w[n] = (uint16_t)pulseIn(PIN_DHT, HIGH, 400);
      if (w[n] == 0) break;
    }
  }
  interrupts();
  dhtLastN = n;
  pinMode(PIN_DHT, OUTPUT);
  digitalWrite(PIN_DHT, HIGH);

  // Alineación: w[0..39] si pulseIn vio el inicio limpio; w[1..40] si contó
  // el HIGH de respuesta como primer pulso. El checksum decide cuál vale.
  for (uint8_t off = 0; off < 2; off++) {
    if (n < 40 + off) continue;
    uint8_t d[5] = {0, 0, 0, 0, 0};
    for (uint8_t i = 0; i < 40; i++) {
      d[i >> 3] <<= 1;
      if (w[i + off] > 45) d[i >> 3] |= 1;
    }
    if ((uint8_t)(d[0] + d[1] + d[2] + d[3]) == d[4] && d[0] <= 100) {
      rhExt = d[0];
      int16_t t = (int16_t)(d[2] & 0x7F) * 10 + (d[3] & 0x0F);
      if (d[3] & 0x80) t = -t;
      tExt10 = t;
      return true;
    }
  }
  return false;
}

void dhtTick(unsigned long now) {
  if (!dhtWaiting) {
    if (now - lastDhtMs < (dhtLastFailed ? DHT_RETRY_MS : DHT_PERIOD_MS)) return;
    pinMode(PIN_DHT, OUTPUT);
    digitalWrite(PIN_DHT, LOW);
    dhtStartMs = now;
    dhtWaiting = true;
    return;
  }
  if (now - dhtStartMs < 20UL) return;
  const bool rxBusy = niclaSerial.available() > 0 || (now - lastRxMs) < 15UL;
  if (rxBusy && (now - dhtStartMs) < 120UL) return;   // espera hueco en el UART
  if (!rxBusy) {
    const bool ok = dhtCapture();
    dhtLastFailed = !ok;
    if (ok) { dhtOkCount++; dhtLastOkMs = millis(); }
    else dhtFailCount++;
  } else {
    pinMode(PIN_DHT, OUTPUT); digitalWrite(PIN_DHT, HIGH);   // UART ocupado: reintenta pronto
    dhtLastFailed = true;
  }
  dhtWaiting = false;
  lastDhtMs = millis();
}

// ---------------- estado de sensores de ambiente ----------------
// bit0 BMP inicializó | bit1 BMP con lectura fresca | bit2 DHT11 con lectura fresca
uint8_t envStatus() {
  return (bmpOK ? 1 : 0) | (tCab10 != INVALID_T ? 2 : 0) | (tExt10 != INVALID_T ? 4 : 0);
}

void envTick(unsigned long now) {
  // Comparación con signo: 'now' se tomó antes de la lectura y puede ser menor que dhtLastOkMs.
  if (tExt10 != INVALID_T && (long)(now - dhtLastOkMs) > (long)ENV_STALE_MS) { tExt10 = INVALID_T; rhExt = -1; }
#if DEBUG_ENV
  if (now - lastEnvDbgMs >= 2000UL) {
    lastEnvDbgMs = now;
    Serial.print(F("[ENV] DHT ok=")); Serial.print(dhtOkCount);
    Serial.print(F(" fail=")); Serial.print(dhtFailCount);
    Serial.print(F(" pulsos=")); Serial.print(dhtLastN);
    Serial.print(F(" T10=")); Serial.print(tExt10);
    Serial.print(F(" HR=")); Serial.print(rhExt);
    Serial.print(F(" | BMP ")); Serial.print(bmpOK ? F("init-ok") : F("SIN-INIT"));
    Serial.print(F(" fail=")); Serial.print(bmpFailCount);
    Serial.print(F(" T10=")); Serial.print(tCab10);
    Serial.print(F(" P10=")); Serial.println(press10);
  }
#endif
}

// ---------------- seguridad ----------------
void safetyTick(unsigned long now) {
  if (now - lastLinkMs > LINK_TIMEOUT_MS && currentMotion != 'S') {
    detener();
    currentMotion = 'S';
    Serial.println(F("[SAFE] sin enlace NICLA -> STOP"));
  }
  if (radarActive && now - lastLinkMs > RADAR_LINK_TIMEOUT_MS) {
    radarActive = false;
    radarServo.detach();
  }
}

// ============================================================
void loop() {
#if USE_WDT
  wdt_reset();
#endif
  readNiclaSerial();
  readUsbSerial();

  const unsigned long now = millis();
  safetyTick(now);
  if (!radarTick(now)) sonarTick(now);   // máximo UN ping bloqueante por ciclo
  bmpTick(now);
  dhtTick(now);
  envTick(now);
}

// ---------------- salida hacia NICLA ----------------
void sendNicla(const char *line) {
  niclaSerial.print(line);
  niclaSerial.write('\n');
  niclaSerial.listen();
}

void sendAck(char kind, char code) {
  niclaSerial.print(F("K,"));
  niclaSerial.write(kind);
  niclaSerial.write(',');
  niclaSerial.write(code);
  niclaSerial.write('\n');
  niclaSerial.listen();
}

static unsigned int clamp16(unsigned long v) { return (unsigned int)(v > 65535UL ? 65535UL : v); }

void sendTelemetry() {
  niclaSerial.print(F("T,"));
  niclaSerial.print(distL);   niclaSerial.write(',');
  niclaSerial.print(distR);   niclaSerial.write(',');
  niclaSerial.print(dfPlayerOK ? 1 : 0); niclaSerial.write(',');
  niclaSerial.write(currentMotion); niclaSerial.write(',');
  niclaSerial.print(clamp16(rxBadBytes));          niclaSerial.write(',');
  niclaSerial.print(clamp16(softwareRxOverflows)); niclaSerial.write(',');
  niclaSerial.print(clamp16(rxLineOverflows));     niclaSerial.write(',');
  niclaSerial.print(tExt10);  niclaSerial.write(',');
  niclaSerial.print(rhExt);   niclaSerial.write(',');
  niclaSerial.print(tCab10);  niclaSerial.write(',');
  niclaSerial.print(press10); niclaSerial.write(',');
  niclaSerial.print(radarActive ? 1 : 0); niclaSerial.write(',');
  niclaSerial.println((int)envStatus());
  niclaSerial.listen();
}

void sendRadar() {
  niclaSerial.write('R');
  uint8_t n = 0;
  while (rCount > 0 && n < 4) {
    niclaSerial.write(',');
    niclaSerial.print(radarBuf[rTail].ang);
    niclaSerial.write(':');
    niclaSerial.print(radarBuf[rTail].cm);
    rTail = (rTail + 1) % RADAR_BUF;
    rCount--;
    n++;
  }
  niclaSerial.write('\n');
  niclaSerial.listen();
}
