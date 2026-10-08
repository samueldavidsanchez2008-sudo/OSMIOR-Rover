#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>

// ============================================================
// OSMIOR - Arduino UNO
// Motores: se mantienen exactamente en la logica/pines original
// UART UNO <-> NICLA: D10 RX, D11 TX
// DFPlayer: D12 TX -> RX del DFPlayer (D13 queda libre)
// HC-SR04 izquierdo: TRIG A0, ECHO A1
// HC-SR04 derecho:   TRIG A2, ECHO A3
// ============================================================

// ---------------- UART hacia NICLA ----------------
SoftwareSerial niclaSerial(10, 11);

// ---------------- DFPlayer ----------------
// SoftwareSerial(rx, tx).
// Solo usamos TX del UNO hacia RX del DFPlayer.
// No conectamos el TX del DFPlayer para evitar otro receptor
// SoftwareSerial compitiendo con la UART del NICLA.
SoftwareSerial dfPlayerSerial(13, 12);
DFRobotDFPlayerMini dfPlayer;
bool dfPlayerOK = false;

// ---------------- Driver Izquierdo (TB6612 #1) ----------------
const int PWM_L = 3;
const int IN1_L = 4;
const int IN2_L = 5;

// ---------------- Driver Derecho (TB6612 #2) ----------------
const int PWM_R = 6;
const int IN1_R = 7;
const int IN2_R = 8;

// ---------------- Standby compartido ----------------
const int STBY = 9;

// ---------------- Velocidad base ----------------
const int VELOCIDAD = 220;

// ---------------- HC-SR04 ----------------
const int TRIG_L = A0;
const int ECHO_L = A1;
const int TRIG_R = A2;
const int ECHO_R = A3;

// ---------------- Telemetria ----------------
unsigned long lastTelemetry = 0;
const unsigned long TELEMETRY_PERIOD_MS = 200;

// ---------------- Parser NICLA ----------------
String niclaLine = "";

void procesarComando(char cmd, String origen);
void procesarLineaNicla(String linea);

void adelante();
void atras();
void izquierda();
void derecha();
void detener();

float medirSensor(int trigPin, int echoPin);
void enviarTelemetria();
void reproducirAudio(uint8_t pista);

void setup() {
  Serial.begin(115200);
  niclaSerial.begin(9600);
  dfPlayerSerial.begin(9600);

  pinMode(PWM_L, OUTPUT);
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);

  pinMode(PWM_R, OUTPUT);
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);

  pinMode(STBY, OUTPUT);

  pinMode(TRIG_L, OUTPUT);
  pinMode(ECHO_L, INPUT);

  pinMode(TRIG_R, OUTPUT);
  pinMode(ECHO_R, INPUT);

  digitalWrite(STBY, HIGH);
  detener();

  Serial.println("==================================================");
  Serial.println("OSMIOR UNO: inicializando...");
  Serial.println("Motores OK.");
  Serial.println("HC-SR04: A0/A1 + A2/A3.");
  Serial.println("DFPlayer: D12 -> RX.");
  Serial.println("==================================================");

  delay(500);

  // Audio 001: inicializacion del rover.
  // No dependemos de una respuesta del DFPlayer.
  if (dfPlayer.begin(dfPlayerSerial, false, true)) {
    dfPlayerOK = true;
    dfPlayer.volume(25);
    delay(250);
    dfPlayer.play(1);
    Serial.println("[DF] Track 001 reproducido.");
  } else {
    dfPlayerOK = false;
    Serial.println("[DF] No se pudo inicializar.");
  }

  // Aviso de seguridad: arrancar detenido.
  detener();
}

void loop() {

  // ----------------------------------------------------------
  // Leer comandos provenientes del NICLA
  // ----------------------------------------------------------
  while (niclaSerial.available() > 0) {
    char c = (char)niclaSerial.read();

    if (c == '\n' || c == '\r') {
      if (niclaLine.length() > 0) {
        procesarLineaNicla(niclaLine);
        niclaLine = "";
      }
    } else {
      // Protocolo ASCII solamente.
      // Evita acumulacion descontrolada.
      if (niclaLine.length() < 40) {
        niclaLine += c;
      } else {
        niclaLine = "";
      }
    }
  }

  // ----------------------------------------------------------
  // Comandos directos por USB para depuracion
  // F/B/L/R/S
  // ----------------------------------------------------------
  if (Serial.available() > 0) {
    char cmd = (char)Serial.read();
    procesarComando(cmd, "USB");
  }

  // ----------------------------------------------------------
  // Telemetria a NICLA
  // ----------------------------------------------------------
  if (millis() - lastTelemetry >= TELEMETRY_PERIOD_MS) {
    lastTelemetry = millis();
    enviarTelemetria();
  }
}

// ============================================================
// PROTOCOLO DESDE NICLA
// ============================================================

void procesarLineaNicla(String linea) {
  linea.trim();

  if (linea.length() == 0) return;

  // Comandos de audio: AUD:1 ... AUD:5
  if (linea.startsWith("AUD:")) {
    int pista = linea.substring(4).toInt();

    if (pista >= 1 && pista <= 5) {
      reproducirAudio((uint8_t)pista);
    }
    return;
  }

  // Comandos de movimiento: F/B/L/R/S
  if (linea.length() == 1) {
    char cmd = linea.charAt(0);
    procesarComando(cmd, "NICLA UART");
    return;
  }

  Serial.print("[NICLA] Linea no reconocida: ");
  Serial.println(linea);
}

void procesarComando(char cmd, String origen) {

  if (cmd == '\n' || cmd == '\r' || cmd == ' ') return;

  cmd = toupper(cmd);

  Serial.print("[UNO] Comando recibido desde [");
  Serial.print(origen);
  Serial.print("]: ");
  Serial.println(cmd);

  switch (cmd) {

    case 'F':
      adelante();
      Serial.println(" -> Accion: Adelante");
      break;

    case 'B':
      atras();
      Serial.println(" -> Accion: Atras");
      break;

    case 'L':
      izquierda();
      Serial.println(" -> Accion: Izquierda");
      break;

    case 'R':
      derecha();
      Serial.println(" -> Accion: Derecha");
      break;

    case 'S':
      detener();
      Serial.println(" -> Accion: Detener");
      break;

    default:
      Serial.println(" -> Comando no reconocido.");
      break;
  }
}

// ============================================================
// MOTORES - MISMA LOGICA DEL CODIGO ORIGINAL
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

void reproducirAudio(uint8_t pista) {

  if (!dfPlayerOK) {
    Serial.print("[DF] No disponible. Pista: ");
    Serial.println(pista);
    return;
  }

  dfPlayer.play(pista);

  Serial.print("[DF] Reproduciendo pista ");
  Serial.println(pista);
}

// ============================================================
// HC-SR04
// ============================================================

float medirSensor(int trigPin, int echoPin) {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  long duracion = pulseIn(echoPin, HIGH, 30000);

  if (duracion <= 0) {
    return 0.0;
  }

  return (duracion * 0.0343) / 2.0;
}

// ============================================================
// TELEMETRIA UNO -> NICLA
// ============================================================

void enviarTelemetria() {

  float distL = medirSensor(TRIG_L, ECHO_L);

  // Separacion temporal para reducir interferencia entre sensores.
  delay(8);

  float distR = medirSensor(TRIG_R, ECHO_R);

  niclaSerial.print("TEL;D1=");
  niclaSerial.print(distL, 1);

  niclaSerial.print(";D2=");
  niclaSerial.print(distR, 1);

  niclaSerial.print(";DF=");
  niclaSerial.print(dfPlayerOK ? 1 : 0);

  niclaSerial.print(";PWM=");
  niclaSerial.print(VELOCIDAD);

  niclaSerial.print("\n");
}