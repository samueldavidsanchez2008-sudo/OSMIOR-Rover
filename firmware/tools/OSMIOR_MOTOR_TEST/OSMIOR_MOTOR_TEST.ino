// OSMIOR - DIAGNOSTICO AISLADO DE MOTORES (V5/V3 pinout)
// Solo para comprobar drivers/cableado, aislando Nicla, UART y sensores.
// Levantar TODAS las ruedas antes de alimentar. Abrir Serial Monitor a 115200
// baudios y enviar F, B, L, R o S con final de linea. Enviar S para detener.
// F: ambos lados adelante | B: ambos atras | L: izquierda atras/derecha adelante
// R: izquierda adelante/derecha atras | S: STOP

#include <ctype.h>

const uint8_t PWM_L = 3;
const uint8_t IN1_L = 4;
const uint8_t IN2_L = 5;
const uint8_t PWM_R = 6;
const uint8_t IN1_R = 7;
const uint8_t IN2_R = 8;
const uint8_t STBY  = 9;
const uint8_t SPEED_TEST = 180;
char lineBuf[8];
uint8_t lineLen = 0;

void stopMotors() {
  digitalWrite(IN1_L, LOW); digitalWrite(IN2_L, LOW);
  digitalWrite(IN1_R, LOW); digitalWrite(IN2_R, LOW);
  analogWrite(PWM_L, 0); analogWrite(PWM_R, 0);
}
void forward() {
  digitalWrite(IN1_L, HIGH); digitalWrite(IN2_L, LOW);
  digitalWrite(IN1_R, HIGH); digitalWrite(IN2_R, LOW);
  analogWrite(PWM_L, SPEED_TEST); analogWrite(PWM_R, SPEED_TEST);
}
void backward() {
  digitalWrite(IN1_L, LOW); digitalWrite(IN2_L, HIGH);
  digitalWrite(IN1_R, LOW); digitalWrite(IN2_R, HIGH);
  analogWrite(PWM_L, SPEED_TEST); analogWrite(PWM_R, SPEED_TEST);
}
void leftTurn() {
  digitalWrite(IN1_L, LOW); digitalWrite(IN2_L, HIGH);
  digitalWrite(IN1_R, HIGH); digitalWrite(IN2_R, LOW);
  analogWrite(PWM_L, SPEED_TEST); analogWrite(PWM_R, SPEED_TEST);
}
void rightTurn() {
  digitalWrite(IN1_L, HIGH); digitalWrite(IN2_L, LOW);
  digitalWrite(IN1_R, LOW); digitalWrite(IN2_R, HIGH);
  analogWrite(PWM_L, SPEED_TEST); analogWrite(PWM_R, SPEED_TEST);
}
void runCommand(char c) {
  c = (char)toupper((unsigned char)c);
  switch (c) {
    case 'F': forward(); break;
    case 'B': backward(); break;
    case 'L': leftTurn(); break;
    case 'R': rightTurn(); break;
    case 'S': stopMotors(); break;
    default: Serial.println(F("Use F/B/L/R/S")); return;
  }
  Serial.print(F("Applied command: ")); Serial.println(c);
}
void setup() {
  Serial.begin(115200);
  pinMode(PWM_L, OUTPUT); pinMode(IN1_L, OUTPUT); pinMode(IN2_L, OUTPUT);
  pinMode(PWM_R, OUTPUT); pinMode(IN1_R, OUTPUT); pinMode(IN2_R, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  stopMotors();
  Serial.println(F("OSMIOR MOTOR TEST ready. Wheels lifted; send F/B/L/R/S."));
}
void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (lineLen == 1) runCommand(lineBuf[0]);
      lineLen = 0;
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;
    }
  }
}
