#include "LoRaWan_APP.h"
#include "Arduino.h"


#define RF_FREQUENCY                                915000000 // Hz
#define LORA_BANDWIDTH                              0
#define LORA_SPREADING_FACTOR                       7
#define LORA_CODINGRATE                              1
#define LORA_PREAMBLE_LENGTH                        8
#define LORA_SYMBOL_TIMEOUT                         0
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 30

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

// --- Motores: 3 por lado ---
// Pines de dirección (IN1/IN2 de cada driver)
const int rightMotorPin1[3] = {16, 25, 27};
const int rightMotorPin2[3] = {17, 26, 14};
const int leftMotorPin1[3]  = {18, 32, 33};
const int leftMotorPin2[3]  = {19, 4,  2};

// Pines de enable/PWM de cada driver
const int rightEnablePin[3] = {22, 21, 13};
const int leftEnablePin[3]  = {23, 5,  15};

// Canales PWM (LEDC) del ESP32 — hay 16 disponibles, usamos 6
const int rightPWMChannel[3] = {0, 1, 2};
const int leftPWMChannel[3]  = {3, 4, 5};

#define MAX_MOTOR_SPEED 255
#define DRIVE_SPEED     200
#define TURN_SPEED      180

const int PWMFreq = 1000;
const int PWMResolution = 8;

// Mueve TODOS los motores de un lado con la misma velocidad/sentido
void moverLado(const int pin1[3], const int pin2[3], const int pwmChannel[3], int speed)
{
  for (int i = 0; i < 3; i++)
  {
    if (speed < 0)
    {
      digitalWrite(pin1[i], LOW);
      digitalWrite(pin2[i], HIGH);
    }
    else if (speed > 0)
    {
      digitalWrite(pin1[i], HIGH);
      digitalWrite(pin2[i], LOW);
    }
    else
    {
      digitalWrite(pin1[i], LOW);
      digitalWrite(pin2[i], LOW);
    }
    ledcWrite(pwmChannel[i], abs(speed));
  }
}

void rotateMotor(int rightSpeed, int leftSpeed)
{
  moverLado(rightMotorPin1, rightMotorPin2, rightPWMChannel, rightSpeed);
  moverLado(leftMotorPin1, leftMotorPin2, leftPWMChannel, leftSpeed);
}

void setUpPinModes()
{
  for (int i = 0; i < 3; i++)
  {
    pinMode(rightMotorPin1[i], OUTPUT);
    pinMode(rightMotorPin2[i], OUTPUT);
    pinMode(leftMotorPin1[i], OUTPUT);
    pinMode(leftMotorPin2[i], OUTPUT);

    ledcSetup(rightPWMChannel[i], PWMFreq, PWMResolution);
    ledcSetup(leftPWMChannel[i], PWMFreq, PWMResolution);
    ledcAttachPin(rightEnablePin[i], rightPWMChannel[i]);
    ledcAttachPin(leftEnablePin[i], leftPWMChannel[i]);
  }
}

// Aplica el comando direccional recibido: 'F','B','L','R','S'
void aplicarComando(char comando)
{
  switch (comando)
  {
    case 'F': rotateMotor(DRIVE_SPEED, DRIVE_SPEED); break;
    case 'B': rotateMotor(-DRIVE_SPEED, -DRIVE_SPEED); break;
    case 'L': rotateMotor(TURN_SPEED, -TURN_SPEED); break;  // gira sobre su eje
    case 'R': rotateMotor(-TURN_SPEED, TURN_SPEED); break;
    case 'S':
    default:  rotateMotor(0, 0); break;
  }
}

// --- LoRa ---
String StrDataLoraIn;
byte PinLed = 6;

static RadioEvents_t RadioEvents;
int16_t rssiVal, rxSize;
bool lora_idle = true;

void setup()
{
  pinMode(PinLed, OUTPUT);
  digitalWrite(PinLed, LOW);

  setUpPinModes();

  Serial.begin(115200);
  Mcu.begin();

  rssiVal = 0;

  RadioEvents.RxDone = OnRxDone;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                     LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                     LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                     0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
}

void loop()
{
  if (lora_idle)
  {
    lora_idle = false;
    Serial.println("into RX mode");
    Radio.Rx(0);
  }
  Radio.IrqProcess();
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
  rssiVal = rssi;
  rxSize = size;
  memcpy(rxpacket, payload, size);
  rxpacket[size] = '\0';
  Radio.Sleep();
  Serial.printf("\r\nPaquete Recibido: \"%s\" with rssi %d , length %d\r\n", rxpacket, rssiVal, rxSize);

  StrDataLoraIn = String(rxpacket);

  if (StrDataLoraIn.startsWith("CMD_") && StrDataLoraIn.length() >= 5)
  {
    char comando = StrDataLoraIn.charAt(4);
    aplicarComando(comando);
    digitalWrite(PinLed, comando == 'S' ? LOW : HIGH);
  }

  lora_idle = true;
}
