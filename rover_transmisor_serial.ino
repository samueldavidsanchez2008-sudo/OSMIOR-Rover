// ============================================================
// Este transmisor YA NO usa botones ni Bluetooth: recibe el
// comando por el puerto Serial (USB), enviado desde un script
// en la PC que lee el teclado (ver control_teclado.py).
//
// Protocolo por Serial: un solo carácter, 'F','B','L','R' o 'S'.
// Cada vez que llega un carácter válido y es distinto al último
// comando enviado, se retransmite por LoRa como "CMD_F", etc.
//
// El receptor NO cambia — sigue esperando "CMD_F/B/L/R/S".
// ============================================================

#include "LoRaWan_APP.h"
#include "Arduino.h"

#define RF_FREQUENCY                                915000000 // Hz
#define TX_OUTPUT_POWER                             5         // dBm
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

char lastCommand = 'S';
bool lora_idle = true;

// Vencimiento de seguridad: si la PC deja de mandar datos (se
// cierra el script, se desconecta el cable, etc.) el rover se
// detiene solo después de este tiempo sin recibir nada por Serial.
#define SERIAL_TIMEOUT_MS 500
unsigned long lastSerialMs = 0;

static RadioEvents_t RadioEvents;
void OnTxDone(void);
void OnTxTimeout(void);

void enviarComando(char comando)
{
  if (comando != lastCommand && lora_idle)
  {
    lastCommand = comando;

    sprintf(txpacket, "CMD_%c", lastCommand);

    Serial.printf("\r\nEnviando Paquete -> \"%s\" , length %d\r\n", txpacket, strlen(txpacket));

    Radio.Send((uint8_t *)txpacket, strlen(txpacket));
    lora_idle = false;
  }
}

void setup()
{
  Serial.begin(115200);
  Mcu.begin();

  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                     LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                     LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                     true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

  lastSerialMs = millis();
}

void loop()
{
  // Lee comandos que llegan de la PC por USB
  while (Serial.available() > 0)
  {
    char c = Serial.read();
    if (c == 'F' || c == 'B' || c == 'L' || c == 'R' || c == 'S')
    {
      enviarComando(c);
      lastSerialMs = millis();
    }
  }

  // Si la PC deja de responder, detener el rover por seguridad
  if (millis() - lastSerialMs > SERIAL_TIMEOUT_MS)
  {
    enviarComando('S');
  }

  Radio.IrqProcess();
}

void OnTxDone(void)
{
  Serial.println("TX done......");
  lora_idle = true;
}

void OnTxTimeout(void)
{
  Radio.Sleep();
  Serial.println("TX Timeout......");
  lora_idle = true;
}
