/*
  Modbus T1S Server


  This sketch demonstrates how to receive commands from a Modbus T1S Client connected
  via T1S Single Pair Ethernet.

  Circuit:
   - T1S shield
   - Uno WiFi R4
*/

#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

RS485Class serial485(RS485_SERIAL, RS485_TX_PIN, RS485_DE_PIN, RS485_RE_PIN);

static uint8_t const T1S_PLCA_NODE_ID = 0;
static uint16_t const UDP_SERVER_PORT = 8889;

Arduino_10BASE_T1S_UDP udp_server;

void setup() {
  Serial.begin(115200);

  ModbusT1SServer.setT1SServer(udp_server);
  ModbusT1SServer.setT1SPort(UDP_SERVER_PORT);
  ModbusT1SServer.setCallback(OnPlcaStatus);

  if (!ModbusT1SServer.begin(T1S_PLCA_NODE_ID, 9600, SERIAL_8N1, serial485)) {
    Serial.println("Failed to start Modbus T1S Server!");
    while (1);
  }
  ModbusT1SServer.disablePOE();
}

void loop() {
  ModbusT1SServer.update();
}

static void OnPlcaStatus(bool success, bool plcaStatus)
{
  if (!success)
  {
    Serial.println("PLCA status register read failed");
    return;
  }

  if (plcaStatus) {
    Serial.println("PLCA Mode active");
  } else {
    Serial.println("CSMA/CD fallback");
    tc6_inst->enablePlca();
  }
}
