/*
  Modbus T1S Client

  This sketch demonstrates how to send commands to a Modbus T1S server connected
  via T1S Single Pair Ethernet.

  Circuit:
   - T1S shield
   - Uno WiFi R4
*/

#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

Arduino_10BASE_T1S_UDP udp_client;
static uint8_t const T1S_PLCA_NODE_ID = 2;
static uint16_t const UDP_SERVER_PORT = 8889;
static uint16_t const UDP_CLIENT_PORT = 8888;
#define MODBUS_ID 42

void setup() {
 Serial.begin(115200);

  ModbusT1SClient.setT1SClient(&udp_client);
  ModbusT1SClient.setT1SPort(UDP_CLIENT_PORT);
  ModbusT1SClient.setServerPort(UDP_SERVER_PORT);
  ModbusT1SClient.setModbusId(MODBUS_ID);
  ModbusT1SClient.setCallback(OnPlcaStatus);

  if (!ModbusT1SClient.begin(T1S_PLCA_NODE_ID)) {
    Serial.println("Failed to start Modbus T1S Client!");
    while (1);
  }
}

void loop() {
  ModbusT1SClient.checkPLCAStatus();

  int res = ModbusT1SClient.coilRead(0x00);
  if (res == -1) {
    Serial.println("Failed to read coil! ");
  } else {
    Serial.print("Coil value: ");
    Serial.println(res);
  }

  res = ModbusT1SClient.coilWrite(0, 0x01);
  if (res == -1) {
    Serial.println("Failed to write coil! ");
  } else {
    Serial.println("write done");
  }

  res =  ModbusT1SClient.inputRegisterRead(0x00);
  if (res == -1) {
    Serial.println("Failed to read Input Register! ");
  } else {
    Serial.print("Input Register value: ");
    Serial.println(res);
  }
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