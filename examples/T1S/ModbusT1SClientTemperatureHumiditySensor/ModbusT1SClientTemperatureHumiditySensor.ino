/*
  Modbus T1S Client Temperature Humidity sensor

  This sketch creates a Modbus T1S Client and demonstrates
  how to use the ModbusT1S API to communicate.
  - Arduino Uno Wifi R4
  - T1S shield
    - SPE ethernet connected to the client
    - all the terminations placed on the hardware
*/

#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

static uint8_t const T1S_PLCA_NODE_ID = 2;
static uint16_t const UDP_SERVER_PORT = 8889;
static uint16_t const UDP_CLIENT_PORT = 8888;

Arduino_10BASE_T1S_UDP udp_client;

void setup() {
  Serial.begin(115200);

  ModbusT1SClient.setT1SClient(udp_client);
  ModbusT1SClient.setT1SPort(UDP_CLIENT_PORT);
  ModbusT1SClient.setServerPort(UDP_SERVER_PORT);
  ModbusT1SClient.setCallback(OnPlcaStatus);

  if (!ModbusT1SClient.begin(T1S_PLCA_NODE_ID)) {
    Serial.println("Failed to start Modbus T1S Client!");
    while (1);
  }
  ModbusT1SClient.disablePOE();
}

unsigned long start = 0;
void loop() {
  ModbusT1SClient.update();

  if ((millis() - start) > 1000)
  {
    int  res = ModbusT1SClient.inputRegisterRead(1, 0x01);
    if (res == -1) {
      Serial.println("Failed to read temperature! ");
    } else {
      int16_t const temperature_raw = res;
      float const temperature_deg = temperature_raw / 10.f;
      Serial.print("Temperature: ");
      Serial.println(temperature_deg);
    }

    res = ModbusT1SClient.inputRegisterRead(1, 0x02);
    if (res == -1) {
      Serial.println("Failed to read humidity! ");
    } else {
      int16_t const humidity_raw = res;
      float const humidity_per_cent = humidity_raw / 10.f;
      Serial.print("Humidity: ");
      Serial.println(humidity_per_cent);
    }
    start = millis();
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
