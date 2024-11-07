/*
  Modbus T1S Server Temperature Humidity sensor

  This sketch creates a Modbus T1S Server and demonstrates
  how to use read temperature and humidity values as sensed
  by the RTU Modbus capable MD02 sensor.

  Circuit:
  - Arduino Uno Wifi R4
   - T1S shield
    - SPE ethernet connected to the client
    - ISO GND connected to GND of the Modbus RTU server
    - Y connected to A/Y of the Modbus RTU client
    - Z connected to B/Z of the Modbus RTU client
    - all the terminations placed on the hardware
*/

#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

static uint8_t const T1S_PLCA_NODE_ID = 0;
static uint16_t const UDP_SERVER_PORT = 8889;

Arduino_10BASE_T1S_UDP udp_server;

void setup() {
  Serial.begin(115200);

  ModbusT1SServer.setT1SServer(&udp_server);
  ModbusT1SServer.setT1SPort(UDP_SERVER_PORT);
  ModbusT1SServer.setBadrate(9600);
  ModbusT1SServer.setCallback(OnPlcaStatus);
  if (!ModbusT1SServer.begin(T1S_PLCA_NODE_ID)) {
    Serial.println("Failed to start Modbus T1S Server!");
    while (1);
  }
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
