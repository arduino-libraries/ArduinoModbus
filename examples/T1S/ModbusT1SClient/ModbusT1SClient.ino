/*
  Modbus T1S Client Toggle

  This sketch demonstrates how to send commands to a Modbus T1S server connected
  via T1S Single Pair Ethernet.

  Circuit:
   - T1S shield
   - Uno WiFi R4
*/

#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>
#include "arduino_secrets.h"
/**************************************************************************************
   CONSTANTS
 **************************************************************************************/
static uint8_t const T1S_PLCA_NODE_ID = 2;

static IPAddress const ip_addr     {
  192, 168,  42, 100 + T1S_PLCA_NODE_ID
};
static IPAddress const network_mask {
  255, 255, 255,   0
};
static IPAddress const gateway     {
  192, 168,  42, 100
};

static T1SPlcaSettings const t1s_plca_settings {
  T1S_PLCA_NODE_ID
};
static T1SMacSettings const t1s_default_mac_settings;

static IPAddress const UDP_SERVER_IP_ADDR = {192, 168,  42, 100 + 0};

/**************************************************************************************
   GLOBAL VARIABLES
 **************************************************************************************/
auto const tc6_io = new TC6::TC6_Io
( SPI
  , CS_PIN
  , RESET_PIN
  , IRQ_PIN);
auto const tc6_inst = new TC6::TC6_Arduino_10BASE_T1S(tc6_io);
Arduino_10BASE_T1S_UDP udp_client;


void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Modbus T1S Client Toggle");

  /* Initialize digital IO interface for interfacing
     with the LAN8651.
  */
  pinMode(IRQ_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IRQ_PIN),
  []() {
    tc6_io->onInterrupt();
  },
  FALLING);

  /* Initialize IO module. */
  if (!tc6_io->begin())
  {
    Serial.println("'tc6_io::begin(...)' failed.");
    for (;;) { }
  }

  MacAddress const mac_addr = MacAddress::create_from_uid();

  if (!tc6_inst->begin(ip_addr
                       , network_mask
                       , gateway
                       , mac_addr
                       , t1s_plca_settings
                       , t1s_default_mac_settings))
  {
    Serial.println("'TC6::begin(...)' failed.");
    for (;;) { }
  }

  Serial.print("IP\t");
  Serial.println(ip_addr);
  Serial.println(mac_addr);
  Serial.println(t1s_plca_settings);
  Serial.println(t1s_default_mac_settings);

  if (!udp_client.begin(UDP_CLIENT_PORT))
  {
    Serial.println("begin(...) failed for UDP client");
    for (;;) { }
  }

  /* A0 -> LOCAL_ENABLE -> DO NOT feed power from board to network. */
  tc6_inst->digitalWrite(TC6::DIO::A0, false);
  /* A1 -> T1S_DISABLE -> Open the switch connecting network to board by pulling EN LOW. */
  tc6_inst->digitalWrite(TC6::DIO::A1, false);

  ModbusT1SClient.setServerIp(UDP_SERVER_IP_ADDR);
  ModbusT1SClient.setServerPort(UDP_SERVER_PORT);
  ModbusT1SClient.setModbusId(MODBUS_ID);

  Serial.println("UDP_Client");
}

void loop() {
  tc6_inst->service();

  static unsigned long prev_beacon_check = 0;
  static unsigned long prev_udp_packet_sent = 0;

  auto const now = millis();

  if ((now - prev_beacon_check) > 1000)
  {
    prev_beacon_check = now;
    if (!tc6_inst->getPlcaStatus(OnPlcaStatus)) {
      Serial.println("getPlcaStatus(...) failed");
    }
  }
  // for (slave) id 1: write the value of 0x01, to the coil at address 0x00
  int res = ModbusT1SClient.coilRead(0x00, &udp_client, UDP_READ_COIL_PORT);

  if (res == -1) {
    Serial.println("Failed to read coil! ");
  } else {
    Serial.print("Coil value: ");
    Serial.println(res);
  }

  res = ModbusT1SClient.coilWrite(0x00, 1, &udp_client, UDP_WRITE_COIL_PORT);
  if (res == -1) {
    Serial.println("Failed to write coil! ");
  } else {
    Serial.println("write done");
  }

  res =  ModbusT1SClient.inputRegisterRead(0x00, &udp_client, UDP_READ_IR_PORT);
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