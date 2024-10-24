/*
  Modbus T1S Client Temperature Humidity sensor

  This sketch creates a Modbus T1S Client and demonstrates
  how to use the ModbusT1S API to communicate.
  - Arduino Uno Wifi R4
  - T1S shield
    - SPE ethernet connected to the client
    - all the terminations placed on the hardware
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
  tc6_inst->digitalWrite(TC6::DIO::A1, true);

  ModbusT1SClient.setServerIp(UDP_SERVER_IP_ADDR);
  ModbusT1SClient.setServerPort(UDP_SERVER_PORT);


  Serial.println("UDP_Client");
}

unsigned long start = 0;
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

  if ((millis() - start) > 1000)
  {
   int  res = ModbusT1SClient.inputRegisterRead(1, 0x01, &udp_client, UDP_READ_IR_PORT);
    if (res == -1) {
      Serial.println("Failed to read temperature! ");
    } else {
      int16_t const temperature_raw =res;
      float const temperature_deg = temperature_raw / 10.f;
      Serial.print("Temperature: ");
      Serial.println(temperature_deg);
    }
    
    res = ModbusT1SClient.inputRegisterRead(1, 0x02, &udp_client, UDP_READ_IR_PORT);
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