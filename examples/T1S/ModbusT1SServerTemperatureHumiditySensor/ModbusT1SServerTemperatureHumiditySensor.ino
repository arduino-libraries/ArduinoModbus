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

#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>
#include "arduino_secrets.h"

/**************************************************************************************
   CONSTANTS
 **************************************************************************************/
#define RS485_SERIAL Serial1
#define RS485_TX_PIN 1
#define RS485_RX_PIN 0
#define RS485_DE_PIN 8
#define RS485_RE_PIN 7
#define PRE_DELAY    100
#define POST_DELAY   100
#define PWR_CARRIER_RATIO  0.18f
RS485Class serial485(RS485_SERIAL, RS485_TX_PIN, RS485_DE_PIN, RS485_RE_PIN);

static unsigned int const MODBUS_BAUDRATE      = 9600;
static float        const MODBUS_BIT_DURATION  = 1.f / MODBUS_BAUDRATE;
static float        const MODBUS_PRE_DELAY_BR  = MODBUS_BIT_DURATION * 9.6f * 3.5f * 1e6;
static float        const MODBUS_POST_DELAY_BR = MODBUS_BIT_DURATION * 9.6f * 3.5f * 1e6;

static int          const MODBUS_DEVICE_ID                   = 1;
static int          const MODBUS_DEVICE_TEMPERATURE_REGISTER = 0x0001;
static int          const MODBUS_DEVICE_HUMIDITY_REGISTER    = 0x0002;

static uint8_t const T1S_PLCA_NODE_ID = 0; /* The UDP server doubles as PLCA coordinator. */

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

/**************************************************************************************
   GLOBAL VARIABLES
 **************************************************************************************/
auto const tc6_io = new TC6::TC6_Io
( SPI
  , CS_PIN
  , RESET_PIN
  , IRQ_PIN);
auto const tc6_inst = new TC6::TC6_Arduino_10BASE_T1S(tc6_io);
Arduino_10BASE_T1S_UDP udp_server;

/**************************************************************************************
   SETUP/LOOP
 **************************************************************************************/
void setup() {
  Serial.begin(115200);

  Serial.println("Modbus RTU Server LED");

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
    Serial.println("'TC6_Io::begin(...)' failed.");
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

  if (!udp_server.begin(UDP_SERVER_PORT))
  {
    Serial.println("begin(...) failed for UDP coil read server ");
    for (;;) { }
  }

  tc6_inst->digitalWrite(TC6::DIO::A0, false);
  /* A1 -> T1S_DISABLE -> close the switch connecting network to board. */
  tc6_inst->digitalWrite(TC6::DIO::A1, true);

  serial485.setDelays(MODBUS_PRE_DELAY_BR, MODBUS_POST_DELAY_BR);
  unsigned long br = MODBUS_BAUDRATE;
  uint16_t config = SERIAL_8N1;
  if (!ModbusT1SServer.begin(serial485, br, config)) {
    Serial.println("Failed to start Modbus RTU Client!");
    while (1);
  }

  ModbusT1SServer.setT1SServer(&udp_server);
  Serial.println("UDP_Server");
  Serial.println(MODBUS_PRE_DELAY_BR);
  Serial.println(MODBUS_POST_DELAY_BR);
}

void loop() {
  /* Services the hardware and the protocol stack.
     Must be called cyclic. The faster the better.
  */
  tc6_inst->service();

  static unsigned long prev_beacon_check = 0;

  auto const now = millis();
  if ((now - prev_beacon_check) > 1000)
  {
    prev_beacon_check = now;
    if (!tc6_inst->getPlcaStatus(OnPlcaStatus)) {
      Serial.println("getPlcaStatus(...) failed");
    }
  }

  switch (ModbusT1SServer.parsePacket())
  {
    case UDP_READ_COIL_PORT:
      Serial.println("Read coil");
      ModbusT1SServer.coilRead();
      break;

    case UDP_WRITE_COIL_PORT:
      Serial.println("Write coil");
      ModbusT1SServer.coilWrite();
      break;

    case UDP_READ_DI_PORT:
      Serial.println("Read discrete input");
      ModbusT1SServer.discreteInputRead();
      break;

    case UDP_READ_IR_PORT:
      Serial.println("Read input register");
      ModbusT1SServer.inputRegisterRead();
      break;

    case UDP_READ_HR_PORT:
      Serial.println("Read holding register");
      ModbusT1SServer.holdingRegisterRead();
      break;

    case UDP_WRITE_HR_PORT:
      Serial.println("Write holding register");
      ModbusT1SServer.holdingRegisterWrite();
      break;

    default:
      break;
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