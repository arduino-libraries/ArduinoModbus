/*
  This file is part of the ArduinoModbus library.
  Copyright (c) 2018 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/
#if (defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNOR4_MINIMA))
#include <errno.h>

extern "C" {
#include "libmodbus/modbus.h"
#include "libmodbus/modbus-rtu.h"
}

#include "ModbusT1SServer.h"

size_t const UDP_RX_MSG_BUF_SIZE = 16 + 1;
RS485Class serial485(RS485_SERIAL, RS485_TX_PIN, RS485_DE_PIN, RS485_RE_PIN);

/**
 * @class ModbusT1SServerClass
 * Class for Modbus T1S Server communication.
 *
 * This class provides functionalities to communicate with a Modbus T1S server.
 */
ModbusT1SServerClass::ModbusT1SServerClass()
{
}

/**
 * Constructor for ModbusT1SServerClass with RS485 support.
 *
 * Initializes the Modbus server with RS485 communication.
 *
 * @param rs485 Reference to an RS485Class object for RS485 communication.
 */
ModbusT1SServerClass::ModbusT1SServerClass(RS485Class& rs485) : _rs485(&rs485)
{

}

/**
 * Destructor for ModbusT1SServerClass.
 */
ModbusT1SServerClass::~ModbusT1SServerClass()
{
}

/**
 * Initializes the Modbus server with the specified RS485 instance, baud rate, and configuration.
 *
 * This function sets up the Modbus server for communication using the specified RS485 instance, baud rate, and configuration.
 *
 * @param id (slave) id of the server
 * @param baudrate The baud rate for the Modbus communication.
 * @param config The configuration for the Modbus communication (e.g., parity, stop bits).
 * @return int Returns 1 if initialization is successful, 0 otherwise.
 */
int ModbusT1SServerClass::begin(int id, unsigned long baudrate, uint16_t config)
{
  if(!ModbusRTUClient.begin(*_rs485, baudrate, config)) {
    return -1;
  }

  ModbusRTUClient.setTimeout(2*1000UL);
  return 1;
}

/**
 * Initializes the Modbus server with the specified RS485 instance, baud rate, and configuration.
 *
 * This function sets up the Modbus server for communication using the specified RS485 instance, baud rate, and configuration.
 *
 * @param rs485 Reference to an RS485Class object for RS485 communication.
 * @param id (slave) id of the server
 * @param baudrate The baud rate for the Modbus communication.
 * @param config The configuration for the Modbus communication (e.g., parity, stop bits).
 * @return int Returns 1 if initialization is successful, 0 otherwise.
 */
int ModbusT1SServerClass::begin(RS485Class& rs485, int id, unsigned long baudrate, uint16_t config)
{
  _rs485 = &rs485;
  return begin(id, baudrate, config);
}

/**
 * Initializes the Modbus server with the specified RS485 instance, baud rate, and configuration.
 *
 * This function sets up the Modbus server for communication using the specified RS485 instance, baud rate, and configuration.
 *
 * @param rs485 Reference to an RS485Class object for RS485 communication.
 * @param baudrate The baud rate for the Modbus communication.
 * @param config The configuration for the Modbus communication (e.g., parity, stop bits).
 * @return int Returns 1 if initialization is successful, 0 otherwise.
 */
int ModbusT1SServerClass::begin(RS485Class& rs485, unsigned long baudrate, uint16_t config)
{
  _rs485 = &rs485;
  if(!ModbusRTUClient.begin(rs485, baudrate, config)) {
    return 0;
  }
  return 1;
}


int ModbusT1SServerClass::begin(int node_id)
{
  _node_id = node_id;
  pinMode(IRQ_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IRQ_PIN),

  []() {
    tc6_io->onInterrupt();
  },
  FALLING);

  /* Initialize IO module. */
  if (!tc6_io->begin())
  {
    return 0;
  }

  IPAddress ip_addr = IPAddress(192, 168,  42, 100 + _node_id);
  IPAddress network_mask = IPAddress(255, 255, 255, 0);
  IPAddress gateway = IPAddress(192, 168,  42, 100);

  T1SPlcaSettings t1s_plca_settings {
    _node_id
  };
  T1SMacSettings const t1s_default_mac_settings;
  MacAddress const mac_addr = MacAddress::create_from_uid();

  if (!tc6_inst->begin(ip_addr
                       , network_mask
                       , gateway
                       , mac_addr
                       , t1s_plca_settings
                       , t1s_default_mac_settings))
  {
    return 0;
  }

  Serial.print("IP\t");
  Serial.println(ip_addr);
  Serial.println(mac_addr);
  Serial.println(t1s_plca_settings);
  Serial.println(t1s_default_mac_settings);

  if (!_server->begin(udp_port))
  {
    return 0;
  }

  float MODBUS_BIT_DURATION  = 1.f / _baudrate;
  float MODBUS_PRE_DELAY_BR  = MODBUS_BIT_DURATION * 9.6f * 3.5f * 1e6;
  float MODBUS_POST_DELAY_BR = MODBUS_BIT_DURATION * 9.6f * 3.5f * 1e6;

  serial485.setDelays(MODBUS_PRE_DELAY_BR, MODBUS_POST_DELAY_BR);

  if(!ModbusRTUClient.begin(serial485, (unsigned long) _baudrate, (uint16_t) SERIAL_8N1)) {
    return 0;
  }

  ModbusRTUClient.setTimeout(2*1000UL);

  return 1;
}

/**
 * Sets the Arduino_10BASE_T1S_UDP server for communication.
 *
 * This function sets the Arduino_10BASE_T1S_UDP server that the Modbus server will communicate with.
 *
 * @param server A pointer to the Arduino_10BASE_T1S_UDP server.
 */
int ModbusT1SServerClass::poll()
{
  uint8_t request[MODBUS_RTU_MAX_ADU_LENGTH];

  int requestLength = modbus_receive(_mb, request);

  if (requestLength > 0) {
    modbus_reply(_mb, request, requestLength, &_mbMapping);
    return 1;
  }
  return 0;
}

/**
 * Sets the Arduino_10BASE_T1S_UDP server for communication.
 *
 * This function sets the Arduino_10BASE_T1S_UDP server that the Modbus server will communicate with.
 *
 * @param server A pointer to the Arduino_10BASE_T1S_UDP server.
 */
int ModbusT1SServerClass::coilRead(int address)
{
  int res = -1;
  if(_server == nullptr) {
    return res;
  }

  int modbus_id =  udp_rx_buf.at(2) << 8 |  udp_rx_buf.at(3);
  int address_mod =  udp_rx_buf.at(4) << 8 |  udp_rx_buf.at(5);
  if(address != -1) {
    address_mod = address;
  }

  if (ModbusRTUClient.requestFrom(modbus_id, COILS, address_mod, 1)) {
      if (ModbusRTUClient.available()) {
      res = ModbusRTUClient.read();
      udp_rx_buf.push_back((uint8_t)res);
      _server->beginPacket(_last_ip, _last_port);
      _server->write((uint8_t *)udp_rx_buf.data(), udp_rx_buf.size());
      _server->endPacket();
    }
  }
  udp_rx_buf.erase(udp_rx_buf.begin(), udp_rx_buf.end());
  return res;
}

/**
 * Writes a value to a coil on the Modbus server.
 *
 * This function sends a request to write a value to a coil at the specified address
 * on the Modbus server using the provided UDP client.
 *
 * @param address The address of the coil to write to.
 * @param value The value to write to the coil (1 for ON, 0 for OFF).
 * @return int 1 if the write operation is successful, -1 if an error occurs.
 */
int ModbusT1SServerClass::coilWrite(int address, uint8_t value)
{
  int res = -1;
  if(_server == nullptr) {
    return res;
  }

  int modbus_id =  udp_rx_buf.at(2) << 8 |  udp_rx_buf.at(3);
  int address_mod =  udp_rx_buf.at(4) << 8 |  udp_rx_buf.at(5);
  if(address != -1) {
    address_mod = address;
  }
  uint8_t coilValue = int( udp_rx_buf.at(6));

  ModbusRTUClient.beginTransmission(modbus_id, COILS, address_mod, 1);
  res = ModbusRTUClient.write(coilValue);
  if (!ModbusRTUClient.endTransmission()) {
    res = -1;
  }
  udp_rx_buf.erase(udp_rx_buf.begin(), udp_rx_buf.end());
  return res;
}

/**
 * Reads the status of a discrete input from the Modbus server.
 *
 * This function sends a request to read the status of a discrete input at the specified address
 * from the Modbus server using the provided UDP client.
 *
 * @param address The address of the discrete input to read.
 * @return int The status of the discrete input (1 for ON, 0 for OFF) or -1 if an error occurs.
 */
int ModbusT1SServerClass::discreteInputRead(int address) {
  int res = -1;
  if(_server == nullptr) {
    return res;
  }

  int modbus_id =  udp_rx_buf.at(2) << 8 |  udp_rx_buf.at(3);
  int address_mod =  udp_rx_buf.at(4) << 8 |  udp_rx_buf.at(5);
  if(address != -1) {
       address_mod = address;
  }

  if (ModbusRTUClient.requestFrom(modbus_id, DISCRETE_INPUTS, address_mod, 1)) {
    if (ModbusRTUClient.available()) {
      res = ModbusRTUClient.read();
      udp_rx_buf.push_back((uint8_t)res);
      _server->beginPacket(_last_ip, _last_port);
      _server->write((uint8_t *)udp_rx_buf.data(), udp_rx_buf.size());
      _server->endPacket();
    }
  }
  udp_rx_buf.erase(udp_rx_buf.begin(), udp_rx_buf.end());
  return res;
}

/**
 * Reads the value of an input register from the Modbus server.
 *
 * This function sends a request to read the value of an input register at the specified address
 * from the Modbus server using the provided UDP client.
 *
 * @param address The address of the input register to read.
 * @return long The value of the input register or -1 if an error occurs.
 */
long ModbusT1SServerClass::inputRegisterRead(int address)
{
  long res = -1;
  if(_server == nullptr) {
    return res;
  }
  int modbus_id =  udp_rx_buf.at(2) << 8 |  udp_rx_buf.at(3);
  int address_mod =  udp_rx_buf.at(4) << 8 |  udp_rx_buf.at(5);
  if(address != -1) {
    address_mod = address;
  }

  if (ModbusRTUClient.requestFrom(modbus_id, INPUT_REGISTERS, address_mod, 1)) {
    if (ModbusRTUClient.available()) {
      int16_t data = ModbusRTUClient.read();
      uint8_t tx_buf[2] = {(uint8_t)((data & 0xFF00) >> 8), (uint8_t)(data & 0x00FF)};
      std::copy(tx_buf, tx_buf + 2, std::back_inserter(udp_rx_buf));
      _server->beginPacket(_last_ip, _last_port);
      _server->write((uint8_t *)udp_rx_buf.data(), udp_rx_buf.size());
      _server->endPacket();
    }
  }
  udp_rx_buf.erase(udp_rx_buf.begin(), udp_rx_buf.end());
  return res;
}

/**
 * Writes a value to a holding register on the Modbus server.
 *
 * This function sends a request to write a value to a holding register at the specified address
 * on the Modbus server using the provided UDP client.
 *
 * @param address The address of the holding register to write to.
 * @param value The value to write to the holding register.
 * @return int 1 if the write operation is successful, -1 if an error occurs.
 */
long ModbusT1SServerClass::holdingRegisterRead(int address) {
  long res = -1;
  if(_server == nullptr) {
    return res;
  }

  int modbus_id =  udp_rx_buf.at(2) << 8 |  udp_rx_buf.at(3);
  int address_mod =  udp_rx_buf.at(4) << 8 |  udp_rx_buf.at(5);
  if(address != -1) {
    address_mod = address;
  }

  if (ModbusRTUClient.requestFrom(modbus_id, HOLDING_REGISTERS, address_mod, 1)) {
    if (ModbusRTUClient.available()) {
      int16_t data = ModbusRTUClient.read();
      uint8_t tx_buf[2] = {(uint8_t)((data & 0xFF00) >> 8), (uint8_t)(data & 0x00FF)};
      std::copy(tx_buf, tx_buf + 2, std::back_inserter(udp_rx_buf));
      _server->beginPacket(_last_ip, _last_port);
      _server->write((uint8_t *)udp_rx_buf.data(), udp_rx_buf.size());
      _server->endPacket();
    }
  }
  udp_rx_buf.erase(udp_rx_buf.begin(), udp_rx_buf.end());
  return res;
}

/**
 * Writes a value to a holding register on the Modbus server.
 *
 * This function sends a request to write a value to a holding register at the specified address
 * on the Modbus server using the provided UDP client.
 *
 * @param address The address of the holding register to write to.
 * @param value The value to write to the holding register.
 * @return int 1 if the write operation is successful, -1 if an error occurs.
 */
int ModbusT1SServerClass::holdingRegisterWrite(int address) {
  int res = -1;
  if(_server == nullptr) {
    return res;
  }

  int modbus_id =  udp_rx_buf.at(2) << 8 |  udp_rx_buf.at(3);
  int address_mod =  udp_rx_buf.at(4) << 8 |  udp_rx_buf.at(5);
  if(address != -1) {
    address_mod = address;
  }
  uint16_t value =  udp_rx_buf.at(6) << 8 |  udp_rx_buf.at(7);

  ModbusRTUClient.beginTransmission(modbus_id, HOLDING_REGISTERS, address_mod, 1);
  res = ModbusRTUClient.write(value);
  if (!ModbusRTUClient.endTransmission()) {
    res = -1;
  }
  udp_rx_buf.erase(udp_rx_buf.begin(), udp_rx_buf.end());
  return res;
}

/**
 * Parses the Modbus packet received from the server.
 *
 * This function parses the Modbus packet received from the server.
 *
 * @return int The parsed packet or -1 if an error occurs.
 */
int ModbusT1SServerClass::parsePacket() {
  int res = -1;
  if(_server == nullptr) {
    return res;
  }

  int const rx_packet_size = _server->parsePacket();
  if (rx_packet_size)
  {
    uint8_t rx_msg_buf[UDP_RX_MSG_BUF_SIZE] = {0};
    int bytes_read = _server->read(rx_msg_buf, UDP_RX_MSG_BUF_SIZE - 1);
    while (bytes_read != 0) {
      std::copy(rx_msg_buf, rx_msg_buf + bytes_read, std::back_inserter(udp_rx_buf));
      bytes_read = _server->read(rx_msg_buf, UDP_RX_MSG_BUF_SIZE - 1);
    }
    res = udp_rx_buf.at(0) << 8 | udp_rx_buf.at(1);
  }

  _last_ip = _server->remoteIP();
  _last_port = _server->remotePort();
  _server->flush();

  return res;
}

void ModbusT1SServerClass::checkPLCAStatus()
{
  tc6_inst->service();

  static unsigned long prev_beacon_check = 0;
  static unsigned long prev_udp_packet_sent = 0;

  auto const now = millis();

  if ((now - prev_beacon_check) > 1000)
  {
    prev_beacon_check = now;
    if(callback == nullptr)
    {
      if (!tc6_inst->getPlcaStatus(OnPlcaStatus_server)) {
        Serial.println("getPlcaStatus(...) failed");
      }
    } else {
      if (!tc6_inst->getPlcaStatus(callback)) {
        Serial.println("getPlcaStatus(...) failed");
      }
    }
  }
}

void ModbusT1SServerClass::update() {
  /* Services the hardware and the protocol stack.
     Must be called cyclic. The faster the better.
  */
  checkPLCAStatus();
  switch (ModbusT1SServer.parsePacket())
  {
    case UDP_READ_COIL_PORT:
      ModbusT1SServer.coilRead();
      break;

    case UDP_WRITE_COIL_PORT:
      ModbusT1SServer.coilWrite();
      break;

    case UDP_READ_DI_PORT:
      discreteInputRead();
      break;

    case UDP_READ_IR_PORT:
      inputRegisterRead();
      break;

    case UDP_READ_HR_PORT:
      holdingRegisterRead();
      break;

    case UDP_WRITE_HR_PORT:
      holdingRegisterWrite();
      break;

    default:
      break;
  }
}

/**
 * Sets the Arduino_10BASE_T1S_UDP server for communication.
 *
 * This function sets the Arduino_10BASE_T1S_UDP server that the Modbus server will communicate with.
 *
 * @param server A pointer to the Arduino_10BASE_T1S_UDP server.
 */
void ModbusT1SServerClass::setT1SServer(Arduino_10BASE_T1S_UDP * server) {
  _server = server;
}


void ModbusT1SServerClass::setT1SPort(int port) {
  udp_port = port;
}
void ModbusT1SServerClass::setBadrate(int baudrate) {
  _baudrate = baudrate;
}

void ModbusT1SServerClass::setCallback(callback_f cb) {
  if(cb != nullptr) {
    callback = cb;
  }
}

static void OnPlcaStatus_server(bool success, bool plcaStatus)
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

ModbusT1SServerClass ModbusT1SServer;
#endif
