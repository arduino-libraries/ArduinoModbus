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
    return -1;
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

ModbusT1SServerClass ModbusT1SServer;
#endif
