/*
  This file is part of the ArduinoModbus library.
  Copyright (c) 2024 Arduino SA. All rights reserved.

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

#include "ModbusT1SClient.h"

size_t const UDP_RX_MSG_BUF_SIZE = 16 + 1;

/**
 * @class ModbusT1SClientClass
 * Class for Modbus T1S Client communication.
 *
 * This class provides functionalities to communicate with a Modbus T1S server.
 */

/**
 * Default constructor for ModbusT1SClientClass.
 *
 * Initializes the Modbus client with a default timeout of 1000 milliseconds.
 */
ModbusT1SClientClass::ModbusT1SClientClass() :
  ModbusClient(1000)
{
}

/**
 * Constructor for ModbusT1SClientClass with RS485 support.
 *
 * Initializes the Modbus client with a default timeout of 1000 milliseconds and sets up RS485 communication.
 *
 * @param rs485 Reference to an RS485Class object for RS485 communication.
 */
ModbusT1SClientClass::ModbusT1SClientClass(RS485Class& rs485) :
  ModbusClient(1000),
  _rs485(&rs485)
{
}

/**
 * Destructor for ModbusT1SClientClass.
 *
 * Cleans up any resources used by the ModbusT1SClientClass.
 */
ModbusT1SClientClass::~ModbusT1SClientClass()
{
}

/**
 * Initializes the Modbus client with the specified RS485 instance, baud rate, and configuration.
 *
 * This function sets up the Modbus client for communication using the specified RS485 instance, baud rate, and configuration.
 *
 * @param rs485 Reference to an RS485Class object for RS485 communication.
 * @param baudrate The baud rate for the Modbus communication.
 * @param config The configuration for the Modbus communication (e.g., parity, stop bits).
 * @return int Returns 1 if initialization is successful, 0 otherwise.
 */
int ModbusT1SClientClass::begin(RS485Class& rs485, unsigned long baudrate, uint16_t config)
{
  _rs485 = &rs485;
  return begin(baudrate, config);
}

int ModbusT1SClientClass::begin(int node_id)
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
  IPAddress server_addr = IPAddress(192, 168,  42, 100);
  _server_ip = server_addr;

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

  if (!_client->begin(udp_port))
  {
    return 0;
  }
  return 1;
}

/**
 * Sets the IP address of the Modbus server.
 *
 * This function sets the IP address of the Modbus server that the client will communicate with.
 *
 * @param server_ip The IP address of the Modbus server.
 */
void ModbusT1SClientClass::setServerIp(IPAddress server_ip)
{
  _server_ip = server_ip;
}

/**
 * Sets the port number of the Modbus server.
 *
 * This function sets the port number of the Modbus server that the client will communicate with.
 *
 * @param server_port The port number of the Modbus server.
 */
void ModbusT1SClientClass::setServerPort(uint16_t server_port)
{
  _server_port = server_port;
}

/**
 * Sets the Modbus ID for the client.
 *
 * This function sets the Modbus ID for the client to use when communicating with the Modbus server.
 *
 * @param id The Modbus ID to use.
 */
void ModbusT1SClientClass::setModbusId(uint16_t id)
{
  _modbus_id = id;
}

/**
 * Sets the timeout for receiving a response from the Modbus server.
 *
 * This function sets the timeout for receiving a response from the Modbus server.
 *
 * @param timeout The timeout value in milliseconds.
 */
void ModbusT1SClientClass::setRxTimeout(unsigned long timeout)
{
  _rx_timeout = timeout;
}
/**
 * Reads the status of a coil from the Modbus server.
 *
 * This function sends a request to read the status of a coil at the specified address
 * from the Modbus server using the provided UDP client and port.
 *
 * @param address The address of the coil to read.
 * @param client A pointer to the Arduino_10BASE_T1S_UDP client used for communication.
 * @param port The port number to use for the communication.
 * @return int The status of the coil (1 for ON, 0 for OFF) or -1 if an error occurs.
 */
int ModbusT1SClientClass::coilRead(int address, Arduino_10BASE_T1S_UDP * client, int port)
{
  int res = -1;
  if(client != nullptr) {
    _client = client;
  }

  int _port = port;
  if(port == -1) {
    _port = UDP_READ_COIL_PORT;
  }

  uint8_t tx_buf[6] = {(uint8_t)(_port & 0xFF00) >> 8, (uint8_t)(_port & 0x00FF),
                                           (uint8_t)((_modbus_id & 0xFF00) >> 8), (uint8_t)_modbus_id,
                                              (uint8_t)((address & 0xFF00) >> 8), (uint8_t)address};
  int tx_packet_size = sizeof(tx_buf);
  write(tx_buf, tx_packet_size, _client);
  unsigned long start = millis();
  while(millis() - start < _rx_timeout) {
    if(read(_client) > 0) {
      if(checkPacket(_port, _modbus_id, address)) {
        res = int(udp_rx_buf.at(6));
        udp_rx_buf.erase(udp_rx_buf.begin(), udp_rx_buf.end());
        break;
      }
      udp_rx_buf.erase(udp_rx_buf.begin(), udp_rx_buf.end());
    }
  }
  return res;
}

/**
 * Reads the status of a coil from the Modbus server with a specified ID.
 *
 * This function sends a request to read the status of a coil at the specified address
 * from the Modbus server with the given ID using the provided UDP client and port.
 *
 * @param id The ID of the Modbus server.
 * @param address The address of the coil to read.
 * @param client A pointer to the Arduino_10BASE_T1S_UDP client used for communication.
 * @param port The port number to use for the communication.
 * @return int The status of the coil (1 for ON, 0 for OFF) or -1 if an error occurs.
 */
int ModbusT1SClientClass::coilRead(int id, int address, Arduino_10BASE_T1S_UDP * client, int port)
{
  int res = -1;
  if(client != nullptr) {
    _client = client;
  }

  int _port = port;
  if(port == -1) {
    _port = UDP_READ_COIL_PORT;
  }

  uint8_t tx_buf[6] = {(uint8_t)(_port & 0xFF00) >> 8, (uint8_t)(_port & 0x00FF),
                                           (uint8_t)((id & 0xFF00) >> 8), (uint8_t)id,
                                              (uint8_t)((address & 0xFF00) >> 8), (uint8_t)address};
  int tx_packet_size = sizeof(tx_buf);
  write(tx_buf, tx_packet_size, _client);
  unsigned long start = millis();
  while(millis() - start < _rx_timeout) {
    if(read(_client)) {
      if(checkPacket(_port, id, address)) {
        res = int(udp_rx_buf.at(6));
        udp_rx_buf.erase(udp_rx_buf.begin(), udp_rx_buf.end());
        break;
      }
      udp_rx_buf.erase(udp_rx_buf.begin(), udp_rx_buf.end());
    }
  }
  return res;
}

/**
 * Writes a value to a coil on the Modbus server.
 *
 * This function sends a request to write a value to a coil at the specified address
 * on the Modbus server using the provided UDP client and port.
 *
 * @param address The address of the coil to write to.
 * @param value The value to write to the coil (1 for ON, 0 for OFF).
 * @param client A pointer to the Arduino_10BASE_T1S_UDP client used for communication.
 * @param port The port number to use for the communication.
 * @return int 1 if the write operation is successful, -1 if an error occurs.
 */
int ModbusT1SClientClass::coilWrite(int address, uint8_t value, Arduino_10BASE_T1S_UDP * client, int port)
{
  if(client != nullptr) {
    _client = client;
  }

  int _port = port;
  if(port == -1) {
    _port = UDP_WRITE_COIL_PORT;
  }

  uint8_t tx_buf[7] = {(uint8_t)(_port & 0xFF00) >> 8, (uint8_t)(_port & 0x00FF),
                                      (uint8_t)((_modbus_id & 0xFF00) >> 8), (uint8_t)_modbus_id,
                                               (uint8_t)((address & 0xFF00) >> 8), (uint8_t)address, value};
  int tx_packet_size = sizeof(tx_buf);
  write(tx_buf, tx_packet_size, _client);
  return 1;
}

/**
 * Writes a value to a coil on the Modbus server with a specified ID.
 *
 * This function sends a request to write a value to a coil at the specified address
 * on the Modbus server with the given ID using the provided UDP client and port.
 *
 * @param id The ID of the Modbus server.
 * @param address The address of the coil to write to.
 * @param value The value to write to the coil (1 for ON, 0 for OFF).
 * @param client A pointer to the Arduino_10BASE_T1S_UDP client used for communication.
 * @param port The port number to use for the communication.
 * @return int 1 if the write operation is successful, -1 if an error occurs.
 */
int ModbusT1SClientClass::coilWrite(int id, int address, uint8_t value, Arduino_10BASE_T1S_UDP * client, int port)
{
  if(client != nullptr) {
    _client = client;
  }


  int _port = port;
  if(port == -1) {
    _port = UDP_WRITE_COIL_PORT;
  }

  uint8_t tx_buf[7] = {(uint8_t)(_port & 0xFF00) >> 8, (uint8_t)(_port & 0x00FF), (uint8_t)((id & 0xFF00) >> 8), (uint8_t)id,
                                          (uint8_t)((address & 0xFF00) >> 8), (uint8_t)address, value};
  int tx_packet_size = sizeof(tx_buf);
  write(tx_buf, tx_packet_size, _client);
  return 1;
}

/**
 * Reads the status of a discrete input from the Modbus server.
 *
 * This function sends a request to read the status of a discrete input at the specified address
 * from the Modbus server using the provided UDP client and port.
 *
 * @param address The address of the discrete input to read.
 * @param client A pointer to the Arduino_10BASE_T1S_UDP client used for communication.
 * @param port The port number to use for the communication.
 * @return int The status of the discrete input (1 for ON, 0 for OFF) or -1 if an error occurs.
 */
int ModbusT1SClientClass::discreteInputRead(int address, Arduino_10BASE_T1S_UDP * client, int port)
{
  int res = -1;
  if(client != nullptr) {
    _client = client;
  }

  int _port = port;
  if(port == -1) {
    _port = UDP_READ_DI_PORT;
  }

  uint8_t tx_buf[6] = {(uint8_t)(_port & 0xFF00) >> 8, (uint8_t)(_port & 0x00FF),
                                           (uint8_t)((_modbus_id & 0xFF00) >> 8), (uint8_t)_modbus_id,
                                              (uint8_t)((address & 0xFF00) >> 8), (uint8_t)address};
  int tx_packet_size = sizeof(tx_buf);
  write(tx_buf, tx_packet_size, _client);
  unsigned long start = millis();
  while(millis() - start < _rx_timeout) {
    if(read(_client)) {
      if(checkPacket(_port, _modbus_id, address)) {
        res = int(udp_rx_buf.at(6));
        udp_rx_buf.erase(udp_rx_buf.begin(), udp_rx_buf.end());
        break;
      }
      udp_rx_buf.erase(udp_rx_buf.begin(), udp_rx_buf.end());
    }
  }
  return res;
}

/**
 * Reads the status of a discrete input from the Modbus server with a specified ID.
 *
 * This function sends a request to read the status of a discrete input at the specified address
 * from the Modbus server with the given ID using the provided UDP client and port.
 *
 * @param id The ID of the Modbus server.
 * @param address The address of the discrete input to read.
 * @param client A pointer to the Arduino_10BASE_T1S_UDP client used for communication.
 * @param port The port number to use for the communication.
 * @return int The status of the discrete input (1 for ON, 0 for OFF) or -1 if an error occurs.
 */
int ModbusT1SClientClass::discreteInputRead(int id, int address, Arduino_10BASE_T1S_UDP * client, int port)
{
  int res = -1;
  if(client != nullptr) {
    _client = client;
  }

  int _port = port;
  if(port == -1) {
    _port = UDP_READ_DI_PORT;
  }

  uint8_t tx_buf[6] = {(uint8_t)(_port & 0xFF00) >> 8, (uint8_t)(_port & 0x00FF),
                                           (uint8_t)((id & 0xFF00) >> 8), (uint8_t)id,
                                              (uint8_t)((address & 0xFF00) >> 8), (uint8_t)address};
  int tx_packet_size = sizeof(tx_buf);
  write(tx_buf, tx_packet_size, _client);
  unsigned long start = millis();
  while(millis() - start < _rx_timeout) {
    if(read(_client)) {
      if(checkPacket(_port, id, address)) {
        res = int(udp_rx_buf.at(6));
        udp_rx_buf.erase(udp_rx_buf.begin(), udp_rx_buf.end());
        break;
      }
      udp_rx_buf.erase(udp_rx_buf.begin(), udp_rx_buf.end());
    }
  }
  return res;
}

/**
 * Reads the value of an input register from the Modbus server.
 *
 * This function sends a request to read the value of an input register at the specified address
 * from the Modbus server using the provided UDP client and port.
 *
 * @param address The address of the input register to read.
 * @param client A pointer to the Arduino_10BASE_T1S_UDP client used for communication.
 * @param port The port number to use for the communication.
 * @return long The value of the input register or -1 if an error occurs.
 */
long ModbusT1SClientClass::inputRegisterRead(int address, Arduino_10BASE_T1S_UDP * client, int port)
{
  long res = -1;
  if(client != nullptr) {
    _client = client;
  }

  int _port = port;
  if(port == -1) {
    _port = UDP_READ_IR_PORT;
  }

  uint8_t tx_buf[6] = {(uint8_t)(_port & 0xFF00) >> 8, (uint8_t)(_port & 0x00FF),
                                           (uint8_t)((_modbus_id & 0xFF00) >> 8), (uint8_t)_modbus_id,
                                              (uint8_t)((address & 0xFF00) >> 8), (uint8_t)address};
  int tx_packet_size = sizeof(tx_buf);
  write(tx_buf, tx_packet_size, _client);
  unsigned long start = millis();
  while(millis() - start < _rx_timeout) {
    if(read(_client)) {
      if(checkPacket(_port, _modbus_id, address)) {
        res = int(udp_rx_buf.at(6) << 8 | udp_rx_buf.at(7));
        udp_rx_buf.erase(udp_rx_buf.begin(), udp_rx_buf.end());
        break;
      }
      udp_rx_buf.erase(udp_rx_buf.begin(), udp_rx_buf.end());
    }
  }
  return res;
}

/**
 * Reads the value of an input register from the Modbus server with a specified ID.
 *
 * This function sends a request to read the value of an input register at the specified address
 * from the Modbus server with the given ID using the provided UDP client and port.
 *
 * @param id The ID of the Modbus server.
 * @param address The address of the input register to read.
 * @param client A pointer to the Arduino_10BASE_T1S_UDP client used for communication.
 * @param port The port number to use for the communication.
 * @return long The value of the input register or -1 if an error occurs.
 */
long ModbusT1SClientClass::inputRegisterRead(int id, int address, Arduino_10BASE_T1S_UDP * client, int port)
{
  long res = -1;
  if(client != nullptr) {
    _client = client;
  }

  int _port = port;
  if(port == -1) {
    _port = UDP_READ_IR_PORT;
  }

  uint8_t tx_buf[6] = {(uint8_t)(_port & 0xFF00) >> 8, (uint8_t)(_port & 0x00FF),
                                           (uint8_t)((id & 0xFF00) >> 8), (uint8_t)id,
                                              (uint8_t)((address & 0xFF00) >> 8), (uint8_t)address};
  int tx_packet_size = sizeof(tx_buf);
  write(tx_buf, tx_packet_size, _client);
  unsigned long start = millis();
  while(millis() - start < _rx_timeout) {
    if(read(_client) > 0) {
      if(checkPacket(_port, id, address)) {
        res = int(udp_rx_buf.at(6) << 8 | udp_rx_buf.at(7));
        udp_rx_buf.erase(udp_rx_buf.begin(), udp_rx_buf.end());
        break;
      }
      udp_rx_buf.erase(udp_rx_buf.begin(), udp_rx_buf.end());
    }
  }
  return res;
}

/**
 * Reads a value from a holding register on a Modbus server.
 *
 * This function sends a request to read a single holding register on a Modbus server
 * using the specified client and port.
 *
 * @param address The address of the holding register to read from.
 * @param client A pointer to an Arduino_10BASE_T1S_UDP client used for communication.
 * @param port The port number to use for the communication.
 * @return Returns the value of the holding register on success, or -1 if the client is null.
 */
long ModbusT1SClientClass::holdingRegisterRead(int address, Arduino_10BASE_T1S_UDP * client, int port)
{
  long res = -1;
  if(client != nullptr) {
    _client = client;
  }
  
  int _port = port;
  if(port == -1) {
    _port = UDP_READ_HR_PORT;
  }
  uint8_t tx_buf[6] = {(uint8_t)(_port & 0xFF00) >> 8, (uint8_t)(_port & 0x00FF),
                                           (uint8_t)((_modbus_id & 0xFF00) >> 8), (uint8_t)_modbus_id,
                                              (uint8_t)((address & 0xFF00) >> 8), (uint8_t)address};
  int tx_packet_size = sizeof(tx_buf);
  write(tx_buf, tx_packet_size, client);
  unsigned long start = millis();
  while(millis() - start < _rx_timeout) {
    int const rx_packet_size = _client->parsePacket();
    if(read(_client)) {
      if(checkPacket(_port, _modbus_id, address)) {
        res = int(udp_rx_buf.at(6) << 8 | udp_rx_buf.at(7));
        udp_rx_buf.erase(udp_rx_buf.begin(), udp_rx_buf.end());
        break;
      }
      udp_rx_buf.erase(udp_rx_buf.begin(), udp_rx_buf.end());
    }
  }
  return res;
}

/**
 * Writes a value to a holding register on a Modbus server.
 *
 * This function sends a request to write a single holding register on a Modbus server
 * using the specified client and port.
 *
 * @param id The ID of the Modbus server.
 * @param address The address of the holding register to write to.
 * @param value The value to write to the holding register.
 * @param client A pointer to an Arduino_10BASE_T1S_UDP client used for communication.
 * @param port The port number to use for the communication.
 * @return Returns 1 on success, or -1 if the client is null.
 */
long ModbusT1SClientClass::holdingRegisterRead(int id, int address, Arduino_10BASE_T1S_UDP * client, int port)
{
  long res = -1;
  if(client != nullptr) {
    _client = client;
  }

  int _port = port;
  if(port == -1) {
    _port = UDP_READ_HR_PORT;
  }

  uint8_t tx_buf[6] = {(uint8_t)(_port & 0xFF00) >> 8, (uint8_t)(_port & 0x00FF),
                                           (uint8_t)((id & 0xFF00) >> 8), (uint8_t)(id & 0x00FF),
                                              (uint8_t)((address & 0xFF00) >> 8), (uint8_t)address};

  int tx_packet_size = sizeof(tx_buf);
  write(tx_buf, tx_packet_size, _client);
  unsigned long start = millis();
  while(millis() - start < _rx_timeout) {
    if(read(_client)) {
      if(checkPacket(_port, id, address)) {
        res = int(udp_rx_buf.at(6) << 8 | udp_rx_buf.at(7));
        udp_rx_buf.erase(udp_rx_buf.begin(), udp_rx_buf.end());
        break;
      }
      udp_rx_buf.erase(udp_rx_buf.begin(), udp_rx_buf.end());
    }
  }
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
 * @param client A pointer to the Arduino_10BASE_T1S_UDP client used for communication.
 * @return int 1 if the write operation is successful, -1 if an error occurs.
 */
int ModbusT1SClientClass::holdingRegisterWrite(int address, uint16_t value, Arduino_10BASE_T1S_UDP * client, int port)
{
  if(client != nullptr) {
    _client = client;
  }

    int _port = port;
  if(port == -1) {
    _port = UDP_WRITE_HR_PORT;
  }

  uint8_t tx_buf[8] = {(uint8_t)(_port & 0xFF00) >> 8, (uint8_t)(_port & 0x00FF),
                                           (uint8_t)((_modbus_id & 0xFF00) >> 8), (uint8_t)_modbus_id,
                                              (uint8_t)((address & 0xFF00) >> 8), (uint8_t)address, (uint8_t)((value & 0xFF00) >> 8), (uint8_t)(value & 0x00FF)};
  int tx_packet_size = sizeof(tx_buf);
  write(tx_buf, tx_packet_size, _client);
  return 1;
}

/**
 * Writes a value to a holding register on a Modbus server.
 *
 * This function sends a request to write a single holding register on a Modbus server
 * using the specified client and port.
 *
 * @param id The ID of the Modbus server.
 * @param address The address of the holding register to write to.
 * @param value The value to write to the holding register.
 * @param client A pointer to an Arduino_10BASE_T1S_UDP client used for communication.
 * @param port The port number to use for the communication.
 * @return Returns 1 on success, or -1 if the client is null.
 */
int ModbusT1SClientClass::holdingRegisterWrite(int id, int address, uint16_t value, Arduino_10BASE_T1S_UDP * client, int port)
{
  if(client != nullptr) {
    _client = client;
  }

  int _port = port;
  if(port == -1) {
    _port = UDP_WRITE_HR_PORT;
  }

  uint8_t tx_buf[8] = {(uint8_t)(_port & 0xFF00) >> 8, (uint8_t)(_port & 0x00FF),
                              (uint8_t)((id & 0xFF00) >> 8), (uint8_t)id,
                                (uint8_t)((address & 0xFF00) >> 8), (uint8_t)address,
                                    (uint8_t)((value & 0xFF00) >> 8), (uint8_t)(value & 0x00FF)};

  int tx_packet_size = sizeof(tx_buf);
  write(tx_buf, tx_packet_size, _client);
  return 1;
}

void ModbusT1SClientClass::write(uint8_t * buf, int len, Arduino_10BASE_T1S_UDP * client)
{
  _client->beginPacket(_server_ip, _server_port);
  _client->write((const uint8_t *)buf, len);
  _client->endPacket();
}

int ModbusT1SClientClass::read(Arduino_10BASE_T1S_UDP * client)
{
  int const rx_packet_size = _client->parsePacket();
  if (rx_packet_size)
  {
    uint8_t rx_msg_buf[rx_packet_size] = {0};
    int bytes_read = _client->read(rx_msg_buf, rx_packet_size - 1);
    while (bytes_read != 0) {
      std::copy(rx_msg_buf, rx_msg_buf + bytes_read, std::back_inserter(udp_rx_buf));
      bytes_read = _client->read(rx_msg_buf, UDP_RX_MSG_BUF_SIZE - 1);
    }
    _client->flush();
    return udp_rx_buf.size();
  }
  return 0;
}

bool ModbusT1SClientClass::checkPacket(int port, uint16_t id, uint16_t address)
{
  int port_rec = udp_rx_buf.at(0) << 8 | udp_rx_buf.at(1);
  uint16_t id_rcv = udp_rx_buf.at(2) << 8 | udp_rx_buf.at(3);
  uint16_t add_rcv = udp_rx_buf.at(4) << 8 | udp_rx_buf.at(5);
  if(port_rec == port && add_rcv == address && id_rcv == id) {
    return true;
  }
  return false;
}
void ModbusT1SClientClass::setT1SClient(Arduino_10BASE_T1S_UDP * client)
{
  _client = client;
}

void ModbusT1SClientClass::setT1SPort(int port)
{
  udp_port = port;
}

void ModbusT1SClientClass::checkPLCAStatus()
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
      if (!tc6_inst->getPlcaStatus(OnPlcaStatus_client)) {
        Serial.println("getPlcaStatus(...) failed");
      }
    } else {
      if (!tc6_inst->getPlcaStatus(callback)) {
        Serial.println("getPlcaStatus(...) failed");
      }
    }
  }
}

void ModbusT1SClientClass::setCallback(callback_f cb) {
  if(cb != nullptr) {
    callback = cb;
  }
}

static void OnPlcaStatus_client(bool success, bool plcaStatus)
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
ModbusT1SClientClass ModbusT1SClient;
#endif
