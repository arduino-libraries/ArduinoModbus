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

#ifndef _MODBUS_T1S_CLIENT_H_INCLUDED
#define _MODBUS_T1S_CLIENT_H_INCLUDED
#if (defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNOR4_MINIMA))


#include "ModbusClient.h"
#include <ArduinoRS485.h>
#include <Arduino_10BASE_T1S.h>
#include "ModbusT1SCommon.h"
#include <SPI.h>

#define RX_TIMEOUT 1000

using callback_f = void (*)(bool, bool);
class ModbusT1SClientClass : public ModbusClient {
public:
  ModbusT1SClientClass();
  ModbusT1SClientClass(RS485Class& rs485);
  virtual ~ModbusT1SClientClass();

  /**
   * Start the Modbus T1S client with the specified parameters
   *
   * @param baudrate Baud rate to use
   * @param config serial config. to use defaults to SERIAL_8N1
   *
   * Return 1 on success, 0 on failure
   */
  int begin(int node_id);
/**
 * Sets the IP address of the Modbus server.
 *
 * This function sets the IP address of the Modbus server that the client will communicate with.
 *
 * @param server_ip The IP address of the Modbus server. Default is IPAddress(0, 0, 0, 0).
 */
void setServerIp(IPAddress server_ip = IPAddress(0, 0, 0, 0));

/**
 * Sets the port number of the Modbus server.
 *
 * This function sets the port number of the Modbus server that the client will communicate with.
 *
 * @param server_port The port number of the Modbus server. Default is 8889.
 */
void setServerPort(uint16_t server_port = 8889);

/**
 * Sets the Modbus ID of the client.
 *
 * This function sets the Modbus ID that the client will use for communication.
 *
 * @param id The Modbus ID.
 */
void setModbusId(uint16_t id);

/**
 * Reads the status of a coil from the Modbus server.
 *
 * This function sends a request to read the status of a coil at the specified address
 * from the Modbus server using the provided UDP client and port.
 *
 * @param address The address of the coil to read.
 * @param client A pointer to the Arduino_10BASE_T1S_UDP client used for communication. Default is nullptr.
 * @param port The port number of the Modbus server. Default is 0.
 * @return int The status of the coil (1 for ON, 0 for OFF) or -1 if an error occurs.
 */
int coilRead(int address, Arduino_10BASE_T1S_UDP * client = nullptr);

/**
 * Reads the status of a coil from the Modbus server with a specified ID.
 *
 * This function sends a request to read the status of a coil at the specified address
 * from the Modbus server with the given ID using the provided UDP client and port.
 *
 * @param id The ID of the Modbus server.
 * @param address The address of the coil to read.
 * @param client A pointer to the Arduino_10BASE_T1S_UDP client used for communication. Default is nullptr.
 * @param port The port number of the Modbus server. Default is 0.
 * @return int The status of the coil (1 for ON, 0 for OFF) or -1 if an error occurs.
 */
int coilRead(int id, int address, Arduino_10BASE_T1S_UDP * client = nullptr);

/**
 * Writes a value to a coil on the Modbus server.
 *
 * This function sends a request to write a value to a coil at the specified address
 * on the Modbus server using the provided UDP client and port.
 *
 * @param address The address of the coil to write to.
 * @param value The value to write to the coil (1 for ON, 0 for OFF).
 * @param client A pointer to the Arduino_10BASE_T1S_UDP client used for communication. Default is nullptr.
 * @param port The port number of the Modbus server. Default is 0.
 * @return int Returns 1 if the write operation is successful, 0 otherwise.
 */
int coilWrite(int address, uint16_t value, Arduino_10BASE_T1S_UDP * client = nullptr);

/**
 * Writes a value to a coil on the Modbus server with a specified ID.
 *
 * This function sends a request to write a value to a coil at the specified address
 * on the Modbus server with the given ID using the provided UDP client and port.
 *
 * @param id The ID of the Modbus server.
 * @param address The address of the coil to write to.
 * @param value The value to write to the coil (1 for ON, 0 for OFF).
 * @param client A pointer to the Arduino_10BASE_T1S_UDP client used for communication. Default is nullptr.
 * @param port The port number of the Modbus server. Default is 0.
 * @return int Returns 1 if the write operation is successful, 0 otherwise.
 */
int coilWrite(int id, int address, uint16_t value, Arduino_10BASE_T1S_UDP * client = nullptr);

/**
 * Reads the status of a discrete input from the Modbus server.
 *
 * This function sends a request to read the status of a discrete input at the specified address
 * from the Modbus server using the provided UDP client and port.
 *
 * @param address The address of the discrete input to read.
 * @param client A pointer to the Arduino_10BASE_T1S_UDP client used for communication. Default is nullptr.
 * @param port The port number of the Modbus server. Default is 0.
 * @return int The status of the discrete input (1 for ON, 0 for OFF) or -1 if an error occurs.
 */
int discreteInputRead(int address, Arduino_10BASE_T1S_UDP * client = nullptr);

/**
 * Reads the status of a discrete input from the Modbus server with a specified ID.
 *
 * This function sends a request to read the status of a discrete input at the specified address
 * from the Modbus server with the given ID using the provided UDP client and port.
 *
 * @param id The ID of the Modbus server.
 * @param address The address of the discrete input to read.
 * @param client A pointer to the Arduino_10BASE_T1S_UDP client used for communication. Default is nullptr.
 * @param port The port number of the Modbus server. Default is 0.
 * @return int The status of the discrete input (1 for ON, 0 for OFF) or -1 if an error occurs.
 */
int discreteInputRead(int id, int address, Arduino_10BASE_T1S_UDP * client = nullptr);

/**
 * Reads the value of an input register from the Modbus server.
 *
 * This function sends a request to read the value of an input register at the specified address
 * from the Modbus server using the provided UDP client and port.
 *
 * @param address The address of the input register to read.
 * @param client A pointer to the Arduino_10BASE_T1S_UDP client used for communication. Default is nullptr.
 * @param port The port number of the Modbus server. Default is 0.
 * @return long The value of the input register or -1 if an error occurs.
 */
long inputRegisterRead(int address, Arduino_10BASE_T1S_UDP * client = nullptr);

/**
 * Reads the value of an input register from the Modbus server with a specified ID.
 *
 * This function sends a request to read the value of an input register at the specified address
 * from the Modbus server with the given ID using the provided UDP client and port.
 *
 * @param id The ID of the Modbus server.
 * @param address The address of the input register to read.
 * @param client A pointer to the Arduino_10BASE_T1S_UDP client used for communication. Default is nullptr.
 * @param port The port number of the Modbus server. Default is 0.
 * @return long The value of the input register or -1 if an error occurs.
 */
long inputRegisterRead(int id, int address, Arduino_10BASE_T1S_UDP * client = nullptr);

/**
 * Writes a value to a holding register on the Modbus server.
 *
 * This function sends a request to write a value to a holding register at the specified address
 * on the Modbus server using the provided UDP client and port.
 *
 * @param address The address of the holding register to write to.
 * @param value The value to write to the holding register.
 * @param client A pointer to the Arduino_10BASE_T1S_UDP client used for communication. Default is nullptr.
 * @param port The port number of the Modbus server. Default is 0.
 * @return int Returns 1 if the write operation is successful, 0 otherwise.
 */
int holdingRegisterWrite(int address, uint16_t value, Arduino_10BASE_T1S_UDP * client = nullptr);

/**
 * Writes a value to a holding register on the Modbus server with a specified ID.
 *
 * This function sends a request to write a value to a holding register at the specified address
 * on the Modbus server with the given ID using the provided UDP client and port.
 *
 * @param id The ID of the Modbus server.
 * @param address The address of the holding register to write to.
 * @param value The value to write to the holding register.
 * @param client A pointer to the Arduino_10BASE_T1S_UDP client used for communication. Default is nullptr.
 * @param port The port number of the Modbus server. Default is 0.
 * @return int Returns 1 if the write operation is successful, 0 otherwise.
 */
int holdingRegisterWrite(int id, int address, uint16_t value, Arduino_10BASE_T1S_UDP * client = nullptr);

/**
 * Reads the value of a holding register from the Modbus server.
 *
 * This function sends a request to read the value of a holding register at the specified address
 * from the Modbus server using the provided UDP client and port.
 *
 * @param address The address of the holding register to read.
 * @param client A pointer to the Arduino_10BASE_T1S_UDP client used for communication. Default is nullptr.
 * @param port The port number of the Modbus server. Default is 0.
 * @return long The value of the holding register or -1 if an error occurs.
 */
long holdingRegisterRead(int address, Arduino_10BASE_T1S_UDP * client = nullptr);

/**
 * Reads the value of a holding register from the Modbus server with a specified ID.
 *
 * This function sends a request to read the value of a holding register at the specified address
 * from the Modbus server with the given ID using the provided UDP client and port.
 *
 * @param id The ID of the Modbus server.
 * @param address The address of the holding register to read.
 * @param client A pointer to the Arduino_10BASE_T1S_UDP client used for communication. Default is nullptr.
 * @param port The port number of the Modbus server. Default is 0.
 * @return long The value of the holding register or -1 if an error occurs.
 */
long holdingRegisterRead(int id, int address, Arduino_10BASE_T1S_UDP * client = nullptr);
void setT1SClient(Arduino_10BASE_T1S_UDP & client);
void setRxTimeout(unsigned long timeout = RX_TIMEOUT);
void setT1SPort(int port = 8889);
void update();
void setCallback(callback_f cb = nullptr);
void enablePOE();
void disablePOE();


private:
  long receive(int id, int address, Arduino_10BASE_T1S_UDP * client, int functionCode);
  int  send(int id, int address, uint16_t value, Arduino_10BASE_T1S_UDP * client, int functionCode);
  void write(uint8_t * buf, int len, Arduino_10BASE_T1S_UDP * client);
  int read(Arduino_10BASE_T1S_UDP * client);
  bool checkPacket(int port, uint16_t id, uint16_t address);

private:
  IPAddress _gateway = IPAddress(0, 0, 0, 0);
  unsigned long _rx_timeout = RX_TIMEOUT;
  callback_f callback = nullptr;
  IPAddress _server_ip = IPAddress(0, 0, 0, 0);
  uint16_t _server_port = 8889;
  uint16_t _modbus_id = 0;
  uint8_t udp_rx_buf[8] = {0};

  RS485Class* _rs485 = &RS485;
  Arduino_10BASE_T1S_UDP * _client = nullptr;
  int _node_id = 1;
  int udp_port = 0;
};

extern ModbusT1SClientClass ModbusT1SClient;
#endif
#endif
