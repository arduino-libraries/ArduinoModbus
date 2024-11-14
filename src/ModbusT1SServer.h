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

#ifndef _MODBUS_T1S_SERVER_H_INCLUDED
#define _MODBUS_T1S_SERVER_H_INCLUDED

#ifndef __AVR__
#include "ModbusServer.h"
#include <ArduinoRS485.h>
#include <Arduino_10BASE_T1S.h>
#include "ModbusRTUClient.h"
#include "ModbusT1SCommon.h"
#include <SPI.h>

using callback_f = void (*)(bool, bool);
class ModbusT1SServerClass : public ModbusServer {
public:
  /**
   * Default constructor for ModbusT1SServerClass.
   *
   * Initializes the Modbus server with RS485 communication.
   */
  ModbusT1SServerClass();
  ModbusT1SServerClass(RS485Class& rs485);

  /**
   * Destructor for ModbusT1SServerClass.
   */
  virtual ~ModbusT1SServerClass();

  /**
   * Start the Modbus T1S server with the specified parameters
   *
   * @param id (slave) id of the server
   * @param baudrate Baud rate to use
   * @param config serial config. to use defaults to SERIAL_8N1
   *
   * Return 1 on success, 0 on failure
   */

  int begin(int node_id, unsigned long baudrate, uint16_t config = SERIAL_8N1, RS485Class& rs485 = RS485);
  /**
   * Reads a coil from the Modbus server.
   *
   * This function sends a request to read a coil at the specified address
   * from the Modbus server using the provided UDP client.
   *
   * @param address The address of the coil to read.
   * @return int The value of the coil or -1 if an error occurs.
   */
  int coilRead(int address = -1);

  /**
   * Writes a value to a coil on the Modbus server.
   *
   * This function sends a request to write a value to a coil at the specified address
   * on the Modbus server using the provided UDP client.
   *
   * @param address The address of the coil to write to.
   * @param value The value to write to the coil (1 for ON, 0 for OFF).
   * @return int Returns 1 if the write operation is successful, 0 otherwise.
   */
  int coilWrite(int address = -1, uint8_t value = 255);

  /**
   * Reads the status of a discrete input from the Modbus server.
   *
   * This function sends a request to read the status of a discrete input at the specified address
   * from the Modbus server using the provided UDP client.
   *
   * @param address The address of the discrete input to read.
   * @return int The status of the discrete input (1 for ON, 0 for OFF) or -1 if an error occurs.
   */
  int discreteInputRead(int address = -1);

  /**
   * Reads the value of an input register from the Modbus server.
   *
   * This function sends a request to read the value of an input register at the specified address
   * from the Modbus server using the provided UDP client.
   *
   * @param address The address of the input register to read.
   * @return long The value of the input register or -1 if an error occurs.
   */
  long inputRegisterRead(int address = -1);

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
  long holdingRegisterRead(int address = -1);

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
  int holdingRegisterWrite(int address = -1);

  /**
   * Parses the Modbus packet received from the server.
   *
   * This function parses the Modbus packet received from the server.
   *
   * @return int The parsed packet or -1 if an error occurs.
   */
  int parsePacket();

  /**
   * Set the T1S server to use for communication
   *
   * @param server The T1S server to use
   */
  void setT1SServer(Arduino_10BASE_T1S_UDP & server);

  void setT1SPort(int port = 8889);

  void update();

  void setBaudrate(int baudrate);


  /**
   * Poll interface for requests
   */
  void setGatwayIP(IPAddress ip);
  virtual int poll();
  void checkStatus();
  void setCallback(callback_f cb = nullptr);
  long read(int address, int functionCode);
  long write(int address, int functionCode);
  void enablePOE();
  void disablePOE();
  void setTimeout(unsigned long timeout);

private:
  IPAddress _gateway = IPAddress(0, 0, 0, 0);
  uint8_t udp_rx_buf[8] = {0};
  callback_f callback = nullptr;
  RS485Class* _rs485 = &RS485;
  Arduino_10BASE_T1S_UDP * _server = nullptr;
  IPAddress _last_ip;
  uint16_t _last_port;
  int _baudrate = 9600;
  int _node_id = 1;
  int udp_port = 0;
};

extern ModbusT1SServerClass ModbusT1SServer;

#endif
#endif
