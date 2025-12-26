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
   * @param node_id id of the server
   * @param baudrate Baud rate to use
   * @param config serial config. to use defaults to SERIAL_8N1
   * @param rs485 RS485 object to use
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
   * @return int Returns 1 if the write operation is successful, 0 otherwise.
   */
  int coilWrite(int address = -1);

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

  /**
   * Set the port to use for communication
   *
   * @param port The port to use
   */
  void setT1SPort(int port = 8889);

  /**
   * Update the Modbus server.
   *
   * This function updates the Modbus server.
   */
  void update();

  /**
   * Set the baudrate to use for communication
   *
   * @param baudrate The baudrate to use
   */
  void setBaudrate(int baudrate);

/**
 * Sets the gateway IP address.
 *
 * This function sets the gateway IP address.
 *
 * @param ip The gateway IP address.
 */
  void setGatwayIP(IPAddress ip);

  /**
   * Poll the Modbus server.
   *
   * This function polls the Modbus server.
   *
   * @return int The status of the poll.
   */
  virtual int poll();

  /**
   * Checks the PLCA status and Services the hardware and the protocol stack.
   *
   * This function checks the PLCA status and services the hardware and the
   * protocol stack.
   *
   */
  void checkStatus();

  /**
   * Set the callback function to use for PLCA status check.
   *
   * This function sets the callback function to use for PLCA status check.
   *
   * @param cb The callback function to use.
   */
  void setCallback(callback_f cb = nullptr);

   /**
   * Enables Power Over Ethernet (POE).
   *
   * This function enables Power Over Ethernet (POE) setting the device as power source.
   */
  void enablePOE();

  /**
   * Disables Power Over Ethernet (POE).
   *
   * This function disables Power Over Ethernet (POE) and set the USB as power source.
   */
  void disablePOE();

  /**
   * Set the Modbus RTU client timeout.
   *
   * This function sets the Modbus RTU client timeout.
   *
   *  @param timeout The timeout value in milliseconds.
   */
  void setTimeout(unsigned long timeout);

private:
  long read(int address, int functionCode);
  long write(int address, int functionCode);

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
