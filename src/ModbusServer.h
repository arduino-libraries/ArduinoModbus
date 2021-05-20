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

#ifndef _MODBUS_SERVER_H_INCLUDED
#define _MODBUS_SERVER_H_INCLUDED

#include <Arduino.h>

extern "C" {
  #include "libmodbus/modbus.h"
}

class ModbusServer {

public:
  /**
   * Configure the servers coils.
   *
   * @param startAddress start address of coils
   * @param nb number of coils to configure
   *
   * @return 0 on success, 1 on failure
   */
  int configureCoils(int startAddress, int nb);

  /**
   * Configure the servers discrete inputs.
   *
   * @param startAddress start address of discrete inputs
   * @param nb number of discrete inputs to configure
   *
   * @return 0 on success, 1 on failure
   */
  int configureDiscreteInputs(int startAddress, int nb);

  /**
   * Configure the servers holding registers.
   *
   * @param startAddress start address of holding registers
   * @param nb number of holding registers to configure
   *
   * @return 0 on success, 1 on failure
   */
  int configureHoldingRegisters(int startAddress, int nb);

  /**
   * Configure the servers input registers.
   *
   * @param startAddress start address of input registers
   * @param nb number of input registers to configure
   *
   * @return 0 on success, 1 on failure
   */
  int configureInputRegisters(int startAddress, int nb);

  // same as ModbusClient.h
  int coilRead(int address);
  int discreteInputRead(int address);
  long holdingRegisterRead(int address);
  long inputRegisterRead(int address);
  int coilWrite(int address, uint8_t value);
  int holdingRegisterWrite(int address, uint16_t value);
  int registerMaskWrite(int address, uint16_t andMask, uint16_t orMask);

  /**
   * Write the value of the server's Discrete Input for the specified address
   * and value.
   *
   * @param address address to use for operation
   * @param value discrete input value to write
   *
   * @return 1 on success, 0 on failure.
   */
  int discreteInputWrite(int address, uint8_t value);

  /**
   * Write values to the server's Discrete Inputs for the specified address
   * and values.
   *
   * @param address address to use for operation
   * @param values array of discrete inputs values to write
   * @param nb number of discrete inputs to write
   *
   * @return 1 on success, 0 on failure.
   */
  int writeDiscreteInputs(int address, uint8_t values[], int nb);

  /**
   * Write the value of the server's Input Register for the specified address
   * and value.
   *
   * @param address address to use for operation
   * @param value input register value to write
   *
   * @return 1 on success, 0 on failure.
   */
  int inputRegisterWrite(int address, uint16_t value);

  /**
   * Write values to the server's Input Registers for the specified address
   * and values.
   *
   * @param address address to use for operation
   * @param values array of input registers values to write
   * @param nb number of input registers to write
   *
   * @return 1 on success, 0 on failure.
   */
  int writeInputRegisters(int address, uint16_t values[], int nb);

  /**
   * Poll for requests
   * 
   * @return 1 on request, 0 on no request.
   */
  virtual int poll() = 0;

  /**
   * Stop the server
   */
  void end();

protected:
  ModbusServer();
  virtual ~ModbusServer();

  int begin(modbus_t* _mb, int id);

protected:
  modbus_t* _mb;
  modbus_mapping_t _mbMapping;
};

#endif
