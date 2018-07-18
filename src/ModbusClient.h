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

#ifndef _MODBUS_CLIENT_H_INCLUDED
#define _MODBUS_CLIENT_H_INCLUDED

extern "C" {
  #include "libmodbus/modbus.h"
}

#include <Arduino.h>

class ModbusClient {

public:
  /**
   * Perform a "Read Coils" operation for the specified address for a single
   * coil.
   *
   * @param id (slave) id of target, defaults to 0x00 if not specified
   * @param address address to use for operation
   *
   * @return coil value on success, -1 on failure.
   */
  int readCoil(int address);
  int readCoil(int id, int address);

  /**
   * Perform a "Read Coils" operation for the specified address and number of
   * coils.
   *
   * @param id (slave) id of target, defaults to 0x00 if not specified
   * @param address start address to use for operation
   * @param values array of bytes to store coil values
   * @param nb number of coils to read
   *
   * @return 1 success, 0 on failure.
   */
  int readCoils(int address, uint8_t values[], int nb);
  int readCoils(int id, int address, uint8_t values[], int nb);

  /**
   * Perform a "Read Discrete Inputs" operation for the specified address for a
   * single discrete input.
   *
   * @param id (slave) id of target, defaults to 0x00 if not specified
   * @param address address to use for operation
   *
   * @return discrete input value on success, -1 on failure.
   */
  int readDiscreteInput(int address);
  int readDiscreteInput(int id, int address);

  /**
   * Perform a "Read Discrete Inputs" operation for the specified address and 
   * number of inputs.
   *
   * @param id (slave) id of target, defaults to 0x00 if not specified
   * @param address start address to use for operation
   * @param values array of bytes to store discrete input values
   * @param nb number of discrete inputs to read
   *
   * @return 1 success, 0 on failure.
   */
  int readDiscreteInputs(int address, uint8_t values[], int nb);
  int readDiscreteInputs(int id, int address, uint8_t values[], int nb);

  /**
   * Perform a "Read Holding Registers" operation for a single holding
   * register.
   *
   * @param id (slave) id of target, defaults to 0x00 if not specified
   * @param address start address to use for operation
   *
   * @return holiding register value on success, -1 on failure.
   */
  long readHoldingRegister(int address);
  long readHoldingRegister(int id, int address);

  /**
   * Perform a "Read Holding Registers" operation for the specified address and
   * number of holding registers.
   *
   * @param id (slave) id of target, defaults to 0x00 if not specified
   * @param address start address to use for operation
   * @param values array of words to store holding register values
   * @param nb number of holding registers to read
   *
   * @return 1 success, 0 on failure.
   */
  int readHoldingRegisters(int address, uint16_t values[], int nb);
  int readHoldingRegisters(int id, int address, uint16_t values[], int nb);

  /**
   * Perform a "Read Input Registers" operation for a single input
   * register.
   *
   * @param id (slave) id of target, defaults to 0x00 if not specified
   * @param address address to use for operation
   *
   * @return input register value on success, -1 on failure.
   */
  long readInputRegister(int address);
  long readInputRegister(int id, int address);

  /**
   * Perform a "Read Input Registers" operation for the specified address and
   * number of input registers.
   *
   * @param id (slave) id of target, defaults to 0x00 if not specified
   * @param address start address to use for operation
   * @param values array of words to store input register values
   * @param nb number of holding input to read
   *
   * @return 1 success, 0 on failure.
   */
  int readInputRegisters(int address, uint16_t values[], int nb);
  int readInputRegisters(int id, int address, uint16_t values[], int nb);

  /**
   * Perform a "Write Single Coil" operation for the specified address and
   * value.
   *
   * @param id (slave) id of target, defaults to 0x00 if not specified
   * @param address address to use for operation
   * @param value coil value to write
   *
   * @return 1 on success, 0 on failure.
   */
  int writeCoil(int address, uint8_t value);
  int writeCoil(int id, int address, uint8_t value);

  /**
   * Perform a "Write Multiple Coils" operation for the specified address and
   * values.
   *
   * @param id (slave) id of target, defaults to 0x00 if not specified
   * @param address start address to use for operation
   * @param values array of coil values to write
   * @param nb number of coil values to write
   *
   * @return 1 on success, 0 on failure.
   */
  int writeCoils(int address, const uint8_t values[], int nb);
  int writeCoils(int id, int address, const uint8_t values[], int nb);

  /**
   * Perform a "Write Single Holding Register" operation for the specified
   * address and value.
   *
   * @param id (slave) id of target, defaults to 0x00 if not specified
   * @param address address to use for operation
   * @param value holding register value to write
   *
   * @return 1 on success, 0 on failure.
   */
  int writeHoldingRegister(int address, uint16_t value);
  int writeHoldingRegister(int id, int address, uint16_t value);

  /**
   * Perform a "Write Multiple Holding Registers" operation for the specified
   * address and values.
   *
   * @param id (slave) id of target, defaults to 0x00 if not specified
   * @param address start address to use for operation
   * @param values array of holding register values to write
   * @param nb number of holding register values to write
   *
   * @return 1 on success, 0 on failure.
   */
  int writeHoldingRegisters(int address, const uint16_t values[], int nb);
  int writeHoldingRegisters(int id, int address, const uint16_t values[], int nb);

  /**
   * Perform a "Mask Write Registers" operation for the specified
   * address, AND mask and OR mask.
   *
   * @param id (slave) id of target, defaults to 0x00 if not specified
   * @param address address to use for operation
   * @param andMask AND mask to use for operation
   * @param orMask OR mask to use for operation
   *
   * @return 1 on success, 0 on failure.
   */
  int maskWriteRegister(int address, uint16_t andMask, uint16_t orMask);
  int maskWriteRegister(int id, int address, uint16_t andMask, uint16_t orMask);

  /**
   * Perform a "Read/Write Registers" operation.
   *
   * @param id (slave) id of target, defaults to 0x00 if not specified
   * @param writeAddress write address to use for operation
   * @param writeValues array of words to write
   * @param writeNb number of registers to write
   * @param readAddress read address to use for operation
   * @param readValues array of words to store register values
   * @param readNb number of registers to read
   *
   * @return 1 on success, 0 on failure.
   */
  int writeAndReadRegisters(int writeAddress, const uint16_t writeValues[], int writeNb, int readAddress, uint16_t readValues[], int readNb);
  int writeAndReadRegisters(int id, int writeAddress, const uint16_t writeValues[], int writeNb, int readAddress, uint16_t readValues[], int readNb);

  /**
   * Read the last error reason as a string
   *
   * @return Last error reason as a C string
   */
  const char* lastError();

  /**
   * Stop the client and clean up
   */
  void end();

protected:
  ModbusClient();
  virtual ~ModbusClient();

  int begin(modbus_t* _mb, int defaultId);

private:
  modbus_t* _mb;
  int _defaultId;
};

#endif
