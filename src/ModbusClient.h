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

#define COILS             0
#define DISCRETE_INPUTS   1
#define HOLDING_REGISTERS 2
#define INPUT_REGISTERS   3

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
  int coilRead(int address);
  int coilRead(int id, int address);

  /**
   * Perform a "Read Discrete Inputs" operation for the specified address for a
   * single discrete input.
   *
   * @param id (slave) id of target, defaults to 0x00 if not specified
   * @param address address to use for operation
   *
   * @return discrete input value on success, -1 on failure.
   */
  int discreteInputRead(int address);
  int discreteInputRead(int id, int address);

  /**
   * Perform a "Read Holding Registers" operation for a single holding
   * register.
   *
   * @param id (slave) id of target, defaults to 0x00 if not specified
   * @param address start address to use for operation
   *
   * @return holding register value on success, -1 on failure.
   */
  long holdingRegisterRead(int address);
  long holdingRegisterRead(int id, int address);

  /**
   * Perform a "Read Input Registers" operation for a single input
   * register.
   *
   * @param id (slave) id of target, defaults to 0x00 if not specified
   * @param address address to use for operation
   *
   * @return input register value on success, -1 on failure.
   */
  long inputRegisterRead(int address);
  long inputRegisterRead(int id, int address);

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
  int coilWrite(int address, uint8_t value);
  int coilWrite(int id, int address, uint8_t value);

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
  int holdingRegisterWrite(int address, uint16_t value);
  int holdingRegisterWrite(int id, int address, uint16_t value);

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
  int registerMaskWrite(int address, uint16_t andMask, uint16_t orMask);
  int registerMaskWrite(int id, int address, uint16_t andMask, uint16_t orMask);

  /**
   * Begin the process of a writing multiple coils or holding registers.
   *
   * Use write(value) to set the values you want to send, and endTransmission()
   * to send request on the wire.
   *
   * @param id (slave) id of target, defaults to 0x00 if not specified
   * @param type type of write to perform, either COILS or HOLDING_REGISTERS
   * @param address start address to use for operation
   * @param nb number of values to write
   *
   * @return 1 on success, 0 on failure
   */
  int beginTransmission(int type, int address, int nb);
  int beginTransmission(int id, int type, int address, int nb);

  /**
   * Set the values of a write operation started by beginTransmission(...).
   *
   * @param value value to write
   *
   * @return 1 on success, 0 on failure
   */
  int write(unsigned int value);

  /**
   * End the process of a writing multiple coils or holding registers.
   *
   * @return 1 on success, 0 on failure
   */
  int endTransmission();

  /**
   * Read multiple coils, discrete inputs, holding registers, or input 
   * register values.
   *
   * Use available() and read() to process the read values.
   *
   * @param id (slave) id of target, defaults to 0x00 if not specified
   * @param type type of read to perform, either COILS, DISCRETE_INPUTS, 
   *             HOLDING_REGISTERS, or INPUT_REGISTERS
   * @param address start address to use for operation
   * @param nb number of values to read
   *
   * @return 0 on failure, number of values read on success
   */
  int requestFrom(int type, int address, int nb);
  int requestFrom(int id, int type, int address,int nb);

  /**
   * Query the number of values available to read after calling
   * requestFrom(...)
   *
   * @return number of values available for reading use read()
   */
  int available();

  /**
   * Read a value after calling requestFrom(...)
   *
   * @return -1 on failure, value on success
   */
  long read();

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

  /**
   * Set response timeout (in milliseconds)
   */
  void setTimeout(unsigned long ms);

protected:
  ModbusClient(unsigned long defaultTimeout);
  virtual ~ModbusClient();

  int begin(modbus_t* _mb, int defaultId);

private:
  modbus_t* _mb;
  unsigned long _timeout;
  int _defaultId;

  bool _transmissionBegun;
  int _id;
  int _type;
  int _address;
  int _nb;

  void* _values;
  int _available;
  int _read;
  int _availableForWrite;
  int _written;
};

#endif
