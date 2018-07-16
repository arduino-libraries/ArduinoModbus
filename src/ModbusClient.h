/*
  This file is part of the Modbus library.
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
  void setId(int id);

  int readCoil(int address);
  int readCoils(int address, uint8_t values[], int nb);
  int readDiscreteInput(int address);
  int readDiscreteInputs(int address, uint8_t values[], int nb);
  long readHoldingRegister(int address);
  int readHoldingRegisters(int address, uint16_t values[], int nb);
  long readInputRegister(int address);
  int readInputRegisters(int address, uint16_t values[], int nb);

  int writeCoil(int address, uint8_t value);
  int writeCoils(int address, const uint8_t values[], int nb);
  int writeRegister(int address, uint16_t value);
  int writeRegisters(int address, const uint16_t values[], int nb);
  int maskWriteRegister(int address, uint16_t andMask, uint16_t orMask);

  int writeAndReadRegisters(int writeAddress, const uint16_t writeValues[], int writeNb, int readAddress, uint16_t readValues[], int readNb);

  const char* lastError();

  void end();

protected:
  ModbusClient();
  virtual ~ModbusClient();

  int begin(modbus_t* _mb);

private:
  modbus_t* _mb;
};

#endif
