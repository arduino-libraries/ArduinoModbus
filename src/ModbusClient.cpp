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

#include <errno.h>

#include "ModbusClient.h"

ModbusClient::ModbusClient() :
  _mb(NULL)
{
}

ModbusClient::~ModbusClient()
{
  if (_mb != NULL) {
    modbus_free(_mb);
  }
}

int ModbusClient::begin(modbus_t* mb)
{
  end();

  _mb = mb;
  if (_mb == NULL) {
    return 0;
  }

  if (modbus_connect(_mb) != 0) {
    modbus_free(_mb);

    _mb = NULL;
    return 0;
  }

  return 1;
}

void ModbusClient::end()
{
  if (_mb != NULL) {
    modbus_close(_mb);
    modbus_free(_mb);

    _mb = NULL;
  }
}

void ModbusClient::setId(int id)
{
  modbus_set_slave(_mb, id);
}

int ModbusClient::readCoil(int address)
{
  uint8_t value;

  if (readCoils(address, &value, 1) < 0) {
    return -1;
  }

  return value;
}

int ModbusClient::readCoils(int address, uint8_t values[], int nb)
{
  if (modbus_read_bits(_mb, address, nb, values) < 0) {
    return 0;
  }

  return 1;
}

int ModbusClient::readDiscreteInput(int address)
{
  uint8_t value;

  if (!readDiscreteInputs(address, &value, 1)) {
    return -1;
  }

  return value;
}

int ModbusClient::readDiscreteInputs(int address, uint8_t values[], int nb)
{
  if (modbus_read_input_bits(_mb, address, nb, values) < 0) {
    return 0;
  }

  return 1;
}

long ModbusClient::readHoldingRegister(int address)
{
  uint16_t value;

  if (!readHoldingRegisters(address, &value, 1)) {
    return -1;
  }

  return value;
}

int ModbusClient::readHoldingRegisters(int address, uint16_t values[], int nb)
{
  if (modbus_read_registers(_mb, address, nb, values) < 0) {
    return 0;
  }

  return 1;
}

long ModbusClient::readInputRegister(int address)
{
  uint16_t value;

  if (!readInputRegisters(address, &value, 1)) {
    return -1;
  }

  return value;
}

int ModbusClient::readInputRegisters(int address, uint16_t values[], int nb)
{
  if (modbus_read_input_registers(_mb, address, nb, values) < 0) {
    return 0;
  }

  return 1;
}

int ModbusClient::writeCoil(int address, uint8_t value)
{
  if (modbus_write_bit(_mb, address, value) < 0) {
    return 0;
  }

  return 1;
}

int ModbusClient::writeCoils(int address, const uint8_t values[], int nb)
{
  if (modbus_write_bits(_mb, address, nb, values) < 0) {
    return 0;
  }

  return 1;
}

int ModbusClient::writeRegister(int address, uint16_t value)
{
  if (modbus_write_register(_mb, address, value) < 0) {
    return 0;
  }

  return 1;
}

int ModbusClient::writeRegisters(int address, const uint16_t values[], int nb)
{
  if (modbus_write_registers(_mb, address, nb, values) < 0) {
    return 0;
  }

  return 1;
}

int ModbusClient::maskWriteRegister(int address, uint16_t andMask, uint16_t orMask)
{
  if (modbus_mask_write_register(_mb, address, andMask, orMask) < 0) {
    return 0;
  }

  return 1;
}

int ModbusClient::writeAndReadRegisters(int writeAddress, const uint16_t writeValues[], int writeNb, int readAddress, uint16_t readValues[], int readNb)
{
  if (modbus_write_and_read_registers(_mb, writeAddress, writeNb, writeValues, readAddress, readNb, readValues) < 0) {
    return 0;
  }

  return 1;
}

const char* ModbusClient::lastError()
{
  if (errno == 0) {
    return NULL;
  }

  return modbus_strerror(errno); 
}
