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
  _mb(NULL),
  _defaultId(0x00)
{
}

ModbusClient::~ModbusClient()
{
  if (_mb != NULL) {
    modbus_free(_mb);
  }
}

int ModbusClient::begin(modbus_t* mb, int defaultId)
{
  end();

  _mb = mb;
  _defaultId = defaultId;
  if (_mb == NULL) {
    return 0;
  }

  if (modbus_connect(_mb) != 0) {
    modbus_free(_mb);

    _mb = NULL;
    return 0;
  }

  modbus_set_error_recovery(_mb, MODBUS_ERROR_RECOVERY_PROTOCOL);

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

int ModbusClient::readCoil(int address)
{
  return readCoil(_defaultId, address);
}

int ModbusClient::readCoil(int id, int address)
{
  uint8_t value;

  if (readCoils(id, address, &value, 1) < 0) {
    return -1;
  }

  return value;
}

int ModbusClient::readCoils(int address, uint8_t values[], int nb)
{
  return readCoils(_defaultId, address, values, nb);
}

int ModbusClient::readCoils(int id, int address, uint8_t values[], int nb)
{
  modbus_set_slave(_mb, id);

  if (modbus_read_bits(_mb, address, nb, values) < 0) {
    return 0;
  }

  return 1;
}

int ModbusClient::readDiscreteInput(int address)
{
  return readDiscreteInput(_defaultId, address);
}

int ModbusClient::readDiscreteInput(int id, int address)
{
  uint8_t value;

  if (!readDiscreteInputs(id, address, &value, 1)) {
    return -1;
  }

  return value;
}

int ModbusClient::readDiscreteInputs(int address, uint8_t values[], int nb)
{
  return readDiscreteInputs(_defaultId, address, values, nb);
}

int ModbusClient::readDiscreteInputs(int id, int address, uint8_t values[], int nb)
{
  modbus_set_slave(_mb, id);

  if (modbus_read_input_bits(_mb, address, nb, values) < 0) {
    return 0;
  }

  return 1;
}

long ModbusClient::readHoldingRegister(int address)
{
  return readHoldingRegister(_defaultId, address);
}

long ModbusClient::readHoldingRegister(int id, int address)
{
  uint16_t value;

  if (!readHoldingRegisters(id, address, &value, 1)) {
    return -1;
  }

  return value;
}

int ModbusClient::readHoldingRegisters(int address, uint16_t values[], int nb)
{
  return readHoldingRegisters(_defaultId, address, values, nb);
}

int ModbusClient::readHoldingRegisters(int id, int address, uint16_t values[], int nb)
{
  modbus_set_slave(_mb, id);

  if (modbus_read_registers(_mb, address, nb, values) < 0) {
    return 0;
  }

  return 1;
}

long ModbusClient::readInputRegister(int address)
{
  return readInputRegister(_defaultId, address);
}

long ModbusClient::readInputRegister(int id, int address)
{
  uint16_t value;

  if (!readInputRegisters(id, address, &value, 1)) {
    return -1;
  }

  return value;
}

int ModbusClient::readInputRegisters(int address, uint16_t values[], int nb)
{
  return readInputRegisters(_defaultId, address, values, nb);
}

int ModbusClient::readInputRegisters(int id, int address, uint16_t values[], int nb)
{
  modbus_set_slave(_mb, id);

  if (modbus_read_input_registers(_mb, address, nb, values) < 0) {
    return 0;
  }

  return 1;
}

int ModbusClient::writeCoil(int address, uint8_t value)
{
  return writeCoil(_defaultId, address, value);
}

int ModbusClient::writeCoil(int id, int address, uint8_t value)
{
  modbus_set_slave(_mb, id);

  if (modbus_write_bit(_mb, address, value) < 0) {
    return 0;
  }

  return 1;
}

int ModbusClient::writeCoils(int address, const uint8_t values[], int nb)
{
  return writeCoils(_defaultId, address, values, nb);
}

int ModbusClient::writeCoils(int id, int address, const uint8_t values[], int nb)
{
  modbus_set_slave(_mb, id);

  if (modbus_write_bits(_mb, address, nb, values) < 0) {
    return 0;
  }

  return 1;
}

int ModbusClient::writeHoldingRegister(int address, uint16_t value)
{
  return writeHoldingRegister(_defaultId, address, value);
}

int ModbusClient::writeHoldingRegister(int id, int address, uint16_t value)
{
  modbus_set_slave(_mb, id);

  if (modbus_write_register(_mb, address, value) < 0) {
    return 0;
  }

  return 1;
}

int ModbusClient::writeHoldingRegisters(int address, const uint16_t values[], int nb)
{
  return writeHoldingRegisters(_defaultId, address, values, nb);
}

int ModbusClient::writeHoldingRegisters(int id, int address, const uint16_t values[], int nb)
{
  modbus_set_slave(_mb, id);

  if (modbus_write_registers(_mb, address, nb, values) < 0) {
    return 0;
  }

  return 1;
}

int ModbusClient::maskWriteRegister(int address, uint16_t andMask, uint16_t orMask)
{
  return maskWriteRegister(_defaultId, address, andMask, orMask);
}

int ModbusClient::maskWriteRegister(int id, int address, uint16_t andMask, uint16_t orMask)
{
  modbus_set_slave(_mb, id);

  if (modbus_mask_write_register(_mb, address, andMask, orMask) < 0) {
    return 0;
  }

  return 1;
}

int ModbusClient::writeAndReadRegisters(int writeAddress, const uint16_t writeValues[], int writeNb, int readAddress, uint16_t readValues[], int readNb)
{
  return writeAndReadRegisters(_defaultId, writeAddress, writeValues, writeNb, readAddress, readValues, readNb);
}

int ModbusClient::writeAndReadRegisters(int id, int writeAddress, const uint16_t writeValues[], int writeNb, int readAddress, uint16_t readValues[], int readNb)
{
  modbus_set_slave(_mb, id);

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
