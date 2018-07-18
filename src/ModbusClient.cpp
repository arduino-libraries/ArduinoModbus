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
  _defaultId(0x00),
  _transmissionBegun(false),
  _values(NULL),
  _available(0),
  _read(0),
  _availableForWrite(0),
  _written(0)
{
}

ModbusClient::~ModbusClient()
{
  if (_values != NULL) {
    free(_values);
  }

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

  _transmissionBegun = false;
  _available = 0;
  _read = 0;
  _availableForWrite = 0;
  _written = 0;

  modbus_set_error_recovery(_mb, MODBUS_ERROR_RECOVERY_PROTOCOL);

  return 1;
}

void ModbusClient::end()
{
  if (_values != NULL) {
    free(_values);

    _values = NULL;
  }

  if (_mb != NULL) {
    modbus_close(_mb);
    modbus_free(_mb);

    _mb = NULL;
  }
}

int ModbusClient::coilRead(int address)
{
  return coilRead(_defaultId, address);
}

int ModbusClient::coilRead(int id, int address)
{
  uint8_t value;

  modbus_set_slave(_mb, id);
  
  if (modbus_read_bits(_mb, address, 1, &value) < 0) {
    return -1;
  }

  return value;
}

int ModbusClient::discreteInputRead(int address)
{
  return discreteInputRead(_defaultId, address);
}

int ModbusClient::discreteInputRead(int id, int address)
{
  uint8_t value;

  modbus_set_slave(_mb, id);
  
  if (modbus_read_input_bits(_mb, address, 1, &value) < 0) {
    return -1;
  }

  return value;
}

long ModbusClient::holdingRegisterRead(int address)
{
  return holdingRegisterRead(_defaultId, address);
}

long ModbusClient::holdingRegisterRead(int id, int address)
{
  uint16_t value;

  modbus_set_slave(_mb, id);
  
  if (modbus_read_registers(_mb, address, 1, &value) < 0) {
    return -1;
  }

  return value;
}

long ModbusClient::inputRegisterRead(int address)
{
  return inputRegisterRead(_defaultId, address);
}

long ModbusClient::inputRegisterRead(int id, int address)
{
  uint16_t value;

  modbus_set_slave(_mb, id);
  
  if (modbus_read_input_registers(_mb, address, 1, &value) < 0) {
    return -1;
  }

  return value;
}

int ModbusClient::coilWrite(int address, uint8_t value)
{
  return coilWrite(_defaultId, address, value);
}

int ModbusClient::coilWrite(int id, int address, uint8_t value)
{
  modbus_set_slave(_mb, id);

  if (modbus_write_bit(_mb, address, value) < 0) {
    return 0;
  }

  return 1;
}

int ModbusClient::holdingRegisterWrite(int address, uint16_t value)
{
  return holdingRegisterWrite(_defaultId, address, value);
}

int ModbusClient::holdingRegisterWrite(int id, int address, uint16_t value)
{
  modbus_set_slave(_mb, id);

  if (modbus_write_register(_mb, address, value) < 0) {
    return 0;
  }

  return 1;
}

int ModbusClient::registerMaskWrite(int address, uint16_t andMask, uint16_t orMask)
{
  return registerMaskWrite(_defaultId, address, andMask, orMask);
}

int ModbusClient::registerMaskWrite(int id, int address, uint16_t andMask, uint16_t orMask)
{
  modbus_set_slave(_mb, id);

  if (modbus_mask_write_register(_mb, address, andMask, orMask) < 0) {
    return 0;
  }

  return 1;
}

int ModbusClient::beginTransmission(int type, int address, int nb)
{
  return beginTransmission(_defaultId, type, address, nb);
}

int ModbusClient::beginTransmission(int id, int type, int address, int nb)
{
  if ((type != COILS && type != HOLDING_REGISTERS) || nb < 1) {
    errno = EINVAL;

    return 0;
  }

  int valueSize = (type == COILS) ? sizeof(uint8_t) : sizeof(uint16_t);

  _values = realloc(_values, nb * valueSize);

  if (_values == NULL) {
    errno = ENOMEM;

    return 0;
  }

  memset(_values, 0x00, nb * valueSize);

  _transmissionBegun = true;
  _id = id;
  _type = type;
  _address = address;
  _nb = nb;

  _available = 0;
  _read = 0;
  _availableForWrite = nb;
  _written = 0;

  return 1;
}

int ModbusClient::write(unsigned int value)
{
  if (!_transmissionBegun || _availableForWrite <= 0) {
    return 0;
  }

  switch (_type) {
    case COILS:
      ((uint8_t*)_values)[_written++] = value;
      _availableForWrite--;
      return 1;

    case HOLDING_REGISTERS:
      ((uint16_t*)_values)[_written++] = value;
      _availableForWrite--;
      return 1;

    default:
      return 0;
  }

  return 1;
}

int ModbusClient::endTransmission()
{
  if (!_transmissionBegun) {
    return 0;
  }

  int result = -1;

  modbus_set_slave(_mb, _id);

  switch (_type) {
    case COILS:
      result = modbus_write_bits(_mb, _address, _nb, (const uint8_t*)_values);
      break;

    case HOLDING_REGISTERS:
      result = modbus_write_registers(_mb, _address, _nb, (const uint16_t*)_values);
      break;

    default:
      return 0;
  }

  _transmissionBegun = false;
  _available = 0;
  _read = 0;
  _availableForWrite = 0;
  _written = 0;

  return (result < 0) ? 0 : 1;
}

int ModbusClient::requestFrom(int type, int address, int nb)
{
  return requestFrom(_defaultId, type, address, nb);
}

int ModbusClient::requestFrom(int id, int type, int address, int nb)
{
  if ((type != COILS && type != DISCRETE_INPUTS && type != HOLDING_REGISTERS && type != INPUT_REGISTERS) 
      || (nb < 1)) {
    errno = EINVAL;

    return 0;
  }

  int valueSize = (type == COILS || type == DISCRETE_INPUTS) ? sizeof(uint8_t) : sizeof(uint16_t);

  _values = realloc(_values, nb * valueSize);

  if (_values == NULL) {
    errno = ENOMEM;

    return 0;
  }

  int result = -1;

  modbus_set_slave(_mb, id);

  switch (type) {
    case COILS:
      result = modbus_read_bits(_mb, address, nb, (uint8_t*)_values);
      break;

    case DISCRETE_INPUTS:
      result = modbus_read_input_bits(_mb, address, nb, (uint8_t*)_values);
      break;

    case HOLDING_REGISTERS:
      result = modbus_read_registers(_mb, address, nb, (uint16_t*)_values);
      break;

    case INPUT_REGISTERS:
      result = modbus_read_input_registers(_mb, address, nb, (uint16_t*)_values);
      break;

    default:
      break; 
  }

  if (result == -1) {
    return 0;
  }

  _transmissionBegun = false;
  _type = type;
  _available = nb;
  _read = 0;
  _availableForWrite = 0;
  _written = 0;

  return nb;
}

int ModbusClient::available()
{
  return _available;
}

long ModbusClient::read()
{
  if (_available <= 0) {
    return -1;
  }

  long result = -1;

  switch (_type) {
    case COILS:
    case DISCRETE_INPUTS:
      result = ((uint8_t*)_values)[_read];
      break;

    case HOLDING_REGISTERS:
    case INPUT_REGISTERS:
      result = ((uint16_t*)_values)[_read];
      break;

    default:
      break; 
  }

  if (result != -1) {
    _available--;
    _read++;
  }

  return result;
}

const char* ModbusClient::lastError()
{
  if (errno == 0) {
    return NULL;
  }

  return modbus_strerror(errno); 
}
