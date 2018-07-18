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

#include "ModbusServer.h"

ModbusServer::ModbusServer() :
  _mb(NULL)
{
  memset(&_mbMapping, 0x00, sizeof(_mbMapping));
}

ModbusServer::~ModbusServer()
{
  if (_mbMapping.tab_bits != NULL) {
    free(_mbMapping.tab_bits);
  }

  if (_mbMapping.tab_input_bits != NULL) {
    free(_mbMapping.tab_input_bits);
  }

  if (_mbMapping.tab_input_registers != NULL) {
    free(_mbMapping.tab_input_registers);
  }

  if (_mbMapping.tab_registers != NULL) {
    free(_mbMapping.tab_registers);
  }

  if (_mb != NULL) {
    modbus_free(_mb);
  }
}

int ModbusServer::configureCoils(int startAddress, int nb)
{
  if (startAddress < 0 || nb < 1) {
    errno = EINVAL;

    return -1;
  }

  size_t s = sizeof(_mbMapping.tab_bits[0]) * nb;

  _mbMapping.tab_bits = (uint8_t*)realloc(_mbMapping.tab_bits, s);

  if (_mbMapping.tab_bits == NULL) {
    _mbMapping.start_bits = 0;
    _mbMapping.nb_bits = 0;

    return 0;
  }

  memset(_mbMapping.tab_bits, 0x00, s);
  _mbMapping.start_bits = startAddress;
  _mbMapping.nb_bits = nb;

  return 1;
}

int ModbusServer::configureDiscreteInputs(int startAddress, int nb)
{
  if (startAddress < 0 || nb < 1) {
    errno = EINVAL;

    return -1;
  }

  size_t s = sizeof(_mbMapping.tab_input_bits[0]) * nb;

  _mbMapping.tab_input_bits = (uint8_t*)realloc(_mbMapping.tab_input_bits, s);

  if (_mbMapping.tab_input_bits == NULL) {
    _mbMapping.start_input_bits = 0;
    _mbMapping.nb_input_bits = 0;

    return 0;
  }

  memset(_mbMapping.tab_input_bits, 0x00, s);
  _mbMapping.start_input_bits = startAddress;
  _mbMapping.nb_input_bits = nb;

  return 1;
}

int ModbusServer::configureHoldingRegisters(int startAddress, int nb)
{
  if (startAddress < 0 || nb < 1) {
    errno = EINVAL;

    return -1;
  }

  size_t s = sizeof(_mbMapping.tab_registers[0]) * nb;

  _mbMapping.tab_registers = (uint16_t*)realloc(_mbMapping.tab_registers, s);

  if (_mbMapping.tab_registers == NULL) {
    _mbMapping.start_registers = 0;
    _mbMapping.nb_registers = 0;

    return 0;
  }

  memset(_mbMapping.tab_registers, 0x00, s);
  _mbMapping.start_registers = startAddress;
  _mbMapping.nb_registers = nb;

  return 1;
}

int ModbusServer::configureInputRegisters(int startAddress, int nb)
{
  if (startAddress < 0 || nb < 1) {
    errno = EINVAL;

    return -1;
  }

  size_t s = sizeof(_mbMapping.tab_input_registers[0]) * nb;

  _mbMapping.tab_input_registers = (uint16_t*)realloc(_mbMapping.tab_input_registers, s);

  if (_mbMapping.tab_input_registers == NULL) {
    _mbMapping.start_input_registers = 0;
    _mbMapping.nb_input_registers = 0;

    return 0;
  }

  memset(_mbMapping.tab_input_registers, 0x00, s);
  _mbMapping.start_input_registers = startAddress;
  _mbMapping.nb_input_registers = nb;

  return 1;
}

int ModbusServer::coilRead(int address)
{
  if (_mbMapping.start_bits > address || 
      (_mbMapping.start_bits + _mbMapping.nb_bits) < (address + 1)) {
    errno = EMBXILADD;

    return -1;
  }

  return _mbMapping.tab_bits[address - _mbMapping.start_bits];
}

int ModbusServer::discreteInputRead(int address)
{
  if (_mbMapping.start_input_bits > address || 
      (_mbMapping.start_input_bits + _mbMapping.nb_input_bits) < (address + 1)) {
    errno = EMBXILADD;

    return -1;
  }

  return _mbMapping.tab_input_bits[address - _mbMapping.start_input_bits];
}

long ModbusServer::holdingRegisterRead(int address)
{
  if (_mbMapping.start_registers > address ||
      (_mbMapping.start_registers + _mbMapping.nb_registers) < (address + 1)) {
    errno = EMBXILADD;

    return -1;
  }

  return _mbMapping.tab_registers[address - _mbMapping.start_registers];
}

long ModbusServer::inputRegisterRead(int address)
{
  if (_mbMapping.start_input_registers > address || 
      (_mbMapping.start_input_registers + _mbMapping.nb_input_registers) < (address + 1)) {
    errno = EMBXILADD;

    return -1;
  }

 return _mbMapping.tab_input_registers[address - _mbMapping.start_input_registers];
}

int ModbusServer::coilWrite(int address, uint8_t value)
{
  if (_mbMapping.start_bits > address ||
      (_mbMapping.start_bits + _mbMapping.nb_bits) < (address + 1)) {
    errno = EMBXILADD;

    return 0;
  }

  _mbMapping.tab_bits[address - _mbMapping.start_bits] = value;

  return 1;
}

int ModbusServer::holdingRegisterWrite(int address, uint16_t value)
{
  if (_mbMapping.start_registers > address || 
      (_mbMapping.start_registers + _mbMapping.nb_registers) < (address + 1)) {
    errno = EMBXILADD;

    return 0;
  }

  _mbMapping.tab_registers[address - _mbMapping.start_registers] = value;

  return 1;
}

int ModbusServer::registerMaskWrite(int address, uint16_t andMask, uint16_t orMask)
{
  long value = holdingRegisterRead(address);

  if (value < 0) {
    return 0;
  }

  value &= andMask;
  value |= orMask;

  if (!holdingRegisterWrite(address, value)) {
    return 0;
  }

  return 1;
}

int ModbusServer::discreteInputWrite(int address, uint8_t value)
{
  if (_mbMapping.start_input_bits > address || 
      (_mbMapping.start_input_bits + _mbMapping.nb_input_bits) < (address + 1)) {
    errno = EMBXILADD;

    return 0;
  }

  _mbMapping.tab_input_bits[address - _mbMapping.start_input_bits] = value;

  return 1;
}

int ModbusServer::inputRegisterWrite(int address, uint16_t value)
{
  if (_mbMapping.start_input_registers > address || 
      (_mbMapping.start_input_registers + _mbMapping.nb_input_registers) < (address + 1)) {
    errno = EMBXILADD;

    return 0;
  }

  _mbMapping.tab_input_registers[address - _mbMapping.start_input_registers] = value;

  return 1;
}

int ModbusServer::begin(modbus_t* mb, int id)
{
  end();

  _mb = mb;
  if (_mb == NULL) {
    return 0;
  }

  modbus_set_slave(_mb, id);

  return 1;
}

void ModbusServer::end()
{
  if (_mbMapping.tab_bits != NULL) {
    free(_mbMapping.tab_bits);
  }

  if (_mbMapping.tab_input_bits != NULL) {
    free(_mbMapping.tab_input_bits);
  }

  if (_mbMapping.tab_input_registers != NULL) {
    free(_mbMapping.tab_input_registers);
  }

  if (_mbMapping.tab_registers != NULL) {
    free(_mbMapping.tab_registers);
  }

  memset(&_mbMapping, 0x00, sizeof(_mbMapping));

  if (_mb != NULL) {
    modbus_close(_mb);
    modbus_free(_mb);

    _mb = NULL;
  }
}
