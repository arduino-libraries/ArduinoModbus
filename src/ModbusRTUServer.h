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


#ifndef _MODBUS_RTU_SERVER_H_INCLUDED
#define _MODBUS_RTU_SERVER_H_INCLUDED


#ifndef RS485_SER_CONF_TYPE
#define RS485_SER_CONF_TYPE uint16_t
#endif

#include "ModbusServer.h"
#include <ArduinoRS485.h>

class ModbusRTUServerClass : public ModbusServer {
public:
  ModbusRTUServerClass();
  ModbusRTUServerClass(RS485Class& rs485);
  virtual ~ModbusRTUServerClass();

  /**
   * Start the Modbus RTU server with the specified parameters
   *
   * @param id (slave) id of the server
   * @param baudrate Baud rate to use
   * @param config serial config. to use defaults to SERIAL_8N1
   *
   * Return 1 on success, 0 on failure
   */
  int begin(int id, unsigned long baudrate, RS485_SER_CONF_TYPE config = SERIAL_8N1);
  int begin(RS485Class& rs485, int id, unsigned long baudrate, RS485_SER_CONF_TYPE config = SERIAL_8N1);

  /**
   * Poll interface for requests
   */
  virtual int poll();

private:
  RS485Class* _rs485 = &RS485;
};

extern ModbusRTUServerClass ModbusRTUServer;

#endif
