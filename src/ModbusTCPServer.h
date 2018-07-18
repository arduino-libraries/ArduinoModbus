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

#ifndef _MODBUS_TCP_SERVER_H_INCLUDED
#define _MODBUS_TCP_SERVER_H_INCLUDED

#include <Client.h>

#include "ModbusServer.h"

class ModbusTCPServer : public ModbusServer {
public:
  ModbusTCPServer();
  virtual ~ModbusTCPServer();

  /**
   * Start the Modbus TCP server with the specified parameters
   *
   * @param id (slave) id of the server, defaults to 0xff (TCP)
   *
   * Return 1 on success, 0 on failure
   */
  int begin(int id = 0xff);

  /**
   * Accept client connection
   *
   * @param client client to accept
   */
  void accept(Client& client);

  /**
   * Poll accepted client for requests
   */
  virtual void poll();

private:
  Client* _client;
};

#endif
