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

#ifndef _MODBUS_TCP_CLIENT_H_INCLUDED
#define _MODBUS_TCP_CLIENT_H_INCLUDED

#include <Client.h>
#include <IPAddress.h>

#include "ModbusClient.h"

class ModbusTCPClient : public ModbusClient {
public:
  /**
   * ModbusTCPClient constructor
   *
   * @param client Client to use for TCP connection
   */
  ModbusTCPClient(Client& client);
  virtual ~ModbusTCPClient();

  /**
   * Start the Modbus TCP client with the specified parameters
   *
   * @param ip IP Address of the Modbus server
   * @param port TCP port number of Modbus server, defaults to 502
   *
   * @return 1 on success, 0 on failure
   */
  int begin(IPAddress ip, uint16_t port = 502);

  /**
   * Query connection status.
   *
   * @return 1 if connected, 0 if not connected
   */
  int connected();

  /**
   * Disconnect the client.
   */
  void stop();

private:
  Client* _client;
};

#endif
