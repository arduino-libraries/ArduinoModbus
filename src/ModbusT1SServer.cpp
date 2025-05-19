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

extern "C" {
#include "libmodbus/modbus.h"
#include "libmodbus/modbus-rtu.h"
}

#include "ModbusT1SServer.h"
#ifndef __AVR__

/**
 * Default constructor for ModbusT1SServerClass.
 *
 * Initializes the Modbus server with RS485 communication.
 */
ModbusT1SServerClass::ModbusT1SServerClass():
  callback(default_OnPlcaStatus)
{

}

/**
 * Constructor for ModbusT1SServerClass with RS485 support.
 *
 * Initializes the Modbus server with RS485 communication.
 *
 * @param rs485 Reference to an RS485Class object for RS485 communication.
 */
ModbusT1SServerClass::ModbusT1SServerClass(RS485Class& rs485) : _rs485(&rs485)
{

}

/**
 * Destructor for ModbusT1SServerClass.
 */
ModbusT1SServerClass::~ModbusT1SServerClass()
{
}

/**
 * Start the Modbus T1S server with the specified parameters and RS485 communication.
 *
 * @param node_id id of the server
 * @param baudrate Baud rate to use
 * @param config serial config. to use defaults to SERIAL_8N1
 * @param rs485 RS485 object to use
 *
 * Return 1 on success, 0 on failure
 */
int ModbusT1SServerClass::begin(int node_id, unsigned long baudrate, uint16_t config, RS485Class& rs485)
{
  _node_id = node_id;
  if (&rs485 != nullptr)
  {
    _rs485 = &rs485;
  }

  pinMode(IRQ_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IRQ_PIN),

  []() {
    tc6_io->onInterrupt();
  },
  FALLING);

  /* Initialize IO module. */
  if (!tc6_io->begin())
  {
    return 0;
  }

  if(_gateway == IPAddress(0, 0, 0, 0)) {
    _gateway = IPAddress(192, 168, 42, 100);
  }

  IPAddress ip_addr = IPAddress(_gateway[0], _gateway[1], _gateway[2], _gateway[3] + _node_id);
  IPAddress network_mask = IPAddress(255, 255, 255, 0);
  IPAddress gateway = IPAddress(_gateway[0], _gateway[1], _gateway[2], _gateway[3]);

  T1SPlcaSettings t1s_plca_settings(_node_id);
  T1SMacSettings const t1s_default_mac_settings;
  MacAddress const mac_addr = MacAddress::create_from_uid();

  if (!tc6_inst->begin(ip_addr
                       , network_mask
                       , gateway
                       , mac_addr
                       , t1s_plca_settings
                       , t1s_default_mac_settings))
  {
    return 0;
  }
    if (!_server->begin(udp_port))
  {
    return 0;
  }

  float MODBUS_BIT_DURATION  = 1.f / _baudrate;
  float MODBUS_PRE_DELAY_BR  = MODBUS_BIT_DURATION * 9.6f * 3.5f * 1e6;
  float MODBUS_POST_DELAY_BR = MODBUS_BIT_DURATION * 9.6f * 3.5f * 1e6;

  _rs485->setDelays(MODBUS_PRE_DELAY_BR, MODBUS_POST_DELAY_BR);

  if(!ModbusRTUClient.begin(*_rs485, (unsigned long) _baudrate, (uint16_t) SERIAL_8N1)) {
    return 0;
  }

  ModbusRTUClient.setTimeout(2*1000);

  return 1;
}

/**
 * Set the Modbus RTU client timeout.
 *
 * This function sets the Modbus RTU client timeout.
 *
 *  @param timeout The timeout value in milliseconds.
 */
void ModbusT1SServerClass::setTimeout(unsigned long timeout) {
  ModbusRTUClient.setTimeout(timeout);
}

/**
 * Sets the Arduino_10BASE_T1S_UDP server for communication.
 *
 * This function sets the Arduino_10BASE_T1S_UDP server that the Modbus server will communicate with.
 *
 * @param server A pointer to the Arduino_10BASE_T1S_UDP server.
 */
int ModbusT1SServerClass::poll()
{
  return 0;
}

/**
 * Sets the Arduino_10BASE_T1S_UDP server for communication.
 *
 * This function sets the Arduino_10BASE_T1S_UDP server that the Modbus server will communicate with.
 *
 * @param server A pointer to the Arduino_10BASE_T1S_UDP server.
 */
int ModbusT1SServerClass::coilRead(int address)
{
  return read(address, COILS);
}

/**
 * Writes a value to a coil on the Modbus server.
 *
 * This function sends a request to write a value to a coil at the specified address
 * on the Modbus server using the provided UDP client.
 *
 * @param address The address of the coil to write to.
 * @return int 1 if the write operation is successful, -1 if an error occurs.
 */
int ModbusT1SServerClass::coilWrite(int address)
{
  return write(address, COILS);
}
/**
 * Reads the status of a discrete input from the Modbus server.
 *
 * This function sends a request to read the status of a discrete input at the specified address
 * from the Modbus server using the provided UDP client.
 *
 * @param address The address of the discrete input to read.
 * @return int The status of the discrete input (1 for ON, 0 for OFF) or -1 if an error occurs.
 */
int ModbusT1SServerClass::discreteInputRead(int address) {
  return read(address, DISCRETE_INPUTS);
}

/**
 * Reads the value of an input register from the Modbus server.
 *
 * This function sends a request to read the value of an input register at the specified address
 * from the Modbus server using the provided UDP client.
 *
 * @param address The address of the input register to read.
 * @return long The value of the input register or -1 if an error occurs.
 */
long ModbusT1SServerClass::inputRegisterRead(int address)
{
  return read(address, INPUT_REGISTERS);
}

/**
 * Writes a value to a holding register on the Modbus server.
 *
 * This function sends a request to write a value to a holding register at the specified address
 * on the Modbus server using the provided UDP client.
 *
 * @param address The address of the holding register to write to.
 * @return int 1 if the write operation is successful, -1 if an error occurs.
 */
long ModbusT1SServerClass::holdingRegisterRead(int address) {
  return read(address, HOLDING_REGISTERS);
}

/**
 * Writes a value to a holding register on the Modbus server.
 *
 * This function sends a request to write a value to a holding register at the specified address
 * on the Modbus server using the provided UDP client.
 *
 * @param address The address of the holding register to write to.
 * @return int 1 if the write operation is successful, -1 if an error occurs.
 */
int ModbusT1SServerClass::holdingRegisterWrite(int address) {
  return write(address, HOLDING_REGISTERS);
}

/**
 * Parses the Modbus packet received from the server.
 *
 * This function parses the Modbus packet received from the server.
 *
 * @return int The parsed packet or -1 if an error occurs.
 */
int ModbusT1SServerClass::parsePacket() {
  int res = -1;
  if(_server == nullptr) {
    return res;
  }

  int rx_packet_size = _server->parsePacket();
  if (rx_packet_size == 8)
  {
    int bytes_read = _server->read(udp_rx_buf, 8);
    res = udp_rx_buf[0] << 8 | udp_rx_buf[1];
    _last_ip = _server->remoteIP();
    _last_port = _server->remotePort();
     return rx_packet_size == bytes_read ? res : -1;
  }
  return res;
}

/**
 * Checks the PLCA status and Services the hardware and the protocol stack.
 *
 * This function checks the PLCA status and services the hardware and the
 * protocol stack.
 *
 */
void ModbusT1SServerClass::checkStatus()
{
  tc6_inst->service();

  static unsigned long prev_beacon_check = 0;
  static unsigned long prev_udp_packet_sent = 0;

  auto const now = millis();

  if ((now - prev_beacon_check) > 1000)
  {
    prev_beacon_check = now;
    tc6_inst->getPlcaStatus(callback);
  }
}

/**
 * Manage the incoming T1S packets and make the appropriate Modbus request.
 *
 * This function manages the incoming T1S packets and makes the appropriate Modbus request.
 */
void ModbusT1SServerClass::update() {

   checkStatus();
    switch (parsePacket())
    {
    case UDP_READ_COIL_PORT:
      coilRead();
      break;

    case UDP_WRITE_COIL_PORT:
      coilWrite();
      break;

    case UDP_READ_DI_PORT:
      discreteInputRead();
      break;

    case UDP_READ_IR_PORT:
      inputRegisterRead();
      break;

    case UDP_READ_HR_PORT:
      holdingRegisterRead();
      break;

    case UDP_WRITE_HR_PORT:
      holdingRegisterWrite();
      break;

    default:
      break;
   }
}

/**
 * Reads a value from the T1S client, sends it to the Modbus server and sends the response back to the client.
 *
 * This function reads a value from the T1S client, sends it to the Modbus server and sends the response back to the client.
 *
 * @param address The address of the value to read.
 * @param functionCode The function code to use for the Modbus request.
 *
 * @return long The value read from the Modbus server.
 */
long ModbusT1SServerClass::read(int address, int functionCode) {
  long res = -1;
  if(_server == nullptr) {
    return res;
  }

  int modbus_id =  udp_rx_buf[2] << 8 |  udp_rx_buf[3];
  int address_mod =  udp_rx_buf[4] << 8 |  udp_rx_buf[5];
  if (ModbusRTUClient.requestFrom(modbus_id, functionCode, address_mod, 1)) {
     if (ModbusRTUClient.available()) {
       switch (functionCode) {
         case INPUT_REGISTERS:
         case HOLDING_REGISTERS:
           res = ModbusRTUClient.read();
           break;
        case COILS:
        case DISCRETE_INPUTS:
          res = (uint8_t) ModbusRTUClient.read();
          break;
        default:
          return res;
      }

      if(res != -1) {
        udp_rx_buf[6] = (uint8_t)((res & 0xFF00) >> 8);
        udp_rx_buf[7] = (uint8_t)(res & 0x00FF);
        _server->beginPacket(_last_ip, _last_port);
        _server->write((uint8_t *)udp_rx_buf, sizeof(udp_rx_buf));
        _server->endPacket();
      }
    }
  }
  return res;
}

/**
 * reads a value from the T1S client and sends it to the Modbus server.
 *
 * This function reads a value from the T1S client and sends it to the Modbus server.
 *
 * @param address The address of the value to read.
 * @param functionCode The function code to use for the Modbus request.
 *
 * @return long The value read from the Modbus server.
 */
long ModbusT1SServerClass::write(int address,  int functionCode) {
  long res = -1;
  if(_server == nullptr) {
    return res;
  }

  int modbus_id =  udp_rx_buf[2] << 8 |  udp_rx_buf[3];
  int address_mod =  udp_rx_buf[4] << 8 |  udp_rx_buf[5];
  int value = -1;
  switch (functionCode) {
    case HOLDING_REGISTERS:
      value =  udp_rx_buf[6] << 8 |  udp_rx_buf[7];
      break;
    case COILS:
      value = udp_rx_buf[7];
      break;
    default:
      return res;
      break;
  }

  ModbusRTUClient.beginTransmission(modbus_id, functionCode, address_mod, 1);
  res = ModbusRTUClient.write(value);
  if (!ModbusRTUClient.endTransmission()) {
    res = -1;
  }
  return res;
}

/**
 * Sets the Arduino_10BASE_T1S_UDP server for communication.
 *
 * This function sets the Arduino_10BASE_T1S_UDP server that the Modbus server will communicate with.
 *
 * @param server A pointer to the Arduino_10BASE_T1S_UDP server.
 */
void ModbusT1SServerClass::setT1SServer(Arduino_10BASE_T1S_UDP & server) {
  _server = &server;
}

/**
 * Sets the port to use for communication.
 *
 * This function sets the port to use for communication.
 *
 * @param port The port to use.
 */
void ModbusT1SServerClass::setT1SPort(int port) {
  udp_port = port;
}

/**
 * Sets the baud rate to use for communication.
 *
 * This function sets the baud rate to use for communication.
 *
 * @param baudrate The baud rate to use.
 */
void ModbusT1SServerClass::setBaudrate(int baudrate) {
  _baudrate = baudrate;
}

/**
 * Sets the callback function to use for PLCA status check.
 *
 * This function sets the callback function to use for PLCA status check.
 *
 * @param cb The callback function to use.
 */
void ModbusT1SServerClass::setCallback(callback_f cb) {
  if(cb != nullptr) {
    callback = cb;
  }
}

/**
 * Default callback function for PLCA status check.
 *
 * This function is the default callback function for PLCA status check.
 *
 * @param success The success status of the PLCA status check.
 * @param plcaStatus The PLCA status.
 */
static void default_OnPlcaStatus(bool success, bool plcaStatus)
{
  if (!success)
  {
    return;
  }

  if (!plcaStatus) {
    tc6_inst->enablePlca();
  }
}

/**
 * Enables Power Over Ethernet (POE).
 *
 * This function enables Power Over Ethernet (POE) setting the device as power source.
 */
void ModbusT1SServerClass::enablePOE() {
  tc6_inst->digitalWrite(TC6::DIO::A0, true);
  tc6_inst->digitalWrite(TC6::DIO::A1, true);
}

/**
 * Disables Power Over Ethernet (POE).
 *
 * This function disables Power Over Ethernet (POE) and set the USB as power source.
 */
void ModbusT1SServerClass::disablePOE() {
  tc6_inst->digitalWrite(TC6::DIO::A0, false);
  tc6_inst->digitalWrite(TC6::DIO::A1, true);
}

/**
 * Sets the gateway IP address.
 *
 * This function sets the gateway IP address.
 *
 * @param ip The gateway IP address.
 */
void ModbusT1SServerClass::setGatwayIP(IPAddress ip) {
  _gateway = ip;
}

ModbusT1SServerClass ModbusT1SServer;
#endif
