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

#ifndef _MODBUS_RTU_DELAY_H_INCLUDED
#define _MODBUS_RTU_DELAY_H_INCLUDED

class ModbusRTUDelay
{
public:
  /* Do not allow construction or copying. */
  ModbusRTUDelay() = delete;
  ModbusRTUDelay(ModbusRTUDelay const &) = delete;

  /* Calculate __minimum__ preDelay and postDelay in
   * microseconds as per Modbus RTU Specification.
   *
   * MODBUS over serial line specification and implementation guide V1.02
   * Paragraph 2.5.1.1 MODBUS Message RTU Framing
   * https://modbus.org/docs/Modbus_over_serial_line_V1_02.pdf
   */
  static unsigned long const preDelay(unsigned long const baudrate)
  {
    double const bit_duration = 1. / baudrate;
    double const word_len = 9.6f; // try also with 10.0f
    double const pre_delay_us = bit_duration * word_len * 3.5f * 1e6;
    return static_cast<unsigned long>(pre_delay_us);
  }
  static unsigned long const postDelay(unsigned long const baudrate)
  {
    return preDelay(baudrate);
  }
};

#endif /* _MODBUS_RTU_DELAY_H_INCLUDED */
