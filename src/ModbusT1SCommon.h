#ifndef _MODBUS_T1S_COMMON_H_INCLUDED
#define _MODBUS_T1S_COMMON_H_INCLUDED
#if (defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNOR4_MINIMA))
#include <Arduino_10BASE_T1S.h>

enum ModbusT1SFunctionCode {
  UDP_READ_COIL_PORT = 1,
  UDP_WRITE_COIL_PORT,
  UDP_READ_DI_PORT,
  UDP_READ_IR_PORT,
  UDP_READ_HR_PORT,
  UDP_WRITE_HR_PORT
};

auto const tc6_io = new TC6::TC6_Io
( SPI
  , CS_PIN
  , RESET_PIN
  , IRQ_PIN);
auto const tc6_inst = new TC6::TC6_Arduino_10BASE_T1S(tc6_io);
#endif
#endif