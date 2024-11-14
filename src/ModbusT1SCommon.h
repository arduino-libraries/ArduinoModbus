#ifndef _MODBUS_T1S_COMMON_H_INCLUDED
#define _MODBUS_T1S_COMMON_H_INCLUDED

#include <Arduino_10BASE_T1S.h>

#define INIT_TC6(_SPI, _CS_PIN, _RESET_PIN, _IRQ_PIN) \
  TC6::TC6_Io* tc6_io = new TC6::TC6_Io \
  ( _SPI \
    , _CS_PIN \
    , _RESET_PIN \
    , _IRQ_PIN); \
  TC6::TC6_Arduino_10BASE_T1S* tc6_inst = new TC6::TC6_Arduino_10BASE_T1S(tc6_io);

extern TC6::TC6_Arduino_10BASE_T1S* tc6_inst;
extern TC6::TC6_Io* tc6_io;
static void default_OnPlcaStatus(bool success, bool plcaStatus);

#if (defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNOR4_MINIMA))
#define RS485_SERIAL Serial1
#define RS485_TX_PIN 1
#define RS485_RX_PIN 0
#define RS485_DE_PIN 8
#define RS485_RE_PIN 7
#endif

enum ModbusT1SFunctionCode {
  UDP_READ_COIL_PORT = 1,
  UDP_WRITE_COIL_PORT,
  UDP_READ_DI_PORT,
  UDP_READ_IR_PORT,
  UDP_READ_HR_PORT,
  UDP_WRITE_HR_PORT
};

#endif