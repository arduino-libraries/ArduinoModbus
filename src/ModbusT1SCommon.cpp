#include "ModbusT1SCommon.h"
#ifndef __AVR__
INIT_TC6(SPI, CS_PIN, RESET_PIN, IRQ_PIN);
#endif