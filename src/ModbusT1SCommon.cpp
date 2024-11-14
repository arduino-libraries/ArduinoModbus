#include "ModbusT1SCommon.h"
#if (defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNOR4_MINIMA))
INIT_TC6(SPI, CS_PIN, RESET_PIN, IRQ_PIN);
#endif