/*
  Modbus RTU Client MD02 Temperature Humidity sensor

  This sketch creates a Modbus RTU Client and demonstrates
  how to use read temperature and humidity values as sensed
  by the RTU Modbus capable MD02 sensor.
*/

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static unsigned int const MODBUS_BAUDRATE      = 9600;
static float        const MODBUS_BIT_DURATION  = 1.f / MODBUS_BAUDRATE;
static float        const MODBUS_PRE_DELAY_BR  = MODBUS_BIT_DURATION * 9.6f * 3.5f * 1e6;
static float        const MODBUS_POST_DELAY_BR = MODBUS_BIT_DURATION * 9.6f * 3.5f * 1e6;

static int          const MODBUS_DEVICE_ID                   = 1;
static int          const MODBUS_DEVICE_TEMPERATURE_REGISTER = 0x0001;
static int          const MODBUS_DEVICE_HUMIDITY_REGISTER    = 0x0002;

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin(9600);
  while (!Serial);

  RS485.setDelays(MODBUS_PRE_DELAY_BR, MODBUS_POST_DELAY_BR);

  if (!ModbusRTUClient.begin(MODBUS_BAUDRATE, SERIAL_8N1))
  {
    Serial.println("Failed to start Modbus RTU Client!");
    for (;;) { }
  }

  ModbusRTUClient.setTimeout(2 * 1000UL); /* 2 seconds. */
}

void loop()
{
  if (!ModbusRTUClient.requestFrom(MODBUS_DEVICE_ID, INPUT_REGISTERS, MODBUS_DEVICE_TEMPERATURE_REGISTER, 1)) {
    Serial.print("failed to read temperature register! ");
    Serial.println(ModbusRTUClient.lastError());
    return;
  }
  if (ModbusRTUClient.available())
  {
    int16_t const temperature_raw = ModbusRTUClient.read();
    float const temperature_deg = temperature_raw / 100.f;
    Serial.println(temperature_deg);
  }

  if (!ModbusRTUClient.requestFrom(MODBUS_DEVICE_ID, INPUT_REGISTERS, MODBUS_DEVICE_HUMIDITY_REGISTER, 1)) {
    Serial.print("failed to read humidity register! ");
    Serial.println(ModbusRTUClient.lastError());
    return;
  }
  if (ModbusRTUClient.available())
  {
    int16_t const humidity_raw = ModbusRTUClient.read();
    float const humidity_per_cent = humidity_raw / 100.f;
    Serial.println(humidity_per_cent);
  }

  delay(1000);
}
