/*
  Modbus RTU Temeperature Sensor

  This sketch show how to use the modbus library, in in order to sent a request to a slave RTU sensor
  unit and read the responce packet return by the remote unit.

  Circuit:
   - MKR board
   - TModbus RS485 Temperature
   - MKR 485 shield
     - ISO GND connected to GND of the Modbus RTU server
     - Y connected to A/Y of the Modbus RTU sensor
     - Z connected to B/Z of the Modbus RTU sensor
     - Jumper positions
       - FULL set to OFF
       - Z \/\/ Y set to OFF

  created 16 July 2018
  by Sandeep Mistry
*/

#include <ArduinoModbus.h>

float temperature, humidity;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("Modbus Temperature Humidity Sensor");
  // start the Modbus RTU client
  if (!ModbusRTUClient.begin(9600)) {
    Serial.println("Failed to start Modbus RTU Client!");
    while (1);
  }
}

void loop() {
  //send a reading request to the RTU slave with id=1, type=HOLDING_REGISTERS address=0, number of values to read, nb=2
  if (!ModbusRTUClient.requestFrom(1, HOLDING_REGISTERS, 0x00, 2)) {
    Serial.print("failed to read registers! ");
    Serial.println(ModbusRTUClient.lastError());
  } else {
    //if the request goes fine read the value with the read() function
    short rawtemperature = ModbusRTUClient.read();
    short rawhumidity = ModbusRTUClient.read();
    temperature = rawtemperature / 10.0;
    humidity = rawhumidity / 10.0;
    Serial.println(temperature);
    Serial.println(humidity);
  }

  delay(5000);
}
