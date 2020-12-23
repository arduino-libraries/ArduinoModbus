/*
  Modbus RTU Temperature Sensor

  This sketch shows you how to interact with a Modbus RTU temperature and humidity sensor.
  It reads the temperature and humidity values every 5 seconds and outputs them to the
  serial monitor.

  Circuit:
   - MKR board
   - Winners® Modbus RS485 Temperature and Humidity:
     https://www.banggood.com/Modbus-RS485-Temperature-and-Humidity-Transmitter-Sensor-High-Precision-Monitoring-p-1159961.html?cur_warehouse=CN
   - External 9-36 V power Supply
   - MKR 485 shield
     - ISO GND connected to GND of the Modbus RTU sensor and the Power supply V-
     - Power supply V+ connected to V+ sensor
     - Y connected to A/Y of the Modbus RTU sensor
     - Z connected to B/Z of the Modbus RTU sensor
     - Jumper positions
       - FULL set to OFF
       - Z \/\/ Y set to ON

  created 8 August 2018
  by Riccardo Rizzo
*/

#include <ArduinoModbus.h>

float temperature;
float humidity;

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

  // send a Holding registers read request to (slave) id 1, for 2 registers
  if (!ModbusRTUClient.requestFrom(1, HOLDING_REGISTERS, 0x00, 2)) {
    Serial.print("failed to read registers! ");
    Serial.println(ModbusRTUClient.lastError());
  } else {
    // If the request succeeds, the sensor sends the readings, that are
    // stored in the holding registers. The read() method can be used to
    // get the raw temperature and the humidity values.
    short rawtemperature = ModbusRTUClient.read();
    short rawhumidity = ModbusRTUClient.read();

    // To get the temperature in Celsius and the humidity reading as
    // a percentage, divide the raw value by 10.0.
    temperature = rawtemperature / 10.0;
    humidity = rawhumidity / 10.0;
    Serial.println(temperature);
    Serial.println(humidity);
  }

  delay(5000);
}
