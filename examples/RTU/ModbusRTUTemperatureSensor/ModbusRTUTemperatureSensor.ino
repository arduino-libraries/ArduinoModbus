/*
  Modbus RTU Temeperature Sensor

  This sketch shows you how to interact with a Modbus RTU temperature and humidity sensor.
  It reads the temperature and humidity values every 5 seconds and outputs them to the
  serial monitor.

  Circuit:
   - MKR board
   - Modbus RS485 Temperature:
   - External power Supply
   - MKR 485 shield
     - ISO GND connected to GND of the Modbus RTU sensor and the Power supply V-;
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

    // If the request goes fine, the sensor sent the readings as bytes packet,
    // through the read() function is possible read the measurments.
    // the readings is parsed as a short integer from the packets
    short rawtemperature = ModbusRTUClient.read();
    short rawhumidity = ModbusRTUClient.read();

    // Is required divide by 10.0 the value readed, because the sensor sent the
    // readings as an integer obtained multipling the float value readed by 10
    temperature = rawtemperature / 10.0;
    humidity = rawhumidity / 10.0;
    Serial.println(temperature);
    Serial.println(humidity);
  }

  delay(5000);
}