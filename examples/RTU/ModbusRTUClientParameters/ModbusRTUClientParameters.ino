/*
  Modbus RTU Client Parameters

  This sketch is the same as ModbusRTUClientToggle and shows how to set
  a few RS485 parameters.

  Circuit:
   - Arduino Opta
     - GND connected to GND of the Modbus RTU server
     - A(-) connected to A of the Modbus RTU server
     - B(+) connected to B of the Modbus RTU server
*/

#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>

constexpr auto baudrate { 19200 };

// Calculate preDelay and postDelay in microseconds as per Modbus RTU Specification
//
// MODBUS over serial line specification and implementation guide V1.02
// Paragraph 2.5.1.1 MODBUS Message RTU Framing
// https://modbus.org/docs/Modbus_over_serial_line_V1_02.pdf
constexpr auto bitduration { 1.f / baudrate };
constexpr auto wordlen { 9.6f }; // try also with 10.0f
constexpr auto preDelayBR { bitduration * wordlen * 3.5f * 1e6 };
constexpr auto postDelayBR { bitduration * wordlen * 3.5f * 1e6 };

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("Modbus RTU Client Toggle w/ Parameters");

  RS485.setDelays(preDelayBR, postDelayBR);

  // start the Modbus RTU client in 8E1 mode
  if (!ModbusRTUClient.begin(baudrate, SERIAL_8E1)) {
    Serial.println("Failed to start Modbus RTU Client!");
    while (1);
  }
}

void loop() {
  // for (slave) id 1: write the value of 0x01, to the coil at address 0x00 
  if (!ModbusRTUClient.coilWrite(1, 0x00, 0x01)) {
    Serial.print("Failed to write coil! ");
    Serial.println(ModbusRTUClient.lastError());
  }

  // wait for 0.5 second
  delay(500);

  // for (slave) id 1: write the value of 0x00, to the coil at address 0x00 
  if (!ModbusRTUClient.coilWrite(1, 0x00, 0x00)) {
    Serial.print("Failed to write coil! ");
    Serial.println(ModbusRTUClient.lastError());
  }

  // wait for 0.5 second
  delay(500);

  auto coil = ModbusRTUClient.coilRead(0x02, 0x00);

  if (coil < 0) {
    Serial.print("Failed to read coil: ");
    Serial.println(ModbusRTUClient.lastError());
  } else {
    Serial.print("Coil: ");
      Serial.println(coil);
  }

  // wait for 0.5 second
  delay(500); 
}
