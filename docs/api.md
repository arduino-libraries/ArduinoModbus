# Arduino Modbus Library

## Modbus Client

### `client.coilRead()`

#### Description

Perform a "Read Coils" operation for the specified address for a single coil.

#### Syntax

```
int coilRead(int address);
int coilRead(int id, int address);
```

#### Parameters
- id (slave) - id of target, defaults to 0x00 if not specified
- address address to use for operation


#### Returns
coil value on success, -1 on failure.

### `client.discreteInputRead()`

#### Description

Perform a "Read Discrete Inputs" operation for the specified address for a single discrete input.

#### Syntax

```
int discreteInputRead(int address);
int discreteInputRead(int id, int address);
```

#### Parameters
- id (slave) - id of target, defaults to 0x00 if not specified
- address address to use for operation


#### Returns
discrete input value on success, -1 on failure.

### `client.holdingRegisterRead()`

#### Description

Perform a "Read Holding Registers" operation for a single holding register.

#### Syntax

```
long holdingRegisterRead(int address);
long holdingRegisterRead(int id, int address);
```

#### Parameters

- id (slave) - id of target, defaults to 0x00 if not specified
- address start address to use for operation
- holding register value on success, -1 on failure.

### `client.inputRegisterRead()`

#### Description

Perform a "Read Input Registers" operation for a single input register.

#### Syntax

```
long inputRegisterRead(int address);
long inputRegisterRead(int id, int address);
```

#### Parameters
- id (slave) - id of target, defaults to 0x00 if not specified
- address address to use for operation


#### Returns
input register value on success, -1 on failure.

### `client.coilWrite()`

#### Description

Perform a "Write Single Coil" operation for the specified address and value.

#### Syntax

```
int coilWrite(int address, uint8_t value);
int coilWrite(int id, int address, uint8_t value);
```

#### Parameters
- id (slave) - id of target, defaults to 0x00 if not specified
- address address to use for operation
- value coi -l value to write


#### Returns
1 on success, 0 on failure.

### `client.holdingRegisterWrite()`

#### Description

Perform a "Write Single Holding Register" operation for the specified address and value.

#### Syntax

```
int holdingRegisterWrite(int address, uint16_t value);
int holdingRegisterWrite(int id, int address, uint16_t value);
```

#### Parameters
- id (slave) - id of target, defaults to 0x00 if not specified
- address address to use for operation
- value - holding register value to write


#### Returns
1 on success, 0 on failure.

### `client.registerMaskWrite()`

#### Description

Perform a "Mask Write Registers" operation for the specified address, AND mask and OR mask.

#### Syntax

```
int registerMaskWrite(int address, uint16_t andMask, uint16_t orMask);
int registerMaskWrite(int id, int address, uint16_t andMask, uint16_t orMask);
```

#### Parameters
- id (slave) - id of target, defaults to 0x00 if not specified
- address address to use for operation
- andMask - AND mask to use for operation
- orMask - OR mask to use for operation


#### Returns
1 on success, 0 on failure.

### `client.beginTransmission()`

#### Description

Begin the process of a writing multiple coils or holding registers.
Use write(value) to set the values you want to send, and endTransmission() to send request on the wire.

#### Syntax

```
int beginTransmission(int type, int address, int nb);
int beginTransmission(int id, int type, int address, int nb);
```

#### Parameters
- id (slave) - id of target, defaults to 0x00 if not specified
- type - type of write to perform, either COILS or HOLDING_REGISTERS
- address start address to use for operation
- nb - number of values to write


#### Returns
1 on success, 0 on failure

### `client.write()`

#### Description

Set the values of a write operation started by beginTransmission(...).

#### Syntax

```
int write(unsigned int value);
```

#### Parameters
- value - value to write


#### Returns
1 on success, 0 on failure

### `client.endTransmission()`

#### Description

End the process of a writing multiple coils or holding registers.

#### Syntax

```
int endTransmission();
```

#### Parameters
none


#### Returns
1 on success, 0 on failure

### `client.requestFrom()`

#### Description

Read multiple coils, discrete inputs, holding registers, or input register values.
Use available() and read() to process the read values.

#### Syntax

```
int requestFrom(int type, int address, int nb);
int requestFrom(int id, int type, int address,int nb);
```

#### Parameters
- id (slave) - id of target, defaults to 0x00 if not specified
type - type of read to perform, either COILS, DISCRETE_INPUTS, HOLDING_REGISTERS, or INPUT_REGISTERS
- address start address to use for operation
- nb - number of values to read


#### Returns
0 on failure, number of values read on success

### `client.available()`

#### Description

Query the number of values available to read after calling requestFrom(...)

#### Syntax

```
int available();
```

#### Parameters
none


#### Returns
number of values available for reading use read()

### `client.read()`

#### Description

Read a value after calling requestFrom(...)

#### Syntax

```
long read();
```

#### Parameters
None


#### Returns
-1 on failure, value on success

### `client.lastError()`

#### Description

Read the last error reason as a string

#### Syntax

```
const char* lastError();
```

#### Parameters
none


#### Returns
Last error reason as a C string

### `client.end()`

#### Description

Stop the client and clean up

#### Syntax

```
void end();
```

#### Parameters
None


#### Returns
nothing

## ModbusRTUClient Class

### `modbusRTUClient.begin()`

#### Description

Start the Modbus RTU client with the specified parameters.

#### Syntax

```
ModbusRTUClient.begin(baudrate);
ModbusRTUClient.begin(baudrate, config);
```

#### Parameters
- baudrate - Baud rate to use for serial
- config - Config to use for serial (see Serial.begin(...) for more info.) defaults to SERIAL_8N1 if not provided


#### Returns
1 on success, 0 on failure

## ModbusTCPClient Class

### `ModbusTCPClient()`

#### Description

Creates a Modbus TCP client using the provided Client for the transport.

#### Syntax

```
ModbusTCPClient(client);
```

#### Parameters
- Client - to use for the transport

### `modbusTCPClient.begin()`

#### Description

Start the Modbus TCP client with the specified parameters.

#### Syntax

```
modbusTCPClient.begin(ip, port);
```

#### Parameters
- ip - the IP Address the client will connect to
- port - port to the client will connect to


#### Returns
1 on success, 0 on failure

### `modbusTCPClient.connected()`

#### Description

Returns the connection status.

#### Syntax

```
modbusTCPClient.connected();
```

#### Parameters
None


#### Returns
Returns true if the client is connected, false if not.

### `modbusTCPClient.stop()`

#### Description

Disconnect from the server.

#### Syntax

```
modbusTCPClient.stop();
```

#### Parameters
None


#### Returns
Nothing

## ModbusServer Class

### `modbusServer.configureCoils()`

#### Description

Configure the servers coils.

#### Syntax

```
int configureCoils(int startAddress, int nb);
```

#### Parameters
- startAddress - start address of coils
- nb - number of coils to configure


#### Returns
0 on success, 1 on failure

### `modbusServer.configureDiscreteInputs()`

#### Description

Configure the servers discrete inputs.

#### Syntax

```
int configureDiscreteInputs(int startAddress, int nb);
```

#### Parameters
- startAddress - start address of discrete inputs
- nb - number of discrete inputs to configure


#### Returns
0 on success, 1 on failure

### `modbusServer.configureHoldingRegisters()`

#### Description

Configure the servers holding registers.

#### Syntax

```
int configureHoldingRegisters(int startAddress, int nb);
```

#### Parameters
- startAddress - start address of holding registers
- nb - number of holding registers to configure


#### Returns
0 on success, 1 on failure

### `modbusServer.configureInputRegisters()`

#### Description

Configure the servers input registers.

#### Syntax

```
int configureInputRegisters(int startAddress, int nb);
```

#### Parameters
- startAddress - start address of input registers
- nb - number of input registers to configure


#### Returns
0 on success, 1 on failure

### `modbusServer.coilRead()`

#### Description

Perform a "Read Coils" operation for the specified address for a single coil.

#### Syntax

```
int coilRead(int address);
int coilRead(int id, int address);
```

#### Parameters
- id (slave) - id of target, defaults to 0x00 if not specified
- address address to use for operation


#### Returns
coil value on success, -1 on failure.

### `modbusServer.discreteInputRead()`

#### Description

Perform a "Read Discrete Inputs" operation for the specified address for a single discrete input.

#### Syntax

```
int discreteInputRead(int address);
int discreteInputRead(int id, int address);
```

#### Parameters
- id (slave) - id of target, defaults to 0x00 if not specified
- address address to use for operation


#### Returns
discrete input value on success, -1 on failure.

### `modbusServer.holdingRegisterRead()`

#### Description

Perform a "Read Holding Registers" operation for a single holding register.

#### Syntax

```
long holdingRegisterRead(int address);
long holdingRegisterRead(int id, int address);
```

#### Parameters
- id (slave) - id of target, defaults to 0x00 if not specified
- address start address to use for operation
- holding register value on success, -1 on failure.

### `modbusServer.inputRegisterRead()`

#### Description

Perform a "Read Input Registers" operation for a single input register.

#### Syntax

```
long inputRegisterRead(int address);
long inputRegisterRead(int id, int address);
```

#### Parameters
- id (slave) - id of target, defaults to 0x00 if not specified
- address address to use for operation


#### Returns
input register value on success, -1 on failure.

### `modbusServer.coilWrite()`

#### Description

Perform a "Write Single Coil" operation for the specified address and value.

#### Syntax

```
int coilWrite(int address, uint8_t value);
int coilWrite(int id, int address, uint8_t value);
```

#### Parameters
- id (slave) - id of target, defaults to 0x00 if not specified
- address address to use for operation
- value coi -l value to write


#### Returns
1 on success, 0 on failure.

### `modbusServer.holdingRegisterWrite()`

#### Description

Perform a "Write Single Holding Register" operation for the specified address and value.

#### Syntax

```
int holdingRegisterWrite(int address, uint16_t value);
int holdingRegisterWrite(int id, int address, uint16_t value);
```

#### Parameters
- id (slave) - id of target, defaults to 0x00 if not specified
- address address to use for operation
- value - holding register value to write


#### Returns
1 on success, 0 on failure.

### `modbusServer.registerMaskWrite()`

#### Description

Perform a "Mask Write Registers" operation for the specified address, AND mask and OR mask.

#### Syntax
```
int registerMaskWrite(int address, uint16_t andMask, uint16_t orMask);
int registerMaskWrite(int id, int address, uint16_t andMask, uint16_t orMask);
```

#### Parameters
- id (slave) - id of target, defaults to 0x00 if not specified
- address address to use for operation
- andMask - AND mask to use for operation
- orMask - OR mask to use for operation


#### Returns
1 on success, 0 on failure.

### `modbusServer.discreteInputWrite()`

#### Description

Write the value of the server's Discrete Input for the specified address and value.

#### Syntax

```
int discreteInputWrite(int address, uint8_t value);
```

#### Parameters
- address address to use for operation
- value - discrete input value to write


#### Returns
1 on success, 0 on failure.

### `modbusServer.writeDiscreteInputs()`

#### Description

Write values to the server's Discrete Inputs for the specified address and values.

#### Syntax

```
int writeDiscreteInputs(int address, uint8_t values[], int nb);
```

#### Parameters
- address address to use for operation
- values - array of discrete inputs values to write
- nb - number of discrete inputs to write


#### Returns
1 on success, 0 on failure.

### `modbusServer.inputRegisterWrite()`

#### Description

Write the value of the server's Input Register for the specified address and value.

#### Syntax

```
int inputRegisterWrite(int address, uint16_t value);
```

#### Parameters
- address address to use for operation
- value - input register value to write


#### Returns
1 on success, 0 on failure.

### `modbusServer.writeInputRegisters()`

#### Description

Write values to the server's Input Registers for the specified address and values.

#### Syntax

```
int writeInputRegisters(int address, uint16_t values[], int nb);
```

#### Parameters
- address address to use for operation
- values - array of input registers values to write
- nb - number of input registers to write

#### Returns
1 on success, 0 on failure.

### `modbusServer.poll()`

#### Description

Poll for requests

#### Syntax

```
virtual void poll() = 0;
```

#### Parameters
None

#### Returns
nothing

### `modbusServer.end()`

#### Description

Stop the server

#### Syntax

```
void end();
```

#### Parameters
None

#### Return
nothing

## ModbusRTUServer Class

### `modbusRTUServer.begin()`

#### Description

Start the Modbus RTU server with the specified parameters.

#### Syntax

```
ModbusRTUServer.begin(id, baudrate);
ModbusRTUServer.begin(id, baudrate, config);
```

#### Parameters
- id - (slave) id of the server baudrate - Baud rate to use for serial
- config - Config to use for serial (see Serial.begin(...) for more info.) defaults to SERIAL_8N1 if not provided


#### Returns
1 on success, 0 on failure

## ModbusTCPServer

### `ModbusTCPServer()`

#### Description

Creates a Modbus TCP server.

#### Syntax

```
ModbusTCPServer();
```

#### Parameters
None

### `modbusTCPServer.begin()`

#### Description

Start the Modbus TCP server.

#### Syntax

```
modbusTCPserver.begin();
modbusTCPserver.begin(id);
```

#### Parameters
- id - the (slave) id of the server, defaults to 0xff (TCP);


#### Returns
1 on success, 0 on failure

### `modbusTCPServer.accept()`

#### Description

Accept a client connection.

#### Syntax

```
modbusTCPserver.accept(client);
```

#### Parameters
- client - the Client to accept a connection from;


#### Returns
Nothing
