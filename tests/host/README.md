# Host regression checks

From the repository root, using Clang or GCC:

```sh
c++ -std=c++11 -Wall -Wextra -Werror -fsanitize=address,undefined \
  -Drealloc=controlled_realloc -Dfree=controlled_free \
  -Itests/host -Isrc -c src/ModbusClient.cpp -o /tmp/modbus-client.o
c++ -std=c++11 -Wall -Wextra -Werror -fsanitize=address,undefined \
  -Itests/host -Isrc /tmp/modbus-client.o tests/host/invalid_slave.cpp \
  -o /tmp/arduino-modbus-invalid-slave
/tmp/arduino-modbus-invalid-slave
```

This compiles the production client against a recording libmodbus backend and
minimal Arduino header. It verifies propagation of failed slave selection,
absence of subsequent reads/writes, valid requests, and buffered destination
reselection. It does not exercise serial hardware or replace board compilation
and device testing.

Allocator interception also checks failed buffer resizes and final buffer release.
Failed reads and end() must invalidate buffered read/write state.
