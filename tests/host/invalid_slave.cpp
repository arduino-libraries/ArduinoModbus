// Host regression test: compile the real ModbusClient against a recording backend.
#include "ModbusClient.h"
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <initializer_list>


struct _modbus { int slave; };
static int requests = 0;
static int selected = 0;
static bool failSelection = false;
static bool failRead = false;
static bool failAllocation = false;
static void* buffer = NULL;
extern "C" void* controlled_realloc(void* old, size_t size) {
  if (failAllocation) return NULL;
  void* result = realloc(old, size);
  if (result) buffer = result;
  return result;
}
extern "C" void controlled_free(void* ptr) {
  assert(ptr == buffer);
  free(ptr);
  buffer = NULL;
}
extern "C" {
int modbus_connect(modbus_t*) { return 0; }
void modbus_close(modbus_t*) {}
void modbus_free(modbus_t* ctx) { delete ctx; }
int modbus_set_error_recovery(modbus_t*, modbus_error_recovery_mode) { return 0; }
int modbus_set_response_timeout(modbus_t*, uint32_t, uint32_t) { return 0; }
const char* modbus_strerror(int) { return "test error"; }
int modbus_set_slave(modbus_t* ctx, int id) {
  if (failSelection || id < 0 || id > 247) { errno = EINVAL; return -1; }
  ctx->slave = id;
  return 0;
}
#define READ_FN(name, type) \
int name(modbus_t* ctx, int, int nb, type* dest) { \
  ++requests; selected = ctx->slave; \
  if (failRead) { errno = EIO; return -1; } \
  for (int i = 0; i < nb; ++i) dest[i] = 1; \
  return nb; \
}
READ_FN(modbus_read_bits, uint8_t)
READ_FN(modbus_read_input_bits, uint8_t)
READ_FN(modbus_read_registers, uint16_t)
READ_FN(modbus_read_input_registers, uint16_t)
int modbus_write_bit(modbus_t* ctx, int, int) { ++requests; selected = ctx->slave; return 1; }
int modbus_write_register(modbus_t* ctx, int, int) { ++requests; selected = ctx->slave; return 1; }
int modbus_mask_write_register(modbus_t* ctx, int, uint16_t, uint16_t) { ++requests; selected = ctx->slave; return 1; }
int modbus_write_bits(modbus_t* ctx, int, int nb, const uint8_t*) { ++requests; selected = ctx->slave; return nb; }
int modbus_write_registers(modbus_t* ctx, int, int nb, const uint16_t*) { ++requests; selected = ctx->slave; return nb; }
}
class TestClient : public ModbusClient {
public:
  TestClient() : ModbusClient(1000) { assert(begin(new modbus_t{1}, 1)); }
};
int main() {
  TestClient client;
  for (int id : {-1, 248, 256}) {
    requests = 0;
    assert(client.coilRead(id, 0) == -1);
    assert(client.discreteInputRead(id, 0) == -1);
    assert(client.holdingRegisterRead(id, 0) == -1);
    assert(client.inputRegisterRead(id, 0) == -1);
    assert(client.coilWrite(id, 0, 1) == 0);
    assert(client.holdingRegisterWrite(id, 0, 1) == 0);
    assert(client.registerMaskWrite(id, 0, 0, 1) == 0);
    for (int type = COILS; type <= INPUT_REGISTERS; ++type) {
      assert(client.requestFrom(id, type, 0, 2) == 0);
    }
    assert(client.beginTransmission(id, COILS, 0, 2) == 0);
    assert(client.beginTransmission(id, HOLDING_REGISTERS, 0, 2) == 0);
    assert(errno == EINVAL);
    assert(requests == 0);
  }
  assert(client.coilRead(247, 0) == 1);
  assert(selected == 247);
  assert(client.requestFrom(2, HOLDING_REGISTERS, 0, 2) == 2);
  assert(client.read() == 1 && client.read() == 1 && client.read() == -1);
  assert(client.beginTransmission(3, COILS, 0, 2) == 1);
  assert(client.write(1) && client.write(0));
  assert(client.coilRead(4, 0) == 1);
  assert(client.endTransmission() == 1);
  assert(selected == 3); // reselect the buffered transmission's destination
  assert(client.endTransmission() == 0);
  assert(client.beginTransmission(3, HOLDING_REGISTERS, 0, 1) == 1);
  assert(client.write(42) == 1);
  failSelection = true;
  requests = 0;
  assert(client.endTransmission() == 0);
  assert(requests == 0 && errno == EINVAL);
  failSelection = false;
  assert(client.endTransmission() == 0);
  assert(client.requestFrom(1, HOLDING_REGISTERS, 0, 8) == 8);
  failRead = true;
  assert(client.requestFrom(1, COILS, 0, 1) == 0);
  assert(client.available() == 0);
  assert(client.read() == -1);
  failRead = false;
  assert(client.requestFrom(1, COILS, 0, 2) == 2);
  failAllocation = true;
  assert(client.requestFrom(1, COILS, 0, 16) == 0);
  assert(errno == ENOMEM && client.available() == 0);
  assert(client.beginTransmission(1, COILS, 0, 16) == 0);
  assert(errno == ENOMEM && client.write(1) == 0);
  failAllocation = false;
  assert(client.requestFrom(1, COILS, 0, 2) == 2);
  client.end();
  assert(buffer == NULL);
  assert(client.available() == 0);
  assert(client.read() == -1);
  puts("invalid address propagation and valid request controls passed");
}
