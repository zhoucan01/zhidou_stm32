#include "modbus.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void put16(uint8_t *p, unsigned value)
{
  p[0] = (uint8_t)(value >> 8);
  p[1] = (uint8_t)value;
}

static void seal(uint8_t *p, size_t n)
{
  uint16_t crc = modbus_crc16(p, n - 2);
  p[n - 2] = (uint8_t)crc;
  p[n - 1] = (uint8_t)(crc >> 8);
}

int main(void)
{
  modbus_t ctx = {0x12345678U, MODBUS_STATUS_DOWNLOADING};
  modbus_10_data_t control, old_control;
  modbus_aa_data_t packet, in_place, old_packet;
  uint8_t query[14] = {MODBUS_ADDRESS,0x10,0,230,0,3,0,1,0x12,0x34,0x56,0x78};
  uint8_t command[20] = {MODBUS_ADDRESS,0x10,0,230,0,4,0,0,0,8,0,1,0x12,0x34,0x56,0x78,0,2};
  uint8_t frame[1046] = {MODBUS_ADDRESS,0xaa}, firmware[1024], reply[14], stream[1050];
  uint8_t unchanged[1024];
  const uint8_t *found;
  size_t i, cursor, length;
  int code;
  assert(sizeof(ctx) == 8);
  assert(modbus_crc16((const uint8_t *)"123456789",9) == 0x4b37);
  assert(modbus_crc32((const uint8_t *)"123456789",9) == 0xcbf43926U);
  seal(query,sizeof(query)); seal(command,sizeof(command));
  assert(modbus_read_10(&ctx,query,sizeof(query),&control) == 1);
  assert(control.is_query && control.operation == 0 && control.key == ctx.key);
  assert(modbus_read_10(&ctx,command,sizeof(command),&control) == 1);
  assert(!control.is_query && control.operation == 2 && control.register_count == 4);
  old_control = control;
  for(i = 0; i < sizeof(command); ++i)
  {
    assert(modbus_read_10(&ctx,command,i,&control) == -1);
    assert(memcmp(&control,&old_control,sizeof(control)) == 0);
  }
  assert(modbus_build_10_reply(&ctx,1,reply,sizeof(reply)) == 14);
  assert(reply[6] == 0 && reply[7] == MODBUS_STATUS_DOWNLOADING);
  assert(memcmp(reply+8,query+8,4) == 0 && modbus_crc16(reply,14) == 0);
  assert(modbus_build_10_reply(&ctx,0,reply,sizeof(reply)) == 8);
  assert(reply[5] == 4 && modbus_crc16(reply,8) == 0);
  for(code = MODBUS_ACK_OK; code <= MODBUS_ACK_FLASH_WRITE; ++code)
  {
    assert(modbus_build_aa_reply(&ctx,(modbus_ack_t)code,0x1234,reply,sizeof(reply)) == 7);
    assert(reply[2] == code && reply[3] == 0x12 && reply[4] == 0x34);
    assert(modbus_crc16(reply,7) == 0);
  }
  assert(modbus_build_aa_reply(&ctx,(modbus_ack_t)0,0,reply,sizeof(reply)) == -1);
  assert(modbus_build_10_reply(&ctx,1,reply,13) == -1);
  put16(frame+2,sizeof(frame)); put16(frame+4,1); put16(frame+10,sizeof(firmware));
  put16(frame+12,2); put16(frame+14,1);
  for(i = 0; i < sizeof(firmware); ++i) frame[16+i] = (uint8_t)(i*13);
  frame[1040] = 0x12; frame[1041] = 0x34; frame[1042] = 0x56; frame[1043] = 0x78;
  seal(frame,sizeof(frame));
  assert(modbus_read_aa(&ctx,frame,sizeof(frame),&packet,firmware,sizeof(firmware)) == 1);
  assert(packet.firmware_size == 1024 && packet.received_crc32 == 0x12345678U);
  assert(memcmp(firmware,frame+16,1024) == 0);
  assert(modbus_read_aa(&ctx,frame,sizeof(frame),&in_place,frame+16,1024) == 1);
  assert(memcmp(&packet,&in_place,sizeof(packet)) == 0);
  old_packet = packet; memcpy(unchanged,firmware,sizeof(firmware));
  for(i = 0; i < sizeof(frame); ++i)
    assert(modbus_read_aa(&ctx,frame,i,&packet,firmware,sizeof(firmware)) == -1);
  assert(memcmp(&packet,&old_packet,sizeof(packet)) == 0);
  assert(memcmp(firmware,unchanged,sizeof(firmware)) == 0);
  assert(modbus_read_aa(&ctx,frame,sizeof(frame),&packet,firmware,1023) == -1);
  frame[11] ^= 1;
  assert(modbus_read_aa(&ctx,frame,sizeof(frame),&packet,firmware,1024) == -1);
  frame[11] ^= 1;
  memset(stream,0x55,4); memcpy(stream+4,frame,sizeof(frame)); cursor = 0;
  assert(modbus_find_frame(&ctx,stream,sizeof(stream),&cursor,&found,&length) == 1);
  assert(found == stream+4 && length == sizeof(frame) && cursor == sizeof(stream));
  assert(modbus_find_frame(&ctx,stream,sizeof(stream),&cursor,&found,&length) == 0);
  assert(ctx.key == 0x12345678U && ctx.status == MODBUS_STATUS_DOWNLOADING);
  puts("fixed-context parsing, bounds, in-place copy and reply tests passed");
  return 0;
}
