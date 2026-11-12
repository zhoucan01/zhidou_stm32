"""Install the version query handler in the authorized sibling APP project."""
from pathlib import Path

target = Path(r'C:\Users\29546\Desktop\code\machine\code\V2\at32f415rbt7\code\task\modbusTask.c')
s = target.read_text(encoding='utf-8')
marker = 'static uint16_t processOtaManagement(const uint8_t *data, uint16_t available)'
handler = '''/* 0x11: offset/count big-endian, CRC16 little-endian. Read the OTA
 * header, not compile-time FW_VERSION macros: host changes it each cycle. */
#ifndef MODBUS_VERSION_BASE
#define MODBUS_VERSION_BASE ((const uint8_t *)0x08003800u)
#endif
static uint16_t processVersionQuery(const uint8_t *data, uint16_t available)
{
  uint16_t offset, count, crc;
  uint8_t reply[40];
  if(available < 2U || data[0] != MODBUS_SLAVE_ADDRESS || data[1] != 0x11U)
    return 0U;
  if(available < 8U) return 1U;
  crc = Modbus_CRC16(data, 6U);
  if(data[6] != (uint8_t)crc || data[7] != (uint8_t)(crc >> 8)) return 1U;
  offset = (uint16_t)(((uint16_t)data[2] << 8) | data[3]);
  count = (uint16_t)(((uint16_t)data[4] << 8) | data[5]);
  if(count == 0U || count > 32U || offset > 1024U - count) return 8U;
  modbusRxTotalCount++;
  reply[0] = MODBUS_SLAVE_ADDRESS; reply[1] = 0x11U;
  reply[2] = 1U; /* Only executing APP code reports APP mode. */
  reply[3] = data[2]; reply[4] = data[3]; reply[5] = (uint8_t)count;
  memcpy(reply + 6, MODBUS_VERSION_BASE + offset, count);
  crc = Modbus_CRC16(reply, (uint16_t)(count + 6U));
  reply[count + 6U] = (uint8_t)crc; reply[count + 7U] = (uint8_t)(crc >> 8);
  if(modbusRs485Send(reply, (uint16_t)(count + 8U))) modbusReplyTotalCount++;
  return 8U;
}

'''
assert marker in s
if 'static uint16_t processVersionQuery' not in s:
    s = s.replace(marker, handler + marker)
    needle = '    uint16_t otaLength = processOtaManagement(&data[offset], available);'
    assert needle in s
    s = s.replace(needle, '''    uint16_t otaLength = processVersionQuery(&data[offset], available);
    if(otaLength == 0U) otaLength = processOtaManagement(&data[offset], available);''')
target.write_bytes(s.replace('\r\n', '\n').replace('\n', '\r\n').encode('utf-8'))
print('Updated', target)
