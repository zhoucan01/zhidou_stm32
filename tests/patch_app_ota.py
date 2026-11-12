"""Apply the OTA compatibility bridge to the sibling application project."""
from pathlib import Path

target = Path(r'C:\Users\29546\Desktop\code\machine\code\V2\at32f415rbt7\code\task\modbusTask.c')
s = target.read_text(encoding='utf-8')
marker = 'static void processBurst(const uint8_t *data, uint16_t length, uint8_t *debugStages)'
bridge = '''/* Bootloader-compatible OTA management uses address 1, independently of
 * the application RTU slave (address 2). Never accept firmware in the APP.
 * Return: 0 = not an OTA header, 1 = incomplete/invalid, otherwise frame size.
 * Called only in task context; reset only after the complete ACK left RS485. */
static uint16_t processOtaManagement(const uint8_t *data, uint16_t available)
{
  uint16_t length, crc, replyLength;
  uint8_t reply[14] = {1U, 0x10U, 0U, 0xE6U, 0U, 3U, 0U, 0U,
                       0U, 0U, 0U, 0U, 0U, 0U};
  if(available < 6U || data[0] != 1U || data[1] != 0x10U ||
     data[2] != 0U || data[3] != 0xE6U || data[4] != 0U)
    return 0U;
  if(data[5] == 3U) length = 14U;
  else if(data[5] == 4U) length = 20U;
  else return 1U;
  if(available < length) return 1U;
  crc = Modbus_CRC16(data, (uint16_t)(length - 2U));
  if(data[length - 2U] != (uint8_t)crc ||
     data[length - 1U] != (uint8_t)(crc >> 8)) return 1U;
  modbusRxTotalCount++;
  if(length == 14U)
  {
    /* Local status FORBIDDEN=0, session key=0; request cannot override it. */
    replyLength = 12U;
  }
  else
  {
    /* Match the custom 20-byte control layout, not standard FC16. */
    if(data[6] != 0U || data[7] != 0U || data[8] != 0U || data[9] != 8U ||
       data[10] != 0U || data[11] != 1U || data[16] != 0U ||
       (data[17] != 0U && data[17] != 2U)) return length;
    memcpy(reply, data, 6U);
    replyLength = 6U;
  }
  crc = Modbus_CRC16(reply, replyLength);
  reply[replyLength++] = (uint8_t)crc;
  reply[replyLength++] = (uint8_t)(crc >> 8);
  if(modbusRs485Send(reply, replyLength))
  {
    modbusReplyTotalCount++;
    if(length == 20U && data[17] == 2U) nvic_system_reset();
  }
  return length;
}

'''
assert marker in s
already_patched = 'static uint16_t processOtaManagement' in s
if not already_patched:
    s = s.replace(marker, bridge + marker)
needle = '    uint8_t address = data[offset];'
assert needle in s
if not already_patched:
    s = s.replace(needle, needle + '''
    uint16_t otaLength = processOtaManagement(&data[offset], available);
    if(otaLength != 0U)
    {
      offset = (uint16_t)(offset + otaLength);
      continue;
    }
''')
s = s.replace('management uses address 1, independently of', 'management uses address 2, shared with')
s = s.replace('reply[14] = {1U,', 'reply[14] = {2U,')
s = s.replace('data[0] != 1U || data[1] != 0x10U', 'data[0] != 2U || data[1] != 0x10U')
target.write_bytes(s.replace('\r\n', '\n').replace('\n', '\r\n').encode('utf-8'))
print('Updated', target)
