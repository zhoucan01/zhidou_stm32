from pathlib import Path
import shutil
import subprocess

import pytest


def test_application_ota_management(tmp_path):
    compiler = shutil.which('gcc')
    if not compiler:
        pytest.skip('GCC required')
    source = Path(__file__).resolve().parents[2] / 'at32f415rbt7/code/task/modbusTask.c'
    text = source.read_text(encoding='utf-8')
    bridge = text[text.index('static uint16_t processVersionQuery('):text.index('static void processBurst(')]
    harness = r'''
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
static unsigned modbusRxTotalCount, modbusReplyTotalCount, sent, resets;
static uint8_t output[40];
#define MODBUS_SLAVE_ADDRESS 2U
static const uint8_t header[1024]="9.8.765";
#define MODBUS_VERSION_BASE header
static uint16_t Modbus_CRC16(const uint8_t *p, uint16_t n) {
  uint16_t c=0xffff; unsigned i;
  while(n--) { c^=*p++; for(i=0;i<8;i++) c=(c>>1)^((c&1)?0xa001:0); } return c;
}
static bool modbusRs485Send(const uint8_t *p,uint16_t n) {
  memcpy(output,p,n);sent=n;return true;
}
static void nvic_system_reset(void) { assert(sent==8);resets++; }
'''
    harness += bridge + r'''
static void crc(uint8_t *p,unsigned n) {
  uint16_t c=Modbus_CRC16(p,n-2);p[n-2]=(uint8_t)c;p[n-1]=(uint8_t)(c>>8);
}
int main(void) {
  uint8_t q[14]={2,0x10,0,0xe6,0,3,0xff,0xff,1,2,3,4};
  uint8_t r[20]={2,0x10,0,0xe6,0,4,0,0,0,8,0,1,1,2,3,4,0,2};
  uint8_t v[8]={2,0x11,0,0,0,32};
  crc(q,14);assert(processOtaManagement(q,14)==14);assert(sent==14);
  assert(output[0]==2 && output[6]==0 && output[7]==0);
  assert(output[8]==0 && output[9]==0 && output[10]==0 && output[11]==0);
  sent=0;q[13]^=1;assert(processOtaManagement(q,14)==1);assert(sent==0);
  crc(r,20);assert(processOtaManagement(r,20)==20);assert(resets==1);
  assert(output[0]==2 && output[5]==4);
  sent=0;r[17]=1;crc(r,20);processOtaManagement(r,20);assert(sent==0 && resets==1);
  r[0]=1;crc(r,20);assert(processOtaManagement(r,20)==0);
  q[0]=2;q[3]=0;assert(processOtaManagement(q,14)==0);
  crc(v,8);assert(processVersionQuery(v,8)==8);assert(sent==40);
  assert(output[0]==2 && output[1]==0x11 && output[2]==1 && output[5]==32);
  assert(memcmp(output+6,header,32)==0 && Modbus_CRC16(output,40)==0);
  sent=0;v[7]^=1;assert(processVersionQuery(v,8)==1 && sent==0);
  v[2]=4;v[3]=0;crc(v,8);assert(processVersionQuery(v,8)==8 && sent==0);
  assert(processVersionQuery(v,3)==1 && sent==0);
  return 0;
}
'''
    path = tmp_path / 'bridge.c'
    path.write_text(harness, encoding='utf-8')
    exe = tmp_path / 'bridge.exe'
    subprocess.run([compiler, '-std=c99', '-Wall', '-Wextra', '-Werror', str(path), '-o', str(exe)], check=True, capture_output=True)
    subprocess.run([str(exe)], check=True, capture_output=True)
