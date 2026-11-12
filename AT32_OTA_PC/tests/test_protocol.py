import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import binascii
import pytest
from protocol import *

def test_crc_reference():assert crc16(b'123456789')==0x4B37

def test_frames():
 c=Config();k=0x12345678
 q=query(c);ctl=control(c,1,k);d=data_frame(c,b'123456789',2,1,k)
 assert len(q)==14 and len(ctl)==20 and len(d)==31
 assert all(crc_ok(x,c) for x in (q,ctl,d))
 assert d[2:4]==(31).to_bytes(2,'big')
 assert d[10:12]==(9).to_bytes(2,'big')
 assert d[16:25]==b'123456789'
 assert d[25:29]==(binascii.crc32(b'123456789')&0xffffffff).to_bytes(4,'big')

def test_rx_fragment():
 c=Config();reply=with_crc(b'\x02\xaa\x01\x00\x01',c);r=RxBuffer(c)
 assert r.feed(b'noise'+reply[:3])==[]
 assert r.feed(reply[3:])==[{'type':'data','ack':1,'packet':1,'raw':reply}]

def test_all_sizes():
 c=Config();x=split_firmware(bytes(2050),c)
 assert list(map(len,x))==[1024,1024,2]
 assert [len(data_frame(c,p,3,i+1,0)) for i,p in enumerate(x)]==[1046,1046,24]

def test_keil_image_unchanged():
 from firmware_pack import inspect_image
 import struct
 raw=struct.pack('<II',0x20007000,0x08004009)+b'APP DATA'
 image=b'1.0.1'.ljust(1024,bytes([0]))+bytes(range(32)).ljust(1024,bytes([0]))+raw
 before=image[:]
 meta=inspect_image(image)
 assert image==before
 assert meta['version']=='1.0.1'
 assert meta['digest']==bytes(range(32)).hex()
 assert b''.join(split_firmware(image,Config()))==image

@pytest.mark.parametrize('image',[b'',b'A'*100,bytes(131072),bytes(2064)])
def test_reject_wrong_image(image):
 from firmware_pack import inspect_image
 with pytest.raises(ValueError):inspect_image(image)


def test_version_only_changes_first_partition():
 from firmware_pack import set_version
 import struct
 raw=struct.pack('<II',0x20007000,0x08004009)+b'APP DATA'
 image=b'old'.ljust(1024,bytes([0]))+bytes(range(32)).ljust(1024,bytes([0]))+raw
 result,meta=set_version(image,'2.3.4')
 assert len(result)==len(image)
 assert result[:1024]==b'2.3.4'.ljust(1024,bytes([0]))
 assert result[1024:]==image[1024:]
 assert meta['version']=='2.3.4'
 for version in ['', 'x'*1025, 'bad'+chr(0)+'version']:
  with pytest.raises(ValueError):set_version(image,version)


@pytest.mark.parametrize('size',[32,55296])
def test_raw_app_ota_import_adds_header_and_hash(size):
 from firmware_pack import prepare_ota_image,set_version
 import struct,hashlib
 raw=struct.pack('<II',0x20008000,0x08004009)+bytes(size-8)
 image,meta=prepare_ota_image(raw,'2.1.5')
 assert meta['source_format']=='raw_app' and len(image)==size+2048
 assert image[:1024]==b'2.1.5'.ljust(1024,b'\0')
 assert image[1024:2048]==hashlib.sha256(raw).digest().ljust(1024,b'\0')
 assert image[2048:]==raw
 # Pressure tests can subsequently increment the version without changing the hash.
 updated,_=set_version(image,'2.1.6')
 assert updated[1024:]==image[1024:]


def test_ota_import_keeps_existing_digest():
 from firmware_pack import prepare_ota_image
 import struct
 raw=struct.pack('<II',0x20008000,0x08004009)+bytes(24)
 source=b'old'.ljust(1024,b'\0')+bytes(range(32)).ljust(1024,b'\0')+raw
 image,meta=prepare_ota_image(source,'new')
 assert meta['source_format']=='ota' and image[1024:]==source[1024:]


@pytest.mark.parametrize('base,size',[(0x08000000,32),(0x08012400,32),(0x08004000,55297)])
def test_ota_import_rejects_wrong_raw_target_or_size(base,size):
 from firmware_pack import prepare_ota_image
 import struct
 raw=struct.pack('<II',0x20008000,base+9)+bytes(size-8)
 with pytest.raises(ValueError):prepare_ota_image(raw,'1.0.1')
