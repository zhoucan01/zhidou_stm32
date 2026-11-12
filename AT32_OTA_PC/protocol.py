"""AT32 自定义 Modbus OTA 协议；依据用户提供的 modbus.c/h，而不是标准 FC16。"""
from dataclasses import dataclass
import binascii

ADDRESS=2
REGISTER=0x00E6
MAX_PAYLOAD=1024
VERSION_CHUNK=32
ACK_NAMES={1:'写入成功',2:'固件CRC32错误',3:'接收超时',4:'协议版本错误',5:'密钥错误',6:'包序不连续',7:'帧CRC16错误',8:'数据长度异常或超出分区',9:'Flash写入或分区标志保存失败'}
STATUS_NAMES={0:'禁止升级',1:'空闲',2:'下载中',3:'Flash操作中',0x55:'固件均损坏'}

@dataclass(frozen=True)
class Config:
    little_fields: bool=False
    little_crc16: bool=True
    little_crc32: bool=False
    protocol: int=1
    # 本仓库固件 appUpgrade() 的 currentPackage 静态0起始，首包必须为0
    first_packet: int=0
    payload: int=1024
    def __post_init__(self):
        if not 1<=self.payload<=1024: raise ValueError('单包有效载荷必须在 1..1024 字节')
        if self.first_packet not in (0,1): raise ValueError('首包必须是0或1')
    @property
    def field_order(self): return 'little' if self.little_fields else 'big'

def num(value, width, little=False):
    return int(value).to_bytes(width,'little' if little else 'big')

def val(data, little=False):
    return int.from_bytes(data,'little' if little else 'big')

def crc16(data):
    crc=0xffff
    for b in data:
        crc^=b
        for _ in range(8): crc=(crc>>1)^(0xA001 if crc&1 else 0)
    return crc

def with_crc(data,cfg): return data+num(crc16(data),2,cfg.little_crc16)

def crc_ok(frame,cfg):
    return len(frame)>=4 and crc16(frame[:-2])==val(frame[-2:],cfg.little_crc16)

def query(cfg, status=0, key=0):
    o=cfg.field_order
    return with_crc(bytes([ADDRESS,0x10])+num(REGISTER,2,cfg.little_fields)+num(3,2,cfg.little_fields)+num(status,2,cfg.little_fields)+num(key,4,cfg.little_fields),cfg)

def version_query(cfg,offset=0,count=VERSION_CHUNK):
    if not 0<=offset<1024 or not 1<=count<=VERSION_CHUNK or offset+count>1024:
        raise ValueError('版本查询范围无效')
    return with_crc(bytes([ADDRESS,0x11])+num(offset,2,cfg.little_fields)+num(count,2,cfg.little_fields),cfg)

def control(cfg,operation,key,status=1):
    return with_crc(bytes([ADDRESS,0x10])+num(REGISTER,2,cfg.little_fields)+num(4,2,cfg.little_fields)+num(8,4,cfg.little_fields)+num(status,2,cfg.little_fields)+num(key,4,cfg.little_fields)+num(operation,2,cfg.little_fields),cfg)

def data_frame(cfg, payload, total, packet, key):
    if not 1<=len(payload)<=cfg.payload: raise ValueError('固件分包长度不合法')
    # total 字段语义 = 末包包号 = 包数-1（单包固件为0）；设备以 currentPackage > total 结束
    if not 0<=total<=65535: raise ValueError('末包包号（total字段）超出uint16范围')
    if not 0<=packet<=65535: raise ValueError('包号超出uint16范围')
    # L= payload+22，帧内offset10=payload长度，数据从offset16开始
    prefix=(bytes([ADDRESS,0xAA])+num(len(payload)+22,2,cfg.little_fields)
        +num(cfg.protocol,2,cfg.little_fields)+num(key,4,cfg.little_fields)
        +num(len(payload),2,cfg.little_fields)+num(total,2,cfg.little_fields)
        +num(packet,2,cfg.little_fields)+payload+num(binascii.crc32(payload)&0xffffffff,4,cfg.little_crc32))
    return with_crc(prefix,cfg)

def split_firmware(firmware,cfg):
    chunks=[firmware[i:i+cfg.payload] for i in range(0,len(firmware),cfg.payload)]
    if not chunks: raise ValueError('固件文件为空')
    if len(chunks)>65535 or cfg.first_packet+len(chunks)-1>65535: raise ValueError('固件包数超过协议范围')
    return chunks

class RxBuffer:
    """提取数据/控制/状态/版本应答，支持分段接收、粘包和噪音。"""
    def __init__(self,cfg,trace=None):
        self.cfg=cfg;self.buf=bytearray();self.trace=trace
    def feed(self,data):
        self.buf.extend(data)
        frames=[];discarded=0
        while self.buf:
            if self.buf[0]!=ADDRESS:
                del self.buf[0];discarded+=1;continue
            if len(self.buf)<2:break
            if self.buf[1]==0x11:
                if len(self.buf)<6:break
                lengths=[8+self.buf[5]] if 1<=self.buf[5]<=VERSION_CHUNK else []
            elif self.buf[1]==0xAA: lengths=[7]
            elif self.buf[1]==0x10:
                if len(self.buf)<6:break
                count=val(self.buf[4:6],self.cfg.little_fields)
                lengths=[14] if count==3 else ([8] if count==4 else [])
            else: lengths=[]
            if not lengths:
                del self.buf[0];discarded+=1;continue
            length=lengths[0]
            if len(self.buf)<length:break
            frame=bytes(self.buf[:length])
            if crc_ok(frame,self.cfg):
                try:parsed=parse_reply(frame,self.cfg)
                except ValueError:
                    del self.buf[0];discarded+=1;continue
                frames.append(parsed);del self.buf[:length]
            else:
                if self.trace:
                    self.trace('RX CRC16不匹配：候选=%s，收到=0x%04X，计算=0x%04X；跳过1字节重新找帧头'%(
                        frame.hex(' ').upper(),val(frame[-2:],self.cfg.little_crc16),crc16(frame[:-2])))
                del self.buf[0];discarded+=1
        if len(self.buf)>8192:
            discarded+=len(self.buf)-64;self.buf=self.buf[-64:]
        if self.trace:
            if discarded:self.trace(f'RX 重同步：丢弃 {discarded} 字节噪声/非法候选')
            if self.buf:self.trace(f'RX 等待拼帧：已缓存 {len(self.buf)} 字节，内容={self.buf.hex(" ").upper()}')
        return frames

def parse_reply(frame,cfg):
    if not crc_ok(frame,cfg) or frame[0]!=ADDRESS: raise ValueError('应答CRC16/地址错误')
    if frame[1]==0x11 and len(frame)>=9:
        offset=val(frame[3:5],cfg.little_fields);count=frame[5]
        if frame[2] not in (0,1) or not 1<=count<=VERSION_CHUNK or offset+count>1024 or len(frame)!=count+8:
            raise ValueError('版本应答格式错误')
        return {'type':'version','mode':'app' if frame[2] else 'bootloader','offset':offset,'data':frame[6:-2],'raw':frame}
    if frame[1]==0xAA and len(frame)==7:
        return {'type':'data','ack':frame[2],'packet':val(frame[3:5],cfg.little_fields),'raw':frame}
    if frame[1]==0x10 and len(frame) in (8,14):
        if val(frame[2:4],cfg.little_fields)!=REGISTER:raise ValueError('应答寄存器不匹配')
        if len(frame)==14:
            if val(frame[4:6],cfg.little_fields)!=3:raise ValueError('查询数量不匹配')
            return {'type':'query','status':val(frame[6:8],cfg.little_fields),'key':val(frame[8:12],cfg.little_fields),'raw':frame}
        if val(frame[4:6],cfg.little_fields)!=4:raise ValueError('写入数量不匹配')
        return {'type':'control','raw':frame}
    raise ValueError('未知应答帧')
