import re
import sys
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))

import pytest

from protocol import data_frame, parse_reply, with_crc
from serial_ota import SerialOTA
from stress_test import RunLog


class ReplySerial:
    is_open=True
    def __init__(self,ota):self.ota=ota
    def write(self,frame):
        if frame[1]==0x11:
            body=bytes([2,0x11,1])+frame[2:4]+bytes([32])+b'1.0.2'.ljust(32,b'\0')
        elif frame[1]==0x10:
            body=bytes([2,0x10,0,0xe6,0,3])+bytes(6) if len(frame)==14 else frame[:6]
        else:body=bytes([2,0xaa,1])+frame[14:16]
        self.ota.frames.put(parse_reply(with_crc(body,self.ota.cfg),self.ota.cfg))
        return len(frame)
    def flush(self):pass


def test_command_timeline_pairs_requests_with_replies_and_omits_packets(tmp_path):
    ota=SerialOTA(lambda *args:None);ota.ser=ReplySerial(ota)
    log=RunLog(tmp_path);ota.command_sink=log.write_command;ota.log_sink=log.write
    try:
        ota.query_version();ota.query_status();ota.reset();ota.start(123)
        ota.send(data_frame(ota.cfg,b'firmware bytes',0,0,123));ota.await_reply('data',packet=0)
        # Both human-readable formats are visible on disk before close().
        text=log.path.with_suffix('.commands.log').read_text(encoding='utf-8')
        lines=text.splitlines()
        assert len(lines)==8
        assert '发送：查询运行模式和版本号' in lines[0]
        assert '收到回复：APP 模式，禁止升级，版本=1.0.2' in lines[1]
        assert '发送：查询 OTA 状态' in lines[2]
        assert '禁止升级' in lines[3]
        assert '发送：重启设备' in lines[4] and '重启指令已确认' in lines[5]
        assert '发送：开始 OTA 升级' in lines[6] and '开始升级指令已确认' in lines[7]
        assert all(re.match(r'\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d\.\d{3} ',line) for line in lines)
        assert 'HEX' not in text and 'firmware bytes' not in text and 'CRC' not in text
        assert 'TX HEX' in log.path.with_suffix('.log').read_text(encoding='utf-8')
    finally:log.close()


def test_command_log_records_missing_reply_and_device_rejection():
    ota=SerialOTA(lambda *args:None);events=[];ota.command_sink=events.append
    with pytest.raises(TimeoutError):ota.await_reply('query',.001)
    assert '等待回复超时：查询 OTA 状态' in events[-1]
    ota.frames.put({'type':'data','packet':3,'ack':9})
    ota.await_reply('data',packet=3)
    assert '固件包3写入失败' in events[-1]
