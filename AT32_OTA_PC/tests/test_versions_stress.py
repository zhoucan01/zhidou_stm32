import json
import struct
import threading

import pytest

from firmware_pack import inspect_image
from protocol import Config, RxBuffer, version_query, with_crc, num
from serial_ota import SerialOTA
from stress_test import StressTest, RunLog, next_version


def image():
    raw=struct.pack('<II',0x20008000,0x08004009)+b'APP DATA'
    return b'1.0.1'.ljust(1024,b'\0')+bytes(1024)+raw


def reply(cfg,offset,data,mode=1):
    return with_crc(bytes([2,0x11,mode])+num(offset,2,cfg.little_fields)+bytes([len(data)])+data,cfg)


def test_version_stream_noise_fragmentation_and_bounds():
    cfg=Config();frame=reply(cfg,32,b'abc'.ljust(32,b'\0'))
    bad=frame[:-1]+bytes([frame[-1]^1]);buf=RxBuffer(cfg)
    assert buf.feed(b'noise'+bad+frame[:5])==[]
    result=buf.feed(frame[5:])[0]
    assert result['offset']==32 and result['mode']=='app' and result['data'].startswith(b'abc')
    invalid=with_crc(bytes([2,0x11,1,4,0,32])+bytes(32),cfg)
    assert RxBuffer(cfg).feed(invalid)==[]
    assert version_query(cfg)[:6]==bytes([2,0x11,0,0,0,32])
    with pytest.raises(ValueError):version_query(cfg,1000,32)


def test_version_query_decodes_utf8_across_chunks():
    ota=SerialOTA(lambda *args:None)
    text='1.'+'a'*29+'测试'  # UTF-8 character is split at the 32-byte boundary.
    encoded=text.encode().ljust(1024,b'\0');sent=[]
    ota.send=lambda frame:sent.append(frame)
    def receive(kind,timeout,packet=None,offset=None):
        return {'mode':'app','offset':offset,'data':encoded[offset:offset+32]}
    ota.await_reply=receive
    assert ota.query_version()['version']==text
    assert len(sent)==2


def test_increment_and_current_device_version():
    assert next_version('1.0.9')=='1.0.10'
    assert next_version('1.0.1','1.0.50')=='1.0.51'
    assert next_version('1.0.1','custom')=='1.0.2'
    with pytest.raises(ValueError):next_version('v1.0')


class FakeOTA:
    def __init__(self):
        self.cfg=Config();self.stop_event=threading.Event();self.log_sink=None;self.command_sink=None
        self.events=[];self.versions=[];self.current='1.0.1';self.mode='app';self.fail_first=False
        self.callback=lambda kind,value:self.events.append((kind,value))
    def log(self,text):
        if self.log_sink:self.log_sink(text)
    def command(self,text):
        if self.command_sink:self.command_sink(text)
    def query_version(self,timeout):return {'mode':self.mode,'version':self.current}
    def reset_and_query(self,key,timeout):self.mode='bootloader'
    def upgrade(self,packed,key,timeout):
        version=inspect_image(packed)['version'];self.versions.append(version)
        self.log('TX sample HEX 02 AA')
        if self.fail_first and len(self.versions)==1:raise TimeoutError('ACK missing')
        self.current=version;self.mode='app'


@pytest.mark.parametrize('fail_first',[False,True])
def test_stress_runs_until_manual_stop_and_flushes_each_round(tmp_path,fail_first,monkeypatch):
    ota=FakeOTA();ota.fail_first=fail_first;stress=StressTest(ota)
    # Do not sleep in a test. Stop via the same user-facing stop() after 3 attempts.
    def callback(kind,value):
        ota.events.append((kind,value))
        if kind=='stress':
            logs=list(tmp_path.glob('*.jsonl'))
            assert len(logs[0].read_text(encoding='utf-8').splitlines())==value['rounds']
            if value['rounds']==3:stress.stop()
    ota.callback=callback
    monkeypatch.setattr(stress.stop_event,'wait',lambda delay:stress.stop_event.is_set())
    stress.run(image(),'1.0.1',1,1,tmp_path,'firmware_ota.bin')
    assert ota.versions==['1.0.2','1.0.3','1.0.4']
    records=[json.loads(line) for line in next(tmp_path.glob('*.jsonl')).read_text(encoding='utf-8').splitlines()]
    assert records[-1]['event']=='stopped' and records[-1]['rounds']==3
    assert records[-1]['failed']==int(fail_first)
    assert records[-1]['success']==3-int(fail_first)
    assert records[1]['actual']=={'mode':'app','version':'1.0.3'}
    assert 'TX sample HEX 02 AA' in next(p for p in tmp_path.glob('*.log') if not p.name.endswith('.commands.log')).read_text(encoding='utf-8')
    commands=next(tmp_path.glob('*.commands.log')).read_text(encoding='utf-8')
    assert '第3轮成功' in commands and '用户请求停止' in commands
    assert 'HEX' not in commands and 'TX sample' not in commands
    assert ota.log_sink is None and not stress.running
    assert ota.command_sink is None


def test_app_version_mismatch_fails_and_bootloader_never_counts_success():
    ota=FakeOTA();stress=StressTest(ota)
    with pytest.raises(ValueError,match='APP版本不匹配'):stress.wait_app('1.0.9',.01)
    assert stress.last_version=={'mode':'app','version':'1.0.1'}
    ota.mode='bootloader'
    with pytest.raises(TimeoutError,match='等待APP启动超时'):stress.wait_app('1.0.1',.01)


def test_stress_stop_before_start_does_not_write_flash(tmp_path):
    ota=FakeOTA();stress=StressTest(ota);stress.stop()
    stress.run(image(),'1.0.1',1,1,tmp_path)
    assert not ota.versions


def test_log_write_failure_is_not_silent(tmp_path):
    log=RunLog(tmp_path)
    log.error=OSError('disk full')
    try:
        with pytest.raises(OSError,match='日志写入失败'):log.check()
    finally:log.close()
