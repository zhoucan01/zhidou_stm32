"""模拟串口验证日志增强不会改变发送帧、应答匹配和进度推进条件。"""
import pytest
from protocol import data_frame, parse_reply, with_crc, num, RxBuffer
from serial_ota import SerialOTA


class SimulatedSerial:
    is_open = True

    def __init__(self, ota, failure=None):
        self.ota = ota
        self.failure = failure
        self.sent = []

    def write(self, frame):
        self.sent.append(frame)
        cfg = self.ota.cfg
        if frame[1] == 0x10:
            reply = (b'\x02\x10\x00\xe6\x00\x03\x00\x01' + bytes(4)
                     if len(frame) == 14 else frame[:6])
        else:
            packet = int.from_bytes(frame[14:16], 'big')
            if packet == 1 and self.failure == 'timeout':
                return len(frame)
            ack = 7 if packet == 1 and self.failure == 'reject' else 1
            reply = b'\x02\xaa' + bytes([ack]) + num(packet, 2)
            if packet == 0:
                # 非预期类型和错包号不能推进当前包。
                self.ota.frames.put({'type': 'control'})
                self.ota.frames.put({'type': 'data', 'packet': 99, 'ack': 1})
        raw = with_crc(reply, cfg)
        self.ota.rx_bytes += len(raw)
        self.ota.frames.put(parse_reply(raw, cfg))
        return len(frame)

    def flush(self):
        pass


@pytest.mark.parametrize('failure', [None, 'reject', 'timeout'])
def test_transfer_trace_preserves_frames_and_ack_progress(failure):
    events = []
    ota = SerialOTA(lambda kind, value: events.append((kind, value)))
    ota.ser = SimulatedSerial(ota, failure)
    image = bytes(2064)
    if failure:
        with pytest.raises((RuntimeError, TimeoutError)):
            ota.upgrade(image, 0x12345678, timeout=0.01)
    else:
        ota.upgrade(image, 0x12345678, timeout=0.01)
    packets = ota.ser.sent[2:]
    assert len(packets) == (2 if failure else 3)
    for i, frame in enumerate(packets):
        assert frame == data_frame(ota.cfg, image[i*1024:(i+1)*1024], 2, i, 0x12345678)
    progress = [value for kind, value in events if kind == 'progress']
    final = progress[-1]
    assert final['completed'] == (1 if failure else 3)
    assert final['bytes'] == (1024 if failure else 2064)
    assert final['total'] == 3 and final['total_bytes'] == 2064
    assert final['percent'] == (33 if failure else 100)
    assert final['tx_bytes'] == sum(map(len, ota.ser.sent))
    assert final['rx_bytes'] == ota.rx_bytes
    logs = '\n'.join(value for kind, value in events if kind == 'log')
    assert '忽略非预期应答' in logs and '忽略包号不一致应答' in logs
    assert 'TX 数据字段' in logs and '镜像偏移' in logs
    if failure:
        assert 'OTA停止' in logs and final['stage'].startswith('失败')
    else:
        assert 'SHA-256校验区' in logs and '原始BIN' in logs
        assert 'OTA传输汇总' in logs and '校验和跳转结果待确认' in final['stage']


def test_rx_diagnostics_preserve_resynchronization_and_fragmentation():
    from protocol import Config
    cfg = Config()
    logs = []
    buf = RxBuffer(cfg, logs.append)
    good = with_crc(b'\x02\xaa\x01\x00\x02', cfg)
    bad = good[:-1] + bytes([good[-1] ^ 1])
    assert buf.feed(b'noise' + bad + good[:3]) == []
    assert any('CRC16不匹配' in line for line in logs)
    assert any('重同步' in line for line in logs)
    assert any('等待拼帧' in line for line in logs)
    result = buf.feed(good[3:])
    assert len(result) == 1 and result[0]['packet'] == 2 and result[0]['ack'] == 1


def test_reader_prints_every_raw_read_before_parsing():
    from protocol import Config
    cfg = Config()
    good = with_crc(b'\x02\xaa\x01\x00\x02', cfg)
    reads = [b'debug\r\n', good[:3], good[3:], good[:-1] + bytes([good[-1] ^ 1])]
    events = []
    ota = SerialOTA(lambda kind, value: events.append((kind, value)))

    class ReceiveSerial:
        in_waiting = 1

        def __init__(self):
            self.pending = iter(reads)

        def read(self, size):
            try:
                return next(self.pending)
            except StopIteration:
                ota.stop_event.set()
                return b''

    ota.ser = ReceiveSerial()
    ota._reader()
    assert [v for k, v in events if k == 'received'] == reads
    logs = [v for k, v in events if k == 'log']
    for raw in reads:
        assert 'RX 原始字节 ' + repr(raw) in logs
        assert any(raw.hex(' ').upper() in line for line in logs)
    assert ota.rx_bytes == sum(map(len, reads))
    assert ota.frames.qsize() == 1 and ota.frames.get()['packet'] == 2


@pytest.mark.parametrize('seconds', [0.25, 2, 30])
def test_upgrade_uses_selected_timeout_for_every_reply(seconds):
    ota = SerialOTA(lambda *args: None)
    ota.ser = SimulatedSerial(ota)
    waits = []
    original = ota.await_reply

    def capture(kind, timeout=4, packet=None):
        waits.append((kind, timeout))
        return original(kind, timeout, packet)

    ota.await_reply = capture
    ota.upgrade(bytes(2064), 0x12345678, timeout=seconds)
    assert waits == [(kind, seconds) for kind in ['query', 'control', 'data', 'data', 'data']]


def test_online_tests_use_selected_timeout():
    ota = SerialOTA(lambda *args: None)
    sent = []
    ota.send = sent.append
    waits = []

    def reply(kind, timeout=4, packet=None):
        waits.append((kind, timeout))
        if kind == 'query':
            return {'status': 1, 'key': 0}
        return {'type': 'control'}

    ota.await_reply = reply
    ota.tests(0x12345678, timeout=0.25)
    assert waits == [(kind, 0.25) for kind in ['query', 'control', 'query']]
    assert len(sent) == 3
    assert all(frame[1] == 0x10 for frame in sent)
    assert int.from_bytes(sent[1][16:18], 'big') == 0


@pytest.mark.parametrize('value', ['', 'abc', 'nan', 'inf', '-inf', 0, -1, None])
def test_invalid_timeout_is_rejected_before_transmission(value):
    from serial_ota import validate_timeout
    with pytest.raises(ValueError):
        validate_timeout(value)
    ota = SerialOTA(lambda *args: None)
    ota.ser = SimulatedSerial(ota)
    for operation in [lambda: ota.query_status(value), lambda: ota.start(0, value),
                      lambda: ota.upgrade(b'A', 0, value), lambda: ota.tests(0, value)]:
        with pytest.raises(ValueError):
            operation()
    assert ota.ser.sent == []


@pytest.mark.parametrize('size', [0, 2048, 2055, 57345])
def test_invalid_image_stops_before_query_or_erase(size):
    ota = SerialOTA(lambda *args: None)
    ota.ser = SimulatedSerial(ota)
    with pytest.raises(ValueError):
        ota.upgrade(bytes(size), 0)
    assert ota.ser.sent == []


def test_new_upgrade_can_restart_an_interrupted_session():
    events = []
    ota = SerialOTA(lambda k, v: events.append((k, v)))
    ota.ser = SimulatedSerial(ota)
    ota.query_status = lambda timeout: {'status': 2, 'key': 123}
    ota.upgrade(bytes(2064), 456, timeout=1)
    assert ota.ser.sent[0][1] == 0x10 and ota.ser.sent[0][16:18] == b'\x00\x01'
    assert ota.ser.sent[1][14:16] == b'\x00\x00'
    assert any('重新擦除' in v for k, v in events if k == 'log')


def test_damaged_device_can_receive_repair_upgrade():
    ota = SerialOTA(lambda *args: None)
    ota.ser = SimulatedSerial(ota)
    ota.query_status = lambda timeout: {'status': 0x55, 'key': 0}
    ota.upgrade(bytes(2064), 456, timeout=1)
    assert ota.ser.sent[0][16:18] == bytes([0, 1])
    assert len(ota.ser.sent) == 4


def test_manual_rollback_uses_operation_three():
    ota = SerialOTA(lambda *args: None)
    ota.ser = SimulatedSerial(ota)
    result = ota.rollback(0x12345678, timeout=1)
    assert result['type'] == 'control'
    assert ota.ser.sent[0][16:18] == bytes([0, 3])
    assert ota.ser.sent[0][12:16] == bytes.fromhex('12345678')


def test_reset_waits_for_control_confirmation():
    ota = SerialOTA(lambda *args: None)
    ota.ser = SimulatedSerial(ota)
    assert ota.reset(123, timeout=1)['type'] == 'control'
    assert ota.ser.sent[0][16:18] == bytes([0, 2])
