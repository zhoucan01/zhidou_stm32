import pytest
from protocol import Config, query, control, data_frame
from serial_ota import SerialOTA


def test_all_ota_requests_use_station_two():
    cfg = Config()
    assert all(frame[0] == 2 for frame in (
        query(cfg), control(cfg, 2, 0), data_frame(cfg, b'A', 0, 0, 0)))


def test_reset_query_waits_for_bootloader(monkeypatch):
    ota = SerialOTA(lambda *args: None)
    calls = []
    states = iter([TimeoutError(), {'status': 0}, {'status': 1}])
    monkeypatch.setattr(ota, 'reset', lambda key, timeout: calls.append(key))
    monkeypatch.setattr(ota.stop_event, 'wait', lambda delay: False)
    def receive(timeout):
        state = next(states)
        if isinstance(state, Exception):
            raise state
        return state
    monkeypatch.setattr(ota, 'query_status', receive)
    assert ota.reset_and_query(123)['status'] == 1
    assert calls == [123]


def test_reset_query_does_not_hide_reset_failure(monkeypatch):
    ota = SerialOTA(lambda *args: None)
    def fail(*args):
        raise TimeoutError('reset ACK missing')
    monkeypatch.setattr(ota, 'reset', fail)
    with pytest.raises(TimeoutError, match='reset ACK missing'):
        ota.reset_and_query()
