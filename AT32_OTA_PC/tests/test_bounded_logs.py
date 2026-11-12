from pathlib import Path
from types import SimpleNamespace
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))

import pytest

import stress_test as module
from stress_test import BoundedLog, RunLog


def test_rotation_is_bounded_and_keeps_latest_records(tmp_path):
    path=tmp_path/'test.log';old=tmp_path/'older_run.log';old.write_text('preserve')
    log=BoundedLog(path,128)
    try:
        for index in range(1000):log.write(f'{index:04d} '+('x'*40)+'\n')
        log.flush()
        files=list(tmp_path.glob('test.log*'))
        assert len(files)==4 and sum(p.stat().st_size for p in files)<=4*128
        assert path.read_text().splitlines()[-1].startswith('0999')
        assert old.read_text()=='preserve'
    finally:log.close()


def test_start_refuses_to_add_to_overbudget_history(tmp_path,monkeypatch):
    old=tmp_path/'ota_stress_20261031_123456_123456.log';old.write_bytes(b'old data')
    monkeypatch.setattr(module,'DIRECTORY_LOG_BUDGET',module.RUN_LOG_BUDGET)
    with pytest.raises(OSError,match='512MiB'):RunLog(tmp_path)
    assert old.read_bytes()==b'old data' and len(list(tmp_path.iterdir()))==1


def test_low_space_prevents_creation_and_stops_active_writes(tmp_path,monkeypatch):
    monkeypatch.setattr(module.shutil,'disk_usage',lambda path:SimpleNamespace(free=0))
    with pytest.raises(OSError,match='256MiB'):RunLog(tmp_path)
    assert not list(tmp_path.iterdir())
    monkeypatch.setattr(module.shutil,'disk_usage',lambda path:SimpleNamespace(free=1024**3))
    log=RunLog(tmp_path)
    try:
        log.write('before');size=log.detail.size
        log.next_space_check=0
        monkeypatch.setattr(module.shutil,'disk_usage',lambda path:SimpleNamespace(free=0))
        log.write('must not write')
        with pytest.raises(OSError,match='256MiB'):log.check()
        assert log.detail.size==size
    finally:log.close()
