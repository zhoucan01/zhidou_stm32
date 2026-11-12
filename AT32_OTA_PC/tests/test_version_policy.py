import hashlib
import struct
import tkinter as tk
from types import SimpleNamespace
from unittest import mock

from app import App
from firmware_pack import prepare_ota_image, inspect_image, OTA_FILENAME, RAW_APP_FILENAME, FACTORY_FILENAME
from stress_test import StressTest, next_version
from test_versions_stress import FakeOTA


def raw():
    return struct.pack('<II', 0x20008000, 0x08004009) + bytes(24)


def test_readonly_ota_and_editable_raw_version(tmp_path):
    interp = tk.Tcl()
    source = tk.StringVar(interp)
    version = tk.StringVar(interp, value='manual')
    entry = mock.Mock()
    App.bind_version_source(source, version, entry)
    path = tmp_path / 'OTA.bin'
    ota, _ = prepare_ota_image(raw(), '1104')
    path.write_bytes(ota)
    source.set(str(path))
    assert version.get() == '1104'
    entry.configure.assert_called_with(state='readonly')
    packed, meta = prepare_ota_image(ota, 'must-not-override')
    assert packed == ota and meta['version'] == '1104'
    app_path = tmp_path / 'codeBin.bin'
    app_path.write_bytes(raw())
    source.set(str(app_path))
    assert version.get() == 'manual'
    entry.configure.assert_called_with(state='normal')


def test_export_default_names_and_raw_app_content(tmp_path):
    assert (FACTORY_FILENAME, RAW_APP_FILENAME, OTA_FILENAME) == ('factory.bin', 'codeBin.bin', 'OTA.bin')
    ota, meta = prepare_ota_image(raw(), '1104')
    source = tmp_path / 'source.bin'
    source.write_bytes(ota)
    ui = SimpleNamespace(load_image=lambda: None, image=ota, metadata=meta,
                         bin_path=SimpleNamespace(get=lambda: str(source)), write=mock.Mock())
    for method, filename, expected in [(App.export_ota_bin, OTA_FILENAME, ota),
                                        (App.export_raw_app, RAW_APP_FILENAME, raw())]:
        target = tmp_path / filename
        with mock.patch('app.filedialog.asksaveasfilename', return_value=str(target)) as save, \
             mock.patch('app.messagebox.showinfo'), mock.patch('app.messagebox.showerror') as error:
            method(ui)
            error.assert_not_called()
            assert save.call_args.kwargs['initialfile'] == filename
            assert target.read_bytes() == expected
    assert source.read_bytes() == ota


def test_stress_uses_header_and_preserves_source(tmp_path):
    ota = FakeOTA()
    stress = StressTest(ota)
    ota.callback = lambda kind, value: stress.stop() if kind == 'stress' else None
    image, _ = prepare_ota_image(raw(), '0313')
    stress.run(image, '9999', 1, 1, tmp_path)
    assert ota.versions == ['0314']
    assert inspect_image(image)['version'] == '0313'
    assert image[1024:1056] == hashlib.sha256(raw()).digest()
    assert next_version('0999') == '1000'
    assert next_version('9999') == '10000'
