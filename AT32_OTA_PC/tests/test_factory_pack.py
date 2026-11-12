import hashlib
import struct
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from factory_pack import build_factory_image, build_app_flash_image, to_intel_hex


def raw(base, size=32):
    return struct.pack('<II', 0x20008000, base + 9) + bytes(size - 8)


def test_factory_layout_and_boot_validation_metadata():
    boot, app = raw(0x08000000), raw(0x08004000)
    image, meta = build_factory_image(boot, app, '测试1.2')
    assert len(image) == 131072
    assert image[:len(boot)] == boot
    assert image[len(boot):0x3800] == b'\xff' * (0x3800 - len(boot))
    assert image[0x3800:0x3C00] == '测试1.2'.encode().ljust(1024, b'\0')
    assert image[0x3C00:0x4000] == hashlib.sha256(app).digest().ljust(1024, b'\0')
    assert image[0x4000:0x4000 + len(app)] == app
    assert struct.unpack_from('<II', image, 0x11800) == (0x53484132, len(app))
    assert image[0x11808:] == b'\xff' * (131072 - 0x11808)
    assert meta['digest'] == hashlib.sha256(app).hexdigest()


@pytest.mark.parametrize('size',[32,55296])
@pytest.mark.parametrize('ota',[False,True])
def test_app_only_flash_layout_and_preservation(size,ota):
    app=raw(0x08004000,size)
    # An obsolete source digest must be recomputed for direct Flash export.
    source=b'old'.ljust(1024,b'\0')+bytes(1024)+app if ota else app
    image,meta=build_app_flash_image(source,'测试2.3')
    assert meta['base_address']==0x08003800 and len(image)==58368
    assert image[:1024]==(b'old'.ljust(1024,b'\0') if ota else '测试2.3'.encode().ljust(1024,b'\0'))
    assert image[1024:2048]==hashlib.sha256(app).digest().ljust(1024,b'\0')
    assert image[2048:2048+size]==app
    assert struct.unpack_from('<II',image,0x11800-0x3800)==(0x53484132,size)
    assert image[0x11808-0x3800:]==b'\xff'*1016
    flash=bytearray(b'\x55'*131072)
    upper=0;seen=set()
    for line in to_intel_hex(image,meta['base_address']).splitlines():
        r=bytes.fromhex(line[1:]);assert sum(r)%256==0
        if r[3]==4:upper=int.from_bytes(r[4:6],'big')<<16
        elif r[3]==0:
            offset=upper+int.from_bytes(r[1:3],'big')-0x08000000
            assert 0x3800<=offset<0x11c00
            flash[offset:offset+r[0]]=r[4:-1];seen.update(range(offset,offset+r[0]))
    assert flash[:0x3800]==b'\x55'*0x3800
    assert flash[0x11c00:]==b'\x55'*(131072-0x11c00)
    assert flash[0x3800:0x11c00]==image and len(seen)==len(image)


@pytest.mark.parametrize('source,version',[
    (raw(0x08000000),'1'),(raw(0x08012400),'1'),
    (raw(0x08004000,55297),'1'),(b'','1'),
    (raw(0x08004000),''),(raw(0x08004000),'x'*1025),
])
def test_app_only_rejects_wrong_inputs(source,version):
    with pytest.raises(ValueError):build_app_flash_image(source,version)


def test_hex_round_trip_addresses_and_checksums():
    image, _ = build_factory_image(raw(0x08000000), raw(0x08004000, 55296), '1')
    restored = bytearray(131072)
    seen = set()
    upper = 0
    lines = to_intel_hex(image).splitlines()
    for line in lines:
        record = bytes.fromhex(line[1:])
        assert sum(record) % 256 == 0 and len(record) == record[0] + 5
        size, kind = record[0], record[3]
        if kind == 4:
            upper = int.from_bytes(record[4:6], 'big') << 16
        elif kind == 0:
            offset = upper + int.from_bytes(record[1:3], 'big') - 0x08000000
            assert 0 <= offset <= len(image) - size
            restored[offset:offset + size] = record[4:4 + size]
            seen.update(range(offset, offset + size))
    assert lines[-1] == ':00000001FF'
    assert len(seen) == len(image) and restored == image


@pytest.mark.parametrize('boot,app,version', [
    (b'', raw(0x08004000), '1'),
    (raw(0x08000000, 13313), raw(0x08004000), '1'),
    (raw(0x08000000), raw(0x08004000, 55297), '1'),
    (raw(0x08004000), raw(0x08004000), '1'),
    (raw(0x08000000), raw(0x08012400), '1'),
    (raw(0x08000000), b'\0' * 32, '1'),
    (raw(0x08000000), b'1'.ljust(2048, b'\0') + raw(0x08004000), '1'),
    (raw(0x08000000), raw(0x08004000), ''),
    (raw(0x08000000), raw(0x08004000), '中' * 342),
])
def test_factory_rejects_invalid_input(boot, app, version):
    with pytest.raises(ValueError):
        build_factory_image(boot, app, version)


def test_flash_merged_checks_both_formats_and_dry_run(tmp_path):
    import subprocess
    from flash_merged import read_merged
    image, _ = build_factory_image(raw(0x08000000), raw(0x08004000), '1')
    for suffix in ('.bin', '.hex'):
        path = tmp_path / ('full image' + suffix)
        if suffix == '.bin':
            path.write_bytes(image)
        else:
            path.write_text(to_intel_hex(image), encoding='ascii')
        assert read_merged(path) == image
        result = subprocess.run([sys.executable, str(Path(__file__).parents[1] / 'flash_merged.py'),
                                 str(path), '--dry-run'], capture_output=True)
        assert result.returncode == 0, result.stderr
        assert b'--erase chip' in result.stdout
        assert (b'--base-address 0x08000000' in result.stdout) == (suffix == '.bin')
    bad = tmp_path / 'bad.bin'
    bad.write_bytes(raw(0x08004000))
    with pytest.raises(ValueError):
        read_merged(bad)
    damaged = bytearray(image); damaged[0x4010] ^= 1
    bad.write_bytes(damaged)
    with pytest.raises(ValueError, match='SHA-256'):
        read_merged(bad)
    bad_hex = tmp_path / 'bad.hex'
    bad_hex.write_text(to_intel_hex(image).replace(':020000040800F2', ':020000040800F3'), encoding='ascii')
    with pytest.raises(ValueError):
        read_merged(bad_hex)


def test_pack_target_name_variants():
    from flash_merged import select_target
    for name in ('at32f415rbt7', '_at32f415rbt7'):
        assert select_target([{'name': name}]) == name
    with pytest.raises(ValueError):
        select_target([{'name': '_at32f415r8t7'}, {'name': '_at32f415rbt7_7'}])
