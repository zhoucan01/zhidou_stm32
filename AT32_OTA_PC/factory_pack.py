"""生成固定 A 运行、B 备份布局的完整烧录文件；不改变串口 OTA 包格式。"""
import hashlib
import struct

FLASH_BASE = 0x08000000
FLASH_SIZE = 128 * 1024
BOOT_SIZE = 13 * 1024
VERSION_OFFSET = 0x3800
HASH_OFFSET = 0x3C00
APP_OFFSET = 0x4000
APP_RECORD_OFFSET = 0x11800
APP_MAX_SIZE = APP_RECORD_OFFSET - APP_OFFSET
IMAGE_VALID = 0x53484132
APP_FLASH_BASE = FLASH_BASE + VERSION_OFFSET
APP_FLASH_END = FLASH_BASE + APP_RECORD_OFFSET + 1024


def _check_vectors(data, base, name):
    if len(data) < 8:
        raise ValueError(f'{name} 至少需要8字节向量表')
    stack, entry = struct.unpack_from('<II', data)
    if not (0x20000000 < stack <= 0x20008000 and stack % 8 == 0):
        raise ValueError(f'{name} 初始栈地址无效')
    if not (entry & 1 and base + 8 <= (entry & ~1) < base + len(data)):
        raise ValueError(f'{name} 入口无效，必须链接到 0x{base:08X}；请选择原始BIN')


def _app_header(app, version):
    if len(app) > APP_MAX_SIZE:
        raise ValueError('APP 超过55296字节，会覆盖长度记录扇区')
    _check_vectors(app, FLASH_BASE + APP_OFFSET, 'APP')
    encoded = version.encode('utf-8')
    if not encoded or len(encoded) > 1024 or not version.isprintable() or '\0' in version:
        raise ValueError('版本号必须为可显示文本，UTF-8 编码不能超过1024字节')
    digest = hashlib.sha256(app).digest()
    return encoded.ljust(1024, b'\0') + digest.ljust(1024, b'\0'), digest


def build_app_flash_image(source, version):
    """接受原始APP或Keil OTA BIN，导出仅覆盖A和A记录页的烧录镜像。"""
    try:
        _check_vectors(source, FLASH_BASE + APP_OFFSET, 'APP')
        app = source
    except ValueError:
        from firmware_pack import inspect_image, HEADER_SIZE
        version = inspect_image(source)['version']
        app = source[HEADER_SIZE:]
    header, digest = _app_header(app, version)
    image = bytearray(b'\xff' * (APP_FLASH_END - APP_FLASH_BASE))
    image[:2048] = header
    offset = APP_OFFSET - VERSION_OFFSET
    image[offset:offset + len(app)] = app
    struct.pack_into('<II', image, APP_RECORD_OFFSET - VERSION_OFFSET, IMAGE_VALID, len(app))
    return bytes(image), {'app_size':len(app), 'digest':digest.hex(), 'version':version,
                         'size':len(image), 'base_address':APP_FLASH_BASE, 'end_address':APP_FLASH_END}


def build_factory_image(bootloader, app, version):
    """合并原始 BIN，空洞和 B 填 FF；记录只提交已计算摘要的 A。"""
    if len(bootloader) > BOOT_SIZE:
        raise ValueError('Bootloader 超过13KB，会覆盖保留扇区')
    _check_vectors(bootloader, FLASH_BASE, 'Bootloader')
    header, digest = _app_header(app, version)
    image = bytearray(b'\xff' * FLASH_SIZE)
    image[:len(bootloader)] = bootloader
    image[VERSION_OFFSET:APP_OFFSET] = header
    image[APP_OFFSET:APP_OFFSET + len(app)] = app
    struct.pack_into('<II', image, APP_RECORD_OFFSET, IMAGE_VALID, len(app))
    return bytes(image), {
        'boot_size': len(bootloader), 'app_size': len(app),
        'digest': digest.hex(), 'version': version, 'size': len(image),
    }


def _hex_record(address, kind, payload):
    record = bytes([len(payload)]) + address.to_bytes(2, 'big') + bytes([kind]) + payload
    return ':' + (record + bytes([-sum(record) & 0xFF])).hex().upper()


def to_intel_hex(image, base_address=FLASH_BASE):
    """导出指定起始地址的 Intel HEX，只覆盖传入镜像的地址范围。"""
    if not image or base_address < FLASH_BASE or base_address + len(image) > FLASH_BASE + FLASH_SIZE:
        raise ValueError('烧录镜像地址范围必须位于128KB Flash内')
    lines = []
    upper = None
    for offset in range(0, len(image), 16):
        address = base_address + offset
        if address >> 16 != upper:
            upper = address >> 16
            lines.append(_hex_record(0, 4, upper.to_bytes(2, 'big')))
        lines.append(_hex_record(address & 0xFFFF, 0, image[offset:offset + 16]))
    lines.append(_hex_record(0, 1, b''))
    return '\n'.join(lines) + '\n'
