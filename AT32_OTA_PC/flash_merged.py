"""烧录合并镜像，使用与原 flash.bat 相同的 pyOCD / SWD 方式。"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import struct
import subprocess
import sys

from factory_pack import FLASH_BASE, FLASH_SIZE, APP_OFFSET, APP_RECORD_OFFSET, IMAGE_VALID


def find_local_pack():
    """复用已安装的 Keil 芯片包，支持展开目录，无需联网安装。"""
    roots = [Path('D:/application/KEIL'), Path('C:/Keil_v5/ARM/PACK'),
             Path(os.environ.get('LOCALAPPDATA', str(Path.home()))) / 'Arm/Packs']
    for name in ('CMSIS_PACK_ROOT', 'KEIL_PACK_ROOT'):
        if os.environ.get(name):
            roots.insert(0, Path(os.environ[name]))
    for root in roots:
        for family in ('ArteryTek/AT32F415_DFP', 'Keil/AT32F4xx_DFP'):
            for folder in sorted((root / family).glob('*'), reverse=True):
                if folder.is_dir() and any(folder.glob('*.pdsc')):
                    return str(folder)
    return None


def select_target(targets):
    # ArteryTek 新包的器件名前带“-”，pyOCD 会将它转为“_”。
    for target in targets:
        if target.get('name', '').lower().lstrip('_-') == 'at32f415rbt7':
            return target['name']
    raise ValueError('所选芯片包没有AT32F415RBT7，请指定正确的AT32F415芯片包')


def read_merged(path):
    """擦除前检查全片覆盖、记录和摘要，防止把单独 APP 当作完整镜像。"""
    if path.suffix.lower() == '.bin':
        image = path.read_bytes()
    elif path.suffix.lower() == '.hex':
        image = bytearray(b'\xff' * FLASH_SIZE)
        seen = bytearray(FLASH_SIZE)
        upper = 0
        eof = False
        for line in path.read_text(encoding='ascii').splitlines():
            if not line.strip():
                continue
            if eof or not line.startswith(':'):
                raise ValueError('HEX 格式错误')
            record = bytes.fromhex(line[1:])
            if len(record) < 5 or len(record) != record[0] + 5 or sum(record) & 255:
                raise ValueError('HEX 长度或校验和错误')
            size, kind = record[0], record[3]
            address = int.from_bytes(record[1:3], 'big')
            if kind == 4 and size == 2 and address == 0:
                upper = int.from_bytes(record[4:6], 'big') << 16
            elif kind == 0:
                offset = upper + address - FLASH_BASE
                if not 0 <= offset <= FLASH_SIZE - size or any(seen[offset:offset + size]):
                    raise ValueError('HEX 地址越界或重复')
                image[offset:offset + size] = record[4:4 + size]
                seen[offset:offset + size] = b'\1' * size
            elif kind == 1 and size == 0 and address == 0:
                eof = True
            else:
                raise ValueError('请选择上位机生成的完整合并 HEX')
        if not eof or not all(seen):
            raise ValueError('HEX 必须完整覆盖128KB，请使用合并导出的文件')
    else:
        raise ValueError('请选择合并后的 .bin 或 .hex')
    if len(image) != FLASH_SIZE:
        raise ValueError('合并镜像必须为128KB，不能选择单独APP或OTA包')
    magic, length = struct.unpack_from('<II', image, APP_RECORD_OFFSET)
    if magic != IMAGE_VALID or not 8 <= length <= APP_RECORD_OFFSET - APP_OFFSET:
        raise ValueError('A区长度记录无效')
    digest = hashlib.sha256(image[APP_OFFSET:APP_OFFSET + length]).digest()
    if digest != image[0x3C00:0x3C20]:
        raise ValueError('APP SHA-256与记录不一致')
    for offset, limit in ((0, 13 * 1024), (APP_OFFSET, APP_OFFSET + length)):
        stack, entry = struct.unpack_from('<II', image, offset)
        if not (0x20000000 < stack <= 0x20008000 and stack % 8 == 0 and
                entry & 1 and FLASH_BASE + offset + 8 <= (entry & ~1) < FLASH_BASE + limit):
            raise ValueError('Bootloader或APP向量无效')
    return bytes(image)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image', nargs='?', help='合并的128KB BIN/HEX；省略则打开文件选择框')
    parser.add_argument('--pack', default=os.environ.get('PYOCD_PACK'), help='AT32 .pack文件或已安装芯片包目录，默认自动查找')
    parser.add_argument('--uid', help='多个下载器时指定序列号')
    parser.add_argument('--dry-run', action='store_true', help='仅检查镜像并打印命令，不访问下载器')
    args = parser.parse_args()
    if not args.image:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk(); root.withdraw()
        args.image = filedialog.askopenfilename(title='选择合并烧录文件', filetypes=[('合并 BIN/HEX', '*.bin *.hex')])
        root.destroy()
        if not args.image:
            return 0
    path = Path(args.image).resolve()
    read_merged(path)
    pack = []
    if not args.pack:
        args.pack = find_local_pack()
    if args.pack:
        if not Path(args.pack).exists():
            raise ValueError('指定的AT32芯片包文件或目录不存在：' + args.pack)
        pack = ['--pack', str(Path(args.pack).resolve())]
        print('使用本地芯片包：', args.pack, flush=True)
    command = [sys.executable, '-m', 'pyocd', 'flash', str(path),
               '--target', 'at32f415rbt7', '--frequency', '1000000',
               '--connect', 'under-reset', '--erase', 'chip', '--no-wait'] + pack
    if path.suffix.lower() == '.bin':
        command += ['--base-address', '0x08000000']
    if args.uid:
        command += ['--uid', args.uid]
    print('文件：', path, flush=True)
    print('整片128KB烧录：将清除原Bootloader、APP和B区备份。', flush=True)
    print(subprocess.list2cmdline(command), flush=True)
    if args.dry_run:
        return 0
    # 只查询工具支持的芯片，不在缺少算法时先擦除芯片。
    check = subprocess.run([sys.executable, '-m', 'pyocd', 'json', '--targets'] + pack,
                           capture_output=True, text=True)
    if check.returncode:
        raise RuntimeError(check.stderr or 'pyOCD不可用，请运行：python -m pip install pyocd')
    targets = json.loads(check.stdout).get('targets', [])
    target = select_target(targets)
    command[command.index('--target') + 1] = target
    print('实际烧录目标：', target, flush=True)
    result = subprocess.run(command)
    if result.returncode:
        return result.returncode
    print('烧录完成，正在复位设备。', flush=True)
    return subprocess.run([sys.executable, '-m', 'pyocd', 'reset', '--target', target,
                           '--frequency', '1000000', '--no-wait'] + pack +
                          (['--uid', args.uid] if args.uid else [])).returncode


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, RuntimeError) as exc:
        print('[ERROR]', exc, file=sys.stderr)
        sys.exit(1)
