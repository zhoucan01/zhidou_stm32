"""在PC上编译实际OTA业务代码，用Flash模型注入失败，不访问真实设备。"""
from pathlib import Path
import shutil
import subprocess

import pytest


def test_upgrade_flash_safety(tmp_path):
    compiler = shutil.which('gcc')
    if compiler is None:
        pytest.skip('需要GCC运行Flash故障模型测试')
    root = Path(__file__).resolve().parents[1]
    source = (root / 'bootloader/project/app/upgrade.c').read_text(encoding='utf-8')
    # 只截取升级业务，应用跳转汇编和硬件SHA读数另由板上及算法测试验证。
    end = source.index('static int appVectorValid(')
    end = source.rfind('/**', 0, end)
    source = source[:end]
    source = '\n'.join(line for line in source.splitlines()
                       if not line.startswith('#include "at32f415'))
    original = 'uint8_t appFlag = *(volatile uint8_t *)APPADDRFLAG;'
    assert original in source
    source = source.replace(original, 'uint8_t appFlag = model_read(APPADDRFLAG);')
    source = source.replace('*(volatile uint32_t *)address', 'model_word(address)')
    source = source.replace('*(volatile uint8_t *)(address + i)', 'model_read(address + i)')
    source = source.replace('(const uint8_t *)source,', 'model_pointer(source),')
    (tmp_path / 'upgrade_logic_under_test.inc').write_text(source, encoding='utf-8')
    exe = tmp_path / 'test_upgrade_safety.exe'
    result = subprocess.run([
        compiler, '-DBOOT_LOG_ENABLED=0', '-std=c99', '-Os', '-Wall', '-Wextra', '-Werror',
        '-I' + str(tmp_path), '-I' + str(root / 'bootloader/project/app'),
        '-I' + str(root / 'bootloader/project/algorithm'),
        str(root / 'tests/test_upgrade_safety.c'), '-o', str(exe)
    ], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    result = subprocess.run([str(exe)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'OTA safety scenarios passed' in result.stdout
