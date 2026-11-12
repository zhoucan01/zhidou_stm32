from pathlib import Path
import shutil
import subprocess

import pytest


@pytest.mark.parametrize("verbose", [0, 1])
def test_dma_pause_resume_and_receive(tmp_path, verbose):
    compiler = shutil.which('gcc')
    if compiler is None:
        pytest.skip('需要GCC运行DMA/USART模型测试')
    root = Path(__file__).resolve().parents[1]
    port = (root / 'bootloader/project/src/wk_usart.c').read_text(encoding='utf-8')
    port = port.split('/* add user code begin 1 */', 1)[1].split('/* add user code end 1 */', 1)[0]
    port = port.replace('SysTick->VAL', 'fake_tick()')
    core = (root / 'bootloader/project/app/modbus.c').read_text(encoding='utf-8')
    core = core.replace('#include "wk_crc.h"', '')
    start = core.index('uint32_t modbus_crc32(')
    end = core.index('\n}', start) + 2
    core = core[:start] + '''uint32_t modbus_crc32(const uint8_t *data,size_t size)
{ (void)data;(void)size;return 0; }
''' + core[end:]
    (tmp_path / 'modbus_port_under_test.inc').write_text(port + '\n' + core, encoding='utf-8')
    exe = tmp_path / 'test_modbus_port_safety.exe'
    result = subprocess.run([
        compiler, '-DBOOT_LOG_VERBOSE=' + str(verbose), '-std=c99', '-Os', '-Wall', '-Wextra', '-Werror',
        '-Wno-pointer-to-int-cast', '-I' + str(tmp_path),
        '-I' + str(root / 'bootloader/project/app'),
        str(root / 'tests/test_modbus_port_safety.c'), '-o', str(exe)
    ], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    result = subprocess.run([str(exe)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'DMA pause/resume and receive scenarios passed' in result.stdout
