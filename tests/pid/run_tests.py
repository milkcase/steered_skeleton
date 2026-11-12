"""编译真实 PID 源码及两板 getter，在 ARM 模拟器中测试，不连接电机。"""
import argparse
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile

parser = argparse.ArgumentParser()
parser.add_argument('--unicorn-path', help='可选：临时安装的 unicorn 包目录')
args = parser.parse_args()
if args.unicorn_path:
    sys.path.insert(0, str(Path(args.unicorn_path).resolve()))

from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_PC

test_dir = Path(__file__).resolve().parent
repo = test_dir.parents[1]
compiler = shutil.which('arm-none-eabi-g++')
objcopy = shutil.which('arm-none-eabi-objcopy')
if not compiler or not objcopy:
    raise SystemExit('需要 ARM GNU 工具链，并将其 bin 目录加入 PATH')

with tempfile.TemporaryDirectory(prefix='pid-tests-') as temp_dir:
    for board in ('Chassis', 'Gimbal'):
        elf = Path(temp_dir) / f'{board}.elf'
        binary = elf.with_suffix('.bin')
        # 使用软件浮点，便于在模拟器中执行；生产编译仍使用工程原有的硬件浮点选项。
        command = [compiler, '-mcpu=cortex-m4', '-mthumb', '-mfloat-abi=soft',
                   '-std=gnu++20', '-O1', '-fno-exceptions', '-fno-rtti',
                   '-fno-threadsafe-statics', '-fno-builtin', '-ffunction-sections',
                   '-fdata-sections', '-nostdlib', '-Wl,-e,run_tests',
                   '-Wl,-Ttext=0x10000', '-Wl,-Tdata=0x30000',
                   '-Wl,--section-start=.bss=0x40000', '-Wl,--gc-sections']
        for include in (test_dir / 'mocks', repo / 'RobotComponents/component',
                        repo / board / 'inc', repo / board / 'Instance/component',
                        repo / board / 'Task/component'):
            command += ['-I', str(include)]
        if board == 'Chassis':
            command += ['-DTEST_CHASSIS']
        command += [str(test_dir / 'pid_test.cpp'),
                    str(repo / board / 'Instance/component/ins_pid.cpp'),
                    '-lm', '-lc', '-lgcc', '-o', str(elf)]
        subprocess.run(command, check=True)
        subprocess.run([objcopy, '-O', 'binary', str(elf), str(binary)], check=True)

        entry = struct.unpack_from('<I', elf.read_bytes(), 24)[0]
        emulator = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        emulator.mem_map(0x10000, 0x90000)
        emulator.mem_write(0x10000, binary.read_bytes())
        emulator.mem_map(0x20000000, 0x10000)
        emulator.reg_write(UC_ARM_REG_SP, 0x2000fff0)
        emulator.reg_write(UC_ARM_REG_LR, 0x90001)
        # 限制执行指令数，测试意外卡住时明确失败。
        emulator.emu_start(entry | 1, 0x90000, count=20_000_000)
        if emulator.reg_read(UC_ARM_REG_PC) != 0x90000:
            raise SystemExit(f'{board}: 测试未正常返回')
        result = emulator.reg_read(UC_ARM_REG_R0)
        if result:
            raise SystemExit(f'{board}: pid_test.cpp 第 {result} 行失败')
        print(f'{board}: PASS')
