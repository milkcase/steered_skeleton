"""编译共享云台位置环及真实 PID，以记录型电机替身在 ARM 模拟器验证。"""
import argparse
from pathlib import Path
import shutil
import struct
import subprocess
import sys
sys.stdout.reconfigure(encoding='utf-8')

parser=argparse.ArgumentParser()
parser.add_argument('--unicorn-path',help='可选：临时安装的 unicorn 包目录')
args=parser.parse_args()
if args.unicorn_path:
    sys.path.insert(0,str(Path(args.unicorn_path).resolve()))
from unicorn import Uc,UC_ARCH_ARM,UC_MODE_THUMB,UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_SP,UC_ARM_REG_LR,UC_ARM_REG_PC,UC_ARM_REG_R0

test_dir=Path(__file__).resolve().parent
repo=test_dir.parents[1]
output=repo.parent/'tmp/task7/testbuild'
output.mkdir(parents=True,exist_ok=True)
compiler=shutil.which('arm-none-eabi-g++')
objcopy=shutil.which('arm-none-eabi-objcopy')
if not compiler or not objcopy:
    raise SystemExit('需要 ARM GNU 工具链，并将其 bin 目录加入 PATH')
elf=output/'gimbal.elf'
binary=elf.with_suffix('.bin')
command=[compiler,'-mcpu=cortex-m4','-mthumb','-mfloat-abi=soft','-std=gnu++20',
         '-O1','-fno-exceptions','-fno-rtti','-fno-threadsafe-statics','-fno-builtin',
         '-ffunction-sections','-fdata-sections','-nostdlib','-Wl,-e,run_tests',
         '-Wl,-Ttext=0x10000','-Wl,-Tdata=0x30000','-Wl,--section-start=.bss=0x40000',
         '-Wl,--gc-sections']
for directory in ('tests/gimbal/mocks','RobotComponents/component'):
    command+=['-I',str(repo/directory)]
command+=[str(test_dir/'gimbal_test.cpp'),'-lm','-lc','-lgcc','-o',elf.name]
subprocess.run(command,check=True,cwd=output)
subprocess.run([objcopy,'-O','binary',elf.name,binary.name],check=True,cwd=output)
entry=struct.unpack_from('<I',elf.read_bytes(),24)[0]
emulator=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
emulator.mem_map(0x10000,0x90000)
emulator.mem_write(0x10000,binary.read_bytes())
emulator.mem_map(0x20000000,0x10000)
emulator.reg_write(UC_ARM_REG_SP,0x2000fff0)
emulator.reg_write(UC_ARM_REG_LR,0x90001)
emulator.emu_start(entry|1,0x90000,count=100_000_000)
if emulator.reg_read(UC_ARM_REG_PC)!=0x90000:
    raise SystemExit('测试未正常返回')
result=emulator.reg_read(UC_ARM_REG_R0)
if result:
    raise SystemExit(f'gimbal_test.cpp 第 {result} 行失败')
print('云台两轴回绕、限位、力矩门控及数学闭环：ARM 模拟测试 PASS')
