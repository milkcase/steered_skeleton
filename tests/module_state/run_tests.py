"""以两板生产选项检查共享类型，再用纯 ARM 模拟验证默认值、运算和非法输入。"""
import argparse
from pathlib import Path
import shutil
import struct
import subprocess
import sys
sys.stdout.reconfigure(encoding="utf-8")

parser=argparse.ArgumentParser()
parser.add_argument('--unicorn-path',help='可选：临时安装的 unicorn 包目录')
args=parser.parse_args()
if args.unicorn_path:
    sys.path.insert(0,str(Path(args.unicorn_path).resolve()))
from unicorn import Uc,UC_ARCH_ARM,UC_MODE_THUMB,UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_SP,UC_ARM_REG_LR,UC_ARM_REG_PC,UC_ARM_REG_R0

test_dir=Path(__file__).resolve().parent
repo=test_dir.parents[1]
output=repo.parent/'tmp/task5/testbuild'
output.mkdir(parents=True,exist_ok=True)
compiler=shutil.which('arm-none-eabi-g++')
objcopy=shutil.which('arm-none-eabi-objcopy')
if not compiler or not objcopy:
    raise SystemExit('需要 ARM GNU 工具链，且其 bin 目录须在 PATH 中')
source=test_dir/'module_state_test.cpp'
for board in ('Chassis','Gimbal'):
    includes=[repo/'RobotComponents/component',repo/board/'inc',
              repo/board/'Instance/component',repo/board/'Core/Inc',
              repo/board/'Drivers/STM32F4xx_HAL_Driver/Inc',
              repo/board/'Drivers/CMSIS/Include',
              repo/board/'Drivers/CMSIS/Device/ST/STM32F4xx/Include',
              repo/board/'third_party/dsp/CMSIS-DSP/Include']
    # 此步骤用生产硬件浮点选项，确认与各板已有接口同时包含不会发生声明冲突。
    command=[compiler,'-mcpu=cortex-m4','-mthumb','-mfpu=fpv4-sp-d16','-mfloat-abi=hard',
             '-std=gnu++20','-fno-exceptions','-fno-rtti','-Wno-unknown-pragmas',
             '-DTEST_PROJECT_HEADERS','-DSTM32F407xx','-DUSE_HAL_DRIVER',
             '-DSTM32_HAL_FILENAME="stm32f4xx_hal.h"','-c',str(source),'-o',f'{board}.o']
    for directory in includes:
        command+=['-I',str(directory)]
    subprocess.run(command,check=True,cwd=output)

    # 模拟程序只包含共享类型与标准库，不包含任何板级头文件或 HAL 替身。
    elf=output/f'{board}.elf'
    binary=elf.with_suffix('.bin')
    command=[compiler,'-mcpu=cortex-m4','-mthumb','-mfloat-abi=soft','-std=gnu++20',
             '-O1','-fno-exceptions','-fno-rtti','-fno-builtin','-ffunction-sections',
             '-fdata-sections','-nostdlib','-Wl,-e,run_tests','-Wl,-Ttext=0x10000',
             '-Wl,-Tdata=0x30000','-Wl,--section-start=.bss=0x40000','-Wl,--gc-sections',
             '-I',str(repo/'RobotComponents/component'),str(source),'-lgcc','-o',elf.name]
    subprocess.run(command,check=True,cwd=output)
    subprocess.run([objcopy,'-O','binary',elf.name,binary.name],check=True,cwd=output)
    entry=struct.unpack_from('<I',elf.read_bytes(),24)[0]
    emulator=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
    emulator.mem_map(0x10000,0x90000)
    emulator.mem_write(0x10000,binary.read_bytes())
    emulator.mem_map(0x20000000,0x10000)
    emulator.reg_write(UC_ARM_REG_SP,0x2000fff0)
    emulator.reg_write(UC_ARM_REG_LR,0x90001)
    emulator.emu_start(entry|1,0x90000,count=2_000_000)
    if emulator.reg_read(UC_ARM_REG_PC)!=0x90000:
        raise SystemExit(f'{board}: 测试未正常返回')
    result=emulator.reg_read(UC_ARM_REG_R0)
    if result:
        raise SystemExit(f'{board}: module_state_test.cpp 第 {result} 行失败')
    print(f'{board}: 生产头文件兼容性与 ARM 模拟测试 PASS')
