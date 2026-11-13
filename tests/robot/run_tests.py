"""编译真实状态机与真实驱动回归测试，以 ARM 模拟执行，不连接实车。"""
import argparse
from pathlib import Path
import shutil
import struct
import subprocess
import sys
sys.stdout.reconfigure(encoding='utf-8')
parser=argparse.ArgumentParser()
parser.add_argument('--unicorn-path')
args=parser.parse_args()
if args.unicorn_path:
    sys.path.insert(0,str(Path(args.unicorn_path).resolve()))
from unicorn import Uc,UC_ARCH_ARM,UC_MODE_THUMB,UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_SP,UC_ARM_REG_LR,UC_ARM_REG_PC,UC_ARM_REG_R0
test_dir=Path(__file__).resolve().parent
repo=test_dir.parents[1]
output=repo.parent/'tmp/task9/testbuild'
output.mkdir(parents=True,exist_ok=True)
compiler=shutil.which('arm-none-eabi-g++')
objcopy=shutil.which('arm-none-eabi-objcopy')
if not compiler or not objcopy:
    raise SystemExit('需要 ARM GNU 工具链')

def run(name,sources,includes,defines=()):
    """编译指定真实源文件并运行 ARM 测试，失败返回 C++ 行号。"""
    elf=output/f'{name}.elf'
    binary=elf.with_suffix('.bin')
    cmd=[compiler,'-mcpu=cortex-m4','-mthumb','-mfloat-abi=soft','-std=gnu++20',
         '-O1','-fno-exceptions','-fno-rtti','-fno-threadsafe-statics','-fno-builtin',
         '-ffunction-sections','-fdata-sections','-nostdlib','-Wl,-e,run_tests',
         '-Wl,-Ttext=0x10000','-Wl,-Tdata=0x30000','-Wl,--section-start=.bss=0x40000',
         '-Wl,--gc-sections']
    for item in defines:
        cmd+=['-D'+item]
    for folder in includes:
        cmd+=['-I',str(repo/folder)]
    cmd+=[str(repo/source) for source in sources]
    cmd+=['-lm','-lc','-lgcc','-o',elf.name]
    subprocess.run(cmd,check=True,cwd=output)
    subprocess.run([objcopy,'-O','binary',elf.name,binary.name],check=True,cwd=output)
    entry=struct.unpack_from('<I',elf.read_bytes(),24)[0]
    uc=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
    uc.mem_map(0x10000,0x90000);uc.mem_write(0x10000,binary.read_bytes())
    uc.mem_map(0x20000000,0x20000)
    uc.reg_write(UC_ARM_REG_SP,0x2001fff0);uc.reg_write(UC_ARM_REG_LR,0x90001)
    uc.emu_start(entry|1,0x90000,count=100_000_000)
    if uc.reg_read(UC_ARM_REG_PC)!=0x90000:
        raise SystemExit(f'{name}: 测试未返回')
    return uc.reg_read(UC_ARM_REG_R0)

result=run('supervisor',['tests/robot/supervisor_test.cpp'],
           ['tests/robot','RobotComponents/component','Chassis/inc'])
if result:
    raise SystemExit(f'supervisor_test.cpp 第 {result} 行失败')
print('三态、急停/离线/恢复门控、三模式和反馈有效期：PASS')
for board in ('Chassis','Gimbal'):
    includes=['tests/robot',f'{board}/inc',f'{board}/Core/Inc',
              f'{board}/Drivers/STM32F4xx_HAL_Driver/Inc',f'{board}/Drivers/CMSIS/Include',
              f'{board}/Drivers/CMSIS/Device/ST/STM32F4xx/Include']
    runtime_sources=['tests/robot/runtime_test.cpp','RobotComponents/component/gimbal_chassis_comm.cpp',
                     f'{board}/src/DT7.cpp',f'{board}/src/motor_DaMiao.cpp',f'{board}/src/motor_base.cpp',
                     f'{board}/src/motor_M3508.cpp',f'{board}/src/motor_GM6020.cpp',
                     f'{board}/Task/component/main_task.cpp']
    if board=='Chassis':
        runtime_sources+=['Chassis/src/chassis_iksolver.cpp','Chassis/RobotModules/component/chassis.cpp']
    runtime_includes=['tests/robot/mocks','RobotComponents/component']+includes+[
        f'{board}/Task/component',f'{board}/RobotModules/component']
    runtime_defines=['STM32F407xx','USE_HAL_DRIVER','STM32_HAL_FILENAME="stm32f4xx_hal.h"']
    if board=='Chassis':
        runtime_defines+=['TEST_CHASSIS']
    result=run(f'{board}-runtime',runtime_sources,runtime_includes,runtime_defines)
    if result:
        raise SystemExit(f'{board}: runtime_test.cpp 第 {result} 行失败')
    print(f'{board} 真实 Runtime/PID/电机编码、任务分发及归零发送：PASS')
    result=run(f'{board}-driver',['tests/robot/driver_test.cpp',f'{board}/src/motor_DaMiao.cpp',
                                 f'{board}/src/motor_base.cpp'],includes,
               ['STM32F407xx','USE_HAL_DRIVER','STM32_HAL_FILENAME="stm32f4xx_hal.h"'])
    if result:
        raise SystemExit(f'{board}: driver_test.cpp 第 {result} 行失败')
    else:
        print(f'{board} 真实达妙初始化和纯力矩编码：PASS')
