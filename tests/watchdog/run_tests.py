"""运行纯喂狗策略及两板真实任务接线测试，不模拟物理 IWDG 复位。"""
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
folder=Path(__file__).resolve().parent
repo=folder.parents[1]
output=repo.parent/'tmp/task10/testbuild'
output.mkdir(parents=True,exist_ok=True)
compiler=shutil.which('arm-none-eabi-g++');objcopy=shutil.which('arm-none-eabi-objcopy')
if not compiler or not objcopy:
    raise SystemExit('需要 ARM GNU 工具链')

def run(name,sources,includes,defines=()):
    """编译真实待验证模块并在 ARM 执行，失败报告测试源文件行号。"""
    elf=output/(name+'.elf');binary=elf.with_suffix('.bin')
    cmd=[compiler,'-mcpu=cortex-m4','-mthumb','-mfloat-abi=soft','-std=gnu++20','-O1',
         '-fno-exceptions','-fno-rtti','-fno-threadsafe-statics','-fno-builtin',
         '-ffunction-sections','-fdata-sections','-nostdlib','-Wl,-e,run_tests',
         '-Wl,-Ttext=0x10000','-Wl,-Tdata=0x30000','-Wl,--section-start=.bss=0x40000','-Wl,--gc-sections']
    for value in defines:
        cmd+=['-D'+value]
    for include in includes:
        cmd+=['-I',str(repo/include)]
    cmd+=[str(repo/source) for source in sources]+['-lm','-lc','-lgcc','-o',elf.name]
    subprocess.run(cmd,cwd=output,check=True)
    subprocess.run([objcopy,'-O','binary',elf.name,binary.name],cwd=output,check=True)
    entry=struct.unpack_from('<I',elf.read_bytes(),24)[0]
    uc=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS);uc.mem_map(0x10000,0x90000)
    uc.mem_write(0x10000,binary.read_bytes());uc.mem_map(0x20000000,0x10000)
    uc.reg_write(UC_ARM_REG_SP,0x2000fff0);uc.reg_write(UC_ARM_REG_LR,0x90001)
    uc.emu_start(entry|1,0x90000,count=100_000_000)
    if uc.reg_read(UC_ARM_REG_PC)!=0x90000:
        raise SystemExit(f'{name}: 测试没有返回')
    result=uc.reg_read(UC_ARM_REG_R0)
    if result:
        raise SystemExit(f'{name}: 测试源文件第 {result} 行失败')
    print(name+': ARM 模拟测试 PASS')

run('policy',['tests/watchdog/policy_test.cpp'],['tests/watchdog','RobotComponents/component'])
for board in ('Chassis','Gimbal'):
    sources=['tests/watchdog/task_test.cpp',f'{board}/Task/component/main_task.cpp',
             f'{board}/Task/component/comm_task.cpp',f'{board}/Task/component/watchdog_task.cpp']
    includes=['tests/watchdog/mocks','tests/watchdog','RobotComponents/component',f'{board}/Task/component']
    defines=['TEST_CHASSIS'] if board=='Chassis' else []
    run(board+'-watchdog',sources,includes,defines)
    run(board+'-stop-feeding',sources,includes,defines+['ROBOT_WATCHDOG_REFRESH_ENABLED=0'])
