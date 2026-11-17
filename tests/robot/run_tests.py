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
subprocess.run([sys.executable,str(Path(__file__).with_name('check_debug_config.py'))],check=True)
subprocess.run([sys.executable,str(Path(__file__).with_name('check_four_config.py'))],check=True)
subprocess.run([sys.executable,str(Path(__file__).with_name('check_drive_tuning.py'))],check=True)
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
    # 预算覆盖整段多秒回归及新增锁存/通道复制，不是一次控制周期；实际3ms/500ms/IWDG门槛未改。
    # 四套软浮点闭环验证整段约两秒遥控流程，指令预算独立增加；实车3ms/20ms保护不变。
    uc.emu_start(entry|1,0x90000,count=600_000_000 if 'four-steer' in name else 200_000_000)
    if uc.reg_read(UC_ARM_REG_PC)!=0x90000:
        raise SystemExit(f'{name}: 测试未返回')
    return uc.reg_read(UC_ARM_REG_R0)

result=run('supervisor',['tests/robot/supervisor_test.cpp'],
           ['tests/robot','RobotComponents/component','Chassis/inc'])
if result:
    raise SystemExit(f'supervisor_test.cpp 第 {result} 行失败')
print('三态、急停/离线/恢复门控、三模式和反馈有效期：PASS')
result=run('debug-preview',['tests/robot/debug_preview_test.cpp','RobotComponents/component/gimbal_chassis_comm.cpp'],
           ['tests/robot','RobotComponents/component','Chassis/inc'])
if result:
    raise SystemExit(f'debug_preview_test.cpp 第 {result} 行失败')
print('零输出预演、回中握手、500ms锁止、异常撤销及协议隔离：PASS')
result=run('debug-power',['tests/robot/debug_power_test.cpp','RobotComponents/component/gimbal_chassis_comm.cpp'],
           ['tests/robot','RobotComponents/component','Chassis/inc'])
if result:
    raise SystemExit(f'debug_power_test.cpp 第 {result} 行失败')
print('受限动力双板会话、Working红线、500ms截止和运动边界：PASS')
direction_defines=['ROBOT_MOTOR_DEBUG_POWER_ENABLED=1','ROBOT_STEER_DIRECTION_ENABLED=1']
result=run('debug-direction',['tests/robot/debug_direction_test.cpp','RobotComponents/component/gimbal_chassis_comm.cpp'],
           ['tests/robot','RobotComponents/component','Chassis/inc'],direction_defines)
if result:
    raise SystemExit(f'debug_direction_test.cpp 第 {result} 行失败')
print('二维方向双环、实际速度制动、无500ms截止、回中/失效归零与参数隔离：PASS')
result=run('debug-direction-high-position-p',['tests/robot/debug_direction_test.cpp','RobotComponents/component/gimbal_chassis_comm.cpp'],
           ['tests/robot','RobotComponents/component','Chassis/inc'],direction_defines+['ROBOT_STEER_POSITION_KP=100.0f'])
if result:
    raise SystemExit(f'位置P=100：debug_direction_test.cpp 第 {result} 行失败')
print('位置P=100：运行时放行、速度/电流限幅、回中恢复及故障锁止：PASS')
# 回归使用固定基线，不把用户正在实机整定的P/限流覆盖回源文件，也不受旧档位限制干扰。
drive_defines=['ROBOT_MOTOR_DEBUG_POWER_ENABLED=1','ROBOT_DRIVE_SPEED_ENABLED=1',
    'ROBOT_DRIVE_SPEED_KP=2.0f','ROBOT_DRIVE_SPEED_KI=0.0f','ROBOT_DRIVE_SPEED_KD=0.0f','ROBOT_DRIVE_CURRENT_LIMIT_A=0.3f']
drive_tuning_defines=drive_defines+['ROBOT_DRIVE_TUNING_ENABLED=1']
result=run('vector-math',['tests/robot/vector_math_test.cpp','RobotComponents/component/gimbal_chassis_comm.cpp','Chassis/src/chassis_iksolver.cpp'],
           ['tests/robot','RobotComponents/component','Chassis/inc'],drive_tuning_defines+['ROBOT_VECTOR_DRIVE_ENABLED=1'])
if result:raise SystemExit(f'二维投影/协议第 {result} 行失败')
print('二维连续投影、坐标旋转、等效反转及前推/回中协议区分：PASS')
result=run('drive-tuning',['tests/robot/debug_drive_tuning_test.cpp','RobotComponents/component/gimbal_chassis_comm.cpp'],
           ['tests/robot','RobotComponents/component','Chassis/inc'],drive_tuning_defines)
if result:raise SystemExit(f'持续驱动调参第 {result} 行失败')
print('M3508持续调参、回中恢复、故障锁止及旧版隔离：PASS')
result=run('debug-drive',['tests/robot/debug_drive_test.cpp','RobotComponents/component/gimbal_chassis_comm.cpp'],
           ['tests/robot','RobotComponents/component','Chassis/inc'],drive_defines)
if result:
    raise SystemExit(f'debug_drive_test.cpp 第 {result} 行失败')
print('M3508单速度环、限加速度/电流、500ms/位移保护、本地整定编码及安全包络：PASS')
# 只改变本轮两个公开旋钮，验证更高一档配置同样限流、配置编号匹配且保留所有停止行为。
alternate_tuning=['ROBOT_MOTOR_DEBUG_KP=6.0f','ROBOT_MOTOR_DEBUG_CURRENT_LIMIT_A=0.100f']
result=run('debug-power-adjusted',['tests/robot/debug_power_test.cpp','RobotComponents/component/gimbal_chassis_comm.cpp'],
           ['tests/robot','RobotComponents/component','Chassis/inc'],alternate_tuning)
if result:
    raise SystemExit(f'可调参数debug_power_test.cpp 第 {result} 行失败')
print('P=6/限流0.100A、配置编号唯一及不同参数拒绝：PASS')
result=run('debug-power-low-gain',['tests/robot/debug_power_test.cpp','RobotComponents/component/gimbal_chassis_comm.cpp'],
           ['tests/robot','RobotComponents/component','Chassis/inc'],
           ['ROBOT_MOTOR_DEBUG_KP=0.5f','ROBOT_MOTOR_DEBUG_CURRENT_LIMIT_A=0.200f'])
if result:
    raise SystemExit(f'低P/最高电流档debug_power_test.cpp 第 {result} 行失败')
print('P=0.5/限流0.200A、实际指令低于上限及原截止保护：PASS')
# 非法值必须在编译阶段直接失败，不能默默截断或移除硬边界后继续构建。
for define in ('ROBOT_MOTOR_DEBUG_KP=0.6f','ROBOT_MOTOR_DEBUG_CURRENT_LIMIT_A=0.300f'):
    rejected=subprocess.run([compiler,'-std=gnu++20','-fsyntax-only','-x','c++','-I',str(repo/'RobotComponents/component'),
                             '-D'+define,'-'],input='#include "motor_debug_tuning.hpp"\n',text=True,
                            encoding='utf-8',errors='replace',capture_output=True)
    if rejected.returncode==0 or 'static assertion failed' not in rejected.stderr:
        raise SystemExit('非法调试参数未按预期由编译检查拒绝：'+define)
print('非档位P/超出硬上限电流的编译拒绝：PASS')
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
    result=run(f'{board}-preview-runtime',runtime_sources,runtime_includes,
               runtime_defines+['ROBOT_MOTOR_DEBUG_PREVIEW_ENABLED=1'])
    if result:
        raise SystemExit(f'{board}: 零输出预演 runtime_test.cpp 第 {result} 行失败')
    print(f'{board} 零输出预演真实链路、全部电机帧和使能隔离：PASS')
    result=run(f'{board}-power-runtime',runtime_sources,runtime_includes,
               runtime_defines+['ROBOT_MOTOR_DEBUG_POWER_ENABLED=1'])
    if result:
        raise SystemExit(f'{board}: 受限动力 power_runtime_case.hpp/runtime_test.cpp 第 {result} 行失败')
    print(f'{board} 受限动力真实发送：仅ID1、Working、限幅及急停最终重查：PASS')
    result=run(f'{board}-direction-runtime',runtime_sources,runtime_includes,runtime_defines+direction_defines)
    if result:
        raise SystemExit(f'{board}: direction_runtime_case.hpp/runtime_test.cpp 第 {result} 行失败')
    print(f'{board} 二维方向双环真实Runtime/DT7/CAN帧、长于500ms及Ozone镜像：PASS')
    result=run(f'{board}-four-steer-runtime',runtime_sources,runtime_includes,runtime_defines+direction_defines+['ROBOT_FOUR_STEER_ENABLED=1'])
    if result:raise SystemExit(f'{board}: 四转向direction_runtime_case.hpp 第 {result} 行失败')
    print(f'{board} 四转向独立反馈/PID、四槽合帧、单轮超速全组归零：PASS')
    result=run(f'{board}-drive-runtime',runtime_sources,runtime_includes,runtime_defines+drive_defines)
    if result:
        raise SystemExit(f'{board}: drive_runtime_case.hpp/runtime_test.cpp 第 {result} 行失败')
    print(f'{board} M3508真实驱动、减速比/电流单位、唯一槽位及发送超时：PASS')
    result=run(f'{board}-drive-tuning-runtime',runtime_sources,runtime_includes,runtime_defines+drive_tuning_defines)
    if result:raise SystemExit(f'{board}: 持续驱动真实Runtime第 {result} 行失败')
    print(f'{board} 持续M3508真实帧/零输出隔离/回中恢复/发送超时：PASS')
    result=run(f'{board}-vector-runtime',runtime_sources,runtime_includes,runtime_defines+drive_tuning_defines+['ROBOT_VECTOR_DRIVE_ENABLED=1'])
    if result:raise SystemExit(f'{board}: 二维组合调试真实Runtime第 {result} 行失败')
    print(f'{board} 二维四转向/仅左前驱动/连续投影/回中和发送保护：PASS')
    if board=='Chassis':
        result=run(f'{board}-adjusted-power-runtime',runtime_sources,runtime_includes,
                   runtime_defines+['ROBOT_MOTOR_DEBUG_POWER_ENABLED=1']+alternate_tuning)
        if result:
            raise SystemExit(f'{board}: 可调电流真实发送第 {result} 行失败')
        print('Chassis 可调档位真实编码/限流、独立反馈电流与停止保护：PASS')
    result=run(f'{board}-driver',['tests/robot/driver_test.cpp',f'{board}/src/motor_DaMiao.cpp',
                                 f'{board}/src/motor_base.cpp'],includes,
               ['STM32F407xx','USE_HAL_DRIVER','STM32_HAL_FILENAME="stm32f4xx_hal.h"'])
    if result:
        raise SystemExit(f'{board}: driver_test.cpp 第 {result} 行失败')
    else:
        print(f'{board} 真实达妙初始化和纯力矩编码：PASS')
