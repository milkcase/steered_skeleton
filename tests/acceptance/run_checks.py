"""集中执行十项相关的软件核验；输出可复查日志，明确不代替实车验收。"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys
sys.stdout.reconfigure(encoding='utf-8')
parser=argparse.ArgumentParser()
parser.add_argument('--unicorn-path')
parser.add_argument('--only-motors',action='store_true',help='只运行新增电机工厂/路由核验')
args=parser.parse_args()
folder=Path(__file__).resolve().parent;repo=folder.parents[1];workspace=repo.parent
output=workspace/'tmp/task10/acceptance';output.mkdir(parents=True,exist_ok=True)
temp=workspace/'tmp/task10/system-temp';temp.mkdir(parents=True,exist_ok=True)
env=os.environ.copy();env.update(TEMP=str(temp),TMP=str(temp),PYTHONIOENCODING='utf-8',PYTHONUTF8='1')
unicorn_path=str(Path(args.unicorn_path).resolve()) if args.unicorn_path else None
if unicorn_path:
    sys.path.insert(0,unicorn_path)
from unicorn import Uc,UC_ARCH_ARM,UC_MODE_THUMB,UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_SP,UC_ARM_REG_LR,UC_ARM_REG_PC,UC_ARM_REG_R0
compiler=shutil.which('arm-none-eabi-g++');objcopy=shutil.which('arm-none-eabi-objcopy')
if not compiler or not objcopy:
    raise SystemExit('需要 ARM GNU 工具链')
results=[]

def arm_test(name,sources,includes,defines):
    """运行新增的真实工厂/接收路由测试，电机帧只在 ARM 模拟器中处理。"""
    elf=output/(name+'.elf');binary=elf.with_suffix('.bin')
    cmd=[compiler,'-mcpu=cortex-m4','-mthumb','-mfloat-abi=soft','-std=gnu++20','-O1',
         '-fno-exceptions','-fno-rtti','-fno-threadsafe-statics','-fno-builtin',
         '-ffunction-sections','-fdata-sections','-nostdlib','-Wl,-e,run_tests','-Wl,-Ttext=0x10000',
         '-Wl,-Tdata=0x30000','-Wl,--section-start=.bss=0x40000','-Wl,--gc-sections']
    for define in defines:
        cmd+=['-D'+define]
    for include in includes:
        cmd+=['-I',str(repo/include)]
    cmd+=[str(repo/source) for source in sources]+['-lm','-lc','-lgcc','-o',elf.name]
    subprocess.run(cmd,cwd=output,env=env,check=True)
    subprocess.run([objcopy,'-O','binary',elf.name,binary.name],cwd=output,env=env,check=True)
    entry=struct.unpack_from('<I',elf.read_bytes(),24)[0]
    uc=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS);uc.mem_map(0x10000,0x90000)
    uc.mem_write(0x10000,binary.read_bytes());uc.mem_map(0x20000000,0x10000)
    uc.reg_write(UC_ARM_REG_SP,0x2000fff0);uc.reg_write(UC_ARM_REG_LR,0x90001)
    uc.emu_start(entry|1,0x90000,count=100_000_000)
    if uc.reg_read(UC_ARM_REG_PC)!=0x90000:
        raise SystemExit(f'{name}: 测试未返回')
    result=uc.reg_read(UC_ARM_REG_R0)
    if result:
        raise SystemExit(f'{name}: 测试源第 {result} 行失败')
    results.append({'check':name,'result':'PASS','hardware':False})
    print(name+': PASS',flush=True)

for board in ('Chassis','Gimbal'):
    define=['TEST_CHASSIS'] if board=='Chassis' else []
    arm_test(board+'-motor-routing',
             ['tests/acceptance/can_routing_test.cpp',f'{board}/src/HW_can.cpp',
              f'{board}/Instance/component/ins_chassis_gimbal_comm.cpp','RobotComponents/component/gimbal_chassis_comm.cpp'],
             ['tests/acceptance','tests/board_comm/mocks','RobotComponents/component',f'{board}/inc',f'{board}/Instance/component'],define)
    arm_test(board+'-motor-factory',
             ['tests/acceptance/motor_factory_test.cpp',f'{board}/Instance/component/ins_motor.cpp',
              f'{board}/src/motor_DaMiao.cpp',f'{board}/src/motor_base.cpp',f'{board}/src/motor_M3508.cpp',f'{board}/src/motor_GM6020.cpp'],
             ['tests/acceptance','RobotComponents/component',f'{board}/inc',f'{board}/Instance/component',f'{board}/Core/Inc',
              f'{board}/Drivers/STM32F4xx_HAL_Driver/Inc',f'{board}/Drivers/CMSIS/Include',
              f'{board}/Drivers/CMSIS/Device/ST/STM32F4xx/Include'],
             define+['STM32F407xx','USE_HAL_DRIVER','STM32_HAL_FILENAME="stm32f4xx_hal.h"'])
    arm_test(board+'-provided-driver-protocol',
             ['tests/acceptance/driver_protocol_test.cpp',f'{board}/src/motor_DaMiao.cpp',f'{board}/src/motor_base.cpp'],
             ['tests/acceptance',f'{board}/inc',f'{board}/Core/Inc',f'{board}/Drivers/STM32F4xx_HAL_Driver/Inc',
              f'{board}/Drivers/CMSIS/Include',f'{board}/Drivers/CMSIS/Device/ST/STM32F4xx/Include'],
             ['STM32F407xx','USE_HAL_DRIVER','STM32_HAL_FILENAME="stm32f4xx_hal.h"'])
if args.only_motors:
    raise SystemExit(0)

def suite(name):
    """独立运行已有测试入口，完整保存输出；相互独立的目录可并发执行。"""
    command=[sys.executable,str(repo/'tests'/name/'run_tests.py')]
    if unicorn_path:
        command+=['--unicorn-path',unicorn_path]
    result=subprocess.run(command,cwd=workspace,env=env,capture_output=True,encoding='utf-8',errors='replace',timeout=240)
    (output/(name+'.log')).write_text(result.stdout+result.stderr,encoding='utf-8')
    return {'check':name,'result':'PASS' if result.returncode==0 else 'FAIL','returncode':result.returncode,'hardware':False}

with ThreadPoolExecutor(max_workers=3) as pool:
    pending=[pool.submit(suite,name) for name in ('dbus_receiver','pid','imu','module_state','chassis_ik','gimbal','board_comm','robot','watchdog')]
    for future in as_completed(pending):
        item=future.result();results.append(item);print(item['check']+': '+item['result'],flush=True)
(output/'results.json').write_text(json.dumps({'hardware_validated':False,'checks':results},ensure_ascii=False,indent=2),encoding='utf-8')
if any(item['result']!='PASS' for item in results):
    raise SystemExit('存在软件测试失败，详见 tmp/task10/acceptance 对应日志')

# 正常固件必须保留原 IWDG 参数；生产应用只能有一个实际 HAL 刷新调用位置。
sites=[]
for root in ('RobotComponents','Chassis/Task','Gimbal/Task','Chassis/Core','Gimbal/Core','Chassis/Instance','Gimbal/Instance','Chassis/RobotModules','Gimbal/RobotModules'):
    for path in (repo/root).rglob('*'):
        if path.suffix not in ('.cpp','.hpp','.c','.h'):
            continue
        code=re.sub(r'/\*.*?\*/|//[^\n]*','',path.read_text(encoding='utf-8-sig'),flags=re.S)
        if re.search(r'\bHAL_IWDG_Refresh\s*\(',code):
            sites.append(str(path.relative_to(repo)))
if sites!=['RobotComponents\\component\\watchdog_task_impl.hpp'] and sites!=['RobotComponents/component/watchdog_task_impl.hpp']:
    raise SystemExit(f'发现额外或缺失的应用喂狗点：{sites}')
for board in ('Chassis','Gimbal'):
    text=(repo/board/'Core/Src/iwdg.c').read_text(encoding='utf-8')
    assert 'hiwdg.Init.Prescaler = IWDG_PRESCALER_128;' in text and 'hiwdg.Init.Reload = 50;' in text
results.append({'check':'refresh-site-and-original-iwdg-config','result':'PASS','hardware':False})
print('唯一应用喂狗点及原 IWDG 配置：PASS',flush=True)

# 复用第九项的生产构建方式，不改变它，也不修改 CMake 或提供的链接脚本。
build=subprocess.run([sys.executable,str(repo/'tests/robot/verify_build.py')],cwd=workspace,env=env,
                     capture_output=True,encoding='utf-8',errors='replace',timeout=240)
(output/'production-build.log').write_text(build.stdout+build.stderr,encoding='utf-8')
if build.returncode:
    raise SystemExit('生产链接失败，详见 tmp/task10/acceptance/production-build.log')
production=workspace/'tmp/task10/production';production.mkdir(parents=True,exist_ok=True)
for board in ('Chassis','Gimbal'):
    name='steered_'+board.lower();elf=workspace/'tmp/task9/production'/(name+'.elf')
    symbols=subprocess.check_output(['arm-none-eabi-nm',str(elf)],encoding='utf-8')
    for symbol in ('RobotWatchdogInit','RobotWatchdogForeground','RobotWatchdogControlBegin','RobotWatchdogControlEnd',
                   'RobotWatchdogCommBegin','RobotWatchdogCommEnd','ReadRobotWatchdogDiagnostics','HAL_IWDG_Refresh'):
        assert any(line.endswith(' T '+symbol) for line in symbols.splitlines()),f'{board}: 缺少强定义 {symbol}'
    assert any(line.endswith(' robot_watchdog_diagnostics') for line in symbols.splitlines()),'缺少看门狗诊断镜像'
    for suffix in ('.elf','.bin'):
        shutil.copyfile(elf.with_suffix(suffix),production/(name+suffix))
    results.append({'check':board+'-full-production-link-watchdog-symbols','result':'PASS','hardware':False})
    print(board+' 完整链接及真实看门狗入口：PASS',flush=True)
(output/'results.json').write_text(json.dumps({'hardware_validated':False,'checks':results},ensure_ascii=False,indent=2),encoding='utf-8')
print('软件核验完成；尚未进行实车动作、拔线或真实 IWDG 复位验收。',flush=True)
