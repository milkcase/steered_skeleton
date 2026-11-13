"""沿用两板 CMake 生产参数验证完整链接，不修改 CMake，绕过工具链共享对象输出路径的中文问题。"""
import ctypes
import json
from pathlib import Path
import shutil
import subprocess
import sys
sys.stdout.reconfigure(encoding='utf-8')
repo=Path(__file__).resolve().parents[2]
workspace=repo.parent
output=workspace/'tmp/task9/production'
output.mkdir(parents=True,exist_ok=True)

def arguments(command):
    """使用 Windows 原生规则拆解 CMake 命令，不通过 shell 执行拼接字符串。"""
    count=ctypes.c_int()
    api=ctypes.windll.shell32.CommandLineToArgvW
    api.restype=ctypes.POINTER(ctypes.c_wchar_p)
    argv=api(command,ctypes.byref(count))
    result=[argv[i] for i in range(count.value)]
    ctypes.windll.kernel32.LocalFree(argv)
    return result

for board in ('Chassis','Gimbal'):
    name='steered_'+board.lower()
    build=workspace/'tmp/task1'/('build-'+board.lower())
    subprocess.run(['cmake','-S',str(repo/board),'-B',str(build),'-G','Ninja',
                    '-DCMAKE_BUILD_TYPE=Debug','-DCMAKE_EXPORT_COMPILE_COMMANDS=ON'],check=True,
                   stdout=subprocess.DEVNULL)
    result=subprocess.run(['cmake','--build',str(build),'--','-k','0'],capture_output=True,
                          encoding='utf-8',errors='replace')
    log=result.stdout+result.stderr
    (output/f'{board}-cmake.log').write_text(log,encoding='utf-8')
    if result.returncode:
        failures=[line for line in log.splitlines() if line.startswith('FAILED:')]
        if len(failures)!=1 or 'gimbal_chassis_comm.cpp.obj' not in failures[0] or "can't create" not in log:
            raise SystemExit(f'{board}: CMake 出现其他失败，见 {output / (board+"-cmake.log")}')

    # 共享 cpp 使用原始所有生产参数，仅把输出对象名改成当前目录下的 ASCII 名称。
    entries=json.loads((build/'compile_commands.json').read_text(encoding='utf-8'))
    entry=next(item for item in entries if item['file'].endswith('gimbal_chassis_comm.cpp'))
    command=arguments(entry['command'])
    command[command.index('-o')+1]='verified-board-codec.o'
    subprocess.run(command,cwd=build,check=True)
    commands=subprocess.check_output(['ninja','-C',str(build),'-t','commands',name],encoding='utf-8')
    # CMake 的链接命令位于 cd 与后处理命令之间；只取编译器参数，不执行其他 shell 片段。
    link=arguments(commands.splitlines()[-1].split(' && ')[1])
    for i,arg in enumerate(link):
        if arg.replace('\\','/').endswith('RobotComponents/component/gimbal_chassis_comm.cpp.obj'):
            link[i]='verified-board-codec.o'
    elf=name+'-verified.elf'
    link[link.index('-o')+1]=elf
    subprocess.run(link,cwd=build,check=True)
    subprocess.run(['arm-none-eabi-objcopy','-Obinary',elf,name+'-verified.bin'],cwd=build,check=True)
    symbols=subprocess.check_output(['arm-none-eabi-nm',elf],cwd=build,encoding='utf-8')
    for symbol in ('HAL_TIM_PeriodElapsedCallback','MainTaskInit','MainTask','CommTask','RobotSensorsPoll'):
        if not any(line.endswith(' T '+symbol) for line in symbols.splitlines()):
            raise SystemExit(f'{board}: 缺少 C 链接入口 {symbol}')
    for suffix in ('.elf','.bin'):
        shutil.copyfile(build/(name+'-verified'+suffix),output/(name+suffix))
    print(f'{board}: 生产参数完整链接及 C 入口符号检查 PASS')
