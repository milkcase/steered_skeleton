"""独立构建左前转向受限动力固件；不烧录，不覆盖普通或已验收的零输出预演ELF。"""
from pathlib import Path
import hashlib
import shutil
import subprocess
import sys

sys.stdout.reconfigure(encoding='utf-8')
folder=Path(__file__).resolve().parent
repo=folder.parents[1]


def digest(path):
    """保存既有ELF摘要，用于确认动力构建未覆盖正常或预演产物。"""
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None


def build(board):
    """显式启用本轮动力宏，输出至独立目录并保留构建日志，不调用下载或调试器。"""
    source=repo/board
    output=source/'build/MotorDebugPower'
    name='steered_'+board.lower()
    protected=[source/'build'/(name+'.elf'),source/'build/MotorDebugPreview/firmware'/(name+'.elf')]
    hashes=[digest(path) for path in protected]
    output.mkdir(parents=True,exist_ok=True)
    cmd=['cmake','-S',str(source),'-B',str(output),'-G','Ninja','-DCMAKE_BUILD_TYPE=Debug',
         '-DCMAKE_CXX_FLAGS_DEBUG=-g -DROBOT_MOTOR_DEBUG_POWER_ENABLED=1',
         '-DCMAKE_PROJECT_INCLUDE='+str(folder/'debug_preview_output.cmake')]
    with (output/'build.log').open('w',encoding='utf-8') as log:
        subprocess.run(cmd,check=True,stdout=log,stderr=subprocess.STDOUT)
        subprocess.run(['cmake','--build',str(output),'--parallel','6'],check=True,stdout=log,stderr=subprocess.STDOUT)
    elf=output/'firmware'/(name+'.elf')
    if not elf.exists() or [digest(path) for path in protected]!=hashes:
        raise SystemExit(f'{board}: 产物隔离检查失败，停止其他并发构建后核对 {output / "build.log"}')
    for suffix in ('.bin','.hex'):
        shutil.copy2(output/(name+suffix),elf.with_suffix(suffix))
    print(f'{board} 受限动力构建PASS：{elf}',flush=True)


if __name__=='__main__':
    for board in ('Chassis','Gimbal'):
        build(board)
