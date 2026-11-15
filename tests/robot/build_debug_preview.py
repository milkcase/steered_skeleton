"""构建两板零输出预演固件到独立目录，不烧录，不覆盖正常固件。"""
from pathlib import Path
import hashlib
import shutil
import subprocess
import sys

sys.stdout.reconfigure(encoding='utf-8')
folder=Path(__file__).resolve().parent
repo=folder.parents[1]


def digest(path):
    """记录已有正常ELF的摘要，用于确认独立构建没有覆盖它。"""
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None


def build(board):
    """为指定板配置预演宏、构建并验证目标路径；所有输出仍由代码强制为零。"""
    source=repo/board
    output=source/'build/MotorDebugPreview'
    normal=source/'build'/('steered_'+board.lower()+'.elf')
    before=digest(normal)
    output.mkdir(parents=True,exist_ok=True)
    command=['cmake','-S',str(source),'-B',str(output),'-G','Ninja','-DCMAKE_BUILD_TYPE=Debug',
             '-DCMAKE_CXX_FLAGS_DEBUG=-g -DROBOT_MOTOR_DEBUG_PREVIEW_ENABLED=1',
             '-DCMAKE_PROJECT_INCLUDE='+str(folder/'debug_preview_output.cmake')]
    with (output/'build.log').open('w',encoding='utf-8') as log:
        subprocess.run(command,check=True,stdout=log,stderr=subprocess.STDOUT)
        subprocess.run(['cmake','--build',str(output),'--parallel','6'],check=True,stdout=log,stderr=subprocess.STDOUT)
    elf=output/'firmware'/normal.name
    if not elf.exists():
        raise SystemExit(f'{board}: 输出目录检查失败，请查看 {output / "build.log"}')
    if digest(normal)!=before:
        raise SystemExit(f'{board}: 正常ELF在预演构建期间发生变化，请停止其他构建后重试；未允许覆盖检查通过')
    for suffix in ('.bin','.hex'):
        shutil.copy2(output/normal.with_suffix(suffix).name,elf.with_suffix(suffix))
    print(f'{board} 零输出预演构建PASS：{elf}',flush=True)


for board in ('Chassis','Gimbal'):
    build(board)
