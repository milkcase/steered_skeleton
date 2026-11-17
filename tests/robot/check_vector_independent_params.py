"""独立编译两份实际配置，证明不同加速度/转速/PID/限流不改变二维模式授权。"""
from pathlib import Path
import shutil,struct,subprocess,tempfile


def main():
    """模拟云台旧数值与底盘新数值，提取ARM对象中的实际模式编号和加速度数值。"""
    root=Path(__file__).resolve().parents[2]
    output=root.parent/'tmp/task9/testbuild';output.mkdir(parents=True,exist_ok=True)
    flags=['ROBOT_DRIVE_SPEED_ENABLED=1','ROBOT_DRIVE_TUNING_ENABLED=1','ROBOT_VECTOR_DRIVE_ENABLED=1']
    source='''#include "motor_debug_mode.hpp"
struct Signature {unsigned target,profile;float acceleration,kp,current;};
extern "C" const Signature signature __attribute__((used,section(".signature")))={
 robot_debug::mode::kTarget,robot_debug::mode::kProfile,robot_debug::drive::kMaxAcceleration,
 robot_debug::drive::kKp,robot_debug::drive::kCurrentA};
'''
    values=[]
    for extra in (['ROBOT_DRIVE_ACCELERATION_RPM_S=9.5492966f','ROBOT_DRIVE_SPEED_KP=2.0f','ROBOT_DRIVE_CURRENT_LIMIT_A=0.3f'],
                  ['ROBOT_DRIVE_ACCELERATION_RPM_S=480.0f','ROBOT_DRIVE_SPEED_KP=5.0f','ROBOT_DRIVE_CURRENT_LIMIT_A=1.0f']):
        with tempfile.TemporaryDirectory(dir=output) as folder:
            obj=Path(folder)/'config.o';binary=Path(folder)/'signature.bin'
            subprocess.run([shutil.which('arm-none-eabi-g++'),'-std=gnu++20','-c','-x','c++','-mcpu=cortex-m4','-mthumb',
                '-I',str(root/'RobotComponents/component'),*['-D'+x for x in flags+extra],'-','-o',str(obj)],
                input=source,text=True,encoding='utf-8',check=True)
            subprocess.run([shutil.which('arm-none-eabi-objcopy'),'--dump-section',f'.signature={binary}',str(obj)],check=True)
            values.append(struct.unpack('<IIfff',binary.read_bytes()))
    if values[0][:2]!=(20,53506) or values[0][:2]!=values[1][:2] or values[0][2:]==values[1][2:]:
        raise SystemExit(f'独立参数授权检查失败：{values}')
    print('云台旧参数/底盘新参数独立编译：target/profile相同，实际加速度/P/电流不同：PASS')


if __name__=='__main__':main()
