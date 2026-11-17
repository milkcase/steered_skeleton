"""验证持续驱动可用自由小数、底盘单独改值不改变授权，非法数值仍被拒绝。"""
from pathlib import Path
import shutil
import subprocess


def main():
    """分别编译默认云台和不同底盘参数，固定模式标识必须相同，数值必须直接采用。"""
    root=Path(__file__).resolve().parents[2]
    common=['ROBOT_DRIVE_SPEED_ENABLED=1','ROBOT_DRIVE_TUNING_ENABLED=1']
    cases=[([],True),(['ROBOT_DRIVE_TARGET_SPEED_RPM=37.137f','ROBOT_DRIVE_ACCELERATION_RPM_S=43.127f',
        'ROBOT_DRIVE_TRIP_SPEED_RPM=61.137f'],True),(['ROBOT_DRIVE_TARGET_SPEED_RPM=30.0f',
        'ROBOT_DRIVE_TRIP_SPEED_RPM=30.0f'],False),(['ROBOT_STEER_POSITION_KP=137.123f','ROBOT_STEER_CURRENT_LIMIT_A=0.617f',
        'ROBOT_STEER_SPEED_KP=0.137f'],True),(['ROBOT_DRIVE_SPEED_KP=137.123f','ROBOT_DRIVE_SPEED_KI=0.0137f',
        'ROBOT_DRIVE_SPEED_KD=0.00027f','ROBOT_DRIVE_CURRENT_LIMIT_A=1.237f',
        'ROBOT_DRIVE_INTEGRAL_LIMIT_A=0.177f','ROBOT_DRIVE_TARGET_SPEED_RAD_S=2.137f',
        'ROBOT_DRIVE_ACCELERATION_RAD_S2=3.137f','ROBOT_DRIVE_TRIP_SPEED_RAD_S=4.317f'],True),
        (['ROBOT_DRIVE_SPEED_KP=0.0f'],True),(['ROBOT_DRIVE_SPEED_KI=-0.1f'],False),
        (['ROBOT_DRIVE_SPEED_KP=__builtin_inff()'],False),(['ROBOT_DRIVE_CURRENT_LIMIT_A=20.01f'],False),
        (['ROBOT_DRIVE_CURRENT_LIMIT_A=0.0f'],False),(['ROBOT_DRIVE_TRIP_SPEED_RAD_S=0.5f'],False)]
    source='''#include "motor_debug_mode.hpp"
static_assert(robot_debug::mode::kTarget==19 && robot_debug::mode::kProfile==0xD101);
static_assert(robot_debug::mode::kResumable);
static_assert(robot_debug::drive::kKp==ROBOT_DRIVE_SPEED_KP);
static_assert(robot_debug::drive::kKi==ROBOT_DRIVE_SPEED_KI);
static_assert(robot_debug::drive::kKd==ROBOT_DRIVE_SPEED_KD);
static_assert(robot_debug::drive::kCurrentA==ROBOT_DRIVE_CURRENT_LIMIT_A);
static_assert(robot_debug::drive::kIntegralA==ROBOT_DRIVE_INTEGRAL_LIMIT_A);
'''
    for changes,valid in cases:
        p=subprocess.run([shutil.which('arm-none-eabi-g++'),'-std=gnu++20','-fsyntax-only','-x','c++',
            '-I',str(root/'RobotComponents/component'),*['-D'+x for x in common+changes],'-'],
            input=source,text=True,encoding='utf-8',errors='replace',capture_output=True)
        if (p.returncode==0)!=valid:raise SystemExit(f'持续驱动参数失败：{changes}\n{p.stderr}')
    print('持续驱动任意小数/固定授权/数值及协议边界检查：PASS')


if __name__=='__main__':main()
