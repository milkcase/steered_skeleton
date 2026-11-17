"""检查四转向自由小数参数及必须保留的数值/物理约束，不访问实车。"""
from pathlib import Path
import shutil
import subprocess


def main():
    """编译不同底盘配置，保证非档位数值可用，非法数值/超过驱动电流被拒绝。"""
    root=Path(__file__).resolve().parents[2]
    compiler=shutil.which('arm-none-eabi-g++')
    common=['ROBOT_FOUR_STEER_ENABLED=1','ROBOT_STEER_DIRECTION_ENABLED=1']
    cases=[([],True),(['ROBOT_DRIVE_CURRENT_LIMIT_A=1.237f','ROBOT_DRIVE_SPEED_KP=137.123f'],True),
        (['ROBOT_STEER_POSITION_KP=137.123f','ROBOT_STEER_SPEED_KP=0.137f',
        'ROBOT_STEER_SPEED_KI=0.0137f','ROBOT_STEER_SPEED_KD=0.00027f','ROBOT_STEER_CURRENT_LIMIT_A=0.617f',
        'ROBOT_STEER_POSITION_KI=0.00137f','ROBOT_STEER_POSITION_KD=0.00013f',
        'ROBOT_STEER_POSITION_INTEGRAL_LIMIT_RAD_S=0.371f','ROBOT_STEER_SPEED_INTEGRAL_LIMIT_A=0.273f',
        'ROBOT_STEER_TARGET_SPEED_RPM=99.123f','ROBOT_STEER_TARGET_RATE_RPM=73.137f','ROBOT_STEER_TRIP_SPEED_RPM=157.777f'],True),
        (['ROBOT_STEER_CURRENT_LIMIT_A=3.01f'],False),(['ROBOT_STEER_CURRENT_LIMIT_A=0.0f'],False),
        (['ROBOT_STEER_POSITION_KP=-0.1f'],False),(['ROBOT_STEER_SPEED_KP=__builtin_inff()'],False),
        (['ROBOT_STEER_POSITION_KI=-0.1f'],False),(['ROBOT_STEER_SPEED_INTEGRAL_LIMIT_A=__builtin_inff()'],False),
        (['ROBOT_STEER_TARGET_SPEED_RPM=0.0f'],False),(['ROBOT_STEER_TRIP_SPEED_RPM=120.0f'],False)]
    source='''#include "motor_debug_mode.hpp"
static_assert(robot_debug::mode::kTarget==60 && robot_debug::mode::kProfile==0xF104);
static_assert(robot_debug::direction::kPositionKp==ROBOT_STEER_POSITION_KP);
static_assert(robot_debug::direction::kSpeedKi==ROBOT_STEER_SPEED_KI);
static_assert(robot_debug::direction::kSpeedKd==ROBOT_STEER_SPEED_KD);
static_assert(robot_debug::direction::positionParams().ki==ROBOT_STEER_POSITION_KI);
static_assert(robot_debug::direction::positionParams().kd==ROBOT_STEER_POSITION_KD);
static_assert(robot_debug::direction::positionParams().integral_limit==ROBOT_STEER_POSITION_INTEGRAL_LIMIT_RAD_S);
static_assert(robot_debug::direction::speedParams().integral_limit==ROBOT_STEER_SPEED_INTEGRAL_LIMIT_A);
'''
    for defines,ok in cases:
        result=subprocess.run([compiler,'-std=gnu++20','-fsyntax-only','-x','c++','-I',str(root/'RobotComponents/component'),
            *['-D'+x for x in common+defines],'-'],input=source,text=True,encoding='utf-8',errors='replace',capture_output=True)
        if (result.returncode==0)!=ok:raise SystemExit(f'四舵配置检查失败：{defines}\n{result.stderr}')
    print('四舵自由小数/无整定范围、固定授权、非有限值/负增益/物理电流拒绝：PASS')


if __name__=='__main__':main()
