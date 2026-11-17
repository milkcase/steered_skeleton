"""检查rpm/电流/增益编译边界与两种调试互斥，不连接硬件。"""
from pathlib import Path
import shutil
import subprocess
import struct
import tempfile


def authorization_image(root, compiler, definitions):
    """独立编译一份固件的真实配置，读取授权字段与本地增益编码，模拟未重烧的云台。"""
    if 'ROBOT_DRIVE_SPEED_ENABLED=1' in definitions:
        # 历史协议回归冻结旧版合法基线；用户现行自由参数单独验证，绝不修改用户文件。
        values={'ROBOT_DRIVE_SPEED_KP':'2.0f','ROBOT_DRIVE_SPEED_KI':'0.0f',
                'ROBOT_DRIVE_SPEED_KD':'0.0f','ROBOT_DRIVE_CURRENT_LIMIT_A':'0.3f'}
        values.update(dict(item.split('=',1) for item in definitions))
        definitions=[key+'='+value for key,value in values.items()]
    objcopy = shutil.which('arm-none-eabi-objcopy')
    if objcopy is None:
        raise SystemExit('需要ARM objcopy')
    output = root.parent / 'tmp/task9/testbuild'
    output.mkdir(parents=True, exist_ok=True)
    source = '''#include "motor_debug_mode.hpp"
using namespace robot_debug;
extern "C" const uint32_t identity[] __attribute__((used,section(".authorization_test")))={
  mode::kTarget,mode::kProfile,
  drive::kEnabled?drive::kLocalTuningCode:direction::kProfile};
'''
    with tempfile.TemporaryDirectory(dir=output) as folder:
        folder = Path(folder)
        obj, binary = folder/'config.o', folder/'config.bin'
        subprocess.run([compiler, '-std=gnu++20', '-mcpu=cortex-m4', '-mthumb', '-c', '-x', 'c++',
                        '-I', str(root/'RobotComponents/component'), *['-D'+item for item in definitions],
                        '-', '-o', str(obj)], input=source, text=True, encoding='utf-8', check=True)
        subprocess.run([objcopy, '--dump-section', f'.authorization_test={binary}', str(obj)], check=True)
        return struct.unpack('<3I', binary.read_bytes())


def main():
    """用真实ARM编译器确认合法配置可用、非法配置被拒绝及配置编码可逆。"""
    root = Path(__file__).resolve().parents[2]
    compiler = shutil.which('arm-none-eabi-g++')
    if compiler is None:
        raise SystemExit('需要ARM GNU工具链')
    cases = [
        ([], True),
        (['ROBOT_STEER_POSITION_KP=100.0f'], True),
        (['ROBOT_STEER_SPEED_KP=100.0f'], True),
        (['ROBOT_STEER_CURRENT_LIMIT_A=0.500f','ROBOT_STEER_SPEED_KI=0.1f','ROBOT_STEER_SPEED_KD=0.01f'], True),
        (['ROBOT_STEER_CURRENT_LIMIT_A=0.550f'], False),
        (['ROBOT_STEER_POSITION_KP=100.5f'], False),
        (['ROBOT_STEER_POSITION_KP=0.0f'], False),
        (['ROBOT_STEER_POSITION_KP=8.1f'], False),
        (['ROBOT_STEER_TARGET_SPEED_RPM=5.0f', 'ROBOT_STEER_TARGET_RATE_RPM=5.0f', 'ROBOT_STEER_TRIP_SPEED_RPM=10.0f'], True),
        (['ROBOT_STEER_TARGET_SPEED_RPM=20.0f', 'ROBOT_STEER_TARGET_RATE_RPM=10.0f', 'ROBOT_STEER_TRIP_SPEED_RPM=45.0f'], True),
        (['ROBOT_STEER_TARGET_SPEED_RPM=120.0f','ROBOT_STEER_TARGET_RATE_RPM=120.0f','ROBOT_STEER_TRIP_SPEED_RPM=180.0f'], True),
        (['ROBOT_STEER_TRIP_SPEED_RPM=240.0f'], True),
        (['ROBOT_STEER_TARGET_SPEED_RPM=125.0f'], False),
        (['ROBOT_STEER_TARGET_RATE_RPM=125.0f'], False),
        (['ROBOT_STEER_TRIP_SPEED_RPM=245.0f'], False),
        (['ROBOT_STEER_TRIP_SPEED_RPM=120.0f'], False),
        (['ROBOT_STEER_TARGET_SPEED_RPM=10.0f', 'ROBOT_STEER_TARGET_RATE_RPM=15.0f'], False),
        (['ROBOT_STEER_TARGET_SPEED_RPM=12.0f'], False),
        (['ROBOT_DRIVE_CURRENT_LIMIT_A=0.9f'], True),
        (['ROBOT_DRIVE_SPEED_KP=0.1f'], True),
        (['ROBOT_DRIVE_SPEED_ENABLED=1', 'ROBOT_STEER_DIRECTION_ENABLED=1'], False),
    ]
    source = '''#include "motor_debug_mode.hpp"
using namespace robot_debug::steer_safety;
constexpr unsigned code=(kTarget-kTargetBase)*kProfileSpan+kProfile-kProfileBase;
static_assert(decodeSpeedIndex(code/kCurrentSlots).target==kTargetStep);
static_assert(decodeSpeedIndex(code/kCurrentSlots).rate==kRateStep);
static_assert(decodeSpeedIndex(code/kCurrentSlots).trip==kTripStep);
static_assert(code%kCurrentSlots==robot_debug::direction::kCurrentStep-1);
static_assert(((robot_debug::direction::kProfile>>21)&255u)==robot_debug::direction::kPositionStep-1);
static_assert(((robot_debug::direction::kProfile>>12)&511u)==robot_debug::direction::kSpeedStep-1);
static_assert(((robot_debug::direction::kProfile>>8)&15u)==robot_debug::direction::kIntegralStep-1);
static_assert(((robot_debug::direction::kProfile>>4)&15u)==robot_debug::direction::kDerivativeStep-1);
static_assert(robot_debug::direction::kSpeedKi-ROBOT_STEER_SPEED_KI<0.000001f && robot_debug::direction::kSpeedKi-ROBOT_STEER_SPEED_KI> -0.000001f);
static_assert(robot_debug::direction::kSpeedKd-ROBOT_STEER_SPEED_KD<0.000001f && robot_debug::direction::kSpeedKd-ROBOT_STEER_SPEED_KD> -0.000001f);
'''
    for definitions, accepted in cases:
        result = subprocess.run(
            [compiler, '-std=gnu++20', '-fsyntax-only', '-x', 'c++', '-I', str(root / 'RobotComponents/component'),
             *['-D' + definition for definition in definitions], '-'],
            input=source, text=True, encoding='utf-8', errors='replace', capture_output=True)
        if (result.returncode == 0) != accepted:
            raise SystemExit(f'配置边界不符合预期：{definitions}\n{result.stderr}')
        if not accepted and 'static assertion failed' not in result.stderr:
            raise SystemExit(f'配置没有因安全断言而拒绝：{definitions}\n{result.stderr}')
    # 云台基线单独编译并冻结；每个底盘增益变更也单独编译，不能用同一份宏假装不同固件。
    for enabled, gain_changes, safety_changes in (
        ('ROBOT_STEER_DIRECTION_ENABLED=1',
         ['ROBOT_STEER_POSITION_KP=6.0f','ROBOT_STEER_POSITION_KP=100.0f','ROBOT_STEER_SPEED_KP=1.5f','ROBOT_STEER_SPEED_KI=0.5f','ROBOT_STEER_SPEED_KD=0.001f'],
         [['ROBOT_STEER_CURRENT_LIMIT_A=0.150f'],['ROBOT_STEER_TARGET_SPEED_RPM=20.0f','ROBOT_STEER_TARGET_RATE_RPM=20.0f'],
          ['ROBOT_STEER_TARGET_RATE_RPM=15.0f'],['ROBOT_STEER_TRIP_SPEED_RPM=185.0f']]),
        ('ROBOT_DRIVE_SPEED_ENABLED=1',
         ['ROBOT_DRIVE_SPEED_KP=1.2f','ROBOT_DRIVE_SPEED_KI=0.5f','ROBOT_DRIVE_SPEED_KD=0.001f'],
         [['ROBOT_DRIVE_CURRENT_LIMIT_A=0.400f']]),
    ):
        baseline = authorization_image(root, compiler, [enabled])
        for change in gain_changes:
            changed = authorization_image(root, compiler, [enabled, change])
            # 用户可能正好已把增益调到测试示例值；改用另一合法值，绝不修改其源文件。
            if changed[2] == baseline[2]:
                name = change.split('=', 1)[0]
                alternate = {'ROBOT_STEER_POSITION_KP':'8.0f','ROBOT_STEER_SPEED_KP':'1.0f',
                             'ROBOT_DRIVE_SPEED_KP':'2.0f'}.get(name, '0.0f')
                changed = authorization_image(root, compiler, [enabled, name+'='+alternate])
            if changed[:2] != baseline[:2] or changed[2] == baseline[2]:
                raise SystemExit(f'独立编译后增益变更错误影响了授权，或本地增益没改变：{change}')
        for changes in safety_changes:
            changed = authorization_image(root, compiler, [enabled, *changes])
            if changed[:2] == baseline[:2] and len(changes)==1 and '_CURRENT_LIMIT_A=' in changes[0]:
                # 同理选择与当前限流不同的合法档，避免用户正在使用0.15A/0.4A时误报测试失败。
                name=changes[0].split('=',1)[0]
                if name.startswith('ROBOT_STEER_'):
                    code=(baseline[0]-44)*60000+baseline[1]-4096
                    alternate='0.050f' if code%10!=0 else '0.100f'
                else:
                    alternate='0.100f' if (baseline[1]&7)!=0 else '0.200f'
                changed=authorization_image(root,compiler,[enabled,name+'='+alternate])
            if changed[:2] == baseline[:2]:
                raise SystemExit(f'安全边界变更未改变授权配置：{changes}')
    # 全部9500种速度包络和10档电流必须互异，局部PID编码不参与协议比较。
    print('rpm/电流编译边界、模式互斥：PASS')
    print('独立编译云台基线和不同增益底盘：授权相同、本地增益不同；安全上限变化仍拒绝：PASS')


if __name__ == '__main__':
    main()
